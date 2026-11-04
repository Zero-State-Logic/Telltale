/*
 * ================================================================
 *   TELLTALE - a fingerprint door lock whose own servo is the
 *   tamper sensor.  (Formerly Phantom Door.)
 *   ESP32 + R307 fingerprint sensor + SG90 servo + INA219
 *   By Faysal Ali Shah
 * ================================================================
 *
 *  FEATURES (all of Phantom Door, plus the proprioceptive layer):
 *   - Fingerprint auth with retry + anti-jitter stabilisation
 *   - Hardware PWM servo with overshoot technique and soft ramps, now a
 *     non-blocking motion state machine in its own 1 kHz task
 *   - INA219 on the servo supply: 1 kHz during motion, 100 Hz while locked
 *   - Learns the lock's healthy current signature (first 30 unlocks and 30
 *     relocks), stored in NVS, then flags per actuation: jam / obstruction,
 *     broken or slack linkage, brown-out, added friction
 *   - While locked: flags sustained force on the bolt ("forced while
 *     locked") and loss of servo power, with an onset estimate
 *   - Web dashboard over WebSocket: status, live current trace, alerts
 *   - Remote unlock, enroll / delete users from the web UI
 *   - 4-second unlock hold then auto-relock (retries if the door is ajar)
 *   - LED status, watchdog, NVS user names, 3 failures -> 30 s lockout
 *
 *  WIRING:
 *   R307 RX  -> GPIO 17 (TX2)        R307 TX -> GPIO 16 (RX2)
 *   R307 VCC -> 3.3V (100uF cap near the sensor), GND -> GND
 *   SG90 PWM -> GPIO 18
 *   5V rail  -> INA219 VIN+ ; INA219 VIN- -> SG90 VCC (0.1 ohm shunt)
 *   470uF cap on the 5V rail before the INA219; SG90 GND -> common GND
 *   INA219 SDA -> GPIO 21, SCL -> GPIO 22, VCC -> 3.3V, GND -> GND
 *   LED -> GPIO 2 (built-in) + external LED on GPIO 4
 *   BTN_ENROLL -> GPIO 0 (BOOT button, active LOW)
 *
 *  SECURITY NOTE: the HTTP endpoints below are still unauthenticated on the
 *  LAN, as in Phantom Door. A tested HMAC-SHA256 challenge/response core
 *  (telltale_auth.c) is in this folder; wiring it into the endpoints and
 *  the dashboard is the next roadmap item. Keep the lock on a trusted
 *  network until then.
 *
 *  LIBRARIES (Arduino IDE / PlatformIO, ESP32 Arduino core 2.0.x):
 *   - Adafruit Fingerprint Sensor Library (v2.x)
 *   - WebSocketsServer by Markus Sattler (arduinoWebSockets)
 *   - ArduinoJson 6.x
 *   - Preferences, WebServer, Wire (built into the ESP32 core)
 * ================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Adafruit_Fingerprint.h>
#include <Preferences.h>
#include <esp_task_wdt.h>

#include "tt_sense.h"

// -- WiFi credentials (placeholders) ------------------------------------
#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// -- Pins --------------------------------------------------------------
#define SERVO_PIN     18
#define LED_STATUS    2
#define LED_EXTERNAL  4
#define BTN_ENROLL    0
#define I2C_SDA       21
#define I2C_SCL       22
#define LEDC_CHANNEL  0

// -- Timing ------------------------------------------------------------
#define UNLOCK_HOLD_MS      4000
#define LOCKOUT_DURATION_MS 30000
#define MAX_FAIL_ATTEMPTS   3
#define RELOCK_RETRY_MS     3000
#define MAX_RELOCK_RETRIES  2

// -- Fingerprint sensor ------------------------------------------------
#define FINGERPRINT_RX 16
#define FINGERPRINT_TX 17
HardwareSerial fpSerial(2);
Adafruit_Fingerprint finger(&fpSerial);

WebServer server(80);
WebSocketsServer ws(81);
Preferences prefs;

enum DoorState { STATE_IDLE, STATE_UNLOCKED, STATE_ENROLLING, STATE_LOCKOUT };

volatile DoorState doorState = STATE_IDLE;
unsigned long unlockStartTime = 0;
unsigned long lockoutStartTime = 0;
unsigned long relockRetryAt = 0;
int failAttempts = 0;
int relockRetries = 0;
bool enrollPending = false;
uint8_t enrollSlot = 0;

// -- User names in NVS ("uN" for slot N) --------------------------------
String getUserName(int slot) {
  prefs.begin("door", true);
  String name = prefs.getString(("u" + String(slot)).c_str(), "User " + String(slot));
  prefs.end();
  return name;
}

void setUserName(int slot, const String &name) {
  prefs.begin("door", false);
  prefs.putString(("u" + String(slot)).c_str(), name);
  prefs.end();
}

// -- Access log (last 20 events) ----------------------------------------
struct LogEntry {
  String name;
  bool success;
  unsigned long timestamp;
};
LogEntry accessLog[20];
int logIndex = 0;

void pushLog(const String &name, bool success) {
  accessLog[logIndex % 20] = {name, success, millis()};
  logIndex++;
}

const char *stateName() {
  switch (doorState) {
    case STATE_UNLOCKED: return "unlocked";
    case STATE_LOCKOUT: return "lockout";
    case STATE_ENROLLING: return "enrolling";
    default: return "locked";
  }
}

void addTelltaleStatus(JsonObject tt) {
  tt["sensor"] = ttSenseSensorOk() ? "ok" : "missing";
  JsonArray learned = tt.createNestedArray("learned");
  learned.add(ttSenseLearned(0));
  learned.add(ttSenseLearned(1));
  tt["needed"] = ttSenseLearnNeeded();
  tt["idle"] = ttSenseIdleArmed() ? "armed" : "learning";
  tt["busV"] = ttSenseBusVolts();
}

void broadcastStatus() {
  StaticJsonDocument<1024> doc;
  doc["state"] = stateName();
  doc["enrollCount"] = finger.getTemplateCount() == FINGERPRINT_OK ? finger.templateCount : 0;
  doc["failCount"] = failAttempts;
  addTelltaleStatus(doc.createNestedObject("telltale"));
  JsonArray logs = doc.createNestedArray("recentLog");
  for (int i = 0; i < min(logIndex, 20); i++) {
    int idx = (logIndex - 1 - i + 20) % 20;
    JsonObject entry = logs.createNestedObject();
    entry["name"] = accessLog[idx].name;
    entry["success"] = accessLog[idx].success;
  }
  String out;
  serializeJson(doc, out);
  ws.broadcastTXT(out);
}

void broadcastAlert(const char *alert, const char *kind, const String &message,
                    float score, long onsetMs) {
  StaticJsonDocument<256> doc;
  doc["alert"] = alert;
  doc["kind"] = kind;
  doc["message"] = message;
  doc["score"] = score;
  if (onsetMs >= 0) doc["onset_ms"] = onsetMs;
  String out;
  serializeJson(doc, out);
  ws.broadcastTXT(out);
}

// -- LED ------------------------------------------------------------------
void blinkLED(int times, int onMs = 80, int offMs = 100) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_EXTERNAL, HIGH);
    delay(onMs);
    digitalWrite(LED_EXTERNAL, LOW);
    if (i < times - 1) delay(offMs);
  }
}

// -- Door actions (motion runs in the sense task, never blocks here) ----
void triggerUnlock(const String &who) {
  if (doorState == STATE_UNLOCKED || ttSenseBusy()) return;
  if (!ttSenseCommand(TT_CMD_UNLOCK)) return;
  doorState = STATE_UNLOCKED;
  unlockStartTime = millis();
  failAttempts = 0;
  relockRetries = 0;
  Serial.println("[DOOR] UNLOCKING for: " + who);
  pushLog(who, true);
  blinkLED(2, 200, 100);
  broadcastStatus();
}

void triggerRelock() {
  if (!ttSenseCommand(TT_CMD_RELOCK)) return;
  doorState = STATE_IDLE;
  Serial.println("[DOOR] RELOCKING");
  blinkLED(1, 50);
  broadcastStatus();
}

// -- Telltale events --------------------------------------------------------
void onActuation(const TtEvent &e) {
  const char *kindName = e.kind == 0 ? "unlock" : "relock";
  StaticJsonDocument<256> doc;
  JsonObject a = doc.createNestedObject("actuation");
  a["kind"] = kindName;
  a["status"] = e.status == 0 ? "learning" : (e.status == 1 ? "healthy" : "anomaly");
  a["fault"] = ttFaultName(e.fault);
  a["score"] = e.score;
  a["threshold"] = e.threshold;
  a["peak_ma"] = e.peak_ma;
  a["plateau_ma"] = e.plateau_ma;
  String out;
  serializeJson(doc, out);
  ws.broadcastTXT(out);
  Serial.printf("[TELLTALE] %s score %.2f (%s, %s)\n", kindName, e.score,
                a["status"].as<const char *>(), ttFaultName(e.fault));
  if (e.status != 2) return;

  String msg = String(kindName) + ": " + ttFaultName(e.fault);
  broadcastAlert("fault", ttFaultName(e.fault), msg, e.score, -1);
  pushLog("Fault: " + msg, false);
  if (e.fault == 1 && e.kind == 0 && doorState == STATE_UNLOCKED) {
    // Unlock jammed: back off to locked instead of stalling the servo.
    broadcastAlert("fault", "unlock_jammed", "Unlock blocked - returned to locked", e.score, -1);
    triggerRelock();
  } else if (e.fault == 1 && e.kind == 1) {
    // Relock blocked, typically the door is ajar: retry a couple of times.
    if (relockRetries < MAX_RELOCK_RETRIES) {
      relockRetries++;
      relockRetryAt = millis() + RELOCK_RETRY_MS;
    } else {
      broadcastAlert("fault", "relock_failed", "Door did not lock - check the door", e.score, -1);
    }
  }
}

void onIdleAlarm(const TtEvent &e) {
  const bool forced = e.kind == 2;   // TT_IDLE_FORCED
  const char *kind = forced ? "forced_while_locked" : "servo_power_lost";
  String msg = forced ? "Force on the bolt while locked" : "Servo power or wiring lost";
  Serial.printf("[TAMPER] %s (onset ~%lu ms ago)\n", kind, (unsigned long)e.onset_ms);
  broadcastAlert("tamper", kind, msg, e.score, (long)e.onset_ms);
  pushLog("TAMPER: " + msg, false);
  blinkLED(6, 40, 40);
}

void pumpTelltale() {
  TtEvent e;
  while (ttSensePollEvent(&e)) {
    if (e.type == TT_EVT_ACTUATION) onActuation(e);
    else if (e.type == TT_EVT_IDLE_ALARM) onIdleAlarm(e);
    else broadcastAlert("fault", "sensor_error", "INA219 not responding", 0, -1);
  }
  TtTraceChunk c;
  int budget = 4;   // keep the loop responsive
  while (budget-- > 0 && ttSensePollTrace(&c)) {
    static const char *phases[] = {"unlock", "relock", "locked", "unlocked"};
    StaticJsonDocument<768> doc;
    JsonObject t = doc.createNestedObject("trace");
    t["phase"] = phases[c.phase & 3];
    t["rate"] = c.rate_hz;
    JsonArray ma = t.createNestedArray("ma10");
    for (int i = 0; i < c.n; ++i) ma.add(c.ma10[i]);
    String out;
    serializeJson(doc, out);
    ws.broadcastTXT(out);
  }
  if (relockRetryAt && (long)(millis() - relockRetryAt) >= 0 && !ttSenseBusy()) {
    relockRetryAt = 0;
    triggerRelock();
  }
}

// -- Fingerprint: read with retry --------------------------------------------
int fingerprintSearch() {
  uint8_t p = FINGERPRINT_NOFINGER;
  for (int attempt = 0; attempt < 3; attempt++) {
    p = finger.getImage();
    if (p == FINGERPRINT_OK) break;
    if (p == FINGERPRINT_NOFINGER) return -1;
    delay(50);
  }
  if (p != FINGERPRINT_OK) return -1;
  if (finger.image2Tz() != FINGERPRINT_OK) return -1;
  if (finger.fingerFastSearch() != FINGERPRINT_OK) return -2;
  return finger.fingerID;
}

// -- Fingerprint: enrollment (2-image process) --------------------------------
bool waitForFinger(int tries) {
  for (int i = 0; i < tries; i++) {
    if (finger.getImage() == FINGERPRINT_OK) return true;
    delay(100);
    esp_task_wdt_reset();
  }
  return false;
}

void enrollFail(const char *msg) {
  StaticJsonDocument<128> doc;
  doc["enrollStatus"] = msg;
  doc["step"] = 0;
  String out;
  serializeJson(doc, out);
  ws.broadcastTXT(out);
  doorState = STATE_IDLE;
}

void doEnroll(uint8_t slot, const String &name) {
  Serial.println("[ENROLL] slot " + String(slot) + " (" + name + ")");
  ws.broadcastTXT("{\"enrollStatus\":\"Place finger on sensor...\",\"step\":1}");
  if (!waitForFinger(30)) return enrollFail("Timeout - no finger detected. Try again.");
  if (finger.image2Tz(1) != FINGERPRINT_OK) return enrollFail("Poor image - press firmly and retry.");

  ws.broadcastTXT("{\"enrollStatus\":\"Good! Lift finger and place again...\",\"step\":2}");
  delay(1500);
  for (int i = 0; i < 20; i++) {
    if (finger.getImage() == FINGERPRINT_NOFINGER) break;
    delay(100);
    esp_task_wdt_reset();
  }
  ws.broadcastTXT("{\"enrollStatus\":\"Place same finger again...\",\"step\":2}");
  if (!waitForFinger(30)) return enrollFail("Timeout on second scan. Try again.");
  if (finger.image2Tz(2) != FINGERPRINT_OK) return enrollFail("Poor image on second scan. Try again.");
  if (finger.createModel() != FINGERPRINT_OK) return enrollFail("Fingerprints did not match. Try again.");
  if (finger.storeModel(slot) != FINGERPRINT_OK) return enrollFail("Storage error. Slot may be full.");

  setUserName(slot, name);
  StaticJsonDocument<192> doc;
  doc["enrollStatus"] = "Enrolled! " + name + " saved to slot " + String(slot) + ".";
  doc["step"] = 3;
  doc["success"] = true;
  String out;
  serializeJson(doc, out);
  ws.broadcastTXT(out);
  doorState = STATE_IDLE;
  broadcastStatus();
}

int nextFreeSlot() {
  for (int i = 1; i <= 127; i++) {
    if (finger.loadModel(i) != FINGERPRINT_OK) return i;
  }
  return -1;
}

// -- Web routes -------------------------------------------------------------------
void sendJson(int code, const String &body) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(code, "application/json", body);
}

void handleCorsPreflight() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(204);
}

void handleRoot() { server.send(200, "text/html", getWebPage()); }

void handleUnlock() {
  String who = "Remote User";
  if (server.hasArg("plain")) {
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, server.arg("plain")) == DeserializationError::Ok) {
      who = doc["who"] | "Remote User";
    }
  }
  triggerUnlock(who);
  sendJson(200, "{\"ok\":true}");
}

void handleEnroll() {
  if (doorState != STATE_IDLE) return sendJson(409, "{\"error\":\"Busy\"}");
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
    return sendJson(400, "{\"error\":\"Bad JSON\"}");
  }
  String name = doc["name"] | "Unknown";
  name = name.substring(0, 32);
  int slot = nextFreeSlot();
  if (slot == -1) return sendJson(507, "{\"error\":\"Sensor memory full (127 max)\"}");
  doorState = STATE_ENROLLING;
  enrollSlot = slot;
  enrollPending = true;
  prefs.begin("door", false);
  prefs.putString("pendingName", name);
  prefs.end();
  sendJson(200, "{\"ok\":true,\"slot\":" + String(slot) + "}");
}

void handleDeleteUser() {
  StaticJsonDocument<64> doc;
  deserializeJson(doc, server.arg("plain"));
  int slot = doc["slot"] | -1;
  if (slot < 1 || slot > 127) return sendJson(400, "{\"error\":\"Invalid slot\"}");
  if (finger.deleteModel(slot) != FINGERPRINT_OK) return sendJson(500, "{\"error\":\"Delete failed\"}");
  prefs.begin("door", false);
  prefs.remove(("u" + String(slot)).c_str());
  prefs.end();
  sendJson(200, "{\"ok\":true}");
  broadcastStatus();
}

void handleListUsers() {
  DynamicJsonDocument doc(6144);
  JsonArray users = doc.createNestedArray("users");
  for (int i = 1; i <= 127; i++) {
    if (finger.loadModel(i) == FINGERPRINT_OK) {
      JsonObject u = users.createNestedObject();
      u["slot"] = i;
      u["name"] = getUserName(i);
    }
    esp_task_wdt_reset();
  }
  String out;
  serializeJson(doc, out);
  sendJson(200, out);
}

void handleStatus() {
  finger.getTemplateCount();
  StaticJsonDocument<512> doc;
  doc["state"] = stateName();
  doc["count"] = finger.templateCount;
  doc["failAttempts"] = failAttempts;
  doc["ip"] = WiFi.localIP().toString();
  addTelltaleStatus(doc.createNestedObject("telltale"));
  String out;
  serializeJson(doc, out);
  sendJson(200, out);
}

void handleRelearn() {
  ttSenseCommand(TT_CMD_RELEARN);
  sendJson(200, "{\"ok\":true}");
  broadcastStatus();
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  (void)payload;
  (void)length;
  if (type == WStype_CONNECTED) {
    broadcastStatus();
    Serial.printf("[WS] Client #%u connected\n", num);
  }
}

void addRoute(const char *path, HTTPMethod method, WebServer::THandlerFunction fn) {
  server.on(path, method, fn);
  server.on(path, HTTP_OPTIONS, handleCorsPreflight);
}

// -- Setup ------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== TELLTALE BOOTING ===");
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_EXTERNAL, OUTPUT);
  pinMode(BTN_ENROLL, INPUT_PULLUP);
  digitalWrite(LED_STATUS, LOW);
  digitalWrite(LED_EXTERNAL, LOW);

  esp_task_wdt_init(10, true);
  esp_task_wdt_add(NULL);

  if (!ttSenseBegin(I2C_SDA, I2C_SCL, SERVO_PIN, LEDC_CHANNEL)) {
    Serial.println("[ERROR] Could not start the sense task");
  }

  // UART2 is configured here with explicit pins; Adafruit_Fingerprint 2.x
  // has no argument-less begin(), so presence is checked with the password
  // handshake instead (the original sketch called finger.begin()).
  fpSerial.begin(57600, SERIAL_8N1, FINGERPRINT_RX, FINGERPRINT_TX);
  delay(100);
  if (!finger.verifyPassword()) {
    Serial.println("[ERROR] Fingerprint sensor not found! Check wiring.");
    while (1) {
      blinkLED(5, 50, 50);
      delay(500);
      esp_task_wdt_reset();
    }
  }
  finger.getParameters();
  Serial.printf("[FP] Capacity: %d, Enrolled: %d\n", finger.capacity, finger.templateCount);

  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int tries = 0; WiFi.status() != WL_CONNECTED && tries < 30; tries++) {
    delay(500);
    Serial.print(".");
    esp_task_wdt_reset();
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    blinkLED(3, 100, 100);
  } else {
    Serial.println("\n[WiFi] Failed - running without web features");
  }

  server.on("/", HTTP_GET, handleRoot);
  addRoute("/status", HTTP_GET, handleStatus);
  addRoute("/users", HTTP_GET, handleListUsers);
  addRoute("/unlock", HTTP_POST, handleUnlock);
  addRoute("/enroll", HTTP_POST, handleEnroll);
  addRoute("/delete", HTTP_POST, handleDeleteUser);
  addRoute("/relearn", HTTP_POST, handleRelearn);
  server.begin();
  ws.begin();
  ws.onEvent(onWsEvent);
  Serial.println("[SERVER] HTTP on :80, WS on :81");
  Serial.println("[READY] Telltale is armed.");
  digitalWrite(LED_STATUS, HIGH);
}

// -- Main loop --------------------------------------------------------------------
void loop() {
  esp_task_wdt_reset();
  server.handleClient();
  ws.loop();
  pumpTelltale();

  if (doorState == STATE_UNLOCKED) {
    if (millis() - unlockStartTime >= UNLOCK_HOLD_MS && !ttSenseBusy()) triggerRelock();
    return;
  }
  if (doorState == STATE_LOCKOUT) {
    if (millis() - lockoutStartTime >= LOCKOUT_DURATION_MS) {
      doorState = STATE_IDLE;
      failAttempts = 0;
      Serial.println("[LOCKOUT] Released");
      broadcastStatus();
    }
    return;
  }
  if (enrollPending && doorState == STATE_ENROLLING) {
    enrollPending = false;
    prefs.begin("door", true);
    String pendingName = prefs.getString("pendingName", "User");
    prefs.end();
    doEnroll(enrollSlot, pendingName);
    return;
  }
  if (digitalRead(BTN_ENROLL) == LOW && doorState == STATE_IDLE) {
    delay(50);
    if (digitalRead(BTN_ENROLL) == LOW) {
      int slot = nextFreeSlot();
      if (slot != -1) {
        doorState = STATE_ENROLLING;
        doEnroll(slot, "User " + String(slot));
        while (digitalRead(BTN_ENROLL) == LOW) {
          delay(10);
          esp_task_wdt_reset();
        }
      }
    }
  }
  if (ttSenseBusy()) return;   // no scanning while the bolt is moving

  int result = fingerprintSearch();
  if (result == -1) return;
  if (result == -2) {
    failAttempts++;
    Serial.printf("[DENIED] No match. Fails: %d/%d\n", failAttempts, MAX_FAIL_ATTEMPTS);
    pushLog("Unknown", false);
    blinkLED(3, 50, 50);
    if (failAttempts >= MAX_FAIL_ATTEMPTS) {
      doorState = STATE_LOCKOUT;
      lockoutStartTime = millis();
      Serial.println("[LOCKOUT] Activated for 30 seconds");
      broadcastAlert("lockout", "lockout", "LOCKOUT - 3 failed attempts!", 0, -1);
    }
    broadcastStatus();
    delay(1000);
    return;
  }
  String name = getUserName(result);
  Serial.printf("[GRANTED] ID: %d, Name: %s, Confidence: %d\n", result, name.c_str(),
                finger.confidence);
  triggerUnlock(name);
}

// -- Minimal page served by the lock ------------------------------------------------
String getWebPage() {
  return R"rawhtml(<!DOCTYPE html>
<html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Telltale</title>
<style>body{font-family:sans-serif;background:#F7F8F9;color:#14171A;text-align:center;padding:40px}
h1{color:#E5383B}code{color:#3D5A80}</style></head>
<body><h1>Telltale</h1>
<p>The lock is running. Open the dashboard (index.html from the repository, served
from your computer over http) and enter this lock's IP address.</p>
<p><code>GET /status</code> &middot; <code>GET /users</code> &middot; <code>POST /unlock</code>
&middot; <code>POST /enroll</code> &middot; <code>POST /delete</code> &middot; <code>POST /relearn</code>
&middot; WebSocket on port 81</p></body></html>
)rawhtml";
}
