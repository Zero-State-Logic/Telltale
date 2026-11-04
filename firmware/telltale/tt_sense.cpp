/*
 * tt_sense.cpp - sense task: motion state machine, INA219, detector, NVS.
 * Copyright (c) 2026 Faysal Ali Shah. MIT License.
 */
#include "tt_sense.h"

#include <Preferences.h>
#include <Wire.h>
#include <esp_task_wdt.h>

#include "telltale_detect.h"
#include "telltale_ina219.h"
#include "telltale_motion.h"
#include "telltale_persist.h"

namespace {

const float SHUNT_OHM = 0.1f;
const uint32_t IDLE_DIVIDER = 10;       // 1 kHz task -> 100 Hz idle samples
const uint16_t SAVE_EVERY = 25;         // persist the model every N scored actuations
const char *NVS_NS = "telltale";
const char *NVS_KEY = "model";

enum Mode : uint8_t { MODE_LOCKED_IDLE, MODE_MOTION, MODE_UNLOCKED_HOLD };

struct Shared {                          // read by the loop task, written here
  volatile bool busy;
  volatile bool sensorOk;
  volatile bool idleArmed;
  volatile uint16_t learned[TT_ACT_KINDS];
  volatile float busVolts;
};

tt_detector_t g_det;
tt_motion_t g_plan;
Shared g_shared;
QueueHandle_t g_cmdQ = nullptr;
QueueHandle_t g_evtQ = nullptr;
QueueHandle_t g_traceQ = nullptr;
Preferences g_prefs;

Mode g_mode = MODE_LOCKED_IDLE;
uint8_t g_planKind = TT_ACT_UNLOCK;
bool g_planScored = false;
uint32_t g_tick = 0;                     // samples since the motion started
uint32_t g_divider = 0;
uint16_t g_sinceSave = 0;
bool g_savePending = false;
int g_ledcChannel = 0;
TtTraceChunk g_chunk;

// ------------------------------------------------------------------ INA219
bool inaWrite(uint8_t reg, uint16_t value) {
  Wire.beginTransmission((uint8_t)TT_INA219_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  return Wire.endTransmission() == 0;
}

bool inaRead(uint8_t reg, uint16_t *value) {
  Wire.beginTransmission((uint8_t)TT_INA219_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)TT_INA219_ADDR, (uint8_t)2) != 2) return false;
  const uint16_t hi = Wire.read();
  const uint16_t lo = Wire.read();
  *value = (uint16_t)((hi << 8) | lo);
  return true;
}

bool readCurrent(float *ma) {
  uint16_t raw = 0;
  if (!inaRead(TT_INA219_REG_SHUNT, &raw)) return false;
  *ma = tt_ina219_shunt_ma((int16_t)raw, SHUNT_OHM);
  return true;
}

void readBus() {
  uint16_t raw = 0;
  if (inaRead(TT_INA219_REG_BUS, &raw)) g_shared.busVolts = tt_ina219_bus_mv(raw) * 1e-3f;
}

void sensorFailed() {
  if (!g_shared.sensorOk) return;
  g_shared.sensorOk = false;
  TtEvent e = {};
  e.type = TT_EVT_SENSOR_ERROR;
  xQueueSend(g_evtQ, &e, 0);
}

// ------------------------------------------------------------- telemetry
void tracePush(uint8_t phase, uint16_t rate, float ma) {
  if (g_chunk.n > 0 && (g_chunk.phase != phase || g_chunk.rate_hz != rate)) {
    xQueueSend(g_traceQ, &g_chunk, 0);
    g_chunk.n = 0;
  }
  g_chunk.phase = phase;
  g_chunk.rate_hz = rate;
  float v = ma * 10.0f;
  v = v > 32767.0f ? 32767.0f : (v < -32768.0f ? -32768.0f : v);
  g_chunk.ma10[g_chunk.n++] = (int16_t)v;
  if (g_chunk.n == TT_TRACE_CHUNK) {
    xQueueSend(g_traceQ, &g_chunk, 0);   // telemetry only: drop when full
    g_chunk.n = 0;
  }
}

// ------------------------------------------------------------ persistence
void saveModel() {
  static tt_persist_t blob;
  if (tt_persist_pack(&g_det, &blob) != TT_OK) return;
  g_prefs.begin(NVS_NS, false);
  g_prefs.putBytes(NVS_KEY, &blob, sizeof(blob));
  g_prefs.end();
}

void loadModel() {
  static tt_persist_t blob;
  g_prefs.begin(NVS_NS, true);
  const size_t n = g_prefs.getBytes(NVS_KEY, &blob, sizeof(blob));
  g_prefs.end();
  if (n == sizeof(blob) && tt_persist_unpack(&g_det, &blob) == TT_OK) {
    Serial.println("[TELLTALE] Learned model restored from NVS");
  } else {
    Serial.println("[TELLTALE] No stored model: learning mode");
  }
}

void publishLearning() {
  for (uint8_t k = 0; k < TT_ACT_KINDS; ++k) g_shared.learned[k] = g_det.model[k].n;
  g_shared.idleArmed = tt_idle_learned(&g_det) == 1;
}

// ----------------------------------------------------------------- motion
void servoWrite(float deg) { ledcWrite(g_ledcChannel, tt_servo_duty(deg)); }

void startMotion(uint8_t kind) {
  if (kind == TT_ACT_UNLOCK) {
    tt_motion_unlock(&g_plan);
  } else {
    tt_motion_relock(&g_plan);
  }
  g_planKind = kind;
  g_tick = 0;
  g_mode = MODE_MOTION;
  g_shared.busy = true;
  inaWrite(TT_INA219_REG_CONFIG, tt_ina219_config_motion());
  g_planScored = g_shared.sensorOk &&
                 tt_act_begin(&g_det, kind, tt_motion_total_ms(&g_plan),
                              tt_motion_motion_ms(&g_plan), 1.0f) == TT_OK;
}

void finishActuation() {
  tt_act_result_t r;
  if (tt_act_finish(&g_det, &r) != TT_OK) return;
  TtEvent e = {};
  e.type = TT_EVT_ACTUATION;
  e.kind = g_planKind;
  e.status = (int8_t)r.status;
  e.fault = (int8_t)r.fault;
  e.score = r.score;
  e.threshold = g_det.act_cfg.threshold;
  e.peak_ma = r.features[TT_FEAT_PEAK];
  e.plateau_ma = r.features[TT_FEAT_PLATEAU];
  xQueueSend(g_evtQ, &e, 0);
  publishLearning();
  const bool justLearned = (r.status == TT_STATUS_LEARNING) &&
                           tt_act_learning(&g_det, TT_ACT_UNLOCK) == 0 &&
                           tt_act_learning(&g_det, TT_ACT_RELOCK) == 0;
  if (r.updated && (++g_sinceSave >= SAVE_EVERY || justLearned)) {
    g_sinceSave = 0;
    g_savePending = true;    // written once the lock is idle, never mid-motion
  }
}

void endMotion() {
  g_shared.busy = false;
  if (g_planKind == TT_ACT_UNLOCK) {
    g_mode = MODE_UNLOCKED_HOLD;
  } else {
    g_mode = MODE_LOCKED_IDLE;
    inaWrite(TT_INA219_REG_CONFIG, tt_ina219_config_idle());
    tt_idle_enter(&g_det);
  }
  g_divider = 0;
}

void motionStep() {
  servoWrite(tt_motion_angle_at(&g_plan, g_tick));
  float ma = 0.0f;
  const bool ok = readCurrent(&ma);
  if (!ok) {
    sensorFailed();
  } else {
    tracePush(g_planKind, 1000, ma);
  }
  if (g_planScored) {
    if (!ok) {
      g_planScored = false;          // incomplete window: discard
      tt_act_result_t discard;
      tt_act_finish(&g_det, &discard);
    } else if (tt_act_feed(&g_det, ma) == 1) {
      g_planScored = false;
      finishActuation();
    }
  }
  if (++g_tick >= tt_motion_total_ms(&g_plan)) endMotion();
}

// ------------------------------------------------------------------- idle
void idleStep() {
  float ma = 0.0f;
  if (!readCurrent(&ma)) {
    sensorFailed();
    return;
  }
  g_shared.sensorOk = true;
  tracePush(2, 100, ma);
  float stat = 0.0f;
  const int ev = tt_idle_feed(&g_det, ma, &stat);
  if (ev == TT_IDLE_FORCED || ev == TT_IDLE_POWER_LOSS) {
    TtEvent e = {};
    e.type = TT_EVT_IDLE_ALARM;
    e.kind = (uint8_t)ev;
    e.score = 1.0f;     // the CUSUM crossed its threshold
    e.onset_ms = tt_idle_onset_samples(&g_det) * 10u;
    xQueueSend(g_evtQ, &e, 0);
  }
  if (ev == TT_IDLE_LEARNING || (tt_idle_learned(&g_det) && !g_shared.idleArmed)) {
    publishLearning();
  }
  if (g_savePending && ev != TT_IDLE_FORCED) {
    g_savePending = false;
    saveModel();
  }
}

void handleCommand() {
  uint8_t cmd = 0;
  if (xQueueReceive(g_cmdQ, &cmd, 0) != pdTRUE) return;
  if (cmd == TT_CMD_RELEARN) {
    tt_detector_reset_models(&g_det);
    g_prefs.begin(NVS_NS, false);
    g_prefs.remove(NVS_KEY);
    g_prefs.end();
    publishLearning();
    if (g_mode == MODE_LOCKED_IDLE) tt_idle_enter(&g_det);
    return;
  }
  if (g_mode == MODE_MOTION) return;   // one motion at a time
  startMotion(cmd == TT_CMD_UNLOCK ? TT_ACT_UNLOCK : TT_ACT_RELOCK);
}

void senseTask(void *) {
  esp_task_wdt_add(nullptr);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    esp_task_wdt_reset();
    handleCommand();
    if (g_mode == MODE_MOTION) {
      motionStep();
      continue;
    }
    if (++g_divider < IDLE_DIVIDER) continue;
    g_divider = 0;
    if (g_mode == MODE_LOCKED_IDLE) {
      idleStep();
    } else {
      float ma = 0.0f;
      if (readCurrent(&ma)) tracePush(3, 100, ma);
      readBus();
    }
  }
}

}  // namespace

// ---------------------------------------------------------------- public
bool ttSenseBegin(int sda, int scl, int servoPin, int ledcChannel) {
  g_ledcChannel = ledcChannel;
  ledcSetup(ledcChannel, 50, 16);
  ledcAttachPin(servoPin, ledcChannel);
  servoWrite(TT_ANGLE_LOCKED);

  Wire.begin(sda, scl, 400000);
  g_shared.sensorOk = inaWrite(TT_INA219_REG_CONFIG, tt_ina219_config_idle());
  if (!g_shared.sensorOk) Serial.println("[TELLTALE] INA219 not found: tamper sensing off");

  tt_detector_init(&g_det);
  loadModel();
  tt_idle_enter(&g_det);
  publishLearning();
  readBus();

  g_cmdQ = xQueueCreate(4, sizeof(uint8_t));
  g_evtQ = xQueueCreate(8, sizeof(TtEvent));
  g_traceQ = xQueueCreate(16, sizeof(TtTraceChunk));
  if (!g_cmdQ || !g_evtQ || !g_traceQ) return false;
  // Core 1 (with the Arduino loop) at a higher priority; WiFi lives on core 0.
  return xTaskCreatePinnedToCore(senseTask, "tt_sense", 4096, nullptr, 3, nullptr, 1) == pdPASS;
}

bool ttSenseCommand(TtCommand cmd) {
  const uint8_t c = (uint8_t)cmd;
  return g_cmdQ && xQueueSend(g_cmdQ, &c, 0) == pdTRUE;
}

bool ttSensePollEvent(TtEvent *out) { return g_evtQ && xQueueReceive(g_evtQ, out, 0) == pdTRUE; }
bool ttSensePollTrace(TtTraceChunk *out) { return g_traceQ && xQueueReceive(g_traceQ, out, 0) == pdTRUE; }
bool ttSenseBusy() { return g_shared.busy; }
bool ttSenseSensorOk() { return g_shared.sensorOk; }
uint16_t ttSenseLearned(uint8_t kind) { return kind < TT_ACT_KINDS ? g_shared.learned[kind] : 0; }
uint16_t ttSenseLearnNeeded() { return g_det.act_cfg.learn_runs; }
bool ttSenseIdleArmed() { return g_shared.idleArmed; }
float ttSenseBusVolts() { return g_shared.busVolts; }

const char *ttFaultName(int fault) {
  switch (fault) {
    case TT_FAULT_STALL: return "stall";
    case TT_FAULT_LINKAGE: return "linkage";
    case TT_FAULT_BROWNOUT: return "brownout";
    case TT_FAULT_FRICTION: return "friction";
    case TT_FAULT_UNKNOWN: return "unknown";
    default: return "none";
  }
}
