/*
 * tt_sense.h - the TELLTALE sense task: servo motion, INA219 sampling and the
 * proprioceptive detector, running in its own FreeRTOS task at 1 kHz so the
 * web server and the fingerprint UART can never disturb the sample clock.
 *
 *   motion  : commanded angle every 1 ms (non-blocking state machine),
 *             INA219 9-bit shunt read every 1 ms, fed to the detector
 *   locked  : INA219 12-bit x16 averaged read every 10 ms, idle CUSUM
 *   unlocked: 10 ms reads for telemetry only
 *
 * The main loop talks to it through two queues (commands in, events out).
 *
 * Copyright (c) 2026 Faysal Ali Shah. MIT License.
 */
#ifndef TT_SENSE_H
#define TT_SENSE_H

#include <Arduino.h>

enum TtCommand : uint8_t {
  TT_CMD_UNLOCK = 1,
  TT_CMD_RELOCK = 2,
  TT_CMD_RELEARN = 3
};

enum TtEventType : uint8_t {
  TT_EVT_ACTUATION = 1,   // an actuation window finished and was scored
  TT_EVT_IDLE_ALARM = 2,  // the idle monitor raised an alarm while locked
  TT_EVT_SENSOR_ERROR = 3 // the INA219 stopped answering
};

struct TtEvent {
  uint8_t type;           // TtEventType
  uint8_t kind;           // actuation kind (0 unlock, 1 relock) or TT_IDLE_* code
  int8_t status;          // TT_STATUS_* for actuations
  int8_t fault;           // TT_FAULT_* for actuations
  float score;            // actuation score, or CUSUM statistic / threshold
  float threshold;        // actuation threshold
  uint32_t onset_ms;      // idle alarms: estimated time since the force began
  float peak_ma;
  float plateau_ma;
};

#define TT_TRACE_CHUNK 32
struct TtTraceChunk {
  uint8_t phase;          // 0 unlock, 1 relock, 2 locked idle, 3 unlocked hold
  uint8_t n;
  uint16_t rate_hz;
  int16_t ma10[TT_TRACE_CHUNK];   // current in 0.1 mA
};

bool ttSenseBegin(int sda, int scl, int servoPin, int ledcChannel);
bool ttSenseCommand(TtCommand cmd);
bool ttSensePollEvent(TtEvent *out);
bool ttSensePollTrace(TtTraceChunk *out);
bool ttSenseBusy();             // a motion is running
bool ttSenseSensorOk();
uint16_t ttSenseLearned(uint8_t kind);
uint16_t ttSenseLearnNeeded();
bool ttSenseIdleArmed();
float ttSenseBusVolts();
const char *ttFaultName(int fault);

#endif  // TT_SENSE_H
