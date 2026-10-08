// Lidar rotation controller: Teensy 4.1 + MKS SERVO42C + BNO085 + fake GPS for the Velodyne VLP-16.
//
// Needs library: "Adafruit BNO08x" (Library Manager, accept dependencies)
//
// Sequence:
//   1. Power-up: the axis homes to the index sensor and moves to 0 deg (vertical), then holds.
//   2. Idle until a config line arrives over USB serial:   C,<rpm>,<unix_seconds>\n
//        rpm           1..20
//        unix_seconds  UTC time label of the FIRST PPS pulse, which fires 1 s after the line arrives
//   3. PPS (pin 10) + $GPRMC (Serial1, pin 1) start, the binary data stream starts.
//   4. START_DELAY_MS later the axis ramps to <rpm> and keeps turning.
//   5. "x" (single char, or "x\n") ramps down, runs on to the next index edge and stops at 0 deg.
//      PPS, NMEA and the data stream keep running.
//
// All USB output is binary packets, see PROTOCOL below. Everything shares ONE clock: the simulated
// UTC time that the PPS/NMEA output also uses.
//
// PROTOCOL (little endian):  0xA5 | type u8 | len u8 | payload[len] | crc16 (CCITT-FALSE over type,len,payload)
//   'H' header   (at stream start, then every 10 s): u8 version, f32 steps_per_rev, f32 vertical_after_edge_deg,
//                f32 rpm, u32 first_pulse_unix_s
//   'S' sample   (100 Hz, one per BNO rotation-vector report): u64 t_us (unix us), i32 axis_steps,
//                f32 qw,qx,qy,qz, u16 q_acc (0.01 deg), u8 cal (bits0-1 rotation vector, bits2-3 magnetometer),
//                f32 lax,lay,laz (m/s^2, gravity removed), f32 mx,my,mz (uT), u8 flags (bit0 homed, bit1 running),
//                u8 drops (wrapping count of packets dropped because USB was full)
//   'E' edge     (once per index-sensor edge): u64 t_us, i32 steps_before_reset (~steps_per_rev if no slip;
//                meaningless for the first edge after a start)
//   'M' message  (human readable text)
//   axis angle [deg] = fmod(axis_steps * 360 / steps_per_rev - vertical_after_edge_deg, 360), made positive
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ---- Motor / index sensor ----
const int PIN_STEP = 2;
const int PIN_DIR  = 3;
const int PIN_EN   = 4;
const int PIN_SENS = 5;            // light barrier, one pulse per lidar-shaft revolution
const int SENS_EDGE = FALLING;     // always use the SAME edge

const float STEPS_PER_REV     = 31999.0;   // measured: total ratio 10:1 (3200 steps per motor rev)
const float VERTICAL_AFTER_EDGE_DEG = 93.0;  // measured: lidar is vertical 93 deg after the sensor edge
const float HOME_STEPS_PER_S = 3000.0;    // search speed until the first edge, also used for stopping
const float RAMP_STEPS_PER_S2 = 8000.0;
const float MIN_START_STEPS_PER_S = 400.0; // timer period is only updated after the current one ends
const float MIN_RPM = 1.0, MAX_RPM = 20.0;
const uint32_t DEBOUNCE_US    = 50000;
const uint32_t START_DELAY_MS = 10000;     // rotation starts this long after the config line

// ---- Fake GPS for the Velodyne ----
const int      PIN_PPS = 10;
const uint32_t PPS_WIDTH_MS = 10;
const uint32_t NMEA_DELAY_MS = 100;        // sentence goes out this long after the PPS edge

// ---- BNO085 (I2C on Wire: SDA = 18, SCL = 19) ----
const int      PIN_BNO_RST = 17;          // INT is on pin 16 (unused, never drive it)
const uint8_t  BNO_ADDR    = 0x4A;        // 0x4B if the address jumper is bridged
const uint32_t REPORT_US   = 10000;       // 100 Hz
const uint32_t IMU_SILENT_MS = 3000;

enum Mode { HOLD, HOMING, MOVING_ZERO, RUNNING, STOPPING };
Mode mode = HOLD;

IntervalTimer stepTimer;
IntervalTimer ppsTimer;
bool timerRunning = false;

// shared with the ISRs
volatile int32_t  axisSteps = 0;       // signed steps since the last sensor edge, re-zeroed at every edge
volatile uint32_t lastEdgeUs = 0;
volatile bool     edgeFlag = false;
volatile int32_t  edgeStepsBefore = 0; // axisSteps just before the re-zero
volatile uint32_t edgeTimeUs = 0;
volatile bool     edgeEver = false;    // an edge has been seen since power-up: axisSteps is valid
volatile bool     stepGate = false;    // ISR only steps while true
volatile bool     seekZero = false;    // ISR stops the motor once axisSteps reaches zeroSteps (vertical)
volatile bool     moveDone = false;
int32_t           zeroSteps = 0;

volatile uint32_t ppsCount = 0;        // PPS edges since the config line
volatile uint32_t ppsEdgeUs = 0;       // micros() of the latest PPS edge
volatile bool     ppsFlag = false;

float targetStepsPerS = 0;
float currentStepsPerS = 0;
uint32_t lastRampUs = 0;

// ---- run configuration ----
bool     configured = false;
float    cfgRpm = 0;
uint32_t firstLabelUnix = 0;           // UTC label (unix s) of PPS pulse #1
bool     pendingStart = false;
uint32_t startAtMs = 0;

// ---- IMU state ----
Adafruit_BNO08x bno(PIN_BNO_RST);
sh2_SensorValue_t sv;
bool imuOk = false;
uint32_t lastImuTryMs = 0;
uint32_t lastImuEventMs = 0;
bool imuSilentReported = false;

float la[3]  = {NAN, NAN, NAN};        // linear acceleration, sensor frame (gravity removed)
float mag[3] = {NAN, NAN, NAN};        // calibrated magnetic field, uT
uint8_t magCal = 0;

// ---------------------------------------------------------------- binary output

#pragma pack(push, 1)
struct PktH { uint8_t version; float stepsPerRev; float verticalDeg; float rpm; uint32_t firstUnix; };
struct PktS {
  uint64_t t_us; int32_t steps; float q[4]; uint16_t qAcc; uint8_t cal;
  float la[3]; float mag[3]; uint8_t flags; uint8_t drops;
};
struct PktE { uint64_t t_us; int32_t stepsBefore; };
#pragma pack(pop)

uint8_t dropCount = 0;

uint16_t crc16(const uint8_t *p, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(*p++) << 8;
    for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

// never blocks: if USB has no room (nobody reading) the packet is dropped
bool sendPacket(char type, const void *payload, uint8_t len) {
  uint8_t buf[3 + 255 + 2];
  buf[0] = 0xA5; buf[1] = (uint8_t)type; buf[2] = len;
  memcpy(buf + 3, payload, len);
  uint16_t crc = crc16(buf + 1, 2 + len);
  buf[3 + len] = crc & 0xFF;
  buf[4 + len] = crc >> 8;
  size_t total = 5 + len;
  if (Serial.availableForWrite() < (int)total) { dropCount++; return false; }
  Serial.write(buf, total);
  return true;
}

void msg(const char *fmt, ...) {
  char t[120];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(t, sizeof t, fmt, ap);
  va_end(ap);
  if (n > (int)sizeof t - 1) n = sizeof t - 1;
  if (n > 0) sendPacket('M', t, n);
}

// ---------------------------------------------------------------- time (shared with PPS / NMEA)

// micros() wraps every 71 min; this only needs the 32 bit difference to the last PPS edge.
// Returns unix microseconds for the local micros() value tm, or -1 while no PPS edge has happened yet.
int64_t simTimeUs(uint32_t tm) {
  uint32_t cnt, edge;
  __disable_irq();
  cnt = ppsCount; edge = ppsEdgeUs;
  __enable_irq();
  if (cnt == 0) return -1;
  int32_t delta = (int32_t)(tm - edge);       // negative if tm is before the latest edge
  return ((int64_t)firstLabelUnix + (int64_t)(cnt - 1)) * 1000000LL + delta;
}

bool streaming() { return configured && ppsCount > 0; }

// days since 1970-01-01 -> y/m/d (Howard Hinnant's civil_from_days)
void civilFromDays(int32_t z, int &y, int &m, int &d) {
  z += 719468;
  int32_t era = (z >= 0 ? z : z - 146096) / 146097;
  uint32_t doe = z - era * 146097;
  uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  y = yoe + era * 400;
  uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  uint32_t mp = (5 * doy + 2) / 153;
  d = doy - (153 * mp + 2) / 5 + 1;
  m = mp < 10 ? mp + 3 : mp - 9;
  y += (m <= 2);
}

void sendNmea(uint32_t unixS) {
  uint32_t sod = unixS % 86400;
  int y, m, d;
  civilFromDays(unixS / 86400, y, m, d);
  char body[100];
  snprintf(body, sizeof body, "GPRMC,%02lu%02lu%02lu.00,A,0000.0000,N,00000.0000,E,0.0,0.0,%02d%02d%02d,,,A",
           sod / 3600, (sod / 60) % 60, sod % 60, d, m, y % 100);
  uint8_t cs = 0;
  for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
  Serial1.printf("$%s*%02X\r\n", body, cs);   // buffered (addMemoryForWrite), does not block
}

void ppsISR() {
  digitalWriteFast(PIN_PPS, HIGH);
  ppsEdgeUs = micros();
  ppsCount++;
  ppsFlag = true;
}

void sendHeader() {
  PktH h = {1, STEPS_PER_REV, VERTICAL_AFTER_EDGE_DEG, cfgRpm, firstLabelUnix};
  sendPacket('H', &h, sizeof h);
}

// PPS bookkeeping in the main loop: lower the pulse, send NMEA + header
void servicePps() {
  static uint32_t edgeMs = 0;
  static bool pulseHigh = false, sentenceDue = false;
  static uint32_t sentenceCount = 0;

  if (ppsFlag) {
    ppsFlag = false;
    edgeMs = millis();
    pulseHigh = true;
    sentenceDue = true;
    sentenceCount = ppsCount;
  }
  if (pulseHigh && millis() - edgeMs >= PPS_WIDTH_MS) {
    digitalWriteFast(PIN_PPS, LOW);
    pulseHigh = false;
  }
  if (sentenceDue && millis() - edgeMs >= NMEA_DELAY_MS) {
    sentenceDue = false;
    sendNmea(firstLabelUnix + sentenceCount - 1);          // describes the pulse that just happened
    if ((sentenceCount - 1) % 10 == 0) sendHeader();
  }
}

// ---------------------------------------------------------------- motor

void stepISR() {
  if (!stepGate) return;
  digitalWriteFast(PIN_STEP, HIGH);
  delayMicroseconds(3);
  digitalWriteFast(PIN_STEP, LOW);
  axisSteps++;
  if (seekZero && axisSteps >= zeroSteps) {
    stepGate = false;
    seekZero = false;
    moveDone = true;
  }
}

void sensorISR() {
  uint32_t now = micros();
  if (now - lastEdgeUs < DEBOUNCE_US) return;
  lastEdgeUs = now;
  __disable_irq();                 // read + re-zero must not lose a step in between
  edgeStepsBefore = axisSteps;
  axisSteps = 0;                   // re-zero every turn: corrects belt slippage
  __enable_irq();
  edgeTimeUs = now;
  edgeEver = true;
  edgeFlag = true;
}

void stopHold() {
  stepGate = false;
  seekZero = false;
  mode = HOLD;
  targetStepsPerS = 0;
  currentStepsPerS = 0;
}

void startHoming() {
  currentStepsPerS = 0;
  mode = HOMING;
  targetStepsPerS = HOME_STEPS_PER_S;
  msg("Homing: searching edge...");
}

void startRunning() {
  currentStepsPerS = 0;
  mode = RUNNING;
  targetStepsPerS = cfgRpm / 60.0f * STEPS_PER_REV;
  msg("Running at %.1f rpm", cfgRpm);
}

// after an edge: keep turning at low speed until vertical (0 deg), the ISR stops it there
void seekVertical() {
  mode = MOVING_ZERO;
  targetStepsPerS = min(targetStepsPerS, HOME_STEPS_PER_S);
  zeroSteps = (int32_t)(VERTICAL_AFTER_EDGE_DEG / 360.0f * STEPS_PER_REV + 0.5f);
  seekZero = true;
}

void handleEdge() {
  int32_t before;
  uint32_t tUs;
  __disable_irq();
  edgeFlag = false;
  before = edgeStepsBefore;
  tUs = edgeTimeUs;
  __enable_irq();

  if (mode == HOMING) {
    seekVertical();
    msg("Edge found, moving to 0 deg...");
  } else if (mode == STOPPING) {
    seekVertical();
    msg("Edge found, stopping at 0 deg...");
  }
  if (streaming()) {
    PktE e = {(uint64_t)max<int64_t>(simTimeUs(tUs), 0), before};
    sendPacket('E', &e, sizeof e);
  }
}

void handleMoveDone() {
  moveDone = false;
  stopHold();
  msg("At 0 deg, stopped.");
}

void handleStop() {
  pendingStart = false;
  if (mode == RUNNING) {
    targetStepsPerS = min(targetStepsPerS, HOME_STEPS_PER_S);
    mode = STOPPING;
    msg("Stopping: decelerating, will stop at 0 deg");
  } else if (mode == HOLD) {
    msg("Start cancelled");
  } else {
    msg("Busy homing/stopping, wait");
  }
}

void handleConfig(float rpm, uint32_t unixS) {
  if (mode == RUNNING || mode == STOPPING) { msg("C rejected: rotation active, send x first"); return; }
  if (!(rpm >= MIN_RPM && rpm <= MAX_RPM)) { msg("C rejected: rpm must be %.0f..%.0f", MIN_RPM, MAX_RPM); return; }
  cfgRpm = rpm;
  firstLabelUnix = unixS;
  ppsTimer.end();
  __disable_irq(); ppsCount = 0; ppsFlag = false; __enable_irq();
  digitalWriteFast(PIN_PPS, LOW);
  configured = true;
  ppsTimer.begin(ppsISR, 1000000);       // first pulse 1 s from now = label unixS
  ppsTimer.priority(8);                  // above the step timer: the PPS edge is never delayed
  pendingStart = true;
  startAtMs = millis() + START_DELAY_MS;
  msg("Config ok: %.1f rpm, first pulse = unix %lu, rotation starts in %lu s",
      rpm, (unsigned long)unixS, (unsigned long)(START_DELAY_MS / 1000));
}

void handleLine(char *line) {
  if (line[0] == 'C' && line[1] == ',') {
    char *p;
    float rpm = strtof(line + 2, &p);
    if (*p != ',') { msg("C rejected: format is C,<rpm>,<unix_seconds>"); return; }
    handleConfig(rpm, strtoul(p + 1, &p, 10));
  } else {
    msg("Unknown command: %s", line);
  }
}

void handleSerial() {
  static char line[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) { line[n] = 0; handleLine(line); n = 0; }
    } else if (n == 0 && c == 'x') {
      handleStop();
    } else if (n < sizeof(line) - 1) {
      line[n++] = c;
    }
  }
}

// Ramp step rate toward target, then hand it to the step timer
void updateStepping() {
  uint32_t nowUs = micros();
  if (nowUs - lastRampUs >= 1000) {
    float dt = (nowUs - lastRampUs) * 1e-6f;
    lastRampUs = nowUs;
    float dv = RAMP_STEPS_PER_S2 * dt;
    if (currentStepsPerS < targetStepsPerS) currentStepsPerS = min(targetStepsPerS, currentStepsPerS + dv);
    else if (currentStepsPerS > targetStepsPerS) currentStepsPerS = max(targetStepsPerS, currentStepsPerS - dv);
    if (targetStepsPerS > 0 && currentStepsPerS < MIN_START_STEPS_PER_S)
      currentStepsPerS = min(targetStepsPerS, MIN_START_STEPS_PER_S);

    if (mode == HOLD || currentStepsPerS < 1.0f) return;
    float intervalUs = 1e6f / currentStepsPerS;
    if (!timerRunning) {
      stepTimer.begin(stepISR, intervalUs);
      stepTimer.priority(16);      // above I2C so IMU reads never delay steps
      timerRunning = true;
    } else {
      stepTimer.update(intervalUs);
    }
    if (!moveDone && (mode != MOVING_ZERO || seekZero)) stepGate = true;
  }
}

void servicePendingStart() {
  if (pendingStart && mode == HOLD && edgeEver && (int32_t)(millis() - startAtMs) >= 0) {
    pendingStart = false;
    startRunning();
  }
}

// ---------------------------------------------------------------- IMU

void setReports() {
  bno.enableReport(SH2_ROTATION_VECTOR, REPORT_US);          // north referenced (uses the magnetometer)
  bno.enableReport(SH2_LINEAR_ACCELERATION, REPORT_US);
  bno.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, REPORT_US);
}

bool startImu() {
  if (!bno.begin_I2C(BNO_ADDR, &Wire)) return false;
  Wire.setClock(400000);
  setReports();
  return true;
}

void handleRotationAndStream() {
  uint32_t tm = micros();
  if (!streaming()) return;

  PktS s;
  s.t_us = (uint64_t)max<int64_t>(simTimeUs(tm), 0);
  s.steps = axisSteps;
  s.q[0] = sv.un.rotationVector.real;
  s.q[1] = sv.un.rotationVector.i;
  s.q[2] = sv.un.rotationVector.j;
  s.q[3] = sv.un.rotationVector.k;
  float accCdeg = sv.un.rotationVector.accuracy * 5729.578f;   // rad -> 0.01 deg
  s.qAcc = accCdeg > 65535.0f ? 65535 : (uint16_t)accCdeg;
  s.cal = (sv.status & 0x03) | (magCal << 2);
  memcpy(s.la, la, sizeof la);
  memcpy(s.mag, mag, sizeof mag);
  s.flags = (edgeEver ? 1 : 0) | (mode == RUNNING ? 2 : 0);
  s.drops = dropCount;
  sendPacket('S', &s, sizeof s);
}

void pollImu() {
  if (!imuOk) {
    if (millis() - lastImuTryMs >= 1000) {
      lastImuTryMs = millis();
      imuOk = startImu();
      lastImuEventMs = millis();
      imuSilentReported = false;
      msg(imuOk ? "BNO085 ok" : "BNO085 not found");
    }
    return;
  }
  if (bno.wasReset()) setReports();
  while (bno.getSensorEvent(&sv)) {
    lastImuEventMs = millis();
    imuSilentReported = false;
    switch (sv.sensorId) {
      case SH2_LINEAR_ACCELERATION:
        la[0] = sv.un.linearAcceleration.x;
        la[1] = sv.un.linearAcceleration.y;
        la[2] = sv.un.linearAcceleration.z;
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        mag[0] = sv.un.magneticField.x;
        mag[1] = sv.un.magneticField.y;
        mag[2] = sv.un.magneticField.z;
        magCal = sv.status & 0x03;
        break;
      case SH2_ROTATION_VECTOR:
        handleRotationAndStream();
        break;
    }
  }
  if (!imuSilentReported && millis() - lastImuEventMs > IMU_SILENT_MS) {
    imuSilentReported = true;
    msg("BNO085 sends no data: unplug and replug USB (known issue after an upload)");
  }
}

// ---------------------------------------------------------------- main

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_PPS, OUTPUT);
  pinMode(PIN_SENS, INPUT_PULLUP);
  digitalWrite(PIN_PPS, LOW);
  digitalWrite(PIN_DIR, HIGH);
  digitalWrite(PIN_EN, LOW);
  Serial.begin(115200);
  static uint8_t nmeaBuf[160];
  Serial1.addMemoryForWrite(nmeaBuf, sizeof nmeaBuf);   // extra TX buffer so NMEA never blocks the loop
  Serial1.begin(9600);
  attachInterrupt(digitalPinToInterrupt(PIN_SENS), sensorISR, SENS_EDGE);
  lastImuTryMs = millis();       // pollImu() starts the IMU 1 s from now
  startHoming();
}

void loop() {
  if (Serial.available()) handleSerial();
  if (edgeFlag) handleEdge();
  if (moveDone) handleMoveDone();
  updateStepping();
  servicePendingStart();
  servicePps();
  pollImu();
}
