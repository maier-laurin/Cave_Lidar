// MKS SERVO42C + BNO085 + Teensy: lidar shaft rotation with index sensor,
// streaming axis angle + frame orientation + dead-reckoned displacement over USB serial.
//
// Needs library: "Adafruit BNO08x" (Library Manager, accept dependencies)
//
// On startup the axis turns slowly until the index sensor edge (= axis angle 0),
// then continues at TARGET_RPM. axisSteps is re-zeroed at every sensor edge.
// After homing it stops and waits. Serial commands (115200):  s = start rotation, x = stop and hold
//
// Data lines start with "D," (see WIRING.md); every other line is a human readable message.
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
const float HOME_STEPS_PER_S = 3000.0;    // search speed until the first edge
const float RAMP_STEPS_PER_S2 = 8000.0;
const float MIN_START_STEPS_PER_S = 400.0; // timer period is only updated after the current one ends
const float TARGET_RPM        = 10.0;      // lidar can do 5-10 rpm
const uint32_t DEBOUNCE_US    = 50000;

// ---- BNO085 (I2C on Wire: SDA = 18, SCL = 19) ----
const int      PIN_BNO_RST = 17;          // INT is on pin 16 (unused, never drive it)
const uint8_t  BNO_ADDR    = 0x4A;        // 0x4B if the address jumper is bridged
const uint32_t REPORT_US   = 10000;       // IMU sampled at 100 Hz (dead reckoning keeps full rate)
const uint32_t STREAM_MS   = 1000;        // serial output rate: 1 Hz for testing (use 10 for 100 Hz)

// Treat the frame as stationary (zero velocity) when low-passed |linear accel| stays below this.
// Tune against motor vibration.
const float ZUPT_ACCEL = 0.15f;           // m/s^2

enum Mode { HOLD, HOMING, MOVING_ZERO, RUNNING };
Mode mode = HOLD;

IntervalTimer stepTimer;
bool timerRunning = false;

// shared with the ISRs
volatile int32_t  axisSteps = 0;       // signed steps since the last sensor edge, re-zeroed at every edge
volatile uint32_t lastEdgeUs = 0;
volatile bool     edgeFlag = false;
volatile bool     edgeEver = false;    // an edge has been seen since power-up: axisSteps is valid
volatile bool     edgeSinceLine = false;
volatile bool     stepGate = false;    // ISR only steps while true
volatile bool     seekZero = false;    // ISR stops the motor once axisSteps reaches zeroSteps (vertical)
volatile bool     moveDone = false;
int32_t           zeroSteps = 0;

float    targetStepsPerS = 0;
float    currentStepsPerS = 0;
uint32_t lastRampUs = 0;

// ---- IMU state ----
Adafruit_BNO08x bno(PIN_BNO_RST);
sh2_SensorValue_t sv;
bool imuOk = false;
uint32_t lastImuTryMs = 0;
uint32_t lastStreamMs = 0;

// latest orientation: frame -> world (BNO rotation vector, world = East/North/Up)
float qw = 1, qx = 0, qy = 0, qz = 0;
float qAccDeg = 0;
uint8_t calStatus = 0;
bool haveQuat = false;

// latest linear acceleration (sensor frame, gravity removed)
float la[3] = {0, 0, 0};

// dead reckoning in world frame, accumulated since the last streamed packet
float vel[3] = {0, 0, 0};
float dpos[3] = {0, 0, 0};
float accLp = 0;
uint32_t lastAccUs = 0;

// ---------------------------------------------------------------- motor

// micros() wraps every 71 min; extend to 64 bit (must be called more often than that)
uint64_t micros64() {
  static uint32_t hi = 0, last = 0;
  uint32_t now = micros();
  if (now < last) hi++;
  last = now;
  return ((uint64_t)hi << 32) | now;
}

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
  axisSteps = 0;                   // re-zero every turn: corrects belt slippage
  edgeEver = true;
  edgeSinceLine = true;
  edgeFlag = true;
}

// 0 deg = lidar vertical, which is VERTICAL_AFTER_EDGE_DEG past the sensor edge (so the edge itself reads 267)
float axisAngleDeg() {
  float a = fmodf((float)axisSteps * 360.0f / STEPS_PER_REV - VERTICAL_AFTER_EDGE_DEG, 360.0f);
  if (a < 0) a += 360.0f;
  return a;
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
  Serial.println(F("Homing: searching edge..."));
}

void startRunning() {
  currentStepsPerS = 0;
  mode = RUNNING;
  targetStepsPerS = TARGET_RPM / 60.0f * STEPS_PER_REV;
  Serial.print(F("Running at ")); Serial.print(TARGET_RPM); Serial.println(F(" rpm"));
}

void handleEdge() {
  edgeFlag = false;
  if (mode == HOMING) {
    // keep turning at homing speed until the vertical position (0 deg), the ISR stops it there
    mode = MOVING_ZERO;
    zeroSteps = (int32_t)(VERTICAL_AFTER_EDGE_DEG / 360.0f * STEPS_PER_REV + 0.5f);
    seekZero = true;
    Serial.println(F("Edge found, moving to 0 deg..."));
  }
}

void handleMoveDone() {
  moveDone = false;
  stopHold();
  Serial.print(F("Homed, at ")); Serial.print(axisAngleDeg(), 1);
  Serial.println(F(" deg. Send 's' to start."));
}

void handleSerial() {
  switch (Serial.read()) {
    case 'x': stopHold(); Serial.println(F("Stopped")); break;
    case 's': if (mode == HOLD) { if (edgeEver) startRunning(); else startHoming(); } break;
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
    if (!moveDone) stepGate = true;
  }
}

// ---------------------------------------------------------------- IMU

// v_world = q * v * q^-1
void rotateToWorld(const float v[3], float out[3]) {
  float tx = 2 * (qy * v[2] - qz * v[1]);
  float ty = 2 * (qz * v[0] - qx * v[2]);
  float tz = 2 * (qx * v[1] - qy * v[0]);
  out[0] = v[0] + qw * tx + (qy * tz - qz * ty);
  out[1] = v[1] + qw * ty + (qz * tx - qx * tz);
  out[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}

void setReports() {
  bno.enableReport(SH2_ROTATION_VECTOR, REPORT_US);
  bno.enableReport(SH2_LINEAR_ACCELERATION, REPORT_US);
}

bool startImu() {
  if (!bno.begin_I2C(BNO_ADDR, &Wire)) return false;
  Wire.setClock(400000);
  setReports();
  haveQuat = false;
  lastAccUs = 0;
  return true;
}

void handleLinearAccel() {
  la[0] = sv.un.linearAcceleration.x;
  la[1] = sv.un.linearAcceleration.y;
  la[2] = sv.un.linearAcceleration.z;

  uint32_t now = micros();
  float dt = (now - lastAccUs) * 1e-6f;
  bool first = (lastAccUs == 0);
  lastAccUs = now;
  if (first || !haveQuat || dt > 0.1f) return;

  float aw[3];
  rotateToWorld(la, aw);

  float mag = sqrtf(aw[0] * aw[0] + aw[1] * aw[1] + aw[2] * aw[2]);
  accLp += 0.05f * (mag - accLp);
  if (accLp < ZUPT_ACCEL) {
    vel[0] = vel[1] = vel[2] = 0;
    return;
  }
  for (int i = 0; i < 3; i++) {
    dpos[i] += vel[i] * dt + 0.5f * aw[i] * dt * dt;
    vel[i]  += aw[i] * dt;
  }
}

void handleRotationAndStream() {
  qw = sv.un.rotationVector.real;
  qx = sv.un.rotationVector.i;
  qy = sv.un.rotationVector.j;
  qz = sv.un.rotationVector.k;
  qAccDeg = sv.un.rotationVector.accuracy * 57.29578f;
  calStatus = sv.status & 0x03;
  haveQuat = true;

  if (millis() - lastStreamMs < STREAM_MS) return;
  lastStreamMs = millis();

  bool idx = edgeSinceLine;      // stays set until the next streamed line, so no edge is missed
  edgeSinceLine = false;
  // t_s, axis_deg, homed, idx, qw,qx,qy,qz, q_acc_deg, cal(0-3), dx,dy,dz [m], lax,lay,laz [m/s^2]
  Serial.printf("D,%.6f,%.3f,%d,%d,%.5f,%.5f,%.5f,%.5f,%.2f,%u,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f\n",
                micros64() * 1e-6, axisAngleDeg(), edgeEver, idx, qw, qx, qy, qz, qAccDeg, calStatus,
                dpos[0], dpos[1], dpos[2], la[0], la[1], la[2]);
  dpos[0] = dpos[1] = dpos[2] = 0;
}

void pollImu() {
  if (!imuOk) {
    if (millis() - lastImuTryMs >= 1000) {
      lastImuTryMs = millis();
      imuOk = startImu();
      Serial.println(imuOk ? "# BNO085 ok" : "# BNO085 not found");
    }
    return;
  }
  if (bno.wasReset()) setReports();
  while (bno.getSensorEvent(&sv)) {
    if (sv.sensorId == SH2_LINEAR_ACCELERATION) handleLinearAccel();
    else if (sv.sensorId == SH2_ROTATION_VECTOR) handleRotationAndStream();
  }
}

// ---------------------------------------------------------------- main

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_SENS, INPUT_PULLUP);
  digitalWrite(PIN_DIR, HIGH);
  digitalWrite(PIN_EN, LOW);
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(PIN_SENS), sensorISR, SENS_EDGE);
  Serial.println(F("# D,t_s,axis_deg,homed,idx,qw,qx,qy,qz,q_acc_deg,cal,dx,dy,dz,lax,lay,laz"));
  Serial.println(F("Commands: s = start | x = stop"));
  lastImuTryMs = millis();       // pollImu() starts the IMU 1 s from now
  startHoming();
}

void loop() {
  if (Serial.available()) handleSerial();
  if (edgeFlag) handleEdge();
  if (moveDone) handleMoveDone();
  updateStepping();
  pollImu();
}
