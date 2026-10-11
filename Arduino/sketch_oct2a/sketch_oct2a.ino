// MKS SERVO42C + Teensy 4.0: lidar shaft rotation with index sensor
//
// Serial commands (115200):
//   c   calibrate steps/rev (5 revs)
//   h   home: run to sensor edge, then move OFFSET_DEG further and HOLD (lidar should now be vertical)
//   + / -   offset +/- 1 deg   (then 'h' again to check)
//   ] / [   offset +/- 10 deg
//   g   go: continuous rotation at TARGET_RPM, prints steps/rev per revolution
//   x   stop and hold
//   r   reverse direction
const int PIN_STEP = 2;
const int PIN_DIR  = 3;
const int PIN_EN   = 4;
const int PIN_SENS = 5;            // light barrier, one pulse per lidar-shaft revolution
const int SENS_EDGE = FALLING;     // always use the SAME edge

const int   MICROSTEPS      = 16;
const int   MOTOR_STEPS_REV = 200 * MICROSTEPS;  // 3200 per motor-shaft rev

float calStepsPerRev = 31999.0;    // measured: total ratio 10:1. Press 'c' to re-measure.
float offsetDeg      = 93.0;       // measured: lidar is vertical 93 deg (8266 steps) after the sensor edge

const float CAL_STEPS_PER_S   = 8000.0;
const float HOME_STEPS_PER_S  = 3000.0;
const float RAMP_STEPS_PER_S2 = 8000.0;
const int   CAL_REVS          = 5;
const float TARGET_RPM        = 6.0;      // lidar can do 5-10 rpm
const uint32_t DEBOUNCE_US    = 50000;

enum Mode { HOLD, WAIT_FIRST_EDGE, CALIBRATING, RUNNING, HOMING, MOVING_OFFSET, JOGGING };
Mode mode = HOLD;

volatile uint32_t stepCount = 0;
volatile uint32_t edgeStep = 0;
volatile uint32_t lastEdgeUs = 0;
volatile bool     edgeFlag = false;

float    targetStepsPerS = 0;
float    currentStepsPerS = 0;
uint32_t lastStepUs = 0;
uint32_t lastRampUs = 0;
uint32_t stepsRemaining = 0;

uint32_t revSteps[CAL_REVS];
int      revIdx = 0;
uint32_t lastEdgeStep = 0;
bool     dirHigh = true;

void sensorISR() {
  uint32_t now = micros();
  if (now - lastEdgeUs < DEBOUNCE_US) return;
  lastEdgeUs = now;
  edgeStep = stepCount;
  edgeFlag = true;
}

uint32_t offsetSteps() {
  float d = fmodf(offsetDeg, 360.0f);
  if (d < 0) d += 360.0f;
  return (uint32_t)(d / 360.0f * calStepsPerRev + 0.5f);
}

void setMode(Mode m, float stepsPerS) {
  mode = m;
  targetStepsPerS = stepsPerS;
  edgeFlag = false;
}

void stopHold() {
  mode = HOLD;
  targetStepsPerS = 0;
  currentStepsPerS = 0;
}

// Move the lidar by deltaDeg right now (only when stopped) and update offsetDeg.
// Positive = further in the scan direction, negative = back.
void nudge(float deltaDeg) {
  if (mode != HOLD) { Serial.println(F("Stop first (x) to jog")); return; }
  offsetDeg += deltaDeg;
  stepsRemaining = (uint32_t)(fabsf(deltaDeg) / 360.0f * calStepsPerRev + 0.5f);
  bool forward = deltaDeg >= 0;
  digitalWrite(PIN_DIR, (forward == dirHigh) ? HIGH : LOW);
  delayMicroseconds(10);
  currentStepsPerS = 0;
  setMode(JOGGING, 2000.0f);
}

void printOffset() {
  Serial.print(F("offset: ")); Serial.print(offsetDeg, 1);
  Serial.print(F(" deg = ")); Serial.print(offsetSteps()); Serial.println(F(" steps after edge"));
}

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_SENS, INPUT_PULLUP);
  digitalWrite(PIN_DIR, HIGH);
  digitalWrite(PIN_EN, LOW);
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(PIN_SENS), sensorISR, SENS_EDGE);
  Serial.println(F("Ready. Commands: c calibrate | h home | + - [ ] offset | g go | x stop | r reverse"));
  printOffset();
}

void handleEdge() {
  noInterrupts();
  uint32_t s = edgeStep;
  edgeFlag = false;
  interrupts();

  if (mode == HOMING) {
    uint32_t done = stepCount - s;           // steps already taken since the edge
    uint32_t want = offsetSteps();
    stepsRemaining = (want > done) ? (want - done) : 0;
    Serial.print(F("edge found, moving ")); Serial.print(stepsRemaining); Serial.println(F(" steps"));
    mode = MOVING_OFFSET;
    return;
  }

  if (mode == WAIT_FIRST_EDGE) {
    lastEdgeStep = s;
    mode = CALIBRATING;
    Serial.println(F("First edge found, measuring revolutions..."));
    return;
  }

  uint32_t steps = s - lastEdgeStep;
  lastEdgeStep = s;

  if (mode == CALIBRATING) {
    revSteps[revIdx++] = steps;
    Serial.print(F("rev ")); Serial.print(revIdx);
    Serial.print(F(": ")); Serial.println(steps);
    if (revIdx >= CAL_REVS) {
      uint32_t mn = revSteps[0], mx = revSteps[0];
      double sum = 0;
      for (int i = 0; i < CAL_REVS; i++) {
        sum += revSteps[i];
        if (revSteps[i] < mn) mn = revSteps[i];
        if (revSteps[i] > mx) mx = revSteps[i];
      }
      calStepsPerRev = sum / CAL_REVS;
      Serial.print(F("=== mean steps/rev: ")); Serial.print(calStepsPerRev, 2);
      Serial.print(F("  ratio ")); Serial.print(calStepsPerRev / MOTOR_STEPS_REV, 4);
      Serial.print(F("  spread ")); Serial.println(mx - mn);
      stopHold();
    }
  } else if (mode == RUNNING) {
    Serial.print(F("rev steps: ")); Serial.print(steps);
    Serial.print(F("  dev: ")); Serial.println((float)steps - calStepsPerRev, 1);
  }
}

void handleSerial() {
  char c = Serial.read();
  switch (c) {
    case 'c': revIdx = 0; currentStepsPerS = 0; setMode(WAIT_FIRST_EDGE, CAL_STEPS_PER_S);
              Serial.println(F("Calibrating: waiting for first edge...")); break;
    case 'h': currentStepsPerS = 0; setMode(HOMING, HOME_STEPS_PER_S);
              Serial.println(F("Homing: searching edge...")); break;
    case 'g': setMode(RUNNING, TARGET_RPM / 60.0f * calStepsPerRev);
              Serial.print(F("Running at ")); Serial.print(TARGET_RPM); Serial.println(F(" rpm")); break;
    case 'x': stopHold(); Serial.println(F("Stopped")); break;
    case 'r': dirHigh = !dirHigh; digitalWrite(PIN_DIR, dirHigh ? HIGH : LOW);
              Serial.println(dirHigh ? F("dir HIGH") : F("dir LOW")); break;
    case '+': nudge(1);   break;
    case '-': nudge(-1);  break;
    case ']': nudge(10);  break;
    case '[': nudge(-10); break;
  }
}

void loop() {
  if (Serial.available()) handleSerial();
  if (edgeFlag) handleEdge();

  // Ramp step rate toward target
  uint32_t nowUs = micros();
  if (nowUs - lastRampUs >= 1000) {
    float dt = (nowUs - lastRampUs) * 1e-6f;
    lastRampUs = nowUs;
    float dv = RAMP_STEPS_PER_S2 * dt;
    if (currentStepsPerS < targetStepsPerS) currentStepsPerS = min(targetStepsPerS, currentStepsPerS + dv);
    else if (currentStepsPerS > targetStepsPerS) currentStepsPerS = max(targetStepsPerS, currentStepsPerS - dv);
  }
  if (mode == HOLD || currentStepsPerS < 1.0f) return;

  uint32_t intervalUs = (uint32_t)(1e6f / currentStepsPerS);
  nowUs = micros();
  if (nowUs - lastStepUs >= intervalUs) {
    lastStepUs += intervalUs;
    if (nowUs - lastStepUs > intervalUs * 4) lastStepUs = nowUs;
    digitalWriteFast(PIN_STEP, HIGH);
    delayMicroseconds(3);
    digitalWriteFast(PIN_STEP, LOW);
    stepCount++;

    if (mode == JOGGING) {
      if (stepsRemaining > 0) stepsRemaining--;
      if (stepsRemaining == 0) {
        stopHold();
        digitalWrite(PIN_DIR, dirHigh ? HIGH : LOW);   // restore scan direction
        printOffset();
      }
    }
    if (mode == MOVING_OFFSET) {
      if (stepsRemaining > 0) stepsRemaining--;
      if (stepsRemaining == 0) {
        stopHold();
        Serial.println(F("At offset position (should be vertical). Adjust with + - [ ] and press h again."));
      }
    }
  }
}
