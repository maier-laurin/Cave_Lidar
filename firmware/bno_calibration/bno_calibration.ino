// BNO085 calibration helper for the cave lidar rig (Teensy 4.1). Not the scan firmware: after the calibration
// upload lidar_ctrl again.
//
// What it does: runs the BNO085 with its dynamic calibration (accelerometer, gyroscope, magnetometer) switched on and
// prints the calibration state twice a second on the USB serial port (Arduino serial monitor, any baud rate):
//
//   t 123.4 s | cal mag 2/3 accel 3/3 gyro 3/3 rot 3/3 | heading acc 18.2 deg | |B| 47.2 uT | BNO y heading 87.3 deg | tilt 0.8 deg
//
//   cal      status 0..3 of the sensors (3 = best). Move the rig in figure-8 movements until mag shows 3/3.
//   |B|      magnetic field strength; the earth field is about 45-50 uT in central Europe, much more means disturbance.
//   heading  compass heading of the BNO y axis (rig X axis, nominal mounting), clockwise from magnetic north.
//   tilt     angle between the BNO z axis and vertical.
//
// The calibration is stored in the BNO's flash (DCD): automatically once the magnetometer has been at 3/3 for 10 s,
// and on demand with the serial command 's'. Commands (single character): s = save now, ? = help,
// r = restart the BNO and its reports, C (capital) = ERASE the stored calibration and restart.
//
// The motor driver pins are set as in lidar_ctrl (driver enabled, no stepping), so the magnetic environment is the
// same as during a scan. Power the lidar as well when calibrating.
#include <Wire.h>
#include <Adafruit_BNO08x.h>

const int      PIN_STEP = 2, PIN_DIR = 3, PIN_EN = 4, PIN_PPS = 10;
const int      PIN_BNO_RST = 17;          // INT is on pin 16 (unused, never drive it)
const uint8_t  BNO_ADDR    = 0x4A;
const uint32_t PRINT_MS    = 500;
const uint32_t SILENT_MS   = 3000;
const uint32_t AUTOSAVE_AFTER_MS = 10000;

Adafruit_BNO08x bno(PIN_BNO_RST);
sh2_SensorValue_t sv;
bool imuOk = false;
uint32_t lastTryMs = 0, lastEventMs = 0, lastPrintMs = 0, lastSilentMsg = 0, magGoodSinceMs = 0;
bool autoSaved = false, calConfigOk = false;
uint32_t lastCfgMs = 0;

int8_t calMag = -1, calAccel = -1, calGyro = -1, calRot = -1;   // -1: no report yet
float  q[4] = {1, 0, 0, 0}, headingAcc = NAN, field = NAN;
bool   haveQ = false;

void setReports() {
  bno.enableReport(SH2_ROTATION_VECTOR, 10000);
  bno.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 50000);
  bno.enableReport(SH2_ACCELEROMETER, 50000);
  bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 50000);
  calConfigOk = false;               // set by tryCalConfig(), retried until the BNO accepts it
}

// the BNO may refuse the command right after a (re)start, so it is repeated every 2 s until it works
void tryCalConfig() {
  if (calConfigOk || millis() - lastCfgMs < 2000) return;
  lastCfgMs = millis();
  int r = sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
  if (r == SH2_OK) {
    calConfigOk = true;
    Serial.println("dynamic calibration of accelerometer, gyroscope and magnetometer is on");
  } else {
    Serial.printf("dynamic calibration not set yet (code %d), retrying\n", r);
  }
}

bool startImu() {
  if (!bno.begin_I2C(BNO_ADDR, &Wire)) return false;
  Wire.setClock(400000);
  setReports();
  return true;
}

void saveCalibration(const char *why) {
  int r = sh2_saveDcdNow();
  Serial.printf(">>> calibration %s: %s (code %d)\n", why, r == SH2_OK ? "SAVED to the BNO flash" : "SAVE FAILED", r);
}

void help() {
  Serial.println("commands: s = save calibration now, r = restart the BNO, C = ERASE stored calibration, ? = this text");
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's') saveCalibration("on request");
    else if (c == 'r') { imuOk = false; lastTryMs = 0; magGoodSinceMs = 0; Serial.println("restarting the BNO..."); }
    else if (c == 'C') {
      int r = sh2_clearDcdAndReset();
      Serial.printf(">>> stored calibration erased and BNO reset (code %d)\n", r);
      autoSaved = false; magGoodSinceMs = 0; calMag = calAccel = calGyro = calRot = -1;
    }
    else if (c == '?' || c == 'h') help();
  }
}

void printLine() {
  auto st = [](int8_t v, char *b) { if (v < 0) strcpy(b, "-"); else sprintf(b, "%d/3", v); };
  char m[8], a[8], g[8], r[8];
  st(calMag, m); st(calAccel, a); st(calGyro, g); st(calRot, r);
  float heading = NAN, tilt = NAN;
  if (haveQ) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    heading = atan2f(2 * (x * y - w * z), 1 - 2 * (x * x + z * z)) * 57.29578f;   // BNO y axis: east / north components
    if (heading < 0) heading += 360;
    tilt = acosf(constrain(1 - 2 * (x * x + y * y), -1.0f, 1.0f)) * 57.29578f;
  }
  Serial.printf("t %6.1f s | cal mag %s accel %s gyro %s rot %s | heading acc %5.1f deg | |B| %5.1f uT | BNO y heading %5.1f deg | tilt %4.1f deg%s\n",
                millis() / 1000.0f, m, a, g, r, headingAcc, field, heading, tilt, autoSaved ? " | saved" : "");
}

void pollImu() {
  if (!imuOk) {
    if (millis() - lastTryMs >= 1000) {
      lastTryMs = millis();
      imuOk = startImu();
      lastEventMs = millis();
      Serial.println(imuOk ? "BNO085 ok" : "BNO085 not found");
    }
    return;
  }
  if (bno.wasReset()) { Serial.println("BNO reset, reports enabled again"); setReports(); }
  tryCalConfig();
  while (bno.getSensorEvent(&sv)) {
    lastEventMs = millis();
    switch (sv.sensorId) {
      case SH2_ROTATION_VECTOR:
        q[0] = sv.un.rotationVector.real; q[1] = sv.un.rotationVector.i;
        q[2] = sv.un.rotationVector.j;    q[3] = sv.un.rotationVector.k;
        headingAcc = sv.un.rotationVector.accuracy * 57.29578f;
        calRot = sv.status & 0x03; haveQ = true;
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED: {
        float x = sv.un.magneticField.x, y = sv.un.magneticField.y, z = sv.un.magneticField.z;
        field = sqrtf(x * x + y * y + z * z);
        calMag = sv.status & 0x03;
        break;
      }
      case SH2_ACCELEROMETER:        calAccel = sv.status & 0x03; break;
      case SH2_GYROSCOPE_CALIBRATED: calGyro = sv.status & 0x03; break;
    }
  }
  if (millis() - lastEventMs > SILENT_MS && millis() - lastSilentMsg > 5000) {
    lastSilentMsg = millis();
    Serial.println("BNO085 sends no data: unplug and replug USB (known issue after an upload)");
  }

  // store the calibration once the magnetometer has been good for a while
  if (calMag == 3) {
    if (magGoodSinceMs == 0) magGoodSinceMs = millis();
    if (!autoSaved && millis() - magGoodSinceMs >= AUTOSAVE_AFTER_MS) {
      autoSaved = true;
      saveCalibration("magnetometer 3/3 for 10 s");
    }
  } else {
    magGoodSinceMs = 0;
  }
}

void setup() {
  // same pin states as lidar_ctrl while holding: driver enabled, no steps
  pinMode(PIN_STEP, OUTPUT); pinMode(PIN_DIR, OUTPUT); pinMode(PIN_EN, OUTPUT); pinMode(PIN_PPS, OUTPUT);
  digitalWrite(PIN_STEP, LOW); digitalWrite(PIN_DIR, HIGH); digitalWrite(PIN_EN, LOW); digitalWrite(PIN_PPS, LOW);
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}
  Serial.println("BNO085 calibration helper. Move the rig in figure-8 movements until mag shows 3/3.");
  help();
  lastTryMs = millis();            // pollImu() starts the IMU 1 s from now
}

void loop() {
  if (Serial.available()) handleSerial();
  pollImu();
  if (millis() - lastPrintMs >= PRINT_MS) {
    lastPrintMs = millis();
    if (imuOk) printLine();
  }
}
