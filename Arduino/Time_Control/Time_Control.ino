#include <Adafruit_BNO08x.h>

// --- PIN CONFIGURATION ---
// Sensor 1: Wire1 (Pins 17 SDA, 16 SCL)
const int BNO1_INT = 15;
const int BNO1_RST = 14;

// Sensor 2: Wire2 (Pins 25 SDA, 24 SCL)
const int BNO2_INT = 28;
const int BNO2_RST = 29;

// --- OBJECTS ---
Adafruit_BNO08x imu1(BNO1_RST);
Adafruit_BNO08x imu2(BNO2_RST);

sh2_SensorValue_t val1, val2;

volatile bool imu1Ready = false;
volatile bool imu2Ready = false;

int imu1Counter = 0;
int imu2Counter = 0;

void imu1ISR() { imu1Ready = true; }
void imu2ISR() { imu2Ready = true; }

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("--- Dual BNO085: Wire1 & Wire2 ---");

  // 1. Initialize Sensor 1 on WIRE1 (Pins 16/17)
  Wire1.setSDA(17);
  Wire1.setSCL(16);
  if (!imu1.begin_I2C(0x4A, &Wire1)) {
    Serial.println("IMU #1 (Wire1) not found!");
  } else {
    imu1.enableReport(SH2_ROTATION_VECTOR, 10000);
    imu1.enableReport(SH2_LINEAR_ACCELERATION, 10000);
    pinMode(BNO1_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BNO1_INT), imu1ISR, FALLING);
    Serial.println("IMU #1 Online on Wire1.");
  }

  // 2. Initialize Sensor 2 on WIRE2 (Pins 24/25)
  Wire2.setSDA(25);
  Wire2.setSCL(24);
  if (!imu2.begin_I2C(0x4A, &Wire2)) {
    Serial.println("IMU #2 (Wire2) not found!");
  } else {
    imu2.enableReport(SH2_ROTATION_VECTOR, 10000);
    imu2.enableReport(SH2_LINEAR_ACCELERATION, 10000);
    pinMode(BNO2_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BNO2_INT), imu2ISR, FALLING);
    Serial.println("IMU #2 Online on Wire2.");
  }
}

void loop() {
  if (imu1Ready) {
    if (imu1.getSensorEvent(&val1)) {
      if (++imu1Counter >= 100) {
        printData(1, val1);
        imu1Counter = 0;
      }
    }
    imu1Ready = false;
  }

  if (imu2Ready) {
    if (imu2.getSensorEvent(&val2)) {
      if (++imu2Counter >= 100) {
        printData(2, val2);
        imu2Counter = 0;
      }
    }
    imu2Ready = false;
  }
}

void printData(int id, sh2_SensorValue_t &val) {
  Serial.print("IMU["); Serial.print(id); Serial.print("] ");
  if (val.sensorId == SH2_ROTATION_VECTOR) {
    Serial.print("Quat: "); Serial.println(val.un.rotationVector.real, 3);
  } else if (val.sensorId == SH2_LINEAR_ACCELERATION) {
    Serial.print("Accel: "); Serial.println(val.un.linearAcceleration.x, 3);
  }
}