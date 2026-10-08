#include <Adafruit_BNO08x.h>

// --- PIN CONFIGURATION ---
// Sensor 1: Uses standard Wire (Pins 18 SDA, 19 SCL)
const int BNO1_INT = 15;
const int BNO1_RST = 14;

// Sensor 2: Uses Wire2 (Pins 25 SDA, 24 SCL)
const int BNO2_INT = 28;
const int BNO2_RST = 29;

// --- OBJECTS ---
Adafruit_BNO08x imu1(BNO1_RST);
Adafruit_BNO08x imu2(BNO2_RST);

sh2_SensorValue_t val1, val2;

// Interrupt Flags
volatile bool imu1Ready = false;
volatile bool imu2Ready = false;

// Counters for throttling output
int imu1Counter = 0;
int imu2Counter = 0;

// --- INTERRUPT SERVICE ROUTINES ---
void imu1ISR() { imu1Ready = true; }
void imu2ISR() { imu2Ready = true; }

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for Serial Monitor

  Serial.println("--- Dual BNO085 Wire/Wire2 Test ---");

  // 1. Initialize Sensor 1 on 'Wire'
  if (!imu1.begin_I2C(0x4A, &Wire)) {
    Serial.println("IMU #1 (Wire) not found at 0x4A!");
  } else {
    imu1.enableReport(SH2_ROTATION_VECTOR, 10000);      // 100Hz
    imu1.enableReport(SH2_LINEAR_ACCELERATION, 10000);  // 100Hz
    pinMode(BNO1_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BNO1_INT), imu1ISR, FALLING);
    Serial.println("IMU #1 Online.");
  }

  // 2. Initialize Sensor 2 on 'Wire2'
  Wire2.setSDA(25);
  Wire2.setSCL(24);
  if (!imu2.begin_I2C(0x4A, &Wire2)) {
    Serial.println("IMU #2 (Wire2) not found at 0x4A!");
  } else {
    imu2.enableReport(SH2_ROTATION_VECTOR, 10000);
    imu2.enableReport(SH2_LINEAR_ACCELERATION, 10000);
    pinMode(BNO2_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BNO2_INT), imu2ISR, FALLING);
    Serial.println("IMU #2 Online.");
  }
}

void loop() {
  // Process Sensor 1
  if (imu1Ready) {
    if (imu1.getSensorEvent(&val1)) {
      imu1Counter++;
      if (imu1Counter >= 100) {
        printData(1, val1);
        imu1Counter = 0;
      }
    }
    imu1Ready = false;
  }

  // Process Sensor 2
  if (imu2Ready) {
    if (imu2.getSensorEvent(&val2)) {
      imu2Counter++;
      if (imu2Counter >= 100) {
        printData(2, val2);
        imu2Counter = 0;
      }
    }
    imu2Ready = false;
  }
}

void printData(int sensorID, sh2_SensorValue_t &val) {
  Serial.print("IMU["); Serial.print(sensorID); Serial.print("] ");
  
  if (val.sensorId == SH2_ROTATION_VECTOR) {
    Serial.print("Quat(r,i,j,k): ");
    Serial.print(val.un.rotationVector.real, 3); Serial.print(", ");
    Serial.print(val.un.rotationVector.i, 3);    Serial.print(", ");
    Serial.print(val.un.rotationVector.j, 3);    Serial.print(", ");
    Serial.println(val.un.rotationVector.k, 3);
  } 
  else if (val.sensorId == SH2_LINEAR_ACCELERATION) {
    Serial.print("LinAccel(x,y,z): ");
    Serial.print(val.un.linearAcceleration.x, 3); Serial.print(", ");
    Serial.print(val.un.linearAcceleration.y, 3); Serial.print(", ");
    Serial.println(val.un.linearAcceleration.z, 3);
  }
}