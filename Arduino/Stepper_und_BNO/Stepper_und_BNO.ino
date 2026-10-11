#include <Adafruit_BNO08x.h>
#include <Adafruit_BNO08x.h>

//* CONFIGURATION

const int BNO_INT = 15; // BNO Interupt Pin
const int BNO_RST = 14; // BNO reset Pin
const int stepPin = 4; // tells the stepper motor to take an other step
const int dirPin = 3; // tells the stepper motor to change direction
const int rotsensPin = 2; // sensor pin of the light barrier sensor of the 1/2 weel

const long full_rotation_steps = 6400; //Number of stepper motor steps a full rotation takes
const int microstepDelay = 1000; //Delay between two steps of the stepper motor

Adafruit_BNO08x  bno08x(BNO_RST);
sh2_SensorValue_t sensorValue;

volatile bool dataAvailable = false; // signals the main loop when a reading is available, is volatile because it is set by interruped and otherwise the compiler might optimice it
bool rotationReady = false; // looks if we allready have a new rotation
bool accelReady = false; // ==//== translation

// interruped for the BNO 
void bnoISR() {
  dataAvailable = true;
}

// variables for the data buffer
float latest_rot_r, latest_rot_i, latest_rot_j, latest_rot_k; //BDO rotation
float latest_accel_x, latest_accel_y, latest_accel_z; //BDO acceleration

// variable for the stepper motor rotation counter 
long stepCounter = 0;

// interruped bassed on a hardware timer, to stepp the stepper motor independent of the main loop...
IntervalTimer stepperTimer; // a hardware timer object
void stepperISR() {
  digitalWriteFast(stepPin, HIGH);
  delayNanoseconds(500); // 500 ns is to little to use a seperate second timer here
  digitalWriteFast(stepPin, LOW);
  
  stepCounter++;
  if (stepCounter >= full_rotation_steps) { // later this should be done with the help of the sensor
    stepCounter = 0;
  }
}

void setup() {
  Serial.begin(115200);
  //while (!Serial && millis() < 3000); // Waiting for the serial Monitor, or 3 seconds

  // SCL SDA Connection for BDO
  Wire1.setSDA(17);
  Wire1.setSCL(16);
  // Check if the BDO is there
  if (!bno08x.begin_I2C(0x4A, &Wire1)) {
    Serial.println("Failed to find BNO085 on Wire1!");
    while (1) delay(10);
  }

  // Enable BNO reports at 50Hz
  bno08x.enableReport(SH2_ROTATION_VECTOR, 20000);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, 20000);

  // Setup BNO Interrupt Pin + Hardwareinterrupt
  pinMode(BNO_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BNO_INT), bnoISR, FALLING);

  // setup stepper motor relatet pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(rotsensPin, INPUT);
  digitalWrite(dirPin, HIGH); // Set direction clockwise

  //TODO drive motor to start position and reset the counter

  // start the stepper motor interrupet
  stepperTimer.begin(stepperISR, microstepDelay);
}

void loop() {
  if (dataAvailable) {
    dataAvailable = false; // we emidiatly reset the flag to not miss an interrupt if we are currently in the data processing logic

    // DRAIN THE BUFFER
    while (bno08x.getSensorEvent(&sensorValue)) { // we consume everything that is currently in the buffer, ideally this is one acceleration and one rotation if its just one of them we get the other one from the next or the previous interruped, if we allready have something from the previous interruped but something newer is here we take that
      switch (sensorValue.sensorId) {
        case SH2_ROTATION_VECTOR:
          latest_rot_r = sensorValue.un.rotationVector.real;
          latest_rot_i = sensorValue.un.rotationVector.i;
          latest_rot_j = sensorValue.un.rotationVector.j;
          latest_rot_k = sensorValue.un.rotationVector.k;
          rotationReady = true; // We have a fresh rotation
          break; // the case

        case SH2_LINEAR_ACCELERATION:
          latest_accel_x = sensorValue.un.linearAcceleration.x;
          latest_accel_y = sensorValue.un.linearAcceleration.y;
          latest_accel_z = sensorValue.un.linearAcceleration.z;
          accelReady = true;    // We have a fresh acceleration
          break; // the case
      }
    }

      if (rotationReady && accelReady) { // if both measurments are new we prepare the data to send to the pc and do all the other stuff
        unsigned long now = millis();

        noInterrupts(); //we need to read stepCounter safly
        long currentSteps = stepCounter;
        interrupts();
        float angle = (float)currentSteps / (float)full_rotation_steps * 6.283185307; // calculate the angle of the main axis in radiants we assume no stepslips and correct it all at once at the end of the rotation

        // SEND DATA
        sendSerialPackage(now, 
                          latest_rot_r, latest_rot_i, latest_rot_j, latest_rot_k, 
                          latest_accel_x, latest_accel_y, latest_accel_z,
                          angle);
      
      
        // RESET FLAGS: Now we wait for two brand-new readings
        rotationReady = false;
        accelReady = false;
    }
  }
}
 //PRETY PRINT SEND DATA
void sendSerialPackage(unsigned long timestamp, float r, float i, float j, float k, float x, float y, float z, float a) {
  // Format: Time, Quat_R, Quat_I, Quat_J, Quat_K, Accel_X, Accel_Y, Accel_Z
  Serial.print("T:");
  Serial.print(timestamp);
  Serial.print(",Rr:");
  Serial.print(r, 4); Serial.print(", Ri:");
  Serial.print(i, 4); Serial.print(", Rj:");
  Serial.print(j, 4); Serial.print(", Rk:");
  Serial.print(k, 4); Serial.print(", Tx:");
  Serial.print(x, 3); Serial.print(", Ty:");
  Serial.print(y, 3); Serial.print(", Tz:");
  Serial.print(z, 3); Serial.print(", ang:");
  Serial.println(a, 3);// println signals the end of the packet
}
/*
// Better fast version of sendserialData
void sendSerialPackage(unsigned long timestamp, float r, float i, float j, float k, float x, float y, float z) {
  // A "Header" helps your PC software find the start of the data (0xAA 0xBB)
  uint8_t header[2] = {0xAA, 0xBB};
  Serial.write(header, 2);
  
  // Write the raw bytes of the variables
  Serial.write((uint8_t*)&timestamp, sizeof(timestamp));
  Serial.write((uint8_t*)&r, sizeof(r));
  Serial.write((uint8_t*)&i, sizeof(i));
  Serial.write((uint8_t*)&j, sizeof(j));
  Serial.write((uint8_t*)&k, sizeof(k));
  Serial.write((uint8_t*)&x, sizeof(x));
  Serial.write((uint8_t*)&y, sizeof(y));
  Serial.write((uint8_t*)&z, sizeof(z));
}
*/
