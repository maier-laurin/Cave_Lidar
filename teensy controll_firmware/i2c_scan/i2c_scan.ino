// I2C scanner for the BNO085 bring-up (Teensy, Wire: SDA = 18, SCL = 19).
// Prints the idle level of SDA/SCL, pulses the BNO RST pin, then scans 0x08..0x77 every 2 s.
// BNO085 should answer at 0x4A (0x4B if the address jumper is bridged).
#include <Wire.h>

const int PIN_SDA = 18;
const int PIN_SCL = 19;
const int PIN_BNO_RST = 6;     // set to -1 if RST is not wired
const int PIN_BNO_INT = 7;     // only read, never driven

void pulseReset() {
  if (PIN_BNO_RST < 0) return;
  pinMode(PIN_BNO_RST, OUTPUT);
  digitalWrite(PIN_BNO_RST, HIGH);
  delay(10);
  digitalWrite(PIN_BNO_RST, LOW);
  delay(20);
  digitalWrite(PIN_BNO_RST, HIGH);
  delay(500);                  // BNO085 needs a few hundred ms to boot
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  // before Wire takes over: both lines must idle HIGH (pull-ups on the breakout)
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  delay(10);
  Serial.printf("idle levels: SDA(%d)=%d SCL(%d)=%d  (both should be 1)\n",
                PIN_SDA, digitalRead(PIN_SDA), PIN_SCL, digitalRead(PIN_SCL));
  if (PIN_BNO_INT >= 0) {
    pinMode(PIN_BNO_INT, INPUT);
    Serial.printf("INT(%d)=%d\n", PIN_BNO_INT, digitalRead(PIN_BNO_INT));
  }

  pulseReset();
  Serial.printf("RST pulsed on pin %d\n", PIN_BNO_RST);
  Wire.begin();
  Wire.setClock(100000);       // slow and safe for the scan
}

void loop() {
  int found = 0;
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    Wire.beginTransmission(a);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("  device at 0x%02X%s\n", a, (a == 0x4A || a == 0x4B) ? "  <- BNO085" : "");
      found++;
    }
  }
  Serial.printf("scan done: %d device(s)  [SDA=%d SCL=%d INT=%d]\n", found,
                digitalRead(PIN_SDA), digitalRead(PIN_SCL), PIN_BNO_INT >= 0 ? digitalRead(PIN_BNO_INT) : -1);
  delay(2000);
}
