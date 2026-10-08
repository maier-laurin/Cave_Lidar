// Minimal BNO085 test: library only, no motor. Prints event counts once per second.
#include <Wire.h>
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno(6);          // RST on pin 6 (INT on 7 is not used)
sh2_SensorValue_t sv;
uint32_t nEv = 0, nRot = 0, nReset = 0, lastMs = 0;

void setReports() {
  Serial.println(bno.enableReport(SH2_ROTATION_VECTOR, 10000) ? "enable rot ok" : "enable rot FAILED");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  bool ok = bno.begin_I2C(0x4A, &Wire);
  Serial.println(ok ? "begin ok" : "begin FAILED");
  if (!ok) return;
  setReports();
}

void loop() {
  if (bno.wasReset()) { nReset++; Serial.println("wasReset"); setReports(); }
  while (bno.getSensorEvent(&sv)) {
    nEv++;
    if (sv.sensorId == SH2_ROTATION_VECTOR) {
      nRot++;
      if (nRot % 100 == 1)
        Serial.printf("rot w=%.3f x=%.3f y=%.3f z=%.3f\n", sv.un.rotationVector.real,
                      sv.un.rotationVector.i, sv.un.rotationVector.j, sv.un.rotationVector.k);
    }
  }
  if (millis() - lastMs >= 1000) {
    lastMs = millis();
    Serial.printf("events=%lu rot=%lu resets=%lu\n", nEv, nRot, nReset);
  }
}
