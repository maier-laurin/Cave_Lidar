// Fake GPS for the Velodyne VLP-16: 1 Hz PPS pulse + $GPRMC time sentence.
// Simulated time starts at 2000-01-01 00:00:00 UTC when the Teensy boots.
//
// Wiring (GPS connector of the VLP-16 interface box, check the manual for its pin numbers):
//   Teensy pin 10        -> GPS PPS   (3.3 V, rising edge = top of the second)
//   Teensy pin 1 (TX1)   -> GPS NMEA / serial in (9600 baud, 8N1, TTL)
//   Teensy GND           -> GPS GND
//
// Per second: PPS rises at second N, then the RMC sentence for second N follows within ~100 ms.
#include <Arduino.h>

const int PIN_PPS = 10;
const uint32_t PPS_WIDTH_MS = 10;
const uint32_t NMEA_DELAY_MS = 100;     // sentence goes out this long after the PPS edge

IntervalTimer ppsTimer;
volatile uint32_t ppsCount = 0;          // number of PPS edges so far (= simulated seconds since start)
volatile uint32_t ppsEdgeUs = 0;
volatile bool ppsFlag = false;

void ppsISR() {
  digitalWriteFast(PIN_PPS, HIGH);
  ppsEdgeUs = micros();
  ppsCount++;
  ppsFlag = true;
}

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

const int32_t EPOCH_2000_DAYS = 10957;   // days from 1970-01-01 to 2000-01-01

// Build "$GPRMC,hhmmss.00,A,0000.0000,N,00000.0000,E,0.0,0.0,ddmmyy,,,A*XX" for simulated second n
void buildRmc(uint32_t n, char *out, size_t cap) {
  uint32_t days = n / 86400, sod = n % 86400;
  int y, m, d;
  civilFromDays(EPOCH_2000_DAYS + days, y, m, d);
  char body[100];
  snprintf(body, sizeof body, "GPRMC,%02lu%02lu%02lu.00,A,0000.0000,N,00000.0000,E,0.0,0.0,%02d%02d%02d,,,A",
           sod / 3600, (sod / 60) % 60, sod % 60, d, m, y % 100);
  uint8_t cs = 0;
  for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
  snprintf(out, cap, "$%s*%02X", body, cs);
}

void setup() {
  pinMode(PIN_PPS, OUTPUT);
  digitalWrite(PIN_PPS, LOW);
  Serial.begin(115200);
  Serial1.begin(9600);
  while (!Serial && millis() < 3000) {}
  Serial.println(F("GPS simulator: PPS on pin 10, NMEA on pin 1 (9600 baud), start = 2000-01-01 00:00:00"));
  // first edge is the top of second 1 (= 00:00:01), exactly 1 s from now
  ppsTimer.begin(ppsISR, 1000000);
}

void loop() {
  static uint32_t edgeMs = 0;
  static bool pulseHigh = false, sentenceDue = false;

  if (ppsFlag) {
    ppsFlag = false;
    edgeMs = millis();
    pulseHigh = true;
    sentenceDue = true;
  }
  if (pulseHigh && millis() - edgeMs >= PPS_WIDTH_MS) {
    digitalWriteFast(PIN_PPS, LOW);
    pulseHigh = false;
  }
  if (sentenceDue && millis() - edgeMs >= NMEA_DELAY_MS) {
    sentenceDue = false;
    char line[110];
    buildRmc(ppsCount, line, sizeof line);   // describes the pulse that just happened
    Serial1.println(line);                    // NMEA wants CR LF
    Serial.print(F("PPS #")); Serial.print(ppsCount); Serial.print(F("  ")); Serial.println(line);
  }
}
