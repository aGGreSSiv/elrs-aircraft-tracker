// GPS module on the Heltec V4 GNSS connector
// RX = GPIO39, 9600 baud (verified by measurement).
// Power/reset pins taken from Meshtastic's heltec_v4 variant; the module works with them.
#pragma once
#include <Arduino.h>

#define GPS_RX_PIN  39
#define GPS_TX_PIN  38
#define GPS_EN_PIN  34   // LOW = on
#define GPS_RST_PIN 42   // HIGH = running
#define GPS_STBY    40   // HIGH = awake
#define GPS_BAUD    9600

struct LocalGps {
  bool   fix = false;
  int    sats = 0;
  double lat = 0, lon = 0;
  float  alt = 0;            // m (MSL)
  float  speedKmh = 0;
  float  courseDeg = 0;      // course over ground (RMC)
  unsigned long lastSentence = 0;
  unsigned long lastFix = 0;
};

namespace local_gps {

static LocalGps data;

static bool checksumOk(const char *s) {
  if (*s != '$') return false;
  uint8_t cs = 0;
  const char *p = s + 1;
  while (*p && *p != '*') cs ^= (uint8_t)*p++;
  return *p == '*' && (uint8_t)strtol(p + 1, nullptr, 16) == cs;
}

static int split(char *s, char *f[], int max) {
  int n = 0;
  if (char *st = strchr(s, '*')) *st = 0;
  f[n++] = s;
  for (char *p = s; *p && n < max; p++)
    if (*p == ',') { *p = 0; f[n++] = p + 1; }
  return n;
}

static double nmeaDeg(const char *v, char hemi) {
  if (!*v) return 0;
  double x = atof(v);
  int d = (int)(x / 100);
  double r = d + (x - d * 100) / 60.0;
  return (hemi == 'S' || hemi == 'W') ? -r : r;
}

static void handle(char *line) {
  if (!checksumOk(line)) return;
  data.lastSentence = millis();
  char *f[24];
  int n = split(line + 1, f, 24);
  const char *t = f[0] + 2;
  if (!strcmp(t, "GGA") && n >= 10) {
    data.sats = atoi(f[7]);
    if (atoi(f[6]) > 0 && *f[2] && *f[4]) {
      data.lat = nmeaDeg(f[2], *f[3]);
      data.lon = nmeaDeg(f[4], *f[5]);
      data.alt = atof(f[9]);
      data.lastFix = millis();
    }
  } else if (!strcmp(t, "RMC") && n >= 9) {
    data.fix = (*f[2] == 'A');
    data.speedKmh = atof(f[7]) * 1.852f;
    if (*f[8]) data.courseDeg = atof(f[8]);
  }
}

static void begin() {
  pinMode(GPS_EN_PIN, OUTPUT);  digitalWrite(GPS_EN_PIN, LOW);
  pinMode(GPS_STBY, OUTPUT);    digitalWrite(GPS_STBY, HIGH);
  pinMode(GPS_RST_PIN, OUTPUT);
  digitalWrite(GPS_RST_PIN, LOW); delay(50);
  digitalWrite(GPS_RST_PIN, HIGH);
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

static void poll() {
  static char buf[100];
  static size_t len = 0;
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '$') len = 0;
    if (c == '\r' || c == '\n') {
      if (len > 6) { buf[len] = 0; handle(buf); }
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
  // no new fix within 3 s = fix lost
  if (data.fix && millis() - data.lastFix > 3000) data.fix = false;
}

}  // namespace local_gps
