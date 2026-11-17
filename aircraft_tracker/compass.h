// GY-511 (LSM303DLHC) tilt-compensated compass
// Optional: if the module is not found, the tracker runs without it (north up).
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

// GY-511 pins (separate I2C bus, Wire1).
// Default SDA/SCL from the Heltec V4 core variant. Check that these pins are on
// your board's header before wiring.
#ifndef COMPASS_SDA
#define COMPASS_SDA 3
#endif
#ifndef COMPASS_SCL
#define COMPASS_SCL 4
#endif

// Magnetic declination (degrees, east positive). GPS bearings are relative to true
// north, the compass to magnetic north. Look up the value for your location
// (e.g. NOAA declination calculator) and set it in config.h.
#ifndef MAG_DECLINATION_DEG
#define MAG_DECLINATION_DEG 0.0f
#endif

// The module's "right" axis. Mounted on top of the box with the pins at the back:
// X points right, Y away from you, Z up.
// Facing direction = up x right, so it stays horizontal however much the radio is
// tilted forward/back (as long as it isn't rolled sideways).
#ifndef COMPASS_RIGHT_X
#define COMPASS_RIGHT_X 1.0f
#define COMPASS_RIGHT_Y 0.0f
#define COMPASS_RIGHT_Z 0.0f
#endif

namespace compass {

static constexpr uint8_t ACC_ADDR = 0x19;
static constexpr uint8_t MAG_ADDR = 0x1E;

static bool present = false;
static bool calibrating = false;
static float heading = 0;                  // degrees, true north
static int16_t magMin[3] = {-500, -500, -500};
static int16_t magMax[3] = { 500,  500,  500};
static bool calibrated = false;
static unsigned long calStart = 0;
static Preferences prefs;
static float lastAcc[3] = {0, 0, 0};       // last raw values, for diagnostics
static int16_t lastMag[3] = {0, 0, 0};
static uint32_t readErrors = 0;

static bool wr(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(addr);
  Wire1.write(reg);
  Wire1.write(val);
  return Wire1.endTransmission() == 0;
}

static bool rd(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire1.beginTransmission(addr);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire1.read();
  return true;
}

static void loadCalibration() {
  bool opened = prefs.begin("compass", true);
  calibrated = opened && prefs.getBool("ok", false);
  Serial.printf("Compass NVS: open=%d ok=%d free=%u\n", opened, calibrated, (unsigned)prefs.freeEntries());
  if (!opened) return;
  if (calibrated) {
    prefs.getBytes("min", magMin, sizeof(magMin));
    prefs.getBytes("max", magMax, sizeof(magMax));
  }
  prefs.end();
}

static void saveCalibration() {
  bool opened = prefs.begin("compass", false);
  size_t a = prefs.putBytes("min", magMin, sizeof(magMin));
  size_t b = prefs.putBytes("max", magMax, sizeof(magMax));
  size_t c = prefs.putBool("ok", true);
  prefs.end();
  Serial.printf("Compass NVS save: open=%d min=%u max=%u ok=%u\n", opened, (unsigned)a, (unsigned)b, (unsigned)c);
  calibrated = true;
}

static bool begin() {
  Wire1.begin(COMPASS_SDA, COMPASS_SCL, 400000);
  present = wr(ACC_ADDR, 0x20, 0x57)    // CTRL_REG1_A: 100 Hz, XYZ enabled
         && wr(ACC_ADDR, 0x23, 0x08)    // CTRL_REG4_A: high resolution
         && wr(MAG_ADDR, 0x00, 0x14)    // CRA_REG_M: 30 Hz
         && wr(MAG_ADDR, 0x01, 0x20)    // CRB_REG_M: +-1.3 gauss
         && wr(MAG_ADDR, 0x02, 0x00);   // MR_REG_M: continuous conversion
  if (present) loadCalibration();
  return present;
}

// Start calibration: the user rotates the device in all directions for ~20 s
static void startCalibration() {
  if (!present) return;
  for (int i = 0; i < 3; i++) { magMin[i] = 32767; magMax[i] = -32768; }
  calibrating = true;
  calStart = millis();
}

static int calibrationSecondsLeft() {
  if (!calibrating) return 0;
  long left = 20000 - (long)(millis() - calStart);
  return left > 0 ? (left + 999) / 1000 : 0;
}

static void update() {
  if (!present) return;
  uint8_t b[6];
  if (!rd(ACC_ADDR, 0x28 | 0x80, b, 6)) { readErrors++; return; }
  float a[3];
  for (int i = 0; i < 3; i++) a[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8)) >> 4;

  if (!rd(MAG_ADDR, 0x03, b, 6)) { readErrors++; return; }
  // Register order: X, Z, Y (high byte first)
  int16_t raw[3] = {
    (int16_t)(b[0] << 8 | b[1]),
    (int16_t)(b[4] << 8 | b[5]),
    (int16_t)(b[2] << 8 | b[3]),
  };

  for (int i = 0; i < 3; i++) { lastAcc[i] = a[i]; lastMag[i] = raw[i]; }

  if (calibrating) {
    for (int i = 0; i < 3; i++) {
      if (raw[i] < magMin[i]) magMin[i] = raw[i];
      if (raw[i] > magMax[i]) magMax[i] = raw[i];
    }
    if (millis() - calStart >= 20000) { calibrating = false; saveCalibration(); }
  }

  float m[3];
  for (int i = 0; i < 3; i++) m[i] = raw[i] - (magMin[i] + magMax[i]) / 2.0f;
  m[2] *= 1100.0f / 980.0f;   // Z axis has a different gain

  // Tilt compensation: E = M x A, N = A x E
  float E[3] = { m[1] * a[2] - m[2] * a[1], m[2] * a[0] - m[0] * a[2], m[0] * a[1] - m[1] * a[0] };
  float N[3] = { a[1] * E[2] - a[2] * E[1], a[2] * E[0] - a[0] * E[2], a[0] * E[1] - a[1] * E[0] };
  // E and N differ hugely in magnitude (N ~ |a|*|E|); normalize before taking the angle
  float eLen = sqrtf(E[0] * E[0] + E[1] * E[1] + E[2] * E[2]);
  float nLen = sqrtf(N[0] * N[0] + N[1] * N[1] + N[2] * N[2]);
  if (eLen < 1e-3f || nLen < 1e-3f) return;
  for (int i = 0; i < 3; i++) { E[i] /= eLen; N[i] /= nLen; }
  // forward = up (a) x right
  const float r[3] = {COMPASS_RIGHT_X, COMPASS_RIGHT_Y, COMPASS_RIGHT_Z};
  const float f[3] = { a[1] * r[2] - a[2] * r[1], a[2] * r[0] - a[0] * r[2], a[0] * r[1] - a[1] * r[0] };
  float h = atan2f(E[0] * f[0] + E[1] * f[1] + E[2] * f[2],
                   N[0] * f[0] + N[1] * f[1] + N[2] * f[2]) * 180.0f / M_PI;
  h += MAG_DECLINATION_DEG;
  while (h < 0) h += 360.0f;
  while (h >= 360.0f) h -= 360.0f;

  // Simple smoothing (handles the 360/0 wrap)
  float d = h - heading;
  if (d > 180) d -= 360;
  if (d < -180) d += 360;
  heading += 0.25f * d;
  if (heading < 0) heading += 360;
  if (heading >= 360) heading -= 360;
}

}  // namespace compass
