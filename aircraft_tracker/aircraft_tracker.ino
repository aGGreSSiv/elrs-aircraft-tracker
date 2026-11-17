// Aircraft Tracker - Heltec WiFi LoRa 32 V4
//
// Receives the plane's GPS position from the radio's ExpressLRS TX Backpack over
// ESP-NOW, combines it with the board's own GPS and the GY-511 compass (optional),
// and shows on the OLED which way and how far the plane is from you.
//
// Build: Arduino ESP32 core 3.x, board "Heltec WiFi LoRa 32(V4)", USB CDC On Boot: Enabled.
//   arduino-cli compile -b "esp32:esp32:heltec_wifi_lora_32_V4:CDCOnBoot=cdc" aircraft_tracker
// Copy config.example.h to config.h and set your binding phrase first.
//
// Hold the PRG button (GPIO0) for 2 s: compass calibration (rotate the radio in all
//   directions for 20 s).
// Serial commands (115200): "SIM <lat> <lon>"  fake plane position for 15 s
//                           "RAW"              toggle raw compass log
//                           "CAL" / "CALINFO"  start calibration / show stored calibration

#include "config.h"
#include "geo.h"
#include "display.h"
#include "local_gps.h"
#include "compass.h"
#include "elrs_link.h"

#define LED_PIN      35
#define BUTTON_PIN   0
#define LINK_TIMEOUT 3000   // ms: older telemetry than this = no link

static bool rawLog = false;   // toggled with the "RAW" command
static unsigned long simUntil = 0;
static double simLat = 0, simLon = 0;
static double homeLat = 0, homeLon = 0;   // last known own position
static bool homeValid = false;

static void handleSerial() {
  static char buf[64];
  static size_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[len] = 0;
      if (!strncmp(buf, "SIM ", 4) && sscanf(buf + 4, "%lf %lf", &simLat, &simLon) == 2) {
        simUntil = millis() + 15000;
        Serial.printf("SIM plane at %.6f %.6f for 15 s\n", simLat, simLon);
      } else if (!strcmp(buf, "CAL")) {
        compass::startCalibration();
        Serial.println("Compass calibration started (serial)");
      } else if (!strcmp(buf, "CALINFO")) {
        compass::loadCalibration();
      } else if (!strcmp(buf, "RAW")) {
        rawLog = !rawLog;
        Serial.printf("RAW log %s\n", rawLog ? "on" : "off");
      }
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

static void handleButton() {
  static unsigned long pressedAt = 0;
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (!pressedAt) pressedAt = millis();
    if (millis() - pressedAt > 2000 && !compass::calibrating) {
      compass::startCalibration();
      Serial.println("Compass calibration started");
    }
  } else {
    pressedAt = 0;
  }
}

static void formatDistance(double m, char *out, size_t n) {
  if (m < 1000)       snprintf(out, n, "%dM", (int)(m + 0.5));
  else if (m < 10000) snprintf(out, n, "%.1fKM", m / 1000.0);
  else                snprintf(out, n, "%dKM", (int)(m / 1000.0 + 0.5));
}

static void draw() {
  using namespace display;
  clear();

  if (compass::calibrating) {
    char s[16];
    text(10, 4, "CALIBRATING", 1);
    text(10, 20, "ROTATE BOARD", 1);
    text(10, 30, "IN ALL", 1);
    text(10, 40, "DIRECTIONS", 1);
    snprintf(s, sizeof(s), "%d", compass::calibrationSecondsLeft());
    text(100, 20, s, 2);
    flush();
    return;
  }

  const LocalGps &me = local_gps::data;
  PlaneData p = elrs::snapshot();
  bool sim = millis() < simUntil;
  if (sim) { p.lat = simLat; p.lon = simLon; p.sats = 99; p.lastGps = p.lastFrame = millis(); }

  if (me.fix) { homeLat = me.lat; homeLon = me.lon; homeValid = true; }

  // Link: is any telemetry frame arriving?
  bool link = p.lastFrame && millis() - p.lastFrame < LINK_TIMEOUT;
  bool gpsFresh = p.lastGps && millis() - p.lastGps < LINK_TIMEOUT;
  if (!gpsFresh) p.sats = 0;
  bool planeFix = link && gpsFresh && p.sats > 0 && (p.lat != 0 || p.lon != 0);

  // Top right: H = Heltec, P = plane; "letter:satellites:state"
  const int tx = 58;
  char s[20];
  snprintf(s, sizeof(s), "H:%d:%s", me.sats, me.fix ? "LOCKED" : "NO FIX");
  text(tx, 2, s);
  if (!link)      snprintf(s, sizeof(s), "NO LINK");
  else if (sim)   snprintf(s, sizeof(s), "P:SIM");
  else            snprintf(s, sizeof(s), "P:%d:%s", p.sats, planeFix ? "LOCKED" : "NO FIX");
  text(tx, 14, s);

  // Left: direction indicator
  const int cx = 27, cy = 32, r = 26;
  circle(cx, cy, r);
  bool haveCompass = compass::present;
  float myHeading = haveCompass ? compass::heading : 0;   // no compass: north up

  // North marker (small N on the circle)
  float nAng = geo::wrap360(-myHeading) * M_PI / 180.0f;
  text(cx + (int)(sinf(nAng) * (r - 6)) - 2, cy - (int)(cosf(nAng) * (r - 6)) - 3, "N");

  if (planeFix && homeValid) {
    double brg = geo::bearing(homeLat, homeLon, p.lat, p.lon);
    double dist = geo::distance(homeLat, homeLon, p.lat, p.lon);
    arrow(cx, cy, r - 8, geo::wrap360(brg - myHeading));

    char d[16];
    formatDistance(dist, d, sizeof(d));
    text(128 - textWidth(d, 2), 46, d, 2);
  } else {
    text(cx - 2, cy - 3, "-");
  }

  if (!haveCompass) text(tx, 30, "N UP");
  else if (!compass::calibrated) text(tx, 30, "NO CAL");

  flush();
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n--- AIRCRAFT TRACKER ---");
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.printf("Display: %s\n", display::begin() ? "OK" : "FAIL");
  local_gps::begin();
  Serial.printf("Compass: %s\n", compass::begin() ? (compass::calibrated ? "OK" : "OK (not calibrated)") : "not found");
  bool ok = elrs::begin(ELRS_BINDING_PHRASE);
  Serial.printf("ESP-NOW: %s | MAC %02X:%02X:%02X:%02X:%02X:%02X\n", ok ? "OK" : "FAIL",
                elrs::uid[0], elrs::uid[1], elrs::uid[2], elrs::uid[3], elrs::uid[4], elrs::uid[5]);
}

void loop() {
  local_gps::poll();
  handleSerial();
  handleButton();

  static unsigned long lastCompass = 0;
  if (millis() - lastCompass >= 50) { lastCompass = millis(); compass::update(); }

  static unsigned long lastRaw = 0;
  if (rawLog && millis() - lastRaw >= 200) {
    lastRaw = millis();
    Serial.printf("RAW acc %6.0f %6.0f %6.0f | mag %6d %6d %6d | min %d %d %d max %d %d %d | cal=%d err=%u hdg=%.1f\n",
                  compass::lastAcc[0], compass::lastAcc[1], compass::lastAcc[2],
                  compass::lastMag[0], compass::lastMag[1], compass::lastMag[2],
                  compass::magMin[0], compass::magMin[1], compass::magMin[2],
                  compass::magMax[0], compass::magMax[1], compass::magMax[2],
                  compass::calibrated, compass::readErrors, compass::heading);
  }

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= 250) { lastDraw = millis(); draw(); }

  if (elrs::newGpsRaw) {
    elrs::newGpsRaw = false;
    Serial.print("CRSF GPS raw:");
    for (int i = 0; i < 19; i++) Serial.printf(" %02X", elrs::lastGpsRaw[i]);
    Serial.println();
  }

  static unsigned long lastLog = 0;
  if (millis() - lastLog >= 1000) {
    lastLog = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    PlaneData p = elrs::snapshot();
    const LocalGps &me = local_gps::data;
    Serial.printf("me: fix=%d sats=%d %.6f %.6f | plane: age=%ldms sats=%d %.6f %.6f alt=%.0f | "
                  "espnow pkts=%u other=%u msp ok/bad=%u/%u crsf bad=%u gps=%u last=0x%02X | hdg=%.0f\n",
                  me.fix, me.sats, me.lat, me.lon,
                  p.lastGps ? (long)(millis() - p.lastGps) : -1L, p.sats, p.lat, p.lon, p.altM,
                  elrs::rxPackets, elrs::rxOtherMac, elrs::mspOk, elrs::mspBad, elrs::crsfBad,
                  elrs::gpsFrames, elrs::lastCrsfType, compass::heading);
  }
}
