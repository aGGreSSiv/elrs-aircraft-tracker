// ExpressLRS TX Backpack ESP-NOW telemetry receiver
//
// Protocol (ExpressLRS/Backpack master: Tx_main.cpp, build_flags.py, msp.cpp):
//  - UID = MD5('-DMY_BINDING_PHRASE="<phrase>"') first 6 bytes, bit 0 of uid[0] cleared.
//  - The Backpack sets its own STA MAC to the UID and unicasts telemetry to that MAC.
//    To receive it, this board's STA MAC must also be the UID.
//  - WiFi STA, channel 1, 11B/G/N + LR, no encryption.
//  - Packet: MSP v2 '$X<' flags fn(u16 LE) len(u16 LE) payload crc8(DVB-S2, 0xD5).
//    fn = 0x0011 (MSP_ELRS_BACKPACK_CRSF_TLM); payload = raw CRSF frame.
//  - CRSF GPS (0x02): lat,lon int32 (deg*1e7), speed u16 (km/h*10),
//    heading u16 (unused), alt u16 (m+1000), sats u8.
//    Big-endian as in the CRSF spec (verified against real frames from a T15 Backpack).
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <mbedtls/md5.h>

struct PlaneData {
  double lat = 0, lon = 0;
  float  speedKmh = 0;
  float  altM = 0;
  int    sats = 0;
  unsigned long lastGps = 0;      // last GPS frame (ms)
  unsigned long lastFrame = 0;    // any valid CRSF frame (ms)
};

namespace elrs {

static uint8_t uid[6];
static PlaneData plane;
static volatile uint32_t rxPackets = 0;    // all ESP-NOW packets from the UID
static volatile uint32_t rxOtherMac = 0;   // packets from other MACs
static volatile uint32_t mspOk = 0, mspBad = 0, crsfBad = 0, gpsFrames = 0;
static volatile uint8_t  lastCrsfType = 0;
static uint8_t lastGpsRaw[19];
static volatile bool newGpsRaw = false;

// Packets are handled in the ESP-NOW callback (WiFi task); this lock protects
// the GPS data shared with the main loop
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t crc8_d5(const uint8_t *d, size_t n) {
  uint8_t crc = 0;
  while (n--) {
    crc ^= *d++;
    for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

static inline int32_t be32(const uint8_t *p) { return (int32_t)((uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]); }
static inline uint16_t be16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }

static void computeUid(const char *phrase) {
  String define = String("-DMY_BINDING_PHRASE=\"") + phrase + "\"";
  uint8_t md5[16];
  mbedtls_md5((const unsigned char *)define.c_str(), define.length(), md5);
  memcpy(uid, md5, 6);
  uid[0] &= ~0x01;
}

static void handleCrsf(const uint8_t *f, size_t n) {
  // f: sync, len, type, data..., crc
  if (n < 4 || f[1] + 2 != n) { crsfBad++; return; }
  if (crc8_d5(f + 2, f[1] - 1) != f[n - 1]) { crsfBad++; return; }
  lastCrsfType = f[2];
  portENTER_CRITICAL(&mux);
  plane.lastFrame = millis();
  portEXIT_CRITICAL(&mux);
  if (f[2] == 0x02 && n == 19) {
    const uint8_t *p = f + 3;
    int32_t lat = be32(p), lon = be32(p + 4);
    portENTER_CRITICAL(&mux);
    plane.lat = lat / 1e7;
    plane.lon = lon / 1e7;
    plane.speedKmh = be16(p + 8) / 10.0f;
    plane.altM = (int)be16(p + 12) - 1000;
    plane.sats = p[14];
    plane.lastGps = millis();
    memcpy(lastGpsRaw, f, 19);
    newGpsRaw = true;
    portEXIT_CRITICAL(&mux);
    gpsFrames++;
  }
}

static void onRecv(const esp_now_recv_info_t *info, const uint8_t *d, int len) {
  if (memcmp(info->src_addr, uid, 6) != 0) { rxOtherMac++; return; }
  rxPackets++;
  // MSP v2: '$' 'X' '<' flags fn_lo fn_hi len_lo len_hi payload crc
  if (len < 9 || d[0] != '$' || d[1] != 'X' || (d[2] != '<' && d[2] != '>')) { mspBad++; return; }
  uint16_t fn = d[4] | d[5] << 8;
  uint16_t plen = d[6] | d[7] << 8;
  if (9 + plen != (unsigned)len || crc8_d5(d + 3, 5 + plen) != d[8 + plen]) { mspBad++; return; }
  mspOk++;
  if (fn == 0x0011) handleCrsf(d + 8, plen);
}

static bool begin(const char *phrase) {
  computeUid(phrase);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                                      WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if (esp_wifi_set_mac(WIFI_IF_STA, uid) != ESP_OK) return false;
  if (esp_now_init() != ESP_OK) return false;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, uid, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  return esp_now_register_recv_cb(onRecv) == ESP_OK;
}

// Thread-safe copy for the main loop
static PlaneData snapshot() {
  portENTER_CRITICAL(&mux);
  PlaneData p = plane;
  portEXIT_CRITICAL(&mux);
  return p;
}

}  // namespace elrs
