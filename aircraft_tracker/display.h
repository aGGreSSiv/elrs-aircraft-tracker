// Built-in OLED (SSD1306 128x64): library-free driver and drawing helpers
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define VEXT_PIN  36   // LOW = OLED power on
#define OLED_SDA  17
#define OLED_SCL  18
#define OLED_RST  21
#define OLED_ADDR 0x3C

namespace display {

static uint8_t fb[128 * 64 / 8];

// 5x7 font: ' ' - . / 0-9 : A-Z
static const uint8_t FONT[][5] = {
  {0x00,0x00,0x00,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00},
  {0x20,0x10,0x08,0x04,0x02},
  {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46},
  {0x21,0x41,0x45,0x4B,0x31}, {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
  {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03}, {0x36,0x49,0x49,0x49,0x36},
  {0x06,0x49,0x49,0x29,0x1E},
  {0x00,0x36,0x36,0x00,0x00},
  {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
  {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
  {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
  {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
  {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
  {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
  {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43},
};

static int glyph(char c) {
  if (c >= 'a' && c <= 'z') c -= 32;
  switch (c) { case '-': return 1; case '.': return 2; case '/': return 3; case ':': return 14; }
  if (c >= '0' && c <= '9') return 4 + (c - '0');
  if (c >= 'A' && c <= 'Z') return 15 + (c - 'A');
  return 0;
}

static uint8_t cmd(uint8_t c) {
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(0x00);
  Wire.write(c);
  return Wire.endTransmission();
}

static bool begin() {
  pinMode(VEXT_PIN, OUTPUT); digitalWrite(VEXT_PIN, LOW);
  delay(100);
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);  delay(20);
  digitalWrite(OLED_RST, HIGH); delay(20);
  Wire.begin(OLED_SDA, OLED_SCL, 400000);
  static const uint8_t init[] = {
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
    0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
  };
  uint8_t err = 0;
  for (uint8_t c : init) err |= cmd(c);
  return err == 0;
}

static void clear() { memset(fb, 0, sizeof(fb)); }

static void flush() {
  cmd(0x21); cmd(0); cmd(127);
  cmd(0x22); cmd(0); cmd(7);
  for (size_t i = 0; i < sizeof(fb); i += 16) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(0x40);
    Wire.write(fb + i, 16);
    Wire.endTransmission();
  }
}

static inline void pixel(int x, int y, bool on = true) {
  if (x < 0 || x >= 128 || y < 0 || y >= 64) return;
  uint8_t &b = fb[(y / 8) * 128 + x];
  if (on) b |= 1 << (y % 8); else b &= ~(1 << (y % 8));
}

static void line(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    pixel(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

static void circle(int cx, int cy, int r) {
  int x = r, y = 0, err = 1 - r;
  while (x >= y) {
    pixel(cx + x, cy + y); pixel(cx + y, cy + x); pixel(cx - y, cy + x); pixel(cx - x, cy + y);
    pixel(cx - x, cy - y); pixel(cx - y, cy - x); pixel(cx + y, cy - x); pixel(cx + x, cy - y);
    y++;
    if (err < 0) err += 2 * y + 1; else { x--; err += 2 * (y - x) + 1; }
  }
}

static void fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2) {
  int minY = min(y0, min(y1, y2)), maxY = max(y0, max(y1, y2));
  for (int y = minY; y <= maxY; y++) {
    int xs[3], n = 0;
    int px[3] = {x0, x1, x2}, py[3] = {y0, y1, y2};
    for (int i = 0; i < 3; i++) {
      int j = (i + 1) % 3;
      if ((py[i] <= y && py[j] > y) || (py[j] <= y && py[i] > y))
        xs[n++] = px[i] + (y - py[i]) * (px[j] - px[i]) / (py[j] - py[i]);
    }
    if (n == 2) { if (xs[0] > xs[1]) { int t = xs[0]; xs[0] = xs[1]; xs[1] = t; }
                  for (int x = xs[0]; x <= xs[1]; x++) pixel(x, y); }
  }
}

// Text; scale 1 = 6x8, scale 2 = 12x16 pixels per character
static void text(int x, int y, const char *s, int scale = 1) {
  for (; *s; s++, x += 6 * scale) {
    const uint8_t *g = FONT[glyph(*s)];
    for (int col = 0; col < 5; col++)
      for (int row = 0; row < 7; row++)
        if (g[col] & (1 << row))
          for (int sx = 0; sx < scale; sx++)
            for (int sy = 0; sy < scale; sy++)
              pixel(x + col * scale + sx, y + row * scale + sy);
  }
}

static int textWidth(const char *s, int scale = 1) { return strlen(s) * 6 * scale - scale; }

// Arrow from the center toward angleDeg (0 = up, clockwise)
static void arrow(int cx, int cy, int len, float angleDeg) {
  float a = angleDeg * M_PI / 180.0f;
  float dx = sinf(a), dy = -cosf(a);
  int tipX = cx + dx * len, tipY = cy + dy * len;
  int tailX = cx - dx * len * 0.6f, tailY = cy - dy * len * 0.6f;
  // shaft (3 px wide)
  for (int o = -1; o <= 1; o++)
    line(tailX + (int)(-dy * o), tailY + (int)(dx * o), cx + (int)(-dy * o), cy + (int)(dx * o));
  // head
  float bl = len * 0.55f, bw = len * 0.4f;
  int bx = cx + dx * (len - bl), by = cy + dy * (len - bl);
  fillTriangle(tipX, tipY, bx + (int)(-dy * bw), by + (int)(dx * bw),
               bx - (int)(-dy * bw), by - (int)(dx * bw));
}

// Check mark
static void check(int x, int y, int s) {
  for (int o = 0; o < 3; o++) {
    line(x, y + s / 2 + o, x + s / 3, y + s + o);
    line(x + s / 3, y + s + o, x + s, y + o);
  }
}

}  // namespace display
