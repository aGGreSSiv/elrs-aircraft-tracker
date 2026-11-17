// Distance and bearing (spherical Earth approximation)
#pragma once
#include <math.h>

namespace geo {

static constexpr double EARTH_R = 6371000.0;   // m
static inline double rad(double d) { return d * M_PI / 180.0; }
static inline double deg(double r) { return r * 180.0 / M_PI; }

// Great-circle distance between two points (m), haversine
static inline double distance(double lat1, double lon1, double lat2, double lon2) {
  double dLat = rad(lat2 - lat1), dLon = rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(rad(lat1)) * cos(rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  return 2 * EARTH_R * atan2(sqrt(a), sqrt(1 - a));
}

// Initial bearing 1 -> 2, degrees clockwise from north, 0..360
static inline double bearing(double lat1, double lon1, double lat2, double lon2) {
  double y = sin(rad(lon2 - lon1)) * cos(rad(lat2));
  double x = cos(rad(lat1)) * sin(rad(lat2)) -
             sin(rad(lat1)) * cos(rad(lat2)) * cos(rad(lon2 - lon1));
  double b = deg(atan2(y, x));
  return b < 0 ? b + 360.0 : b;
}

// Wrap an angle into 0..360
static inline float wrap360(float a) {
  while (a < 0) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

}  // namespace geo
