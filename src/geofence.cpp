#include "geofence.h"

bool GEOFENCE_no_tx = true;
HeliumRegion GEOFENCE_region = HeliumRegion::None;

// Replace these with your closed coverage polygons as {latitude, longitude}
// pairs. Empty defaults deliberately prevent transmission everywhere.
static const float EUROPE_POLYGON[1] = {0};
static constexpr uint16_t EUROPE_CORNERS = 0;
static const float AMERICAS_POLYGON[1] = {0};
static constexpr uint16_t AMERICAS_CORNERS = 0;

bool pointInPolygonF(uint16_t corners, const float* polygon, float latitude, float longitude) {
  if (corners < 3 || polygon == nullptr) return false;
  bool inside = false;
  for (uint16_t i = 0, j = corners - 1; i < corners; j = i++) {
    const float latI = polygon[i * 2], lonI = polygon[i * 2 + 1];
    const float latJ = polygon[j * 2], lonJ = polygon[j * 2 + 1];
    if (((latI > latitude) != (latJ > latitude)) &&
        (longitude < (lonJ - lonI) * (latitude - latI) / (latJ - latI) + lonI)) inside = !inside;
  }
  return inside;
}

void GEOFENCE_position(float latitude, float longitude) {
  if (pointInPolygonF(EUROPE_CORNERS, EUROPE_POLYGON, latitude, longitude)) {
    GEOFENCE_region = HeliumRegion::Europe;
    GEOFENCE_no_tx = false;
  } else if (pointInPolygonF(AMERICAS_CORNERS, AMERICAS_POLYGON, latitude, longitude)) {
    GEOFENCE_region = HeliumRegion::Americas;
    GEOFENCE_no_tx = false;
  } else {
    GEOFENCE_region = HeliumRegion::None;
    GEOFENCE_no_tx = true;
  }
}
