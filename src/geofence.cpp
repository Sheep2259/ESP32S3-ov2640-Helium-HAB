#include "geofence.h"

bool GEOFENCE_no_tx = true;
HeliumRegion GEOFENCE_region = HeliumRegion::None;

// Closed regions expressed as {latitude, longitude} pairs.  The source
// coordinates supplied for the project were {longitude, latitude}; they are
// deliberately transposed here to match pointInPolygonF().  The repeated last
// vertex from each source polygon is omitted because the algorithm closes the
// polygon itself.
static constexpr float EUROPE_POLYGON[] = {
    29.39595f,  24.94292f,
    15.71483f,  60.45911f,
    45.97828f,  56.06458f,
    73.00243f,  27.67591f,
    67.07479f, -30.30397f,
    35.31410f, -12.65736f,
    30.83938f,  -5.31849f,
    33.07622f,   8.74401f,
};
static constexpr uint16_t EUROPE_CORNERS = 8;

static constexpr float AMERICAS_POLYGON[] = {
    45.90022f,  -49.48578f,
    29.92957f,  -66.00922f,
    15.21355f,  -57.39593f,
    -3.06004f,  -82.00531f,
    12.99745f, -120.14984f,
    10.06985f, -167.87109f,
    29.16498f, -173.14453f,
    28.08493f, -127.79297f,
    59.67515f, -172.79297f,
    70.39652f, -162.77344f,
};
static constexpr uint16_t AMERICAS_CORNERS = 10;

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

void GEOFENCE_inhibit() {
  GEOFENCE_region = HeliumRegion::None;
  GEOFENCE_no_tx = true;
}
