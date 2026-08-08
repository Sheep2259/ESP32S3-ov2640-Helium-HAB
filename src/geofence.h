#pragma once

#include <stdint.h>

enum class HeliumRegion : uint8_t { None, Europe, Americas };

extern bool GEOFENCE_no_tx;
extern HeliumRegion GEOFENCE_region;

// Coordinates are latitude/longitude. Transmit is permitted only inside a
// supplied Helium coverage polygon.
void GEOFENCE_position(float latitude, float longitude);
bool pointInPolygonF(uint16_t corners, const float* polygon, float latitude, float longitude);
