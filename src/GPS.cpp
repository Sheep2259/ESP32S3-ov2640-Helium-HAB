#include <Arduino.h>
#include <pin_defs.h>
#include <TinyGPSPlus.h>


TinyGPSPlus gps;



void UpdateGPSInfo(
  float &lat, float &lng, float &age_s,                 // location + fix age (s)
  uint16_t &year, uint8_t &month, uint8_t &day,         // date (UTC)
  uint8_t &hour, uint8_t &minute, uint8_t &second, uint8_t &centisecond, // time (UTC)
  float &alt_m,                                         // altitude (m)
  float &speed_kmh,                                     // speed (km/h)
  float &course_deg,                                    // course (deg)
  uint8_t &sats,                                        // satellite count
  float &hdop                                           // HDOP
) {


  // --- Location ---
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lng = gps.location.lng();

    age_s = gps.location.age() / 1000.0f;
  }

  // --- Date / Time (UTC) ---
  if (gps.date.isValid()) {
    year = gps.date.year();
    month = gps.date.month();
    day = gps.date.day();
  }

  if (gps.time.isValid()) {
    hour = gps.time.hour();
    minute = gps.time.minute();
    second = gps.time.second();
    centisecond = gps.time.centisecond();
  }

  // --- Altitude ---
  if (gps.altitude.isValid()) {
    alt_m = gps.altitude.meters();
  }

  // --- Speed ---
  if (gps.speed.isValid()) {
    speed_kmh = gps.speed.kmph();
  }

  // --- Course Heading---
  if (gps.course.isValid()) {
    course_deg = gps.course.deg();
  }

  // --- Satellites ---
  if (gps.satellites.isValid()) {
    sats = (uint8_t)gps.satellites.value();
  }

  // --- HDOP ---
  if (gps.hdop.isValid()) {
    hdop = gps.hdop.hdop();
  }
}

bool GPSPositionFresh(unsigned long maximumAgeMs) {
  return gps.location.isValid() && gps.hdop.isValid() &&
         gps.location.age() <= maximumAgeMs && gps.hdop.age() <= maximumAgeMs &&
         gps.hdop.hdop() < 20.0;
}

bool GPSTimeFresh(unsigned long maximumAgeMs) {
  return gps.time.isValid() && gps.date.isValid() &&
         gps.time.age() <= maximumAgeMs && gps.date.age() <= maximumAgeMs;
}


  /*
  // --- Library diagnostics (optional, good for debugging) ---
  Serial.print(F("  Stats: chars "));
  Serial.print(gps.charsProcessed());
  Serial.print(F("  sentencesWithFix "));
  Serial.print(gps.sentencesWithFix());
  Serial.print(F("  failedCS "));
  Serial.print(gps.failedChecksum());

  Serial.println();
  */





