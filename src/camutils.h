#ifndef CAMUTILS_H
#define CAMUTILS_H

#include <Arduino.h>
#include <Preferences.h>
#include "esp_camera.h"
#include <helium_jpeg.h>

extern Preferences prefs;
constexpr size_t IMAGE_SLOT_COUNT = 128;
constexpr size_t MAX_JPEG_BYTES = 120U * 1024U;
constexpr size_t IMAGE_STORE_RESERVE_BYTES = 2048U;
extern uint16_t savedImages[IMAGE_SLOT_COUNT];
extern uint16_t imageIds[IMAGE_SLOT_COUNT];
extern uint16_t nextImageId;

// -----------------------------------------------------------------
// Error-detection helpers
// -----------------------------------------------------------------

/**
 * Validates a JPEG buffer in RAM.
 * Checks for SOI (FF D8) at byte 0 and EOI (FF D9) in the final 32 bytes.
 * Call this on the raw camera framebuffer before writing to flash.
 */
bool validateJpegBuffer(const uint8_t* buf, size_t len);

/**
 * Validates a JPEG file stored on LittleFS.
 * Checks for SOI at the start and EOI somewhere in the final 300 bytes
 * (safely past the appended ||META: trailer).
 * Returns true if the file looks like a complete, intact JPEG.
 */
bool validateJpegFile(const char* filename);

// -----------------------------------------------------------------
// Core camera functions
// -----------------------------------------------------------------
bool initialiseImageStore();
bool imageStoreCanAcceptCapture();
void persistImageProgress();
void discardImageSlot(int slot);
void completeImageSlot(int slot);
esp_err_t savePhoto(helium_jpeg::HeliumTelemetry telemetry);
int oldestStoredImage();
bool readImageTelemetry(uint16_t imageId, helium_jpeg::HeliumTelemetry& telemetry);
void imageFilename(uint16_t imageId, char* output, size_t outputSize);
void telemetryFilename(uint16_t imageId, char* output, size_t outputSize);
esp_err_t StartCamera();
camera_fb_t* captureJpeg();
void resetCamera();
void stopCamera();

#endif
