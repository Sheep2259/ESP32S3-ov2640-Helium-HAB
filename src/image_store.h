#pragma once

#include <Arduino.h>
#include <esp_camera.h>
#include <helium_jpeg.h>

namespace image_store {

constexpr size_t kSlotCount = 128;
constexpr size_t kMaxJpegBytes = 120U * 1024U;

bool begin();
bool canAcceptCapture();
uint32_t lastCaptureUtc();
uint8_t imageCount();
uint16_t remainingPacketCount();

// Stores an already captured JPEG and its matching telemetry atomically.
esp_err_t save(const camera_fb_t& frame,
               helium_jpeg::HeliumTelemetry telemetry);

int oldestImage();
bool prepareEncoder(int slot, helium_jpeg::HeliumJPEG& encoder);

// Advances queue progress after a confirmed uplink. Returns true when the
// image was completed and removed.
bool packetSent(int slot);

}  // namespace image_store
