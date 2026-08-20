#pragma once

#include <Arduino.h>
#include <esp_camera.h>

namespace camera {

// Powers the camera rails and takes the OV2640 out of power-down.
void powerOn();

// Starts the ESP32 camera driver after powerOn().
esp_err_t begin();

// Returns a JPEG framebuffer owned by the camera driver, or nullptr.
camera_fb_t* captureJpeg();

// Returns a framebuffer obtained from captureJpeg().
void release(camera_fb_t* frame);

// Stops the driver and powers down both camera rails.
void powerOff();

bool validJpeg(const uint8_t* data, size_t length);

}  // namespace camera
