#include "camera.h"

#include <pin_defs.h>

namespace camera {
namespace {
bool driverStarted = false;
}

void powerOn() {
  pinMode(CAM_LDO_EN, OUTPUT);
  digitalWrite(CAM_LDO_EN, HIGH);
  pinMode(CAM_PWDN, OUTPUT);
  digitalWrite(CAM_PWDN, HIGH);
  delay(500);
  digitalWrite(CAM_PWDN, LOW);
  delay(500);
}

esp_err_t begin() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_D0;
  config.pin_d1 = CAM_D1;
  config.pin_d2 = CAM_D2;
  config.pin_d3 = CAM_D3;
  config.pin_d4 = CAM_D4;
  config.pin_d5 = CAM_D5;
  config.pin_d6 = CAM_D6;
  config.pin_d7 = CAM_D7;
  config.pin_xclk = CAM_XCLK;
  config.pin_pclk = CAM_PCLK;
  config.pin_vsync = CAM_VSYNC;
  config.pin_href = CAM_HREF;
  config.pin_sccb_sda = CAM_SDA;
  config.pin_sccb_scl = CAM_SCL;
  config.pin_pwdn = CAM_PWDN;
  config.pin_reset = CAM_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 8;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;
  const esp_err_t result = esp_camera_init(&config);
  driverStarted = result == ESP_OK;
  return result;
}

camera_fb_t* captureJpeg() {
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    camera_fb_t* frame = esp_camera_fb_get();
    if (frame != nullptr && frame->format == PIXFORMAT_JPEG) return frame;
    release(frame);
    delay(200);
  }
  return nullptr;
}

void release(camera_fb_t* frame) {
  if (frame != nullptr) esp_camera_fb_return(frame);
}

void powerOff() {
  if (driverStarted) {
    esp_camera_deinit();
    driverStarted = false;
  }
  pinMode(CAM_PWDN, OUTPUT);
  digitalWrite(CAM_PWDN, HIGH);
  pinMode(CAM_LDO_EN, OUTPUT);
  digitalWrite(CAM_LDO_EN, LOW);
}

bool validJpeg(const uint8_t* data, size_t length) {
  if (data == nullptr || length < 4 || data[0] != 0xFF || data[1] != 0xD8) {
    return false;
  }
  for (size_t i = length > 32 ? length - 32 : 0; i + 1 < length; ++i) {
    if (data[i] == 0xFF && data[i + 1] == 0xD9) return true;
  }
  return false;
}

}  // namespace camera
