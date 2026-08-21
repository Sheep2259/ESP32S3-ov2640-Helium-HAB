#include "camera.h"

#include <Wire.h>
#include <pin_defs.h>

namespace camera {
namespace {
bool driverStarted = false;

constexpr uint8_t kOv2640Address = 0x30;
constexpr uint8_t kSi5351Address = 0x60;
constexpr uint8_t kOv2640BankSelect = 0xFF;
constexpr uint8_t kOv2640SensorBank = 0x01;
constexpr uint8_t kOv2640ProductIdRegister = 0x0A;
constexpr uint8_t kOv2640VersionRegister = 0x0B;
constexpr uint32_t kSccbFrequencyHz = 100000;

void startDiagnosticXclk(uint32_t frequencyHz) {
  ledcSetup(LEDC_CHANNEL_0, frequencyHz, 1);
  ledcAttachPin(CAM_XCLK, LEDC_CHANNEL_0);
  ledcWrite(LEDC_CHANNEL_0, 1);
}

void stopDiagnosticXclk() {
  ledcWrite(LEDC_CHANNEL_0, 0);
  ledcDetachPin(CAM_XCLK);
  pinMode(CAM_XCLK, INPUT);
}

bool sccbAddressAcknowledges(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission(true) == 0;
}

bool sccbWriteRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool sccbReadRegister(uint8_t address, uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, static_cast<uint8_t>(1),
                       static_cast<uint8_t>(true)) != 1) {
    return false;
  }
  value = Wire.read();
  return true;
}

void printLineState(const char* stage) {
  pinMode(CAM_SDA, INPUT);
  pinMode(CAM_SCL, INPUT);
  Serial.printf("[camera-diag] %s: SDA=%s SCL=%s%s\n", stage,
                digitalRead(CAM_SDA) ? "HIGH" : "LOW",
                digitalRead(CAM_SCL) ? "HIGH" : "LOW",
                (!digitalRead(CAM_SDA) || !digitalRead(CAM_SCL))
                    ? " (a low SCCB line may be stuck or missing its pull-up)"
                    : "");
}

void runSccbAttempt(uint32_t xclkFrequencyHz) {
  powerOff();
  delay(250);

  pinMode(CAM_LDO_EN, OUTPUT);
  digitalWrite(CAM_LDO_EN, HIGH);
  pinMode(CAM_PWDN, OUTPUT);
  digitalWrite(CAM_PWDN, HIGH);
  pinMode(CAM_RESET, OUTPUT);
  digitalWrite(CAM_RESET, LOW);
  delay(500);

  startDiagnosticXclk(xclkFrequencyHz);
  delay(20);
  digitalWrite(CAM_RESET, HIGH);
  delay(20);
  digitalWrite(CAM_PWDN, LOW);
  delay(500);

  Serial.printf("[camera-diag] Testing XCLK=%lu Hz.\n",
                static_cast<unsigned long>(xclkFrequencyHz));
  printLineState("before SCCB controller starts");

  if (!Wire.begin(CAM_SDA, CAM_SCL, kSccbFrequencyHz)) {
    Serial.println("[camera-diag] ERROR: ESP32 SCCB controller did not start.");
    stopDiagnosticXclk();
    powerOff();
    return;
  }
  Wire.setTimeOut(50);

  const bool cameraAck = sccbAddressAcknowledges(kOv2640Address);
  const bool clockGeneratorAck = sccbAddressAcknowledges(kSi5351Address);
  Serial.printf("[camera-diag] address 0x30 OV2640=%s; address 0x60 Si5351=%s.\n",
                cameraAck ? "ACK" : "NO_ACK",
                clockGeneratorAck ? "ACK" : "NO_ACK");

  if (cameraAck) {
    uint8_t productId = 0;
    uint8_t version = 0;
    const bool selectedSensorBank =
        sccbWriteRegister(kOv2640Address, kOv2640BankSelect,
                          kOv2640SensorBank);
    const bool productIdRead =
        selectedSensorBank &&
        sccbReadRegister(kOv2640Address, kOv2640ProductIdRegister, productId);
    const bool versionRead =
        selectedSensorBank &&
        sccbReadRegister(kOv2640Address, kOv2640VersionRegister, version);
    if (productIdRead && versionRead) {
      Serial.printf(
          "[camera-diag] sensor identity PID=0x%02X VER=0x%02X: %s.\n",
          productId, version,
          productId == 0x26 ? "OV2640 identity is valid"
                            : "unexpected sensor identity");
    } else {
      Serial.println(
          "[camera-diag] OV2640 ACKed, but its identity registers could not be read.");
    }
  } else {
    Serial.println(
        "[camera-diag] No camera ACK: check rails, ribbon, PWDN/RESET and XCLK path.");
  }

  Wire.end();
  stopDiagnosticXclk();
  powerOff();
  pinMode(CAM_RESET, INPUT);
  delay(250);
}
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

void runConnectionDiagnostics() {
  Serial.println("[camera-diag] Temporary camera connection test starting.");
  Serial.println(
      "[camera-diag] This test does not capture an image or transmit anything.");
  runSccbAttempt(20000000UL);
  runSccbAttempt(16000000UL);
  Serial.println(
      "[camera-diag] Test complete; camera powered off for normal scheduling.");
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
  // Allow automatic exposure and gain to converge after power-up.
  delay(1000);

  for (uint8_t i = 0; i < 3; ++i) {
    camera_fb_t* warmup = esp_camera_fb_get();
    if (warmup != nullptr) {
      esp_camera_fb_return(warmup);
    }
    delay(150);
  }

  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    camera_fb_t* frame = esp_camera_fb_get();
    if (frame != nullptr && frame->format == PIXFORMAT_JPEG) {
      return frame;
    }
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
