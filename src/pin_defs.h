#pragma once

// ESP32-S3-WROOM-1U-N16R2 custom HAB board.
// Keep hardware assignments here so application code never contains board pins.

// OV2640 DVP camera.
constexpr int CAM_D0 = 11;
constexpr int CAM_D1 = 9;
constexpr int CAM_D2 = 8;
constexpr int CAM_D3 = 10;
constexpr int CAM_D4 = 12;
constexpr int CAM_D5 = 18;
constexpr int CAM_D6 = 17;
constexpr int CAM_D7 = 16;
constexpr int CAM_XCLK = 15;
constexpr int CAM_PCLK = 13;
constexpr int CAM_VSYNC = 6;
constexpr int CAM_HREF = 7;
constexpr int CAM_PWDN = 14;
constexpr int CAM_RESET = 21;
constexpr int CAM_SDA = 4;
constexpr int CAM_SCL = 5;
constexpr int CAM_LDO_EN = 36;

// Camera SCCB and Si5351 share Arduino Wire/I2C0. Only access the Si5351 while
// the camera is idle; esp-camera borrows this controller during capture and
// must not install or remove a private SCCB controller on these pins.
constexpr int I2C_SDA = CAM_SDA;
constexpr int I2C_SCL = CAM_SCL;

// E22-900M22S / SX1262.
constexpr int LORA_MOSI = 38;
constexpr int LORA_MISO = 39;
constexpr int LORA_SCK = 40;
constexpr int LORA_NSS = 41;
constexpr int LORA_RST = 42;
constexpr int LORA_BUSY = 47;
constexpr int LORA_DIO1 = 48;
constexpr int LORA_RXEN = 1;
constexpr int LORA_TXEN = -1;  // SX1262 DIO2 controls TX enable on this PCB.

// ATGM336H: GPS TX -> ESP32 RX only.
constexpr int GPS_RX = 35;
constexpr uint32_t GPS_BAUD = 9600;
