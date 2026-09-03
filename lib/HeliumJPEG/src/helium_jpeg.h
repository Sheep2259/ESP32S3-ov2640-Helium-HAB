/*
 * HeliumJPEG — JPEG-over-LoRaWAN Packetizer
 *
 * Splits OV2640 JPEG files into LoRaWAN-friendly packets of up to 210 bytes with
 * per-packet independent decodability for Helium IoT uplink.
 *
 * Hardware: ESP32-S3-WROOM-1U-N16R2
 * Environment: PlatformIO / Arduino Core for ESP32
 * Input: JPEG files on LittleFS (OV2640, SVGA–SXGA, Quality 0–15)
 * Uplink: Helium IoT Network (LoRaWAN US915 DR4 / EU868 DR5, ~222B max)
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once
#include <cstdint>
#include <cstddef>
#include <FS.h>

namespace helium_jpeg {

// ─────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────

/// Maximum packet size in bytes.
static constexpr int PACKET_SIZE = 210;

/// Metadata and data header sizes in bytes.
static constexpr int METADATA_HEADER_SIZE = 8;
static constexpr int DATA_HEADER_SIZE = 7;

/// Metadata payload size in bytes.
static constexpr int PAYLOAD_SIZE = PACKET_SIZE - METADATA_HEADER_SIZE; // 202

/// Usable scan data bytes per data packet.
static constexpr int SCAN_DATA_PER_PACKET = PACKET_SIZE - DATA_HEADER_SIZE; // 203

// ─────────────────────────────────────────────────────────────────────
// Subsampling Mode
// ─────────────────────────────────────────────────────────────────────
enum Subsampling : uint8_t {
    SUBSAMP_420 = 0,    // 4:2:0 (OV2640 default)
    SUBSAMP_422 = 1,    // 4:2:2
    SUBSAMP_444 = 2,    // 4:4:4
};

// ─────────────────────────────────────────────────────────────────────
// HeliumPacket — Packet ready for LoRaWAN uplink
// ─────────────────────────────────────────────────────────────────────
struct HeliumPacket {
    // Data packets have a 7-byte header and scan data begins at byte 7.
    // Metadata is identified by packet ID zero and has an eighth header byte
    // containing its subsampling mode. Metadata remains fixed at 210 bytes.
    uint8_t data[PACKET_SIZE];
    size_t length = 0;
};

// ─────────────────────────────────────────────────────────────────────
// HeliumTelemetry — Sensor data embedded in metadata packet
// ─────────────────────────────────────────────────────────────────────
struct HeliumTelemetry {
    // GPS
    int32_t  latitude;       ///< Microdegrees (±90,000,000 ≈ 0.11m resolution)
    int32_t  longitude;      ///< Microdegrees
    uint16_t altitude;       ///< Meters MSL (0–65,535m)
    uint16_t speed;          ///< cm/s ground speed
    uint16_t heading;        ///< Centidegrees (0–35,999 → 0.00–359.99°)
    int16_t  v_speed;        ///< cm/s vertical (positive = ascending)
    uint8_t  hdop;           ///< ×10 (0.0–25.5)
    uint8_t  satellites;     ///< Number of tracked satellites
    uint8_t  fix_type;       ///< 0=none, 2=2D, 3=3D

    // Timing
    uint32_t timestamp;      ///< Unix epoch (seconds)
    uint32_t uptime;         ///< ESP uptime (seconds)

    // Power
    uint16_t battery_mv;     ///< Battery voltage in millivolts

    // Environment
    int8_t   esp_temp;       ///< ESP32 die temperature (°C)
    int16_t  ext_temp;       ///< External sensor (centi-°C, -327.68 to +327.67°C)
    uint16_t pressure;       ///< deci-hPa (0–6553.5 hPa)
    uint8_t  humidity;       ///< 0–100% RH

    // Image info
    uint32_t jpeg_file_size; ///< Original JPEG file size in bytes
    uint16_t capture_ms;     ///< Camera capture duration (ms)

    // Diagnostics
    uint16_t free_heap_kb;   ///< Free heap memory (KB)
    uint16_t status_flags;   ///< Status bitfield (see STATUS_FLAG_* constants)
};

// Status flag assignments (set bits are latched until the next boot).
static constexpr uint16_t STATUS_FLAG_GPS_INVALID      = (1 << 0);
static constexpr uint16_t STATUS_FLAG_CAMERA_ERROR      = (1 << 1);
static constexpr uint16_t STATUS_FLAG_LOW_BATTERY_RESERVED = (1 << 2);
static constexpr uint16_t STATUS_FLAG_IMAGE_SAVE_FAILED = (1 << 3);
static constexpr uint16_t STATUS_FLAG_FS_ERROR          = (1 << 4);
static constexpr uint16_t STATUS_FLAG_ENCODE_FAILED     = (1 << 5);
static constexpr uint16_t STATUS_FLAG_ABNORMAL_RESET     = (1 << 6);
static constexpr uint16_t STATUS_FLAG_PSRAM_FAULT       = (1 << 7);
static constexpr uint16_t STATUS_FLAG_SENSOR_BUS_RESERVED  = (1 << 8);
static constexpr uint16_t STATUS_FLAG_LORA_TX_FAILURE   = (1 << 9);
static constexpr uint16_t STATUS_FLAG_WSPR_TX_FAILURE   = (1 << 10);

// ─────────────────────────────────────────────────────────────────────
// MCUBoundary — Internal: recorded position of each MCU in the scan
// ─────────────────────────────────────────────────────────────────────
struct MCUBoundary {
    uint32_t byte_offset;    ///< Byte offset from scan data start
    uint8_t  bit_offset;     ///< Bit offset within byte (0–7, MSB-first)
    int16_t  dc_y;           ///< Absolute DC predictor for Y at this MCU
    int16_t  dc_cb;          ///< Absolute DC predictor for Cb
    int16_t  dc_cr;          ///< Absolute DC predictor for Cr
    bool     is_restart_point; ///< True if a JPEG restart marker was consumed
                                ///< immediately before this MCU (DC predictors
                                ///< were reset to 0 here in the original stream).
                                ///< Packets must never span across such a point --
                                ///< it must always be the START of a packet, so the
                                ///< restart marker itself is stripped from the wire
                                ///< format entirely (the packet-boundary DC reset
                                ///< that HeliumJPEG already does subsumes it).
};

// ─────────────────────────────────────────────────────────────────────
// PacketPlan — Internal: describes one data packet's contents
// ─────────────────────────────────────────────────────────────────────
struct PacketPlan {
    uint16_t first_mcu;      ///< Index of first MCU in this packet
    uint16_t mcu_count;      ///< Number of MCUs in this packet
    uint32_t scan_byte_start; ///< Byte offset of first MCU in scan data
    uint8_t  scan_bit_start;  ///< Bit offset of first MCU
    uint32_t scan_byte_end;   ///< Byte offset just past last MCU
    uint8_t  scan_bit_end;    ///< Bit offset just past last MCU
};

// ─────────────────────────────────────────────────────────────────────
// HeliumJPEG — Main encoder class
// ─────────────────────────────────────────────────────────────────────
class HeliumJPEG {
public:
    HeliumJPEG();
    ~HeliumJPEG();

    /// Parse a JPEG file and prepare all packets.
    /// @param jpegFile   Opened LittleFS File object (read mode)
    /// @param imageId    16-bit rolling image counter
    /// @param metaRepeat Number of times to send metadata packet (default 3)
    /// @return Total packet count (metaRepeat + data packets), or -1 on error
    int begin(fs::File& jpegFile, uint16_t imageId, uint8_t metaRepeat = 3);

    /// Attach telemetry data to be embedded in metadata packets.
    /// Call after begin() but before iterating with getNextPacket().
    void setTelemetry(const HeliumTelemetry& telem);

    /// Get the next packet in sequence.
    /// Returns false when all packets have been yielded. On success, pkt.length
    /// is the exact number of bytes to transmit from pkt.data.
    /// Metadata packets are distributed across the image packet stream.
    bool getNextPacket(HeliumPacket& pkt);

    /// Skip already-sent packets when restoring progress after a power loss.
    bool skipPackets(uint16_t count);

    /// Get total packet count (metadata × repeat + data packets).
    int getPacketCount() const;

    /// Get data-only packet count (excludes metadata repeats).
    int getDataPacketCount() const;

    /// Get image dimensions.
    uint16_t getImageWidth() const;
    uint16_t getImageHeight() const;

    /// Get MCU grid dimensions.
    uint16_t getMCUsX() const;
    uint16_t getMCUsY() const;

    /// Reset packet iterator to the beginning.
    void reset();

    /// Get last error message (nullptr if no error).
    const char* getError() const;

private:
    // ── Internal methods ──
    bool walkScanData(fs::File& file);
    bool buildPacketPlans();
    void buildMetadataPayload();
    bool buildDataPacket(HeliumPacket& pkt, int planIndex, fs::File& file);
    bool reEncodeDC(uint8_t* scan_buf, size_t scan_len,
                    uint8_t src_bit_offset,
                    uint16_t mcu_count,
                    int16_t abs_dc_y, int16_t abs_dc_cb, int16_t abs_dc_cr,
                    uint8_t* out_buf, size_t out_capacity,
                    size_t& out_bytes_written);

    // ── State ──
    struct Impl;
    Impl* impl_;
};

} // namespace helium_jpeg
