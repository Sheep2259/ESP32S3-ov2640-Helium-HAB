/*
 * HeliumJPEG — JPEG File Parser
 *
 * Parses JPEG markers (SOI, SOF0, DQT, DHT, SOS) to extract image metadata,
 * quantization tables, and Huffman tables. Locates the scan data start offset.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once
#include <cstdint>
#include <cstddef>
#include "huffman.h"

// Forward declare File class (Arduino/LittleFS)
namespace fs { class File; }

namespace helium_jpeg {

/// Maximum number of JPEG components (Y, Cb, Cr).
static constexpr int MAX_COMPONENTS = 3;

/// Maximum quantization tables.
static constexpr int MAX_QT_TABLES = 4;

// ─────────────────────────────────────────────────────────────────────
// JPEGComponentInfo — Per-component metadata from SOF0
// ─────────────────────────────────────────────────────────────────────
struct JPEGComponentInfo {
    uint8_t id;           // Component identifier (1=Y, 2=Cb, 3=Cr typically)
    uint8_t h_sampling;   // Horizontal sampling factor (1–4)
    uint8_t v_sampling;   // Vertical sampling factor (1–4)
    uint8_t qt_id;        // Quantization table index
    uint8_t dc_table;     // DC Huffman table index (from SOS)
    uint8_t ac_table;     // AC Huffman table index (from SOS)
};

// ─────────────────────────────────────────────────────────────────────
// JPEGInfo — All metadata extracted from a JPEG file
// ─────────────────────────────────────────────────────────────────────
struct JPEGInfo {
    // Image dimensions
    uint16_t width;
    uint16_t height;

    // Component info
    uint8_t num_components;
    JPEGComponentInfo components[MAX_COMPONENTS];

    // Subsampling mode (derived from sampling factors)
    // 0 = 4:2:0, 1 = 4:2:2, 2 = 4:4:4
    uint8_t subsampling;

    // MCU grid dimensions
    uint16_t mcu_width;    // MCU width in pixels (8 for 4:4:4, 16 for 4:2:0/4:2:2)
    uint16_t mcu_height;   // MCU height in pixels (8 for 4:4:4/4:2:2, 16 for 4:2:0)
    uint16_t mcus_x;       // Number of MCU columns
    uint16_t mcus_y;       // Number of MCU rows
    uint16_t total_mcus;   // Total MCUs in the image

    // Blocks per MCU (depends on subsampling)
    // 4:2:0 → 6 blocks (4Y + 1Cb + 1Cr)
    // 4:2:2 → 4 blocks (2Y + 1Cb + 1Cr)
    // 4:4:4 → 3 blocks (1Y + 1Cb + 1Cr)
    uint8_t blocks_per_mcu;
    uint8_t y_blocks_per_mcu;  // Y blocks in MCU (4, 2, or 1)

    // Quantization tables (64 bytes each, zigzag order)
    uint8_t qt[MAX_QT_TABLES][64];
    uint8_t qt_count;   // Number of QT tables found

    // Huffman tables (built from DHT markers)
    // Index: 0 = DC table 0, 1 = DC table 1, 2 = AC table 0, 3 = AC table 1
    HuffmanTable huff_tables[4];
    bool has_custom_huffman;  // True if DHT markers were found in the file

    // Restart interval (from DRI marker), in MCUs. 0 = no restart markers.
    // When nonzero, the scan data contains a 2-byte RSTn marker (cycling
    // 0xFFD0..0xFFD7) after every `restart_interval` MCUs, and DC predictors
    // are reset to 0 at each one. Many hardware JPEG encoders (including the
    // OV2640's built-in encoder used on ESP32-CAM boards) enable this by
    // default for streaming error-resilience.
    uint16_t restart_interval;

    // Scan data position in the file
    uint32_t scan_data_offset;  // File offset of first byte after SOS header
    uint32_t scan_data_length;  // Length of scan data (up to EOI or EOF)

    // Parsing status
    bool valid;
    const char* error;
};

// ─────────────────────────────────────────────────────────────────────
// parseJPEG — Parse a JPEG file and populate JPEGInfo
// ─────────────────────────────────────────────────────────────────────
/// Parse JPEG markers from a File object.
/// The file should be opened for reading and positioned at the start.
/// On success, sets info.valid = true and populates all fields.
/// On failure, sets info.valid = false and info.error to a description.
///
/// This function reads the file sequentially and does NOT read the scan data
/// itself — it only records the scan data offset so the caller can read it.
bool parseJPEG(fs::File& file, JPEGInfo& info);

} // namespace helium_jpeg
