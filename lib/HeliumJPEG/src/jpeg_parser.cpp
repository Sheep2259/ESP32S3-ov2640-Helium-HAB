/*
 * HeliumJPEG — JPEG File Parser Implementation
 *
 * SPDX-License-Identifier: MIT
 */

#include "jpeg_parser.h"
#include <FS.h>   // Arduino FS — provides fs::File

namespace helium_jpeg {

// ─────────────────────────────────────────────────────────────────────
// JPEG Marker Codes
// ─────────────────────────────────────────────────────────────────────
static constexpr uint8_t JPEG_SOI  = 0xD8;
static constexpr uint8_t JPEG_EOI  = 0xD9;
static constexpr uint8_t JPEG_SOF0 = 0xC0;   // Baseline DCT
static constexpr uint8_t JPEG_SOF2 = 0xC2;   // Progressive DCT
static constexpr uint8_t JPEG_DHT  = 0xC4;
static constexpr uint8_t JPEG_DQT  = 0xDB;
static constexpr uint8_t JPEG_SOS  = 0xDA;
static constexpr uint8_t JPEG_DRI  = 0xDD;
static constexpr uint8_t JPEG_RST0 = 0xD0;
static constexpr uint8_t JPEG_RST7 = 0xD7;
static constexpr uint8_t JPEG_APP0 = 0xE0;
static constexpr uint8_t JPEG_COM  = 0xFE;

// ─────────────────────────────────────────────────────────────────────
// Helper: Read 1 byte from file, return -1 on EOF
// ─────────────────────────────────────────────────────────────────────
static int readByte(fs::File& file) {
    uint8_t b;
    if (file.read(&b, 1) != 1) return -1;
    return b;
}

// ─────────────────────────────────────────────────────────────────────
// Helper: Read 2-byte big-endian uint16
// ─────────────────────────────────────────────────────────────────────
static int readU16(fs::File& file) {
    uint8_t buf[2];
    if (file.read(buf, 2) != 2) return -1;
    return (buf[0] << 8) | buf[1];
}

// ─────────────────────────────────────────────────────────────────────
// Parse SOF0 (Start Of Frame — Baseline DCT)
// ─────────────────────────────────────────────────────────────────────
static bool parseSOF(fs::File& file, JPEGInfo& info, int length) {
    // SOF0 structure: precision(1) + height(2) + width(2) + num_components(1)
    //                 + per component: id(1) + sampling(1) + qt_id(1)
    int precision = readByte(file);
    if (precision < 0) return false;
    if (precision != 8) {
        info.error = "Unsupported bit depth (only 8-bit supported)";
        return false;
    }

    int h = readU16(file);
    int w = readU16(file);
    if (h < 0 || w < 0) return false;
    info.height = (uint16_t)h;
    info.width  = (uint16_t)w;

    int nc = readByte(file);
    if (nc < 0 || nc > MAX_COMPONENTS) {
        info.error = "Unsupported component count";
        return false;
    }
    info.num_components = (uint8_t)nc;

    uint8_t max_h = 1, max_v = 1;
    for (int i = 0; i < nc; i++) {
        int id = readByte(file);
        int sampling = readByte(file);
        int qt = readByte(file);
        if (id < 0 || sampling < 0 || qt < 0) return false;

        info.components[i].id         = (uint8_t)id;
        info.components[i].h_sampling = (uint8_t)((sampling >> 4) & 0x0F);
        info.components[i].v_sampling = (uint8_t)(sampling & 0x0F);
        info.components[i].qt_id      = (uint8_t)qt;

        if (info.components[i].h_sampling > max_h) max_h = info.components[i].h_sampling;
        if (info.components[i].v_sampling > max_v) max_v = info.components[i].v_sampling;
    }

    // Determine subsampling mode from Y sampling factors
    if (nc >= 3) {
        uint8_t yh = info.components[0].h_sampling;
        uint8_t yv = info.components[0].v_sampling;
        if (yh == 2 && yv == 2) {
            info.subsampling = 0;   // 4:2:0
            info.blocks_per_mcu = 6;
            info.y_blocks_per_mcu = 4;
        } else if (yh == 2 && yv == 1) {
            info.subsampling = 1;   // 4:2:2
            info.blocks_per_mcu = 4;
            info.y_blocks_per_mcu = 2;
        } else {
            info.subsampling = 2;   // 4:4:4
            info.blocks_per_mcu = 3;
            info.y_blocks_per_mcu = 1;
        }
    } else {
        // Grayscale
        info.subsampling = 2;
        info.blocks_per_mcu = 1;
        info.y_blocks_per_mcu = 1;
    }

    // Calculate MCU grid
    info.mcu_width  = max_h * 8;
    info.mcu_height = max_v * 8;
    info.mcus_x     = (info.width + info.mcu_width - 1) / info.mcu_width;
    info.mcus_y     = (info.height + info.mcu_height - 1) / info.mcu_height;
    info.total_mcus = info.mcus_x * info.mcus_y;

    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Parse DQT (Define Quantization Table)
// ─────────────────────────────────────────────────────────────────────
static bool parseDQT(fs::File& file, JPEGInfo& info, int length) {
    int remaining = length - 2;  // Subtract the 2-byte length field already read

    while (remaining > 0) {
        int pq_tq = readByte(file);
        if (pq_tq < 0) return false;
        remaining--;

        int precision = (pq_tq >> 4) & 0x0F;   // 0 = 8-bit, 1 = 16-bit
        int table_id  = pq_tq & 0x0F;

        if (table_id >= MAX_QT_TABLES) {
            info.error = "QT table ID out of range";
            return false;
        }

        int elem_size = (precision == 0) ? 1 : 2;
        int table_bytes = 64 * elem_size;

        if (remaining < table_bytes) return false;

        if (precision == 0) {
            // 8-bit quantization values — direct read
            if (file.read(info.qt[table_id], 64) != 64) return false;
        } else {
            // 16-bit quantization values — truncate to 8-bit (uncommon)
            for (int i = 0; i < 64; i++) {
                int val = readU16(file);
                if (val < 0) return false;
                info.qt[table_id][i] = (uint8_t)(val > 255 ? 255 : val);
            }
        }

        remaining -= table_bytes;

        if (table_id >= info.qt_count) {
            info.qt_count = table_id + 1;
        }
    }

    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Parse DHT (Define Huffman Table)
// ─────────────────────────────────────────────────────────────────────
static bool parseDHT(fs::File& file, JPEGInfo& info, int length) {
    int remaining = length - 2;

    while (remaining > 0) {
        int tc_th = readByte(file);
        if (tc_th < 0) return false;
        remaining--;

        int table_class = (tc_th >> 4) & 0x0F;   // 0 = DC, 1 = AC
        int table_id    = tc_th & 0x0F;           // 0 or 1

        if (table_id > 1 || table_class > 1) {
            info.error = "Invalid Huffman table class/id";
            return false;
        }

        // Read bit counts (16 bytes)
        uint8_t bits[17];
        bits[0] = 0;
        if (file.read(&bits[1], 16) != 16) return false;
        remaining -= 16;

        // Count total symbols
        int num_syms = 0;
        for (int i = 1; i <= 16; i++) {
            num_syms += bits[i];
        }

        if (num_syms > 256 || remaining < num_syms) return false;

        // Read symbol values
        uint8_t syms[256];
        if (file.read(syms, num_syms) != num_syms) return false;
        remaining -= num_syms;

        // Build Huffman table
        // Index mapping: DC tables at [0] and [1], AC tables at [2] and [3]
        int idx = table_class * 2 + table_id;
        info.huff_tables[idx].build(bits, syms, num_syms);
        info.has_custom_huffman = true;
    }

    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Parse DRI (Define Restart Interval)
// ─────────────────────────────────────────────────────────────────────
static bool parseDRI(fs::File& file, JPEGInfo& info, int length) {
    if (length != 4) {
        info.error = "Bad DRI length";
        return false;
    }
    int interval = readU16(file);
    if (interval < 0) return false;
    info.restart_interval = (uint16_t)interval;
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Parse SOS (Start Of Scan)
// ─────────────────────────────────────────────────────────────────────
static bool parseSOS(fs::File& file, JPEGInfo& info, int length) {
    int nc = readByte(file);
    if (nc < 0 || nc > MAX_COMPONENTS) return false;

    for (int i = 0; i < nc; i++) {
        int cs = readByte(file);   // Component selector
        int td_ta = readByte(file); // DC/AC table selectors
        if (cs < 0 || td_ta < 0) return false;

        // Find the component by ID and set table selectors
        for (int j = 0; j < info.num_components; j++) {
            if (info.components[j].id == (uint8_t)cs) {
                info.components[j].dc_table = (uint8_t)((td_ta >> 4) & 0x0F);
                info.components[j].ac_table = (uint8_t)(td_ta & 0x0F);
                break;
            }
        }
    }

    // Skip spectral selection and successive approximation (3 bytes)
    // Ss, Se, Ah/Al — for baseline JPEG these are 0, 63, 0
    for (int i = 0; i < 3; i++) {
        if (readByte(file) < 0) return false;
    }

    // Record scan data start position
    info.scan_data_offset = (uint32_t)file.position();

    return true;
}

// ─────────────────────────────────────────────────────────────────────
// parseJPEG — Main entry point
// ─────────────────────────────────────────────────────────────────────
bool parseJPEG(fs::File& file, JPEGInfo& info) {
    // Initialize
    memset(&info, 0, sizeof(JPEGInfo));
    info.valid = false;
    info.error = nullptr;

    // Build standard Huffman tables as default (will be overwritten if DHT markers found)
    buildStandardHuffmanTables(info.huff_tables);

    // Check SOI marker
    int b1 = readByte(file);
    int b2 = readByte(file);
    if (b1 != 0xFF || b2 != JPEG_SOI) {
        info.error = "Not a JPEG file (missing SOI)";
        return false;
    }

    bool got_sof = false;
    bool got_sos = false;

    // Parse markers
    while (!got_sos) {
        // Find next marker (0xFF followed by non-zero, non-0xFF byte)
        int marker = 0;
        while (true) {
            int b = readByte(file);
            if (b < 0) {
                info.error = "Unexpected EOF while scanning for marker";
                return false;
            }
            if (b != 0xFF) continue;

            // Skip padding 0xFF bytes
            do {
                b = readByte(file);
                if (b < 0) {
                    info.error = "Unexpected EOF in marker";
                    return false;
                }
            } while (b == 0xFF);

            if (b == 0x00) continue;  // Byte-stuffed 0xFF in data (shouldn't happen here)
            marker = b;
            break;
        }

        // Handle marker
        switch (marker) {
            case JPEG_EOI:
                info.error = "Unexpected EOI before scan data";
                return false;

            case JPEG_SOF0: {
                int len = readU16(file);
                if (len < 0) { info.error = "Bad SOF0 length"; return false; }
                if (!parseSOF(file, info, len)) {
                    if (!info.error) info.error = "Failed to parse SOF0";
                    return false;
                }
                got_sof = true;
                break;
            }

            case JPEG_SOF2:
                info.error = "Progressive JPEG not supported (only baseline)";
                return false;

            case JPEG_DQT: {
                int len = readU16(file);
                if (len < 0) { info.error = "Bad DQT length"; return false; }
                if (!parseDQT(file, info, len)) {
                    if (!info.error) info.error = "Failed to parse DQT";
                    return false;
                }
                break;
            }

            case JPEG_DRI: {
                int len = readU16(file);
                if (len < 0) { info.error = "Bad DRI length"; return false; }
                if (!parseDRI(file, info, len)) {
                    if (!info.error) info.error = "Failed to parse DRI";
                    return false;
                }
                break;
            }

            case JPEG_DHT: {
                int len = readU16(file);
                if (len < 0) { info.error = "Bad DHT length"; return false; }
                if (!parseDHT(file, info, len)) {
                    if (!info.error) info.error = "Failed to parse DHT";
                    return false;
                }
                break;
            }

            case JPEG_SOS: {
                if (!got_sof) {
                    info.error = "SOS before SOF";
                    return false;
                }
                int len = readU16(file);
                if (len < 0) { info.error = "Bad SOS length"; return false; }
                if (!parseSOS(file, info, len)) {
                    if (!info.error) info.error = "Failed to parse SOS";
                    return false;
                }
                got_sos = true;
                break;
            }

            default: {
                // Skip unknown/unneeded markers (APP0–APP15, COM, etc.)
                int len = readU16(file);
                if (len < 2) { info.error = "Bad marker length"; return false; }
                // Skip payload (length includes the 2-byte length field)
                file.seek(file.position() + len - 2);
                break;
            }
        }
    }

    // Calculate scan data length
    // We need to find EOI. Scan forward from current position.
    uint32_t scan_start = info.scan_data_offset;
    uint32_t file_size = file.size();
    info.scan_data_length = file_size - scan_start;

    // Try to find EOI marker to get exact scan data length
    // (The scan data may be followed by EOI and trailing data)
    // For efficiency, scan from the end of the file backwards
    if (file_size > 2) {
        file.seek(file_size - 2);
        uint8_t tail[2];
        if (file.read(tail, 2) == 2) {
            if (tail[0] == 0xFF && tail[1] == JPEG_EOI) {
                info.scan_data_length = (file_size - 2) - scan_start;
            }
        }
    }

    // Reset file position to scan data start
    file.seek(scan_start);

    info.valid = true;
    return true;
}

} // namespace helium_jpeg
