/*
 * HeliumJPEG — Main Encoder Implementation
 *
 * Parses a JPEG file, walks the Huffman bitstream to find MCU boundaries,
 * splits into packets of up to 210 bytes with per-packet DC predictor reset for
 * independent decodability.
 *
 * SPDX-License-Identifier: MIT
 */

#include "helium_jpeg.h"
#include "jpeg_parser.h"
#include "huffman.h"
#include "bitstream.h"

#include <cstring>
#include <cstdlib>

// Use PSRAM allocation on ESP32 if available
#ifdef ESP32
  #include <esp_heap_caps.h>
  #define HJ_MALLOC(sz)  heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
  #define HJ_FREE(ptr)   heap_caps_free(ptr)
#else
  #define HJ_MALLOC(sz)  malloc(sz)
  #define HJ_FREE(ptr)   free(ptr)
#endif

namespace helium_jpeg {

// ─────────────────────────────────────────────────────────────────────
// Implementation struct (hidden from header, allocated on heap/PSRAM)
// ─────────────────────────────────────────────────────────────────────
struct HeliumJPEG::Impl {
    // Parsed JPEG info
    JPEGInfo jpeg_info;

    // Image ID
    uint16_t image_id;

    // Telemetry
    HeliumTelemetry telemetry;
    bool has_telemetry;

    // Metadata packet config
    uint8_t meta_repeat;

    // MCU boundary table (allocated in PSRAM)
    MCUBoundary* mcu_boundaries;
    uint16_t     mcu_count;

    // Packet plans
    PacketPlan*  packet_plans;
    int          data_packet_count;

    // Metadata payload (pre-built)
    uint8_t meta_payload[PAYLOAD_SIZE];

    // Iteration state
    int current_packet_index;  // -meta_repeat..-1 = meta packets, 0..N-1 = data packets
    uint8_t metadata_sent;
    int total_packet_count;

    // Scan data buffer (for re-encoding)
    uint8_t* scan_buffer;
    uint32_t scan_buffer_size;

    // File reference (kept open for scan data reads)
    fs::File* file_ref;

    // Error
    const char* error;

    Impl() : mcu_boundaries(nullptr), packet_plans(nullptr),
             data_packet_count(0), current_packet_index(0), metadata_sent(0),
             total_packet_count(0), scan_buffer(nullptr),
             scan_buffer_size(0), file_ref(nullptr), error(nullptr),
             has_telemetry(false) {
        memset(&telemetry, 0, sizeof(telemetry));
        memset(meta_payload, 0, sizeof(meta_payload));
    }

    ~Impl() {
        if (mcu_boundaries) HJ_FREE(mcu_boundaries);
        if (packet_plans)   HJ_FREE(packet_plans);
        if (scan_buffer)    HJ_FREE(scan_buffer);
    }
};

// ─────────────────────────────────────────────────────────────────────
// Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────
HeliumJPEG::HeliumJPEG() : impl_(nullptr) {}

HeliumJPEG::~HeliumJPEG() {
    if (impl_) {
        delete impl_;
        impl_ = nullptr;
    }
}

// ─────────────────────────────────────────────────────────────────────
// Clamp helper for DC quantization
// ─────────────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────
// Big-endian write helpers
// ─────────────────────────────────────────────────────────────────────
static void writeU16BE(uint8_t* buf, uint16_t val) {
    buf[0] = (uint8_t)(val >> 8);
    buf[1] = (uint8_t)(val & 0xFF);
}

static void writeU32BE(uint8_t* buf, uint32_t val) {
    buf[0] = (uint8_t)(val >> 24);
    buf[1] = (uint8_t)(val >> 16);
    buf[2] = (uint8_t)(val >> 8);
    buf[3] = (uint8_t)(val & 0xFF);
}

static void writeI16BE(uint8_t* buf, int16_t val) {
    writeU16BE(buf, (uint16_t)val);
}

static void writeI32BE(uint8_t* buf, int32_t val) {
    writeU32BE(buf, (uint32_t)val);
}

// ─────────────────────────────────────────────────────────────────────
// begin() — Parse JPEG and prepare all packets
// ─────────────────────────────────────────────────────────────────────
int HeliumJPEG::begin(fs::File& jpegFile, uint16_t imageId, uint8_t metaRepeat) {
    // Clean up any previous state
    if (impl_) {
        delete impl_;
    }
    impl_ = new Impl();
    if (!impl_) return -1;

    impl_->image_id = imageId;
    impl_->meta_repeat = metaRepeat;
    impl_->file_ref = &jpegFile;

    // Step 1: Parse JPEG headers
    jpegFile.seek(0);
    if (!parseJPEG(jpegFile, impl_->jpeg_info)) {
        impl_->error = impl_->jpeg_info.error ? impl_->jpeg_info.error : "JPEG parse failed";
        return -1;
    }

    // Step 2: Read scan data into buffer
    uint32_t scan_len = impl_->jpeg_info.scan_data_length;
    if (scan_len == 0 || scan_len > 2 * 1024 * 1024) {  // Sanity: max 2MB
        impl_->error = "Scan data size invalid";
        return -1;
    }

    impl_->scan_buffer = (uint8_t*)HJ_MALLOC(scan_len);
    if (!impl_->scan_buffer) {
        impl_->error = "Failed to allocate scan data buffer";
        return -1;
    }
    impl_->scan_buffer_size = scan_len;

    jpegFile.seek(impl_->jpeg_info.scan_data_offset);
    size_t read = jpegFile.read(impl_->scan_buffer, scan_len);
    if (read < scan_len) {
        // Might have hit EOF before expected — adjust length
        impl_->scan_buffer_size = (uint32_t)read;
    }

    // Step 3: Walk the Huffman bitstream to find MCU boundaries
    if (!walkScanData(jpegFile)) {
        return -1;
    }

    // Step 4: Build packet plans (group MCUs into packets)
    if (!buildPacketPlans()) {
        return -1;
    }

    // Step 5: Build metadata payload
    buildMetadataPayload();

    // Set up iteration
    impl_->total_packet_count = impl_->meta_repeat + impl_->data_packet_count;
    impl_->current_packet_index = 0;
    impl_->metadata_sent = 0;

    return impl_->total_packet_count;
}

// ─────────────────────────────────────────────────────────────────────
// walkScanData() — Parse Huffman bitstream to find MCU boundaries
// ─────────────────────────────────────────────────────────────────────
bool HeliumJPEG::walkScanData(fs::File& file) {
    JPEGInfo& ji = impl_->jpeg_info;

    // Allocate MCU boundary table
    impl_->mcu_count = ji.total_mcus;
    size_t alloc_size = sizeof(MCUBoundary) * (impl_->mcu_count + 1); // +1 for sentinel
    impl_->mcu_boundaries = (MCUBoundary*)HJ_MALLOC(alloc_size);
    if (!impl_->mcu_boundaries) {
        impl_->error = "Failed to allocate MCU boundary table";
        return false;
    }
    memset(impl_->mcu_boundaries, 0, alloc_size);

    // Set up BitReader on the scan data buffer
    BitReader reader;
    reader.init(impl_->scan_buffer, impl_->scan_buffer_size, true);

    // DC predictors (start at 0)
    int16_t dc_pred[MAX_COMPONENTS] = {0};

    // Get Huffman table references for each component
    const HuffmanTable* dc_table[MAX_COMPONENTS];
    const HuffmanTable* ac_table[MAX_COMPONENTS];
    for (int c = 0; c < ji.num_components; c++) {
        dc_table[c] = &ji.huff_tables[ji.components[c].dc_table];         // DC table index 0 or 1
        ac_table[c] = &ji.huff_tables[2 + ji.components[c].ac_table];     // AC table index 0 or 1
    }

    // Walk through all MCUs
    uint16_t mcus_since_restart = 0;
    for (uint16_t mcu_idx = 0; mcu_idx < impl_->mcu_count; mcu_idx++) {
        // If a restart interval is in effect and we've just crossed one,
        // consume the 2-byte RSTn marker and reset DC predictors before
        // recording this MCU's boundary. Restart markers always appear on a
        // byte boundary (encoders pad with 1-bits beforehand), and the JPEG
        // spec mandates the DC reset at exactly this point.
        bool at_restart_point = false;
        if (ji.restart_interval > 0 && mcu_idx > 0 && mcus_since_restart == ji.restart_interval) {
            reader.alignToByte();
            if (!reader.consumeRestartMarker()) {
                impl_->error = "Expected restart marker not found (bitstream desync)";
                return false;
            }
            dc_pred[0] = dc_pred[1] = dc_pred[2] = 0;
            mcus_since_restart = 0;
            at_restart_point = true;
        }

        // Record MCU boundary BEFORE decoding this MCU
        MCUBoundary& boundary = impl_->mcu_boundaries[mcu_idx];
        boundary.byte_offset = (uint32_t)reader.getBytePos();
        boundary.bit_offset  = reader.getBitPos();
        boundary.dc_y  = dc_pred[0];
        boundary.dc_cb = (ji.num_components > 1) ? dc_pred[1] : 0;
        boundary.dc_cr = (ji.num_components > 2) ? dc_pred[2] : 0;
        boundary.is_restart_point = at_restart_point;
        mcus_since_restart++;

        // Decode each block in the MCU
        // For 4:2:0: Y0, Y1, Y2, Y3, Cb, Cr
        // For 4:2:2: Y0, Y1, Cb, Cr
        // For 4:4:4: Y, Cb, Cr
        for (int c = 0; c < ji.num_components; c++) {
            int blocks = (c == 0) ? ji.y_blocks_per_mcu : 1;
            for (int b = 0; b < blocks; b++) {
                // --- Decode DC coefficient ---
                int dc_symbol = dc_table[c]->decode(reader);
                if (dc_symbol < 0) {
                    impl_->error = "Huffman DC decode error";
                    return false;
                }

                int dc_value = 0;
                if (dc_symbol > 0) {
                    int magnitude = reader.readBits(dc_symbol);
                    if (magnitude < 0) {
                        impl_->error = "Failed to read DC magnitude bits";
                        return false;
                    }
                    dc_value = jpegDecodeMagnitude(dc_symbol, magnitude);
                }

                // Update DC predictor
                dc_pred[c] += (int16_t)dc_value;

                // --- Skip AC coefficients ---
                int ac_count = 0;
                while (ac_count < 63) {
                    int ac_symbol = ac_table[c]->decode(reader);
                    if (ac_symbol < 0) {
                        impl_->error = "Huffman AC decode error";
                        return false;
                    }

                    if (ac_symbol == 0x00) {
                        // EOB — end of block
                        break;
                    }

                    int run  = (ac_symbol >> 4) & 0x0F;  // Zero run length
                    int size = ac_symbol & 0x0F;          // Coefficient size

                    if (ac_symbol == 0xF0) {
                        // ZRL — skip 16 zeros
                        ac_count += 16;
                        continue;
                    }

                    ac_count += run + 1;

                    // Skip the magnitude bits
                    if (size > 0) {
                        reader.skipBits(size);
                    }
                }
            }
        }
    }

    // Record sentinel entry (position after last MCU)
    MCUBoundary& sentinel = impl_->mcu_boundaries[impl_->mcu_count];
    sentinel.byte_offset = (uint32_t)reader.getBytePos();
    sentinel.bit_offset  = reader.getBitPos();
    sentinel.dc_y  = dc_pred[0];
    sentinel.dc_cb = (ji.num_components > 1) ? dc_pred[1] : 0;
    sentinel.dc_cr = (ji.num_components > 2) ? dc_pred[2] : 0;

    return true;
}

// ─────────────────────────────────────────────────────────────────────
// buildPacketPlans() — Group MCUs into packets of ~200 bytes
// ─────────────────────────────────────────────────────────────────────
bool HeliumJPEG::buildPacketPlans() {
    // First pass: count packets
    int packet_count = 0;
    uint16_t mcu = 0;
    while (mcu < impl_->mcu_count) {
        // Calculate how many MCUs fit in SCAN_DATA_PER_PACKET bytes
        uint32_t start_byte = impl_->mcu_boundaries[mcu].byte_offset;
        uint8_t  start_bit  = impl_->mcu_boundaries[mcu].bit_offset;
        // Always include at least one MCU per packet (it can't be split further),
        // regardless of how large it is on its own.
        uint16_t end_mcu = mcu + 1;

        while (end_mcu < impl_->mcu_count) {
            // A restart marker was consumed right before MCU `end_mcu` in the
            // original stream (its DC predictors were reset there). That must
            // always be a packet boundary -- our own per-packet DC reset
            // already provides the same guarantee, and this lets us strip the
            // restart marker bytes out of the wire format entirely rather
            // than teaching the decoder about them.
            if (impl_->mcu_boundaries[end_mcu].is_restart_point) break;

            // Tentatively test whether including ONE MORE MCU (i.e. extending the
            // packet through `candidate`) still fits the budget. We must not accept
            // `end_mcu` itself until its span has been validated -- otherwise a
            // rejected (over-budget) candidate ends up being used anyway.
            uint16_t candidate = end_mcu + 1;
            uint32_t end_byte = impl_->mcu_boundaries[candidate].byte_offset;
            uint8_t  end_bit  = impl_->mcu_boundaries[candidate].bit_offset;

            // Calculate byte span (approximate, DC re-encoding may change slightly)
            uint32_t span_bytes = end_byte - start_byte;
            if (end_bit > 0 || start_bit > 0) span_bytes++;  // Account for partial bytes

            // Resetting three first-block DC predictors can add up to ten
            // bytes in the pathological case.  Keep a conservative margin;
            // reEncodeDC() remains the authoritative capacity check.
            if (span_bytes > (uint32_t)(SCAN_DATA_PER_PACKET - 16)) break;

            end_mcu = candidate;
        }

        packet_count++;
        mcu = end_mcu;
    }

    // Allocate packet plans
    impl_->data_packet_count = packet_count;
    impl_->packet_plans = (PacketPlan*)HJ_MALLOC(sizeof(PacketPlan) * packet_count);
    if (!impl_->packet_plans) {
        impl_->error = "Failed to allocate packet plans";
        return false;
    }

    // Second pass: fill in packet plans
    mcu = 0;
    for (int p = 0; p < packet_count; p++) {
        PacketPlan& plan = impl_->packet_plans[p];
        plan.first_mcu = mcu;
        plan.scan_byte_start = impl_->mcu_boundaries[mcu].byte_offset;
        plan.scan_bit_start  = impl_->mcu_boundaries[mcu].bit_offset;

        // See matching comment in the first (counting) pass above: we must
        // validate a candidate MCU count BEFORE accepting it, or an over-budget
        // span ends up being used anyway.
        uint16_t end_mcu = mcu + 1;
        while (end_mcu < impl_->mcu_count) {
            if (impl_->mcu_boundaries[end_mcu].is_restart_point) break;

            uint16_t candidate = end_mcu + 1;
            uint32_t end_byte = impl_->mcu_boundaries[candidate].byte_offset;
            uint32_t span = end_byte - plan.scan_byte_start;
            if (impl_->mcu_boundaries[candidate].bit_offset > 0 || plan.scan_bit_start > 0)
                span++;
            if (span > (uint32_t)(SCAN_DATA_PER_PACKET - 16)) break;
            end_mcu = candidate;
        }

        plan.mcu_count = end_mcu - mcu;

        // End position: start of next packet or sentinel
        plan.scan_byte_end = impl_->mcu_boundaries[end_mcu].byte_offset;
        plan.scan_bit_end  = impl_->mcu_boundaries[end_mcu].bit_offset;

        mcu = end_mcu;
    }

    return true;
}

// ─────────────────────────────────────────────────────────────────────
// buildMetadataPayload() — Construct the 202-byte metadata payload
// ─────────────────────────────────────────────────────────────────────
void HeliumJPEG::buildMetadataPayload() {
    uint8_t* p = impl_->meta_payload;
    memset(p, 0, PAYLOAD_SIZE);

    JPEGInfo& ji = impl_->jpeg_info;

    // Bytes 0-1: width
    writeU16BE(p + 0, ji.width);
    // Bytes 2-3: height
    writeU16BE(p + 2, ji.height);
    // Bytes 4-5: mcus_x
    writeU16BE(p + 4, ji.mcus_x);
    // Bytes 6-7: mcus_y
    writeU16BE(p + 6, ji.mcus_y);
    // Bytes 8-9: total_data_packets
    writeU16BE(p + 8, (uint16_t)impl_->data_packet_count);
    // Byte 10: quality_hint (0 = unknown from file)
    p[10] = 0;
    // Byte 11: num_qt_tables
    p[11] = ji.qt_count;
    // Bytes 12-75: QT table 0
    if (ji.qt_count > 0) memcpy(p + 12, ji.qt[0], 64);
    // Bytes 76-139: QT table 1
    if (ji.qt_count > 1) memcpy(p + 76, ji.qt[1], 64);

    // Bytes 140-191: Telemetry (if set)
    // Will be populated in setTelemetry() or left as zeros
}

// ─────────────────────────────────────────────────────────────────────
// setTelemetry() — Attach telemetry data to metadata packets
// ─────────────────────────────────────────────────────────────────────
void HeliumJPEG::setTelemetry(const HeliumTelemetry& telem) {
    if (!impl_) return;
    impl_->telemetry = telem;
    impl_->has_telemetry = true;

    // Pack telemetry into metadata payload bytes 140-191
    uint8_t* t = impl_->meta_payload + 140;

    writeI32BE(t + 0,  telem.latitude);
    writeI32BE(t + 4,  telem.longitude);
    writeU16BE(t + 8,  telem.altitude);
    writeU16BE(t + 10, telem.speed);
    writeU16BE(t + 12, telem.heading);
    writeI16BE(t + 14, telem.v_speed);
    t[16] = telem.hdop;
    t[17] = telem.satellites;
    t[18] = telem.fix_type;
    writeU32BE(t + 19, telem.timestamp);
    writeU32BE(t + 23, telem.uptime);
    writeU16BE(t + 27, telem.battery_mv);
    t[29] = (uint8_t)telem.esp_temp;
    writeI16BE(t + 30, telem.ext_temp);
    writeU16BE(t + 32, telem.pressure);
    t[34] = telem.humidity;
    writeU32BE(t + 35, telem.jpeg_file_size);
    writeU16BE(t + 39, telem.capture_ms);
    writeU16BE(t + 41, telem.free_heap_kb);
    writeU16BE(t + 43, telem.status_flags);
    // Bytes 45-51: reserved (already zeroed)
}

// ─────────────────────────────────────────────────────────────────────
// reEncodeDC() — Re-encode first MCU's DC coefficients for predictor reset
// ─────────────────────────────────────────────────────────────────────
bool HeliumJPEG::reEncodeDC(uint8_t* scan_buf, size_t scan_len,
                             uint8_t src_bit_offset,
                             uint16_t mcu_count,
                             int16_t abs_dc_y, int16_t abs_dc_cb, int16_t abs_dc_cr,
                             uint8_t* out_buf, size_t out_capacity,
                             size_t& out_bytes_written) {
    JPEGInfo& ji = impl_->jpeg_info;

    BitReader reader;
    reader.init(scan_buf, scan_len, true);  // scan_buffer is raw JPEG data; must skip 0xFF 0x00 stuffing

    if (src_bit_offset > 0) {
        reader.skipBits(src_bit_offset);
    }

    BitWriter writer;
    writer.init(out_buf, out_capacity);

    // DC/AC table references
    const HuffmanTable* dc_tab[MAX_COMPONENTS];
    const HuffmanTable* ac_tab[MAX_COMPONENTS];
    for (int c = 0; c < ji.num_components; c++) {
        dc_tab[c] = &ji.huff_tables[ji.components[c].dc_table];
        ac_tab[c] = &ji.huff_tables[2 + ji.components[c].ac_table];
    }

    // Absolute DC values for the first block of each component.
    // After predictor reset to 0, the new delta = absolute value.
    int16_t abs_dc[MAX_COMPONENTS] = {abs_dc_y, abs_dc_cb, abs_dc_cr};

    // Track the running DC predictor within the original stream for this MCU.
    // For Y: Y[0]'s original delta was relative to the previous MCU's Y[3].
    //        Y[1]'s delta is relative to Y[0], Y[2] to Y[1], Y[3] to Y[2].
    // After reset:
    //   Y[0] new_delta = abs_dc_y (absolute)
    //   Y[1] new_delta = Y[1]_abs - Y[0]_abs = original Y[1]_delta (unchanged!)
    //   Y[2], Y[3] similarly unchanged.
    // Cb and Cr: first block delta = absolute, but there's only 1 block each.
    //
    // So: only Y[0], Cb[0], Cr[0] need changed DC values.
    //     All other blocks copy DC verbatim.
    bool is_first_block[MAX_COMPONENTS] = {true, true, true};

    // Re-encode exactly the MCUs assigned to this packet.  Do not copy to
    // the end of the byte slice: that slice can contain JPEG pad bits, byte
    // stuffing, or a restart marker immediately after the final MCU.
    for (uint16_t mcu = 0; mcu < mcu_count; mcu++) {
        for (int c = 0; c < ji.num_components; c++) {
            int blocks = (c == 0) ? ji.y_blocks_per_mcu : 1;

            for (int b = 0; b < blocks; b++) {
            // --- DC coefficient ---
            int orig_dc_sym = dc_tab[c]->decode(reader);
            if (orig_dc_sym < 0) return false;

            // Read original magnitude bits (if any)
            int orig_magnitude = 0;
            if (orig_dc_sym > 0) {
                orig_magnitude = reader.readBits(orig_dc_sym);
                if (orig_magnitude < 0) return false;
            }

            if (is_first_block[c]) {
                // Re-encode DC with new delta = absolute value (predictor reset to 0)
                // abs_dc[c] is the DC predictor state BEFORE this MCU (i.e. the
                // accumulated sum of all prior deltas).  The original stream encoded
                // the first block as delta = (first_block_abs - abs_dc[c]).  We need
                // to re-encode it as its absolute value (delta from 0, because the
                // per-packet predictor is reset to 0 at the decoder).
                //
                // Correct absolute value = predictor_before_MCU + original_delta.
                int orig_delta = jpegDecodeMagnitude(orig_dc_sym, orig_magnitude);
                int16_t abs_value = abs_dc[c] + (int16_t)orig_delta;

                uint16_t new_mag;

                int new_cat = jpegEncodeMagnitude((int)abs_value, new_mag);

                if (!dc_tab[c]->encode(writer, (uint8_t)new_cat)) return false;
                if (new_cat > 0) {
                    if (!writer.writeBits(new_mag, new_cat)) return false;
                }

                is_first_block[c] = false;
            } else {
                // Non-first block: copy DC verbatim (delta is correct as-is)
                if (!dc_tab[c]->encode(writer, (uint8_t)orig_dc_sym)) return false;
                if (orig_dc_sym > 0) {
                    if (!writer.writeBits((uint16_t)orig_magnitude, orig_dc_sym)) return false;
                }
            }

                // --- AC coefficients: decode and re-encode verbatim ---
                int ac_count = 0;
                while (ac_count < 63) {
                    int ac_sym = ac_tab[c]->decode(reader);
                    if (ac_sym < 0) return false;

                    if (!ac_tab[c]->encode(writer, (uint8_t)ac_sym)) return false;

                    if (ac_sym == 0x00) break;  // EOB

                    if (ac_sym == 0xF0) {
                        ac_count += 16;
                        continue;
                    }

                    int run  = (ac_sym >> 4) & 0x0F;
                    int size = ac_sym & 0x0F;
                    ac_count += run + 1;

                    if (size > 0) {
                        int mag = reader.readBits(size);
                        if (mag < 0) return false;
                        if (!writer.writeBits((uint16_t)mag, size)) return false;
                    }
                }
            }
        }
    }

    writer.padToByteJPEG();
    out_bytes_written = writer.getBytesWritten();
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// getNextPacket() — Yield the next packet
// ─────────────────────────────────────────────────────────────────────
bool HeliumJPEG::getNextPacket(HeliumPacket& pkt) {
    if (!impl_) return false;

    pkt.length = 0;

    int idx = impl_->current_packet_index;
    if (idx >= impl_->data_packet_count && impl_->metadata_sent >= impl_->meta_repeat) return false;

    // Place metadata at approximately 0%, 33%, and 66% of the data stream.
    const bool emitMetadata = impl_->metadata_sent < impl_->meta_repeat &&
        (idx >= impl_->data_packet_count ||
         ((uint32_t)idx * impl_->meta_repeat) / impl_->data_packet_count >= impl_->metadata_sent);

    memset(pkt.data, 0, PACKET_SIZE);

    if (emitMetadata) {
        // ── Metadata packet ──
        uint8_t* h = pkt.data;

        // Header
        writeU16BE(h + 0, impl_->image_id);
        writeU16BE(h + 2, 0);               // packet_id = 0 (metadata)
        writeU16BE(h + 4, 0);               // mcu_index = 0
        h[6] = 0;                           // mcu_count = 0 for metadata
        h[7] = impl_->jpeg_info.subsampling & 0x07;

        // Payload
        memcpy(h + METADATA_HEADER_SIZE, impl_->meta_payload, PAYLOAD_SIZE);
        pkt.length = PACKET_SIZE;

    } else {
        // ── Data packet ──
        PacketPlan& plan = impl_->packet_plans[idx];
        MCUBoundary& boundary = impl_->mcu_boundaries[plan.first_mcu];
        MCUBoundary& next = impl_->mcu_boundaries[plan.first_mcu + plan.mcu_count];

        uint8_t* h = pkt.data;

        // Header
        writeU16BE(h + 0, impl_->image_id);
        writeU16BE(h + 2, (uint16_t)(idx + 1));      // packet_id (1-based)
        writeU16BE(h + 4, plan.first_mcu);             // mcu_index
        h[6] = (uint8_t)plan.mcu_count;                // exact packet framing
        // Payload: Re-encoded scan data
        // Extract the raw scan bytes for this packet's MCU range
        uint32_t start = plan.scan_byte_start;
        uint32_t end   = next.byte_offset;
        if (next.bit_offset > 0) end++;  // Include partial byte

        uint32_t raw_len = end - start;
        if (start + raw_len > impl_->scan_buffer_size) {
            raw_len = impl_->scan_buffer_size - start;
        }

        uint8_t* scan_src = impl_->scan_buffer + start;
        uint8_t* scan_dst = h + DATA_HEADER_SIZE;

        // Re-encode the first MCU's DC coefficients
        size_t written = 0;
        bool ok = reEncodeDC(scan_src, raw_len,
                              boundary.bit_offset,
                              plan.mcu_count,
                              boundary.dc_y, boundary.dc_cb, boundary.dc_cr,
                              scan_dst, SCAN_DATA_PER_PACKET,
                              written);

        if (!ok) {
            // A raw JPEG slice is not a valid Helium scan stream: it can
            // contain byte-stuffing and a non-zero DC predictor.  Emitting it
            // would silently desynchronise the receiver, so fail cleanly.
            impl_->error = "Packet DC re-encoding exceeded capacity or failed";
            return false;
        }

        // LoRaWAN carries the application payload length, so do not transmit
        // the zero-filled tail of the fixed-capacity packet buffer.  `written`
        // is authoritative; trimming by byte value would corrupt a valid scan
        // stream that happens to end in one or more 0x00 bytes.
        if (written == 0 || written > SCAN_DATA_PER_PACKET) {
            impl_->error = "Packet encoder returned an invalid length";
            return false;
        }
        pkt.length = DATA_HEADER_SIZE + written;
    }

    if (emitMetadata) impl_->metadata_sent++;
    else impl_->current_packet_index++;
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Accessors
// ─────────────────────────────────────────────────────────────────────
int HeliumJPEG::getPacketCount() const {
    return impl_ ? impl_->total_packet_count : 0;
}

int HeliumJPEG::getDataPacketCount() const {
    return impl_ ? impl_->data_packet_count : 0;
}

uint16_t HeliumJPEG::getImageWidth() const {
    return impl_ ? impl_->jpeg_info.width : 0;
}

uint16_t HeliumJPEG::getImageHeight() const {
    return impl_ ? impl_->jpeg_info.height : 0;
}

uint16_t HeliumJPEG::getMCUsX() const {
    return impl_ ? impl_->jpeg_info.mcus_x : 0;
}

uint16_t HeliumJPEG::getMCUsY() const {
    return impl_ ? impl_->jpeg_info.mcus_y : 0;
}

void HeliumJPEG::reset() {
    if (impl_) {
        impl_->current_packet_index = 0;
        impl_->metadata_sent = 0;
    }
}

bool HeliumJPEG::skipPackets(uint16_t count) {
    HeliumPacket ignored;
    while (count-- > 0) {
        if (!getNextPacket(ignored)) return false;
    }
    return true;
}

const char* HeliumJPEG::getError() const {
    return impl_ ? impl_->error : "Not initialized";
}

} // namespace helium_jpeg
