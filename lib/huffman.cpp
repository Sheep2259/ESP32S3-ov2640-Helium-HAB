/*
 * HeliumJPEG — Huffman Codec Implementation
 *
 * SPDX-License-Identifier: MIT
 */

#include "huffman.h"
#include <cstring>

namespace helium_jpeg {

// ─────────────────────────────────────────────────────────────────────
// HuffmanTable::build
// ─────────────────────────────────────────────────────────────────────
void HuffmanTable::build(const uint8_t bits[17], const uint8_t* syms, int num_syms) {
    num_symbols = num_syms;
    memcpy(symbols, syms, num_syms);
    memset(fast_symbol, 0, sizeof(fast_symbol));
    memset(fast_length, 0, sizeof(fast_length));
    memset(enc_code, 0, sizeof(enc_code));
    memset(enc_length, 0, sizeof(enc_length));

    // Build decode tables using the JPEG standard algorithm (Figure C.1 / C.2)
    int code = 0;
    int si = 0;  // Symbol index

    for (int len = 1; len <= 16; len++) {
        mincode[len] = code;
        valptr[len] = si;

        for (int i = 0; i < bits[len]; i++) {
            // Build encode table
            if (si < num_syms) {
                enc_code[symbols[si]]   = (uint16_t)code;
                enc_length[symbols[si]] = (uint8_t)len;
            }

            // Build fast decode table for short codes
            if (len <= HUFF_FAST_BITS) {
                // Replicate into fast table for all possible extensions
                int repl = HUFF_FAST_BITS - len;
                for (int j = 0; j < (1 << repl); j++) {
                    int idx = (code << repl) | j;
                    if (idx < HUFF_FAST_SIZE) {
                        fast_symbol[idx] = symbols[si];
                        fast_length[idx] = (uint8_t)len;
                    }
                }
            }

            code++;
            si++;
        }

        maxcode[len] = code;
        code <<= 1;
    }

    // Sentinel: no valid 17-bit code
    maxcode[0]  = 0;
    maxcode[17] = 0x7FFFFFFF;
}

// ─────────────────────────────────────────────────────────────────────
// HuffmanTable::decode
// ─────────────────────────────────────────────────────────────────────
int HuffmanTable::decode(BitReader& reader) const {
    if (reader.isEOF()) return -1;

    // Fast path: build() populates fast_symbol/fast_length for every
    // possible HUFF_FAST_BITS-bit prefix, so a single table lookup resolves
    // any code of length <= HUFF_FAST_BITS (which covers the large majority
    // of real JPEG Huffman codes). peekBits() returns -1 rather than
    // zero-padding when fewer than HUFF_FAST_BITS real bits remain, so this
    // never fabricates a match out of a truncated stream.
    int peek = reader.peekBits(HUFF_FAST_BITS);
    if (peek >= 0 && fast_length[peek] > 0) {
        reader.skipBits(fast_length[peek]);
        return fast_symbol[peek];
    }

    // Slow path for codes longer than HUFF_FAST_BITS bits (or when we're
    // close enough to EOF that the fast peek couldn't be filled). If the
    // peek succeeded but matched no short code, its HUFF_FAST_BITS-bit
    // prefix is already known to be part of a longer code, so consume it in
    // one shot and resume bit-by-bit from there instead of re-reading it.
    int code = 0;
    int len  = 0;
    if (peek >= 0) {
        reader.skipBits(HUFF_FAST_BITS);
        code = peek;
        len  = HUFF_FAST_BITS;
    }

    for (len = len + 1; len <= 16; len++) {
        int bit = reader.readBit();
        if (bit < 0) return -1;
        code = (code << 1) | bit;

        if (code < maxcode[len]) {
            int idx = valptr[len] + (code - mincode[len]);
            if (idx >= 0 && idx < num_symbols) {
                return symbols[idx];
            }
            return -1;
        }
    }

    return -1;  // No valid code found in 16 bits — corrupt data
}

// ─────────────────────────────────────────────────────────────────────
// HuffmanTable::encode
// ─────────────────────────────────────────────────────────────────────
bool HuffmanTable::encode(BitWriter& writer, uint8_t symbol) const {
    uint8_t len = enc_length[symbol];
    if (len == 0) return false;  // Symbol not in table
    return writer.writeBits(enc_code[symbol], len);
}

// ─────────────────────────────────────────────────────────────────────
// HuffmanTable::getCode
// ─────────────────────────────────────────────────────────────────────
bool HuffmanTable::getCode(uint8_t symbol, uint16_t& code, uint8_t& length) const {
    length = enc_length[symbol];
    if (length == 0) return false;
    code = enc_code[symbol];
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Standard JPEG Huffman Tables (ITU-T T.81, Annex K)
// ─────────────────────────────────────────────────────────────────────

// Table K.3 — DC Luminance
const uint8_t STD_DC_LUMA_BITS[17] = {
    0,  // [0] unused
    0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0
};
const uint8_t STD_DC_LUMA_SYMS[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
};
const int STD_DC_LUMA_NUM_SYMS = 12;

// Table K.4 — DC Chrominance
const uint8_t STD_DC_CHROMA_BITS[17] = {
    0,
    0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0
};
const uint8_t STD_DC_CHROMA_SYMS[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
};
const int STD_DC_CHROMA_NUM_SYMS = 12;

// Table K.5 — AC Luminance
const uint8_t STD_AC_LUMA_BITS[17] = {
    0,
    0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7D
};
const uint8_t STD_AC_LUMA_SYMS[] = {
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
    0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
    0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
    0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
    0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
    0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
    0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
    0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
    0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
    0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
    0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
    0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
    0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
    0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
    0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
    0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa
};
const int STD_AC_LUMA_NUM_SYMS = 162;

// Table K.6 — AC Chrominance
const uint8_t STD_AC_CHROMA_BITS[17] = {
    0,
    0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77
};
const uint8_t STD_AC_CHROMA_SYMS[] = {
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
    0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
    0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
    0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
    0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
    0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
    0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
    0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
    0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
    0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
    0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
    0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
    0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
    0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa
};
const int STD_AC_CHROMA_NUM_SYMS = 162;

// ─────────────────────────────────────────────────────────────────────
// Build standard table set
// ─────────────────────────────────────────────────────────────────────
void buildStandardHuffmanTables(HuffmanTable tables[4]) {
    tables[0].build(STD_DC_LUMA_BITS,   STD_DC_LUMA_SYMS,   STD_DC_LUMA_NUM_SYMS);
    tables[1].build(STD_DC_CHROMA_BITS, STD_DC_CHROMA_SYMS, STD_DC_CHROMA_NUM_SYMS);
    tables[2].build(STD_AC_LUMA_BITS,   STD_AC_LUMA_SYMS,   STD_AC_LUMA_NUM_SYMS);
    tables[3].build(STD_AC_CHROMA_BITS, STD_AC_CHROMA_SYMS, STD_AC_CHROMA_NUM_SYMS);
}

} // namespace helium_jpeg
