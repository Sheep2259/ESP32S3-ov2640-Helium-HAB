/*
 * HeliumJPEG — Huffman Codec
 *
 * JPEG Huffman table construction, decoding, and encoding.
 * Includes standard JPEG Huffman tables (Annex K) as compile-time constants.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once
#include <cstdint>
#include <cstddef>
#include "bitstream.h"

namespace helium_jpeg {

/// Maximum number of symbols in a Huffman table (JPEG spec limit).
static constexpr int HUFF_MAX_SYMBOLS = 256;

/// Fast lookup table size (2^HUFF_FAST_BITS entries).
static constexpr int HUFF_FAST_BITS = 9;
static constexpr int HUFF_FAST_SIZE = 1 << HUFF_FAST_BITS;

// ─────────────────────────────────────────────────────────────────────
// HuffmanTable — Decode + Encode a single JPEG Huffman table
// ─────────────────────────────────────────────────────────────────────
struct HuffmanTable {
    // --- Decode side ---
    // Fast lookup (up to HUFF_FAST_BITS)
    uint8_t  fast_symbol[HUFF_FAST_SIZE];   // Symbol for fast path
    uint8_t  fast_length[HUFF_FAST_SIZE];   // Code length for fast path (0 = slow path)

    // Slow decode tables
    // maxcode has 18 entries: [0] unused, [1..16] one per bit length, and
    // [17] a sentinel (0x7FFFFFFF) so the decode loop's `code < maxcode[len]`
    // check can never spuriously match past the longest valid code length.
    int      maxcode[18];
    int      valptr[17];      // Index into symbols[] for first code of each bit length
    int      mincode[17];     // Min code value for each bit length

    // --- Encode side ---
    uint16_t enc_code[HUFF_MAX_SYMBOLS];    // Huffman code for each symbol
    uint8_t  enc_length[HUFF_MAX_SYMBOLS];  // Code length for each symbol

    // --- Shared ---
    uint8_t  symbols[HUFF_MAX_SYMBOLS];     // Symbol list in code order
    int      num_symbols;

    /// Build decode + encode tables from JPEG DHT bits[1..16] and symbols[].
    /// `bits` = count of codes at each length (bits[1] = # of 1-bit codes, etc.)
    /// `syms` = symbol values in code order
    /// `num_syms` = total number of symbols
    void build(const uint8_t bits[17], const uint8_t* syms, int num_syms);

    /// Decode one symbol from a BitReader. Returns symbol value, or -1 on error.
    int decode(BitReader& reader) const;

    /// Encode one symbol: writes Huffman code to BitWriter.
    /// Returns true on success.
    bool encode(BitWriter& writer, uint8_t symbol) const;

    /// Get the Huffman code and length for a symbol (for manual bit manipulation).
    /// Returns false if symbol is not in the table.
    bool getCode(uint8_t symbol, uint16_t& code, uint8_t& length) const;
};

// ─────────────────────────────────────────────────────────────────────
// JPEG DC coefficient encoding/decoding helpers
// ─────────────────────────────────────────────────────────────────────

/// Compute the JPEG "category" (number of bits) for a DC/AC coefficient value.
/// Category 0 = value is 0, Category 1 = -1 or 1, etc.
inline int jpegCategory(int value) {
    if (value < 0) value = -value;
    int cat = 0;
    while (value > 0) {
        cat++;
        value >>= 1;
    }
    return cat;
}

/// Encode a DC/AC coefficient value into its category + magnitude bits.
/// Returns the magnitude bit pattern in `magnitude` and the category in return value.
inline int jpegEncodeMagnitude(int value, uint16_t& magnitude) {
    int cat = jpegCategory(value);
    if (cat == 0) {
        magnitude = 0;
        return 0;
    }
    if (value < 0) {
        // JPEG uses one's complement for negative values
        magnitude = (uint16_t)(value + (1 << cat) - 1);
    } else {
        magnitude = (uint16_t)value;
    }
    return cat;
}

/// Decode magnitude bits back to a coefficient value.
/// `cat` = category (number of bits), `magnitude` = raw magnitude bits.
inline int jpegDecodeMagnitude(int cat, int magnitude) {
    if (cat == 0) return 0;
    // If MSB of magnitude is 0, the value is negative
    if (magnitude < (1 << (cat - 1))) {
        return magnitude - (1 << cat) + 1;
    }
    return magnitude;
}

// ─────────────────────────────────────────────────────────────────────
// Standard JPEG Huffman Tables (Annex K of ITU-T T.81)
// ─────────────────────────────────────────────────────────────────────

/// Standard DC Luminance (Table K.3)
extern const uint8_t STD_DC_LUMA_BITS[17];
extern const uint8_t STD_DC_LUMA_SYMS[];
extern const int     STD_DC_LUMA_NUM_SYMS;

/// Standard DC Chrominance (Table K.4)
extern const uint8_t STD_DC_CHROMA_BITS[17];
extern const uint8_t STD_DC_CHROMA_SYMS[];
extern const int     STD_DC_CHROMA_NUM_SYMS;

/// Standard AC Luminance (Table K.5)
extern const uint8_t STD_AC_LUMA_BITS[17];
extern const uint8_t STD_AC_LUMA_SYMS[];
extern const int     STD_AC_LUMA_NUM_SYMS;

/// Standard AC Chrominance (Table K.6)
extern const uint8_t STD_AC_CHROMA_BITS[17];
extern const uint8_t STD_AC_CHROMA_SYMS[];
extern const int     STD_AC_CHROMA_NUM_SYMS;

/// Build a set of 4 standard Huffman tables (DC luma, DC chroma, AC luma, AC chroma).
/// `tables` must have space for at least 4 HuffmanTable entries.
/// Index: 0 = DC Luma, 1 = DC Chroma, 2 = AC Luma, 3 = AC Chroma.
void buildStandardHuffmanTables(HuffmanTable tables[4]);

} // namespace helium_jpeg
