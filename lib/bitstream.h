/*
 * HeliumJPEG — Bitstream Utilities
 *
 * MSB-first bit-level reader and writer for JPEG Huffman stream processing.
 * BitReader handles JPEG byte-stuffing (skips 0x00 after 0xFF).
 * BitWriter produces clean byte-aligned output.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>

namespace helium_jpeg {

// ─────────────────────────────────────────────────────────────────────
// BitReader — MSB-first bit reader with JPEG byte-stuffing awareness
// ─────────────────────────────────────────────────────────────────────
class BitReader {
public:
    BitReader() : data_(nullptr), length_(0), byte_pos_(0), bit_pos_(0) {}

    /// Initialize with a raw byte buffer.
    /// If `jpeg_stuffed` is true, 0xFF 0x00 sequences are treated as a single 0xFF byte.
    void init(const uint8_t* data, size_t length, bool jpeg_stuffed = true) {
        data_         = data;
        length_       = length;
        byte_pos_     = 0;
        bit_pos_      = 0;
        jpeg_stuffed_ = jpeg_stuffed;
    }

    /// Read a single bit (0 or 1). Returns -1 on EOF.
    int readBit() {
        if (byte_pos_ >= length_) return -1;
        int bit = (data_[byte_pos_] >> (7 - bit_pos_)) & 1;
        advance(1);
        return bit;
    }

    /// Read up to 16 bits, MSB-first. Returns the value, or -1 on EOF.
    int readBits(int n) {
        if (n <= 0 || n > 16) return -1;
        int value = 0;
        for (int i = 0; i < n; i++) {
            int bit = readBit();
            if (bit < 0) return -1;
            value = (value << 1) | bit;
        }
        return value;
    }

    /// Read up to 16 bits without changing the current position.
    /// Copying the small reader state also preserves JPEG byte-stuffing rules.
    int peekBits(int n) const {
        BitReader lookahead = *this;
        return lookahead.readBits(n);
    }

    /// Skip `n` bits forward.
    void skipBits(int n) {
        for (int i = 0; i < n; i++) {
            advance(1);
        }
    }

    /// Align to the next byte boundary (skip remaining bits in current byte).
    void alignToByte() {
        if (bit_pos_ > 0) {
            bit_pos_ = 0;
            byte_pos_++;
            handleStuffing();
        }
    }

    /// Current byte position in the buffer.
    size_t getBytePos() const { return byte_pos_; }

    /// Current bit position within the current byte (0–7).
    uint8_t getBitPos() const { return bit_pos_; }

    /// Total bit position from start.
    size_t getTotalBitPos() const { return byte_pos_ * 8 + bit_pos_; }

    /// Check if we've reached the end of the buffer.
    bool isEOF() const { return byte_pos_ >= length_; }

    /// True if the reader is currently byte-aligned (no partial bits pending).
    bool isByteAligned() const { return bit_pos_ == 0; }

    /// Consume a JPEG restart marker (0xFFD0-0xFFD7) at the current position.
    /// Must be called only when byte-aligned (call alignToByte() first if the
    /// preceding Huffman/magnitude bits didn't end on a byte boundary --
    /// encoders always pad with 1-bits before emitting a restart marker, so
    /// this is the expected state per the JPEG spec).
    /// Returns true and advances past the 2-byte marker on success; returns
    /// false (without advancing) if no restart marker is present here, e.g.
    /// because the file didn't actually use one at this MCU count or the
    /// stream is corrupt.
    bool consumeRestartMarker() {
        if (bit_pos_ != 0) return false;
        if (byte_pos_ + 1 >= length_) return false;
        if (data_[byte_pos_] != 0xFF) return false;
        uint8_t m = data_[byte_pos_ + 1];
        if (m < 0xD0 || m > 0xD7) return false;
        byte_pos_ += 2;
        return true;
    }

    /// Peek at current byte without advancing.
    uint8_t peekByte() const {
        if (byte_pos_ >= length_) return 0;
        return data_[byte_pos_];
    }

    /// Get remaining bytes.
    size_t remainingBytes() const {
        return (byte_pos_ < length_) ? (length_ - byte_pos_) : 0;
    }

    /// Get pointer to current position in buffer.
    const uint8_t* currentPtr() const {
        return data_ + byte_pos_;
    }

private:
    void advance(int bits) {
        bit_pos_ += bits;
        while (bit_pos_ >= 8) {
            bit_pos_ -= 8;
            byte_pos_++;
            handleStuffing();
        }
    }

    void handleStuffing() {
        // In JPEG scan data, 0xFF is followed by 0x00 (byte stuffing).
        // Skip the 0x00 to get the actual data byte.
        if (jpeg_stuffed_ && byte_pos_ < length_ && byte_pos_ > 0) {
            if (data_[byte_pos_ - 1] == 0xFF && data_[byte_pos_] == 0x00) {
                byte_pos_++;
            }
        }
    }

    const uint8_t* data_;
    size_t         length_;
    size_t         byte_pos_;
    uint8_t        bit_pos_;        // 0 = MSB, 7 = LSB
    bool           jpeg_stuffed_;
};

// ─────────────────────────────────────────────────────────────────────
// BitWriter — MSB-first bit writer for constructing Huffman bitstreams
// ─────────────────────────────────────────────────────────────────────
class BitWriter {
public:
    BitWriter() : buffer_(nullptr), capacity_(0), byte_pos_(0), bit_pos_(0) {}

    /// Initialize with an output buffer.
    void init(uint8_t* buffer, size_t capacity) {
        buffer_   = buffer;
        capacity_ = capacity;
        byte_pos_ = 0;
        bit_pos_  = 0;
        if (capacity_ > 0) {
            memset(buffer_, 0, capacity_);
        }
    }

    /// Write a single bit (0 or 1). Returns false on overflow.
    bool writeBit(int bit) {
        if (byte_pos_ >= capacity_) return false;
        if (bit) {
            buffer_[byte_pos_] |= (1 << (7 - bit_pos_));
        }
        bit_pos_++;
        if (bit_pos_ >= 8) {
            bit_pos_ = 0;
            byte_pos_++;
            if (byte_pos_ < capacity_) {
                buffer_[byte_pos_] = 0;
            }
        }
        return true;
    }

    /// Write `n` bits from `value` (MSB-first). Returns false on overflow.
    bool writeBits(uint16_t value, int n) {
        for (int i = n - 1; i >= 0; i--) {
            if (!writeBit((value >> i) & 1)) return false;
        }
        return true;
    }

    /// Write raw bytes (byte-aligned copy). Caller must ensure byte alignment.
    bool writeBytes(const uint8_t* src, size_t count) {
        if (bit_pos_ != 0) return false;  // Must be byte-aligned
        if (byte_pos_ + count > capacity_) return false;
        memcpy(buffer_ + byte_pos_, src, count);
        byte_pos_ += count;
        return true;
    }

    /// Pad remaining bits in the current byte with 1s (JPEG convention) and advance to next byte.
    void padToByteJPEG() {
        while (bit_pos_ != 0) {
            writeBit(1);  // JPEG pads with 1-bits
        }
    }

    /// Current byte position.
    size_t getBytePos() const { return byte_pos_; }

    /// Current bit position within byte (0–7).
    uint8_t getBitPos() const { return bit_pos_; }

    /// Total bytes written (rounds up if mid-byte).
    size_t getBytesWritten() const {
        return byte_pos_ + (bit_pos_ > 0 ? 1 : 0);
    }

    /// Total bits written.
    size_t getTotalBitsWritten() const {
        return byte_pos_ * 8 + bit_pos_;
    }

    /// Remaining capacity in bytes (approximate).
    size_t remainingCapacity() const {
        return (byte_pos_ < capacity_) ? (capacity_ - byte_pos_) : 0;
    }

    /// Get pointer to buffer start.
    uint8_t* getBuffer() { return buffer_; }
    const uint8_t* getBuffer() const { return buffer_; }

private:
    uint8_t* buffer_;
    size_t   capacity_;
    size_t   byte_pos_;
    uint8_t  bit_pos_;    // 0 = MSB, 7 = LSB
};

} // namespace helium_jpeg
