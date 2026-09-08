#pragma once
/// @file payload_codec.hpp
/// @brief Field-by-field, endian-safe encode/decode for each message payload.
///
/// This is what actually closes the ARM<->x86 portability gap: a raw
/// memcpy() of a struct is NOT portable across architectures (byte order of
/// multi-byte fields, and potentially struct padding/alignment differ). Every
/// field here is encoded individually via the little-endian helpers in
/// wire_format.hpp, so the resulting byte layout is identical and correctly
/// interpretable regardless of which architecture wrote or reads it.
///
/// float fields (Pose2D::x/y/yaw) are reinterpreted as their raw IEEE-754
/// bit pattern (via memcpy into a uint32_t -- NOT a numeric cast) and that
/// bit pattern is then encoded as a normal little-endian uint32_t. This is
/// safe because ARM and x86 both use IEEE-754 for `float`; only the *byte
/// order* of those bits needs normalizing, which encode_u32/decode_u32
/// already handle.

#include <cstdint>
#include <cstring>

#include "logger_types.hpp"
#include "chrono_cap/wire_format.hpp"

namespace logger_wire
{

// ---------------------------------------------------------------------------
// Pose2D  (12 bytes on the wire: x, y, yaw as little-endian float bit patterns)
// ---------------------------------------------------------------------------

inline void EncodePayload(const logger_msgs::Pose2D& v, std::uint8_t* out) noexcept
{
    std::uint32_t bits;
    std::memcpy(&bits, &v.x, sizeof(bits));
    encode_u32(out + 0, bits);
    std::memcpy(&bits, &v.y, sizeof(bits));
    encode_u32(out + 4, bits);
    std::memcpy(&bits, &v.yaw, sizeof(bits));
    encode_u32(out + 8, bits);
}

inline void DecodePayload(const std::uint8_t* in, logger_msgs::Pose2D& v) noexcept
{
    std::uint32_t bits;
    bits = decode_u32(in + 0);
    std::memcpy(&v.x, &bits, sizeof(bits));
    bits = decode_u32(in + 4);
    std::memcpy(&v.y, &bits, sizeof(bits));
    bits = decode_u32(in + 8);
    std::memcpy(&v.yaw, &bits, sizeof(bits));
}

// ---------------------------------------------------------------------------
// Axis  (20 bytes: sequence u64, timestamp_ns u64, axes_count i32)
// ---------------------------------------------------------------------------

inline void EncodePayload(const logger_msgs::Axis& v, std::uint8_t* out) noexcept
{
    encode_u64(out + 0, v.sequence);
    encode_u64(out + 8, v.timestamp_ns);
    encode_u32(out + 16, static_cast<std::uint32_t>(v.axes_count));
}

inline void DecodePayload(const std::uint8_t* in, logger_msgs::Axis& v) noexcept
{
    v.sequence = decode_u64(in + 0);
    v.timestamp_ns = decode_u64(in + 8);
    v.axes_count = static_cast<int>(decode_u32(in + 16));
}

// ---------------------------------------------------------------------------
// Buttons  (20 bytes: sequence u64, timestamp_ns u64, buttons_count i32)
// ---------------------------------------------------------------------------

inline void EncodePayload(const logger_msgs::Buttons& v, std::uint8_t* out) noexcept
{
    encode_u64(out + 0, v.sequence);
    encode_u64(out + 8, v.timestamp_ns);
    encode_u32(out + 16, static_cast<std::uint32_t>(v.buttons_count));
}

inline void DecodePayload(const std::uint8_t* in, logger_msgs::Buttons& v) noexcept
{
    v.sequence = decode_u64(in + 0);
    v.timestamp_ns = decode_u64(in + 8);
    v.buttons_count = static_cast<int>(decode_u32(in + 16));
}





} // namespace logger_wire
