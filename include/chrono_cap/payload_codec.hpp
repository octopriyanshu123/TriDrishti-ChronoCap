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

#include "chrono_cap/wire_format.hpp"

namespace logger_wire
{

// // ---------------------------------------------------------------------------
// // Pose2D  (12 bytes on the wire: x, y, yaw as little-endian float bit patterns)
// // ---------------------------------------------------------------------------

// inline void EncodePayload(const logger_msgs::Pose2D& v, std::uint8_t* out) noexcept
// {
//     std::uint32_t bits;
//     std::memcpy(&bits, &v.x, sizeof(bits));
//     encode_u32(out + 0, bits);
//     std::memcpy(&bits, &v.y, sizeof(bits));
//     encode_u32(out + 4, bits);
//     std::memcpy(&bits, &v.yaw, sizeof(bits));
//     encode_u32(out + 8, bits);
// }

// inline void DecodePayload(const std::uint8_t* in, logger_msgs::Pose2D& v) noexcept
// {
//     std::uint32_t bits;
//     bits = decode_u32(in + 0);
//     std::memcpy(&v.x, &bits, sizeof(bits));
//     bits = decode_u32(in + 4);
//     std::memcpy(&v.y, &bits, sizeof(bits));
//     bits = decode_u32(in + 8);
//     std::memcpy(&v.yaw, &bits, sizeof(bits));
// }

// // ---------------------------------------------------------------------------
// // Axis  (20 bytes: sequence u64, timestamp_ns u64, axes_count i32)
// // ---------------------------------------------------------------------------

// inline void EncodePayload(const logger_msgs::Axis& v, std::uint8_t* out) noexcept
// {
//     encode_u64(out + 0, v.sequence);
//     encode_u64(out + 8, v.timestamp_ns);
//     encode_u32(out + 16, static_cast<std::uint32_t>(v.axes_count));
// }

// inline void DecodePayload(const std::uint8_t* in, logger_msgs::Axis& v) noexcept
// {
//     v.sequence = decode_u64(in + 0);
//     v.timestamp_ns = decode_u64(in + 8);
//     v.axes_count = static_cast<int>(decode_u32(in + 16));
// }

// // ---------------------------------------------------------------------------
// // Buttons  (20 bytes: sequence u64, timestamp_ns u64, buttons_count i32)
// // ---------------------------------------------------------------------------

// inline void EncodePayload(const logger_msgs::Buttons& v, std::uint8_t* out) noexcept
// {
//     encode_u64(out + 0, v.sequence);
//     encode_u64(out + 8, v.timestamp_ns);
//     encode_u32(out + 16, static_cast<std::uint32_t>(v.buttons_count));
// }

// inline void DecodePayload(const std::uint8_t* in, logger_msgs::Buttons& v) noexcept
// {
//     v.sequence = decode_u64(in + 0);
//     v.timestamp_ns = decode_u64(in + 8);
//     v.buttons_count = static_cast<int>(decode_u32(in + 16));
// }


// Ui JoyMsgs  (22 bytes: timestamp u64, axis0 f32, axis2 f32, button0-6 bools)
inline void EncodePayload(const crawler_i2w_msgs::JoyMsgs& v, std::uint8_t* out) noexcept
{
    encode_u64(out + 0, v.timestamp);

    std::uint32_t bits;
    std::memcpy(&bits, &v.axis0, sizeof(bits));
    encode_u32(out + 8, bits);
    std::memcpy(&bits, &v.axis2, sizeof(bits));
    encode_u32(out + 12, bits);

    out[16] = v.button0 ? 1 : 0;
    out[17] = v.button1 ? 1 : 0;
    out[18] = v.button3 ? 1 : 0;
    out[19] = v.button4 ? 1 : 0;
    out[20] = v.button5 ? 1 : 0;
    out[21] = v.button6 ? 1 : 0;
}

inline void DecodePayload(const std::uint8_t* in, crawler_i2w_msgs::JoyMsgs& v) noexcept
{
    v.timestamp = decode_u64(in + 0);

    std::uint32_t bits;
    bits = decode_u32(in + 8);
    std::memcpy(&v.axis0, &bits, sizeof(bits));
    bits = decode_u32(in + 12);
    std::memcpy(&v.axis2, &bits, sizeof(bits));

    v.button0 = in[16] != 0;
    v.button1 = in[17] != 0;
    v.button3 = in[18] != 0;
    v.button4 = in[19] != 0;
    v.button5 = in[20] != 0;
    v.button6 = in[21] != 0;
}

// inline void EncodeMotorStatus(const crawler_i2w_msgs::MotorStatus& v, std::uint8_t* out) noexcept
// {
//     std::uint64_t bits64;
//     std::memcpy(&bits64, &v.velocity_rad_s, sizeof(bits64));
//     encode_u64(out + 0, bits64);

//     encode_u16(out + 8, static_cast<std::uint16_t>(v.fault));

//     out[10] = v.is_brake_released ? 1 : 0;

//     std::memcpy(&bits64, &v.torque_, sizeof(bits64));
//     encode_u64(out + 11, bits64);

//     std::memcpy(&bits64, &v.temperature_c, sizeof(bits64));
//     encode_u64(out + 19, bits64);

//     std::memcpy(&bits64, &v.current_a, sizeof(bits64));
//     encode_u64(out + 27, bits64);

//     out[35] = v.enabled ? 1 : 0;
// }

// inline void DecodeMotorStatus(const std::uint8_t* in, crawler_i2w_msgs::MotorStatus& v) noexcept
// {
//     std::uint64_t bits64 = decode_u64(in + 0);
//     std::memcpy(&v.velocity_rad_s, &bits64, sizeof(bits64));

//     v.fault = static_cast<crawler_i2w_msgs::ErrorCode>(decode_u16(in + 8));

//     v.is_brake_released = in[10] != 0;

//     bits64 = decode_u64(in + 11);
//     std::memcpy(&v.torque_, &bits64, sizeof(bits64));

//     bits64 = decode_u64(in + 19);
//     std::memcpy(&v.temperature_c, &bits64, sizeof(bits64));

//     bits64 = decode_u64(in + 27);
//     std::memcpy(&v.current_a, &bits64, sizeof(bits64));

//     v.enabled = in[35] != 0;
// }

// inline void EncodePayload(const crawler_i2w_msgs::MotorState& v, std::uint8_t* out) noexcept
// {
//     encode_u64(out + 0, v.timestamp_ns);
//     EncodeMotorStatus(v.left, out + 8);
//     EncodeMotorStatus(v.right, out + 8 + 36);
// }

// inline void DecodePayload(const std::uint8_t* in, crawler_i2w_msgs::MotorState& v) noexcept
// {
//     v.timestamp_ns = decode_u64(in + 0);
//     DecodeMotorStatus(in + 8, v.left);
//     DecodeMotorStatus(in + 8 + 36, v.right);
// }



} // namespace logger_wire
