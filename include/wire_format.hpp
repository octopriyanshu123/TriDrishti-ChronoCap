#pragma once
/// @file wire_format.hpp
/// @brief On-disk binary format definitions for the logger recorder/replayer.
///
/// All multi-byte integers are serialized in little-endian byte order,
/// regardless of host CPU endianness, so a recording made on one
/// architecture (e.g. ARM) can be replayed correctly on another (e.g. x86).
/// Do NOT memcpy() these structs directly to/from disk -- always go through
/// the Serialize()/Deserialize() helpers below, which handle byte order
/// explicitly and are immune to compiler-inserted struct padding.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace logger_wire
{

// ---------------------------------------------------------------------------
// Endian-safe primitive encode/decode helpers
// ---------------------------------------------------------------------------

/// @brief Encode a uint16_t into @p out as little-endian, host-endian-agnostic.
inline void encode_u16(std::uint8_t* out, std::uint16_t v) noexcept
{
    out[0] = static_cast<std::uint8_t>(v & 0xFF);
    out[1] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
}

/// @brief Decode a little-endian uint16_t from @p in.
inline std::uint16_t decode_u16(const std::uint8_t* in) noexcept
{
    return static_cast<std::uint16_t>(in[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(in[1]) << 8);
}

/// @brief Encode a uint32_t into @p out as little-endian.
inline void encode_u32(std::uint8_t* out, std::uint32_t v) noexcept
{
    out[0] = static_cast<std::uint8_t>(v & 0xFF);
    out[1] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
    out[2] = static_cast<std::uint8_t>((v >> 16) & 0xFF);
    out[3] = static_cast<std::uint8_t>((v >> 24) & 0xFF);
}

/// @brief Decode a little-endian uint32_t from @p in.
inline std::uint32_t decode_u32(const std::uint8_t* in) noexcept
{
    return static_cast<std::uint32_t>(in[0]) |
           (static_cast<std::uint32_t>(in[1]) << 8) |
           (static_cast<std::uint32_t>(in[2]) << 16) |
           (static_cast<std::uint32_t>(in[3]) << 24);
}

/// @brief Encode a uint64_t into @p out as little-endian.
inline void encode_u64(std::uint8_t* out, std::uint64_t v) noexcept
{
    for (int i = 0; i < 8; ++i)
    {
        out[i] = static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF);
    }
}

/// @brief Decode a little-endian uint64_t from @p in.
inline std::uint64_t decode_u64(const std::uint8_t* in) noexcept
{
    std::uint64_t v = 0;
    for (int i = 0; i < 8; ++i)
    {
        v |= static_cast<std::uint64_t>(in[i]) << (8 * i);
    }
    return v;
}

// ---------------------------------------------------------------------------
// File format constants
// ---------------------------------------------------------------------------

/// Magic bytes at the start of every recording file. Lets any tool quickly
/// verify "is this actually one of our logs" before parsing further.
constexpr char kMagic[8] = {'I', '2', 'W', 'L', 'O', 'G', '\0', '\0'};

/// Bump whenever the on-disk layout changes in a backward-incompatible way.
/// Readers must check this before parsing further.
constexpr std::uint32_t kFormatVersion = 1;

/// Wire size (bytes) of the serialized FileHeader. Fixed, independent of
/// struct padding on any particular compiler/platform.
constexpr std::size_t kFileHeaderWireSize =
    8 /*magic*/ + 4 /*version*/ + 4 /*topic_count*/ + 8 /*created_at_ns*/ + 8 /*reserved*/;

/// Wire size (bytes) of one serialized TopicMetaEntry.
constexpr std::size_t kTopicMetaWireSize =
    2 /*topic_id*/ + 4 /*struct_size*/ + 32 /*name*/;

/// Wire size (bytes) of one serialized record header (excludes payload).
constexpr std::size_t kRecordHeaderWireSize =
    2 /*topic_id*/ + 8 /*stamp_ns*/ + 8 /*seq*/;

/// Wire size (bytes) of one serialized index entry.
constexpr std::size_t kIndexEntryWireSize =
    8 /*stamp_ns*/ + 8 /*byte_offset*/ + 2 /*topic_id*/;

/// Maximum payload size (bytes) reserved per ring-buffer slot / record.
/// Must be >= the largest struct size among all registered topics. Sized
/// with headroom so a modest new topic later doesn't require touching the
/// ring buffer slot layout.
constexpr std::size_t kMaxPayloadBytes = 64;

// ---------------------------------------------------------------------------
// In-memory representations. NOT safe to memcpy to/from disk directly --
// always go through Serialize()/Deserialize().
// ---------------------------------------------------------------------------

/// @brief Fixed-size header written once at the start of every .bin file.
struct FileHeader final
{
    std::uint32_t version{kFormatVersion};
    std::uint32_t topic_count{0};
    std::uint64_t created_at_ns{0};

    /// @brief Serialize into a caller-owned buffer of at least kFileHeaderWireSize bytes.
    void Serialize(std::uint8_t* out) const noexcept
    {
        std::memcpy(out, kMagic, 8);
        encode_u32(out + 8, version);
        encode_u32(out + 12, topic_count);
        encode_u64(out + 16, created_at_ns);
        std::memset(out + 24, 0, 8); // reserved, zero-filled for future fields
    }

    /// @return false if the magic bytes don't match (not one of our files).
    static bool Deserialize(const std::uint8_t* in, FileHeader& out) noexcept
    {
        if (std::memcmp(in, kMagic, 8) != 0)
        {
            return false;
        }
        out.version = decode_u32(in + 8);
        out.topic_count = decode_u32(in + 12);
        out.created_at_ns = decode_u64(in + 16);
        return true;
    }
};

/// @brief One entry in the topic metadata table -- describes a single topic
/// that was enabled for this particular recording session. Only enabled
/// topics appear here; a disabled topic has no entry and never appears in
/// any record in the file.
struct TopicMetaEntry final
{
    std::uint16_t topic_id{0};
    std::uint32_t struct_size{0};
    char name[32]{};

    void Serialize(std::uint8_t* out) const noexcept
    {
        encode_u16(out, topic_id);
        encode_u32(out + 2, struct_size);
        std::memset(out + 6, 0, 32);
        const std::size_t len = std::strlen(name);
        std::memcpy(out + 6, name, len < 32 ? len : 31);
    }

    static void Deserialize(const std::uint8_t* in, TopicMetaEntry& out) noexcept
    {
        out.topic_id = decode_u16(in);
        out.struct_size = decode_u32(in + 2);
        std::memset(out.name, 0, sizeof(out.name));
        std::memcpy(out.name, in + 6, 32);
    }
};

/// @brief Fixed-size header prefixed to every record's payload in the .bin.
/// No length field is stored here on purpose -- the payload length for a
/// given topic_id is fixed and already known from that topic's
/// TopicMetaEntry::struct_size, so it isn't repeated on every single record.
struct RecordHeader final
{
    std::uint16_t topic_id{0};
    std::uint64_t stamp_ns{0}; ///< original publish timestamp (sample.header.stamp_ns)
    std::uint64_t seq{0};

    void Serialize(std::uint8_t* out) const noexcept
    {
        encode_u16(out, topic_id);
        encode_u64(out + 2, stamp_ns);
        encode_u64(out + 10, seq);
    }

    static void Deserialize(const std::uint8_t* in, RecordHeader& out) noexcept
    {
        out.topic_id = decode_u16(in);
        out.stamp_ns = decode_u64(in + 2);
        out.seq = decode_u64(in + 10);
    }
};

/// @brief One entry in the companion .idx file: maps a record's timestamp to
/// the byte offset (into the .bin) where that record's RecordHeader begins,
/// enabling the Replayer to seek to an arbitrary point in time without a
/// full linear scan.
struct IndexEntry final
{
    std::uint64_t stamp_ns{0};
    std::uint64_t byte_offset{0};
    std::uint16_t topic_id{0};

    void Serialize(std::uint8_t* out) const noexcept
    {
        encode_u64(out, stamp_ns);
        encode_u64(out + 8, byte_offset);
        encode_u16(out + 16, topic_id);
    }

    static void Deserialize(const std::uint8_t* in, IndexEntry& out) noexcept
    {
        out.stamp_ns = decode_u64(in);
        out.byte_offset = decode_u64(in + 8);
        out.topic_id = decode_u16(in + 16);
    }
};

} // namespace logger_wire