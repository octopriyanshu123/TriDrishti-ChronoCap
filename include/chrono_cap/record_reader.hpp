#pragma once
/// @file record_reader.hpp
/// @brief Generic, schema-agnostic engine for sequentially replaying a .bin
///        recorded by RecordWriter, at original relative timing.
///
/// This class knows NOTHING about Pose2D/Axis/Buttons or i2w -- it mmap()s
/// the file, parses the header/topic metadata table, and walks records
/// sequentially, invoking a caller-supplied callback with
/// (topic_id, stamp_ns, seq, raw payload bytes, len) at the right wall-clock
/// moment. Decoding those bytes into a real struct and republishing them is
/// entirely up to the caller (e.g. i2wReplayer.cpp).

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace chrono_cap
{

/// @brief One topic present in an opened recording, resolved from its
/// on-disk metadata entry.
struct ResolvedTopic final
{
    std::uint16_t topic_id{0};
    std::uint32_t wire_size{0};
    std::string name;
};

/// @brief Invoked once per record, in original recorded order, at the
/// wall-clock moment corresponding to its original relative timing.
using RecordCallback = std::function<void(std::uint16_t topic_id, std::uint64_t stamp_ns,
                                           std::uint64_t seq, const std::uint8_t* payload,
                                           std::uint32_t payload_len)>;

/// @brief Optional hook: called once per topic found in the file's
/// metadata table, before replay starts, so the caller can validate/
/// prepare (e.g. advertise a publisher) for exactly those topics.
using TopicHook = std::function<bool(const ResolvedTopic&)>;

/// @brief mmap-based reader/replayer for a .bin written by RecordWriter.
///
/// Usage:
///   RecordReader reader;
///   reader.Open("recording_20260908_153012.bin", [](const ResolvedTopic& t) {
///       // advertise a publisher for t.topic_id, return false to reject
///       return true;
///   });
///   reader.Run([](topic_id, stamp_ns, seq, payload, len) {
///       // decode payload according to topic_id, republish
///   });
class RecordReader final
{
public:
    ~RecordReader();

    /// @brief Opens and mmap()s @p path, parses+validates the header and
    /// topic metadata table. Calls @p on_topic once per topic found (if
    /// provided) so the caller can prepare for exactly those topics before
    /// replay starts.
    /// @return false on I/O error, bad magic, unsupported version, or if
    ///         @p on_topic returns false for any topic.
    bool Open(const std::string& path, const TopicHook& on_topic = nullptr);

    /// @brief Runs the full sequential replay to completion, invoking
    /// @p callback for each record at the wall-clock moment matching its
    /// original relative timing (absolute-offset sleeping -- no drift over
    /// a long file).
    void Run(const RecordCallback& callback);

    const std::vector<ResolvedTopic>& Topics() const { return topics_; }

private:
    bool ParseHeaderAndMetadata();

    const std::uint8_t* base_{nullptr};
    std::size_t file_size_{0};
    std::size_t records_start_offset_{0};
    std::vector<ResolvedTopic> topics_;
    std::unordered_map<std::uint16_t, ResolvedTopic> by_id_;
};

} // namespace chrono_cap
