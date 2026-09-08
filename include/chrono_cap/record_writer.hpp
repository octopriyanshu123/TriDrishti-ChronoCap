#pragma once
/// @file record_writer.hpp
/// @brief Generic, schema-agnostic engine for recording typed byte records
///        to a timestamped, self-describing .bin file.
///
/// This class knows NOTHING about Pose2D/Axis/Buttons or i2w -- it only
/// deals in (topic_id, stamp_ns, seq, raw bytes). Callers register topic
/// metadata (name + fixed wire_size) up front, then call Push() from any
/// number of producer threads. A dedicated internal writer thread drains a
/// lock-free MPSC ring buffer and performs all disk I/O, so Push() never
/// blocks on disk.
///
/// This is the reusable "library" half of the logger: an application-
/// specific integration (e.g. i2wRecorder.cpp) supplies the topic
/// metadata and encoded payload bytes; this class handles persistence.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "chrono_cap/mpsc_ring_buffer.hpp"
#include "chrono_cap/wire_format.hpp"

namespace chrono_cap
{

/// @brief Metadata describing one topic to be recorded: its numeric id,
/// fixed encoded payload size, and display name. Supplied by the caller --
/// this library has no compile-time knowledge of any specific message type.
struct TopicInfo final
{
    std::uint16_t topic_id{0};
    std::uint32_t wire_size{0};
    std::string name;
};

/// @brief Records typed byte payloads to a single timestamped .bin file.
///
/// Usage:
///   RecordWriter writer;
///   writer.Open("recording_20260908_153012", {topic_info_a, topic_info_b});
///   ...
///   writer.Push(topic_id, stamp_ns, seq, encoded_bytes, encoded_len); // any thread
///   ...
///   writer.Close();
class RecordWriter final
{
public:
    /// @param flush_interval How often the writer thread fsync()s after a
    ///        drain batch. Bounds the crash-loss window to "whatever is
    ///        still unconsumed in the ring buffer", not to this interval.
    /// @param idle_poll_interval How long the writer thread sleeps when the
    ///        ring buffer is empty, before checking again.
    explicit RecordWriter(std::chrono::milliseconds flush_interval = std::chrono::milliseconds(100),
                           std::chrono::microseconds idle_poll_interval = std::chrono::microseconds(200))
        : flush_interval_(flush_interval), idle_poll_interval_(idle_poll_interval)
    {
    }

    ~RecordWriter() { Close(); }

    /// @brief Creates "<session_base>.bin", writes the file header and
    /// topic metadata table, and starts the background writer thread.
    /// @return false on any I/O failure.
    bool Open(const std::string& session_base, const std::vector<TopicInfo>& topics);

    /// @brief Wait-free from the caller's perspective: pushes one record
    /// into the internal lock-free ring buffer and returns immediately.
    /// Never touches disk. Safe to call concurrently from multiple threads.
    /// @return false if the ring buffer is full (writer thread fell behind)
    ///         or @p len exceeds the per-slot capacity -- treat as a
    ///         dropped sample; see DroppedCount().
    bool Push(std::uint16_t topic_id, std::uint64_t stamp_ns, std::uint64_t seq,
              const void* data, std::uint32_t len)
    {
        return ring_.TryPush(topic_id, stamp_ns, seq, data, len);
    }

    /// @brief Number of records dropped so far due to a full ring buffer.
    std::uint64_t DroppedCount() const { return ring_.DroppedCount(); }

    /// @brief Stops the writer thread (after draining whatever remains in
    /// the ring buffer), does a final fsync, and closes the file. Safe to
    /// call multiple times.
    void Close();

private:
    void WriterThreadMain();
    bool WriteFileHeaderAndMetadata(const std::vector<TopicInfo>& topics);
    bool WriteRecord(const logger_ring::PoppedRecord& rec);
    void MaybeFlush();
    static bool WriteAll(int fd, const void* data, std::size_t len);

    logger_ring::MpscRingBuffer<4096> ring_;
    std::thread writer_thread_;
    std::atomic<bool> shutdown_{false};

    int bin_fd_{-1};
    std::string bin_path_;
    std::uint32_t pending_since_flush_{0};
    std::chrono::steady_clock::time_point last_flush_{};
    std::chrono::milliseconds flush_interval_;
    std::chrono::microseconds idle_poll_interval_;
};

} // namespace chrono_cap
