/// @file replayer.cpp
/// @brief Sequentially replays a .bin recorded by recorder.cpp, republishing
///        each record on its original topic at its original relative timing.
///
/// Usage: ./replayer <path/to/recording.bin>
///
/// Design:
///  - The whole file is mmap()'d read-only (per the earlier write()-to-
///    record / mmap()-to-replay decision): replay is read-heavy, random-ish
///    access into a file of known size, exactly where mmap shines --
///    "advancing" through the file is just pointer arithmetic, no read()
///    syscalls per record.
///  - Only topics actually present in the file's metadata table are
///    advertised -- symmetric with the Recorder only ever recording
///    topics enabled in logger_config.json.
///  - Sequential replay only (no seeking) -- matches the current
///    requirement. Each record's payload length comes from the topic
///    metadata table (fixed wire_size per topic_id), so no length needs to
///    be stored per-record and no index file is needed to advance through
///    the file.
///  - Payloads are decoded field-by-field via payload_codec.hpp (not a raw
///    memcpy reinterpret) so a file recorded on one CPU architecture (e.g.
///    ARM) replays correctly on another (e.g. x86).
///  - Timing uses absolute offsets (replay_start + (stamp_ns - t0)), not
///    cumulative sleep-of-delta, to avoid drift accumulating over a long
///    recording -- same reasoning as the original mmap-vs-write() design
///    discussion.

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "i2w/impl.hpp"
#include "chrono_cap/chrono_cap.hpp"

using logger_msgs::Axis;
using logger_msgs::Buttons;
using logger_msgs::Pose2D;
using logger_msgs::TopicId;
using logger_msgs::TopicTraits;
using logger_wire::FileHeader;
using logger_wire::RecordHeader;
using logger_wire::TopicMetaEntry;

namespace
{

/// @brief Runtime record of one topic present in the opened file, resolved
/// against the compile-time registry so we know its wire_size without
/// needing to store a length per-record.
struct ResolvedTopic final
{
    std::uint16_t topic_id{0};
    std::uint32_t wire_size{0};
    std::string name;
};

/// @brief Sleeps until an absolute monotonic-clock deadline. Using
/// TIMER_ABSTIME (rather than a relative sleep computed fresh each time)
/// is what prevents drift from accumulating over a long recording -- each
/// target is computed from a fixed t0, not from "now + last delta".
void SleepUntilAbsoluteNs(std::uint64_t target_monotonic_ns)
{
    timespec ts;
    ts.tv_sec = static_cast<time_t>(target_monotonic_ns / 1'000'000'000ULL);
    ts.tv_nsec = static_cast<long>(target_monotonic_ns % 1'000'000'000ULL);

    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR)
    {
        // Interrupted by a signal -- retry with the same absolute deadline.
    }
}

std::uint64_t MonotonicNowNs()
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
           static_cast<std::uint64_t>(ts.tv_nsec);
}

} // namespace

// ---------------------------------------------------------------------------
// MappedRecording -- owns the mmap()'d file and parsed header/metadata
// ---------------------------------------------------------------------------

/// @brief Opens and mmap()s a recorded .bin, parses the FileHeader and
/// topic metadata table, and exposes the byte range where records begin.
class MappedRecording final
{
public:
    ~MappedRecording() { Close(); }

    /// @brief Opens, mmaps, and validates @p path.
    /// @return false on any I/O or format error (bad magic, unsupported
    ///         version, or a topic whose recorded wire_size doesn't match
    ///         what this build's registry expects for that topic_id).
    bool Open(const std::string& path)
    {
        const int fd = ::open(path.c_str(), O_RDONLY);
        if (fd < 0)
        {
            std::fprintf(stderr, "[replayer] failed to open %s: %s\n", path.c_str(),
                        std::strerror(errno));
            return false;
        }

        struct stat st{};
        if (::fstat(fd, &st) != 0)
        {
            std::fprintf(stderr, "[replayer] fstat failed: %s\n", std::strerror(errno));
            ::close(fd);
            return false;
        }
        file_size_ = static_cast<std::size_t>(st.st_size);

        void* mapped = ::mmap(nullptr, file_size_, PROT_READ, MAP_PRIVATE, fd, 0);
        ::close(fd); // fd not needed after mmap; the mapping persists independently
        if (mapped == MAP_FAILED)
        {
            std::fprintf(stderr, "[replayer] mmap failed: %s\n", std::strerror(errno));
            return false;
        }
        base_ = static_cast<const std::uint8_t*>(mapped);

        return ParseHeaderAndMetadata();
    }

    void Close()
    {
        if (base_ != nullptr)
        {
            ::munmap(const_cast<std::uint8_t*>(base_), file_size_);
            base_ = nullptr;
        }
    }

    const std::vector<ResolvedTopic>& Topics() const { return topics_; }

    /// @brief Byte offset where the first record begins (right after the
    /// file header and topic metadata table).
    std::size_t RecordsStartOffset() const { return records_start_offset_; }

    std::size_t FileSize() const { return file_size_; }
    const std::uint8_t* Base() const { return base_; }

    /// @brief Look up a resolved topic's wire_size by topic_id.
    /// @return nullptr if topic_id is unknown (not in this file's metadata
    ///         table) -- caller should treat this as a corrupt file.
    const ResolvedTopic* Find(std::uint16_t topic_id) const
    {
        auto it = by_id_.find(topic_id);
        return it == by_id_.end() ? nullptr : &it->second;
    }

private:
    /// @brief Cross-checks one file-provided TopicMetaEntry against this
    /// build's compile-time registry, if the topic_id is a type we know
    /// about at compile time.
    static bool CrossCheckKnownTopic(const TopicMetaEntry& entry)
    {
        const auto id = static_cast<TopicId>(entry.topic_id);
        std::uint32_t expected = 0;
        switch (id)
        {
            case TopicId::Pose:    expected = static_cast<std::uint32_t>(TopicTraits<Pose2D>::wire_size); break;
            case TopicId::Axis:    expected = static_cast<std::uint32_t>(TopicTraits<Axis>::wire_size); break;
            case TopicId::Buttons: expected = static_cast<std::uint32_t>(TopicTraits<Buttons>::wire_size); break;
            default:
                std::fprintf(stderr, "[replayer] unknown topic_id=%u in file -- skipping\n",
                            entry.topic_id);
                return false;
        }
        if (expected != entry.struct_size)
        {
            std::fprintf(stderr,
                        "[replayer] FATAL: topic '%s' wire_size mismatch "
                        "(file=%u, this build's registry=%u) -- refusing to replay\n",
                        entry.name, entry.struct_size, expected);
            return false;
        }
        return true;
    }

    bool ParseHeaderAndMetadata()
    {
        if (file_size_ < logger_wire::kFileHeaderWireSize)
        {
            std::fprintf(stderr, "[replayer] file too small to contain a header\n");
            return false;
        }

        FileHeader header;
        if (!FileHeader::Deserialize(base_, header))
        {
            std::fprintf(stderr, "[replayer] bad magic bytes -- not a recognized log file\n");
            return false;
        }
        if (header.version != logger_wire::kFormatVersion)
        {
            std::fprintf(stderr, "[replayer] unsupported format version %u (expected %u)\n",
                        header.version, logger_wire::kFormatVersion);
            return false;
        }

        std::size_t offset = logger_wire::kFileHeaderWireSize;
        for (std::uint32_t i = 0; i < header.topic_count; ++i)
        {
            if (offset + logger_wire::kTopicMetaWireSize > file_size_)
            {
                std::fprintf(stderr, "[replayer] truncated topic metadata table\n");
                return false;
            }

            TopicMetaEntry entry;
            TopicMetaEntry::Deserialize(base_ + offset, entry);
            offset += logger_wire::kTopicMetaWireSize;

            if (!CrossCheckKnownTopic(entry))
            {
                return false;
            }

            ResolvedTopic resolved;
            resolved.topic_id = entry.topic_id;
            resolved.wire_size = entry.struct_size;
            resolved.name = std::string(entry.name);
            topics_.push_back(resolved);
            by_id_[resolved.topic_id] = resolved;

            std::printf("[replayer] topic in file: id=%u name=%s wire_size=%u\n",
                        resolved.topic_id, resolved.name.c_str(), resolved.wire_size);
        }

        records_start_offset_ = offset;
        return true;
    }

    const std::uint8_t* base_{nullptr};
    std::size_t file_size_{0};
    std::size_t records_start_offset_{0};
    std::vector<ResolvedTopic> topics_;
    std::unordered_map<std::uint16_t, ResolvedTopic> by_id_;
};

// ---------------------------------------------------------------------------
// ReplayerSystem -- advertises only the topics present in the file
// ---------------------------------------------------------------------------

/// @brief Advertises publishers for whichever topics are present in the
/// opened recording (a topic absent from the file is never advertised),
/// then walks the mmap'd records sequentially, republishing each one at
/// its original relative timing.
class ReplayerSystem final : public i2w::SystemBase
{
public:
    ReplayerSystem(i2w::Config config, MappedRecording& recording)
        : i2w::SystemBase(std::move(config)), recording_(recording)
    {
    }

    /// @brief Runs the full sequential replay to completion. Call after
    /// Setup() succeeds.
    void Run()
    {
        const std::uint8_t* base = recording_.Base();
        std::size_t offset = recording_.RecordsStartOffset();
        const std::size_t end = recording_.FileSize();

        bool have_t0 = false;
        std::uint64_t t0_ns = 0;
        std::uint64_t replay_start_monotonic_ns = 0;

        std::uint64_t records_played = 0;

        while (offset + logger_wire::kRecordHeaderWireSize <= end)
        {
            RecordHeader hdr;
            RecordHeader::Deserialize(base + offset, hdr);

            const ResolvedTopic* topic = recording_.Find(hdr.topic_id);
            if (topic == nullptr)
            {
                std::fprintf(stderr, "[replayer] unknown topic_id=%u mid-file -- stopping\n",
                            hdr.topic_id);
                break;
            }

            const std::size_t payload_offset = offset + logger_wire::kRecordHeaderWireSize;
            if (payload_offset + topic->wire_size > end)
            {
                std::fprintf(stderr, "[replayer] truncated record at offset %zu -- stopping\n",
                            offset);
                break;
            }
            const std::uint8_t* payload_ptr = base + payload_offset;

            if (!have_t0)
            {
                t0_ns = hdr.stamp_ns;
                replay_start_monotonic_ns = MonotonicNowNs();
                have_t0 = true;
            }

            const std::uint64_t elapsed_ns = hdr.stamp_ns - t0_ns; // same-arch stamps, always forward
            const std::uint64_t target_ns = replay_start_monotonic_ns + elapsed_ns;
            SleepUntilAbsoluteNs(target_ns);

            Dispatch(static_cast<TopicId>(hdr.topic_id), hdr, payload_ptr);
            ++records_played;

            offset = payload_offset + topic->wire_size;
        }

        std::printf("[replayer] done -- %llu record(s) replayed\n",
                    static_cast<unsigned long long>(records_played));
    }

private:
    i2w::LifecycleResult OnSetup() noexcept override
    {
        i2w::PublisherOptions opts;
        opts.plane = plane_;

        for (const auto& topic : recording_.Topics())
        {
            switch (static_cast<TopicId>(topic.topic_id))
            {
                case TopicId::Pose:
                {
                    auto pub = runtime().advertise<Pose2D>(TopicTraits<Pose2D>::name, opts);
                    if (!pub) return i2w::Fail();
                    pose_pub_ = std::move(pub.value());
                    break;
                }
                case TopicId::Axis:
                {
                    auto pub = runtime().advertise<Axis>(TopicTraits<Axis>::name, opts);
                    if (!pub) return i2w::Fail();
                    axis_pub_ = std::move(pub.value());
                    break;
                }
                case TopicId::Buttons:
                {
                    auto pub = runtime().advertise<Buttons>(TopicTraits<Buttons>::name, opts);
                    if (!pub) return i2w::Fail();
                    buttons_pub_ = std::move(pub.value());
                    break;
                }
                default:
                    return i2w::Fail();
            }
        }
        return i2w::Ok();
    }

    i2w::LifecycleResult OnTick() noexcept override { return i2w::Ok(); }

    /// @brief Decodes @p payload_ptr according to @p id and republishes it
    /// with the original stamp/seq preserved.
    void Dispatch(TopicId id, const RecordHeader& hdr, const std::uint8_t* payload_ptr)
    {
        switch (id)
        {
            case TopicId::Pose:
            {
                Pose2D v{};
                logger_wire::DecodePayload(payload_ptr, v);
                pose_pub_.publish(v, hdr.stamp_ns);
                break;
            }
            case TopicId::Axis:
            {
                Axis v{};
                logger_wire::DecodePayload(payload_ptr, v);
                axis_pub_.publish(v, hdr.stamp_ns);
                break;
            }
            case TopicId::Buttons:
            {
                Buttons v{};
                logger_wire::DecodePayload(payload_ptr, v);
                buttons_pub_.publish(v, hdr.stamp_ns);
                break;
            }
        }
    }

    MappedRecording& recording_;
    i2w::Publisher<Pose2D> pose_pub_{};
    i2w::Publisher<Axis> axis_pub_{};
    i2w::Publisher<Buttons> buttons_pub_{};
    i2w::EndpointPlane plane_{i2w::EndpointPlane::Local};
};

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::fprintf(stderr, "usage: %s <path/to/recording.bin>\n", argv[0]);
        return 1;
    }

    MappedRecording recording;
    if (!recording.Open(argv[1]))
    {
        return 1;
    }
    if (recording.Topics().empty())
    {
        std::printf("[replayer] file contains no topics -- nothing to replay\n");
        return 0;
    }

    i2w::Config config;
    config.node_name = "replayer";
    config.ns = "/demo";

    ReplayerSystem system(std::move(config), recording);
    if (!system.Setup().ok)
    {
        std::printf("[replayer] setup failed\n");
        return 1;
    }

    system.Run();
    return 0;
}