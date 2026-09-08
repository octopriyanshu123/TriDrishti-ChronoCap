/// @file recorder.cpp
/// @brief Records enabled topics (Pose2D, Axis, Buttons) to a new,
///        timestamped, self-describing binary log file each session.
///
/// Data flow:
///
///   [pose subscriber cb] ─┐
///   [axis subscriber cb] ─┼─► MpscRingBuffer (lock-free) ─► BinaryWriter
///   [buttons sub. cb]    ─┘        (1 dedicated writer thread)
///
/// Subscriber callbacks never touch disk -- they only push a fixed-size
/// record into the lock-free ring buffer and return immediately. A single
/// background writer thread drains the ring buffer and performs all actual
/// file I/O (write() + periodic fsync()), so a slow/stalling disk can never
/// block or delay message delivery on the subscriber side.
///
/// On-disk layout (see wire_format.hpp for exact byte layout):
///   <session>.bin  = FileHeader + TopicMetaEntry[topic_count] + Record...
///
/// Replay is sequential start-to-end only (no random seeking requirement
/// for now), so no separate index file is written -- the Replayer walks
/// records in order via mmap, computing each record's size on the fly from
/// the topic metadata table (fixed wire_size per topic_id), which is enough
/// to advance the read pointer without needing a precomputed byte offset
/// table. If arbitrary-timestamp seeking becomes a requirement later, an
/// index can be generated after the fact from an existing .bin, without
/// needing to change this format.
///
/// Payloads are encoded field-by-field via payload_codec.hpp (NOT raw
/// memcpy of the struct) so recordings made on one CPU architecture (e.g.
/// ARM) replay correctly on another (e.g. x86) -- struct padding and
/// multi-byte field endianness are normalized on the wire.
///
/// Only topics enabled in logger_config.json get a TopicMetaEntry and are
/// ever subscribed to -- a disabled topic produces zero records and never
/// appears anywhere in the file.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include "i2w/impl.hpp"
#include "logger_config.hpp"
#include "mpsc_ring_buffer.hpp"
#include "payload_codec.hpp"
#include "topic_registry.hpp"
#include "wire_format.hpp"

using logger_msgs::Axis;
using logger_msgs::Buttons;
using logger_msgs::Pose2D;
using logger_msgs::TopicTraits;
using logger_ring::MpscRingBuffer;
using logger_ring::PoppedRecord;
using logger_wire::FileHeader;
using logger_wire::RecordHeader;
using logger_wire::TopicMetaEntry;

namespace
{

/// Ring buffer capacity -- must be a power of two. Sized generously above
/// worst-case combined producer rate (10Hz pose + 100Hz axis + 20Hz buttons
/// = ~130 msgs/sec) so a brief writer-thread stall doesn't drop samples.
constexpr std::size_t kRingCapacity = 4096;

/// How often the writer thread calls fsync() after draining available
/// records. Keeping this short bounds the crash-loss window to roughly
/// "whatever's still unwritten in the ring buffer since the last drain",
/// not to the flush interval itself.
constexpr std::chrono::milliseconds kFlushInterval{100};

/// If the ring buffer is empty, how long the writer thread sleeps before
/// checking again. Small enough to keep index/data reasonably fresh,
/// large enough not to busy-spin and burn a core.
constexpr std::chrono::microseconds kIdlePollInterval{200};

MpscRingBuffer<kRingCapacity> g_ring_buffer;
logger_cfg::LoggerConfig g_logger_config;
std::atomic<bool> g_shutdown{false};

/// @brief Returns current wall-clock time in nanoseconds since epoch.
/// Used for the file header's created_at_ns and for building the
/// session's filename.
std::uint64_t WallClockNs() noexcept
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

/// @brief Builds a session filename base (no extension) from the current
/// wall-clock time, e.g. "recording_20260908_153012".
std::string MakeSessionBaseName()
{
    const std::time_t t = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);

    char buf[64];
    std::snprintf(buf, sizeof(buf), "recording_%04d%02d%02d_%02d%02d%02d",
                  tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                  tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);
    return std::string(buf);
}

} // namespace

// ---------------------------------------------------------------------------
// BinaryWriter
// ---------------------------------------------------------------------------

/// @brief Owns the .bin file for one recording session: writes the file
/// header + topic metadata table once at Open(), then appends records as
/// they're drained from the ring buffer.
///
/// No separate index file -- replay is sequential start-to-end only, so
/// the Replayer can compute each record's size purely from the topic
/// metadata table (fixed wire_size per topic_id) while walking forward.
///
/// Not thread-safe by design -- intended to be driven exclusively by the
/// single writer thread in main(), which is also the only consumer of the
/// MpscRingBuffer. This keeps the hot write path lock-free end to end.
class BinaryWriter final
{
public:
    ~BinaryWriter() { Close(); }

    /// @brief Creates a new .bin and writes the fixed file header and
    /// topic metadata table.
    /// @param session_base Filename base without extension, e.g.
    ///        "recording_20260908_153012". A new BinaryWriter (and hence a
    ///        new file) is created for every recording session.
    /// @param topics Metadata for only the topics enabled this session.
    /// @return false on any I/O failure.
    bool Open(const std::string& session_base, const std::vector<TopicMetaEntry>& topics)
    {
        bin_path_ = session_base + ".bin";

        bin_fd_ = ::open(bin_path_.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (bin_fd_ < 0)
        {
            std::fprintf(stderr, "[recorder] failed to open %s\n", bin_path_.c_str());
            return false;
        }

        if (!WriteFileHeaderAndMetadata(topics))
        {
            return false;
        }

        last_flush_ = std::chrono::steady_clock::now();
        std::printf("[recorder] session opened: %s, %zu topic(s) enabled\n",
                    bin_path_.c_str(), topics.size());
        return true;
    }

    /// @brief Appends one record to the .bin. Does not itself fsync --
    /// call MaybeFlush() periodically from the writer loop to control the
    /// crash-loss window.
    bool WriteRecord(const PoppedRecord& rec)
    {
        if (bin_fd_ < 0)
        {
            return false;
        }

        RecordHeader hdr;
        hdr.topic_id = rec.topic_id;
        hdr.stamp_ns = rec.stamp_ns;
        hdr.seq = rec.seq;

        std::uint8_t hdr_buf[logger_wire::kRecordHeaderWireSize];
        hdr.Serialize(hdr_buf);

        if (!WriteAll(bin_fd_, hdr_buf, sizeof(hdr_buf)))
        {
            return false;
        }
        if (!WriteAll(bin_fd_, rec.payload, rec.payload_len))
        {
            return false;
        }
        bin_write_offset_ += sizeof(hdr_buf) + rec.payload_len;

        ++pending_since_flush_;
        return true;
    }

    /// @brief fsync()s the .bin if kFlushInterval has elapsed since the
    /// last flush and at least one record has been written since then.
    /// Called once per writer-loop iteration (i.e. once per drain batch),
    /// which naturally amortizes fsync cost across however many records
    /// arrived in that batch while still bounding the crash-loss window.
    void MaybeFlush()
    {
        if (pending_since_flush_ == 0)
        {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_flush_ < kFlushInterval)
        {
            return;
        }

        if (bin_fd_ >= 0)
        {
            ::fsync(bin_fd_);
        }
        pending_since_flush_ = 0;
        last_flush_ = now;
    }

    /// @brief Final flush and close. Safe to call multiple times.
    void Close()
    {
        if (bin_fd_ >= 0)
        {
            ::fsync(bin_fd_);
            ::close(bin_fd_);
            bin_fd_ = -1;
        }
    }

private:
    /// @brief write() can perform a short write; loop until all bytes are
    /// written or a real error occurs.
    static bool WriteAll(int fd, const void* data, std::size_t len)
    {
        const std::uint8_t* p = static_cast<const std::uint8_t*>(data);
        std::size_t remaining = len;
        while (remaining > 0)
        {
            const ssize_t n = ::write(fd, p, remaining);
            if (n < 0)
            {
                std::fprintf(stderr, "[recorder] write() failed: %s\n", std::strerror(errno));
                return false;
            }
            p += n;
            remaining -= static_cast<std::size_t>(n);
        }
        return true;
    }

    bool WriteFileHeaderAndMetadata(const std::vector<TopicMetaEntry>& topics)
    {
        FileHeader header;
        header.version = logger_wire::kFormatVersion;
        header.topic_count = static_cast<std::uint32_t>(topics.size());
        header.created_at_ns = WallClockNs();

        std::uint8_t header_buf[logger_wire::kFileHeaderWireSize];
        header.Serialize(header_buf);
        if (!WriteAll(bin_fd_, header_buf, sizeof(header_buf)))
        {
            return false;
        }
        bin_write_offset_ += sizeof(header_buf);

        for (const auto& topic : topics)
        {
            std::uint8_t meta_buf[logger_wire::kTopicMetaWireSize];
            topic.Serialize(meta_buf);
            if (!WriteAll(bin_fd_, meta_buf, sizeof(meta_buf)))
            {
                return false;
            }
            bin_write_offset_ += sizeof(meta_buf);
        }
        return true;
    }

    int bin_fd_{-1};
    std::string bin_path_;
    std::uint64_t bin_write_offset_{0};
    std::uint32_t pending_since_flush_{0};
    std::chrono::steady_clock::time_point last_flush_{};
};

// ---------------------------------------------------------------------------
// Subscriber callbacks -- push into the ring buffer only, never touch disk.
// ---------------------------------------------------------------------------

namespace
{

// Each callback encodes the payload field-by-field into a small stack
// buffer (endian-safe, fixed wire_size -- see payload_codec.hpp) BEFORE
// pushing to the ring buffer. This is what makes recordings portable
// across CPU architectures: the ring buffer and BinaryWriter downstream
// only ever see already-normalized bytes, never a raw struct memcpy.

void OnPose(const i2w::Sample<Pose2D>& sample)
{
    std::uint8_t buf[TopicTraits<Pose2D>::wire_size];
    logger_wire::EncodePayload(sample.value, buf);
    g_ring_buffer.TryPush(static_cast<std::uint16_t>(TopicTraits<Pose2D>::id),
                           sample.header.stamp_ns, sample.header.seq,
                           buf, static_cast<std::uint32_t>(sizeof(buf)));
}

void OnAxis(const i2w::Sample<Axis>& sample)
{
    std::uint8_t buf[TopicTraits<Axis>::wire_size];
    logger_wire::EncodePayload(sample.value, buf);
    g_ring_buffer.TryPush(static_cast<std::uint16_t>(TopicTraits<Axis>::id),
                           sample.header.stamp_ns, sample.header.seq,
                           buf, static_cast<std::uint32_t>(sizeof(buf)));
}

void OnButtons(const i2w::Sample<Buttons>& sample)
{
    std::uint8_t buf[TopicTraits<Buttons>::wire_size];
    logger_wire::EncodePayload(sample.value, buf);
    g_ring_buffer.TryPush(static_cast<std::uint16_t>(TopicTraits<Buttons>::id),
                           sample.header.stamp_ns, sample.header.seq,
                           buf, static_cast<std::uint32_t>(sizeof(buf)));
}

} // namespace

// ---------------------------------------------------------------------------
// RecorderSystem -- subscribes only to topics enabled in logger_config.json
// ---------------------------------------------------------------------------

/// @brief Single i2w system holding up to three subscriptions (Pose2D, Axis,
/// Buttons), each created only if enabled in the loaded LoggerConfig. All
/// three callbacks funnel into the same global MpscRingBuffer.
class RecorderSystem final : public i2w::SystemBase
{
public:
    explicit RecorderSystem(i2w::Config config) : i2w::SystemBase(std::move(config)) {}

    /// @brief Metadata for topics that were actually enabled -- filled in
    /// during OnSetup(), read by main() to build the .bin's metadata table.
    const std::vector<TopicMetaEntry>& EnabledTopics() const { return enabled_topics_; }

private:
    i2w::LifecycleResult OnSetup() noexcept override
    {
        i2w::SubscriptionOptions opts;
        opts.plane = plane_;
        opts.reliability = i2w::Reliability::BestEffort;
        opts.queue_depth = 64;
        opts.overflow_policy = i2w::OverflowPolicy::DropOldest;

        if (g_logger_config.IsEnabled<Pose2D>())
        {
            auto sub = runtime().subscribe<Pose2D>(TopicTraits<Pose2D>::name, &OnPose, opts);
            if (!sub)
            {
                return i2w::Fail();
            }
            pose_sub_ = std::move(sub.value());
            AddTopicMeta<Pose2D>();
            std::printf("[recorder] pose:    ENABLED\n");
        }
        else
        {
            std::printf("[recorder] pose:    disabled by config\n");
        }

        if (g_logger_config.IsEnabled<Axis>())
        {
            auto sub = runtime().subscribe<Axis>(TopicTraits<Axis>::name, &OnAxis, opts);
            if (!sub)
            {
                return i2w::Fail();
            }
            axis_sub_ = std::move(sub.value());
            AddTopicMeta<Axis>();
            std::printf("[recorder] axis:    ENABLED\n");
        }
        else
        {
            std::printf("[recorder] axis:    disabled by config\n");
        }

        if (g_logger_config.IsEnabled<Buttons>())
        {
            auto sub = runtime().subscribe<Buttons>(TopicTraits<Buttons>::name, &OnButtons, opts);
            if (!sub)
            {
                return i2w::Fail();
            }
            buttons_sub_ = std::move(sub.value());
            AddTopicMeta<Buttons>();
            std::printf("[recorder] buttons: ENABLED\n");
        }
        else
        {
            std::printf("[recorder] buttons: disabled by config\n");
        }

        return i2w::Ok();
    }

    i2w::LifecycleResult OnTick() noexcept override { return i2w::Ok(); }

    template <typename T>
    void AddTopicMeta()
    {
        TopicMetaEntry meta;
        meta.topic_id = static_cast<std::uint16_t>(TopicTraits<T>::id);
        // wire_size (fixed encoded byte count), NOT sizeof(T) -- sizeof(T)
        // can differ across architectures due to struct padding/alignment.
        meta.struct_size = static_cast<std::uint32_t>(TopicTraits<T>::wire_size);
        std::snprintf(meta.name, sizeof(meta.name), "%s", TopicTraits<T>::name);
        enabled_topics_.push_back(meta);
    }

    i2w::Subscription<Pose2D> pose_sub_{};
    i2w::Subscription<Axis> axis_sub_{};
    i2w::Subscription<Buttons> buttons_sub_{};
    i2w::EndpointPlane plane_{i2w::EndpointPlane::Local};
    std::vector<TopicMetaEntry> enabled_topics_;
};

// ---------------------------------------------------------------------------
// Writer thread -- the single consumer of the ring buffer
// ---------------------------------------------------------------------------

/// @brief Drains g_ring_buffer continuously and persists records via
/// @p writer until g_shutdown is set. Runs on its own dedicated thread so
/// disk I/O never blocks the subscriber callback threads above.
void WriterThreadMain(BinaryWriter& writer)
{
    PoppedRecord rec;
    while (!g_shutdown.load(std::memory_order_relaxed))
    {
        bool drained_any = false;
        while (g_ring_buffer.TryPop(rec))
        {
            writer.WriteRecord(rec);
            drained_any = true;
        }

        if (drained_any)
        {
            writer.MaybeFlush();
        }
        else
        {
            std::this_thread::sleep_for(kIdlePollInterval);
        }
    }

    // Drain anything left, then do a final flush on the way out.
    while (g_ring_buffer.TryPop(rec))
    {
        writer.WriteRecord(rec);
    }
    writer.MaybeFlush();
}

int main(int argc, char** argv)
{
    if (!g_logger_config.LoadFromFile("/home/octobot/Github/TriDrishti-ws/src/TriDrishti-ChronoCap/config/logger_config.json"))
    {
        std::printf("[recorder] failed to load logger config\n");
        return 1;
    }

    i2w::Config config;
    config.node_name = "recorder";
    config.ns = "/demo";

    RecorderSystem system(std::move(config));
    if (!system.Setup().ok)
    {
        std::printf("[recorder] setup failed\n");
        return 1;
    }

    if (system.EnabledTopics().empty())
    {
        std::printf("[recorder] WARNING: no topics enabled -- nothing will be recorded\n");
    }

    BinaryWriter writer;
    if (!writer.Open(MakeSessionBaseName(), system.EnabledTopics()))
    {
        std::printf("[recorder] failed to open output files\n");
        return 1;
    }

    std::thread writer_thread(WriterThreadMain, std::ref(writer));

    // Main thread just keeps the i2w system alive so subscriptions stay
    // active and callbacks keep firing into the ring buffer.
    while (true)
    {
        if (!system.Tick().ok)
        {
            std::printf("[recorder] tick failed\n");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    g_shutdown.store(true, std::memory_order_relaxed);
    writer_thread.join();
    writer.Close();

    std::printf("[recorder] shutdown complete, dropped_count=%llu\n",
                static_cast<unsigned long long>(g_ring_buffer.DroppedCount()));
    return 0;
}