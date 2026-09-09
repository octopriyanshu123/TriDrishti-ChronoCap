/// @file record_reader.cpp
#include "chrono_cap/record_reader.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "chrono_cap/wire_format.hpp"

namespace chrono_cap
{

    namespace
    {

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

    RecordReader::~RecordReader()
    {
        if (base_ != nullptr)
        {
            ::munmap(const_cast<std::uint8_t *>(base_), file_size_);
            base_ = nullptr;
        }
    }

    bool RecordReader::Open(const std::string &path, const TopicHook &on_topic)
    {
        const int fd = ::open(path.c_str(), O_RDONLY);
        if (fd < 0)
        {
            std::fprintf(stderr, "[record_reader] failed to open %s: %s\n", path.c_str(),
                         std::strerror(errno));
            return false;
        }

        struct stat st{};
        if (::fstat(fd, &st) != 0)
        {
            std::fprintf(stderr, "[record_reader] fstat failed: %s\n", std::strerror(errno));
            ::close(fd);
            return false;
        }
        file_size_ = static_cast<std::size_t>(st.st_size);

        void *mapped = ::mmap(nullptr, file_size_, PROT_READ, MAP_PRIVATE, fd, 0);
        ::close(fd);
        if (mapped == MAP_FAILED)
        {
            std::fprintf(stderr, "[record_reader] mmap failed: %s\n", std::strerror(errno));
            return false;
        }
        base_ = static_cast<const std::uint8_t *>(mapped);

        if (!ParseHeaderAndMetadata())
        {
            return false;
        }

        if (on_topic)
        {
            for (const auto &topic : topics_)
            {
                if (!on_topic(topic))
                {
                    std::fprintf(stderr, "[record_reader] topic hook rejected topic '%s'\n",
                                 topic.name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    bool RecordReader::ParseHeaderAndMetadata()
    {
        if (file_size_ < logger_wire::kFileHeaderWireSize)
        {
            std::fprintf(stderr, "[record_reader] file too small to contain a header\n");
            return false;
        }

        logger_wire::FileHeader header;
        if (!logger_wire::FileHeader::Deserialize(base_, header))
        {
            std::fprintf(stderr, "[record_reader] bad magic bytes -- not a recognized log file\n");
            return false;
        }
        if (header.version != logger_wire::kFormatVersion)
        {
            std::fprintf(stderr, "[record_reader] unsupported format version %u (expected %u)\n",
                         header.version, logger_wire::kFormatVersion);
            return false;
        }

        std::size_t offset = logger_wire::kFileHeaderWireSize;
        for (std::uint32_t i = 0; i < header.topic_count; ++i)
        {
            if (offset + logger_wire::kTopicMetaWireSize > file_size_)
            {
                std::fprintf(stderr, "[record_reader] truncated topic metadata table\n");
                return false;
            }

            logger_wire::TopicMetaEntry entry;
            logger_wire::TopicMetaEntry::Deserialize(base_ + offset, entry);
            offset += logger_wire::kTopicMetaWireSize;

            ResolvedTopic resolved;
            resolved.topic_id = entry.topic_id;
            resolved.wire_size = entry.struct_size;
            resolved.name = std::string(entry.name);
            topics_.push_back(resolved);
            by_id_[resolved.topic_id] = resolved;

            std::printf("[record_reader] topic in file: id=%u name=%s wire_size=%u\n",
                        resolved.topic_id, resolved.name.c_str(), resolved.wire_size);
        }

        records_start_offset_ = offset;
        return true;
    }

    void RecordReader::Run(const RecordCallback &callback)
    {
        std::size_t offset = records_start_offset_;
        const std::size_t end = file_size_;

        bool have_t0 = false;
        std::uint64_t t0_ns = 0;
        std::uint64_t replay_start_monotonic_ns = 0;
        std::uint64_t records_played = 0;

        while (offset + logger_wire::kRecordHeaderWireSize <= end)
        {
            logger_wire::RecordHeader hdr;
            logger_wire::RecordHeader::Deserialize(base_ + offset, hdr);

            auto it = by_id_.find(hdr.topic_id);
            if (it == by_id_.end())
            {
                std::fprintf(stderr, "[record_reader] unknown topic_id=%u mid-file -- stopping\n",
                             hdr.topic_id);
                break;
            }
            const ResolvedTopic &topic = it->second;

            const std::size_t payload_offset = offset + logger_wire::kRecordHeaderWireSize;
            if (payload_offset + topic.wire_size > end)
            {
                std::fprintf(stderr, "[record_reader] truncated record at offset %zu -- stopping\n",
                             offset);
                break;
            }
            const std::uint8_t *payload_ptr = base_ + payload_offset;

            if (!have_t0)
            {
                t0_ns = hdr.stamp_ns;
                replay_start_monotonic_ns = MonotonicNowNs();
                have_t0 = true;
            }

            // Guard against a rare out-of-order stamp (e.g. cross-thread
            // jitter) causing unsigned underflow -- clamp to "play immediately"
            // rather than wrapping to a huge delay.
            const std::uint64_t elapsed_ns = (hdr.stamp_ns >= t0_ns) ? (hdr.stamp_ns - t0_ns) : 0;
            const std::uint64_t target_ns = replay_start_monotonic_ns + elapsed_ns;
            SleepUntilAbsoluteNs(target_ns);

            callback(hdr.topic_id, hdr.stamp_ns, hdr.seq, payload_ptr, topic.wire_size);
            ++records_played;

            offset = payload_offset + topic.wire_size;
        }

        std::printf("[record_reader] done -- %llu record(s) replayed\n",
                    static_cast<unsigned long long>(records_played));
    }

    void RecordReader::Walk(const RecordCallback &callback, bool with_timing)
    {
        std::size_t offset = records_start_offset_;
        const std::size_t end = file_size_;

        bool have_t0 = false;
        std::uint64_t t0_ns = 0;
        std::uint64_t replay_start_monotonic_ns = 0;
        std::uint64_t records_played = 0;

        while (offset + logger_wire::kRecordHeaderWireSize <= end)
        {
            logger_wire::RecordHeader hdr;
            logger_wire::RecordHeader::Deserialize(base_ + offset, hdr);

            auto it = by_id_.find(hdr.topic_id);
            if (it == by_id_.end())
            {
                std::fprintf(stderr, "[record_reader] unknown topic_id=%u mid-file -- stopping\n",
                             hdr.topic_id);
                break;
            }
            const ResolvedTopic &topic = it->second;

            const std::size_t payload_offset = offset + logger_wire::kRecordHeaderWireSize;
            if (payload_offset + topic.wire_size > end)
            {
                std::fprintf(stderr, "[record_reader] truncated record at offset %zu -- stopping\n",
                             offset);
                break;
            }
            const std::uint8_t *payload_ptr = base_ + payload_offset;

            if (with_timing)
            {
                if (!have_t0)
                {
                    t0_ns = hdr.stamp_ns;
                    replay_start_monotonic_ns = MonotonicNowNs();
                    have_t0 = true;
                }

                // Guard against a rare out-of-order stamp (e.g. cross-thread
                // jitter) causing unsigned underflow -- clamp to "play
                // immediately" rather than wrapping to a huge delay.
                const std::uint64_t elapsed_ns = (hdr.stamp_ns >= t0_ns) ? (hdr.stamp_ns - t0_ns) : 0;
                const std::uint64_t target_ns = replay_start_monotonic_ns + elapsed_ns;
                SleepUntilAbsoluteNs(target_ns);
            }
            // else: RunFast -- no sleeping, walk through at full speed. Useful
            // for offline dumping/log-extraction where real-time pacing would
            // just make the tool take as long as the original recording did.

            callback(hdr.topic_id, hdr.stamp_ns, hdr.seq, payload_ptr, topic.wire_size);
            ++records_played;

            offset = payload_offset + topic.wire_size;
        }

        std::printf("[record_reader] done -- %llu record(s) replayed\n",
                    static_cast<unsigned long long>(records_played));
    }
} // namespace chrono_cap
