/// @file record_writer.cpp
#include "chrono_cap/record_writer.hpp"

#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <unistd.h>

namespace chrono_cap
{

bool RecordWriter::Open(const std::string& session_base, const std::vector<TopicInfo>& topics)
{
    bin_path_ = session_base + ".bin";
    bin_fd_ = ::open(bin_path_.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (bin_fd_ < 0)
    {
        std::fprintf(stderr, "[record_writer] failed to open %s\n", bin_path_.c_str());
        return false;
    }

    if (!WriteFileHeaderAndMetadata(topics))
    {
        return false;
    }

    last_flush_ = std::chrono::steady_clock::now();
    std::printf("[record_writer] session opened: %s, %zu topic(s)\n", bin_path_.c_str(), topics.size());

    shutdown_.store(false, std::memory_order_relaxed);
    writer_thread_ = std::thread(&RecordWriter::WriterThreadMain, this);
    return true;
}

void RecordWriter::Close()
{
    if (writer_thread_.joinable())
    {
        shutdown_.store(true, std::memory_order_relaxed);
        writer_thread_.join();
    }
    if (bin_fd_ >= 0)
    {
        ::fsync(bin_fd_);
        ::close(bin_fd_);
        bin_fd_ = -1;
        std::printf("[record_writer] closed %s, dropped_count=%llu\n", bin_path_.c_str(),
                    static_cast<unsigned long long>(ring_.DroppedCount()));
    }
}

void RecordWriter::WriterThreadMain()
{
    logger_ring::PoppedRecord rec;
    while (!shutdown_.load(std::memory_order_relaxed))
    {
        bool drained_any = false;
        while (ring_.TryPop(rec))
        {
            WriteRecord(rec);
            drained_any = true;
        }

        if (drained_any)
        {
            MaybeFlush();
        }
        else
        {
            std::this_thread::sleep_for(idle_poll_interval_);
        }
    }

    // Drain whatever's left, then a final flush on the way out.
    while (ring_.TryPop(rec))
    {
        WriteRecord(rec);
    }
    MaybeFlush();
}

bool RecordWriter::WriteFileHeaderAndMetadata(const std::vector<TopicInfo>& topics)
{
    logger_wire::FileHeader header;
    header.version = logger_wire::kFormatVersion;
    header.topic_count = static_cast<std::uint32_t>(topics.size());
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    header.created_at_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());

    std::uint8_t header_buf[logger_wire::kFileHeaderWireSize];
    header.Serialize(header_buf);
    if (!WriteAll(bin_fd_, header_buf, sizeof(header_buf)))
    {
        return false;
    }

    for (const auto& topic : topics)
    {
        logger_wire::TopicMetaEntry meta;
        meta.topic_id = topic.topic_id;
        meta.struct_size = topic.wire_size;
        std::snprintf(meta.name, sizeof(meta.name), "%s", topic.name.c_str());

        std::uint8_t meta_buf[logger_wire::kTopicMetaWireSize];
        meta.Serialize(meta_buf);
        if (!WriteAll(bin_fd_, meta_buf, sizeof(meta_buf)))
        {
            return false;
        }
    }
    return true;
}

bool RecordWriter::WriteRecord(const logger_ring::PoppedRecord& rec)
{
    if (bin_fd_ < 0)
    {
        return false;
    }

    logger_wire::RecordHeader hdr;
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

    ++pending_since_flush_;
    return true;
}

void RecordWriter::MaybeFlush()
{
    if (pending_since_flush_ == 0)
    {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - last_flush_ < flush_interval_)
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

bool RecordWriter::WriteAll(int fd, const void* data, std::size_t len)
{
    const std::uint8_t* p = static_cast<const std::uint8_t*>(data);
    std::size_t remaining = len;
    while (remaining > 0)
    {
        const ssize_t n = ::write(fd, p, remaining);
        if (n < 0)
        {
            std::fprintf(stderr, "[record_writer] write() failed: %s\n", std::strerror(errno));
            return false;
        }
        p += n;
        remaining -= static_cast<std::size_t>(n);
    }
    return true;
}

} // namespace chrono_cap
