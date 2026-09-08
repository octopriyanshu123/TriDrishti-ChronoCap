/// @file generate_and_record.cpp
/// @brief Standalone test tool -- generates random Pose2D/Axis/Buttons
///        samples (no i2w framework involved at all) and records them to a
///        .bin via the chrono_cap library, so the library can be exercised
///        and verified independently of i2w.
///
/// Usage: ./generate_and_record [duration_seconds]  (default: 5)
///
/// Simulates the same rates used elsewhere in this project (Pose 10Hz,
/// Axis 100Hz, Buttons 20Hz), generates all samples up front sorted by
/// timestamp (mimicking the interleaved order a real recorder would see),
/// then pushes them into chrono_cap::RecordWriter in that order.

#include <algorithm>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "chrono_cap/chrono_cap.hpp"

#include "logger_types.hpp"
#include "payload_codec.hpp"
#include "topic_registry.hpp"

using logger_msgs::Axis;
using logger_msgs::Buttons;
using logger_msgs::Pose2D;
using logger_msgs::TopicTraits;

namespace
{

/// @brief One pending event before it's pushed to the writer, kept generic
/// so all three topic types can be sorted together by timestamp.
struct PendingEvent final
{
    std::uint64_t stamp_ns{0};
    std::uint64_t seq{0};
    std::uint16_t topic_id{0};
    std::vector<std::uint8_t> encoded;
};

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

int main(int argc, char** argv)
{
    const double duration_s = (argc > 1) ? std::atof(argv[1]) : 5.0;

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> pose_dist(-10.0f, 10.0f);
    std::uniform_int_distribution<int> axis_count_dist(1, 8);
    std::uniform_int_distribution<int> buttons_count_dist(0, 16);

    std::vector<PendingEvent> events;

    // Pose2D @ 10Hz
    {
        const std::uint64_t period_ns = 100'000'000ULL; // 100ms
        const std::uint64_t total = static_cast<std::uint64_t>(duration_s * 1e9);
        std::uint64_t seq = 0;
        for (std::uint64_t t = 0; t < total; t += period_ns, ++seq)
        {
            Pose2D pose{pose_dist(rng), pose_dist(rng), pose_dist(rng)};
            std::vector<std::uint8_t> buf(TopicTraits<Pose2D>::wire_size);
            logger_wire::EncodePayload(pose, buf.data());
            events.push_back({t, seq, static_cast<std::uint16_t>(TopicTraits<Pose2D>::id), std::move(buf)});
        }
    }

    // Axis @ 100Hz
    {
        const std::uint64_t period_ns = 10'000'000ULL; // 10ms
        const std::uint64_t total = static_cast<std::uint64_t>(duration_s * 1e9);
        std::uint64_t seq = 0;
        for (std::uint64_t t = 0; t < total; t += period_ns, ++seq)
        {
            Axis axis{};
            axis.sequence = seq;
            axis.timestamp_ns = t;
            axis.axes_count = axis_count_dist(rng);
            std::vector<std::uint8_t> buf(TopicTraits<Axis>::wire_size);
            logger_wire::EncodePayload(axis, buf.data());
            events.push_back({t, seq, static_cast<std::uint16_t>(TopicTraits<Axis>::id), std::move(buf)});
        }
    }

    // Buttons @ 20Hz
    {
        const std::uint64_t period_ns = 50'000'000ULL; // 50ms
        const std::uint64_t total = static_cast<std::uint64_t>(duration_s * 1e9);
        std::uint64_t seq = 0;
        for (std::uint64_t t = 0; t < total; t += period_ns, ++seq)
        {
            Buttons buttons{};
            buttons.sequence = seq;
            buttons.timestamp_ns = t;
            buttons.buttons_count = buttons_count_dist(rng);
            std::vector<std::uint8_t> buf(TopicTraits<Buttons>::wire_size);
            logger_wire::EncodePayload(buttons, buf.data());
            events.push_back({t, seq, static_cast<std::uint16_t>(TopicTraits<Buttons>::id), std::move(buf)});
        }
    }

    // Sort by timestamp so records land in the file in the same
    // chronologically-interleaved order a real multi-topic recorder
    // would naturally produce.
    std::sort(events.begin(), events.end(),
              [](const PendingEvent& a, const PendingEvent& b) { return a.stamp_ns < b.stamp_ns; });

    std::vector<chrono_cap::TopicInfo> topics;
    topics.push_back({static_cast<std::uint16_t>(TopicTraits<Pose2D>::id),
                       static_cast<std::uint32_t>(TopicTraits<Pose2D>::wire_size), TopicTraits<Pose2D>::name});
    topics.push_back({static_cast<std::uint16_t>(TopicTraits<Axis>::id),
                       static_cast<std::uint32_t>(TopicTraits<Axis>::wire_size), TopicTraits<Axis>::name});
    topics.push_back({static_cast<std::uint16_t>(TopicTraits<Buttons>::id),
                       static_cast<std::uint32_t>(TopicTraits<Buttons>::wire_size), TopicTraits<Buttons>::name});

    chrono_cap::RecordWriter writer;
    const std::string session_base = MakeSessionBaseName();
    if (!writer.Open(session_base, topics))
    {
        std::fprintf(stderr, "[generate_and_record] failed to open output file\n");
        return 1;
    }

    for (const auto& ev : events)
    {
        if (!writer.Push(ev.topic_id, ev.stamp_ns, ev.seq, ev.encoded.data(),
                          static_cast<std::uint32_t>(ev.encoded.size())))
        {
            std::fprintf(stderr, "[generate_and_record] WARNING: record dropped (ring buffer full)\n");
        }
    }

    writer.Close();

    std::printf("[generate_and_record] wrote %zu records to %s.bin\n", events.size(), session_base.c_str());
    return 0;
}
