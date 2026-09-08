/// @file i2wReplayer.cpp
/// @brief i2w-specific Replayer: opens a .bin via chrono_cap::RecordReader,
///        advertises i2w publishers only for topics present in the file,
///        and decodes+republishes each record as it's replayed.
///
/// Usage: ./i2wReplayer <path/to/recording.bin>
///
/// All mmap'ing, header/metadata parsing, and replay-timing logic live in
/// the chrono_cap library (see lib/). This file only knows how to talk to
/// i2w and how to decode this project's specific message types.

#include <cstdio>
#include <utility>

#include "i2w/impl.hpp"
#include "chrono_cap/chrono_cap.hpp"


using logger_msgs::Axis;
using logger_msgs::Buttons;
using logger_msgs::Pose2D;
using logger_msgs::TopicId;
using logger_msgs::TopicTraits;

/// @brief Advertises i2w publishers only for the topics present in the
/// opened recording, then decodes+republishes each record handed to it by
/// chrono_cap::RecordReader::Run().
class ReplayerSystem final : public i2w::SystemBase
{
public:
    explicit ReplayerSystem(i2w::Config config) : i2w::SystemBase(std::move(config)) {}

    /// @brief Called once per topic found in the file, before replay
    /// starts. Advertises the matching i2w publisher.
    bool OnTopicFound(const chrono_cap::ResolvedTopic& topic)
    {
        i2w::PublisherOptions opts;
        opts.plane = plane_;

        switch (static_cast<TopicId>(topic.topic_id))
        {
            case TopicId::Pose:
            {
                if (topic.wire_size != TopicTraits<Pose2D>::wire_size)
                {
                    std::fprintf(stderr, "[i2wReplayer] pose wire_size mismatch\n");
                    return false;
                }
                auto pub = runtime().advertise<Pose2D>(TopicTraits<Pose2D>::name, opts);
                if (!pub) return false;
                pose_pub_ = std::move(pub.value());
                std::printf("[i2wReplayer] pose:    advertised\n");
                return true;
            }
            case TopicId::Axis:
            {
                if (topic.wire_size != TopicTraits<Axis>::wire_size)
                {
                    std::fprintf(stderr, "[i2wReplayer] axis wire_size mismatch\n");
                    return false;
                }
                auto pub = runtime().advertise<Axis>(TopicTraits<Axis>::name, opts);
                if (!pub) return false;
                axis_pub_ = std::move(pub.value());
                std::printf("[i2wReplayer] axis:    advertised\n");
                return true;
            }
            case TopicId::Buttons:
            {
                if (topic.wire_size != TopicTraits<Buttons>::wire_size)
                {
                    std::fprintf(stderr, "[i2wReplayer] buttons wire_size mismatch\n");
                    return false;
                }
                auto pub = runtime().advertise<Buttons>(TopicTraits<Buttons>::name, opts);
                if (!pub) return false;
                buttons_pub_ = std::move(pub.value());
                std::printf("[i2wReplayer] buttons: advertised\n");
                return true;
            }
            default:
                std::fprintf(stderr, "[i2wReplayer] unknown topic_id=%u in file\n", topic.topic_id);
                return false;
        }
    }

    /// @brief Called once per record by RecordReader::Run(), at the
    /// wall-clock moment matching its original relative timing. Decodes
    /// and republishes with the original stamp preserved.
    void OnRecord(std::uint16_t topic_id, std::uint64_t stamp_ns, std::uint64_t /*seq*/,
                  const std::uint8_t* payload, std::uint32_t /*len*/)
    {
        switch (static_cast<TopicId>(topic_id))
        {
            case TopicId::Pose:
            {
                Pose2D v{};
                logger_wire::DecodePayload(payload, v);
                pose_pub_.publish(v, stamp_ns);
                break;
            }
            case TopicId::Axis:
            {
                Axis v{};
                logger_wire::DecodePayload(payload, v);
                axis_pub_.publish(v, stamp_ns);
                break;
            }
            case TopicId::Buttons:
            {
                Buttons v{};
                logger_wire::DecodePayload(payload, v);
                buttons_pub_.publish(v, stamp_ns);
                break;
            }
        }
    }

private:
    i2w::LifecycleResult OnSetup() noexcept override { return i2w::Ok(); }
    i2w::LifecycleResult OnTick() noexcept override { return i2w::Ok(); }

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

    i2w::Config config;
    config.node_name = "i2w_replayer";
    config.ns = "/demo";

    ReplayerSystem system(std::move(config));
    if (!system.Setup().ok)
    {
        std::printf("[i2wReplayer] setup failed\n");
        return 1;
    }

    chrono_cap::RecordReader reader;
    const bool opened = reader.Open(argv[1], [&system](const chrono_cap::ResolvedTopic& topic) {
        return system.OnTopicFound(topic);
    });
    if (!opened)
    {
        return 1;
    }
    if (reader.Topics().empty())
    {
        std::printf("[i2wReplayer] file contains no topics -- nothing to replay\n");
        return 0;
    }

    reader.Run([&system](std::uint16_t topic_id, std::uint64_t stamp_ns, std::uint64_t seq,
                         const std::uint8_t* payload, std::uint32_t len) {
        system.OnRecord(topic_id, stamp_ns, seq, payload, len);
    });

    return 0;
}
