/// @file replay_and_print.cpp
/// @brief Standalone test tool -- replays a .bin recorded by
///        generate_and_record (or i2wRecorder) using chrono_cap, decodes
///        each record, and writes a human-readable line per record to an
///        output text file. No i2w framework involved.
///
/// Usage: ./replay_and_print <input.bin> [output.txt]
///        (default output.txt: "replay_output.txt")

#include <cstdio>
#include <string>

#include "chrono_cap/chrono_cap.hpp"

#include "logger_types.hpp"
#include "payload_codec.hpp"
#include "topic_registry.hpp"

using logger_msgs::Axis;
using logger_msgs::Buttons;
using logger_msgs::Pose2D;
using logger_msgs::TopicId;
using logger_msgs::TopicTraits;

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::fprintf(stderr, "usage: %s <input.bin> [output.txt]\n", argv[0]);
        return 1;
    }

    const std::string input_path = argv[1];
    const std::string output_path = (argc > 2) ? argv[2] : "replay_output.txt";

    FILE* out = std::fopen(output_path.c_str(), "w");
    if (out == nullptr)
    {
        std::fprintf(stderr, "[replay_and_print] failed to open %s for writing\n", output_path.c_str());
        return 1;
    }

    chrono_cap::RecordReader reader;
    // No i2w publishers to advertise here -- just accept every topic found.
    const bool opened = reader.Open(input_path, [](const chrono_cap::ResolvedTopic&) { return true; });
    if (!opened)
    {
        std::fclose(out);
        return 1;
    }
    if (reader.Topics().empty())
    {
        std::printf("[replay_and_print] file contains no topics -- nothing to replay\n");
        std::fclose(out);
        return 0;
    }

    std::uint64_t line_count = 0;

    reader.Run([&](std::uint16_t topic_id, std::uint64_t stamp_ns, std::uint64_t seq,
                    const std::uint8_t* payload, std::uint32_t /*len*/) {
        switch (static_cast<TopicId>(topic_id))
        {
            case TopicId::Pose:
            {
                Pose2D v{};
                logger_wire::DecodePayload(payload, v);
                std::fprintf(out, "[pose]    seq=%llu stamp_ns=%llu x=%.3f y=%.3f yaw=%.3f\n",
                            static_cast<unsigned long long>(seq),
                            static_cast<unsigned long long>(stamp_ns), v.x, v.y, v.yaw);
                break;
            }
            case TopicId::Axis:
            {
                Axis v{};
                logger_wire::DecodePayload(payload, v);
                std::fprintf(out, "[axis]    seq=%llu stamp_ns=%llu axes_count=%d\n",
                            static_cast<unsigned long long>(seq),
                            static_cast<unsigned long long>(stamp_ns), v.axes_count);
                break;
            }
            case TopicId::Buttons:
            {
                Buttons v{};
                logger_wire::DecodePayload(payload, v);
                std::fprintf(out, "[buttons] seq=%llu stamp_ns=%llu buttons_count=%d\n",
                            static_cast<unsigned long long>(seq),
                            static_cast<unsigned long long>(stamp_ns), v.buttons_count);
                break;
            }
            default:
                std::fprintf(out, "[unknown] topic_id=%u seq=%llu stamp_ns=%llu\n", topic_id,
                            static_cast<unsigned long long>(seq),
                            static_cast<unsigned long long>(stamp_ns));
                break;
        }
        std::fflush(out); // flush per record so partial output survives Ctrl+C mid-replay
    });

    std::fclose(out);
    std::printf("[replay_and_print] wrote decoded records to %s\n", output_path.c_str());
    return 0;
}
