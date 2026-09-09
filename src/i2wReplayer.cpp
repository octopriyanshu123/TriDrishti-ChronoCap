/// @file i2wReplayer.cpp
/// @brief Generic Replayer: opens a .bin via chrono_cap::RecordReader,
///        advertises i2w publishers only for topics present in the file
///        (matched against AllTopics), and decodes+republishes each
///        record as it's replayed.
///
/// Usage: ./i2wReplayer <path/to/recording.bin>
///
/// Adding a topic never requires editing this file -- see topic_list.hpp.

#include <cstdio>
#include <utility>

#include "i2w/impl.hpp"
#include "chrono_cap/chrono_cap.hpp"

using ReplayerSystem = chrono_cap::GenericReplayerSystem<AllTopics>;

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::fprintf(stderr, "usage: %s <path/to/recording.bin>\n", argv[0]);
        return 1;
    }

    i2w::Config config;
    config.node_name = "i2w_replayer";
    config.ns = "";

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
