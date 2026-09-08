/// @file i2wRecorder.cpp
/// @brief Generic Recorder: subscribes to every ENABLED topic in AllTopics
///        (topic_list.hpp), encodes each payload endian-safely, and hands
///        raw bytes to the chrono_cap::RecordWriter library.
///
/// Adding a topic never requires editing this file -- see topic_list.hpp.
/// Enabling/disabling a topic is entirely controlled by logger_config.json
/// at runtime, no recompile needed.

#include <chrono>
#include <cstdio>
#include <ctime>
#include <string>
#include <thread>

#include "i2w/impl.hpp"
#include "chrono_cap/chrono_cap.hpp"


namespace
{

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

using RecorderSystem = chrono_cap::GenericRecorderSystem<AllTopics>;

int main(int argc, char** argv)
{
    logger_cfg::LoggerConfig config;
    if (!config.LoadFromFile(std::string(CONFIG_DIR) + "/logger_config.json"))
    {
        std::printf("[i2wRecorder] failed to load logger config\n");
        return 1;
    }

    chrono_cap::RecordWriter writer;

    i2w::Config i2w_cfg;
    i2w_cfg.node_name = "i2w_recorder";
    i2w_cfg.ns = "/demo";

    RecorderSystem system(std::move(i2w_cfg), chrono_cap::RecorderContext{&config, &writer});
    if (!system.Setup().ok)
    {
        std::printf("[i2wRecorder] setup failed\n");
        return 1;
    }

    if (system.EnabledTopics().empty())
    {
        std::printf("[i2wRecorder] WARNING: no topics enabled -- nothing will be recorded\n");
    }

    if (!writer.Open(MakeSessionBaseName(), system.EnabledTopics()))
    {
        std::printf("[i2wRecorder] failed to open output file\n");
        return 1;
    }

    while (true)
    {
        if (!system.Tick().ok)
        {
            std::printf("[i2wRecorder] tick failed\n");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    writer.Close();
    return 0;
}
