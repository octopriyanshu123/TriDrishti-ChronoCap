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
#include <atomic>
#include <csignal>

#include "i2w/impl.hpp"
#include "chrono_cap/chrono_cap.hpp"
#include "logger.hpp"

namespace
{

    std::atomic<bool> g_running{true};

    void SignalHandler(int signal)
    {
        if (signal == SIGINT)
        {
            g_running = false;
        }
    }

    std::string MakeSessionBaseName()
    {
        const std::time_t t = std::time(nullptr);
        std::tm tm_buf{};
        localtime_r(&t, &tm_buf);

        char buf[64];

        std::snprintf(buf, sizeof(buf),
                      "recording_%04d%02d%02d_%02d%02d%02d",
                      tm_buf.tm_year + 1900,
                      tm_buf.tm_mon + 1,
                      tm_buf.tm_mday,
                      tm_buf.tm_hour,
                      tm_buf.tm_min,
                      tm_buf.tm_sec);

        return std::string(buf);
    }

} // namespace
namespace
{

    // std::string MakeSessionBaseName()
    // {
    //     const std::time_t t = std::time(nullptr);
    //     std::tm tm_buf{};
    //     localtime_r(&t, &tm_buf);
    //     char buf[64];
    //     std::snprintf(buf, sizeof(buf), "recording_%04d%02d%02d_%02d%02d%02d",
    //                   tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
    //                   tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);
    //     return std::string(buf);
    // }

} // namespace

void configerLogger()
{
    // Get current time
    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);

    std::tm localTime = *std::localtime(&time);

    // Get current user's home directory
    const char *home = std::getenv("HOME");

    std::ostringstream timestamp;
    timestamp << std::put_time(&localTime, "%Y%m%d_%H%M%S");

    std::string value = timestamp.str();

    // Create date directory: 27_sep_2016
    std::ostringstream date;
    date << std::put_time(&localTime, "%d_%b_%Y");

    std::string logDir = std::string(home) + "/logs/chronoCapRecorder/" + date.str();

    // Create directory if it doesn't exist
    std::filesystem::create_directories(logDir);

    // Create log file
    std::string logFile = logDir + "/" + value + ".log";

    std::ofstream file(logFile, std::ios::app);

    // if (!file.is_open())
    // {
    //     return 1;
    // }

    // file << "Log file created/opened\n";

    // file.close();

    // default folder ($HOME/log/robot.log)

    // Logger::getInstance().configure(Logger::LogLevel::DEBUG, "robot.log", false);

    // Pass an absolute path directly as the filename, no logDir needed

    Logger::getInstance().configure(Logger::LogLevel::DEBUG, logFile, false, "", false);
    auto &log = Logger::getInstance();

    LOG_DEBUG("Main", "init");
}

using RecorderSystem = chrono_cap::GenericRecorderSystem<AllTopics>;

int main(int argc, char **argv)
{

    std::signal(SIGINT, SignalHandler);
    configerLogger();
    logger_cfg::LoggerConfig config;
    if (!config.LoadFromFile(std::string(CONFIG_DIR) + "/logger_config.json"))
    {
        std::printf("[i2wRecorder] failed to load logger config\n");
        return 1;
    }

    config.PrintSummary();

    chrono_cap::RecordWriter writer;

    i2w::Config i2w_cfg;
    i2w_cfg.node_name = "i2w_recorder";
    i2w_cfg.ns = "";

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

    while (g_running)
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
