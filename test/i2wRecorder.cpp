/// @file i2wRecorder.cpp
/// @brief i2w-specific Recorder: subscribes to enabled topics (Pose2D, Axis,
///        Buttons), encodes each payload endian-safely, and hands raw bytes
///        to the generic chrono_cap::RecordWriter library for persistence.
///
/// All disk I/O, ring-buffering, threading, and file-format concerns live
/// in the chrono_cap library (see lib/). This file only knows how to talk
/// to i2w and how to encode this project's specific message types -- it
/// is the "glue" layer, not the storage engine.

#include <chrono>
#include <cstdio>
#include <thread>

#include "i2w/impl.hpp"
#include "chrono_cap/record_writer.hpp"

#include "logger_config.hpp"
#include "logger_types.hpp"
#include "payload_codec.hpp"
#include "topic_registry.hpp"

using logger_msgs::Axis;
using logger_msgs::Buttons;
using logger_msgs::Pose2D;
using logger_msgs::TopicTraits;

namespace
{

logger_cfg::LoggerConfig g_logger_config;
chrono_cap::RecordWriter g_writer;

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

void OnPose(const i2w::Sample<Pose2D>& sample)
{
    std::uint8_t buf[TopicTraits<Pose2D>::wire_size];
    logger_wire::EncodePayload(sample.value, buf);
    g_writer.Push(static_cast<std::uint16_t>(TopicTraits<Pose2D>::id), sample.header.stamp_ns,
                  sample.header.seq, buf, sizeof(buf));
}

void OnAxis(const i2w::Sample<Axis>& sample)
{
    std::uint8_t buf[TopicTraits<Axis>::wire_size];
    logger_wire::EncodePayload(sample.value, buf);
    g_writer.Push(static_cast<std::uint16_t>(TopicTraits<Axis>::id), sample.header.stamp_ns,
                  sample.header.seq, buf, sizeof(buf));
}

void OnButtons(const i2w::Sample<Buttons>& sample)
{
    std::uint8_t buf[TopicTraits<Buttons>::wire_size];
    logger_wire::EncodePayload(sample.value, buf);
    g_writer.Push(static_cast<std::uint16_t>(TopicTraits<Buttons>::id), sample.header.stamp_ns,
                  sample.header.seq, buf, sizeof(buf));
}

} // namespace

/// @brief Subscribes only to topics enabled in logger_config.json, each
/// callback pushing encoded bytes into the shared chrono_cap::RecordWriter.
class RecorderSystem final : public i2w::SystemBase
{
public:
    explicit RecorderSystem(i2w::Config config) : i2w::SystemBase(std::move(config)) {}

    const std::vector<chrono_cap::TopicInfo>& EnabledTopics() const { return enabled_topics_; }

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
            if (!sub) return i2w::Fail();
            pose_sub_ = std::move(sub.value());
            AddTopicInfo<Pose2D>();
            std::printf("[i2wRecorder] pose:    ENABLED\n");
        }
        else
        {
            std::printf("[i2wRecorder] pose:    disabled by config\n");
        }

        if (g_logger_config.IsEnabled<Axis>())
        {
            auto sub = runtime().subscribe<Axis>(TopicTraits<Axis>::name, &OnAxis, opts);
            if (!sub) return i2w::Fail();
            axis_sub_ = std::move(sub.value());
            AddTopicInfo<Axis>();
            std::printf("[i2wRecorder] axis:    ENABLED\n");
        }
        else
        {
            std::printf("[i2wRecorder] axis:    disabled by config\n");
        }

        if (g_logger_config.IsEnabled<Buttons>())
        {
            auto sub = runtime().subscribe<Buttons>(TopicTraits<Buttons>::name, &OnButtons, opts);
            if (!sub) return i2w::Fail();
            buttons_sub_ = std::move(sub.value());
            AddTopicInfo<Buttons>();
            std::printf("[i2wRecorder] buttons: ENABLED\n");
        }
        else
        {
            std::printf("[i2wRecorder] buttons: disabled by config\n");
        }

        return i2w::Ok();
    }

    i2w::LifecycleResult OnTick() noexcept override { return i2w::Ok(); }

    template <typename T>
    void AddTopicInfo()
    {
        chrono_cap::TopicInfo info;
        info.topic_id = static_cast<std::uint16_t>(TopicTraits<T>::id);
        info.wire_size = static_cast<std::uint32_t>(TopicTraits<T>::wire_size);
        info.name = TopicTraits<T>::name;
        enabled_topics_.push_back(info);
    }

    i2w::Subscription<Pose2D> pose_sub_{};
    i2w::Subscription<Axis> axis_sub_{};
    i2w::Subscription<Buttons> buttons_sub_{};
    i2w::EndpointPlane plane_{i2w::EndpointPlane::Local};
    std::vector<chrono_cap::TopicInfo> enabled_topics_;
};

int main(int argc, char** argv)
{
    if (!g_logger_config.LoadFromFile("../config/logger_config.json"))
    {
        std::printf("[i2wRecorder] failed to load logger config\n");
        return 1;
    }

    i2w::Config config;
    config.node_name = "i2w_recorder";
    config.ns = "/demo";

    RecorderSystem system(std::move(config));
    if (!system.Setup().ok)
    {
        std::printf("[i2wRecorder] setup failed\n");
        return 1;
    }

    if (system.EnabledTopics().empty())
    {
        std::printf("[i2wRecorder] WARNING: no topics enabled -- nothing will be recorded\n");
    }

    if (!g_writer.Open(MakeSessionBaseName(), system.EnabledTopics()))
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

    g_writer.Close();
    return 0;
}
