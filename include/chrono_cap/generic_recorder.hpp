#pragma once
/// @file generic_recorder.hpp
/// @brief One recorder class that handles ANY number of topics, driven by
///        a compile-time TopicList<Ts...> and a runtime JSON config.
///
/// Adding a new topic to the recorder never requires touching this file --
/// only topic_list.hpp (add the type), topic_registry.hpp (add TopicTraits),
/// payload_codec.hpp (add Encode/DecodePayload), and logger_config.json
/// need to change. This file uses C++17 fold expressions to expand
/// "one subscribe + one push callback" per type in the pack automatically.

#include <cstdio>
#include <cstdint>
#include <tuple>
#include <utility>
#include <vector>

#include "i2w/impl.hpp"
#include "chrono_cap/chrono_cap.hpp"

namespace chrono_cap
{

/// @brief Injected by i2wRecorder.cpp's main() before Setup() is called.
/// Kept as a free function pointer pair rather than a global so this
/// header has no hidden global state of its own -- the caller supplies
/// both the config and the writer explicitly.
struct RecorderContext final
{
    logger_cfg::LoggerConfig* config{nullptr};
    RecordWriter* writer{nullptr};
};

template <typename TopicListT>
class GenericRecorderSystem;

template <typename... Ts>
class GenericRecorderSystem<TopicList<Ts...>> final : public i2w::SystemBase
{
public:
    GenericRecorderSystem(i2w::Config config, RecorderContext ctx)
        : i2w::SystemBase(std::move(config)), ctx_(ctx)
    {
    }

    const std::vector<TopicInfo>& EnabledTopics() const { return enabled_topics_; }

private:
    i2w::LifecycleResult OnSetup() noexcept override
    {
        bool ok = true;
        // Fold expression: expands to SetupOne<Ts>(ok) for every type in
        // the pack, in declaration order -- this is what makes the class
        // "generic": it never lists Pose2D/Axis/Buttons/Imu/... by name.
        (SetupOne<Ts>(ok), ...);
        return ok ? i2w::Ok() : i2w::Fail();
    }

    i2w::LifecycleResult OnTick() noexcept override { return i2w::Ok(); }

    template <typename T>
    void SetupOne(bool& ok)
    {
        using Traits = logger_msgs::TopicTraits<T>;

        if (!ctx_.config->IsEnabled<T>())
        {
            std::printf("[recorder] %-16s disabled by config\n", Traits::name);
            return;
        }

        i2w::SubscriptionOptions opts;
        opts.plane = i2w::EndpointPlane::Local;
        opts.reliability = i2w::Reliability::BestEffort;
        opts.queue_depth = 64;
        opts.overflow_policy = i2w::OverflowPolicy::DropOldest;

        RecordWriter* writer = ctx_.writer; // captured by value into the lambda below

        auto sub = runtime().subscribe<T>(
            Traits::name,
            [writer](const i2w::Sample<T>& sample) {
                std::uint8_t buf[Traits::wire_size];
                logger_wire::EncodePayload(sample.value, buf);
                writer->Push(static_cast<std::uint16_t>(Traits::id), sample.header.stamp_ns,
                             sample.header.seq, buf, sizeof(buf));
            },
            opts);

        if (!sub)
        {
            std::printf("[recorder] %-16s FAILED to subscribe\n", Traits::name);
            ok = false;
            return;
        }

        // Keep the subscription alive for the system's lifetime -- one
        // tuple slot per type in Ts..., selected by type at compile time.
        std::get<i2w::Subscription<T>>(subs_) = std::move(sub.value());

        TopicInfo info;
        info.topic_id = static_cast<std::uint16_t>(Traits::id);
        info.wire_size = static_cast<std::uint32_t>(Traits::wire_size);
        info.name = Traits::name;
        enabled_topics_.push_back(info);

        std::printf("[recorder] %-16s ENABLED\n", Traits::name);
    }

    RecorderContext ctx_;
    std::tuple<i2w::Subscription<Ts>...> subs_;
    std::vector<TopicInfo> enabled_topics_;
};

} // namespace chrono_cap
