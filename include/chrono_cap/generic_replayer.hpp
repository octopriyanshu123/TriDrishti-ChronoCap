#pragma once
/// @file generic_replayer.hpp
/// @brief One replayer class that handles ANY number of topics, driven by
///        a compile-time TopicList<Ts...>. The set of topics actually
///        advertised is determined at runtime from whichever topics are
///        present in the opened .bin's metadata table.
///
/// Adding a new topic to the replayer never requires touching this file --
/// same rule as generic_recorder.hpp. TryAdvertise<T>/DispatchOne<T> are
/// expanded once per type in the pack via C++17 fold expressions.

#include <cstdio>
#include <cstdint>
#include <tuple>
#include <utility>

#include "i2w/impl.hpp"
#include "chrono_cap/chrono_cap.hpp"


namespace chrono_cap
{

template <typename TopicListT>
class GenericReplayerSystem;

template <typename... Ts>
class GenericReplayerSystem<TopicList<Ts...>> final : public i2w::SystemBase
{
public:
    using i2w::SystemBase::SystemBase;

    /// @brief Called once per topic found in the file (via RecordReader's
    /// TopicHook), before replay starts. Advertises the matching i2w
    /// publisher for whichever type in Ts... matches this topic_id.
    /// @return false if no known type matches, or advertise() fails, or
    ///         the file's recorded wire_size doesn't match this build's
    ///         registry for that type.
    bool OnTopicFound(const ResolvedTopic& topic)
    {
        bool found = false;
        (TryAdvertise<Ts>(topic, found), ...);
        if (!found)
        {
            std::fprintf(stderr, "[replayer] topic_id=%u ('%s') not in this build's TopicList\n",
                        topic.topic_id, topic.name.c_str());
        }
        return found;
    }

    /// @brief Called once per record by RecordReader::Run(). Decodes and
    /// republishes via whichever type in Ts... matches topic_id.
    void OnRecord(std::uint16_t topic_id, std::uint64_t stamp_ns, std::uint64_t /*seq*/,
                  const std::uint8_t* payload, std::uint32_t /*len*/)
    {
        (DispatchOne<Ts>(topic_id, stamp_ns, payload), ...);
    }

private:
    i2w::LifecycleResult OnSetup() noexcept override { return i2w::Ok(); }
    i2w::LifecycleResult OnTick() noexcept override { return i2w::Ok(); }

    template <typename T>
    void TryAdvertise(const ResolvedTopic& topic, bool& found)
    {
        using Traits = logger_msgs::TopicTraits<T>;
        if (topic.topic_id != static_cast<std::uint16_t>(Traits::id))
        {
            return; // not this type -- another fold-expression call may match
        }

        if (topic.wire_size != Traits::wire_size)
        {
            std::fprintf(stderr,
                        "[replayer] %s: wire_size mismatch (file=%u, this build=%zu) -- refusing\n",
                        Traits::topicName, topic.wire_size, Traits::wire_size);
            return;
        }

        i2w::PublisherOptions opts;
        opts.plane = i2w::EndpointPlane::Local;
        auto pub = runtime().advertise<T>(Traits::topicName, opts);
        if (!pub)
        {
            std::fprintf(stderr, "[replayer] %s: advertise() failed\n", Traits::topicName);
            return;
        }

        std::get<i2w::Publisher<T>>(pubs_) = std::move(pub.value());
        found = true;
        std::printf("[replayer] %-16s advertised\n", Traits::topicName);
    }

    template <typename T>
    void DispatchOne(std::uint16_t topic_id, std::uint64_t stamp_ns, const std::uint8_t* payload)
    {
        using Traits = logger_msgs::TopicTraits<T>;
        if (topic_id != static_cast<std::uint16_t>(Traits::id))
        {
            return;
        }
        T v{};
        logger_wire::DecodePayload(payload, v);
        std::get<i2w::Publisher<T>>(pubs_).publish(v, stamp_ns);
    }

    std::tuple<i2w::Publisher<Ts>...> pubs_;
};

} // namespace chrono_cap
