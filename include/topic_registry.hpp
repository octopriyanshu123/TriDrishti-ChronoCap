#pragma once

#include <cstddef>
#include <cstdint>

#include "logger_types.hpp"

namespace logger_msgs
{

// Numeric IDs stamped into the log file for each topic/struct type.
// These values are the on-disk format — do not reorder or reuse an
// existing value for a different type; only ever append new entries.
enum class TopicId : std::uint16_t
{
    Pose    = 0,
    Axis    = 1,
    Buttons = 2,
};

// Primary template intentionally left undefined: attempting to use
// TopicTraits<T> for a type with no specialization is a compile error,
// not a silent wrong answer.
template <typename T>
struct TopicTraits;

template <>
struct TopicTraits<Pose2D> final
{
    static constexpr TopicId id           = TopicId::Pose;
    static constexpr const char* name     = "pose";
    static constexpr std::size_t wire_size = sizeof(Pose2D);
};

template <>
struct TopicTraits<Axis> final
{
    static constexpr TopicId id           = TopicId::Axis;
    static constexpr const char* name     = "axis";
    static constexpr std::size_t wire_size = sizeof(Axis);
};

template <>
struct TopicTraits<Buttons> final
{
    static constexpr TopicId id           = TopicId::Buttons;
    static constexpr const char* name     = "buttons";
    static constexpr std::size_t wire_size = sizeof(Buttons);
};

} // namespace logger_msgs