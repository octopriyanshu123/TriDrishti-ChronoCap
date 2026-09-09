#pragma once
/// @file topic_registry.hpp
/// @brief Compile-time mapping: struct type <-> TopicId <-> topic name <-> wire size.
///
/// wire_size is the FIXED number of bytes this topic's payload occupies on
/// disk once encoded field-by-field (see payload_codec.hpp) -- it is NOT
/// sizeof(T). sizeof(T) can differ across compilers/platforms (struct
/// padding, alignment, int width), which would silently break replay when
/// recording on one architecture (e.g. ARM) and replaying on another
/// (e.g. x86). wire_size is a format constant, fixed forever once chosen.

#include <cstddef>
#include <cstdint>

#include "logger_types.hpp"
#include "crawler_i2w_msgs/ui/joy.hpp"

namespace logger_msgs
{

// Numeric IDs stamped into the log file for each topic/struct type.
// These values are the on-disk format -- do not reorder or reuse an
// existing value for a different type; only ever append new entries.
enum class TopicId : std::uint16_t
{
    Pose    = 0,
    Axis    = 1,
    Buttons = 2,
    Joy     = 3, 
};

// Primary template intentionally left undefined: attempting to use
// TopicTraits<T> for a type with no specialization is a compile error,
// not a silent wrong answer.
template <typename T>
struct TopicTraits;

// template <>
// struct TopicTraits<Pose2D> final
// {
//     static constexpr TopicId id       = TopicId::Pose;
//     static constexpr const char* name = "pose";
//     // 3 x float, each encoded as 4 bytes -> 12 bytes, always, on any platform.
//     static constexpr std::size_t wire_size = 12;
// };

// template <>
// struct TopicTraits<Axis> final
// {
//     static constexpr TopicId id       = TopicId::Axis;
//     static constexpr const char* name = "axis";
//     // uint64 sequence(8) + uint64 timestamp_ns(8) + int32 axes_count(4) = 20 bytes.
//     static constexpr std::size_t wire_size = 20;
// };

// template <>
// struct TopicTraits<Buttons> final
// {
//     static constexpr TopicId id       = TopicId::Buttons;
//     static constexpr const char* name = "buttons";
//     // uint64 sequence(8) + uint64 timestamp_ns(8) + int32 buttons_count(4) = 20 bytes.
//     static constexpr std::size_t wire_size = 20;
// };


template <>
struct TopicTraits<crawler_i2w_msgs::JoyMsgs> final
{
    static constexpr TopicId id       = TopicId::Joy;
    static constexpr const char* name = "joy";

    // timestamp(u64,8) + axis0(float,4) + axis2(float,4)
    // + button0,1,3,4,5,6 (bool, 1 byte each x 6) = 22 bytes.
    static constexpr std::size_t wire_size = 22;
};
} // namespace logger_msgs

