#pragma once
/// @file topic_list.hpp
/// @brief The single place where a new topic is added to the generic
///        Recorder/Replayer's compile-time knowledge.
///
/// To add a topic:
///   1. #include its message header below.
///   2. Add the type to AllTopics.
///   3. Add a TopicTraits<T> specialization in topic_registry.hpp.
///   4. Add EncodePayload/DecodePayload in payload_codec.hpp.
///   5. Add an entry in logger_config.json.
/// generic_recorder.hpp / generic_replayer.hpp never need to change.

// #include "logger_types.hpp" // Pose2D, Axis, Buttons -- demo/test types

#include "crawler_i2w_msgs/robot/diff_drive_odometry.hpp"
#include "crawler_i2w_msgs/dwe_camera.hpp"
#include "crawler_i2w_msgs/hbk_imu.hpp"
#include "crawler_i2w_msgs/ui/joy.hpp"
#include "crawler_i2w_msgs/pose_estimate.hpp"
#include "crawler_i2w_msgs/robot/imu.hpp"
#include "crawler_i2w_msgs/robot/motor_status.hpp"
#include "crawler_i2w_msgs/tank_pose.hpp"
// ... one #include per message type you want recordable

template <typename... Ts>
struct TopicList
{
};

/// The full set of topics this build's Recorder/Replayer knows about.
/// Every type listed here MUST have a matching TopicTraits<T>
/// specialization (topic_registry.hpp) and EncodePayload/DecodePayload
/// pair (payload_codec.hpp), or this will fail to compile.
using AllTopics = TopicList<
    // logger_msgs::Pose2D,
    // logger_msgs::Axis,
    // logger_msgs::Buttons,

    // crawler_i2w_msgs::I2wDiffDriveOdometry,
    crawler_i2w_msgs::JoyMsgs
    >;
