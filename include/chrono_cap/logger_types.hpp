#pragma once

#include <array>
#include <cstdint>

namespace logger_msgs
{

    struct Axis final
    {
        std::uint64_t sequence{0};
        std::uint64_t timestamp_ns{0};
        int axes_count{0};
    };

    struct Buttons final
    {
        std::uint64_t sequence{0};
        std::uint64_t timestamp_ns{0};
        int buttons_count{0};
    };

    struct Pose2D final
    {
        float x;
        float y;
        float yaw;
    };

} // namespace logger_msgs
