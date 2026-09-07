#include "logger.hpp"

#include <thread>
#include <chrono>

int main()
{
    Logger logger("robot.log");

    // Record logs
    logger.log("Robot Started");
    logger.log("Motor Connected");

    for (int i = 0; i < 10; ++i)
    {
        logger.log(
            "Motor speed set to " +
            std::to_string(i * 10) +
            "%"
        );

        std::this_thread::sleep_for(
            std::chrono::seconds(1)
        );
    }

    logger.log("Robot Stopped");
    logger.log("Motor Disconnected");

    return 0;
}