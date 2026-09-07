#include "logger.hpp"

#include <thread>
#include <chrono>

int main()
{
    Logger logger("robot.log");

    logger.replay();
    return 0;
}