#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime>
#include <thread>
#include <iomanip>

void log(const std::string &message)
{
    std::ofstream file("robot.log", std::ios::app);

    if (!file.is_open())
        return;

    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);

    std::cout << "[" << time
              << "] " << message << '\n';

    file << "["
         << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S")
         << "] "
         << message
         << '\n';
}

int main()
{
    log("Robot Started");
    log("Motor Connected");
    for (int i = 0; i < 10; ++i)
    {
        log("Motor speed set to " + std::to_string(i * 10) + "%");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    log("Robot Stopped");
    log("Motor Disconnected");
}