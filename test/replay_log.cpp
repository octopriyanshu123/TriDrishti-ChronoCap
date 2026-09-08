#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <ctime>

int main()
{
    std::ifstream file("robot.log");

    if (!file.is_open())
    {
        std::cerr << "Failed to open robot.log\n";
        return 1;
    }

    std::string line;

    if (!std::getline(file, line))
    {
        return 1;
    }

    // Extract first timestamp
    size_t start = line.find('[');
    size_t end = line.find(']');

    if (start == std::string::npos || end == std::string::npos)
    {
        return 1;
    }

    long long firstTimestamp =
        std::stoll(line.substr(start + 1, end - start - 1));

    // This is our "time = 0"
    auto startTime = std::chrono::steady_clock::now();

    // Process first line immediately
    {
        std::time_t time = static_cast<std::time_t>(firstTimestamp);

        std::cout << "["
                  << std::put_time(std::localtime(&time),
                                   "%Y-%m-%d %H:%M:%S")
                  << "] "
                  << line.substr(end + 1)
                  << '\n';
    }

    while (std::getline(file, line))
    {
        start = line.find('[');
        end = line.find(']');

        if (start == std::string::npos || end == std::string::npos)
            continue;

        // Extract timestamp
        long long timestamp =
            std::stoll(line.substr(start + 1, end - start - 1));

        // Relative time from first log
        long long delaySeconds =
            timestamp - firstTimestamp;

        // Target time when this log should appear
        auto targetTime =
            startTime + std::chrono::seconds(delaySeconds);

        // Wait until target time
        std::this_thread::sleep_until(targetTime);

        // Convert Unix timestamp to human-readable time
        std::time_t time =
            static_cast<std::time_t>(timestamp);

        // Print converted timestamp + message
        std::cout << "["
                  << std::put_time(std::localtime(&time),
                                   "%Y-%m-%d %H:%M:%S")
                  << "] "
                  << line.substr(end + 1)
                  << '\n';
    }

    return 0;
}

