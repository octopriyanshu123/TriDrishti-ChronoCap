#include "logger.hpp"

#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>

Logger::Logger(const std::string& filename)
    : filename(filename)
{
}

void Logger::log(const std::string& message)
{
    std::ofstream file(filename, std::ios::app);

    if (!file.is_open())
    {
        std::cerr << "Failed to open log file\n";
        return;
    }

    auto now = std::chrono::system_clock::now();

    std::time_t time =
        std::chrono::system_clock::to_time_t(now);

    // Print to terminal
    std::cout << "[" << time << "] "
              << message << '\n';

    // Write to file
    file << "[" << time << "] "
         << message << '\n';
}

void Logger::replay()
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Failed to open log file\n";
        return;
    }

    std::string line;

    // Read first line
    if (!std::getline(file, line))
    {
        return;
    }

    // Find '[' and ']'
    size_t start = line.find('[');
    size_t end   = line.find(']');

    if (start == std::string::npos ||
        end == std::string::npos)
    {
        std::cerr << "Invalid log format\n";
        return;
    }

    // Extract first timestamp
    long long firstTimestamp =
        std::stoll(
            line.substr(
                start + 1,
                end - start - 1
            )
        );

    // Replay starts now
    auto replayStart =
        std::chrono::steady_clock::now();

    // First log is printed immediately
    std::cout << line << '\n';

    // Read remaining lines
    while (std::getline(file, line))
    {
        start = line.find('[');
        end   = line.find(']');

        if (start == std::string::npos ||
            end == std::string::npos)
        {
            continue;
        }

        // Extract timestamp
        long long timestamp =
            std::stoll(
                line.substr(
                    start + 1,
                    end - start - 1
                )
            );

        // Time relative to first log
        long long delaySeconds =
            timestamp - firstTimestamp;

        // When this message should be printed
        auto targetTime =
            replayStart +
            std::chrono::seconds(delaySeconds);

        // Wait until target time
        std::this_thread::sleep_until(targetTime);

        // Print log
        std::cout << line << '\n';
    }
}