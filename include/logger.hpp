#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <string>
#include <fstream>
#include <iostream>

class Logger
{
private:
    std::string filename;

public:
    explicit Logger(const std::string& filename = "robot.log");

    void log(const std::string& message);

    void replay();
};

#endif