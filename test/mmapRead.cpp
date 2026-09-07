#include <fstream>
#include <iostream>
#include <string>

int main()
{
    std::ifstream file("robot.log");

    if (!file.is_open())
    {
        std::cerr << "Failed to open robot.log\n";
        return 1;
    }

    std::string line;

    while (std::getline(file, line))
    {
        std::cout << line << '\n';
    }

    file.close();

    return 0;
}