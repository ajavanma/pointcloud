#include <iostream>
#include <filesystem>

int main()
{
    std::filesystem::path current_working_dir = std::filesystem::current_path();
    std::cout << "Current working directory: " << current_working_dir << std::endl;
    return 0;
}
