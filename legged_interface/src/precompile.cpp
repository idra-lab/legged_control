#include <iostream>
#include <legged_interface/LeggedInterface.h>

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: legged_precompile <taskFile> <urdfFile> <referenceFile>\n";
        return 1;
    }
    std::string taskFile = argv[1];
    std::string urdfFile = argv[2];
    std::string referenceFile = argv[3];

    std::cout << "[Precompile] Starting OCS2 CppAD compilation...\n";
    std::cout << "[Precompile] taskFile: " << taskFile << "\n";
    std::cout << "[Precompile] urdfFile: " << urdfFile << "\n";
    std::cout << "[Precompile] referenceFile: " << referenceFile << "\n";

    try {
        legged::LeggedInterface leggedInterface(taskFile, urdfFile, referenceFile);
        leggedInterface.setupOptimalControlProblem(taskFile, urdfFile, referenceFile, true);
        std::cout << "[Precompile] CppAD compilation completed successfully!\n";
    } catch (const std::exception& e) {
        std::cerr << "[Precompile] Error during compilation: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
