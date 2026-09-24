// Standalone (de-ROS'd) LT-SLAM: multi-session radar pose-graph alignment.
// Usage: ltslam_node <config.yaml>
#include "ltslam/LTslam.h"
#include "ltslam/RosParamServer.h"
#include <chrono>
#include <iostream>

int main(int argc, char** argv)
{
    std::string config = (argc > 1) ? argv[1] : "config/ltslam_params.yaml";
    setLtslamConfigPath(config);

    std::cout << "\033[1;32m----> LTslam (standalone) starts. config=" << config << "\033[0m" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    LTslam ltslam3d;
    ltslam3d.run();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "\033[1;33mElapsed time: " << elapsed.count() << " sec\033[0m" << std::endl;
    std::cout << "\033[1;32m----> LTslam done.\033[0m" << std::endl;
    return 0;
}
