#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

int main() {
    // Determine compiler-specific C++ standard version
    long cpp_version = __cplusplus;
#ifdef _MSVC_LANG
    cpp_version = _MSVC_LANG;
#endif

    std::cout << "============================================" << std::endl;
    std::cout << "GolfSim Build Environment Verification" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "C++ Standard: " << cpp_version << " (e.g., 202002 for C++20)" << std::endl;
    std::cout << "OpenCV Version: " << CV_VERSION << std::endl;
    std::cout << "Eigen Version: " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;
    std::cout << "spdlog Version: " << SPDLOG_VER_MAJOR << "." << SPDLOG_VER_MINOR << "." << SPDLOG_VER_PATCH << std::endl;
    std::cout << "nlohmann/json Version: " << NLOHMANN_JSON_VERSION_MAJOR << "." << NLOHMANN_JSON_VERSION_MINOR << "." << NLOHMANN_JSON_VERSION_PATCH << std::endl;
    std::cout << "============================================" << std::endl;

    // Test nlohmann/json
    nlohmann::json test_json;
    test_json["status"] = "OK";
    test_json["message"] = "Build environment is fully operational!";
    std::cout << "JSON Test Output: " << test_json.dump() << std::endl;

    // Test spdlog
    spdlog::info("spdlog is working correctly.");

    // Test Eigen Matrix multiplication
    Eigen::Matrix2d mat;
    mat << 1, 2, 3, 4;
    std::cout << "Eigen Matrix multiplication test:\n" << mat * mat << std::endl;

    // Test OpenCV Matrix creation
    cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC3);
    std::cout << "OpenCV Matrix created successfully. Dimensions: " << image.rows << "x" << image.cols << std::endl;

    std::cout << "============================================" << std::endl;
    std::cout << "Verification completed successfully!" << std::endl;
    return 0;
}
