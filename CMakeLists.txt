cmake_minimum_required(VERSION 3.20)
project(GolfSim LANGUAGES CXX)

# Configure C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Locate package dependencies (managed via vcpkg)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)

# Define the executable target
file(GLOB SOURCES "src/*.cpp")
add_executable(GolfSim ${SOURCES})

# Include directories
target_include_directories(GolfSim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link libraries and packages
target_link_libraries(GolfSim PRIVATE
    ${OpenCV_LIBS}
    Eigen3::Eigen
    spdlog::spdlog
    nlohmann_json::nlohmann_json
)

# OS-Specific API configuration
if(WIN32)
    # Link Windows Media Foundation and Winsock libraries
    target_link_libraries(GolfSim PRIVATE
        mfplat
        mfreadwrite
        mfuuid
        ws2_32
    )
endif()
