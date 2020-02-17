cmake_minimum_required(VERSION 3.12)
project(EKF)

# Use C++17
# SET(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export for intelliSense to find it
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Create executable
add_executable(gaussian gaussian.cpp)
add_executable(solution solution.cpp)
add_executable(MUpdate measurement_update.cpp)
add_executable(SPrediction state_prediction.cpp)
add_executable(1DKF 1Dkalmanfilter.cpp)