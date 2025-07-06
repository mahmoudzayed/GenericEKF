cmake_minimum_required(VERSION 3.14)
project(MultithreadedSearchUtility)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(inc)

# Source files
set(SRC_FILES
    src/ExtendedKalmanFilter.cpp
    src/CVMode2D.cpp
    src/XYMeasurementModel.cpp
    src/main.cpp
)

# Add the main executable
add_executable(KalmanFilter ${SRC_FILES})