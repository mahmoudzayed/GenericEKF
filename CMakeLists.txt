cmake_minimum_required(VERSION 3.13.3)
project(MultithreadedSearchUtility)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(inc)

# Find Eigen library
find_package(Eigen3 REQUIRED)

# Include Eigen directories
include_directories(${EIGEN3_INCLUDE_DIR})

# Ensure system include directories are added for standard headers
include_directories(${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})

# Source files
set(SRC_FILES
    src/CVMode2D.cpp
    src/XYMeasurementModel.cpp
    src/main.cpp
)

# Add the main executable
add_executable(KalmanFilter ${SRC_FILES})
