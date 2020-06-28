cmake_minimum_required(VERSION 2.8.3)
project(test_ds)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package( OpenCV REQUIRED )

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
# )

## Declare a C++ executable
add_executable(test_ds src/test_ds.cpp)
