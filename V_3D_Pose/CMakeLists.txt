cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(poseEstimation)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CVV_STANDARD REQUIRED ON)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (poseEstimation poseEstimation.cpp)
target_link_libraries (poseEstimation ${PCL_LIBRARIES})
