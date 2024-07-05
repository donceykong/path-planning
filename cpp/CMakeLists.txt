cmake_minimum_required(VERSION 3.10)

# Project
project(PathPlanningExamples VERSION 1.0)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find packages
find_package(octomap REQUIRED)

# Executables
add_executable(pointcloud_to_octomap src/pointcloudToOctomap.cpp)
target_link_libraries(pointcloud_to_octomap ${OCTOMAP_LIBRARIES})
target_include_directories(pointcloud_to_octomap PUBLIC ${OCTOMAP_INCLUDE_DIRS})