cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCLSegmentationTest)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PCLSegmentationTest}/includes)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(SegmentationAlgorithms source/RegionGrowingSegAlgorithm.cpp)

add_executable (main source/main.cpp)
target_link_libraries (main ${PCL_LIBRARIES} SegmentationAlgorithms)
