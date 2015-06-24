cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Region_growing_select_object)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (Region_growing_select_object Region_growing_select_object.cpp)
target_link_libraries (Region_growing_select_object ${PCL_LIBRARIES})
