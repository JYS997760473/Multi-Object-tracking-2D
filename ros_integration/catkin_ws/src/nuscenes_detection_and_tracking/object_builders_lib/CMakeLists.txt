cmake_minimum_required(VERSION 3.0.2)
project(object_builders_lib_nuscenes)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../" ${CMAKE_MOUDLE_PATH})
include(cpp_standard_selector)

find_package(
  catkin REQUIRED
  COMPONENTS # ROS core
  pcl_conversions
  pcl_ros
  common_lib_nuscenes
  # 3rd modules for ROS, like Eigen
  cmake_modules)

catkin_package(
  CATKIN_DEPENDS
  common_lib_nuscenes
  # DEPENDS
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME})
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# Declare a C++ executable
add_library(${PROJECT_NAME} src/min_box_object_builder.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
