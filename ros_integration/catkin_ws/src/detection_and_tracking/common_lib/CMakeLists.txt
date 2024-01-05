cmake_minimum_required(VERSION 3.0.2)
project(common_lib)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../" ${CMAKE_MOUDLE_PATH})
include(cpp_standard_selector)

find_package(
  catkin REQUIRED
  COMPONENTS # ROS core
  roscpp
  roslib

  # ROS messages
  std_msgs
  sensor_msgs

  # ROS PCL
  pcl_conversions
  pcl_ros

  # ROS tf
  tf
  tf_conversions

  # 3rd modules for ROS, like Eigen
  cmake_modules)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  roslib
  tf
  tf_conversions

  # DEPENDS
  INCLUDE_DIRS
  include

  LIBRARIES)

# include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
# message(${catkin_INCLUDE_DIRS})
