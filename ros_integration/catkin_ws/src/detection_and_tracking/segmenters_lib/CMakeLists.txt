cmake_minimum_required(VERSION 3.0.2)
project(segmenters_lib)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../" ${CMAKE_MOUDLE_PATH})
include(cpp_standard_selector)

find_package(
  catkin REQUIRED
  COMPONENTS # ROS core
  pcl_conversions
  pcl_ros
  common_lib
  # 3rd modules for ROS, like Eigen
  cmake_modules)

catkin_package(
  CATKIN_DEPENDS
  common_lib
  # DEPENDS
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}_euclidean_segment
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# Declare a C++ executable
add_library(${PROJECT_NAME}_euclidean_segment src/euclidean_segmenter.cpp)
target_link_libraries(${PROJECT_NAME}_euclidean_segment ${catkin_LIBRARIES}
${PCL_LIBRARIES})

