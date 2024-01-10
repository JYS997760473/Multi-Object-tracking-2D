cmake_minimum_required(VERSION 3.0.2)
project(demo)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../" ${CMAKE_MOUDLE_PATH})
include(cpp_standard_selector)

find_package(
  catkin REQUIRED
  COMPONENTS # ROS core
  pcl_conversions
  pcl_ros
  common_lib
  segmenters_lib
  object_builders_lib
  # 3rd modules for ROS, like Eigen
  cmake_modules)

catkin_package(
  CATKIN_DEPENDS
  # common_lib
  # segmenters_lib
  # object_builders_lib
  # DEPENDS
  INCLUDE_DIRS
#   include
  LIBRARIES)
#   common_lib)
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(demo_lib src/demo.cpp)
target_link_libraries(demo_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Declare a C++ executable
add_executable(${PROJECT_NAME} src/main.cpp src/demo.cpp)
target_link_libraries(${PROJECT_NAME} demo_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES})
