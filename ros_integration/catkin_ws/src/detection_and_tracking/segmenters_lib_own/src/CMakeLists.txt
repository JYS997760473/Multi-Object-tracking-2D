cmake_minimum_required(VERSION 3.0.2)
project(detection)

find_package(
    catkin REQUIRED
    COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    pcl_conversions
    pcl_conversions
)

catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    common_lib
)

# include(pcl_finder)
    
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node main.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRA})

install(
  TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})