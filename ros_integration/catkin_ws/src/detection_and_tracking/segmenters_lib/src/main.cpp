#include <iostream>
#include <string>

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "segmente_node");

  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle private_nh = ros::NodeHandle("~");

  std::string point_cloud_name = "";
  
}
