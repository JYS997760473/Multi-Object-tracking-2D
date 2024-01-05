#include <iostream>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include "common_lib/time.h"
#include "segmenters_lib/base_segmenter.h"
#include "segmenters_lib/euclidean_segmenter.h"

boost::shared_ptr<segmenter::BaseSegmenter> segmenter_;
ros::Subscriber pointcloud_sub_;

void OnLidarPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in_ptr) {
  common::Clock clock;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_in_ptr, *cloud_ptr);
  printf("Lidar Point Cloud inputs: %d points \n", cloud_ptr->size());

  std_msgs::Header header = cloud_in_ptr->header;
  header.frame_id = "base_link";
  header.stamp = ros::Time::now();

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_non_ground_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  segmenter_->segment(*cloud_ptr, cloud_clusters);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "segmente_node");

  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle private_nh = ros::NodeHandle("~");

  std::string point_cloud_name = "/lidar_top";
  segmenter_ = std::unique_ptr<segmenter::BaseSegmenter>(new segmenter::EuclideanSegmenter());
  pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(point_cloud_name, 10, OnLidarPointCloud);

  ros::Rate fps(40);
  while (ros::ok()) {
    ros::spinOnce();
    fps.sleep();
  }
  return 0;
}
