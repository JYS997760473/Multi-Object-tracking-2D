#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include "demo/demo.h"

Demo::Demo() {
  // ros NodeHandle initialization
  nh_ = ros::NodeHandle();
  private_nh_ = ros::NodeHandle("~");

  // Parameters initialization and loading
  params_ = Params(nh_, private_nh_);
  params_.loadParams();

  // initialize segmenter
  initializeSegmenter();

  // ros subscribers and publisher initialization
  pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(params_.sub_pc_topic_, params_.sub_pc_queue_size_,
                                                            Demo::OnPointCloud);  // lidar_top
}

void Demo::OnPointCloud(const sensor_msgs::PointCloud2ConstPtr ros_pc) {
  std_msgs::Header header = ros_pc->header;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_on;
  cloud_on = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*ros_pc, *cloud_on);
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
  segmenter_->segment(*cloud_on, clusters);
}

void Demo::initializeSegmenter() {
  if (params_.segmenter_type_ == segmenter::SegmentType::EuclideanSegmenter) {
    segmenter_ = std::unique_ptr<segmenter::BaseSegmenter>(new segmenter::EuclideanSegmenter(params_.segmenter_params_));
  } else {
    ROS_ERROR("Only support Euclidean Segment now");
    return;
  }
}
