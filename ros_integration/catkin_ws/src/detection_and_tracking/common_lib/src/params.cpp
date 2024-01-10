#include <string>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>

#include "common_lib/params.h"

Params::Params(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  nh_ = nh;
  private_nh_ = private_nh;

  // segment uses euclidean segmenter
  segmenter_type_ = segmenter::SegmentType::EuclideanSegmenter;
}

Params::Params() {}

void Params::loadParams() {
  const std::string car_prefix = "";
  const std::string detect_prefix = "/detect";

  private_nh_.getParam(detect_prefix + "/pub_objt_topic", pub_objt_topic_);
  private_nh_.getParam(detect_prefix + "/sub_pc_topic", sub_pc_topic_);

  private_nh_.param(detect_prefix + "/cluster_max_length", cluster_max_length_);
  private_nh_.param(detect_prefix + "/cluster_max_velocity", cluster_max_velocity_);
  private_nh_.param(detect_prefix + "/filter_size", filter_size_);

  // segmente parameters
  segmenter_params_.ec_tolerance = 0.25;
  segmenter_params_.ec_max_cluster_size = 30000;
  segmenter_params_.ec_min_cluster_size = 5;
}
