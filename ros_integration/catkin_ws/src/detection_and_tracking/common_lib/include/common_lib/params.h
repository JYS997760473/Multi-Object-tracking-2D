#pragma once
#include <string>

#include <ros/ros.h>

#include "common_lib/types/segmenters_type.h"
#ifndef COMMON_LIB_PARAMS_H_
#define COMMON_LIB_PARAMS_H_

class Params {
 public:
  Params(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  Params();
  void loadParams();

 public:
  // params
  std::string pub_objt_topic_;
  std::string sub_pc_topic_;

  double cluster_max_length_;
  double cluster_max_velocity_;
  double filter_size_;

  int sub_pc_queue_size_;

  int accum_que_size = 0;

  // segment parameters
  segmenter::SegmentType segmenter_type_;
  SegmenterParams segmenter_params_;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
};
#endif