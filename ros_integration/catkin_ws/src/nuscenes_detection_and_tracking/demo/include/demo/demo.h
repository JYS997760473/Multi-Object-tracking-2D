#pragma once
#ifndef DEMO_INCLUDE_DEMO_DEMO_H_
#define DEMO_INCLUDE_DEMO_DEMO_H_

#include <vector>

#include <ros/ros.h>

#include "common_lib/params.h"
#include "common_lib/types/object.h"
#include "object_builders/base_object_builder.h"
#include "segmenters_lib/base_segmenter.h"
#include "segmenters_lib/euclidean_segmenter.h"

class Demo {
 public:
  Demo(ros::NodeHandle nh, ros::NodeHandle private_nh);

 public:
  // ros sub and pub
  ros::Subscriber pointcloud_sub_;

  ros::Publisher objs_pub_;
  ros::Publisher trjs_pub_;
  ros::Publisher polygon_pub_;
  ros::Publisher tracks_pub_;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  Params params_;
  std::unique_ptr<segmenter::BaseSegmenter> segmenter_;
  boost::shared_ptr<BaseObjectBuilder> object_builder_;
  bool first_pc_msg_ = true;
  ros::Time init_time_;

 private:
  void OnPointCloud(const sensor_msgs::PointCloud2ConstPtr ros_pc);
  void initializeSegmenter();
  void buildMeasurementObjs(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters_ptr, double latest_time,
                            std::vector<ObjectPtr>& objs, pcl::PointCloud<pcl::PointXYZI>::Ptr none_obj_pts);
};
#endif
