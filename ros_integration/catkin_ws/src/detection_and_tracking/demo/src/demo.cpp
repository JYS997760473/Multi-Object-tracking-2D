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

/// @brief Build object measurements from the clusters. If a cluster only contains points from the current frame, its velocity
/// will be treated as 0 and with static flag; otherwise, a velocity vector and static flag will be deduced (also tracking
/// time). Clusters of "weird shapes" will not be included in the object list. Instead, they
///        will be put in the none_obj_pts pointcloud.
/// @param clusters_ptr
/// @param objs
/// @param none_obj_pts
void Demo::buildMeasurementObjs(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters_ptr,
                                std::vector<ObjectPtr>& objs, pcl::PointCloud<pcl::PointXYZI>::Ptr none_obj_pts) {
  const int accum_que_size = params_.accum_que_size;
  objs = std::vector<ObjectPtr>(clusters_ptr.size());
  for (size_t i = 0; i < clusters_ptr.size(); i++) {
    objs[i].reset(new Object);
    objs[i]->cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    objs[i]->id = i;

    // basic staticstics of points from current and past pointclouds
    std::vector<double> x_max(accum_que_size, -DBL_MAX);
    std::vector<double> y_max(accum_que_size, -DBL_MAX);
    std::vector<double> x_min(accum_que_size, DBL_MAX);
    std::vector<double> y_min(accum_que_size, DBL_MAX);
    std::vector<double> count(accum_que_size, 0);

    const size_t pt_num = clusters_ptr[i]->points.size();
    if (accum_que_size > 0) {
      // havenot figure out yet
    } else {
      objs[i]->cloud_ptr_->points = clusters_ptr[i]->points;
    }
    objs[i]
  }
}
