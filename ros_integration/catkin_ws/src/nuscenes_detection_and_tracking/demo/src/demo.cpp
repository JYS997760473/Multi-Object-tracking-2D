#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include "demo/demo.h"
#include "object_builders/object_builder_manager.h"

Demo::Demo(ros::NodeHandle nh, ros::NodeHandle private_nh) {
  // ros NodeHandle initialization
  nh_ = nh;
  private_nh_ = private_nh;
  ROS_INFO("Start Demo");
  // Parameters initialization and loading
  params_ = Params(nh_, private_nh_);
  params_.loadParams();

  // initialize segmenter
  initializeSegmenter();

  // initialize object_builder
  object_builder_ = createObjectBuilder(params_.baselink_fakebaselink_length_along_x_axis);

  // ros subscribers and publisher initialization
  ROS_INFO("Lidar topic: %s", params_.sub_pc_topic_.c_str());
  pointcloud_sub_ = nh_.subscribe(params_.sub_pc_topic_, params_.sub_pc_queue_size_, &Demo::OnPointCloud, this);  // lidar_top
}

void Demo::OnPointCloud(const sensor_msgs::PointCloud2ConstPtr ros_pc) {
  if (first_pc_msg_) {
    ROS_INFO("Receive Lidar point cloud");
    init_time_ = ros::Time::now();
    first_pc_msg_ = false;
    return;
  }
  std_msgs::Header header = ros_pc->header;
  double header_time = header.stamp.toSec() - init_time_.toSec();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_on;
  cloud_on = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*ros_pc, *cloud_on);
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_total_in_baselink_clusters;
  segmenter_->segment(*cloud_on, cloud_total_in_baselink_clusters);

  std::vector<ObjectPtr> meas_objs;
  pcl::PointCloud<pcl::PointXYZI>::Ptr none_obj_pts(new pcl::PointCloud<pcl::PointXYZI>);
  buildMeasurementObjs(cloud_total_in_baselink_clusters, header_time, meas_objs, none_obj_pts);

  // common::publishMeasurementObjMarkers(dbg_meas_objs_pub_, header, meas_objs, color, 0.5);
}

void Demo::initializeSegmenter() {
  if (params_.segmenter_type_ == segmenter::SegmentType::EuclideanSegmenter) {
    segmenter_ = std::unique_ptr<segmenter::BaseSegmenter>(new segmenter::EuclideanSegmenter(params_.segmenter_params_));
  } else {
    ROS_ERROR("Only support Euclidean Segment now");
    return;
  }
}

// /// \brief calculate `angle_front_lidar` and `d_2_nearest_lidar` features for SVM
// void getAdditionalFeatures(const ObjectPtr& obj) {
//   // calculate position of box center in 3xM1 LiDARs
//   pcl::PointXYZI center;
//   center.x = obj->ground_center[0];
//   center.y = obj->ground_center[1];
//   center.z = obj->ground_center[2];

//   pcl::PointXYZI center_m1_front(center);
//   autosense::common::transform::transformPoint<PointI>(baselink_2_m1_front, &center_m1_front);
//   pcl::PointXYZI center_m1_left(center);
//   autosense::common::transform::transformPoint<PointI>(baselink_2_m1_left, &center_m1_left);
//   pcl::PointXYZI center_m1_right(center);
//   autosense::common::transform::transformPoint<PointI>(baselink_2_m1_right, &center_m1_right);

//   float d_min;
//   const float angle_x = std::atan2(center_m1_front.y, center_m1_front.x);

//   if (angle_x > M_PI / 3.0) {  // M1_left -- [M_PI/3, M_PI]
//     d_min = std::sqrt(center_m1_left.x * center_m1_left.x + center_m1_left.y * center_m1_left.y);
//   } else if (angle_x > -M_PI / 3.0)  // M1_front -- [-M_PI/3, M_PI/3]
//   {
//     d_min = std::sqrt(center_m1_front.x * center_m1_front.x + center_m1_front.y * center_m1_front.y);
//   } else {  // M1_right -- [-M_PI, -M_PI/3]
//     d_min = std::sqrt(center_m1_right.x * center_m1_right.x + center_m1_right.y * center_m1_right.y);
//   }
//   obj->angle_front_lidar = angle_x;
//   obj->d_2_nearest_lidar = d_min;

//   return;
// }

/// @brief Build object measurements from the clusters. If a cluster only contains points from the current frame, its velocity
/// will be treated as 0 and with static flag; otherwise, a velocity vector and static flag will be deduced (also tracking
/// time). Clusters of "weird shapes" will not be included in the object list. Instead, they
///        will be put in the none_obj_pts pointcloud.
/// @param clusters_ptr
/// @param objs
/// @param none_obj_pts
void Demo::buildMeasurementObjs(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters_ptr, double latest_time,
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
    object_builder_->build(objs[i]);

    // getAdditionalFeatures(objs[i]);

    if (accum_que_size == 0) {
      objs[i]->velocity[0] = 0.0;
      objs[i]->velocity[1] = 0.0;
      objs[i]->is_static = true;
      objs[i]->tracking_time = 0.0;
      continue;
    }

    // std::vector<double> x_mean(accum_que_size, 0);
    // std::vector<double> y_mean(accum_que_size, 0);
    // for (int k = accum_que_size - 1; k >= 0; k--) {
    //   x_mean[k] = 0.5 * (x_max[k] + x_min[k]);
    //   y_mean[k] = 0.5 * (y_max[k] + y_min[k]);
    // }

    // double vx = 0.0, vy = 0.0;
    // double count_valid = 0.0;
    // for (int k = accum_que_size - 1; k >= (accum_que_size / 3); k--) {
    //   if (fabs(count[k] - 0.0) < common::EPSILON) {
    //     continue;
    //   } else {
    //     double dt = 0.1 * (double)k;
    //     vx += ((x_mean[0] - x_mean[k]) / dt);
    //     vy += ((y_mean[0] - y_mean[k]) / dt);
    //     count_valid += 1.0;
    //   }
    // }
    // if (count[0] > 0.0 && count_valid > 0.0) {
    //   vx = vx / count_valid;
    //   vy = vy / count_valid;

    //   objs[i]->velocity[0] = vx;
    //   objs[i]->velocity[1] = vy;
    //   if (pow(vx * vx + vy * vy, 0.5) > 1.0) {
    //     objs[i]->is_static = false;
    //   } else {
    //     objs[i]->is_static = true;
    //     objs[i]->velocity[0] = 0.0;
    //     objs[i]->velocity[1] = 0.0;
    //   }
    // } else {
    //   objs[i]->velocity[0] = 0.0;
    //   objs[i]->velocity[1] = 0.0;
    //   objs[i]->is_static = true;
    // }
    // for (size_t k = 0; k < count.size(); k++) {
    //   if (count[k] > 0) {
    //     objs[i]->tracking_time = k * 0.1;
    //   }
    // }
  }
}
