#pragma once
#include <string>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef uint32_t IdType;


struct EIGEN_ALIGN16 PointD {
  PCL_ADD_POINT4D;
  uint8_t intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointD, (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity))

struct SegmenterParams {
  std::string segmenter_type = "EuclideanSegmenter";  // Difference of Normals segment

  // DoN segment parameters
  double don_segmenter_small_scale = 0.5;
  double don_segmenter_large_scale = 2;
  double don_segmenter_range_threshold = 0.2;
  int don_segmenter_ec_min_size = 50;
  int don_segmenter_ec_max_size = 100000;
  double don_segmenter_ec_tolerance = 1.0;

  // Euclidean segmenter parameters
  double ec_tolerance;
  int ec_max_cluster_size;
  int ec_min_cluster_size;
};

struct TrackingWorkerParams {
  //----------------- Matcher: tracker<->observed object association
  std::string matcher_method_name = "hungarian_matcher";
  float matcher_match_distance_maximum = 4.0;  // 2.2;//3.0;//4.0;
  float matcher_location_distance_weight = 0.5f;
  float matcher_direction_distance_weight = 1.5f;  // 0.2f;
  float matcher_bbox_size_distance_weight = 1.5f;  // 0.3f;
  float matcher_point_num_distance_weight = 0.1f;
  float matcher_histogram_distance_weight = 0.3f;
  float matcher_iou_distance_weight = 1.0f;

  //----------------- Tracker
  // Tracker Filter setup
  std::string filter_method_name =
      "precision_tracking_ukf";  //"precision_tracking_ukf";//"robust_kalman_filter";//"precision_tracking_filter";
  bool filter_use_adaptive = false;
  double filter_association_score_maximum = matcher_match_distance_maximum;
  float filter_measurement_noise = 0.4f;
  float filter_initial_velocity_noise = 5.0f;
  float filter_xy_propagation_noise = 10.0f;
  float filter_z_propagation_noise = 10.0f;
  float filter_breakdown_threshold_maximum = 10.0;

  // precision_tracking_ukf setup
  bool filter_use_precision_tracking = false;
  float filter_position_propagation_noise = 10.0f;
  float filter_velocity_propagation_noise = 5.0f;
  float filter_acceleration_propagation_noise = 0.2f;
  float filter_yawd_propagation_noise = 0.05f;
  float filter_xy_measure_noise = 0.5f;
  float filter_velocity_measure_noise = 0.05f;
  float filter_yaw_measure_noise = 0.32f;

  // Basic Tracker setup
  int tracker_cached_history_size_maximum = 5;
  int tracker_consecutive_invisible_maximum = 10;  // 3;
  float tracker_visible_ratio_minimum = 0.6;
  float tracker_acceleration_noise_maximum = 5;
  float tracker_speed_noise_maximum = 1.0;  // 0.4;
  float tracker_dynamic_length_maximum = 32.0;

  //----------------- Tracking Objects collect conditions
  bool tracking_use_histogram_for_match = false;
  float tracking_histogram_bin_size = 10.;
  int tracking_collect_age_minimum = 0;                    // 1;
  int tracking_collect_consecutive_invisible_maximum = 1;  // 3;//0;
  std::string svm_model_filename = "~/model/svm_param_n_class.xml";
};  // struct TrackingWorkerParams
