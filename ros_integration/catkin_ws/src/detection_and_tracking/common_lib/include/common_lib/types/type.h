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
