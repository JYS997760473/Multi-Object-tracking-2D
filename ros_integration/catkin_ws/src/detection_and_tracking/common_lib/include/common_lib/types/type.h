#pragma once
#include <string>

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