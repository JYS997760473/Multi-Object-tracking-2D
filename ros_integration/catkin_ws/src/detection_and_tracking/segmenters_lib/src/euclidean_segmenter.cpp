#include <pcl/filters/extract_indices.h>
#include <ros/ros.h>

#include "euclidean_segmenter.h"

namespace segmenter {
EuclideanSegmenter::EuclideanSegmenter() {}

EuclideanSegmenter::EuclideanSegmenter(const SegmenterParams& params)
    : params_{params}
    , kd_tree_ptr_{new pcl::search::KdTree<pcl::PointXYZI>} {
  euclidean_cluster_extractor_.setSearchMethod(kd_tree_ptr_);
  euclidean_cluster_extractor_.setMinClusterSize(params_.ec_min_cluster_size);
  euclidean_cluster_extractor_.setMaxClusterSize(params_.ec_max_cluster_size);
  euclidean_cluster_extractor_.setClusterTolerance(params_.ec_tolerance);
}

EuclideanSegmenter::~EuclideanSegmenter() {}

void EuclideanSegmenter::segment(const pcl::PointCloud<pcl::PointXYZI>& cloud_in,
                                 std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloud_clusters) {
  if (cloud_in.empty()) {
    std::cout << "Empty non-ground for segmentation, do nothing" << std::endl;
    return;
  }
  // clear segments
  cloud_clusters.clear();
}

}  // namespace segmenter
