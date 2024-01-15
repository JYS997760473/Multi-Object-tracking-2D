#include <pcl/filters/extract_indices.h>
#include <ros/ros.h>

#include "common_lib/time.h"

#include "segmenters_lib/euclidean_segmenter.h"

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

  common::Clock clock;
  std::cout << "Starting Euclidean segmentation" << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  *cloud_ptr = cloud_in;

  std::vector<pcl::PointIndices> cluster_indices;

  // extract clusters
  euclidean_cluster_extractor_.setInputCloud(cloud_ptr);
  euclidean_cluster_extractor_.extract(cluster_indices);

  if (cluster_indices.size() > 0) {
    for (size_t cluster_idx = 0u; cluster_idx < cluster_indices.size(); cluster_idx++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      // extract the indices of a given point cloud as a new point cloud
      pcl::copyPointCloud(*cloud_ptr, cluster_indices[cluster_idx], *cluster_cloud_ptr);
      // append the indice
      cloud_clusters.push_back(cluster_cloud_ptr);
    }
  }

  printf("Segmentation complete, took %f ms.", clock.takeRealTime());
}

}  // namespace segmenter
