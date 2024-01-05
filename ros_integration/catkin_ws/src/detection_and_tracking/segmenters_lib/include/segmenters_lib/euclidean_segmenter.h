#ifndef EUCLIDEAN_SEGMENTER_H_
#define EUCLIDEAN_SEGMENTER_H_

#include <string>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "common_lib/types/type.h"

#include "base_segmenter.h"

namespace segmenter {
class EuclideanSegmenter : public BaseSegmenter {
 public:
  EuclideanSegmenter();
  explicit EuclideanSegmenter(const SegmenterParams& params);
  ~EuclideanSegmenter();

  virtual void segment(const pcl::PointCloud<pcl::PointXYZI>& cloud_in,
                       std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloud_clusters) override;

  virtual std::string name() const { return "EuclideanSegmenter"; }

 private:
  SegmenterParams params_;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree_ptr_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclidean_cluster_extractor_;
};
}  // namespace segmenter

#endif
