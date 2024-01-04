#ifndef BASE_SEGMENTER_H_
#define BASE_SEGMENTER_H_

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace segmenter {
class BaseSegmenter {
 public:
  /// @brief Segment the point cloud
  /// @param cloud_in
  /// @param cloud_clusters
  virtual void segment(const pcl::PointCloud<pcl::PointXYZI>& cloud_in,
                       std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloud_clusters) = 0;

  virtual std::string name() const = 0;
};
}  // namespace segmenter

#endif
