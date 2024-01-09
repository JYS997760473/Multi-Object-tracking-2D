#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common_lib/types/type.h"

namespace common {

const float EPSILON = 1e-9;

/**
 * @brief convert PointI cloud in indices to PointD cloud
 * @param cloud
 * @param indices
 * @param trans_cloud
 */
static void convertPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr icloud, const std::vector<int>& indices,
                              pcl::PointCloud<PointD>* dcloud) {
  if (dcloud->size() != indices.size()) {
    dcloud->resize(indices.size());
  }
  for (size_t i = 0u; i < indices.size(); ++i) {
    const pcl::PointXYZI& p = icloud->at(indices[i]);
    Eigen::Vector3d v(p.x, p.y, p.z);
    PointD& tp = dcloud->at(i);
    tp.x = v.x();
    tp.y = v.y();
    tp.z = v.z();
    tp.intensity = p.intensity;
  }
}
}  // namespace common
