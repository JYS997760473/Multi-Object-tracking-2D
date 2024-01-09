#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "common_lib/types/object.h"
#include "common_lib/types/type.h"

class BaseObjectBuilder {
 public:
  virtual void build(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloud_clusters,
                     std::vector<ObjectPtr>* objects) = 0;

  virtual void build(const ObjectPtr& object) = 0;

  virtual std::string name() const = 0;
};
