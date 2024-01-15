#pragma once

#include "object_builders/base_object_builder.h"

class MinBoxObjectBuilder : public BaseObjectBuilder {
 public:
  MinBoxObjectBuilder(double baselink_fakebaselink_length_along_x_axis);

  virtual ~MinBoxObjectBuilder();

  virtual void build(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloud_clusters, std::vector<ObjectPtr>* objects);

  virtual void build(const ObjectPtr& object);

  virtual std::string name() const { return "MinBoxObjectBuilder"; }

 protected:
  // bool buildObjects(std::vector<ObjectPtr>* objects);

  void buildObject(const ObjectPtr& object);

  void setDefaultValue(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const ObjectPtr& obj, Eigen::Vector4f* min_pt,
                       Eigen::Vector4f* max_pt);

  void computeGeometricFeature(const ObjectPtr& obj);

  void computePolygon2dxy(const ObjectPtr& obj);

  void refinePolygon2d(const ObjectPtr& obj);

  void reconstructPolygon3d(const ObjectPtr& obj);

  double computeClosenessScore(const ObjectPtr& obj, const size_t pt1, const size_t pt2, Eigen::Vector3d& center,
                               double& length, double& width, Eigen::Vector3d& dir);

  const double baselink_fakebaselink_length_along_x_axis_;
};
