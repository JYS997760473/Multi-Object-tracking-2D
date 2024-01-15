#pragma once
#ifndef COMMON_LIB_TYPES_OBJECT_H_
#define COMMON_LIB_TYPES_OBJECT_H_
#include <Eigen/Core>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common_lib/types/feature.h"
#include "common_lib/types/type.h"

typedef pcl::PointCloud<PointD> PolygonDType;

struct alignas(16) Object {
  //------------ basic information
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr_;  // point cloud of the object
  IdType id = -1;                                   // object id per frame
  PolygonDType polygon;                             // convex hull of the object
  int min_point_index = 0;                          // left most convexhull vertex
  int max_point_index = 0;                          // right most convexhull vertex
  int final_edge_pt_idx = 0;
  double total_len = 0;       // total length of convexhull edges facing the sensor
  double max_dist = 0;        // max len of the edge amoung the convexhull deges facing the sensor
  double hull_perimeter = 0;  // Perimeter (sum of all edges) of the convexhull
  /*
   * @note Apollo's Object Coordinate
   *          |x
   *      C   |   D-----------
   *          |              |
   *  y---------------     length
   *          |              |
   *      B   |   A-----------
   */
  // oriented boundingbox information: main direction vector(x, y, 0)
  Eigen::Vector3d direction;
  // the yaw angle, theta = 0.0 <=> direction = (1, 0, 0)
  double yaw_rad = 0.0;
  // ground center of the object (cx, cy, z_min)
  Eigen::Vector3d ground_center;
  // size of the oriented bbox, length is the size in the main direction
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;

  float score = 0.0;
  bool is_background = false;
  bool is_static = false;
  double static_prob = 0.5;
  double orientation_jitter = M_PI;

  //---------------------- tracking information
  // shape feature used for tracker-observation match
  Feature shape_features;
  /// @note one tracker maintaina tracked trajectory
  IdType tracker_id = 0;
  // tracking state
  // stable anchor_point during time, e.g., barycenter
  Eigen::Vector3d anchor_point;
  Eigen::Vector3d orientation;
  Eigen::Vector3d velocity;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d acceleration;
  // age of the tracked object
  double tracking_time = 0.0;
  double latest_tracked_time = 0.0;
  // noise covariance matrix for uncertainty of position and velocity
  Eigen::Matrix3d position_uncertainty;
  Eigen::Matrix3d velocity_uncertainty;

  float association_score = 0.0;
  bool is_valid{true};  ///< Whether the object is valid or not

  Object() {
    ground_center = Eigen::Vector3d::Zero();
    velocity = Eigen::Vector3d::Zero();
    direction = Eigen::Vector3d(1, 0, 0);
    cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    /*
     *
     * | 0.01  0    0   |
     * |  0   0.01  0   |
     * |  0    0   0.01 |
     */
    position_uncertainty << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
    velocity_uncertainty << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
  }

  void clone(const Object& rhs) {
    *this = rhs;
    this->cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZI>(*(rhs.cloud_ptr_), *cloud_ptr_);
  }

  void setId(IdType id) { this->id = id; }

  void invalidate() { is_valid = false; }

  void setValid(bool is_valid) { is_valid = is_valid; }

  bool isValid() const { return is_valid; }

  /// \brief Modify the minimum oriented bounding box to align with the given heading direction
  void adjustBoxToHeading(const Eigen::Vector2d& dir) {
    Eigen::Vector2d unit_dir = dir.normalized();
    Eigen::Vector2d unit_orth(-unit_dir[1], unit_dir[0]);

    double min_r_along_dir = std::numeric_limits<double>::max();
    double min_r_orth_dir = min_r_along_dir;
    double max_r_along_dir = std::numeric_limits<double>::lowest();
    double max_r_orth_dir = max_r_along_dir;
    for (size_t i = 0; i < polygon.points.size(); i++) {
      Eigen::Vector2d p(polygon.points[i].x, polygon.points[i].y);
      double r_along_dir = p.dot(unit_dir);
      min_r_along_dir = std::min(min_r_along_dir, r_along_dir);
      max_r_along_dir = std::max(max_r_along_dir, r_along_dir);
      double r_orth_dir = p.dot(unit_orth);
      min_r_orth_dir = std::min(min_r_orth_dir, r_orth_dir);
      max_r_orth_dir = std::max(max_r_orth_dir, r_orth_dir);
    }

    double c = unit_dir[0];  // cos(yaw)
    double s = unit_dir[1];  // sin(yaw)
    Eigen::Matrix2d rot;
    rot << c, -s, s, c;
    Eigen::Vector2d center((max_r_along_dir + min_r_along_dir) / 2.0, (max_r_orth_dir + min_r_orth_dir) / 2.0);
    center = rot * center;

    // Update box related attributes
    direction = Eigen::Vector3d(dir[0], dir[1], direction[2]);
    yaw_rad = std::atan2(dir[1], dir[0]);
    ground_center = Eigen::Vector3d(center[0], center[1], ground_center[2]);
    length = max_r_along_dir - min_r_along_dir;
    width = max_r_orth_dir - min_r_orth_dir;
  }
};

typedef std::shared_ptr<Object> ObjectPtr;
typedef std::shared_ptr<const Object> ObjectConstPtr;

#endif
