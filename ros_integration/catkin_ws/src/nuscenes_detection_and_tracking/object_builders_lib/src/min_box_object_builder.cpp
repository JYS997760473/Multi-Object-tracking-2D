#include <vector>

#include <pcl/common/common.h>

#include "common_lib/algos/convex_hullxy.hpp"
#include "common_lib/common.hpp"
#include "common_lib/geometry.hpp"
#include "object_builders/min_box_object_builder.h"

__attribute__((always_inline)) inline double cross_product2d(const PointD& a, const PointD& b, const double offsetx) {
  return ((a.x + offsetx) * b.y - (b.x + offsetx) * a.y);
}

__attribute__((always_inline)) inline size_t increment_mod(const size_t val, const size_t mod) {
  return (val + 1 == mod) ? 0 : (val + 1);
}

__attribute__((always_inline)) inline double cross_product2d(const PointD& a, const PointD& b) {
  return cross_product2d(a, b, 0.0);
}

MinBoxObjectBuilder::MinBoxObjectBuilder(const double baselink_fakebaselink_length_along_x_axis)
    : baselink_fakebaselink_length_along_x_axis_(baselink_fakebaselink_length_along_x_axis) {}

void MinBoxObjectBuilder::build(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloud_clusters,
                                std::vector<ObjectPtr>* objects) {
  if (objects == nullptr) {
    return;
  }

  (*objects).clear();

  for (size_t idx = 0u; idx < cloud_clusters.size(); ++idx) {
    ObjectPtr obj(new Object);
    *(obj->cloud_ptr_) = *(cloud_clusters[idx]);
    buildObject(obj);
    (*objects).push_back(obj);
  }
}

void MinBoxObjectBuilder::build(const ObjectPtr& object) { buildObject(object); }

MinBoxObjectBuilder::~MinBoxObjectBuilder() {}

// object: 当前帧检测到的一个障碍物: 障碍物分类/包含的点云等信息
/*
    // convex hull of the object
    PolygonDType polygon;

    // oriented boundingbox information
    // main direction
    Eigen::Vector3d direction;
    // the yaw angle, theta = 0.0 <=> direction = (1, 0, 0)
    double theta = 0.0;
    // ground center of the object (cx, cy, z_min)
    Eigen::Vector3d center;
    // size of the oriented bbox, length is the size in the main direction
    double length = 0.0;
    double width = 0.0;
    double height = 0.0;
 */
// built took ~3.5ms for one object
void MinBoxObjectBuilder::buildObject(const ObjectPtr& object) {
  computeGeometricFeature(object);
  /*
   * @note Apollo's Object Coordinate
   *              |x
   *              |
   *              |
   *              |
   *              |
   * y-------------
   * @note Apollo's yaw: direction is a 2d vector from ground center
   * if (fabs(obj->direction[0]) < DBL_MIN) {
          obj->theta = obj->direction(1) > 0 ? M_PI / 2 : -M_PI / 2;
      } else {
          obj->theta = atan2(obj->direction[1], obj->direction[0]);
      }
   */
  Eigen::Vector3d coord_dir(1.0, 0.0, 0.0);
  object->yaw_rad = common::geometry::computeTheta2dXyBetweenVectors<Eigen::Vector3d>(coord_dir, object->direction);
}

void MinBoxObjectBuilder::refinePolygon2d(const ObjectPtr& obj) {
  size_t polygon_vertice_num = obj->polygon.points.size();
  if (polygon_vertice_num <= 0) {
    return;
  }

  // <1> Find the beginning vertice.
  // Given polygon lines: p0----p1----p2 , calculate the angle between vector <p0p1> and <p1p2>.
  // If the angle is large, such as greater than 5 degree, the vertice "p1" will not be in the
  // middle of any line segment of the polygon, so choose "p1" as start vertice is suitable.
  int start_index = 0;
  for (size_t i = 0; i < polygon_vertice_num; ++i) {
    int i1 = (i + 1) % polygon_vertice_num;
    int i2 = (i + 2) % polygon_vertice_num;

    Eigen::Vector3d p0, p1, p2;
    p0[0] = obj->polygon.points[i].x;
    p0[1] = obj->polygon.points[i].y;
    p0[2] = obj->polygon.points[i].z;
    p1[0] = obj->polygon.points[i1].x;
    p1[1] = obj->polygon.points[i1].y;
    p1[2] = obj->polygon.points[i1].z;
    p2[0] = obj->polygon.points[i2].x;
    p2[1] = obj->polygon.points[i2].y;
    p2[2] = obj->polygon.points[i2].z;

    Eigen::Vector3d line_p0_p1, line_p1_p2;
    line_p0_p1 = p1 - p0;
    line_p1_p2 = p2 - p1;

    double angle = common::geometry::computeTheta2dXyBetweenVectors<Eigen::Vector3d>(line_p0_p1, line_p1_p2);
    if (fabs(angle) > M_PI * 5. / 180.)  // [0, PI]
    {
      start_index = i1;
      break;
    }
  }
  // printf("start_index: %d | polygon_vertice_num: %d\n", start_index, polygon_vertice_num);

  // <2> Find vertices who can be removed and have very little effects on the final ground box.
  // Given polygon lines: p0----p1----p*----p2''----p2'----p2, calculate angle "angle_p1_p0_p2"
  // between vector <p0p1> and <p0p2> and "adjacent_angle" between vector <p2''p2'> and <p2'p2>.
  // The vertice <p2'> and <p2''> are the first and second point from <p2> in anti-clockwise
  // direction.
  // If "angle_p1_p0_p2" or "adjacent_angle" is large, such as greater than 5 degree, vertice
  // "p2'" will be a vertice in the new polygon. Otherwise, the vertice is considered on the
  // line segment connecting two points <p0> and <p2>.
  std::vector<size_t> drop_points_index;
  for (size_t i = start_index, visited_count = 0; visited_count < polygon_vertice_num;) {
    int i1 = (i + 1) % polygon_vertice_num;

    Eigen::Vector3d p0, p1, line_p0_p1;
    p0[0] = obj->polygon.points[i].x;
    p0[1] = obj->polygon.points[i].y;
    p0[2] = obj->polygon.points[i].z;
    p1[0] = obj->polygon.points[i1].x;
    p1[1] = obj->polygon.points[i1].y;
    p1[2] = obj->polygon.points[i1].z;
    line_p0_p1 = p1 - p0;

    size_t end_index = -1;
    for (size_t i2 = (i + 2) % polygon_vertice_num, count_i2 = 0; count_i2 < polygon_vertice_num;
         count_i2++, i2 = (i2 + 1) % polygon_vertice_num) {
      int i2_before_1 = (i2 + polygon_vertice_num - 1) % polygon_vertice_num;
      int i2_before_2 = (i2 + polygon_vertice_num - 2) % polygon_vertice_num;

      Eigen::Vector3d p2, p2_before_1, p2_before_2;
      p2[0] = obj->polygon.points[i2].x;
      p2[1] = obj->polygon.points[i2].y;
      p2[2] = obj->polygon.points[i2].z;
      p2_before_1[0] = obj->polygon.points[i2_before_1].x;
      p2_before_1[1] = obj->polygon.points[i2_before_1].y;
      p2_before_1[2] = obj->polygon.points[i2_before_1].z;
      p2_before_2[0] = obj->polygon.points[i2_before_2].x;
      p2_before_2[1] = obj->polygon.points[i2_before_2].y;
      p2_before_2[2] = obj->polygon.points[i2_before_2].z;

      Eigen::Vector3d line_p0_p2, line1, line2;
      line_p0_p2 = p2 - p0;
      line1 = p2_before_1 - p2;
      line2 = p2_before_2 - p2_before_1;

      double angle_p1_p0_p2 = common::geometry::computeTheta2dXyBetweenVectors<Eigen::Vector3d>(line_p0_p1, line_p0_p2);
      double adjacent_angle = common::geometry::computeTheta2dXyBetweenVectors<Eigen::Vector3d>(line1, line2);
      // printf("i: %d, i1: %d, i2: %d, angle_p1_p0_p2: %f, adjacent_angle: %f ----",
      //        i, i1, i2, angle_p1_p0_p2, adjacent_angle);
      if ((fabs(angle_p1_p0_p2) > M_PI * 5. / 180.) || (fabs(adjacent_angle) > M_PI * 5. / 180.)) {
        end_index = i2_before_1;
        // printf("end_index: %d\n", end_index);
        break;
      } else {
        // printf("push back: %d\n", i2_before_1);
        drop_points_index.push_back(i2_before_1);
      }
    }

    if (end_index == (i + 1) % polygon_vertice_num) {
      visited_count++;
      i = (i + 1) % polygon_vertice_num;
    } else {
      visited_count += ((end_index - i + polygon_vertice_num) % polygon_vertice_num);
      i = end_index;
    }
  }

  // <3> Update polygon
  PolygonDType new_polygon;
  for (size_t i = 0; i < polygon_vertice_num; ++i) {
    auto iter = std::find(drop_points_index.begin(), drop_points_index.end(), i);
    if (iter == drop_points_index.end()) {
      new_polygon.points.push_back(obj->polygon.points[i]);
      // printf("[new_polygon] vertive: %d\n", i);
    }
  }
  obj->polygon = new_polygon;
}

/**
 * @brief Build object obj
 *  computePolygon2dxy(obj)
 *  reconstructPolygon3d(obj)
 * @param obj
 */
void MinBoxObjectBuilder::computeGeometricFeature(const ObjectPtr& obj) {
  // 得到最低点(min_pt.z) 上的2D边框: PolygonDType
  computePolygon2dxy(obj);
  // 更新polygon，将近似在一条直线上的中间顶点删掉，使ground box更稳定
  refinePolygon2d(obj);
  // 障碍物3D boundingbox(航向角表示方向/地表中心点+长宽高表示空间位置)
  reconstructPolygon3d(obj);
}

/**
 * @brief 得到最低点(min_pt.z) 上的2D多边形边框,构建点云分割多边形凸包
 * PolygonDType polygon;
 * @result
 *  ObjectPtr->height = max_z - min_z
 *  ObjectPtr->polygon Convex Hull of the object(物体凸边框) projected into
 * xy-plane
 */
void MinBoxObjectBuilder::computePolygon2dxy(const ObjectPtr& obj) {
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = obj->cloud_ptr_;

  setDefaultValue(cloud, obj, &min_pt, &max_pt);

  if (cloud->points.size() < 4u /* unsigned 4 */) {
    return;
  }
  // 直接获取点云 min_pt.x/y/z; max_pt.x/y/z
  pcl::getMinMax3D(*cloud, min_pt, max_pt);  // pcl/common/common.h

  // 高度信息
  obj->height = static_cast<double>(max_pt[2]) - static_cast<double>(min_pt[2]);

  // epsilon: the difference between 1.0 and the next value representable by
  // the
  // floating-point type
  // const double min_eps = 10 * std::numeric_limits<double>::epsilon();
  const double min_eps = 1e-3;
  // double min_eps = 0.1;
  // if ((max_pt[0] - min_pt[0]) < min_eps) {
  //     _cloud->points[0].x += min_eps;
  // }
  // if ((max_pt[1] - min_pt[1]) < min_eps) {
  //     _cloud->points[0].y += min_eps;
  // }

  const double diff_x = cloud->points[1].x - cloud->points[0].x;
  const double diff_y = cloud->points[1].y - cloud->points[0].y;
  size_t idx = 0;
  for (idx = 2; idx < cloud->points.size(); ++idx) {
    const double tdiff_x = cloud->points[idx].x - cloud->points[0].x;
    const double tdiff_y = cloud->points[idx].y - cloud->points[0].y;
    if (fabs(diff_x * tdiff_y - tdiff_x * diff_y) > min_eps) {
      break;
    }
  }
  if (idx >= cloud->points.size()) {
    cloud->points[0].x += min_eps;
    cloud->points[0].y += min_eps;
    cloud->points[1].x -= min_eps;
  }
  // Project point cloud into xy-plane

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZI>);
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    pcl::PointXYZI p = cloud->points[i];
    p.z = min_pt[2];
    cloud_xy->push_back(p);
  }

  auto ii_begin = begin(cloud_xy->points);
  auto ii_end = end(cloud_xy->points);

  while (ii_begin != ii_end) {
    ii_end = std::remove_if(std::next(ii_begin), ii_end, [&](const pcl::PointXYZI& p) {
      const double x_diff = ii_begin->x - p.x;
      const double y_diff = ii_begin->y - p.y;
      const double dist_pt = x_diff * x_diff + y_diff * y_diff;

      return dist_pt < 1.0e-14;
    });

    ++ii_begin;
  }

  cloud_xy->points.erase(ii_end, end(cloud_xy->points));

  // get polygon if filtered point num < 4
  if (cloud_xy->points.size() < 4) {
    obj->polygon.points.resize(4);
    obj->polygon.points[0].x = static_cast<double>(min_pt[0]);
    obj->polygon.points[0].y = static_cast<double>(min_pt[1]);
    obj->polygon.points[0].z = static_cast<double>(min_pt[2]);
    obj->polygon.points[1].x = static_cast<double>(max_pt[0]);
    obj->polygon.points[1].y = static_cast<double>(min_pt[1]);
    obj->polygon.points[1].z = static_cast<double>(min_pt[2]);
    obj->polygon.points[2].x = static_cast<double>(max_pt[0]);
    obj->polygon.points[2].y = static_cast<double>(max_pt[1]);
    obj->polygon.points[2].z = static_cast<double>(min_pt[2]);
    obj->polygon.points[3].x = static_cast<double>(min_pt[0]);
    obj->polygon.points[3].y = static_cast<double>(max_pt[1]);
    obj->polygon.points[3].z = static_cast<double>(min_pt[2]);

    return;
  }

  // Get the Convex Hull of the xy-plane point cloud
  // TODO(gary): Apollo's pcl::ConvexHull is memory-efficient?
  common::algos::ConvexHull2DXY<pcl::PointXYZI> hull;
  // pcl::ConvexHull<PointI> hull;
  hull.setInputCloud(cloud_xy);
  hull.setDimension(2);
  std::vector<pcl::Vertices> poly_vt;
  pcl::PointCloud<pcl::PointXYZI>::Ptr plane_hull(new pcl::PointCloud<pcl::PointXYZI>);
  hull.Reconstruct2dxy(plane_hull, &poly_vt);

  // a valid Convex Polygon
  if (poly_vt.size() == 1u) {
    std::vector<int> indices(poly_vt[0].vertices.begin(), poly_vt[0].vertices.end());
    // Get Polygon vertices cloud from Convex Hull cloud
    common::convertPointCloud(plane_hull, indices, &obj->polygon);
    // cannot find a valid Convex Polygon
  } else {
    /*
     * 2D bounding box
     *                  |x
     *      C       D   |
     *                  |
     *                  |
     *      B       A   |
     * y----------------|
     */
    obj->polygon.points.resize(4);
    obj->polygon.points[0].x = static_cast<double>(min_pt[0]);
    obj->polygon.points[0].y = static_cast<double>(min_pt[1]);
    obj->polygon.points[0].z = static_cast<double>(min_pt[2]);

    obj->polygon.points[1].x = static_cast<double>(min_pt[0]);
    obj->polygon.points[1].y = static_cast<double>(max_pt[1]);
    obj->polygon.points[1].z = static_cast<double>(min_pt[2]);

    obj->polygon.points[2].x = static_cast<double>(max_pt[0]);
    obj->polygon.points[2].y = static_cast<double>(max_pt[1]);
    obj->polygon.points[2].z = static_cast<double>(min_pt[2]);

    obj->polygon.points[3].x = static_cast<double>(max_pt[0]);
    obj->polygon.points[3].y = static_cast<double>(min_pt[1]);
    obj->polygon.points[3].z = static_cast<double>(min_pt[2]);
  }
}

void MinBoxObjectBuilder::setDefaultValue(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const ObjectPtr& obj,
                                          Eigen::Vector4f* min_pt, Eigen::Vector4f* max_pt) {
  pcl::getMinMax3D(*cloud, *min_pt, *max_pt);
  Eigen::Vector3f center(((*min_pt)[0] + (*max_pt)[0]) / 2, ((*min_pt)[1] + (*max_pt)[1]) / 2,
                         ((*min_pt)[2] + (*max_pt)[2]) / 2);

  // handle degeneration case 退化情况
  float epslin = 1e-3;
  for (int i = 0; i < 3; i++) {
    if ((*max_pt)[i] - (*min_pt)[i] < epslin) {
      (*max_pt)[i] = center[i] + epslin / 2;
      (*min_pt)[i] = center[i] - epslin / 2;
    }
  }

  // length
  obj->length = (*max_pt)[0] - (*min_pt)[0];
  // width
  obj->width = (*max_pt)[1] - (*min_pt)[1];
  if (obj->length - obj->width < 0) {
    float tmp = obj->length;
    obj->length = obj->width;
    obj->width = tmp;
    obj->direction = Eigen::Vector3d(0.0, 1.0, 0.0);
  } else {
    obj->direction = Eigen::Vector3d(1.0, 0.0, 0.0);
  }
  // height
  obj->height = (*max_pt)[2] - (*min_pt)[2];
  // center
  obj->ground_center =
      Eigen::Vector3d(((*max_pt)[0] + (*min_pt)[0]) / 2, ((*max_pt)[1] + (*min_pt)[1]) / 2, ((*max_pt)[2] + (*min_pt)[2]) / 2);
  // polygon
  if (cloud->size() < 4) {
    obj->polygon.points.resize(4);
    obj->polygon.points[0].x = static_cast<double>((*min_pt)[0]);
    obj->polygon.points[0].y = static_cast<double>((*min_pt)[1]);
    obj->polygon.points[0].z = static_cast<double>((*min_pt)[2]);

    obj->polygon.points[1].x = static_cast<double>((*max_pt)[0]);
    obj->polygon.points[1].y = static_cast<double>((*min_pt)[1]);
    obj->polygon.points[1].z = static_cast<double>((*min_pt)[2]);

    obj->polygon.points[2].x = static_cast<double>((*max_pt)[0]);
    obj->polygon.points[2].y = static_cast<double>((*max_pt)[1]);
    obj->polygon.points[2].z = static_cast<double>((*min_pt)[2]);

    obj->polygon.points[3].x = static_cast<double>((*min_pt)[0]);
    obj->polygon.points[3].y = static_cast<double>((*max_pt)[1]);
    obj->polygon.points[3].z = static_cast<double>((*min_pt)[2]);
  }
}

/**
 * @brief 以多边形凸包最长边出发, 选择最小面积Ground Box构建3D OBB
 * @param obj
 */
void MinBoxObjectBuilder::reconstructPolygon3d(const ObjectPtr& obj) {
  const auto& points = obj->polygon.points;
  if (points.size() == 0) {
    return;
  }

  // <1> Find the longest line in Convex Hull
  size_t max_point_index = 0;
  size_t min_point_index = 0;
  for (size_t i = 1; i < points.size(); ++i) {
    // clock direction
    double cross_product_result =
        cross_product2d(points[max_point_index], points[i], -baselink_fakebaselink_length_along_x_axis_);
    if (cross_product_result < common::EPSILON) {
      max_point_index = i;
    }
    // anti-clock direction
    cross_product_result = cross_product2d(points[min_point_index], points[i], -baselink_fakebaselink_length_along_x_axis_);
    if (cross_product_result > common::EPSILON) {
      min_point_index = i;
    }
  }
  obj->min_point_index = min_point_index;
  obj->max_point_index = max_point_index;

  // for use later
  const Eigen::Vector2d line(points[max_point_index].x - points[min_point_index].x,
                             points[max_point_index].y - points[min_point_index].y);
  const Eigen::Vector3d min_point(points[min_point_index].x, points[min_point_index].y, points[min_point_index].z);

  // TODO(gary): <2>???
  // 其它顶点投影到最长边得到矩形边框长边边长(total_len),宽边边长(max_dis)
  // translation: Other vertices projected onto the longest edge yield the length of the rectangle's long side
  // (total_len) and the length of the short side (max_dis).
  double total_len = 0;
  double max_dis = 0;
  double hull_perimeter = 0.;
  bool has_out = false;  // if has outerside points according to Lidar
  // the rather confusing if,elseif,elseif inside the for loop is for checking each edge, whether they are
  // "valid" if the edge is between the focal point and min max line, then it is "valid" and the line will be
  // considered for the box angle, if the edge's length fulfill certain condition, in the next for loop
  for (size_t i = min_point_index, count = 0; count < obj->polygon.points.size();
       i = (i + 1) % obj->polygon.points.size(), ++count) {
    Eigen::Vector3d p_x;
    p_x[0] = obj->polygon.points[i].x;
    p_x[1] = obj->polygon.points[i].y;
    p_x[2] = obj->polygon.points[i].z;

    size_t j = (i + 1) % obj->polygon.points.size();
    Eigen::Vector3d p_j;
    p_j[0] = obj->polygon.points[j].x;
    p_j[1] = obj->polygon.points[j].y;
    p_j[2] = obj->polygon.points[j].z;
    double dist = sqrt((p_j[0] - p_x[0]) * (p_j[0] - p_x[0]) + (p_j[1] - p_x[1]) * (p_j[1] - p_x[1]));

    hull_perimeter += dist;

    // 确保构成最长边之外的其它边
    // translation: Make sure that the other edges do not form the longest side.
    if (j != min_point_index && j != max_point_index) {
      Eigen::Vector3d ray = p_j - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < common::EPSILON) {
        total_len += dist;
        if (dist - max_dis > common::EPSILON) {
          max_dis = dist;
        }
      } else {
        // outline
        has_out = true;
      }
    } else if ((i == min_point_index && j == max_point_index) || (i == max_point_index && j == min_point_index)) {
      size_t k = (j + 1) % obj->polygon.points.size();
      Eigen::Vector3d p_k;
      p_k[0] = obj->polygon.points[k].x;
      p_k[1] = obj->polygon.points[k].y;
      p_k[2] = obj->polygon.points[k].z;
      Eigen::Vector3d ray = p_k - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < 0) {
      } else {  // i->j or j->i is an edge near the sensor
        // outline
        has_out = true;
        total_len += dist;
        max_dis = std::max(max_dis, dist);
      }
    } else if (j == min_point_index || j == max_point_index) {
      Eigen::Vector3d ray = p_x - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < common::EPSILON) {
        total_len += dist;
        if (dist > max_dis) {
          max_dis = dist;
        }
      } else {
        // outline
        has_out = true;
      }
    }
  }
  obj->total_len = total_len;
  obj->max_dist = max_dis;
  obj->hull_perimeter = hull_perimeter;

  // <3> 求解所有以最长边投影的可能矩形中面积最小的那个
  // translation: Find the smallest possible area among all the rectangles formed by projecting onto the
  // longest side.
  size_t count = 0;
  double max_score = -std::numeric_limits<double>::max();
  for (size_t i = min_point_index; count < obj->polygon.points.size(); i = (i + 1) % obj->polygon.points.size(), ++count) {
    Eigen::Vector3d p_i;
    p_i[0] = obj->polygon.points[i].x;
    p_i[1] = obj->polygon.points[i].y;
    p_i[2] = obj->polygon.points[i].z;
    size_t j = (i + 1) % obj->polygon.points.size();
    Eigen::Vector3d p_j;
    p_j[0] = obj->polygon.points[j].x;
    p_j[1] = obj->polygon.points[j].y;
    p_j[2] = obj->polygon.points[j].z;
    double dist = sqrt((p_i[0] - p_j[0]) * (p_i[0] - p_j[0]) + (p_i[1] - p_j[1]) * (p_i[1] - p_j[1]));
    if (dist < max_dis && (dist / total_len) < 0.5) {
      continue;
    }

    if (j != min_point_index && j != max_point_index) {
      Eigen::Vector3d p;
      p[0] = obj->polygon.points[j].x;
      p[1] = obj->polygon.points[j].y;
      p[2] = obj->polygon.points[j].z;
      Eigen::Vector3d ray = p - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < 0) {
        Eigen::Vector3d center;
        double length = 0;
        double width = 0;
        Eigen::Vector3d dir;
        double score = computeClosenessScore(obj, i, increment_mod(i, obj->polygon.points.size()), center, length, width, dir);
        if (score > max_score) {
          obj->ground_center = center;
          obj->length = length;
          obj->width = width;
          obj->direction = dir;
          max_score = score;
          obj->final_edge_pt_idx = i;
        }
      } else {
        // outline
      }
    } else if ((i == min_point_index && j == max_point_index) || (i == max_point_index && j == min_point_index)) {
      if (!has_out) {
        continue;
      }
      Eigen::Vector3d center;
      double length = 0;
      double width = 0;
      Eigen::Vector3d dir;
      double score = computeClosenessScore(obj, i, increment_mod(i, obj->polygon.points.size()), center, length, width, dir);
      if (score > max_score) {
        obj->ground_center = center;
        obj->length = length;
        obj->width = width;
        obj->direction = dir;
        max_score = score;
        obj->final_edge_pt_idx = i;
      }
    } else if (j == min_point_index || j == max_point_index) {
      Eigen::Vector3d p;
      p[0] = obj->polygon.points[i].x;
      p[1] = obj->polygon.points[i].y;
      p[2] = obj->polygon.points[i].z;
      Eigen::Vector3d ray = p - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < 0) {
        Eigen::Vector3d center;
        double length = 0.0;
        double width = 0.0;
        Eigen::Vector3d dir;
        double score = computeClosenessScore(obj, i, increment_mod(i, obj->polygon.points.size()), center, length, width, dir);
        if (score > max_score) {
          obj->ground_center = center;
          obj->length = length;
          obj->width = width;
          obj->direction = dir;
          max_score = score;
          obj->final_edge_pt_idx = i;
        }
      } else {
        // outline
      }
    }
  }

  // if max_point_index+1==min_point_index, means already done, then skip
  size_t next_index = increment_mod(max_point_index, obj->polygon.points.size());
  if (next_index != min_point_index) {
    Eigen::Vector3d center;
    double length = 0;
    double width = 0;
    Eigen::Vector3d dir;
    // double area = computeAreaAlongOneEdge(obj, i, &center, &length, &width, &dir);
    double score = computeClosenessScore(obj, min_point_index, max_point_index, center, length, width, dir);
    if (score > max_score) {
      obj->ground_center = center;
      obj->length = length;
      obj->width = width;
      obj->direction = dir;
      max_score = score;
      obj->final_edge_pt_idx = min_point_index;
    }
  }

  if ((obj->width < DBL_EPSILON) || (obj->length < DBL_EPSILON)) {
    ROS_WARN_ONCE("[buildObject] ground box is too narrow, w: %f, l: %f", obj->width, obj->length);

    // Pad width or length to ensure polygon generation, if either of them is zero.
    // Otherwise, combination two geometries by boost::geometry::union_ will be failed.
    obj->width = obj->width > DBL_EPSILON ? obj->width : 0.05f;
    obj->length = obj->length > DBL_EPSILON ? obj->length : 0.05f;
  }

  // <4> 边框方向
  obj->direction.normalize();
}

double MinBoxObjectBuilder::computeClosenessScore(const ObjectPtr& obj, const size_t pt1, const size_t pt2,
                                                  Eigen::Vector3d& center, double& length, double& width,
                                                  Eigen::Vector3d& dir) {
  double score = 0.0;

  const auto& original_points = obj->cloud_ptr_->points;
  const auto& polygon_points = obj->polygon.points;

  // will normalize later, no need to do now
  dir[0] = polygon_points[pt2].x - polygon_points[pt1].x;
  dir[1] = polygon_points[pt2].y - polygon_points[pt1].y;
  dir[2] = 0.0;

  const float box_angle =
      std::atan2(polygon_points[pt2].y - polygon_points[pt1].y, polygon_points[pt2].x - polygon_points[pt1].x);

  const float cos_angle = std::cos(-box_angle);
  const float sin_angle = std::sin(-box_angle);

  std::vector<float> x_rotated(original_points.size());
  std::vector<float> y_rotated(original_points.size());

  // only need to rotate, we can optimise a little by not having to do translation
  // rotation alone is enough for us to assess the score of the box
  for (size_t i = 0; i < original_points.size(); ++i) {
    x_rotated[i] = original_points[i].x * cos_angle - original_points[i].y * sin_angle;
    y_rotated[i] = original_points[i].x * sin_angle + original_points[i].y * cos_angle;
  }

  auto findMinMax = [](const std::vector<float>& vec, float& min, float& max) -> void {
    min = vec[0];
    max = vec[0];
    for (size_t i = 1; i < vec.size(); ++i) {
      if (vec[i] < min)
        min = vec[i];
      else if (vec[i] > max)
        max = vec[i];
    }
  };

  float minX;
  float maxX;
  float minY;
  float maxY;
  findMinMax(x_rotated, minX, maxX);
  findMinMax(y_rotated, minY, maxY);

  // the 0.1 is selected to reduce the impact of certain points-box_angle combination effect
  // if it is 0.01, and 10 points fulfill the min, the it will be +100 score, so this cannot be too low
  // the max of 5.0 is arbitrary for now, don't see the need to tune it lower/higher
  static const float d_min = 0.10;
  static const float d_max = 5.00;

  int goodCountScoreX = 0;
  int goodCountScoreY = 0;
  for (size_t i = 0; i < original_points.size(); ++i) {
    float scoreX = std::min(maxX - x_rotated[i], x_rotated[i] - minX);
    float scoreY = std::min(maxY - y_rotated[i], y_rotated[i] - minY);
    float bestAxisScore = std::min(scoreX, scoreY);

    // so that small values don't greatly amplify by reciprocal later
    bestAxisScore = std::max(bestAxisScore, d_min);

    // so that big values, such as noise don't interfere so much
    bestAxisScore = std::min(bestAxisScore, d_max);

    goodCountScoreX = (scoreX < 0.5) ? (goodCountScoreX + 1) : goodCountScoreX;
    goodCountScoreY = (scoreY < 0.5) ? (goodCountScoreY + 1) : goodCountScoreY;

    score += (1.0 / bestAxisScore);
  }

  // to further improve the odds of optimum box for L-shaped objects
  // when there is e.g. a curved barricade
  score += std::min(goodCountScoreX, goodCountScoreY);

  center[0] = (minX + maxX) / 2.0;
  center[1] = (minY + maxY) / 2.0;
  float tmpX = center[0];
  // reverse the rotation for x and y
  center[0] = tmpX * cos_angle + center[1] * sin_angle;
  center[1] = -tmpX * sin_angle + center[1] * cos_angle;
  center[2] = polygon_points[0].z;

  length = maxX - minX;
  width = maxY - minY;

  // length should be the "longer" side
  if (width > length) {
    std::swap(length, width);
    // x1 = x0 * cosA - y0 * sinA
    // y1 = x0 * sinA - y0 * cosA
    // rotate 90 degrees, cosA = 0, sinA = 1
    // x1 = -y0
    // y1 =  x0
    std::swap(dir[0], dir[1]);
    dir[0] = -dir[0];
  }

  return score;
}


