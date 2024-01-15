/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef ROI_FILTERS_INCLUDE_ROI_FILTERS_ROI_HPP_
#define ROI_FILTERS_INCLUDE_ROI_FILTERS_ROI_HPP_

#include <pcl/filters/approximate_voxel_grid.h>  // pcl::ApproximateVoxelGrid
#include <pcl/filters/passthrough.h>             // pcl::PassThrough
#include <pcl/filters/voxel_grid.h>              // pcl::VoxelGrid
#include <vector>

#include "common/common.hpp"    // common::EPSILON
#include "common/geometry.hpp"  // common::geometry::calcCylinderDistNorm
#include "common/types/type.h"  // VolumetricModel_Human/VolumetricModel_Car

namespace autosense {
namespace roi {

template <typename PointT>
static void elevationBandPass(float z_limit_min,
                              float z_limit_max,
                              typename pcl::PointCloud<PointT>::Ptr cloud) {
    if (cloud->size()) {
        typename pcl::PointCloud<PointT>::Ptr cloud_in(
            new pcl::PointCloud<PointT>);
        *cloud_in = *cloud;
        cloud->clear();

        pcl::PassThrough<Point> passFilter;
        passFilter.setFilterFieldName("z");
        passFilter.setFilterLimits(z_limit_min, z_limit_max);
        passFilter.setInputCloud(cloud_in);
        passFilter.filter(*cloud);
    }
}

/*
 * @brief Voxel Grid Downsampling
 * @note
 *  SegMatch: A voxel grid is then applied to the resulting source cloud,
 *      in order to filter-out noise in voxels where there is not enough
 * evidence for occupancy.
 * @note Configuration follows SegMatch
 * @note
 *  voxel_size: 0.1f
 *  min_point_number_per_voxel: 1
 */
template <typename PointT>
static void voxelGridFilter(float voxel_size,
                            int min_point_number_per_voxel,
                            typename pcl::PointCloud<PointT>::Ptr cloud) {
    if (cloud->size()) {
        typename pcl::PointCloud<PointT>::Ptr cloud_in(
            new pcl::PointCloud<PointT>);
        *cloud_in = *cloud;
        cloud->clear();

        pcl::VoxelGrid<PointT> voxelFilter;
        voxelFilter.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxelFilter.setMinimumPointsNumberPerVoxel(min_point_number_per_voxel);
        voxelFilter.setInputCloud(cloud_in);
        voxelFilter.filter(*cloud);
    }
}

template <typename PointT>
static void approxVoxelGridFilter(float voxel_size,
                                  int min_point_number_per_voxel,
                                  typename pcl::PointCloud<PointT>::Ptr cloud) {
    if (cloud->size()) {
        typename pcl::PointCloud<PointT>::Ptr cloud_in(
            new pcl::PointCloud<PointT>);
        *cloud_in = *cloud;
        cloud->clear();

        pcl::ApproximateVoxelGrid<PointT> voxelFilter;
        voxelFilter.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxelFilter.setMinimumPointsNumberPerVoxel(min_point_number_per_voxel);
        voxelFilter.setInputCloud(cloud_in);
        voxelFilter.filter(*cloud);
    }
}

/*
 * @brief Cylinder(middle center[x,y,z], radius_m, height_above_m,
 * height_below_m) Filter
 * @result cloud after filter
 */
template <typename PointT>
static void cylinderROIFilter(float radius_min,
                              float radius_max,
                              float z_limit_min,
                              float z_limit_max,
                              typename pcl::PointCloud<PointT>::Ptr cloud) {
    if (cloud->size()) {
        typename pcl::PointCloud<PointT>::Ptr cloud_in(
            new pcl::PointCloud<PointT>);
        *cloud_in = *cloud;
        cloud->clear();

        const double radius_min_squared = pow(radius_min, 2.0);
        const double radius_max_squared = pow(radius_max, 2.0);

        for (size_t pt = 0u; pt < cloud_in->size(); ++pt) {
            const PointT& point = cloud_in->points[pt];
            const double dist =
                common::geometry::calcCylinderDistNorm<PointT>(point);
            // Step 1: filter out a large part
            if (dist > radius_min_squared && dist < radius_max_squared) {
                if (point.z > z_limit_min && point.z < z_limit_max) {
                    cloud->points.push_back(point);
                }
            }
        }
    }
}

/**
 * @brief forward radius_max(/m), left/right radius_min separately
 * @note Velodyne Coordinate
 *          |x(forward)
 *      C   |   D
 *          |
 *  y---------------
 *          |
 *      B   |   A
 */
template <typename PointT>
static void squareROIFilter(float radius_min,
                            float radius_max,
                            float z_limit_min,
                            float z_limit_max,
                            typename pcl::PointCloud<PointT>::Ptr cloud) {
    if (cloud->size()) {
        typename pcl::PointCloud<PointT>::Ptr cloud_in(
            new pcl::PointCloud<PointT>);
        *cloud_in = *cloud;
        cloud->clear();

        const float forward = radius_max;
        const float back = -radius_max;
        const float left = -radius_min;
        const float right = radius_min;

        for (size_t pt = 0u; pt < cloud_in->size(); ++pt) {
            const PointT& point = cloud_in->points[pt];
            // Step 1: filter out a large part
            if (point.y > left && point.y < right) {
                // Step 2: filter out small part
                if (point.z > z_limit_min && point.z < z_limit_max) {
                    // Step 3: almost not filter out
                    if (point.x > back && point.x < forward) {
                        cloud->points.push_back(point);
                    }
                }
            }
        }
    }
}

template <typename PointT>
static void applyROIFilter(const ROIParams& params,
                           typename pcl::PointCloud<PointT>::Ptr cloud) {
    const float roi_radius_min = params.roi_radius_min_m;
    const float roi_radius_max = params.roi_radius_max_m;
    const float roi_z_limit_min =
        (-1.0) * (params.roi_lidar_height_m + params.roi_height_below_m);
    const float roi_z_limit_max =
        params.roi_height_above_m - params.roi_lidar_height_m;

    if (params.type == "Cylinder") {
        cylinderROIFilter<PointI>(roi_radius_min, roi_radius_max,
                                  roi_z_limit_min, roi_z_limit_max, cloud);
    } else if (params.type == "Square") {
        squareROIFilter<PointI>(roi_radius_min, roi_radius_max, roi_z_limit_min,
                                roi_z_limit_max, cloud);
    } else {
        cylinderROIFilter<PointI>(roi_radius_min, roi_radius_max,
                                  roi_z_limit_min, roi_z_limit_max, cloud);
    }
}

/**
 * @brief Bird Eye View
 *          |x(forward)
 *      C---|---D
 *      |   |   |
 *      |   |   |
 *  y---------------
 *      B---|---A
 *
 */
template <typename PointT>
static void bevROIFilter(float x_forward,
                         float x_back,
                         float half_width,
                         typename pcl::PointCloud<PointT>::Ptr cloud) {
    if (cloud->size()) {
        typename pcl::PointCloud<PointT>::Ptr cloud_in(
            new pcl::PointCloud<PointT>);
        *cloud_in = *cloud;
        cloud->clear();

        const float forward = x_forward;
        const float back = -x_back;
        const float left = half_width;
        const float right = -half_width;

        for (size_t pt = 0u; pt < cloud_in->size(); ++pt) {
            const PointT& point = cloud_in->points[pt];

            if (point.x > back && point.x < forward) {
                if (point.y < left && point.y > right) {
                    continue;
                }
            }

            cloud->points.push_back(point);
        }
    }
}

/**
 * @brief car/human-like volumetric model is used to filter out over- and
 * under-segmented clusters
 */
static void humanModelFilter(const std::vector<ObjectPtr>& objects_in,
                             const VolumetricModel& model,
                             std::vector<ObjectPtr>* objects_filtered) {
    std::vector<ObjectPtr> objects(objects_in.begin(), objects_in.end());
    (*objects_filtered).clear();
    for (size_t obj_id = 0u; obj_id < objects.size(); ++obj_id) {
        double length = objects[obj_id]->length;
        double width = objects[obj_id]->width;
        double height = objects[obj_id]->height;

        if (abs(model.l_min - model.l_max) > common::EPSILON &&
            (length < model.l_min || length > model.l_max)) {
            continue;
        }

        if (abs(model.w_min - model.w_max) > common::EPSILON &&
            (width < model.w_min || width > model.w_max)) {
            continue;
        }

        if (abs(model.h_min - model.h_max) > common::EPSILON &&
            (height < model.h_min || height > model.h_max)) {
            continue;
        }

        objects[obj_id]->type = PEDESTRIAN;
        (*objects_filtered).push_back(objects[obj_id]);
    }
}

static void carModelFilter(const std::vector<ObjectPtr>& objects_in,
                           const VolumetricModel& model,
                           std::vector<ObjectPtr>* objects_filtered) {
    std::vector<ObjectPtr> objects(objects_in.begin(), objects_in.end());
    (*objects_filtered).clear();
    for (size_t obj_id = 0u; obj_id < objects.size(); ++obj_id) {
        double length = objects[obj_id]->length;
        double width = objects[obj_id]->width;
        double height = objects[obj_id]->height;

        if (abs(model.l_min - model.l_max) > common::EPSILON &&
            (length < model.l_min || length > model.l_max)) {
            continue;
        }

        if (abs(model.w_min - model.w_max) > common::EPSILON &&
            (width < model.w_min || width > model.w_max)) {
            continue;
        }

        if (abs(model.h_min - model.h_max) > common::EPSILON &&
            (height < model.h_min || height > model.h_max)) {
            continue;
        }

        objects[obj_id]->type = CAR;
        (*objects_filtered).push_back(objects[obj_id]);
    }
}

static void humanCarModelFilter(const std::vector<ObjectPtr>& objects_in,
                                const VolumetricModel& human_model,
                                const VolumetricModel& car_model,
                                std::vector<ObjectPtr>* objects_filtered) {
    std::vector<ObjectPtr> objects(objects_in.begin(), objects_in.end());
    (*objects_filtered).clear();

    humanModelFilter(objects, human_model, objects_filtered);
    carModelFilter(objects, car_model, &objects);
    (*objects_filtered)
        .insert((*objects_filtered).end(), objects.begin(), objects.end());
}

}  // namespace roi
}  // namespace autosense

#endif  // ROI_FILTERS_INCLUDE_ROI_FILTERS_ROI_HPP_
