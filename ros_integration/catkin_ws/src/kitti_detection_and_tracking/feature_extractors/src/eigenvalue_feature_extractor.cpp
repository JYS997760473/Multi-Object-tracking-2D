/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#include "feature_extractors/eigenvalue_feature_extractor.hpp"

#include <pcl/common/centroid.h>  // pcl::compute3DCentroid
#include <pcl/common/common.h>    // pcl::getMinMax3D
#include <pcl/io/io.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>  // Eigen::EigenSolver

#include "common/common.hpp"  // common::swap_if_gt
#include "common/time.hpp"    // common::Clock

namespace autosense {
namespace feature {

EigenValueFeatureExtractor::EigenValueFeatureExtractor() {}

EigenValueFeatureExtractor::~EigenValueFeatureExtractor() {}

void EigenValueFeatureExtractor::compute(
    const std::vector<PointICloudPtr> &cloud_clusters,
    std::vector<Feature> &features) {
    common::Clock clock;
    features.clear();

    for (size_t idx = 0u; idx < cloud_clusters.size(); ++idx) {
        Feature feature;
        compute(*cloud_clusters[idx], &feature);
        if (feature.size() == kDimension) {
            features.push_back(feature);
        }
    }
    ROS_INFO_STREAM("Eigenvalue-based features extracted. Took "
                    << clock.takeRealTime() << "ms.");
}

void EigenValueFeatureExtractor::compute(const PointICloud &cloud_in,
                                         Feature *feature) {
    if (feature == nullptr) {
        return;
    }
    PointCloudPtr cloud(new PointCloud);
    pcl::copyPointCloud<PointI, Point>(cloud_in, *cloud);

    // <1>Find the variances.
    const size_t num_points = cloud_in.points.size();
    PointCloud variances;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cloud_in, centroid);
    for (size_t p = 0u; p < num_points; ++p) {
        variances.push_back(Point());
        variances.points[p].x = cloud_in.points[p].x - centroid(0);
        variances.points[p].y = cloud_in.points[p].y - centroid(1);
        variances.points[p].z = cloud_in.points[p].z - centroid(2);
    }

    /*
     * <2>Find the covariance matrix. Since it is symmetric, we only bother with
     * the upper diagonal.
     * |                                |
     * | cov(x,x)   cov(y,x)   cov(z,x) |
     * | cov(x,y)   cov(y,y)   cov(z,y) |
     * | cov(x,z)   cov(y,z)   cov(z,z) |
     * |                                |
     * covariance tensor C = 1/k \sum{(pi − p)(pi − p)}, (pi \in P);
     *  where P is the set of k nearest neighbors and p = med(pi \in P) is its
     * medoid.
     */
    const std::vector<size_t> row_indices_to_access = {0, 0, 0, 1, 1, 2};
    const std::vector<size_t> col_indices_to_access = {0, 1, 2, 1, 2, 2};
    Eigen::Matrix3f covariance_matrix;
    for (size_t i = 0u; i < row_indices_to_access.size(); ++i) {
        const size_t row = row_indices_to_access[i];
        const size_t col = col_indices_to_access[i];
        double covariance = 0;
        for (size_t k = 0u; k < num_points; ++k) {
            covariance +=
                variances.points[k].data[row] * variances.points[k].data[col];
        }
        covariance /= num_points;
        covariance_matrix(row, col) = covariance;
        covariance_matrix(col, row) = covariance;
    }

    /*
     * <3>Compute eigenvalues of covariance matrix.
     *  3D features based on eigenvalues1 λ1 ≥ λ2 ≥ λ3 ≥ 0
     */
    constexpr bool compute_eigenvectors = false;
    Eigen::EigenSolver<Eigen::Matrix3f> eigenvalues_solver(
        covariance_matrix, compute_eigenvectors);
    std::vector<float> eigenvalues(3, 0.0);
    eigenvalues.at(0) = eigenvalues_solver.eigenvalues()[0].real();
    eigenvalues.at(1) = eigenvalues_solver.eigenvalues()[1].real();
    eigenvalues.at(2) = eigenvalues_solver.eigenvalues()[2].real();
    if (eigenvalues_solver.eigenvalues()[0].imag() != 0.0 ||
        eigenvalues_solver.eigenvalues()[1].imag() != 0.0 ||
        eigenvalues_solver.eigenvalues()[2].imag() != 0.0) {
        ROS_ERROR("Eigenvalues should not have non-zero imaginary component.");
        feature = NULL;
        return;
    }

    // <4>Sort eigenvalues from smallest to largest.
    common::swap_if_gt(eigenvalues.at(0), eigenvalues.at(1));
    common::swap_if_gt(eigenvalues.at(0), eigenvalues.at(2));
    common::swap_if_gt(eigenvalues.at(1), eigenvalues.at(2));

    // <5>Eigenvalues are normalised to sum up to 1, so as to increase
    //   robustness against changes in point density.
    double sum_eigenvalues =
        eigenvalues.at(0) + eigenvalues.at(1) + eigenvalues.at(2);
    if (sum_eigenvalues == 0.0) {
        ROS_ERROR("Eigenvalues should not all be zeros.");
        feature = nullptr;
        return;
    }
    double e1 = eigenvalues.at(0) / sum_eigenvalues;
    double e2 = eigenvalues.at(1) / sum_eigenvalues;
    double e3 = eigenvalues.at(2) / sum_eigenvalues;
    if (e1 == e2 || e2 == e3 || e1 == e3) {
        ROS_ERROR("Eigenvalues should not be equal.");
        feature = NULL;
        return;
    }

    /**
     * <6>Calculate features.
     * @note refer to 《FAST SEMANTIC SEGMENTATION OF 3D POINT CLOUDS WITH
     * STRONGLY VARYING DENSITY》
     */
    const double sum_of_eigenvalues = e1 + e2 + e3;
    constexpr double kOneThird = 1.0 / 3.0;
    if (e1 == 0.0 || sum_of_eigenvalues == 0.0) {
        feature = nullptr;
        return;
    }

    const double kNormalizationPercentile = 1.0;
    // feature value upper bound
    const double kLinearityMax = 28890.9 * kNormalizationPercentile;
    const double kPlanarityMax = 95919.2 * kNormalizationPercentile;
    const double kScatteringMax = 124811 * kNormalizationPercentile;
    const double kOmnivarianceMax = 0.278636 * kNormalizationPercentile;
    const double kAnisotropyMax = 124810 * kNormalizationPercentile;
    const double kEigenEntropyMax = 0.956129 * kNormalizationPercentile;
    const double kChangeOfCurvatureMax = 0.99702 * kNormalizationPercentile;
    // const double kNPointsMax = 13200 * kNormalizationPercentile;

    // TODO(gary): number of points is important, seems that different segments
    //   with different size
    (*feature).push_back(FeatureElement("Num", num_points));

    (*feature).push_back(
        FeatureElement("Linearity", (e1 - e2) / e1 / kLinearityMax));
    (*feature).push_back(
        FeatureElement("Planarity", (e2 - e3) / e1 / kPlanarityMax));
    (*feature).push_back(
        FeatureElement("Sphericity", e3 / e1 / kScatteringMax));
    (*feature).push_back(FeatureElement(
        "Omnivariance", std::pow(e1 * e2 * e3, kOneThird) / kOmnivarianceMax));
    (*feature).push_back(
        FeatureElement("Anisotropy", (e1 - e3) / e1 / kAnisotropyMax));
    (*feature).push_back(
        FeatureElement("Eigenentropy",
                       (e1 * std::log(e1)) + (e2 * std::log(e2)) +
                           (e3 * std::log(e3)) / kEigenEntropyMax));
    (*feature).push_back(FeatureElement(
        "SurfaceVariation", e3 / sum_of_eigenvalues / kChangeOfCurvatureMax));
    /// @note follow SegMatch
    Point point_min, point_max;
    pcl::getMinMax3D(*cloud, point_min, point_max);

    double diff_x, diff_y, diff_z;

    diff_x = point_max.x - point_min.x;
    diff_y = point_max.y - point_min.y;
    diff_z = point_max.z - point_min.z;

    if (diff_z < diff_x && diff_z < diff_y) {
        (*feature).push_back(FeatureElement("Verticality", 0.2));
    } else {
        (*feature).push_back(FeatureElement("Verticality", 0.0));
    }

    if ((*feature).size() != kDimension) {
        ROS_ERROR("Feature has the wrong dimension.");
        feature = NULL;
        return;
    }
}

}  // namespace feature
}  // namespace autosense
