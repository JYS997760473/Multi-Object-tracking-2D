/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#include "feature_extractors/esf_feature_extractor.hpp"

#include <pcl/io/io.h>
#include <ros/ros.h>

#include "common/time.hpp"  // common::Clock

namespace autosense {
namespace feature {

ESFFeatureExtractor::ESFFeatureExtractor() {}

ESFFeatureExtractor::~ESFFeatureExtractor() {}

void ESFFeatureExtractor::compute(
    const std::vector<PointICloudPtr> &cloud_clusters,
    std::vector<Feature> &features) {
    common::Clock clock;
    features.clear();

    for (size_t idx = 0u; idx < cloud_clusters.size(); ++idx) {
        Feature feature;
        compute(*cloud_clusters[idx], &feature);
        if (feature.size() == kESFDimension) {
            features.push_back(feature);
        }
    }
    ROS_INFO_STREAM("Ensemble of Shapes Function features extracted. Took "
                    << clock.takeRealTime() << "ms.");
}

void ESFFeatureExtractor::compute(const PointICloud &cloud_in,
                                  Feature *feature) {
    if (feature == nullptr) {
        return;
    }
    PointCloudPtr cloud(new PointCloud);
    pcl::copyPointCloud<PointI, Point>(cloud_in, *cloud);
    // Object for storing the ESF descriptor.
    pcl::PointCloud<pcl::ESFSignature640>::Ptr esf640(
        new pcl::PointCloud<pcl::ESFSignature640>);
    esf_estimator_.setInputCloud(cloud);
    esf_estimator_.compute(*esf640);
    // After estimating the ensemble of shape functions, the esf640 should be of
    // size 1
    if (esf640->size() != 1u) {
        ROS_ERROR("Failed to extract the ESF feature.");
        feature = NULL;
        return;
    }
    for (unsigned int i = 0u; i < kESFDimension; ++i) {
        (*feature).push_back(FeatureElement("esf_" + std::to_string(i),
                                            esf640->points[0].histogram[i]));
    }
    if ((*feature).size() != kESFDimension) {
        ROS_ERROR("Feature has the wrong dimension.");
        feature = NULL;
        return;
    }
}

}  // namespace feature
}  // namespace autosense
