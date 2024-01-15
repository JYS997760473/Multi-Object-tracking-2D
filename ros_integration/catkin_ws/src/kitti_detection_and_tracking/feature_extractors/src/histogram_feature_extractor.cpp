/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#include "feature_extractors/histogram_feature_extractor.hpp"
#include <cfloat>  // FLT_MAX

#include "common/time.hpp"

namespace autosense {
namespace feature {

void HistogramFeatureExtractor::setHistogramBinSize(float histogram_bin_size) {
    bin_size_ = histogram_bin_size;
}

void HistogramFeatureExtractor::compute(
    const std::vector<PointICloudPtr> &cloud_clusters,
    std::vector<Feature> &features) {
    common::Clock clock;
    features.clear();

    for (size_t idx = 0u; idx < cloud_clusters.size(); ++idx) {
        Feature feature;
        compute(*cloud_clusters[idx], &feature);
        features.push_back(feature);
    }
    ROS_INFO_STREAM("Histogram features extracted. Took "
                    << clock.takeRealTime() << "ms.");
}

/**
 * @brief compute histogram feature of given cloud
 * @params[IN] bin_size: bin size of histogram
 * @params[OUT] feature: histogram feature of given cloud
 * @return nothing
 * @result A (3*bin_size)-dim feature
 */
void HistogramFeatureExtractor::compute(const PointICloud &cloud_in,
                                        Feature *feature) {
    getMinMaxCenter(cloud_in);

    float xstep = bin_size_;
    float ystep = bin_size_;
    float zstep = bin_size_;

    int xsize = (max_pt_.x - min_pt_.x) / xstep + 0.000001;
    int ysize = (max_pt_.y - min_pt_.y) / ystep + 0.000001;
    int zsize = (max_pt_.z - min_pt_.z) / zstep + 0.000001;

    int stat_len = xsize + ysize + zsize;
    std::vector<int> stat_feat(stat_len, 0);

    int pt_num = cloud_in.points.size();
    for (int i = 0u; i < pt_num; ++i) {
        const PointI &pt = cloud_in.points[i];
        stat_feat[static_cast<int>((pt.x - min_pt_.x) / xstep)]++;
        stat_feat[static_cast<int>(xsize + (pt.y - min_pt_.y) / ystep)]++;
        stat_feat[static_cast<int>(xsize + ysize +
                                   (pt.z - min_pt_.z) / zstep)]++;
    }
    // update feature
    for (size_t i = 0u; i < xsize; ++i) {
        std::string feature_name = "x_" + i;
        (*feature).push_back(FeatureElement(
            feature_name,
            static_cast<float>(stat_feat[i]) / static_cast<float>(pt_num)));
    }
    for (size_t i = 0u; i < ysize; ++i) {
        std::string feature_name = "y_" + i;
        (*feature).push_back(FeatureElement(
            feature_name,
            static_cast<float>(stat_feat[i]) / static_cast<float>(pt_num)));
    }
    for (size_t i = 0u; i < zsize; ++i) {
        std::string feature_name = "z_" + i;
        (*feature).push_back(FeatureElement(
            feature_name,
            static_cast<float>(stat_feat[i]) / static_cast<float>(pt_num)));
    }
}

void HistogramFeatureExtractor::getMinMaxCenter(const PointICloud &cloud_in) {
    float xsum = 0.0;
    float ysum = 0.0;
    float zsum = 0.0;
    min_pt_.x = min_pt_.y = min_pt_.z = FLT_MAX;
    max_pt_.x = max_pt_.y = max_pt_.z = -FLT_MAX;
    // min max pt
    int pt_num = cloud_in.points.size();
    for (int i = 0; i < pt_num; ++i) {
        const PointI &pt = cloud_in.points[i];
        xsum += pt.x;
        ysum += pt.y;
        zsum += pt.z;
        min_pt_.x = std::min(min_pt_.x, pt.x);
        max_pt_.x = std::max(max_pt_.x, pt.x);
        min_pt_.y = std::min(min_pt_.y, pt.y);
        max_pt_.y = std::max(max_pt_.y, pt.y);
        min_pt_.z = std::min(min_pt_.z, pt.z);
        max_pt_.z = std::max(max_pt_.z, pt.z);
    }
    // center position
    center_pt_.x = xsum / pt_num;
    center_pt_.y = ysum / pt_num;
    center_pt_.z = zsum / pt_num;
}

}  // namespace feature
}  // namespace autosense
