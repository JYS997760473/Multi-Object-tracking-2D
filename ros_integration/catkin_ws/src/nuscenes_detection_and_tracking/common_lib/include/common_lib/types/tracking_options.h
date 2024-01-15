#pragma once

#include <memory>

#include <Eigen/Core>

struct TrackingOptions {
  TrackingOptions() = default;

  explicit TrackingOptions(Eigen::Matrix4d* pose)
      : velo2world_trans(pose) {}

  std::shared_ptr<Eigen::Matrix4d> velo2world_trans;
  bool is_increased_near_lane_dynamicity{false};
  bool is_occlusion_based_refinement_on{false};
};
