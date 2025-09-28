/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  geometric_path.cc
 */

#include "planning/open_space/coarse_path_generator/geometric_path.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>

#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "planning/common/open_space_info.h"
#include "planning/open_space/coarse_path_generator/path_generator.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {
namespace {
constexpr double kEpsilon = 1.0e-3;
}  // namespace

GeometricPath::GeometricPath(const WarmStartConfig& warm_start_config)
    : PathGenerator(warm_start_config),
      extra_distance_for_geometry_(
          warm_start_config_.extra_distance_for_geometry_path()),
      max_steer_angle_margin_(
          warm_start_config_.geometry_planner_steer_angle_margin()) {
  scs_shape_path_planner_ = std::make_unique<SCSShapePath>();
}

bool GeometricPath::Plan(
    const std::atomic<bool>& atomic_early_stop_flag,
    const common::PathPoint& start_point, const common::PathPoint& end_point,
    const std::vector<double>& xy_bounds,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec,
    const DestRegionWithAng& dest_region_with_angle,
    const PathSearchStrategy& path_search_strategy,
    PathGeneratorResult* const result) {
  UNUSED(atomic_early_stop_flag);
  if (xy_bounds.size() != 4 || result == nullptr) {
    AERROR << " GeometricPath input check fails";
    return false;
  }
  const double min_radius = GetMinRadius();
  xy_bounds_ = xy_bounds;
  std::vector<std::pair<common::math::LineSegment2d, double>>
      obstacles_without_virtual;
  std::vector<std::pair<common::math::LineSegment2d, double>>
      obstacles_segments_vec_geometry;
  for (auto obstacles_segments : obstacles_segments_vec) {
    if (obstacles_segments.second > kEpsilon) {
      obstacles_without_virtual.push_back(obstacles_segments);
      obstacles_segments.second += extra_distance_for_geometry_;
      obstacles_segments_vec_geometry.push_back(obstacles_segments);
    }
  }
  common::math::LineSegment2d line_1(Vec2d(xy_bounds.at(0), xy_bounds.at(2)),
                                     Vec2d(xy_bounds.at(0), xy_bounds.at(3)));
  common::math::LineSegment2d line_2(Vec2d(xy_bounds.at(1), xy_bounds.at(2)),
                                     Vec2d(xy_bounds.at(1), xy_bounds.at(3)));
  common::math::LineSegment2d line_3(Vec2d(xy_bounds.at(0), xy_bounds.at(2)),
                                     Vec2d(xy_bounds.at(1), xy_bounds.at(2)));
  common::math::LineSegment2d line_4(Vec2d(xy_bounds.at(0), xy_bounds.at(3)),
                                     Vec2d(xy_bounds.at(1), xy_bounds.at(3)));
  obstacles_without_virtual.emplace_back(line_1, kEpsilon);
  obstacles_without_virtual.emplace_back(line_2, kEpsilon);
  obstacles_without_virtual.emplace_back(line_3, kEpsilon);
  obstacles_without_virtual.emplace_back(line_4, kEpsilon);
  obstacles_segments_vec_geometry.emplace_back(line_1, kEpsilon);
  obstacles_segments_vec_geometry.emplace_back(line_2, kEpsilon);
  obstacles_segments_vec_geometry.emplace_back(line_3, kEpsilon);
  obstacles_segments_vec_geometry.emplace_back(line_4, kEpsilon);

  DestRegionWithAng dest_region_with_angle_local;
  if (path_search_strategy.is_plan_from_start) {
    dest_region_with_angle_local = dest_region_with_angle;
  }
  const bool is_park_out =
      (path_search_strategy.park_direction == LEFTPARKOUT ||
       path_search_strategy.park_direction == RIGHTPARKOUT);
  if (!scs_shape_path_planner_->GenerateSCSShapePath(
          start_point, end_point, dest_region_with_angle_local, min_radius,
          is_park_out, result)) {
    AERROR << "GenerateSCSShapePath failed";
    result->reset();
    return false;
  }

  if (result->x.empty() || result->x.size() != result->y.size() ||
      result->x.size() != result->phi.size()) {
    AERROR << "result x y phi size missmathch";
    result->reset();
    return false;
  }

  if (!CollisonCheck(*result, obstacles_without_virtual,
                     obstacles_segments_vec_geometry)) {
    AERROR << " generated path is not collision free";
    result->reset();
    return false;
  }
  result->path_type = planning_internal::PathUpdateStatus::SCS_GEOMETRY;
  return true;
}

bool GeometricPath::CollisonCheck(
    const PathGeneratorResult& geometry_path,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_without_virtual,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec_geometry) {
  double end_phi = geometry_path.phi.back();
  auto is_straight_line = [&](const double phi) {
    return fabs(common::math::NormalizeAngle(phi - end_phi)) < 0.01;
  };
  for (int i = 0; i < geometry_path.x.size(); ++i) {
    std::shared_ptr<Node3d> path_nodes = std::make_shared<Node3d>(
        geometry_path.x[i], geometry_path.y[i], geometry_path.phi[i],
        xy_bounds_, xy_grid_resolution_, phi_grid_resolution_);
    const auto& obs_segment = is_straight_line(geometry_path.phi[i])
                                  ? obstacles_without_virtual
                                  : obstacles_segments_vec_geometry;
    if (!ValidityCheck(path_nodes, obs_segment)) {
      AERROR << " collision check failed " << path_nodes->GetX() << " "
             << path_nodes->GetY() << " " << path_nodes->GetPhi();
      return false;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace TL
