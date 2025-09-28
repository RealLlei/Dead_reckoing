/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  geometry_path_generator.cc
 */

#include "planning/open_space/coarse_path_generator/geometry_path_generator.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>

#include "common/math/line_segment2d.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "planning/common/open_space_info.h"
#include "planning/open_space/coarse_path_generator/path_generator.h"
#include "planning/open_space/coarse_path_generator/shape/geometry_path.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {
namespace {
constexpr double kEpsilon = 1.0e-3;
}  // namespace

GeometryPathGenerator::GeometryPathGenerator(
    const WarmStartConfig& warm_start_config)
    : PathGenerator(warm_start_config),
      extra_distance_for_geometry_(
          warm_start_config_.extra_distance_for_geometry_path()),
      max_steer_angle_margin_(
          warm_start_config_.geometry_planner_steer_angle_margin()) {
  geometry_planner_ =
      std::make_unique<Geometry>(vehicle_param_, warm_start_config_);
}

bool GeometryPathGenerator::IsGeometryPathAcceptable(
    const bool is_cycle_straight_connect, const common::PathPoint& start_point,
    const common::PathPoint& end_point,
    const std::pair<double, double>& intermediate_point_longitudal_bound,
    const GeometryPath& geometry_path) const {
  if (xy_bounds_.size() < 4 || geometry_path.empty() ||
      geometry_path.back().first.x.empty() ||
      geometry_path.back().first.y.empty() ||
      geometry_path.back().first.phi.empty()) {
    ADEBUG << " IsGeometryPathAcceptable input check fails";
    return false;
  }
  const double intermediate_point_x = geometry_path.back().first.x.at(0);
  const double intermediate_point_y = geometry_path.back().first.y.at(0);
  const double intermediate_point_phi = geometry_path.back().first.phi.at(0);
  if (intermediate_point_x < xy_bounds_.at(0) ||
      intermediate_point_x > xy_bounds_.at(1) ||
      intermediate_point_y < xy_bounds_.at(2) ||
      intermediate_point_y > xy_bounds_.at(3)) {
    ADEBUG << "intermediate point is out of range";
    return false;
  }

  // principle one: gear shift limit validity
  if (path_search_strategy_.init_path_direction != 0 &&
      geometry_path.at(0).first.x.size() >= 2 &&
      geometry_path.at(0).first.y.size() >= 2) {
    const common::math::Vec2d start_point_vec(
        geometry_path.at(0).first.x.at(1) - start_point.x(),
        geometry_path.at(0).first.y.at(1) - start_point.y());
    int path_gear_direction = 0;
    if (start_point_vec.Length() >= kEpsilon) {
      path_gear_direction =
          fabs(common::math::NormalizeAngle(start_point_vec.Angle() -
                                            start_point.theta())) <= M_PI_2
              ? 1
              : -1;
    }
    if (path_search_strategy_.init_path_direction * path_gear_direction < 0) {
      ADEBUG << " gear shift limit is not satified";
      return false;
    }
  }

  // principle two: keypoint can not cross y bounds
  const double cos_theta = cos(intermediate_point_phi);
  const double sin_theta = sin(intermediate_point_phi);
  const double up_y = intermediate_point_y +
                      vehicle_param_.width() / 2 * fabs(cos_theta) +
                      fmax(vehicle_param_.front_edge_to_center() * sin_theta,
                           -vehicle_param_.back_edge_to_center() * sin_theta);
  const double low_y = intermediate_point_y -
                       vehicle_param_.width() / 2 * fabs(cos_theta) +
                       fmin(vehicle_param_.front_edge_to_center() * sin_theta,
                            -vehicle_param_.back_edge_to_center() * sin_theta);
  if (low_y < xy_bounds_.at(2) || up_y > xy_bounds_.at(3)) {
    ADEBUG << " key point cross y bounds, do not adapt geometry path";
    return false;
  }
  // principle three: keypoint should in longitudal_bound
  const auto target_point = is_cycle_straight_connect ? end_point : start_point;
  const double project_longitudal_length =
      cos_theta * (intermediate_point_x - target_point.x()) +
      sin_theta * (intermediate_point_y - target_point.y());

  if (project_longitudal_length < intermediate_point_longitudal_bound.first ||
      project_longitudal_length > intermediate_point_longitudal_bound.second) {
    ADEBUG << " key point is not in longitudal range, dist is "
           << project_longitudal_length;
    return false;
  }
  return true;
}

bool GeometryPathGenerator::GetResult(const bool pursuit_precise_pose_connect,
                                      const GeometryPath& geometry_path,
                                      PathGeneratorResult* const result) {
  if (result == nullptr || geometry_path.empty()) {
    return false;
  }
  result->x.clear();
  result->y.clear();
  result->phi.clear();
  for (const auto& path : geometry_path) {
    if (path.first.x.size() != path.first.y.size() ||
        path.first.x.size() != path.first.phi.size() ||
        path.first.x.size() < 2) {
      AERROR << "geometry path is not valid";
      return false;
    }
    result->x.insert(result->x.end(), path.first.x.begin(), path.first.x.end());
    result->y.insert(result->y.end(), path.first.y.begin(), path.first.y.end());
    result->phi.insert(result->phi.end(), path.first.phi.begin(),
                       path.first.phi.end());
    // remove last point of each path
    result->x.pop_back();
    result->y.pop_back();
    result->phi.pop_back();
  }
  // add last point
  result->x.emplace_back(geometry_path.back().first.x.back());
  result->y.emplace_back(geometry_path.back().first.y.back());
  result->phi.emplace_back(geometry_path.back().first.phi.back());
  result->path_type =
      pursuit_precise_pose_connect
          ? planning_internal::PathUpdateStatus::GEOMETRY
          : planning_internal::PathUpdateStatus::GEOMETRY_ADJUST;

  for (size_t i = 0; i < result->x.size(); i++) {
    ADEBUG << "x:" << result->x.at(i) << " y:" << result->y.at(i)
           << " phi:" << result->phi.at(i);
  }
  AINFO << "geometry path success";
  return true;
}

bool GeometryPathGenerator::Plan(
    const std::atomic<bool>& atomic_early_stop_flag,
    const common::PathPoint& start_point, const common::PathPoint& end_point,
    const std::vector<double>& xy_bounds,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec,
    const DestRegionWithAng& /*dest_region_with_angle*/,
    const PathSearchStrategy& path_search_strategy,
    PathGeneratorResult* const result) {
  UNUSED(atomic_early_stop_flag);
  const size_t num_strategy =
      path_search_strategy.use_geometry_strategy.geometry_path_type.size();
  if (xy_bounds.size() != 4 || result == nullptr ||
      path_search_strategy.use_geometry_strategy.use_purpose.empty() ||
      num_strategy == 0 ||
      path_search_strategy.use_geometry_strategy.longitudal_bound.size() !=
          num_strategy) {
    AERROR << " geometry path generator input check fails";
    return false;
  }
  path_search_strategy_ = path_search_strategy;
  ADEBUG << path_search_strategy_.DebugString();
  // step1 load bound and obstacle segments, etc.
  const double min_radius = GetMinRadius();
  const bool consider_kappa_diff =
      path_search_strategy.use_geometry_strategy.consider_kappa_diff;
  // use purposes in path_search_strategy are same, otherwise need iter all
  const bool pursuit_precise_pose_connect =
      path_search_strategy_.use_geometry_strategy.use_purpose.at(0) ==
      UseGeometryPurpose::PRECISEPOSE;
  xy_bounds_ = xy_bounds;
  static constexpr double kEpision = 1e-3;
  extra_distance_for_geometry_ =
      (path_search_strategy.space_structure == LAT_PARK_LOT &&
       path_search_strategy.park_direction == PARKIN &&
       !pursuit_precise_pose_connect)
          ? 0
          : warm_start_config_.extra_distance_for_geometry_path();
  std::vector<std::pair<common::math::LineSegment2d, double>>
      obstacles_without_virtual;
  std::vector<std::pair<common::math::LineSegment2d, double>>
      obstacles_segments_vec_geometry;
  for (auto obstacles_segments : obstacles_segments_vec) {
    if (obstacles_segments.second > kEpision) {
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
  obstacles_without_virtual.emplace_back(line_1, kEpision);
  obstacles_without_virtual.emplace_back(line_2, kEpision);
  obstacles_without_virtual.emplace_back(line_3, kEpision);
  obstacles_without_virtual.emplace_back(line_4, kEpision);
  obstacles_segments_vec_geometry.emplace_back(line_1, kEpision);
  obstacles_segments_vec_geometry.emplace_back(line_2, kEpision);
  obstacles_segments_vec_geometry.emplace_back(line_3, kEpision);
  obstacles_segments_vec_geometry.emplace_back(line_4, kEpision);
  // step 2 decide cycle direction
  CycleDirection cycle_direction = CycleDirection::ANY;
  CycleDirectionStrategy(pursuit_precise_pose_connect, start_point, end_point,
                         path_search_strategy_.park_direction,
                         &cycle_direction);
  // step 3 generate path
  bool plan_ret = false;
  GeometryPath geometry_path;
  for (size_t i = 0; i < num_strategy; i++) {
    const auto is_cycle_straight_connect =
        path_search_strategy.use_geometry_strategy.geometry_path_type.at(i) ==
        CYCLE_STRAIGHT;
    const bool consider_kappa_diff_temp =
        is_cycle_straight_connect && consider_kappa_diff;
    // 3.1 generate path
    if (!geometry_planner_->GenerateGeometryPath(
            start_point, end_point, is_cycle_straight_connect, min_radius,
            consider_kappa_diff_temp, cycle_direction, &geometry_path,
            pursuit_precise_pose_connect) ||
        geometry_path.empty()) {
      ADEBUG << " Generate GeometryPath fails once";
      continue;
    }
    // for lat park precise angle, load path for fallback
    if (!pursuit_precise_pose_connect &&
        path_search_strategy_.space_structure == LAT_PARK_LOT &&
        !GetResult(pursuit_precise_pose_connect, geometry_path, result)) {
      ADEBUG << "load precise  angle result fail";
      continue;
    }
    // 3.2 judge ptah is reasonable  or not
    if (!IsGeometryPathAcceptable(
            is_cycle_straight_connect, start_point, end_point,
            path_search_strategy.use_geometry_strategy.longitudal_bound.at(i),
            geometry_path)) {
      ADEBUG << " Path is not acceptable";
      continue;
    }
    if (!CollisonFreeCheck(geometry_path, obstacles_without_virtual,
                           obstacles_segments_vec_geometry)) {
      ADEBUG << " generated path is not collision free";
      continue;
    }
    ADEBUG << " geometry path is find";
    plan_ret = true;
    break;
  }
  if (!plan_ret || geometry_path.empty()) {
    AERROR << "geometry planner fails";
    return false;
  }
  // step4 load result
  return GetResult(pursuit_precise_pose_connect, geometry_path, result);
}

bool GeometryPathGenerator::CollisonFreeCheck(
    const GeometryPath& geometry_path,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_without_virtual,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec_geometry) {
  for (const auto& path : geometry_path) {
    if (path.first.x.empty()) {
      continue;
    }
    if (path.first.x.size() != path.first.y.size() ||
        path.first.x.size() != path.first.phi.size()) {
      AERROR << "geometry path is not valid";
      return false;
    }
    std::shared_ptr<Node3d> path_nodes = std::make_shared<Node3d>(
        path.first.x, path.first.y, path.first.phi, xy_bounds_,
        xy_grid_resolution_, phi_grid_resolution_);
    const auto& obs_segment = path.second == GeometryPathType::STRAIGHT
                                  ? obstacles_without_virtual
                                  : obstacles_segments_vec_geometry;
    if (!ValidityCheck(path_nodes, obs_segment)) {
      AERROR << " collision check failed";
      return false;
    }
  }
  return true;
}

void GeometryPathGenerator::CycleDirectionStrategy(
    const bool pursuit_precise_pose_connect,
    const common::PathPoint& start_point, const common::PathPoint& end_point,
    const ParkDirection& park_direction,
    CycleDirection* const cycle_direction) const {
  if (nullptr == cycle_direction || pursuit_precise_pose_connect) {
    *cycle_direction = CycleDirection::ANY;
    return;
  }
  if (path_search_strategy_.space_structure == VER_PARK_LOT &&
      (park_direction == LEFTPARKOUT || park_direction == RIGHTPARKOUT)) {
    // in vertical/oblique, cycle direction is same with park out direction
    *cycle_direction = (park_direction == RIGHTPARKOUT)
                           ? CycleDirection::FORCERIGHT
                           : CycleDirection::FORCELEFT;
    return;
  }
  if (path_search_strategy_.space_structure == LAT_PARK_LOT &&
      park_direction == PARKIN &&
      path_search_strategy_.collision_free_search_strategy
          .replan_due_to_collision) {
    const bool is_right_side_slot =
        fabs(common::math::AngleDiff(end_point.theta(), 0)) <= M_PI_2;
    // -1-- left side of slot 1 -- right side of slot, do not mean gear direction
    int collision_direction = 0;
    if (path_search_strategy_.init_path_direction != 0) {
      collision_direction = is_right_side_slot
                                ? -path_search_strategy_.init_path_direction
                                : path_search_strategy_.init_path_direction;
    } else {
      collision_direction =
          path_search_strategy_.collision_free_search_strategy
                      .collision_path_point.x() <= start_point.x()
              ? -1
              : 1;
    }
    if (collision_direction == -1) {
      *cycle_direction =
          (!is_right_side_slot &&
           common::math::AngleDiff(start_point.theta(), end_point.theta()) < 0)
              ? CycleDirection::FORCELEFT
              : CycleDirection::FORCERIGHT;
    } else {
      *cycle_direction =
          (is_right_side_slot &&
           common::math::AngleDiff(start_point.theta(), end_point.theta()) > 0)
              ? CycleDirection::FORCERIGHT
              : CycleDirection::FORCELEFT;
    }
  }
}
}  // namespace planning
}  // namespace TL
