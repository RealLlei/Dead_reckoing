/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description: path_generator.cc
 */

#include "planning/open_space/coarse_path_generator/path_generator.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <utility>

#include "common/math/double_type.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

using TL::common::math::Vec2d;

PathGenerator::PathGenerator(const WarmStartConfig& warm_start_config)
    : warm_start_config_(warm_start_config),
      xy_grid_resolution_(warm_start_config_.xy_grid_resolution()),
      phi_grid_resolution_(warm_start_config_.phi_grid_resolution()) {}

bool PathGenerator::ValidityCheck(
    const std::shared_ptr<Node3d>& node,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec,
    double* const distance_to_obstalce) {
  CHECK_NOTNULL(node);
  CHECK_GT(node->GetStepSize(), 0U);
  if (distance_to_obstalce != nullptr) {
    *distance_to_obstalce = std::numeric_limits<double>::infinity();
  }
  if (obstacles_segments_vec.empty()) {
    return true;
  }
  size_t node_step_size = node->GetStepSize();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;
  std::vector<size_t> node_id_oder;
  if (node_step_size == 1) {
    node_id_oder.emplace_back(0);
  } else {
    check_start_index = 1;
    node_id_oder.reserve(node_step_size - check_start_index);
    node_id_oder.emplace_back(check_start_index);
    if (node_step_size - check_start_index > 1) {
      node_id_oder.emplace_back(node_step_size - 1);
      InsertWithBinaryOrder(
          std::make_pair(check_start_index, node_step_size - 1), &node_id_oder);
    }
  }
  for (const auto& i : node_id_oder) {
    if (traversed_x[i] > xy_bounds_[1] || traversed_x[i] < xy_bounds_[0] ||
        traversed_y[i] > xy_bounds_[3] || traversed_y[i] < xy_bounds_[2]) {
      return false;
    }
    if (distance_to_obstalce != nullptr) {
      *distance_to_obstalce = common::math::GetMinDistance2ObstaclesSegments(
          traversed_x[i], traversed_y[i], traversed_phi[i],
          obstacles_segments_vec, obstacle_filter_distance_);
      if (TL::common::math::double_type::IsZero(*distance_to_obstalce)) {
        ADEBUG << "Check Collision With Vehicle Polygon2d failed ";
        return false;
      }
    } else {
      if (common::math::CheckCollisionWithVehiclePolygon2d(
              traversed_x[i], traversed_y[i], traversed_phi[i],
              obstacles_segments_vec)) {
        ADEBUG << "Check Collision With Vehicle Polygon2d failed ";
        ADEBUG << "collision path point: " << traversed_x[i] << ", "
               << traversed_y[i] << ", " << traversed_phi[i];
        return false;
      }
    }
  }
  return true;
}

void PathGenerator::InsertWithBinaryOrder(
    const std::pair<size_t, size_t>& par,
    std::vector<size_t>* const vec) const {
  if (vec == nullptr || par.second - par.first <= 1) {
    return;
  }
  std::queue<std::pair<size_t, size_t>> q;
  std::pair<size_t, size_t> pt;
  q.push(par);
  while (!q.empty()) {
    pt = q.front();
    q.pop();
    vec->emplace_back(std::floor((pt.second + pt.first) / 2));
    if (static_cast<size_t>(std::floor((pt.second + pt.first) / 2)) >
        pt.first + 1)
      q.push(std::make_pair(pt.first, std::floor((pt.second + pt.first) / 2)));
    if (pt.second >
        static_cast<size_t>(std::floor((pt.second + pt.first) / 2)) + 1)
      q.push(std::make_pair(std::floor((pt.second + pt.first) / 2), pt.second));
  }
}

bool PathGenerator::PathPartition(
    const PathGeneratorResult& result,
    std::vector<PathGearPair>* const partition_paths) {
  if (partition_paths == nullptr || result.x.size() < 2) {
    AERROR << "input checck fails";
    return false;
  }
  const auto& x = result.x;
  const auto& y = result.y;
  const auto& phi = result.phi;
  if (x.empty() || x.size() != y.size() || x.size() != phi.size()) {
    AERROR << "states sizes are not equal or empty when do path "
              "partitioning of "
              "Hybrid A Star result";
    return false;
  }
  size_t horizon = x.size();
  partition_paths->clear();
  partition_paths->emplace_back();
  auto* current_path = &(partition_paths->back().first);
  double heading_angle = phi.front();
  const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
  double tracking_angle = init_tracking_vector.Angle();
  bool current_gear =
      std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
      (M_PI_2);
  common::PathPoint tmp;
  // to partition the path according to the gears
  for (size_t i = 0; i < horizon - 1; ++i) {
    heading_angle = phi[i];
    const Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
    tracking_angle = tracking_vector.Angle();
    bool gear =
        std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
        (M_PI_2);
    if (gear != current_gear) {
      tmp.set_x(x[i]);
      tmp.set_y(y[i]);
      tmp.set_theta(phi[i]);
      current_path->push_back(tmp);
      partition_paths->emplace_back();
      current_path = &(partition_paths->back().first);
      current_gear = gear;
    }
    tmp.set_x(x[i]);
    tmp.set_y(y[i]);
    tmp.set_theta(phi[i]);
    current_path->push_back(tmp);
  }
  tmp.set_x(x.back());
  tmp.set_y(y.back());
  tmp.set_theta(phi.back());
  current_path->push_back(tmp);

  return true;
}

}  // namespace planning
}  // namespace TL
