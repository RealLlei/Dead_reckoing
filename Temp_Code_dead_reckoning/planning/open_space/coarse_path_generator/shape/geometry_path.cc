/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  open_space_obstacle.h
 */

#include "planning/open_space/coarse_path_generator/shape/geometry_path.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <iterator>
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {
using TL::common::math::Vec2d;

namespace {
constexpr double kEpsilon = 1.0e-3;
constexpr double kDefaultInterpolatedDeltaS = 0.1;
constexpr double kOnLineThrehold = 3e-2;
constexpr double kAngleSameThrehold = 1e-2;
constexpr double kAlmostStraightKappa = 3.0e-2;
}  // namespace

Geometry::Geometry(const common::VehicleParam& vehicle_param,
                   const WarmStartConfig& warm_start_config)
    : vehicle_param_(vehicle_param), warm_start_config_(warm_start_config) {}

bool Geometry::GenerateGeometryPath(
    const common::PathPoint& start_node, const common::PathPoint& end_node,
    const bool is_cycle_straight_connect, const double min_radius,
    const bool consider_kappa_diff, const CycleDirection& cycle_direction,
    GeometryPath* const geometry_path, const bool precise_pose_connect) {
  if (geometry_path == nullptr) {
    AERROR << "GenerateGeometryPath input check fails";
    return false;
  }
  geometry_path->clear();
  const double angle_diff =
      common::math::AngleDiff(start_node.theta(), end_node.theta());
  const double cos_from_point = cos(start_node.theta());
  const double sin_from_point = sin(start_node.theta());
  const Vec2d start_end_vec(end_node.x() - start_node.x(),
                            end_node.y() - start_node.y());
  // If start point and end point difference in Angle is less than threshold
  // considered as colliner
  if (fabs(angle_diff) <= kAngleSameThrehold) {
    const double from_end_node_lat_distance =
        fabs(cos_from_point * start_end_vec.y() -
             sin_from_point * start_end_vec.x());
    if (from_end_node_lat_distance >
        fmax(kOnLineThrehold, start_end_vec.Length() * fabs(angle_diff))) {
      AERROR << "from and to points have same angle,  but not on same line";
      return false;
    }
    ADEBUG << "from and to points can connected by straight line";
    return LoadStraightPath(start_node, end_node, geometry_path);
  }
  // input from pursuit_precise_pose_connect
  if (precise_pose_connect) {
    ADEBUG << " connect purpose is precise pose";
    const auto& start_node_temp =
        is_cycle_straight_connect ? start_node : end_node;
    const auto& end_node_temp =
        is_cycle_straight_connect ? end_node : start_node;
    if (!GeneratePrecisePosePath(start_node_temp, end_node_temp, min_radius,
                                 consider_kappa_diff, geometry_path)) {
      AERROR << "generate precise pose path fail";
      return false;
    }
    // decide use straight or cycle path first
    // default is straight_cycle path(straight first)
    if (is_cycle_straight_connect) {
      ReversePath(geometry_path);
    }
    return true;
  }
  ADEBUG << " connect purpose is precise angle";
  return GeneratePreciseAnglePath(start_node, end_node, cycle_direction,
                                  min_radius, geometry_path);
}

bool Geometry::GeneratePrecisePosePath(const common::PathPoint& start_node,
                                       const common::PathPoint& end_node,
                                       const double min_radius,
                                       const bool consider_kappa_diff,
                                       GeometryPath* const geometry_path) {
  if (geometry_path == nullptr) {
    AERROR << "GeneratePrecisePosePath input check fails";
    return false;
  }
  /****** l+r*sin(dphi)=x;r(1-cos(dphi))=y;s=r*dphi;****/
  // convert local coordinate to strat point and get the end point(x,y,dphi) to caculate the r,s,l
  // Normalized conversion to local coordinate
  const double relative_angle =
      common::math::NormalizeAngle(start_node.theta() - end_node.theta());
  const double cos_relative_angle = cos(relative_angle);
  double radius = 0;
  if (fabs(relative_angle) > M_PI_4 * 3 ||
      fabs(1 - cos_relative_angle) <= kEpsilon) {
    AERROR << " from and  to point angle diff is too large or two small";
    return false;
  }
  Vec2d relative_point(start_node.x() - end_node.x(),
                       start_node.y() - end_node.y());
  // Rotate the vector from the end point to the start point to the X-axis
  relative_point = relative_point.rotate(-end_node.theta());
  radius = relative_point.y() / (1 - cos_relative_angle);
  if (std::fabs(radius) < min_radius) {
    AERROR << " radius is too small, raiud: " << radius;
    return false;
  }
  if (consider_kappa_diff && 1 / fabs(radius) > kAlmostStraightKappa &&
      fabs(start_node.kappa()) >= kAlmostStraightKappa &&
      radius * start_node.kappa() < 0) {
    AERROR << "start point kappa and geometry path kappa diff is too large";
    return false;
  }
  // default is straight_cycle path
  // load straight path
  common::PathPoint straight_start_point;
  common::PathPoint intermedate_point;
  const double sin_relative_angle = sin(relative_angle);
  intermedate_point.set_x(relative_point.x() - radius * sin_relative_angle);
  if (!LoadStraightPath(straight_start_point, intermedate_point,
                        geometry_path)) {
    AERROR << "load straight path fails";
    return false;
  }
  // load cycle path
  Vec2d radius_center(intermedate_point.x(), radius);
  if (!LoadCyclePath(intermedate_point, relative_point, radius_center, radius,
                     fabs(radius * relative_angle), geometry_path)) {
    AERROR << "load cycle path fails";
    return false;
  }
  // convert local to orignal coordinate
  if (!LocalCoordinateTransform(end_node, geometry_path)) {
    AERROR << " local transform fail";
    return false;
  }

  return true;
}

bool Geometry::GeneratePreciseAnglePath(const common::PathPoint& start_node,
                                        const common::PathPoint& end_node,
                                        const CycleDirection& cycle_direction,
                                        const double min_radius,
                                        GeometryPath* const path_result) {
  if (path_result == nullptr) {
    AERROR << "GeneratePreciseAnglePath input check fails";
    return false;
  }
  const double angle_diff =
      common::math::AngleDiff(start_node.theta(), end_node.theta());
  if (fabs(angle_diff) > M_PI_2) {
    AERROR << " from and  to point angle diff is too large";
    return false;
  }
  const double cos_from_point = cos(start_node.theta());
  const double sin_from_point = sin(start_node.theta());
  const double cos_to_point = cos(end_node.theta());
  const double sin_to_point = sin(end_node.theta());
  const common::math::Vec2d end_node_vec = Vec2d(end_node.x(), end_node.y());
  Vec2d radius_center;
  const auto unit_direction = Vec2d::CreateUnitVec2d(end_node.theta());
  Vec2d intermedate_point_vec;
  double radius = min_radius;
  // calculate_intermediate point
  // First calculate the radius of the circle(turn left or right),
  // then calculate end point of circle as intermediate point
  // center one: xo =x_from_point + r* sin(theta_point), yo= y_from_point -r* cos(theta_point)
  const Vec2d center_one(start_node.x() + min_radius * sin_from_point,
                         start_node.y() - min_radius * cos_from_point);
  const Vec2d intermedate_point_vec_one =
      center_one + Vec2d(-min_radius * sin_to_point, min_radius * cos_to_point);
  // center two: xo =x_from_point - r* sin(theta_point), yo= y_from_point +r* cos(theta_point)
  const Vec2d center_two(start_node.x() - min_radius * sin_from_point,
                         start_node.y() + min_radius * cos_from_point);
  const Vec2d intermedate_point_vec_two =
      center_two + Vec2d(min_radius * sin_to_point, -min_radius * cos_to_point);
  if (cycle_direction == CycleDirection::FORCERIGHT) {
    ADEBUG << "forced right cycle";
    intermedate_point_vec = intermedate_point_vec_one;
    radius_center = center_one;
    radius = -min_radius;
  } else if (cycle_direction == CycleDirection::FORCELEFT) {
    ADEBUG << "forced left cycle";
    intermedate_point_vec = intermedate_point_vec_two;
    radius_center = center_two;
  } else {
    const double dist_to_unit_direction_one = fabs(
        (intermedate_point_vec_one - end_node_vec).CrossProd(unit_direction));
    const double dist_to_unit_direction_two = fabs(
        (intermedate_point_vec_two - end_node_vec).CrossProd(unit_direction));
    ADEBUG << "dist_to_unit_direction_one: " << dist_to_unit_direction_one;
    ADEBUG << "dist_to_unit_direction_two: " << dist_to_unit_direction_two;
    if (dist_to_unit_direction_one <= dist_to_unit_direction_two) {
      intermedate_point_vec = intermedate_point_vec_one;
      radius_center = center_one;
      radius = -min_radius;
    } else {
      intermedate_point_vec = intermedate_point_vec_two;
      radius_center = center_two;
    }
  }
  ADEBUG << "radius: " << radius;
  ADEBUG << "intermedate_point_vec: " << intermedate_point_vec.x() << ","
         << intermedate_point_vec.y();
  ADEBUG << "radius_center: " << radius_center.x() << "," << radius_center.y();

  //  load cycle path
  if (!LoadCyclePath(start_node, intermedate_point_vec, radius_center, radius,
                     fabs(radius * angle_diff), path_result)) {
    AERROR << "load cycle path fail";
    return false;
  }
  //  load straight path
  common::PathPoint intermedate_point;
  intermedate_point.set_x(intermedate_point_vec.x());
  intermedate_point.set_y(intermedate_point_vec.y());
  intermedate_point.set_theta(end_node.theta());
  const Vec2d relax_end_point_vec =
      intermedate_point_vec +
      (end_node_vec - intermedate_point_vec).InnerProd(unit_direction) *
          unit_direction;
  // make relax_end_point colliner with end_node in Lat direction
  common::PathPoint relax_end_point;
  relax_end_point.set_x(relax_end_point_vec.x());
  relax_end_point.set_y(relax_end_point_vec.y());
  relax_end_point.set_theta(end_node.theta());
  return LoadStraightPath(intermedate_point, relax_end_point, path_result);
}

bool Geometry::LoadStraightPath(const common::PathPoint& start_node,
                                const common::PathPoint& end_node,
                                GeometryPath* const path_result) {
  if (nullptr == path_result) {
    AERROR << "LoadStraightPath input check fails";
    return false;
  }
  const double length = std::sqrt(
      (start_node.x() - end_node.x()) * (start_node.x() - end_node.x()) +
      (start_node.y() - end_node.y()) * (start_node.y() - end_node.y()));
  if (length <= warm_start_config_.min_one_direction_length()) {
    ADEBUG << "length is too small, do not load";
    return true;
  }
  const int interpolated_point_num =
      std::max(static_cast<int>(length / kDefaultInterpolatedDeltaS) - 1, 0);
  soc::Chassis::GearPosition gear =
      soc::Chassis::GearPosition::Chassis_GearPosition_GEAR_NEUTRAL;
  if (!common::math::GetGearFromPath(start_node, end_node, &gear)) {
    AERROR << "infer gear fail";
    return false;
  }
  const int move_direction =
      (gear == soc::Chassis::GearPosition::Chassis_GearPosition_GEAR_REVERSE)
          ? -1
          : 1;
  ADEBUG << "move direction: " << move_direction;
  const double cos_theta = cos(start_node.theta());
  const double sin_theta = sin(start_node.theta());
  const double delta_x =
      move_direction * kDefaultInterpolatedDeltaS * cos_theta;
  const double delta_y =
      move_direction * kDefaultInterpolatedDeltaS * sin_theta;
  std::pair<GeometryOnePathPoint, GeometryPathType> straight_path;
  straight_path.second = GeometryPathType::STRAIGHT;
  for (auto j = 0; j <= interpolated_point_num; j++) {
    const double current_x = start_node.x() + j * delta_x;
    const double current_y = start_node.y() + j * delta_y;
    straight_path.first.x.emplace_back(current_x);
    straight_path.first.y.emplace_back(current_y);
    straight_path.first.phi.emplace_back(start_node.theta());
  }
  straight_path.first.x.emplace_back(end_node.x());
  straight_path.first.y.emplace_back(end_node.y());
  straight_path.first.phi.emplace_back(end_node.theta());
  path_result->emplace_back(straight_path);
  return true;
}

bool Geometry::LoadCyclePath(const common::PathPoint& start_node,
                             const common::math::Vec2d& end_point,
                             const common::math::Vec2d& radius_center,
                             const double radius, const double cycle_length,
                             GeometryPath* const path_result) {
  if (nullptr == path_result || fabs(radius) < kEpsilon ||
      cycle_length < kEpsilon) {
    AERROR << "LoadCyclePath input check fails";
    return false;
  }
  const int interpolated_point_num = std::max(
      static_cast<int>(cycle_length / kDefaultInterpolatedDeltaS) - 1, 0);
  const double rotation_direction =
      (Vec2d(start_node.x(), start_node.y()) - radius_center)
                  .CrossProd(Vec2d(end_point.x(), end_point.y()) -
                             radius_center) >= 0
          ? 1
          : -1;
  const double delta_angle =
      kDefaultInterpolatedDeltaS / fabs(radius) * rotation_direction;
  std::pair<GeometryOnePathPoint, GeometryPathType> cycle_path;
  cycle_path.second = GeometryPathType::CYCLE;
  double current_angle = start_node.theta();
  for (auto j = 0; j <= interpolated_point_num; j++) {
    current_angle = start_node.theta() + j * delta_angle;
    cycle_path.first.x.emplace_back(radius_center.x() +
                                    radius * sin(current_angle));
    cycle_path.first.y.emplace_back(radius_center.y() -
                                    radius * cos(current_angle));
    cycle_path.first.phi.emplace_back(current_angle);
  }
  cycle_path.first.x.emplace_back(end_point.x());
  cycle_path.first.y.emplace_back(end_point.y());
  cycle_path.first.phi.emplace_back(
      start_node.theta() + cycle_length / fabs(radius) * rotation_direction);
  path_result->emplace_back(cycle_path);
  return true;
}

bool Geometry::LocalCoordinateTransform(const common::PathPoint& start_node,
                                        GeometryPath* const geometry_path) {
  if (nullptr == geometry_path) {
    AERROR << "LocalCoordinateTransform input check fails";
    return false;
  }
  const double translation_x = start_node.x();
  const double translation_y = start_node.y();
  const double rotation_angle = start_node.theta();
  Vec2d temp_point;
  for (auto path_ptr = geometry_path->begin(); path_ptr < geometry_path->end();
       path_ptr++) {
    auto* result_x = &(path_ptr->first.x);
    auto* result_y = &(path_ptr->first.y);
    auto* result_phi = &(path_ptr->first.phi);
    const size_t n = result_x->size();
    if (n != result_y->size() || n != result_phi->size()) {
      AERROR << " geometry path is not valid";
      return false;
    }
    for (size_t i = 0; i < n; i++) {
      temp_point = Vec2d(result_x->at(i), result_y->at(i));
      temp_point = temp_point.rotate(rotation_angle);
      result_x->at(i) = translation_x + temp_point.x();
      result_y->at(i) = translation_y + temp_point.y();
      result_phi->at(i) =
          common::math::NormalizeAngle(result_phi->at(i) + rotation_angle);
    }
  }
  return true;
}

void Geometry::ReversePath(GeometryPath* const geometry_path) {
  if (nullptr == geometry_path) {
    AERROR << "ReversePath input check fails";
    return;
  }
  std::reverse(geometry_path->begin(), geometry_path->end());
  for (auto path = geometry_path->begin(); path < geometry_path->end();
       path++) {
    std::reverse(path->first.x.begin(), path->first.x.end());
    std::reverse(path->first.y.begin(), path->first.y.end());
    std::reverse(path->first.phi.begin(), path->first.phi.end());
  }
}

}  // namespace planning
}  // namespace TL
