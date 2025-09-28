/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  safety_guard_planner.cc
 */

#include "planning/planner/safety_guard/safety_guard_planner.h"

#include <algorithm>
#include <limits>
#include "common/math/linear_interpolation.h"

#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "proto/fsm/avp_fct.pb.h"
#include "proto/fsm/function_manager.pb.h"

namespace TL {
namespace planning {

using TL::common::Status;
using TL::common::math::double_type::DefinitelyLessEqual;
using TL::common::math::double_type::IsZero;

namespace {
constexpr double kSampleDeltaS = 0.02;
constexpr double kMaxSampleS = 2.0;
constexpr double kEpsilon = 1e-5;
constexpr double kOppositeSteerAngleThreshold = 0.1;
}  // namespace

SafetyGuardPlanner::SafetyGuardPlanner(
    const std::shared_ptr<DependencyInjector>& injector)
    : Planner(injector),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()),
      path_point_num_(static_cast<size_t>(kMaxSampleS / kSampleDeltaS + 1)) {}

Status SafetyGuardPlanner::Init(const PlanningConfig& config) {
  config_ = config;
  return Status::OK();
}

Status SafetyGuardPlanner::Plan(
    const common::VehicleState& vehicle_state,
    const TL::planning::LocalView& local_view,
    ADCTrajectory* const ptr_computed_trajectory) const {
  DCHECK(nullptr != ptr_computed_trajectory);
  if ((!local_view.HasFunctionManagerIn()) || (!vehicle_state.has_gear()) ||
      (vehicle_state.gear() != soc::Chassis::GEAR_DRIVE &&
       vehicle_state.gear() != soc::Chassis::GEAR_REVERSE)) {
    return Status::OK();
  }

  const auto& guard_config = config_.safety_guard_planning_config();
  std::vector<std::pair<std::string, std::vector<common::PathPoint>>>
      discretized_paths;
  GenerateDiscretizedPaths(vehicle_state, guard_config.steer_angle_offset(),
                           &discretized_paths);

  const double v_speed = std::max(std::fabs(vehicle_state.linear_velocity()),
                                  guard_config.predict_min_speed());
  const double dec_t = std::sqrt(2.0 * v_speed * guard_config.press_up_time() /
                                 std::fabs(guard_config.max_deceleration()));
  const double delay_ds = v_speed * (guard_config.control_delay_time() +
                                     guard_config.transmission_delay_time());

  const auto& fct_in = local_view.GetFunctionManagerIn();
  double sensor_dead_zone_distance_with_buffer =
      guard_config.sensor_dead_zone_distance() *
      guard_config.sensor_dead_zone_distance_ratio();
  if (fct_in->fct_avp_in().sys_mode() == functionmanager::AvpFctIn::RPA &&
      (fct_in->fct_avp_in().sys_command() ==
           functionmanager::AvpFctIn::FORWARDCONTROL ||
       fct_in->fct_avp_in().sys_command() ==
           functionmanager::AvpFctIn::BACKWARDCONTROL)) {
    sensor_dead_zone_distance_with_buffer =
        guard_config.sensor_dead_zone_distance() *
        guard_config.rpa_straight_control_sensor_dead_zone_distance_ratio();
  } else if ((fct_in->fct_avp_in().sys_mode() ==
                  functionmanager::AvpFctIn::LAPA &&
              fct_in->fct_avp_in().sys_command() ==
                  functionmanager::AvpFctIn::PARKINPOLIT) ||
             (fct_in->fct_avp_in().sys_mode() ==
                  functionmanager::AvpFctIn::TBA &&
              fct_in->fct_avp_in().sys_command() ==
                  functionmanager::AvpFctIn::TBACONTROL)) {
    sensor_dead_zone_distance_with_buffer =
        guard_config.sensor_dead_zone_distance() *
        guard_config.cruise_sensor_dead_zone_distance_ratio();
  }
  double press_up_ds = 0.0;
  double compress_ds = 0.0;
  if (dec_t < guard_config.press_up_time()) {
    press_up_ds = v_speed * dec_t + guard_config.max_deceleration() /
                                        guard_config.press_up_time() * dec_t *
                                        dec_t * dec_t / 6.0;
  } else {
    press_up_ds = v_speed * guard_config.press_up_time() +
                  guard_config.max_deceleration() *
                      guard_config.press_up_time() *
                      guard_config.press_up_time() / 6.0;
    double press_up_v = v_speed + guard_config.max_deceleration() *
                                      guard_config.press_up_time() / 2.0;
    compress_ds = press_up_v * press_up_v /
                  std::fabs(guard_config.max_deceleration()) / 2.0;
  }
  const double min_safe_distance = delay_ds + press_up_ds + compress_ds +
                                   sensor_dead_zone_distance_with_buffer;
  auto* safety_guard_info_ptr =
      ptr_computed_trajectory->mutable_debug()->mutable_safety_guard_info();
  safety_guard_info_ptr->set_delay_ds(delay_ds);
  safety_guard_info_ptr->set_press_up_ds(press_up_ds);
  safety_guard_info_ptr->set_compress_ds(compress_ds);
  safety_guard_info_ptr->set_sensor_dead_zone_distance_with_buffer(
      sensor_dead_zone_distance_with_buffer);
  safety_guard_info_ptr->set_min_safe_distance(min_safe_distance);
  if (local_view.HasPerceptionObstacles()) {
    ProcessUSSObstacle(*local_view.GetPerceptionObstacles(), discretized_paths,
                       vehicle_state.gear(), safety_guard_info_ptr);
  }
  if (guard_config.enable_free_space_guard() &&
      local_view.HasFreeSpaceOutArray()) {
    ProcessFreeSpace(*local_view.GetFreeSpaceOutArray(), discretized_paths,
                     vehicle_state, safety_guard_info_ptr);
  }
  return Status::OK();
}

void SafetyGuardPlanner::ProcessUSSObstacle(
    const perception::PerceptionObstacles& perception_obstacles,
    const std::vector<std::pair<std::string, std::vector<common::PathPoint>>>&
        discretized_paths,
    const soc::Chassis::GearPosition& gear_position,
    planning_internal::SafetyGuardInfo* const safety_guard_info_ptr) const {
  DCHECK(nullptr != safety_guard_info_ptr);
  double distance_to_uss = 0.0;
  std::string collision_path_label;
  int32_t collision_uss_id = -1;
  if (!CheckCollisionWithUSSObstacle(
          perception_obstacles, discretized_paths, gear_position,
          &distance_to_uss, &collision_path_label, &collision_uss_id)) {
    return;
  }

  safety_guard_info_ptr->set_min_distance_to_uss(distance_to_uss);
  safety_guard_info_ptr->set_collision_uss_path_label(collision_path_label);
  safety_guard_info_ptr->set_collision_uss_id(collision_uss_id);
  if (distance_to_uss > safety_guard_info_ptr->min_safe_distance()) {
    safety_guard_info_ptr->set_is_real_time_triggered_uss(false);
    return;
  }

  safety_guard_info_ptr->set_is_real_time_triggered_uss(true);
}

void SafetyGuardPlanner::ProcessFreeSpace(
    const perception::FreeSpaceOutArray& free_space,
    const std::vector<std::pair<std::string, std::vector<common::PathPoint>>>&
        discretized_paths,
    const common::VehicleState& vehicle_state,
    planning_internal::SafetyGuardInfo* const safety_guard_info_ptr) const {
  DCHECK(nullptr != safety_guard_info_ptr);
  double distance_to_free_space = 0.0;
  if (!CheckCollisionWithFreeSpace(free_space, discretized_paths, vehicle_state,
                                   &distance_to_free_space)) {
    return;
  }

  safety_guard_info_ptr->set_min_distance_to_free_space(distance_to_free_space);
  if (distance_to_free_space > safety_guard_info_ptr->min_safe_distance()) {
    safety_guard_info_ptr->set_is_real_time_triggered_free_space(false);
    return;
  }

  safety_guard_info_ptr->set_is_real_time_triggered_free_space(true);
}

void SafetyGuardPlanner::GenerateDiscretizedPath(
    const soc::Chassis::GearPosition& gear_position, const double steer_angle,
    std::vector<common::PathPoint>* const path_points) const {
  DCHECK(nullptr != path_points);
  path_points->clear();
  const double sign = gear_position == soc::Chassis::GEAR_REVERSE ? -1.0 : 1.0;
  common::PathPoint cur_point;
  cur_point.set_x(0.0);
  cur_point.set_y(0.0);
  cur_point.set_theta(0.0);
  cur_point.set_s(0.0);
  path_points->push_back(cur_point);
  for (size_t i = 1; i < path_point_num_; i++) {
    double cpx = path_points->back().x() +
                 sign * kSampleDeltaS * std::cos(path_points->back().theta());
    double cpy = path_points->back().y() +
                 sign * kSampleDeltaS * std::sin(path_points->back().theta());
    double cptheta = path_points->back().theta() +
                     sign * kSampleDeltaS * std::tan(steer_angle) /
                         vehicle_param_.wheel_base();
    double cps = path_points->back().s() + kSampleDeltaS;
    cur_point.set_x(cpx);
    cur_point.set_y(cpy);
    cur_point.set_theta(cptheta);
    cur_point.set_s(cps);
    path_points->push_back(cur_point);
  }
}

void SafetyGuardPlanner::GenerateDiscretizedPaths(
    const common::VehicleState& vehicle_state, const double dphi,
    std::vector<std::pair<std::string, std::vector<common::PathPoint>>>* const
        discretized_paths) const {
  DCHECK(nullptr != discretized_paths);
  DCHECK_GT(dphi, kEpsilon);
  const double center_steer_angle = vehicle_state.steering_percentage() / 100 *
                                    vehicle_param_.max_steer_angle();
  const double center_beta_angle =
      common::VehicleStateProvider::EstimateBetaAngle(center_steer_angle,
                                                      vehicle_param_);
  double left_beta_angle = 0.0;
  double right_beta_angle = 0.0;
  if (IsZero(center_beta_angle)) {
    left_beta_angle = dphi;
    right_beta_angle = -dphi;
  } else if (DefinitelyLessEqual(center_beta_angle, 0.0)) {
    left_beta_angle =
        std::min(center_beta_angle + dphi, kOppositeSteerAngleThreshold);
    right_beta_angle = center_beta_angle - dphi;
  } else {
    left_beta_angle = center_beta_angle + dphi;
    right_beta_angle =
        std::max(center_beta_angle - dphi, -kOppositeSteerAngleThreshold);
  }

  std::vector<common::PathPoint> cur_path;
  cur_path.reserve(path_point_num_);
  GenerateDiscretizedPath(vehicle_state.gear(), center_beta_angle, &cur_path);
  discretized_paths->emplace_back("center line", std::move(cur_path));
  if (vehicle_state.gear() == soc::Chassis::GEAR_DRIVE) {
    std::vector<common::PathPoint> left_path;
    left_path.reserve(path_point_num_);
    GenerateDiscretizedPath(vehicle_state.gear(), left_beta_angle, &left_path);
    discretized_paths->emplace_back("left line", std::move(left_path));

    std::vector<common::PathPoint> right_path;
    right_path.reserve(path_point_num_);
    GenerateDiscretizedPath(vehicle_state.gear(), right_beta_angle,
                            &right_path);
    discretized_paths->emplace_back("right line", std::move(right_path));
  }
}

bool SafetyGuardPlanner::CheckCollisionWithUSSObstacle(
    const perception::PerceptionObstacles& perception_obstacles,
    const std::vector<std::pair<std::string, std::vector<common::PathPoint>>>&
        discretized_paths,
    const soc::Chassis::GearPosition& gear_position,
    double* const distance_to_uss, std::string* const path_label,
    int32_t* const collision_uss_id) const {
  DCHECK(nullptr != distance_to_uss);
  DCHECK(nullptr != path_label);
  DCHECK(nullptr != collision_uss_id);
  bool has_collision = false;
  *distance_to_uss = std::numeric_limits<double>::max();
  path_label->clear();
  *collision_uss_id = -1;
  if (perception_obstacles.perception_obstacle().empty()) {
    return has_collision;
  }

  const double collision_buffer =
      gear_position == soc::Chassis::GEAR_REVERSE
          ? config_.safety_guard_planning_config().collision_buffer()
          : 0.0;
  for (size_t i = 0; i < path_point_num_; i++) {
    for (const auto& path : discretized_paths) {
      const auto& point = path.second.at(i);
      const auto v_polygon2d =
          common::VehicleConfigHelper::GetPolygon2dWithBuffer(
              point.x(), point.y(), point.theta(), 0.0, 0.0, collision_buffer,
              collision_buffer);
      for (const auto& obj : perception_obstacles.perception_obstacle()) {
        if (obj.sub_type() != perception::PerceptionObstacle::ST_USS) {
          continue;
        }
        const auto uss_point =
            common::math::Vec2d(obj.position_flu().x(), obj.position_flu().y());
        if (IsFarAwayFromVehicle(point.x(), point.y(), point.theta(), uss_point,
                                 collision_buffer)) {
          continue;
        }
        if (v_polygon2d.IsPointIn(uss_point)) {
          *distance_to_uss = std::max(0.0, point.s() - kSampleDeltaS);
          *path_label = path.first;
          *collision_uss_id = obj.id();
          has_collision = true;
          break;
        }
      }
      if (has_collision) {
        break;
      }
    }
    if (has_collision) {
      break;
    }
  }

  return has_collision;
}

bool SafetyGuardPlanner::CheckCollisionWithFreeSpace(
    const perception::FreeSpaceOutArray& free_space,
    const std::vector<std::pair<std::string, std::vector<common::PathPoint>>>&
        discretized_paths,
    const common::VehicleState& vehicle_state,
    double* const distance_to_free_space) const {
  DCHECK(nullptr != distance_to_free_space);
  bool has_collision = false;
  *distance_to_free_space = std::numeric_limits<double>::max();
  std::vector<common::math::Vec2d> freespace_point_flu;
  for (const auto& fp_enu : free_space.freespace_out()) {
    for (const auto& enu_p : fp_enu.freespace_keypoint()) {
      const auto fp_flu_pair =
          common::math::ENUToFLU(enu_p.x(), enu_p.y(), vehicle_state.x(),
                                 vehicle_state.y(), vehicle_state.heading());
      freespace_point_flu.emplace_back(fp_flu_pair.first, fp_flu_pair.second);
    }
  }
  if (freespace_point_flu.empty()) {
    return has_collision;
  }

  for (size_t i = 0; i < path_point_num_; i++) {
    for (const auto& path : discretized_paths) {
      const auto& point = path.second.at(i);
      const auto v_polygon2d =
          common::VehicleConfigHelper::GetPolygon2dWithBuffer(
              point.x(), point.y(), point.theta());
      for (const auto& fp_flu : freespace_point_flu) {
        if (IsFarAwayFromVehicle(point.x(), point.y(), point.theta(), fp_flu,
                                 0.0)) {
          continue;
        }
        if (v_polygon2d.IsPointIn(fp_flu)) {
          *distance_to_free_space = std::max(0.0, point.s() - kSampleDeltaS);
          has_collision = true;
          break;
        }
      }
      if (has_collision) {
        break;
      }
    }
    if (has_collision) {
      break;
    }
  }

  return has_collision;
}

bool SafetyGuardPlanner::IsFarAwayFromVehicle(
    const double vehicle_x, const double vehicle_y, const double vehicle_theta,
    const common::math::Vec2d& point, const double collision_buffer) const {
  const double diff = (vehicle_param_.front_edge_to_center() -
                       vehicle_param_.back_edge_to_center()) /
                      2.0;
  common::math::Vec2d true_center(vehicle_x + diff * std::cos(vehicle_theta),
                                  vehicle_y + diff * std::sin(vehicle_theta));
  const auto unit_vec2d = common::math::Vec2d::CreateUnitVec2d(vehicle_theta);
  auto center_to_point = point - true_center;
  double product = unit_vec2d.CrossProd(center_to_point);
  double project = unit_vec2d.InnerProd(center_to_point);
  const double min_lat_distance =
      vehicle_param_.width() / 2.0 + collision_buffer + kEpsilon;
  const double min_lon_distance =
      vehicle_param_.length() / 2.0 + collision_buffer + kEpsilon;

  return std::fabs(product) > min_lat_distance ||
         std::fabs(project) > min_lon_distance;
}

}  // namespace planning
}  // namespace TL
