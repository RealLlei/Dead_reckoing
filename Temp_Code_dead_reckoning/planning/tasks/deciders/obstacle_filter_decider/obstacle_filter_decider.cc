/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Description:  obstacle_filter_decider.cc
 */

#include "planning/tasks/deciders/obstacle_filter_decider/obstacle_filter_decider.h"

#include <algorithm>
#include <cmath>

#include "common/configs/config_gflags.h"
#include "common/util/util.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/reference_line_info.h"
#include "planning/proto/reference_line_smoother_config.pb.h"

namespace TL {
namespace planning {
namespace {
constexpr double kDefaultLaneWidth = 1.5;
}

using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::util::IsFloatEqual;

ObstacleFilterDecider::ObstacleFilterDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {}

Status ObstacleFilterDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  const PathBound& path_bounds = reference_line_info->GetAstarSearchBound();
  if (path_bounds.size() < 2) {
    const std::string msg =
        "Obstacle Filter Decider path_bounds is less than 2!";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_OBSTACLEFILTER_ERROR, msg);
  }
  const double path_bounds_resolution =
      std::get<0>(path_bounds[1]) - std::get<0>(path_bounds[0]);
  const auto vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  for (const auto* const obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }

    if (obstacle->PerceptionSLBoundary().start_s() >
        reference_line_info->reference_line().Length() -
            FLAGS_hdmap_avp_path_extend_buffer) {
      ObjectDecisionType object_decision;
      object_decision.mutable_ignore();
      reference_line_info->path_decision()->AddLateralDecision(
          "ObstacleFilterDecider/out destination", obstacle->Id(),
          object_decision);
      ADEBUG << "SDistanceToDestination: "
             << reference_line_info->SDistanceToDestination();
      ADEBUG << "obstacle start s: "
             << obstacle->PerceptionSLBoundary().start_s();
      ADEBUG << "acd end s: " << reference_line_info->AdcSlBoundary().end_s();
      ADEBUG << "static obstacle_id: " << obstacle->Id() << ", is ignore!";
      continue;
    }
    if (obstacle->IsStatic()) {
      FilterStaticObstacle(path_bounds_resolution, path_bounds, *obstacle,
                           reference_line_info);
    } else {
      FilterDynamicObstacle(path_bounds_resolution, path_bounds, *obstacle,
                            reference_line_info);
    }
  }
  return Status::OK();
}

bool ObstacleFilterDecider::CheckObstacleIsBlockedBounds(
    const PathBound& path_bounds, const Obstacle& obstacle,
    const double path_bounds_resolution) {
  if (IsFloatEqual(path_bounds_resolution, 0.0)) {
    return false;
  }
  static constexpr double kWidthToLaneEdge = 0.1;
  const double vehicle_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width();
  const double filter_width = vehicle_width + kWidthToLaneEdge +
                              FLAGS_adc_l_buffer_for_static_obstacle_avp_mode;
  const double start_s = obstacle.PerceptionSLBoundary().start_s();
  const double end_s = obstacle.PerceptionSLBoundary().end_s();
  const double start_l = obstacle.PerceptionSLBoundary().start_l();
  const double end_l = obstacle.PerceptionSLBoundary().end_l();
  const double middle_s = 0.5 * (start_s + end_s);
  double left_passable_width = 0.0;
  double right_passable_width = 0.0;
  const int start_index = static_cast<int>(
      (start_s - std::get<0>(path_bounds.front())) / path_bounds_resolution);
  const int end_index = static_cast<int>(
      (end_s - std::get<0>(path_bounds.front())) / path_bounds_resolution);
  const int middle_index = static_cast<int>(
      (middle_s - std::get<0>(path_bounds.front())) / path_bounds_resolution);

  if (0 <= start_index && start_index < static_cast<int>(path_bounds.size())) {
    left_passable_width = std::get<2>(path_bounds[start_index]) - end_l;
    right_passable_width = start_l - std::get<1>(path_bounds[start_index]);
    if (left_passable_width < filter_width &&
        right_passable_width < filter_width) {
      return true;
    }
  }

  if (0 <= end_index && end_index < static_cast<int>(path_bounds.size())) {
    left_passable_width = std::get<2>(path_bounds[end_index]) - end_l;
    right_passable_width = start_l - std::get<1>(path_bounds[end_index]);
    if (left_passable_width < filter_width &&
        right_passable_width < filter_width) {
      return true;
    }
  }
  if (0 <= middle_index &&
      middle_index < static_cast<int>(path_bounds.size())) {
    left_passable_width = std::get<2>(path_bounds[middle_index]) - end_l;
    right_passable_width = start_l - std::get<1>(path_bounds[middle_index]);
    if (left_passable_width < filter_width &&
        right_passable_width < filter_width) {
      return true;
    }
  }
  return false;
}

bool ObstacleFilterDecider::CheckEdgeIsOutBounds(
    const bool is_pedestrian, const PathBound& path_bounds,
    const double check_s, const double check_start_l, const double check_end_l,
    const double path_bounds_resolution) const {
  if (IsFloatEqual(path_bounds_resolution, 0.0)) {
    return false;
  }
  const double check_static_obs_distance =
      config_.obstacle_filter_decider_config().check_static_obs_distance();

  int index = static_cast<int>((check_s - std::get<0>(path_bounds.front())) /
                               path_bounds_resolution);
  if (frame_->GetMachineStateType() !=
      functionmanager::MachineStateType::HISTORY_TRACE_TYPE) {
    if (index < 0 && index > -static_cast<int>(check_static_obs_distance /
                                               path_bounds_resolution)) {
      index = 0;
    } else if (index <= -static_cast<int>(check_static_obs_distance /
                                          path_bounds_resolution)) {
      return true;
    }
  } else {
    if (index >= static_cast<int>(path_bounds.size()) &&
        index < static_cast<int>(path_bounds.size()) +
                    static_cast<int>(check_static_obs_distance /
                                     path_bounds_resolution)) {
      index = static_cast<int>(path_bounds.size()) - 1;
    } else if (index >= static_cast<int>(path_bounds.size()) +
                            static_cast<int>(check_static_obs_distance /
                                             path_bounds_resolution)) {
      return true;
    }
  }
  index =
      std::max(std::min(index, static_cast<int>(path_bounds.size()) - 1), 0);
  // constaintion of the nudge of the pedestrian on the way
  const double checkout_pedstrian_dist =
      config_.obstacle_filter_decider_config()
          .pedestrian_onway_ignore_distance();
  bool is_pedestrian_on_the_way =
      is_pedestrian ? (check_end_l < std::get<2>(path_bounds.at(index)) -
                                         checkout_pedstrian_dist &&
                       check_start_l > std::get<1>(path_bounds.at(index)) +
                                           checkout_pedstrian_dist)
                    : false;
  is_pedestrian_on_the_way = false;
#ifdef FOR_BAIDU_SIMULATION
  is_pedestrian_on_the_way = false;
#endif
  return check_start_l >
             std::get<2>(path_bounds.at(index)) + kDefaultLaneWidth ||
         check_end_l < std::get<1>(path_bounds.at(index)) - kDefaultLaneWidth ||
         is_pedestrian_on_the_way;
}

bool ObstacleFilterDecider::CheckPointIsOutBounds(
    const bool is_pedestrian, const PathBound& path_bounds,
    const double check_s, const double check_l,
    const double path_bounds_resolution) const {
  if (IsFloatEqual(path_bounds_resolution, 0.0) || path_bounds.size() < 2) {
    return false;
  }
  const double check_dynamic_obs_distance =
      config_.obstacle_filter_decider_config().check_dynamic_obs_distance();
  int index = static_cast<int>((check_s - std::get<0>(path_bounds.front())) /
                               path_bounds_resolution);
  if (frame_->GetMachineStateType() !=
      functionmanager::MachineStateType::HISTORY_TRACE_TYPE) {
    if (index < 0 && index > -static_cast<int>(check_dynamic_obs_distance /
                                               path_bounds_resolution)) {
      index = 0;
    } else if (index <= -static_cast<int>(check_dynamic_obs_distance /
                                          path_bounds_resolution)) {
      return true;
    }
  } else {
    if (index >= static_cast<int>(path_bounds.size()) &&
        index < static_cast<int>(path_bounds.size()) +
                    static_cast<int>(check_dynamic_obs_distance /
                                     path_bounds_resolution)) {
      index = static_cast<int>(path_bounds.size()) - 1;
    } else if (index >= static_cast<int>(path_bounds.size()) +
                            static_cast<int>(check_dynamic_obs_distance /
                                             path_bounds_resolution)) {
      return true;
    }
  }
  index =
      std::max(std::min(index, static_cast<int>(path_bounds.size()) - 1), 0);
  // constaintion of the nudge of the pedestrian on the way
  const double checkout_pedstrian_dist =
      config_.obstacle_filter_decider_config()
          .pedestrian_onway_ignore_distance();
  bool is_pedestrian_on_the_way =
      is_pedestrian ? (check_l < std::get<2>(path_bounds.at(index)) -
                                     checkout_pedstrian_dist &&
                       check_l > std::get<1>(path_bounds.at(index)) +
                                     checkout_pedstrian_dist)
                    : false;
  is_pedestrian_on_the_way = false;
#ifdef FOR_BAIDU_SIMULATION
  is_pedestrian_on_the_way = false;
#endif
  return check_l > std::get<2>(path_bounds.at(index)) ||
         check_l < std::get<1>(path_bounds.at(index)) ||
         is_pedestrian_on_the_way;
}

void ObstacleFilterDecider::FilterStaticObstacle(
    const double path_bounds_resolution, const PathBound& path_bounds,
    const Obstacle& obstacle,
    ReferenceLineInfo* const reference_line_info) const {
  if (reference_line_info == nullptr ||
      reference_line_info->path_decision() == nullptr) {
    return;
  }

  const std::string& obstacle_id = obstacle.Id();
  const double obstacle_start_s = obstacle.PerceptionSLBoundary().start_s();
  const double obstacle_end_s = obstacle.PerceptionSLBoundary().end_s();
  const double obstacle_start_l = obstacle.PerceptionSLBoundary().start_l();
  const double obstacle_end_l = obstacle.PerceptionSLBoundary().end_l();
  const double obstacle_middle_s = 0.5 * (obstacle_start_s + obstacle_end_s);
  auto* path_decision = reference_line_info->path_decision();
  const double consider_adc_back_distance =
      config_.obstacle_filter_decider_config().consider_adc_back_distance();
  const double consider_bound_front_distance =
      config_.obstacle_filter_decider_config().consider_bound_front_distance();
  bool is_ignore_position = false;
  if (frame_->GetMachineStateType() !=
      functionmanager::MachineStateType::HISTORY_TRACE_TYPE) {
    // 前进状态
    if (obstacle_end_s + consider_adc_back_distance <
            reference_line_info->AdcSlBoundary().start_s() ||
        obstacle_start_s >
            std::get<0>(path_bounds.back()) + consider_bound_front_distance) {
      is_ignore_position = true;
    }
  } else {
    // 倒车状态
    if (obstacle_start_s - consider_adc_back_distance >
            reference_line_info->AdcSlBoundary().end_s() ||
        obstacle_end_s <
            std::get<0>(path_bounds.front()) - consider_bound_front_distance) {
      is_ignore_position = true;
    }
  }

  if (obstacle_start_s > reference_line_info->reference_line().Length() ||
      obstacle_end_s < 0.0) {
    is_ignore_position = true;
  }

  ObjectDecisionType object_decision;
  object_decision.mutable_ignore();
  const bool is_ped = obstacle.Perception().type() ==
                      TL::perception::PerceptionObstacle::PEDESTRIAN;
  if (CheckObstacleIsBlockedBounds(path_bounds, obstacle,
                                   path_bounds_resolution) ||
      is_ignore_position ||
      (CheckEdgeIsOutBounds(is_ped, path_bounds, obstacle_start_s,
                            obstacle_start_l, obstacle_end_l,
                            path_bounds_resolution) &&
       CheckEdgeIsOutBounds(is_ped, path_bounds, obstacle_end_s,
                            obstacle_start_l, obstacle_end_l,
                            path_bounds_resolution) &&
       CheckEdgeIsOutBounds(is_ped, path_bounds, obstacle_middle_s,
                            obstacle_start_l, obstacle_end_l,
                            path_bounds_resolution))) {
    path_decision->AddLateralDecision(
        "ObstacleFilterDecider/not-in-guide-line-bounds", obstacle_id,
        object_decision);
    ADEBUG << "static obstacle_id: " << obstacle_id << ", is ignore!";
  }
}

void ObstacleFilterDecider::FilterDynamicObstacle(
    const double path_bounds_resolution, const PathBound& path_bounds,
    const Obstacle& obstacle,
    ReferenceLineInfo* const reference_line_info) const {
  if (reference_line_info == nullptr ||
      reference_line_info->path_decision() == nullptr ||
      obstacle.Trajectory().trajectory_point().empty()) {
    return;
  }
  const double interval_time = config_.obstacle_filter_decider_config()
                                   .dynamic_obstacle_check_interval_time();
  const int point_count =
      static_cast<int>(config_.obstacle_filter_decider_config()
                           .dynamic_obstacle_check_point_count());
  const double total_time =
      obstacle.Trajectory()
          .trajectory_point()
          .at(obstacle.Trajectory().trajectory_point().size() - 1)
          .relative_time();
  bool is_in_guide_line_bounds = false;
  constexpr double kEpsilon = 1e-5;
  const int step_sizes =
      static_cast<int>(std::min(total_time, interval_time * point_count) /
                       std::max(interval_time, kEpsilon));
  for (int i = 0; i < step_sizes; ++i) {
    const double check_time = i * interval_time;
    const common::TrajectoryPoint check_point =
        obstacle.GetPointAtTime(check_time);

    const common::math::Box2d bounding_box =
        obstacle.GetBoundingBox(check_point);
    const double consider_adc_back_distance =
        config_.obstacle_filter_decider_config().consider_adc_back_distance();
    const std::vector<Vec2d>& corner_points = bounding_box.GetAllCorners();
    for (const Vec2d& corner_point : corner_points) {
      common::SLPoint sl_point;
      reference_line_info->reference_line().XYToSL(corner_point, &sl_point);
      if (frame_->GetMachineStateType() !=
          functionmanager::MachineStateType::HISTORY_TRACE_TYPE) {
        if (sl_point.s() + consider_adc_back_distance <
            reference_line_info->AdcSlBoundary().start_s()) {
          continue;
        }
      } else {
        if (sl_point.s() - consider_adc_back_distance >
            reference_line_info->AdcSlBoundary().end_s()) {
          continue;
        }
      }
      if (sl_point.s() > reference_line_info->reference_line().Length() ||
          sl_point.s() < 0.0) {
        continue;
      }
      if (!CheckPointIsOutBounds(
              obstacle.Perception().type() ==
                  TL::perception::PerceptionObstacle::PEDESTRIAN,
              path_bounds, sl_point.s(), sl_point.l(),
              path_bounds_resolution)) {
        is_in_guide_line_bounds = true;
        break;
      }
    }
    if (is_in_guide_line_bounds) {
      break;
    }
  }

  const std::string& obstacle_id = obstacle.Id();
  auto* path_decision = reference_line_info->path_decision();
  ObjectDecisionType object_decision;
  object_decision.mutable_ignore();
  const double obstacle_moving_direction =
      obstacle.Trajectory().trajectory_point(0).path_point().theta();
  const auto obstacle_ref_point =
      reference_line_info->reference_line()
          .GetReferencePointForGreaterThanRefMaxS(
              obstacle.PerceptionSLBoundary().start_s());
  const double heading_difference = common::math::NormalizeAngle(
      obstacle_ref_point.heading() - obstacle_moving_direction);
  const double obstacle_speed_l =
      std::abs(obstacle.speed() * std::sin(heading_difference));

  const double check_dynamic_lateral_speed =
      config_.obstacle_filter_decider_config().check_dynamic_lateral_speed();
  if (!is_in_guide_line_bounds ||
      obstacle_speed_l > check_dynamic_lateral_speed) {
    path_decision->AddLateralDecision(
        "ObstacleFilterDecider/not-in-guide-line-bounds", obstacle_id,
        object_decision);
    ADEBUG << "dynamic obstacle_id: " << obstacle_id << ", is ignore!";
  }
}

}  // namespace planning
}  // namespace TL
