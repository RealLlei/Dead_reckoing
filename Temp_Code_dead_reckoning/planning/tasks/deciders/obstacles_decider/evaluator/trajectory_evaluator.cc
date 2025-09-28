/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  trajectory_evaluator.cc
 */

#include "planning/tasks/deciders/obstacles_decider/evaluator/trajectory_evaluator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ios>
#include <limits>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>

#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/util/util.h"
#include "planning/common/obstacle.h"
#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

using TL::common::TrajectoryPoint;

namespace {
constexpr double kAccelerateNearZero = 0.7;
constexpr double kLateralDistance = 0.5;
constexpr double kStartCalculateMinDistTime = 1.39;
constexpr double kBelievePredictionTime = 2.0;
constexpr double kSIgnoreDistance = 30.0;
constexpr double kDefaultFollowDistance = 30.0;
constexpr double kDefaultFollowTime = 5.0;
constexpr double kEpsilon = 0.0001;
constexpr double kEstimationCutin = 0.6;
}  // namespace

TrajectoryEvaluator::TrajectoryEvaluator(ObstaclesDeciderConfig config)
    : config_(std::move(config)) {}

void TrajectoryEvaluator::DebugAvoidAlongsideDecision(
    ReferenceLineInfo* reference_line_info) {
  if (reference_line_info == nullptr ||
      reference_line_info->mutable_debug() == nullptr) {
    AERROR << "debug input nullptr, failed!";
    return;
  }
  auto* avoid_alongside_debug = reference_line_info->mutable_debug()
                                    ->mutable_avoid_alongside_decider_info();
  for (const auto& debug_obstacle : debug_info_) {
    const auto& debug_obstacle_id = debug_obstacle.first;
    const auto& adc_sample_trajectory = debug_obstacle.second.first;
    const auto& trajectory_cost = debug_obstacle.second.second;
    const Obstacle* obstacle =
        reference_line_info->path_decision()->obstacles().Find(
            debug_obstacle_id);
    if (obstacle == nullptr) {
      AERROR
          << "cant debug avoid alongside decision, because "
             "reference_line_info path_decision obstacle == nullptr, failed!";
      return;
    }

    auto* decision_info = avoid_alongside_debug->mutable_decision_info()->Add();
    decision_info->set_id(debug_obstacle_id);

    auto* tags = decision_info->mutable_decision_tags();
    for (const auto& tag : obstacle->decider_tags()) {
      tags->Add()->assign(tag);
    }
    if (!adc_sample_trajectory.empty()) {
      decision_info->set_decision_a(adc_sample_trajectory.front().a());
      decision_info->set_decision_l(
          adc_sample_trajectory.back().path_point().l());
    }
    decision_info->set_parallel_time_cost(trajectory_cost.at(0));
    decision_info->set_safety_cost(trajectory_cost.at(1));
    decision_info->set_efficiency_cost(trajectory_cost.at(2));
    decision_info->set_lon_comfort_cost(trajectory_cost.at(3));
    decision_info->set_lat_offset_cost(trajectory_cost.at(4));
    decision_info->set_lat_stable_cost(trajectory_cost.at(5));
    decision_info->set_lon_stable_cost(trajectory_cost.at(6));
    decision_info->set_total_cost(trajectory_cost.at(7));
    for (const auto& trajectory_point : adc_sample_trajectory) {
      const auto adc_box = common::VehicleConfigHelper::GetBoundingBox(
          trajectory_point.path_point());
      auto* debug_adc_box = avoid_alongside_debug->add_adc_box();
      decision_info->add_adc_accelerate(trajectory_point.a());
      decision_info->add_adc_s(trajectory_point.path_point().s());
      decision_info->add_adc_l(trajectory_point.path_point().l());
      decision_info->add_adc_speed(trajectory_point.v());
      decision_info->add_adc_jerk(trajectory_point.da());
      for (const auto& point : adc_box.GetAllCorners()) {
        auto* debug_adc_point = debug_adc_box->add_point();
        debug_adc_point->set_x(point.x());
        debug_adc_point->set_y(point.y());
      }
      debug_adc_box->set_id(
          debug_obstacle_id + "_" +
          std::to_string(trajectory_point.relative_time()).substr(0, 3));

      const auto obs_trajectory_point =
          obstacle->GetPointAtTime(trajectory_point.relative_time());
      const auto obs_box = obstacle->GetBoundingBox(obs_trajectory_point);
      auto* debug_obs_box = avoid_alongside_debug->add_obs_box();
      for (const auto& point : obs_box.GetAllCorners()) {
        auto* debug_obs_point = debug_obs_box->add_point();
        debug_obs_point->set_x(point.x());
        debug_obs_point->set_y(point.y());
      }
      debug_obs_box->set_id(
          std::to_string(trajectory_point.relative_time()).substr(0, 3));
    }
  }
}

void TrajectoryEvaluator::CalculateCostMapNotIncludeInteraction(
    const std::vector<DiscretizedTrajectory>& discretized_trajectories,
    const std::unordered_set<std::string>& dangerous_obstacles,
    const ReferenceLineInfo& reference_line_info) {
  cost_map_.clear();
  ADEBUG << "discretized_trajectories size:" << discretized_trajectories.size();
  for (size_t i = 0; i < discretized_trajectories.size(); ++i) {
    const auto& trajectory = discretized_trajectories[i];
    if (trajectory.empty()) {
      continue;
    }
    bool is_collision_trajectory = false;
    std::array<double, 8> trajectory_cost =
        Evaluate(reference_line_info, trajectory, dangerous_obstacles,
                 &is_collision_trajectory);
    if (!is_collision_trajectory) {
      cost_map_[i] = trajectory_cost;
    }
  }
  ADEBUG << "cost_map_ size: " << cost_map_.size();
}

void TrajectoryEvaluator::DoDecision(const Obstacle& decision_obstacle,
                                     ReferenceLineInfo* reference_line_info) {
  // 纵向决策
  ObjectDecisionType object_decision;
  auto* path_decision = reference_line_info->path_decision();
  if (!cost_queue_.empty()) {
    if (std::fabs(cost_queue_.top().first.front().a()) <
        std::fabs(kAccelerateNearZero)) {
      // ignore不打标签
      ADEBUG << "###decision_obstacle id:" << decision_obstacle.Id()
             << " is ignore!";
    } else if (cost_queue_.top().first.front().a() > kAccelerateNearZero) {
      ADEBUG << "###decision_obstacle id:" << decision_obstacle.Id()
             << " is need overtake!";
      auto* fence_point =
          object_decision.mutable_overtake()->mutable_fence_point();
      auto* path_decision = reference_line_info->path_decision();
      fence_point->set_x(decision_obstacle.PerceptionBoundingBox().center_x());
      fence_point->set_y(decision_obstacle.PerceptionBoundingBox().center_y());
      object_decision.mutable_overtake()->set_fence_heading(
          decision_obstacle.PerceptionBoundingBox().heading());
      path_decision->AddLongitudinalDecision("obstacles_decider/-overtake",
                                             decision_obstacle.Id(),
                                             object_decision);
    } else {
      ADEBUG << "###decision_obstacle id:" << decision_obstacle.Id()
             << " is need yield!";
      auto* fence_point =
          object_decision.mutable_yield()->mutable_fence_point();
      auto* path_decision = reference_line_info->path_decision();
      fence_point->set_x(decision_obstacle.PerceptionBoundingBox().center_x());
      fence_point->set_y(decision_obstacle.PerceptionBoundingBox().center_y());
      object_decision.mutable_yield()->set_fence_heading(
          decision_obstacle.PerceptionBoundingBox().heading());
      path_decision->AddLongitudinalDecision(
          "obstacles_decider/-yield", decision_obstacle.Id(), object_decision);
    }

    // 横向决策
    if (std::fabs(cost_queue_.top().first.back().path_point().l()) <
        std::fabs(kLateralDistance)) {
    } else if (cost_queue_.top().first.back().path_point().l() >
               kLateralDistance) {
      ADEBUG << "decision_obstacle id:" << decision_obstacle.Id()
             << " is need left nudge!";
      ObjectNudge* object_nudge_ptr = object_decision.mutable_nudge();
      object_nudge_ptr->set_distance_l(0.1);
      if (decision_obstacle.IsStatic()) {
        object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
        path_decision->AddLateralDecision("obstacles_decider/static-left-nudge",
                                          decision_obstacle.Id(),
                                          object_decision);
      } else {
        object_nudge_ptr->set_type(ObjectNudge::DYNAMIC_LEFT_NUDGE);
        path_decision->AddLateralDecision(
            "obstacles_decider/dynamic-left-nudge", decision_obstacle.Id(),
            object_decision);
      }

    } else {
      ADEBUG << "decision_obstacle id:" << decision_obstacle.Id()
             << " is need right nudge!";
      ObjectNudge* object_nudge_ptr = object_decision.mutable_nudge();
      object_nudge_ptr->set_distance_l(0.1);
      if (decision_obstacle.IsStatic()) {
        object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
        path_decision->AddLateralDecision(
            "obstacles_decider/static-right-nudge", decision_obstacle.Id(),
            object_decision);
      } else {
        object_nudge_ptr->set_type(ObjectNudge::DYNAMIC_RIGHT_NUDGE);
        path_decision->AddLateralDecision(
            "obstacles_decider/dynamic-right-nudge", decision_obstacle.Id(),
            object_decision);
      }
    }
  }

  // 存储决策信息
  if (!cost_queue_.empty()) {
    last_frame_decision_trajectory_[decision_obstacle.Id()].first =
        decision_obstacle.PerceptionId();
    last_frame_decision_trajectory_[decision_obstacle.Id()].second =
        cost_queue_.top().first;
  }

  while (!cost_queue_.empty()) {
    if (cost_queue_.top().first.empty()) {
      ADEBUG << "****debug cost queue! trajectory is empty!";
    } else {
      ADEBUG << "###obs id:" << decision_obstacle.Id()
             << ", a:" << cost_queue_.top().first.front().a()
             << ", l:" << cost_queue_.top().first.back().path_point().l()
             << ", total:" << cost_queue_.top().second.back()
             << ", parallel:" << cost_queue_.top().second.at(0)
             << ", safety:" << cost_queue_.top().second.at(1)
             << ", efficiency:" << cost_queue_.top().second.at(2)
             << ", lon_comfort:" << cost_queue_.top().second.at(3)
             << ", lat_offset:" << cost_queue_.top().second.at(4)
             << ", lat_stable:" << cost_queue_.top().second.at(5)
             << ", lon_stable:" << cost_queue_.top().second.at(6);
    }
    cost_queue_.pop();
  }
}

double TrajectoryEvaluator::CalculateVirtualWallS(
    const std::string& nearest_cutin_obs_id, const std::string& follow_obs_id,
    const ReferenceLineInfo& reference_line_info) {
  double virtual_wall_s = std::numeric_limits<double>::max();
  const auto* follow_obs =
      reference_line_info.path_decision().obstacles().Find(follow_obs_id);
  const auto* nearest_cutin_obs =
      reference_line_info.path_decision().obstacles().Find(
          nearest_cutin_obs_id);
  if (follow_obs != nullptr) {
    virtual_wall_s =
        std::fmin(virtual_wall_s, follow_obs->PerceptionSLBoundary().start_s());
  }
  if (nearest_cutin_obs != nullptr) {
    virtual_wall_s = std::fmin(
        virtual_wall_s, nearest_cutin_obs->PerceptionSLBoundary().start_s());
  }
  return virtual_wall_s;
}

bool TrajectoryEvaluator::PruningStrategy(
    const DiscretizedTrajectory& discretized_trajectory,
    const Obstacle& decision_obstacle, const double& virtual_wall_s,
    const ReferenceLineInfo& reference_line_info) {
  if (discretized_trajectory.empty()) {
    return true;
  }
  const double obs_start_l = decision_obstacle.PerceptionSLBoundary().start_l();
  const double obs_end_l = decision_obstacle.PerceptionSLBoundary().end_l();
  const double obs_end_s = decision_obstacle.PerceptionSLBoundary().end_s();
  const bool is_left_obs = obs_start_l + obs_end_l > 0.0;
  const bool is_left_trajectory =
      discretized_trajectory.back().path_point().l() > kLateralDistance;
  const bool is_right_trajectory =
      discretized_trajectory.back().path_point().l() < -kLateralDistance;
  const bool is_decelerate_trajectory =
      discretized_trajectory.front().a() < -kAccelerateNearZero;
  const bool is_accelerate_trajectory =
      discretized_trajectory.front().a() > kAccelerateNearZero;
  const bool is_near_virtual_wall =
      virtual_wall_s - reference_line_info.AdcSlBoundary().end_s() <
      kDefaultFollowDistance +
          kDefaultFollowTime *
              reference_line_info.vehicle_state().linear_velocity();
  const double Adc_follow_time =
      (virtual_wall_s - reference_line_info.AdcSlBoundary().end_s()) /
      std::fmax(reference_line_info.vehicle_state().linear_velocity(),
                kEpsilon);
  constexpr double kMaxFollowTime = 2.0;
  constexpr double kMinFollowTime = 1.0;
  // 剪枝策略 跟车时距较大时、砍掉减速解，跟车时距较小时、砍掉加速解
  if ((Adc_follow_time > kMaxFollowTime && is_decelerate_trajectory) ||
      (Adc_follow_time < kMinFollowTime && is_accelerate_trajectory)) {
    return true;
  }
  // 剪枝策略 自车车速快、砍掉减速解
  constexpr double kSpeedError = 2.0;
  if (reference_line_info.vehicle_state().linear_velocity() -
              decision_obstacle.speed() >
          kSpeedError &&
      is_decelerate_trajectory) {
    return true;
  }
  // 剪枝策略 后方车、砍掉减速解
  const bool is_behind_obs =
      reference_line_info.AdcSlBoundary().end_s() > obs_end_s;
  if (is_decelerate_trajectory && is_behind_obs) {
    return true;
  }
  constexpr double kLowSpeed = 10.0;
  const bool is_low_speed =
      reference_line_info.vehicle_state().linear_velocity() < kLowSpeed;
  // 剪枝策略 横向避障、砍掉减速解
  if ((is_left_trajectory || is_right_trajectory) && is_decelerate_trajectory &&
      is_low_speed) {
    return true;
  }
  // 剪枝策略 砍掉向决策障碍物横向靠近的解
  if ((is_left_obs && is_left_trajectory) ||
      (!is_left_obs && is_right_trajectory)) {
    return true;
  }
  // 剪枝策略 砍掉纵向距离小的加速解
  if (is_near_virtual_wall && is_accelerate_trajectory) {
    return true;
  }
  // 剪枝策略 切入意图大的障碍物砍去加速解
  if ((cutin_estimation_infos_.find(decision_obstacle.PerceptionId()) !=
           cutin_estimation_infos_.end() &&
       cutin_estimation_infos_[decision_obstacle.PerceptionId()] >
           kEstimationCutin) &&
      is_accelerate_trajectory) {
    return true;
  }

  // 剪枝策略 压线小车砍去加速解
  if (!decision_obstacle.IsOversizedVehicle() && is_accelerate_trajectory) {
    return true;
  }

  // 剪枝策略 砍去超速解
  if (discretized_trajectory.back().v() >
          reference_line_info.GetCruiseSpeed() &&
      is_accelerate_trajectory) {
    ADEBUG << "sample back v: " << discretized_trajectory.back().v()
           << ", cruise speed: " << reference_line_info.GetCruiseSpeed()
           << ", sample a: " << discretized_trajectory.front().a();
    return true;
  }

  // 剪枝策略 主车和障碍物纵向有重叠，砍去减速解
  const auto obs_start_s = decision_obstacle.PerceptionSLBoundary().start_s();
  const auto ego_end_s = reference_line_info.AdcSlBoundary().end_s();
  const auto ego_start_s = reference_line_info.AdcSlBoundary().end_s();
  return obs_start_s < ego_end_s && obs_end_s > ego_start_s &&
         is_decelerate_trajectory;
}

void TrajectoryEvaluator::CalculateInteractionCost(
    const std::vector<DiscretizedTrajectory>& discretized_trajectories,
    const Obstacle& decision_obstacle, const std::string& nearest_cutin_obs_id,
    const std::string& follow_obs_id,
    const ReferenceLineInfo& reference_line_info) {
  if (discretized_trajectories.empty()) {
    return;
  }
  ADEBUG << "avoid alongside discretized_trajectories size:"
         << discretized_trajectories.size();
  double virtual_wall_s = CalculateVirtualWallS(
      nearest_cutin_obs_id, follow_obs_id, reference_line_info);
  for (size_t i = 0; i < discretized_trajectories.size(); ++i) {
    if (cost_map_.find(i) == cost_map_.end()) {
      continue;
    }
    if (discretized_trajectories[i].empty()) {
      continue;
    }
    if (PruningStrategy(discretized_trajectories[i], decision_obstacle,
                        virtual_wall_s, reference_line_info)) {
      continue;
    }

    cost_map_[i].at(0) = ParallelTimeCost(
        CalculateParallelTime(reference_line_info, discretized_trajectories[i],
                              decision_obstacle.Id()));
    cost_map_[i].at(5) = LateralStableCost(discretized_trajectories[i],
                                           decision_obstacle.PerceptionId());
    cost_map_[i].at(6) = LongitudinalStableCost(
        discretized_trajectories[i], decision_obstacle.PerceptionId());
    cost_map_[i].at(7) = cost_map_[i].at(0) + cost_map_[i].at(1) +
                         cost_map_[i].at(2) + cost_map_[i].at(3) +
                         cost_map_[i].at(4) + cost_map_[i].at(5) +
                         cost_map_[i].at(6);
    cost_queue_.emplace(discretized_trajectories[i], cost_map_[i]);
  }
}

void TrajectoryEvaluator::SaveDebugInfo(const std::string& debug_obs_id) {
  if (config_.enable_debug_avoid_alongside_decision() && !cost_queue_.empty()) {
    debug_info_[debug_obs_id] = cost_queue_.top();
  }
}

void TrajectoryEvaluator::ClearCache(
    const ReferenceLineInfo& reference_line_info) {
  debug_info_.clear();
  std::unordered_set<std::string> disappear_obs;
  for (const auto& last_obs_decision : last_frame_decision_trajectory_) {
    const auto& last_obs_perception_id = last_obs_decision.second.first;
    bool is_real_disappear = true;
    for (const auto* obstacle :
         reference_line_info.path_decision().obstacles().Items()) {
      if (obstacle == nullptr) {
        continue;
      }
      if (obstacle->PerceptionId() == last_obs_perception_id) {
        is_real_disappear = false;
        break;
      }
    }
    if (is_real_disappear) {
      disappear_obs.emplace(last_obs_decision.first);
    }
  }
  for (const auto& obs_id : disappear_obs) {
    last_frame_decision_trajectory_.erase(obs_id);
  }
}

void TrajectoryEvaluator::MakeObstacleDecision(
    const std::vector<DiscretizedTrajectory>& discretized_trajectories,
    const std::unordered_set<std::string>& decision_obstacles,
    const std::unordered_set<std::string>& dangerous_obstacles,
    const std::string& nearest_cutin_obs_id, const std::string& follow_obs_id,
    const std::unordered_map<int, double>& cutin_estimation_infos,
    ReferenceLineInfo* reference_line_info) {
  if (reference_line_info == nullptr ||
      reference_line_info->path_decision() == nullptr) {
    return;
  }
  cutin_estimation_infos_ = cutin_estimation_infos;
  auto* path_decision = reference_line_info->path_decision();
  ClearCache(*reference_line_info);
  CalculateCostMapNotIncludeInteraction(
      discretized_trajectories, dangerous_obstacles, *reference_line_info);

  for (const Obstacle* decision_obstacle : path_decision->obstacles().Items()) {
    if (decision_obstacle == nullptr ||
        decision_obstacles.find(decision_obstacle->Id()) ==
            decision_obstacles.end()) {
      continue;
    }

    CalculateInteractionCost(discretized_trajectories, *decision_obstacle,
                             nearest_cutin_obs_id, follow_obs_id,
                             *reference_line_info);
    SaveDebugInfo(decision_obstacle->Id());
    DoDecision(*decision_obstacle, reference_line_info);
  }
}

std::array<double, 8> TrajectoryEvaluator::Evaluate(
    const ReferenceLineInfo& reference_line_info,
    const DiscretizedTrajectory& trajectory,
    const std::unordered_set<std::string>& dangerous_obstacles,
    bool* const is_collision_trajectory) const {
  std::array<double, 8> cost_info = {};
  for (auto& cost : cost_info) {
    cost = 0.0;
  }
  if (is_collision_trajectory == nullptr) {
    return cost_info;
  }
  *is_collision_trajectory = false;
  double obs_adc_min_distance = std::numeric_limits<double>::max();
  *is_collision_trajectory =
      CalculateObsAdcMinDistance(reference_line_info, trajectory,
                                 dangerous_obstacles, &obs_adc_min_distance);

  cost_info.at(1) = SafetyCost(obs_adc_min_distance);

  // Longitudinal costs
  cost_info.at(2) = EfficiencyCost(
      trajectory, reference_line_info.planning_target().cruise_speed());
  cost_info.at(3) = LonComfortCost(trajectory);

  // Lateral costs
  cost_info.at(4) = LatOffsetCost(trajectory);

  return cost_info;
}

double TrajectoryEvaluator::ParallelTimeCost(double parallel_time) const {
  const auto& weight_param = config_.cost_weight_shape_param();
  return weight_param.parallel_time_cost_k() *
         (common::math::Sigmoid(weight_param.parallel_time_cost_a() *
                                    std::fabs(parallel_time) +
                                weight_param.parallel_time_cost_b()) -
          common::math::Sigmoid(weight_param.parallel_time_cost_b()));
}

double TrajectoryEvaluator::SafetyCost(double adc_obs_min_distance) const {
  const auto& weight_param = config_.cost_weight_shape_param();
  ADEBUG << "adc_obs_min_distance:" << adc_obs_min_distance;
  return weight_param.safety_cost_k() *
         (common::math::Sigmoid(weight_param.safety_cost_a() *
                                    std::fabs(adc_obs_min_distance) +
                                weight_param.safety_cost_b()));
}

double TrajectoryEvaluator::CalculateParallelTime(
    const ReferenceLineInfo& reference_line_info,
    const DiscretizedTrajectory& trajectory,
    const std::string& decision_obstacle_id) {
  bool has_s_overlap_start = false;
  bool has_s_overlap_end = false;
  double start_s_overlap_time = 0.0;
  double end_s_overlap_time = 0.0;

  const auto* const obstacle =
      reference_line_info.path_decision().obstacles().Find(
          decision_obstacle_id);
  if (obstacle == nullptr) {
    return 0.0;
  }
  for (const auto& adc_trajectory_point : trajectory) {
    const double adc_start_s = adc_trajectory_point.path_point().s() -
                               common::VehicleConfigHelper::GetConfig()
                                   .vehicle_param()
                                   .back_edge_to_center();
    const double adc_end_s = adc_trajectory_point.path_point().s() +
                             common::VehicleConfigHelper::GetConfig()
                                 .vehicle_param()
                                 .back_edge_to_center();
    const double relative_time =
        std::fmax(adc_trajectory_point.relative_time(), 0.0);
    const TrajectoryPoint obs_trajectory_point =
        obstacle->GetPointAtTime(relative_time);

    const double obstacle_start_s = obs_trajectory_point.path_point().s() +
                                    obstacle->PerceptionSLBoundary().start_s();
    const double obstacle_end_s = obs_trajectory_point.path_point().s() +
                                  obstacle->PerceptionSLBoundary().end_s();

    const bool current_is_s_overlap =
        obstacle_start_s < adc_end_s && obstacle_end_s > adc_start_s;
    ADEBUG << "current_is_s_overlap: " << current_is_s_overlap
           << ", adc_start_s: " << adc_start_s << ", adc_end_s: " << adc_end_s
           << ", obstacle_start_s: " << obstacle_start_s
           << ", obstacle_end_s: " << obstacle_end_s;

    if (current_is_s_overlap && !has_s_overlap_start) {
      has_s_overlap_start = true;
      start_s_overlap_time = relative_time;
    }

    if (!current_is_s_overlap && has_s_overlap_start && !has_s_overlap_end) {
      has_s_overlap_end = true;
      end_s_overlap_time = relative_time;
    }
  }

  if (has_s_overlap_start && !has_s_overlap_end &&
      !obstacle->GetTrajectoryEnvelope().empty()) {
    end_s_overlap_time = obstacle->GetTrajectoryEnvelope().back().time;
  }

  if (!trajectory.empty()) {
    ADEBUG << "decision_obstacle_id:" << decision_obstacle_id
           << ", sample acc:" << trajectory.front().a()
           << ", overlap time:" << end_s_overlap_time - start_s_overlap_time
           << ", has_s_overlap_start: " << has_s_overlap_start
           << ", has_s_overlap_end:" << has_s_overlap_end
           << ", start_s_overlap_time:" << start_s_overlap_time
           << ", end_s_overlap_time:" << end_s_overlap_time;
  }

  return end_s_overlap_time - start_s_overlap_time;
}

bool TrajectoryEvaluator::CalculateObsAdcSlDistance(
    const SLBoundary& adc_sl_boundary, const SLBoundary& obs_sl_boundary,
    double* const s_distance, double* const l_distance) {
  // 输入有效性检查
  if (s_distance == nullptr || l_distance == nullptr) {
    return false;
  }
  const bool is_front_obstacle =
      obs_sl_boundary.start_s() + obs_sl_boundary.end_s() >
      adc_sl_boundary.start_s() + adc_sl_boundary.end_s();
  *s_distance =
      is_front_obstacle
          ? std::fmax(obs_sl_boundary.start_s() - adc_sl_boundary.end_s(), 0.0)
          : std::fmax(adc_sl_boundary.start_s() - obs_sl_boundary.end_s(), 0.0);
  const bool is_left_obstacle =
      obs_sl_boundary.start_l() + obs_sl_boundary.end_l() >
      adc_sl_boundary.start_l() + adc_sl_boundary.end_l();
  *l_distance =
      is_left_obstacle
          ? std::fmax(obs_sl_boundary.start_l() - adc_sl_boundary.end_l(), 0.0)
          : std::fmax(adc_sl_boundary.start_l() - obs_sl_boundary.end_l(), 0.0);
  return true;
}

bool TrajectoryEvaluator::CalculateObsAdcSlBoundary(
    const TrajectoryPoint& adc_trajectory_point,
    const std::vector<ObsPointDescription>& trajectory_envelopes,
    SLBoundary* const adc_sl_boundary, SLBoundary* const obs_sl_boundary) {
  // 输入校验
  if (adc_sl_boundary == nullptr || obs_sl_boundary == nullptr ||
      trajectory_envelopes.empty()) {
    return false;
  }
  // 计算adc sl boundary
  adc_sl_boundary->set_start_s(adc_trajectory_point.path_point().s() -
                               common::VehicleConfigHelper::GetConfig()
                                   .vehicle_param()
                                   .back_edge_to_center());
  adc_sl_boundary->set_end_s(adc_trajectory_point.path_point().s() +
                             common::VehicleConfigHelper::GetConfig()
                                 .vehicle_param()
                                 .front_edge_to_center());
  adc_sl_boundary->set_start_l(
      adc_trajectory_point.path_point().l() -
      0.5 * common::VehicleConfigHelper::GetConfig().vehicle_param().width());
  adc_sl_boundary->set_end_l(
      adc_trajectory_point.path_point().l() +
      0.5 * common::VehicleConfigHelper::GetConfig().vehicle_param().width());
  // 计算同一时刻的obs sl boundary
  const auto iter =
      std::lower_bound(trajectory_envelopes.begin(), trajectory_envelopes.end(),
                       adc_trajectory_point.relative_time(),
                       [](const ObsPointDescription& obs_point_description,
                          const double check_time) {
                         return obs_point_description.time < check_time;
                       });
  const auto trajectory_envelope =
      iter == trajectory_envelopes.end() ? trajectory_envelopes.back() : *iter;

  obs_sl_boundary->set_start_l(std::fmin(trajectory_envelope.low_left_p.l(),
                                         trajectory_envelope.low_right_p.l()));
  obs_sl_boundary->set_end_l(std::fmax(trajectory_envelope.upper_left_p.l(),
                                       trajectory_envelope.upper_right_p.l()));
  obs_sl_boundary->set_end_s(std::fmax(trajectory_envelope.upper_right_p.s(),
                                       trajectory_envelope.low_right_p.s()));
  obs_sl_boundary->set_start_s(std::fmin(trajectory_envelope.upper_left_p.s(),
                                         trajectory_envelope.low_left_p.s()));
  return true;
}

bool TrajectoryEvaluator::CalculateDistanceFilter(
    const ReferenceLineInfo& reference_line_info,
    const SLBoundary& adc_sl_boundary, const SLBoundary& obs_sl_boundary,
    const double relative_time, const double trajectory_end_s,
    const double s_distance) {
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!reference_line_info.reference_line().GetLaneWidth(
          0.5 * (obs_sl_boundary.start_s() + obs_sl_boundary.end_s()),
          &lane_left_width, &lane_right_width)) {
    return true;
  }

  const bool is_left_obs =
      (obs_sl_boundary.start_l() + obs_sl_boundary.end_l()) > 0.0;
  // 过滤策略1 左侧box压线程度大的box不计算
  constexpr double kOverLapDis = 0.2;
  if (is_left_obs &&
      obs_sl_boundary.start_l() < lane_left_width - kOverLapDis) {
    return true;
  }
  // 过滤策略2 右侧box压线程度大的box不计算
  if (!is_left_obs &&
      obs_sl_boundary.end_l() > -lane_right_width + kOverLapDis) {
    return true;
  }
  // 过滤策略3 自车后、采样轨迹前的box不计算
  if (adc_sl_boundary.start_s() > obs_sl_boundary.end_s() ||
      obs_sl_boundary.start_s() > trajectory_end_s) {
    return true;
  }
  // 过滤策略5 自车初始采样的一段时间的box不计算、超过预测置信时间的box不计算
  if (relative_time < kStartCalculateMinDistTime ||
      relative_time > kBelievePredictionTime) {
    return true;
  }
  // 过滤策略4 障碍物box和自车box距离超过30m不计算
  return s_distance > kSIgnoreDistance +
                          reference_line_info.vehicle_state().linear_velocity();
}

bool TrajectoryEvaluator::CalculateObsAdcMinDistance(
    const ReferenceLineInfo& reference_line_info,
    const DiscretizedTrajectory& trajectory,
    const std::unordered_set<std::string>& dangerous_obstacles,
    double* const obs_adc_min_distance) {
  // 输入有效性检查
  if (obs_adc_min_distance == nullptr) {
    AERROR << "InteractionCost input error, failed!";
    return true;
  }

  // 输出初始化
  *obs_adc_min_distance = std::numeric_limits<double>::max();
  if (trajectory.empty()) {
    return true;
  }
  const double trajectory_end_s = trajectory.back().path_point().s();
  // 遍历障碍物
  std::string nearest_obs_id;
  double nearest_time = -1.0;
  double nearest_distance = -1.0;
  for (const Obstacle* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    // 只处理真实的危险障碍物
    if (obstacle == nullptr || obstacle->IsVirtual() ||
        dangerous_obstacles.find(obstacle->Id()) == dangerous_obstacles.end()) {
      continue;
    }
    const auto& trajectory_envelopes = obstacle->GetTrajectoryEnvelope();
    if (trajectory_envelopes.empty()) {
      continue;
    }
    // 对adc轨迹点进行遍历计算adc和obs的最近l距离
    for (const auto& adc_trajectory_point : trajectory) {
      SLBoundary adc_sl_boundary;
      SLBoundary obs_sl_boundary;
      if (!CalculateObsAdcSlBoundary(adc_trajectory_point, trajectory_envelopes,
                                     &adc_sl_boundary, &obs_sl_boundary)) {
        continue;
      }
      double s_distance = 0.0;
      double l_distance = 0.0;
      if (!CalculateObsAdcSlDistance(adc_sl_boundary, obs_sl_boundary,
                                     &s_distance, &l_distance)) {
        continue;
      }
      // 碰撞检查
      if (common::util::IsFloatEqual(l_distance, 0.0) &&
          common::util::IsFloatEqual(s_distance, 0.0)) {
        *obs_adc_min_distance = 0.0;
        ADEBUG << "obs id: {" << obstacle->Id() << "} is collision!"
               << " sample a: {" << trajectory.front().a() << "}, sample l: {"
               << trajectory.back().path_point().l() << "}";
        return true;
      }
      // 过滤策略
      if (CalculateDistanceFilter(reference_line_info, adc_sl_boundary,
                                  obs_sl_boundary,
                                  adc_trajectory_point.relative_time(),
                                  trajectory_end_s, s_distance)) {
        continue;
      }
      if (l_distance < *obs_adc_min_distance) {
        *obs_adc_min_distance = l_distance;
        nearest_distance = *obs_adc_min_distance;
        nearest_obs_id = obstacle->Id();
        nearest_time = adc_trajectory_point.relative_time();
      }
    }
  }

  ADEBUG << "sample a:" << trajectory.front().a()
         << ", sample l:" << trajectory.back().path_point().l()
         << ", near_obs_id:" << nearest_obs_id
         << ", nearest_time:" << nearest_time
         << ", nearest_distance:" << nearest_distance;

  return false;
}

double TrajectoryEvaluator::LatOffsetCost(
    const DiscretizedTrajectory& trajectory) const {
  if (trajectory.empty()) {
    return 0.0;
  }
  const double lat_offset = std::fabs(trajectory.back().path_point().l());
  const auto& weight_param = config_.cost_weight_shape_param();
  return weight_param.lat_offset_cost_k() *
         (common::math::Sigmoid(weight_param.lat_offset_cost_a() * lat_offset +
                                weight_param.lat_offset_cost_b()) -
          common::math::Sigmoid(weight_param.lat_offset_cost_b()));
}

double TrajectoryEvaluator::LonComfortCost(
    const DiscretizedTrajectory& trajectory) const {
  if (trajectory.empty()) {
    return 0.0;
  }
  const double max_accel = std::fabs(trajectory.front().a());
  const double max_jerk = std::fabs(trajectory.front().da());
  const auto& weight_param = config_.cost_weight_shape_param();
  return weight_param.lon_acc_comfort_cost_k() *
             (common::math::Sigmoid(weight_param.lon_acc_comfort_cost_a() *
                                        max_accel +
                                    weight_param.lon_acc_comfort_cost_b()) -
              common::math::Sigmoid(weight_param.lon_acc_comfort_cost_b())) +
         weight_param.lon_jerk_comfort_cost_k() *
             (common::math::Sigmoid(weight_param.lon_jerk_comfort_cost_a() *
                                        max_jerk +
                                    weight_param.lon_jerk_comfort_cost_b()) -
              common::math::Sigmoid(weight_param.lon_jerk_comfort_cost_b()));
}

double TrajectoryEvaluator::EfficiencyCost(
    const DiscretizedTrajectory& trajectory, double ref_speed) const {
  if (trajectory.empty()) {
    return 0.0;
  }
  const double speed_max_error =
      std::fmax(ref_speed - trajectory.back().v(), 0.0);
  const auto& weight_param = config_.cost_weight_shape_param();
  return weight_param.efficiency_cost_k() *
         (common::math::Sigmoid(weight_param.efficiency_cost_a() *
                                    std::fabs(speed_max_error) +
                                weight_param.efficiency_cost_b()) -
          common::math::Sigmoid(weight_param.efficiency_cost_b()));
}

double TrajectoryEvaluator::LateralStableCost(
    const DiscretizedTrajectory& trajectory, const int32_t& perception_obs_id) {
  if (trajectory.empty()) {
    return 0.0;
  }
  double l_decision_error = 0.0;
  for (const auto& last_frame_info : last_frame_decision_trajectory_) {
    const int32_t& last_obs_perception_id = last_frame_info.second.first;
    const auto& last_trajectory = last_frame_info.second.second;
    if (last_trajectory.empty()) {
      continue;
    }
    if (perception_obs_id == last_obs_perception_id) {
      const double stable_coff =
          std::fabs(last_trajectory.back().path_point().l()) < kLateralDistance
              ? 0.5
              : 1.0;
      l_decision_error =
          stable_coff *
          std::fmax(l_decision_error,
                    std::fabs(trajectory.back().path_point().l() -
                              last_trajectory.back().path_point().l()));
    }
  }
  const auto& weight_param = config_.cost_weight_shape_param();
  return weight_param.lateral_stable_cost_k() *
         (common::math::Sigmoid(weight_param.lateral_stable_cost_a() *
                                    l_decision_error +
                                weight_param.lateral_stable_cost_b()) -
          common::math::Sigmoid(weight_param.lateral_stable_cost_b()));
}

double TrajectoryEvaluator::LongitudinalStableCost(
    const DiscretizedTrajectory& trajectory, const int32_t& perception_obs_id) {
  if (trajectory.empty()) {
    return 0.0;
  }
  double accelerate_decision_error = 0.0;
  for (const auto& last_frame_info : last_frame_decision_trajectory_) {
    const int32_t& last_obs_perception_id = last_frame_info.second.first;
    const auto& last_trajectory = last_frame_info.second.second;
    if (last_trajectory.empty()) {
      continue;
    }
    if (last_obs_perception_id == perception_obs_id) {
      if (std::fabs(last_trajectory.front().a()) < kAccelerateNearZero) {
        continue;
      }
      accelerate_decision_error = std::fmax(
          std::fabs(trajectory.front().a() - last_trajectory.front().a()),
          accelerate_decision_error);
    }
  }

  const auto& weight_param = config_.cost_weight_shape_param();
  return weight_param.longitudinal_stable_cost_k() *
         (common::math::Sigmoid(weight_param.longitudinal_stable_cost_a() *
                                    accelerate_decision_error +
                                weight_param.longitudinal_stable_cost_b()) -
          common::math::Sigmoid(weight_param.longitudinal_stable_cost_b()));
}

}  // namespace planning
}  // namespace TL
