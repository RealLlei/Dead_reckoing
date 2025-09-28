/*
 * Copyright (c) 2022 TL Technologies Co., Ltd. All rights reserved.
 */

#include "planning/tasks/deciders/obstacles_decider/obstacles_decider.h"

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "common/math/cartesian_frenet_conversion.h"
#include "common/status/status.h"
#include "common/util/util.h"
#include "planning/common/frame.h"
#include "planning/common/indexed_list.h"
#include "planning/common/obstacle.h"
#include "planning/common/obstacle_blocking_analyzer.h"
#include "planning/common/path_decision.h"
#include "planning/common/reference_line_info.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/planning/decider_debug.pb.h"
#include "proto/planning/decision.pb.h"

namespace TL::planning {
namespace {
// constexpr double kEpsilon = 0.0000001;
// constexpr double kHalf = 0.5;
// constexpr double kAdcObsMaxSpeedDiffer = 5.0;
// constexpr double kAdcObsMaxLgtDistance = 50.0;
// constexpr double kAdcAvgDeceleration = 3.0;
// // the following parameters should be same as in obstacles_info_updater.cc
// constexpr double kAdcObsLatSafeDistance = 3.0;
constexpr double kBackObstacleIgnoreTtc = 2.5;
}  // namespace

using TL::common::Clock;
using TL::common::PathPoint;
using TL::common::Status;
using TL::common::TrajectoryPoint;
using TL::common::math::CartesianFrenetConverter;
using TL::common::math::double_type::DefinitelyGreater;  // NOLINT

ObstaclesDecider::ObstaclesDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  if (config.has_obstacles_decider_config()) {
    if (config.obstacles_decider_config().has_ignore_obs_before_cipv()) {
      ignore_obs_before_cipv_ =
          config.obstacles_decider_config().ignore_obs_before_cipv();
    }
    if (config.obstacles_decider_config().has_enable_debug_obstacles_info()) {
      enable_debug_obstacles_info_ =
          config.obstacles_decider_config().enable_debug_obstacles_info();
    }
    trajectory_generator_ = std::make_unique<TrajectoryGenerator>(
        config.obstacles_decider_config());
    trajectory_evaluator_ = std::make_unique<TrajectoryEvaluator>(
        config.obstacles_decider_config());
    obstacle_selector_ = std::make_unique<ObstacleSelector>();
    if (config_.obstacles_decider_config()
            .has_enable_debug_avoid_alongside_decision()) {
      enable_debug_avoid_alongside_decision_ =
          config_.obstacles_decider_config()
              .enable_debug_avoid_alongside_decision();
      conflict_resolution_ = std::make_unique<ConflictResolution>(
          config_.obstacles_decider_config()
              .enable_debug_avoid_alongside_decision());
    }
  }
}

common::Status ObstaclesDecider::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (frame == nullptr || reference_line_info == nullptr) {
    const std::string msg = "frame or reference_line_info is nullptr.";
    ADEBUG << msg;
    return common::Status(
        common::ErrorCode::PLANNER_CRUISING_OBSTACLESDECIDER_ERROR, msg);
  }
  if (injector_ == nullptr || injector_->planning_context() == nullptr) {
    return common::Status::OK();
  }
  const auto& path_decider =
      injector_->planning_context()->planning_status().path_decider();
  if (path_decider.has_is_in_path_lane_borrow_scenario() &&
      path_decider.is_in_path_lane_borrow_scenario()) {
    conflict_resolution_->ClearCache();
    behavior_conflict_resolution_.ClearCache();
    obs_info_.ClearCache();
    ADEBUG << "adc is in lane borrow.";
    return common::Status::OK();
  }

  const auto& change_lane =
      injector_->planning_context()->planning_status().change_lane();
  const auto& change_lane_reason = change_lane.change_lane_reason();
  const auto& lane_change_status = change_lane.status();
  if (lane_change_status == ChangeLaneStatus::IN_CHANGE_LANE ||
      change_lane_reason != functionmanager::HmiChangeLaneReason::HMI_NONE) {
    conflict_resolution_->ClearCache();
    behavior_conflict_resolution_.ClearCache();
    ADEBUG << "adc is in change lane";
    return common::Status::OK();
  }

  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_decider_debug()
      ->clear_obstacles_decider_debug();
  mutable_obstacles_decider_debug_ = injector_->planning_context()
                                         ->mutable_planning_status()
                                         ->mutable_decider_debug()
                                         ->mutable_obstacles_decider_debug();

  ADEBUG << "Frame_time=" << frame->SequenceNum();
  obs_info_.GetObsInitialBehavior(*reference_line_info);
  ObstaclesDeciderDebugInfo();
  // MakeInteractiveObsDecision(reference_line_info);

  if (ignore_obs_before_cipv_) {
    ADEBUG << "make_front_cipv_ignore_decision";
    MakeIgnoreDecision(reference_line_info->path_decision());
  }
  const auto driving_mode = injector_->vehicle_state()->driving_mode();
  if (!is_auto_mode_) {
    is_auto_mode_ = driving_mode == TL::soc::Chassis::COMPLETE_AUTO_DRIVE ||
                    driving_mode == TL::soc::Chassis::AUTO_STEER_ONLY;
    if (is_auto_mode_) {
      start_time_avoid_alongside_ = Clock::NowInSeconds();
    }
  } else {
    is_auto_mode_ = driving_mode == TL::soc::Chassis::COMPLETE_AUTO_DRIVE ||
                    driving_mode == TL::soc::Chassis::AUTO_STEER_ONLY;
  }
  constexpr double kStartTimeAvoidAlongside = 5.0;
  const bool is_time_to_start =
      (Clock::NowInSeconds() - start_time_avoid_alongside_) * 1000 >
      kStartTimeAvoidAlongside;
  frame->SetObstacleInfo(obs_info_);
  ADEBUG << "is_auto_mode_:" << is_auto_mode_
         << ", is_time_to_start: " << is_time_to_start;
  if (config_.obstacles_decider_config().enable_avoid_alongside_decision() &&
      is_auto_mode_ && is_time_to_start) {
    MakeAvoidAlongsideDecision(frame, reference_line_info);
  }

  return common::Status::OK();
}

void ObstaclesDecider::ObstaclesDeciderDebugInfo() {
  const std::string& nearest_blocking_obs_id =
      obs_info_.GetNearestBlockingObs();
  const std::string& cipv_id = obs_info_.GetCIPV();
  // const std::vector<std::string>& interactive_obs_id =
  //     obs_info_.GetInteractiveObs();

  mutable_obstacles_decider_debug_->set_nearest_follow_id(
      nearest_blocking_obs_id);
  mutable_obstacles_decider_debug_->set_cipv_id(cipv_id);
  if (enable_debug_obstacles_info_) {
    const auto& obstacle_behavior_info = obs_info_.GetObstaclesInfo();
    for (const auto& obs_info : obstacle_behavior_info) {
      ADEBUG << "obs_info.id=" << obs_info.first;
      ObstacleBehavior* mutable_obstacle_behavior_info_debug =
          mutable_obstacles_decider_debug_->add_obstacle_behavior_info();
      mutable_obstacle_behavior_info_debug->set_obs_id(obs_info.first);
      mutable_obstacle_behavior_info_debug->set_obs_initial_behavior(
          obs_info.second.init_behavior);
    }
  }
}

void ObstaclesDecider::MakeInteractiveObsDecision(
    ReferenceLineInfo* reference_line_info) {
  ADEBUG << "MakeInteractiveObsDecision";
  std::pair<uint32_t, InteractiveDecision> current_behavior_info = {};
  auto* adc_obs_behavior_pair_vec =
      reference_line_info->GetMutableAdcObsBehavior();
  adc_obs_behavior_pair_vec->clear();
  const auto& adc_leading_obs_id = obs_info_.GetAdcLeadingObsId();
  const auto& interactive_obs = obs_info_.GetInteractiveObs();
  ObjectDecisionType decision;
  std::pair<InteractiveObsBehavior::Behavior, InteractiveObsBehavior::Behavior>
      adc_obs_behavior = {InteractiveObsBehavior::ConstantSpeed,
                          InteractiveObsBehavior::ConstantSpeed};
  bool is_yield_flag = true;
  for (const auto& inter_obs : interactive_obs) {
    const auto& inter_id = inter_obs.first;
    // once overtake the front_inter, not continue to Evaluate
    if (is_yield_flag && !inter_id.empty()) {
      double min_cost = std::numeric_limits<double>::infinity();
      is_yield_flag = behavior_evaluator_.Evaluate(
          reference_line_info, inter_id, obs_info_.GetObstaclesInfo(),
          {adc_leading_obs_id, inter_obs.second}, obs_info_.GetAdcMergeInfo(),
          &adc_obs_behavior, &min_cost);
      if (is_yield_flag) {
        decision.mutable_yield();
      } else {
        decision.mutable_overtake();
      }
      InteractiveDecision adc_obs_behavior_pair = {
          inter_id, {decision, adc_obs_behavior}};
      adc_obs_behavior_pair_vec->emplace_back(adc_obs_behavior_pair);
    }
  }
  if (adc_obs_behavior_pair_vec != nullptr &&
      !adc_obs_behavior_pair_vec->empty()) {
    ADEBUG << "adc_obs_behavior_pair_vec:";
    for (const auto& obs : *adc_obs_behavior_pair_vec) {
      ADEBUG << obs.first << " " << obs.second.first.DebugString();

      InteractiveObsBehavior* mutable_interactive_obs_behavior_debug =
          mutable_obstacles_decider_debug_->add_interactive_obs_behavior();
      mutable_interactive_obs_behavior_debug->set_interactive_obs_id(obs.first);
      if (obs.second.first.has_yield()) {
        mutable_interactive_obs_behavior_debug->set_obs_decision(
            InteractiveObsBehavior::Yield);
      } else if (obs.second.first.has_overtake()) {
        mutable_interactive_obs_behavior_debug->set_obs_decision(
            InteractiveObsBehavior::Overtake);
      }
    }
  }
}

void ObstaclesDecider::MakeIgnoreDecision(PathDecision* path_decision) {
  std::vector<std::string> ignore_ids;
  obs_info_.GetObsFrontCIPV(&ignore_ids, path_decision->obstacles());
  ObjectDecisionType ignore;
  ignore.mutable_ignore();
  for (const auto& ignore_id : ignore_ids) {
    path_decision->AddLongitudinalDecision("obstacle_decider/cipvfront",
                                           ignore_id, ignore);
    path_decision->AddLateralDecision("obstacle_decider/cipvfront", ignore_id,
                                      ignore);
  }
}

void ObstaclesDecider::SetIgnoreObstacle(
    ReferenceLineInfo* const reference_line_info) {
  if (reference_line_info == nullptr) {
    return;
  }
  const double adc_start_s = reference_line_info->AdcSlBoundary().start_s();
  const double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
  ObstaclesDeciderDebug* mutable_obstacles_decider_debug =
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_decider_debug()
          ->mutable_obstacles_decider_debug();
  ObjectDecisionType decision;
  decision.mutable_ignore();
  for (const Obstacle* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }
    const double start_s = obstacle->PerceptionSLBoundary().start_s();
    const double end_s = obstacle->PerceptionSLBoundary().end_s();
    const bool is_back_obs = (start_s + end_s) < (adc_start_s + adc_end_s);
    if (is_back_obs) {
      const double obs_distance_to_adc =
          GetDistanceBetweenADCAndObstacle(reference_line_info, obstacle);
      const double ttc =
          common::util::IsFloatEqual(
              obstacle->speed(),
              reference_line_info->vehicle_state().linear_velocity())
              ? std::numeric_limits<double>::max()
              : obs_distance_to_adc /
                    (obstacle->speed() -
                     reference_line_info->vehicle_state().linear_velocity());
      ADEBUG << "obs id: " << obstacle->Id() << ", ttc: " << ttc;
      constexpr double kBackIgnoreDistance = 5.0;
      if (adc_start_s - end_s > kBackIgnoreDistance &&
          (ttc > kBackObstacleIgnoreTtc || ttc < 0.0)) {
        ADEBUG << "obs id: " << obstacle->Id() << ", back away! "
               << "ttc: " << ttc;
        reference_line_info->path_decision()->AddLongitudinalDecision(
            "obstacle_decider/ignore", obstacle->Id(), decision);
        InteractiveObsBehavior* mutable_interactive_obs_behavior_debug =
            mutable_obstacles_decider_debug->add_interactive_obs_behavior();
        mutable_interactive_obs_behavior_debug->set_interactive_obs_id(
            obstacle->Id());
        mutable_interactive_obs_behavior_debug->set_obs_decision(
            InteractiveObsBehavior::Ignore);
      }
    }
  }
}

common::Status ObstaclesDecider::MakeAvoidAlongsideDecision(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (frame == nullptr || reference_line_info == nullptr ||
      trajectory_generator_ == nullptr || trajectory_evaluator_ == nullptr ||
      conflict_resolution_ == nullptr || obstacle_selector_ == nullptr) {
    AERROR << "Failed! Check trajectory_generator_ == nullptr || "
              "trajectory_evaluator_ == "
              "nullptr || conflict_resolution_ == nullptr || "
              "obstacle_selector_ == nullptr!";
    // TODO(cc): 需返回错误码
    return Status::OK();
  }
  auto* avoid_alongside_debug = reference_line_info->mutable_debug()
                                    ->mutable_avoid_alongside_decider_info();
  SetIgnoreObstacle(reference_line_info);
  // 1. 选择需决策障碍物
  double start_time = Clock::NowInSeconds();
  double current_time = start_time;
  // decision_obstacles：需要做决策的障碍物
  // dangerous_obstacles：影响做决策的障碍物
  std::unordered_set<std::string> decision_obstacles;
  std::unordered_set<std::string> dangerous_obstacles;
  obstacle_selector_->Process(
      obs_info_.GetAlongsideObsPerceptionIds(),
      obs_info_.GetNearestFrontCutInObs(), obs_info_.GetNearestBlockingObs(),
      obs_info_.GetCutinObsPerceptionIds(), reference_line_info,
      &decision_obstacles, &dangerous_obstacles);
  for (const auto& decision_obs_id : decision_obstacles) {
    ADEBUG << "decision_obs_id" << decision_obs_id;
  }
  for (const auto& dangerous_obs_id : dangerous_obstacles) {
    ADEBUG << "dangerous_obs_id" << dangerous_obs_id;
  }
  const double obstacle_selector_time_delay =
      (Clock::NowInSeconds() - current_time) * 1000;
  ADEBUG << "Obstacle Select Time = " << obstacle_selector_time_delay;
  avoid_alongside_debug->mutable_debug_time_delay()
      ->set_obstacle_selector_time_delay(obstacle_selector_time_delay);
  current_time = Clock::NowInSeconds();

  // 2. 计算初始采样状态
  common::TrajectoryPoint start_point = frame->PlanningStartPoint();
  common::SLPoint sl_point;
  common::math::Vec2d xy_point;
  xy_point.set_x(start_point.path_point().x());
  xy_point.set_y(start_point.path_point().y());
  reference_line_info->reference_line().XYToSL(xy_point, &sl_point);
  start_point.mutable_path_point()->set_s(sl_point.s());
  start_point.mutable_path_point()->set_l(sl_point.l());
  PathPoint matched_point = reference_line_info->reference_line()
                                .GetReferencePoint(start_point.path_point().s())
                                .ToPathPoint(start_point.path_point().s());
  std::array<double, 3> init_s{};
  std::array<double, 3> init_d{};
  ComputeInitFrenetState(matched_point, start_point, &init_s, &init_d);
  const double init_state_time_delay =
      (Clock::NowInSeconds() - current_time) * 1000;
  ADEBUG << "init_state_time_delay = " << init_state_time_delay;
  avoid_alongside_debug->mutable_debug_time_delay()->set_init_state_time_delay(
      init_state_time_delay);
  current_time = Clock::NowInSeconds();

  // 3. 采样轨迹
  trajectory_generator_->Init(init_s, init_d, *reference_line_info);
  const std::vector<DiscretizedTrajectory>& discretized_trajectories =
      trajectory_generator_->GetDiscretizedTrajectories();
  const double trajectory_generator_time_delay =
      (Clock::NowInSeconds() - current_time) * 1000;
  ADEBUG << "trajectory_generator_time_delay = "
         << trajectory_generator_time_delay;
  avoid_alongside_debug->mutable_debug_time_delay()
      ->set_trajectory_generator_time_delay(trajectory_generator_time_delay);
  current_time = Clock::NowInSeconds();

  // 3.5. 针对危险障碍物做意图估计
  std::unordered_map<int, double> cutin_estimation_infos;
  CutinProbEstimate(decision_obstacles, reference_line_info,
                    &cutin_estimation_infos);
  const double cutin_prob_estimation_time_delay =
      (Clock::NowInSeconds() - current_time) * 1000;
  avoid_alongside_debug->mutable_debug_time_delay()
      ->set_cutin_prob_estimation_time_delay(cutin_prob_estimation_time_delay);
  current_time = Clock::NowInSeconds();

  // 4. 为每个障碍物选择最佳的自车轨迹，同时做出行为决策
  trajectory_evaluator_->MakeObstacleDecision(
      discretized_trajectories, decision_obstacles, dangerous_obstacles,
      obs_info_.GetNearestFrontCutInObs(), obs_info_.GetNearestBlockingObs(),
      cutin_estimation_infos, reference_line_info);
  const double make_obstacle_decision_time_delay =
      (Clock::NowInSeconds() - current_time) * 1000;
  ADEBUG << "make_obstacle_decision_time_delay = "
         << make_obstacle_decision_time_delay;
  avoid_alongside_debug->mutable_debug_time_delay()
      ->set_make_obstacle_decision_time_delay(
          make_obstacle_decision_time_delay);
  current_time = Clock::NowInSeconds();

  // 5. 冲突解决
  conflict_resolution_->Process(reference_line_info);
  const double conflict_resolution_time_delay =
      (Clock::NowInSeconds() - current_time) * 1000;
  ADEBUG << "conflict_resolution_time_delay = "
         << conflict_resolution_time_delay;
  avoid_alongside_debug->mutable_debug_time_delay()
      ->set_conflict_resolution_time_delay(conflict_resolution_time_delay);
  current_time = Clock::NowInSeconds();

  // 6. debug
  if (enable_debug_avoid_alongside_decision_) {
    trajectory_evaluator_->DebugAvoidAlongsideDecision(reference_line_info);
  }
  const double debug_time_delay = (Clock::NowInSeconds() - current_time) * 1000;
  ADEBUG << "debug_time_delay = " << debug_time_delay;
  avoid_alongside_debug->mutable_debug_time_delay()
      ->set_debug_avoid_alongside_decision_time_delay(debug_time_delay);
  const double all_time_delay = (Clock::NowInSeconds() - start_time) * 1000;
  ADEBUG << "avoid alongside total time = " << all_time_delay;
  avoid_alongside_debug->mutable_debug_time_delay()->set_all_time_delay(
      all_time_delay);

  return Status::OK();
}

void ObstaclesDecider::CutinProbEstimate(
    const std::unordered_set<std::string>& decision_obstacles,
    ReferenceLineInfo* const reference_line_info,
    std::unordered_map<int, double>* const cutin_infos) {
  if (reference_line_info == nullptr || cutin_infos == nullptr) {
    return;
  }
  std::unordered_set<int> perception_ids;
  constexpr double kStableDistance = 80.0;
  for (const auto& obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle != nullptr &&
        decision_obstacles.find(obstacle->Id()) != decision_obstacles.end() &&
        obstacle->PerceptionSLBoundary().start_s() -
                reference_line_info->AdcSlBoundary().end_s() <
            kStableDistance &&
        obstacle->PerceptionSLBoundary().end_s() >
            reference_line_info->AdcSlBoundary().start_s()) {
      perception_ids.emplace(obstacle->PerceptionId());
    }
  }

  *cutin_infos = cutin_intention_estimation_.NaiveBayesCutinProbEstimate(
      perception_ids, reference_line_info);
  for (auto cutin_info : *cutin_infos) {
    ADEBUG << "obs id: " << cutin_info.first
           << ", cutin_prob:" << cutin_info.second;
  }
}

void ObstaclesDecider::ComputeInitFrenetState(
    const PathPoint& matched_point, const TrajectoryPoint& cartesian_state,
    std::array<double, 3>* ptr_s, std::array<double, 3>* ptr_d) {
  if (ptr_s == nullptr || ptr_d == nullptr) {
    return;
  }
  CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.s(), matched_point.x(), matched_point.y(),
      matched_point.theta(), matched_point.kappa(), matched_point.dkappa(),
      cartesian_state.path_point().x(), cartesian_state.path_point().y(),
      cartesian_state.v(), cartesian_state.a(),
      cartesian_state.path_point().theta(),
      cartesian_state.path_point().kappa(), ptr_s, ptr_d);
}

}  // namespace TL::planning
