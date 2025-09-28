/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file forward_gear_scenario.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/forward_gear_scenario.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>

#include "common/file/log.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/slt_obstacle_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/nudge_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/obstacle_expected_distance_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/obstacle_safe_distance_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_stage.h"
#include "planning/proto/planning_config.pb.h"
#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

ForwardGearScenario::ForwardGearScenario(const SpeedScenarioConfig& config)
    : SpeedScenario(config),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
}

bool ForwardGearScenario::PreProcess(
    const std::shared_ptr<DependencyInjector>& injector, Frame* frame,
    ReferenceLineInfo* reference_line_info, common::TrajectoryPoint* init_point,
    SpeedCache* const cache) {
  if (injector == nullptr || frame == nullptr ||
      reference_line_info == nullptr || init_point == nullptr ||
      cache == nullptr) {
    return false;
  }

  cache->Init(injector, *frame, reference_line_info, *init_point);

  // stage preprocess
  const auto& current_stage = GetCurrentStage();
  if (current_stage == nullptr ||
      !current_stage->PreProcess(frame, reference_line_info)) {
    AERROR << "current stage preprocess failed";
    return false;
  }

  // init cache
  cache->InitObstacle(injector, *frame, *reference_line_info, *init_point);

  CheckIfCutInBegin(*frame, *reference_line_info, *init_point, cache);
  UpdateFollowCost(*cache);
  return true;
}

bool ForwardGearScenario::PostProcess(
    Frame* frame, ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, SpeedCache* const cache,
    const std::shared_ptr<SpeedDataGenerator>& generator,
    SpeedData* speed_data) {
  if (frame == nullptr || reference_line_info == nullptr || cache == nullptr ||
      generator == nullptr || speed_data == nullptr) {
    return false;
  }

  // stage postprocess
  const auto& current_stage = GetCurrentStage();
  if (current_stage == nullptr ||
      !current_stage->PostProcess(frame, reference_line_info, init_point,
                                  *cache, generator, speed_data)) {
    AERROR << "current stage post process failed";
    return false;
  }

  SelectFollowTime(reference_line_info, cache,
                   current_stage->GetSpeedEvaluator());
  CheckIfStableFollow(cache, *speed_data);
  CheckIfCutInFinish(cache, *speed_data);
  MakeNudgeDecision(reference_line_info, init_point, *cache);
  MarkOvertakeForNudgeObstacle(reference_line_info, *cache);
  cache->UpdateCurrentNudgeState(speed_data);
  cache->SetIsLastFallback(speed_data->GetIsFallback() &&
                           !frame->IsVehicleStandStill());
  return true;
}

void ForwardGearScenario::SelectFollowTime(
    ReferenceLineInfo* reference_line_info, SpeedCache* const cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator) {
  if (cache == nullptr || evaluator == nullptr ||
      reference_line_info == nullptr) {
    return;
  }

  // if no follow target, or no nudge obstacle, keep LonCtrlTime
  const auto& nudge_obstacle_caches = cache->GetNudgeObstacleCaches();
  const auto set_follow_time = reference_line_info->GetLonCtrlTime();
  if (cache->GetFollowSTObstacleCache() == nullptr ||
      nudge_obstacle_caches.empty()) {
    cache->SetFollowTime(set_follow_time);
    return;
  }

  // if adc has not reach stable follow state, return
  if (!cache->GetBasicCache().GetIsStableFollow()) {
    return;
  }

  // determine whether we use nudge cost
  const auto& speed_costs = evaluator->GetSpeedCosts();
  std::shared_ptr<NudgeCost> nudge_cost = nullptr;
  for (const auto& speed_cost : speed_costs) {
    nudge_cost = std::dynamic_pointer_cast<NudgeCost>(speed_cost);
    if (nudge_cost != nullptr) {
      break;
    }
  }
  if (nudge_cost == nullptr) {
    cache->SetFollowTime(set_follow_time);
    return;
  }

  // calculate follow time cost, select best follow time
  double selected_follow_time = set_follow_time;
  double min_follow_time_cost = std::numeric_limits<double>::max();
  for (const auto& follow_time : GetScenarioConfig().follow_time()) {
    const auto follow_time_cost = nudge_cost->CalculateFollowTimeCost(
        *cache, *reference_line_info, follow_time, set_follow_time);
    if (follow_time_cost < min_follow_time_cost) {
      min_follow_time_cost = follow_time_cost;
      selected_follow_time = follow_time;
    }
    ADEBUG << "follow_time:" << follow_time
           << ", follow_time_cost:" << follow_time_cost;
  }
  ADEBUG << "selected_follow_time:" << selected_follow_time
         << ", :" << cache->GetHasNonIgnoreNudgeObstacle();
  cache->SetFollowTime(selected_follow_time);
}

void ForwardGearScenario::UpdateFollowCost(const SpeedCache& cache) {
  const auto& current_stage = GetCurrentStage();
  if (current_stage == nullptr ||
      current_stage->GetSpeedEvaluator() == nullptr) {
    return;
  }
  for (const auto& speed_cost :
       current_stage->GetSpeedEvaluator()->GetSpeedCosts()) {
    const auto& follow_cost =
        std::dynamic_pointer_cast<ObstacleExpectedDistanceCost>(speed_cost);
    if (follow_cost != nullptr) {
      follow_cost->SetFollowTime(cache.GetFollowTime());
    }
  }
}

void ForwardGearScenario::CheckIfStableFollow(
    SpeedCache* const cache, const SpeedData& speed_data) const {
  if (cache == nullptr) {
    return;
  }

  auto* basic_cache = cache->GetMutableBasicCache();
  if (basic_cache == nullptr) {
    return;
  }

  const auto* follow_obstacle_cache = cache->GetFollowSLTObstacleCache();
  if (follow_obstacle_cache == nullptr) {
    basic_cache->SetIsStableFollow(true);
    return;
  }

  auto less_follow_time_error = 0.0;
  auto greater_follow_time_error = 0.0;
  const auto follow_time = cache->GetFollowTime();
  for (const auto& point : speed_data) {
    const auto& obstacle_info =
        follow_obstacle_cache->GetObstacleInfoAtTime(point.t());

    const auto expected_distance = follow_obstacle_cache->GetMinStopDistance() +
                                   fmax(follow_time * point.v(), 0.0);
    auto real_distance = obstacle_info.s_lower -
                         (point.s() + vehicle_param_.front_edge_to_center());
    const auto v = fmax(point.v(), 1.0);

    if (real_distance > expected_distance) {
      greater_follow_time_error += (real_distance - expected_distance) / v;
    } else {
      less_follow_time_error += (expected_distance - real_distance) / v;
    }
  }

  less_follow_time_error /= static_cast<int>(speed_data.size());
  greater_follow_time_error /= static_cast<int>(speed_data.size());

  basic_cache->SetIsStableFollow(
      fmax(less_follow_time_error, greater_follow_time_error) < 0.12);
}

bool ForwardGearScenario::CheckIfBeginNudge(
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const NudgeObstacleCache& obstacle_cache) const {
  const auto* obstacle = obstacle_cache.GetObstacle();
  if (obstacle == nullptr) {
    return false;
  }
  const auto* last_frame = cache.GetBasicCache().GetLastFrame();
  if (last_frame == nullptr) {
    return false;
  }
  const auto* last_reference_line_info = last_frame->DriveReferenceLineInfo();
  if (last_reference_line_info == nullptr) {
    return false;
  }

  const auto& last_obstacles =
      last_reference_line_info->path_decision().obstacles().Items();
  const auto iter = std::find_if(
      last_obstacles.begin(), last_obstacles.end(),
      [&](const auto& last_obstacle) {
        return (last_obstacle != nullptr) &&
               (last_obstacle->Perception().id() ==
                obstacle->Perception().id()) &&
               last_obstacle->LongitudinalDecision().has_lon_nudge();
      });

  if (iter != last_obstacles.end()) {
    return true;
  }

  const auto& obstacle_info = obstacle_cache.GetObstacleInfoAtTime(0.0);
  const auto s_upper = vehicle_param_.front_edge_to_center();

  if (obstacle_info.s_lower < s_upper) {
    return true;
  }

  static constexpr double nudge_ttc = 5.0;
  const auto& delta_v = (init_point.v() - obstacle_info.ds);
  return (delta_v > 0.0 &&
          (obstacle_info.s_lower - s_upper) / delta_v < nudge_ttc);
}

void ForwardGearScenario::MakeNudgeDecision(
    ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache) {
  if (reference_line_info == nullptr) {
    return;
  }
  auto* path_decision = reference_line_info->path_decision();
  if (path_decision == nullptr) {
    return;
  }
  for (const auto& nudge_obstacle_cache : cache.GetNudgeObstacleCaches()) {
    const auto* obstacle = nudge_obstacle_cache.GetObstacle();
    if (obstacle == nullptr ||
        obstacle->GetPathSLTBoundary().GetBoundaryTypes().count(
            SLTBoundary::BoundaryType::NUDGE_CAUTION) > 0 ||
        nudge_obstacle_cache.GetCurrentNudgeState() !=
            SpeedCacheConfig::OVERTAKE ||
        !CheckIfBeginNudge(init_point, cache, nudge_obstacle_cache)) {
      continue;
    }

    auto* mutable_obstacle = path_decision->Find(obstacle->Id());
    if (mutable_obstacle == nullptr) {
      continue;
    }

    ObjectDecisionType nudge_decision;
    nudge_decision.mutable_lon_nudge();
    mutable_obstacle->AddLongitudinalDecision("lon_nudge", nudge_decision);
  }
}

void ForwardGearScenario::MarkOvertakeForNudgeObstacle(
    ReferenceLineInfo* const reference_line_info, const SpeedCache& cache) {
  if (reference_line_info == nullptr) {
    return;
  }
  auto* big_car_set = reference_line_info->GetMutableOvertakeBigCarSet();
  if (big_car_set == nullptr) {
    return;
  }
  for (const auto& nudge_obstacle_cache : cache.GetNudgeObstacleCaches()) {
    const auto* obstacle = nudge_obstacle_cache.GetObstacle();
    if (obstacle != nullptr && nudge_obstacle_cache.GetCurrentNudgeState() ==
                                   SpeedCacheConfig::OVERTAKE) {
      big_car_set->insert(obstacle->Id());
    }
  }
}

bool ForwardGearScenario::CheckIfKeepStandStill(
    Frame* const frame, const ReferenceLineInfo& reference_line_info,
    SpeedCache* const cache, common::TrajectoryPoint* const init_point) {
  if (init_point == nullptr || frame == nullptr || cache == nullptr) {
    AERROR << "input data is null";
    return true;
  }

  const double max_adc_stop_speed = common::VehicleConfigHelper::GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  const auto is_acc_standstill_wait =
      frame->local_view().HasFunctionManagerIn() &&
      frame->local_view().GetFunctionManagerIn() != nullptr &&
      frame->local_view()
              .GetFunctionManagerIn()
              ->fct_nnp_in()
              .longitud_ctrl_cruise_speedms() < 0.0 &&
      frame->local_view().GetFunctionManagerIn()->fct_nnp_in().acc_state() ==
          functionmanager::FctToNnpInput::ACC_STANDSTILL_WAIT &&
      fabs(frame->vehicle_state().linear_velocity()) < max_adc_stop_speed;

  const auto adc_start_distance_threshold_after_stop =
      GetScenarioConfig().adc_start_distance_threshold_after_stop();
  if (frame->vehicle_state().linear_velocity() < max_adc_stop_speed &&
      (cache->GetBasicCache().GetRealStopS() >
           adc_start_distance_threshold_after_stop ||
       (cache->GetLastStartAfterStop() &&
        cache->GetBasicCache().GetRealStopS() >
            adc_start_distance_threshold_after_stop * 0.5))) {
    init_point->set_v(fmax(init_point->v(), 0.0));
    init_point->set_a(fmax(init_point->a(), 0.0));
    cache->SetStartAfterStop(true);
    ADEBUG << "start after stop";
    return is_acc_standstill_wait;
  }

  if (std::isinf(cache->GetBasicCache().GetRealStopS()) ||
      cache->GetIsLastFallback()) {
    frame->SetSpeedPlanStandstill(
        frame->vehicle_state().linear_velocity() < max_adc_stop_speed &&
        init_point->v() < max_adc_stop_speed && cache->GetIsLastFallback());
    return is_acc_standstill_wait;
  }

  const auto* last_frame = cache->GetBasicCache().GetLastFrame();
  if (last_frame == nullptr) {
    return is_acc_standstill_wait;
  }

  const auto* last_reference_line_info = last_frame->DriveReferenceLineInfo();
  const auto need_standstill =
      last_reference_line_info != nullptr &&
      cache->GetBasicCache().GetRealStopS() <
          GetScenarioConfig().adc_start_distance_threshold_after_stop() &&
      last_reference_line_info->path_data()
              .frenet_frame_path()
              .is_forward_path() == reference_line_info.path_data()
                                        .frenet_frame_path()
                                        .is_forward_path() &&
      init_point->v() < max_adc_stop_speed;
  frame->SetSpeedPlanStandstill(need_standstill);
  return (need_standstill || is_acc_standstill_wait);
}

bool ForwardGearScenario::GenerateStandStillSpeedData(
    const common::TrajectoryPoint& init_point, SpeedData* const speed_data) {
  if (speed_data == nullptr) {
    return false;
  }

  const auto a = fmin(GetScenarioConfig().standstill_accel(), -1e-6);
  const auto max_decel_t = -init_point.v() / a;
  const auto max_s = 0.5 * init_point.v() * max_decel_t;
  common::SpeedPoint speed_point;
  const auto t_count = static_cast<int>(std::round(
      FLAGS_trajectory_time_length / FLAGS_trajectory_time_resolution));
  for (int i = 0; i <= t_count; ++i) {
    const auto t = i * FLAGS_trajectory_time_resolution;
    if (t < max_decel_t) {
      speed_point.set_s(init_point.v() * t + 0.5 * a * t * t);
      speed_point.set_v(init_point.v() + a * t);
    } else {
      speed_point.set_s(max_s);
      speed_point.set_v(0.0);
    }
    speed_point.set_t(t);
    speed_point.set_a(a);
    speed_point.set_da(0.0);
    speed_data->push_back(speed_point);
  }

  return true;
}

void ForwardGearScenario::CheckIfCutInBegin(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const common::TrajectoryPoint& init_point, SpeedCache* cache) {
  // check cut in status
  auto* obstacle_caches = cache->GetMutableSLTObstacleCaches();
  if (obstacle_caches == nullptr) {
    return;
  }

  const auto& old_obstacle_caches = cache->GetOldSLTObstacleCaches();
  for (auto& obstacle_cache : *obstacle_caches) {
    obstacle_cache.SetIsCutIn(
        std::any_of(old_obstacle_caches.begin(), old_obstacle_caches.end(),
                    [&](const auto& old_obstacle_cache) {
                      return old_obstacle_cache.GetPerceptionId() ==
                                 obstacle_cache.GetPerceptionId() &&
                             old_obstacle_cache.GetIsCutIn();
                    }));
    if (!obstacle_cache.GetIsCutIn()) {
      obstacle_cache.CheckIfCutInBegin(frame, reference_line_info);
    }

    if (obstacle_cache.GetIsCutIn() &&
        GetScenarioConfig().scenario_type() !=
            SpeedScenarioConfig::LANE_MERGE_SCENARIO) {
      const auto dv =
          obstacle_cache.GetObstacleInfoAtTime(obstacle_cache.GetMinT()).ds -
          init_point.v();
      obstacle_cache.SetCutInSafty(dv > 0.0 ? dv * 6 : 0.0);
    }
  }
}

void ForwardGearScenario::CheckIfCutInFinish(SpeedCache* cache,
                                             const SpeedData& speed_data) {
  if (cache == nullptr) {
    return;
  }

  const auto& current_stage = GetCurrentStage();
  if (current_stage == nullptr) {
    return;
  }
  const auto& evaluator = current_stage->GetSpeedEvaluator();
  if (evaluator == nullptr) {
    return;
  }

  std::shared_ptr<ObstacleSafeDistanceCost> obstacle_safe_distance_cost =
      nullptr;
  for (const auto& speed_cost : evaluator->GetSpeedCosts()) {
    obstacle_safe_distance_cost =
        std::dynamic_pointer_cast<ObstacleSafeDistanceCost>(speed_cost);
    if (obstacle_safe_distance_cost != nullptr) {
      break;
    }
  }
  if (obstacle_safe_distance_cost == nullptr) {
    return;
  }

  auto* obstacle_caches = cache->GetMutableSLTObstacleCaches();
  if (obstacle_caches == nullptr) {
    return;
  }
  for (auto& obstacle_cache : *obstacle_caches) {
    if (obstacle_cache.GetIsCutIn() &&
        obstacle_safe_distance_cost->CheckIsLongitudinalSafe(speed_data,
                                                             obstacle_cache)) {
      obstacle_cache.SetIsCutIn(false);
    }
  }
}

}  // namespace planning
}  // namespace TL
