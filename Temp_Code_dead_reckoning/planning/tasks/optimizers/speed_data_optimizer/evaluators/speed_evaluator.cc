/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_curve_evaluator.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator.h"

#include <algorithm>
#include <limits>

#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost_factory.h"
#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

SpeedEvaluator::SpeedEvaluator(const SpeedEvaluatorConfig& config)
    : speed_evaluator_config_(config) {

  for (const auto& speed_cost_config : config.speed_cost_config()) {
    speed_costs_.emplace_back(
        SpeedCostFactory::CreateSpeedCost(speed_cost_config));
  }
}

void SpeedEvaluator::ResetCost(SpeedCurveCostResult* const cost_result) {
  if (cost_result == nullptr) {
    return;
  }

  cost_result->non_follow_safe_distance_cost = 0.0;
  cost_result->obstacle_safe_distance_cost = 0.0;
  cost_result->lateral_distance_cost = 0.0;
  cost_result->accel_cost = 0.0;
  cost_result->jerk_cost = 0.0;
  cost_result->low_speed_cost = 0.0;
  cost_result->over_curvature_speed_limit_cost = 0.0;
  cost_result->over_decision_speed_limit_cost = 0.0;
  cost_result->over_comfortable_speed_limit_cost = 0.0;
  cost_result->over_critical_speed_limit_cost = 0.0;
  cost_result->over_nudge_speed_limit_cost = 0.0;
  cost_result->over_speed_cost = 0.0;
  cost_result->safe_over_speed_cost = 0.0;
  cost_result->stop_distance_cost = 0.0;
  cost_result->gap_cost = 0.0;
  cost_result->parallel_drive_cost = 0.0;
  cost_result->max_over_cruise_speed_limit_ratio = 0.0;
  cost_result->max_over_map_speed_limit_ratio = 0.0;
  cost_result->obstacle_less_expected_distance_cost = 0.0;
  cost_result->obstacle_greater_expected_distance_cost = 0.0;
  cost_result->curve_priority_cost = 0.0;
  cost_result->yield_cost = 0.0;
  cost_result->collision_cost = 0.0;
  cost_result->mean_less_follow_time_error = 0.0;
  cost_result->mean_greater_follow_time_error = 0.0;
  cost_result->mode_cost = std::numeric_limits<double>::infinity();
  cost_result->total_cost = std::numeric_limits<double>::infinity();
  cost_result->front_obstacle_cost.collision_cost = 0.0;
  cost_result->front_obstacle_cost.non_follow_safe_distance_cost = 0.0;
  cost_result->front_obstacle_cost.obstacle_safe_distance_cost = 0.0;
  cost_result->front_obstacle_cost.obstacle_less_expected_distance_cost = 0.0;
  cost_result->front_obstacle_cost.obstacle_greater_expected_distance_cost =
      0.0;
  cost_result->front_obstacle_cost.lateral_distance_cost = 0.0;
  cost_result->front_obstacle_cost.yield_cost = 0.0;
  cost_result->front_obstacle_cost.mean_less_follow_time_error = 0.0;
  cost_result->front_obstacle_cost.mean_greater_follow_time_error = 0.0;
  cost_result->nudge_merge_obstacle = false;
}

void SpeedEvaluator::ResetToFrontCost(SpeedCurveCostResult* const cost_result) {
  if (cost_result == nullptr) {
    return;
  }

  cost_result->collision_cost = cost_result->front_obstacle_cost.collision_cost;
  cost_result->non_follow_safe_distance_cost =
      cost_result->front_obstacle_cost.non_follow_safe_distance_cost;
  cost_result->obstacle_safe_distance_cost =
      cost_result->front_obstacle_cost.obstacle_safe_distance_cost;
  cost_result->obstacle_less_expected_distance_cost =
      cost_result->front_obstacle_cost.obstacle_less_expected_distance_cost;
  cost_result->obstacle_greater_expected_distance_cost =
      cost_result->front_obstacle_cost.obstacle_greater_expected_distance_cost;
  cost_result->lateral_distance_cost =
      cost_result->front_obstacle_cost.lateral_distance_cost;
  cost_result->yield_cost = cost_result->front_obstacle_cost.yield_cost;
  cost_result->mean_less_follow_time_error =
      cost_result->front_obstacle_cost.mean_less_follow_time_error;
  cost_result->mean_greater_follow_time_error =
      cost_result->front_obstacle_cost.mean_greater_follow_time_error;
  cost_result->total_cost =
      cost_result->non_follow_safe_distance_cost +
      cost_result->obstacle_safe_distance_cost + cost_result->accel_cost +
      cost_result->jerk_cost + cost_result->low_speed_cost +
      cost_result->over_speed_cost + cost_result->stop_distance_cost +
      cost_result->gap_cost + cost_result->parallel_drive_cost +
      cost_result->obstacle_less_expected_distance_cost +
      cost_result->obstacle_greater_expected_distance_cost +
      cost_result->curve_priority_cost + cost_result->yield_cost +
      cost_result->lateral_distance_cost + cost_result->collision_cost;
}

bool SpeedEvaluator::CalculateMergeStopDistanceCost(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) {
  if (cost_result == nullptr || stop_distance_cost_evaluator_ == nullptr) {
    return false;
  }
  cost_result->total_cost -= cost_result->stop_distance_cost;
  cost_result->stop_distance_cost = 0.0;
  stop_distance_cost_evaluator_->CalculateCost(cache, reference_line_info,
                                               cost_result);
  cost_result->total_cost += cost_result->stop_distance_cost;
  return true;
}

void SpeedEvaluator::SetMergeStop(const bool is_merge_stop) {
  for (const auto& speed_cost : speed_costs_) {
    if (speed_cost->CostType() == SpeedCostConfig::STOP_DISTANCE_COST) {
      speed_cost->SetIsMergeStop(is_merge_stop);
      stop_distance_cost_evaluator_ = is_merge_stop ? speed_cost : nullptr;
      break;
    }
  }
}

}  // namespace planning
}  // namespace TL
