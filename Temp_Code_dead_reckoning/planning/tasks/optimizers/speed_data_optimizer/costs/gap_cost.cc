/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file gap_cost.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/costs/gap_cost.h"

#include <algorithm>
#include <cmath>

namespace TL {
namespace planning {

GapCost::GapCost(const SpeedCostConfig& config)
    : SpeedCost(config),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
  if (config.has_gap_cost_config()) {
    config_.CopyFrom(config.gap_cost_config());
  }
}

bool GapCost::CalculateCost(const SpeedCache& cache,
                            const ReferenceLineInfo& reference_line_info,
                            SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->gap_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  cost_result->gap_cost = 0.0;
  const auto& basic_cache = cache.GetBasicCache();

  const auto& lane_change_prepare_target = cache.GetLaneChangePrepareTarget();

  if (GetPrintDebug()) {
    AERROR << "[GapCost]target_s:" << lane_change_prepare_target.s
           << ", target_v:" << lane_change_prepare_target.v
           << ", target_t:" << lane_change_prepare_target.t;
  }

  // if there is no lane change path
  // calculate acceleration and deceleration cost
  if (lane_change_prepare_target.t > 0.0 &&
      !basic_cache.GetIsChangeLanePath()) {
    const auto point =
        cost_result->curve->GetPoint(lane_change_prepare_target.t);
    cost_result->gap_cost =
        pow(point.s() - lane_change_prepare_target.s, 2) * 1e1 +
        pow(point.v() - lane_change_prepare_target.v, 2) * 1e20;
    if (GetPrintDebug()) {
      AERROR << "[GapCost]point_s:" << point.s() << ", v:" << point.v()
             << ", target_s:" << lane_change_prepare_target.s
             << ", target_v:" << lane_change_prepare_target.v
             << ", target_t:" << lane_change_prepare_target.t
             << ", gap_cost:" << cost_result->gap_cost;
    }
    return true;
  }
  // if there is no lane change path, do not consider gap cost
  if (!basic_cache.GetIsChangeLanePath()) {
    return true;
  }
  // if there is no gap_lead_obstacle_cache and gap_lead_obstacle_cache, do
  // not consider gap cost
  const auto* gap_front_obstacle_info = cache.GetGapFrontObstacleInfo();
  const auto* gap_rear_obstacle_info = cache.GetGapRearObstacleInfo();
  if (gap_front_obstacle_info == nullptr && gap_rear_obstacle_info == nullptr) {
    return true;
  }

  // check if adc is in the front of gap lead when finish lane change
  if (gap_front_obstacle_info != nullptr) {
    const auto& point =
        cost_result->curve->GetNearestPoint(gap_front_obstacle_info->t);
    const auto s_upper = point.s() + vehicle_param_.front_edge_to_center();
    if (s_upper > gap_front_obstacle_info->s_lower) {
      const auto distance_to_gap =
          fmin(config_.max_distance_to_gap(),
               s_upper - gap_front_obstacle_info->s_lower);
      cost_result->gap_cost =
          config_.not_in_gap_cost() + distance_to_gap * 500000;
      return true;
    }
  }

  // check if adc is in the back of gap tail when finish lane change
  if (gap_rear_obstacle_info != nullptr) {
    const auto& point =
        cost_result->curve->GetNearestPoint(gap_rear_obstacle_info->t);
    const auto s_lower = point.s() - vehicle_param_.back_edge_to_center();
    if (s_lower < gap_rear_obstacle_info->s_upper) {
      const auto distance_to_gap =
          fmin(config_.max_distance_to_gap(),
               gap_rear_obstacle_info->s_upper - s_lower);
      cost_result->gap_cost =
          config_.not_in_gap_cost() + distance_to_gap * 100000;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace TL
