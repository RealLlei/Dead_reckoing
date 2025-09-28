/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file over_speed_cost.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/costs/over_speed_cost.h"

#include <algorithm>
#include <cmath>

namespace TL {
namespace planning {

OverSpeedCost::OverSpeedCost(const SpeedCostConfig& config)
    : SpeedCost(config) {
  if (config.has_over_speed_cost_config()) {
    config_.CopyFrom(config.over_speed_cost_config());
  }
}

bool OverSpeedCost::CalculateCost(const SpeedCache& cache,
                                  const ReferenceLineInfo& reference_line_info,
                                  SpeedCurveCostResult* cost_result) const {
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->over_speed_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  const auto& speed_limit_cache = cache.GetSpeedLimitCache();
  const auto& dense_points = cost_result->curve->GetDensePoints();

  // check position speed limit
  double over_position_map_speed_limit_cost = 0.0;
  const auto max_dense_point_count =
      cost_result->curve->GetMaxDensePointCount();
  double over_curvature_speed_limit_cost_coef =
      config_.over_curvature_speed_limit_cost_coef();
#ifdef FOR_BAIDU_SIMULATION
  over_curvature_speed_limit_cost_coef =
      config_.baidu_over_curvature_speed_limit_cost_coef();
#endif
  for (int i = 1; i < max_dense_point_count; ++i) {
    const auto s = dense_points[i].s();
    const auto v = dense_points[i].v();
    const auto& position_speed_limit =
        speed_limit_cache.GetPositionSpeedLimit(s);

    // check curvature speed limit
    if (v > position_speed_limit.curvature_speed_limit) {
      cost_result->over_curvature_speed_limit_cost +=
          over_curvature_speed_limit_cost_coef *
          pow(v - position_speed_limit.curvature_speed_limit, 2);
    }

    // check map speed limit
    const auto over_map_speed_limit = v - position_speed_limit.map_speed_limit;
    if (over_map_speed_limit > config_.over_speed_limit_threshold()) {
      over_position_map_speed_limit_cost +=
          config_.over_comfortable_speed_limit_cost_coef() *
          pow(over_map_speed_limit, 2);
    }

    // check decision speed limit
    const auto over_decision_speed_limit =
        v - position_speed_limit.decision_speed_limit;
    if (over_decision_speed_limit > config_.over_speed_limit_threshold()) {
      cost_result->over_decision_speed_limit_cost +=
          config_.over_decision_speed_limit_cost_coef() *
          pow(over_decision_speed_limit, 2);
    }

    // check pedestrian speed limit
    const auto over_pedestrian_speed_limit =
        v - position_speed_limit.pedestrian_speed_limit;
    if (over_pedestrian_speed_limit > config_.over_speed_limit_threshold()) {
      cost_result->over_decision_speed_limit_cost +=
          config_.over_decision_speed_limit_cost_coef() *
          pow(over_pedestrian_speed_limit, 2);
    }

    // calculate max_over_map_speed_limit_ratio
    const auto over_real_map_speed_limit_ratio =
        (v - position_speed_limit.real_map_speed_limit) /
        position_speed_limit.real_map_speed_limit;
    if (cost_result->max_over_map_speed_limit_ratio <
        over_real_map_speed_limit_ratio) {
      cost_result->max_over_map_speed_limit_ratio =
          over_real_map_speed_limit_ratio;
    }
  }

  // calculate max_over_cruise_speed_limit
  const auto cruise_speed = reference_line_info.GetCruiseSpeed();
  cost_result->max_over_cruise_speed_limit_ratio =
      (cost_result->curve->GetMaxV() - cruise_speed) / cruise_speed;

  cost_result->over_curvature_speed_limit_cost /= (max_dense_point_count - 1);
  cost_result->over_decision_speed_limit_cost /= (max_dense_point_count - 1);

  // check time speed limit
  double over_time_map_speed_limit_cost = 0.0;
  const auto min_dense_point_count =
      cost_result->curve->GetMinDensePointCount();
  for (int i = 1; i < min_dense_point_count; ++i) {
    const auto t = dense_points[i].t();
    const auto v = dense_points[i].v();
    const auto& time_speed_limit = speed_limit_cache.GetTimeSpeedLimit(t);

    // check comfortable speed limit
    const auto over_comfortable_speed_limit =
        v - time_speed_limit.comfortable_speed_limit;
    if (over_comfortable_speed_limit > config_.over_speed_limit_threshold()) {
      over_time_map_speed_limit_cost +=
          config_.over_comfortable_speed_limit_cost_coef() *
          pow(over_comfortable_speed_limit, 2);
    }

    // check confortable speed limit
    const auto over_critical_speed_limit =
        v - time_speed_limit.critical_speed_limit;
    if (over_critical_speed_limit > config_.over_speed_limit_threshold()) {
      cost_result->over_critical_speed_limit_cost +=
          config_.over_critical_speed_limit_cost_coef() *
          pow(over_critical_speed_limit, 2);
    }

    // check nudge speed limit
    const auto over_nudge_speed_limit =
        v - speed_limit_cache.GetNudgeSpeedLimit(t, dense_points[i].s());
    if (over_nudge_speed_limit > config_.over_speed_limit_threshold()) {
      cost_result->over_nudge_speed_limit_cost +=
          config_.over_nudge_speed_limit_cost_coef() *
          pow(over_nudge_speed_limit, 2);
    }
  }

  cost_result->over_comfortable_speed_limit_cost =
      over_position_map_speed_limit_cost / (max_dense_point_count - 1) +
      over_time_map_speed_limit_cost / (min_dense_point_count - 1);
  cost_result->over_critical_speed_limit_cost /= (min_dense_point_count - 1);

  cost_result->safe_over_speed_cost =
      std::max({cost_result->over_curvature_speed_limit_cost,
                cost_result->over_decision_speed_limit_cost,
                cost_result->over_critical_speed_limit_cost,
                cost_result->over_nudge_speed_limit_cost});

  cost_result->over_speed_cost =
      fmax(cost_result->over_comfortable_speed_limit_cost,
           cost_result->safe_over_speed_cost);
  return true;
}

}  // namespace planning
}  // namespace TL
