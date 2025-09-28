/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file low_speed_cost.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/costs/low_speed_cost.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "common/math/double_type.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyLess;

LowSpeedCost::LowSpeedCost(const SpeedCostConfig& config) : SpeedCost(config) {
  if (config.has_low_speed_cost_config()) {
    config_.CopyFrom(config.low_speed_cost_config());
  }
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
}

bool LowSpeedCost::CalculateCost(const SpeedCache& cache,
                                 const ReferenceLineInfo& reference_line_info,
                                 SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->low_speed_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  const auto start_v = cost_result->curve->GetStartV();
  const auto end_v = cost_result->curve->GetEndV();
  const auto max_adc_stop_speed = vehicle_param_.max_abs_speed_when_stopped();
  bool has_over_acce = false;
  int keep_clear_stop_count = 0;

  const auto& dense_points = cost_result->curve->GetDensePoints();
  const auto min_dense_point_count =
      cost_result->curve->GetMinDensePointCount();

  cost_result->low_speed_cost = 0.0;
  const auto& speed_limit_cache = cache.GetSpeedLimitCache();
  for (int i = 1; i < min_dense_point_count; ++i) {
    const auto s = dense_points[i].s();
    const auto v = dense_points[i].v();
    const auto t = dense_points[i].t();
    const auto low_speed_threshold =
        speed_limit_cache.GetLowSpeedThresholdByTime(t);
    if (DefinitelyLess(v, low_speed_threshold)) {
      cost_result->low_speed_cost +=
          config_.low_speed_cost_coef() * (low_speed_threshold + 2.0 - v);
    }

    if (v < max_adc_stop_speed && speed_limit_cache.InKeepClearRange(s)) {
      ++keep_clear_stop_count;
    }

    if ((end_v > start_v && v > end_v) || (end_v < start_v && v < end_v)) {
      has_over_acce = true;
    }
  }

  cost_result->low_speed_cost /= (min_dense_point_count - 1);

  // if keep_clear_stop_count > 0, give invalid cost,
  if (start_v > max_adc_stop_speed && keep_clear_stop_count > 0) {
    cost_result->low_speed_cost = std::numeric_limits<double>::infinity();
  }

  // if has_over_acc, give a big cost
  if (has_over_acce) {
    cost_result->low_speed_cost += config_.over_acce_cost();
  }
  return true;
}

}  // namespace planning
}  // namespace TL
