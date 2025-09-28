/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file curve_priority_cost.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/costs/curve_priority_cost.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include "common/math/double_type.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreater;
using common::math::double_type::SeemsNotEqual;

CurvePriorityCost::CurvePriorityCost(const SpeedCostConfig& config)
    : SpeedCost(config) {
  if (config.has_curve_priority_cost_config()) {
    config_.CopyFrom(config.curve_priority_cost_config());
  }
}

bool CurvePriorityCost::CalculateCost(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) const {
  UNUSED(cache);
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->curve_priority_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  // priority: STOP/STOP_TO_STANDSTILL curve > FOLLOW/CRUISE curve > UNKNOWN curve
  switch (cost_result->curve->GetTarget().mode) {
    case SpeedCurveTarget::Mode::UNKNOWN:
      cost_result->curve_priority_cost = config_.unknown_curve_priority_cost();
      break;
    case SpeedCurveTarget::Mode::FOLLOW:
      cost_result->curve_priority_cost =
          (cache.GetFollowTimes().empty() ||
           SeemsNotEqual(cache.GetFollowTimes().front(),
                         cache.GetFollowTime()) ||
           cost_result->mean_greater_follow_time_error > 0.2 ||
           DefinitelyGreater(cost_result->over_speed_cost, 0.0) ||
           DefinitelyGreater(cost_result->yield_cost, 0.0))
              ? std::numeric_limits<double>::infinity()
              : config_.follow_curve_priority_cost();
      break;
    case SpeedCurveTarget::Mode::CRUISE:
      cost_result->curve_priority_cost = config_.cruise_curve_priority_cost();
      break;
    case SpeedCurveTarget::Mode::STOP:
    case SpeedCurveTarget::Mode::STOP_TO_STANDSTILL:
      cost_result->curve_priority_cost = config_.stop_curve_priority_cost();
      break;
    default:
      break;
  }
  return true;
}

}  // namespace planning
}  // namespace TL
