/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file reverse_speed_curve_evaluator.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/reverse_gear_evaluator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>

#include "common/math/double_type.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost_factory.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreater;

ReverseGearEvaluator::ReverseGearEvaluator(const SpeedEvaluatorConfig& config)
    : SpeedEvaluator(config), config_(config.reverse_gear_evaluator_config()) {}

void ReverseGearEvaluator::CalculateCost(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) const {
  // reset cost
  if (!cost_result->already_calculated) {
    ResetCost(cost_result);
  }

  // check whether curve is nullptr
  if (cost_result == nullptr || cost_result->curve == nullptr) {
    return;
  }

  if (!cost_result->already_calculated &&
      (cost_result->curve->DiscretizeSparsePoint() <= 0 ||
       cost_result->curve->DiscretizeDensePoint() <= 0)) {
    return;
  }

  for (const auto& speed_cost : speed_costs_) {
    if (cost_result->already_calculated && !speed_cost->Updatable()) {
      continue;
    }
    if (!speed_cost->CalculateCost(cache, reference_line_info, cost_result)) {
      return;
    }
  }

  cost_result->mode_cost =
      cost_result->obstacle_safe_distance_cost +
      cost_result->over_speed_cost * 1e2 + cost_result->gap_cost +
      cost_result->curve_priority_cost + cost_result->parallel_drive_cost * 1e2;
  cost_result->total_cost =
      cost_result->obstacle_safe_distance_cost +
      cost_result->obstacle_less_expected_distance_cost +
      cost_result->obstacle_greater_expected_distance_cost +
      cost_result->gap_cost + cost_result->accel_cost + cost_result->jerk_cost +
      cost_result->stop_distance_cost + cost_result->low_speed_cost +
      cost_result->over_speed_cost + cost_result->parallel_drive_cost;
  cost_result->already_calculated = true;
}

bool ReverseGearEvaluator::CheckFallbackSpeedData(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    const SpeedData& speed_data) const {
  // check if speed_data is empty
  if (speed_data.empty()) {
    return false;
  }

  // check if overspeed
  const auto& basic_cache = cache.GetBasicCache();
  const auto& speed_limit_cache = cache.GetSpeedLimitCache();
  const auto cruise_speed_limit = reference_line_info.GetCruiseSpeed();
  for (const auto& speed_point : speed_data) {
    const auto& speed_limit_info =
        speed_limit_cache.GetPositionSpeedLimit(speed_point.s());
    const auto speed_upper =
        fmin(speed_limit_info.speed_limit, cruise_speed_limit);
    if (speed_point.v() > speed_upper) {
      return false;
    }
  }

  // check if collision happen
  for (const auto* obstacle_cache : cache.GetSafeSTObstacleCaches()) {
    for (const auto& speed_point : speed_data) {
      const auto t = speed_point.t();
      const auto s = speed_point.s();
      const auto v = speed_point.v();

      // check if obstacle is in st graph at time t
      if (t < obstacle_cache->GetMinT() || t > obstacle_cache->GetMaxT()) {
        continue;
      }

      // calucalate s_upper, s_lower, obstacle_v
      const auto& obstacle_info = obstacle_cache->GetObstacleInfoAtTime(t);
      const auto s_upper =
          obstacle_info.s_upper +
          fmax((fmax(obstacle_info.v, 0.0) - v) *
                   config_.collision_check_time_buffer(),
               obstacle_cache->GetCollisionCheckDistanceBuffer());
      const auto s_lower =
          obstacle_info.s_lower -
          fmax((v - fmax(obstacle_info.v, 0.0)) *
                   config_.collision_check_time_buffer(),
               obstacle_cache->GetCollisionCheckDistanceBuffer());

      // when adc has finished lane change, ignore obstacle behind adc
      if (s > basic_cache.GetLowRoadRightEndS() && s_upper < s) {
        break;
      }

      // if s is in [s_lower, s_upper], collision happen
      if (s > s_lower && s < s_upper) {
        return false;
      }
    }
  }

  // for accelerate fallback check if collision happen
  const auto& point = speed_data.back();
  return point.a() > 0.0 ? point.s() < basic_cache.GetExpectedStopS() : true;
}

bool ReverseGearEvaluator::CompareCostResult(
    const SpeedCurveCostResult* cost_result1,
    const SpeedCurveCostResult* cost_result2) const {
  if (cost_result1 == nullptr || cost_result1->curve == nullptr) {
    return false;
  }

  if (cost_result2 == nullptr || cost_result2->curve == nullptr) {
    return true;
  }

  if (cost_result1->curve->LastSelected()) {
    return !CompareCurrentAndLastCostResult(cost_result2, cost_result1);
  }

  if (cost_result2->curve->LastSelected()) {
    return CompareCurrentAndLastCostResult(cost_result1, cost_result2);
  }

  return CompareCurrentCostResult(cost_result1, cost_result2);
}

bool ReverseGearEvaluator::CompareCurrentCostResult(
    const SpeedCurveCostResult* cost_result1,
    const SpeedCurveCostResult* cost_result2) {
  if (cost_result1 == nullptr || cost_result1->curve == nullptr) {
    return false;
  }

  if (cost_result2 == nullptr || cost_result2->curve == nullptr) {
    return true;
  }

  if (cost_result1->curve->GetTarget().mode !=
      cost_result2->curve->GetTarget().mode) {
    return cost_result1->mode_cost < cost_result2->mode_cost;
  }

  auto total_cost1 = cost_result1->total_cost;
  auto total_cost2 = cost_result2->total_cost;

  if (DefinitelyGreater(cost_result1->over_speed_cost, 0.0) ||
      DefinitelyGreater(cost_result2->over_speed_cost, 0.0)) {
    total_cost1 -= cost_result1->obstacle_greater_expected_distance_cost;
    total_cost2 -= cost_result2->obstacle_greater_expected_distance_cost;
  }

  return total_cost1 < total_cost2;
}

bool ReverseGearEvaluator::CompareCurrentAndLastCostResult(
    const SpeedCurveCostResult* current_cost_result,
    const SpeedCurveCostResult* last_cost_result) {
  if (current_cost_result == nullptr || current_cost_result->curve == nullptr) {
    ADEBUG << "current curve is empty, use last";
    return false;
  }

  if (last_cost_result == nullptr || last_cost_result->curve == nullptr) {
    ADEBUG << "last curve is empty, use current";
    return true;
  }

  const auto& current_curve = current_cost_result->curve;
  const auto& last_curve = last_cost_result->curve;

  if (current_curve->GetTarget().mode != last_curve->GetTarget().mode) {
    ADEBUG << "current curve mode is different from last curve, choose one "
              "with samller mode cost";
    return current_cost_result->mode_cost + 1e5 < last_cost_result->mode_cost;
  }

  ADEBUG << "current curve mode is same as last curve";
  if (current_cost_result->total_cost + 1e5 < last_cost_result->total_cost ||
      (current_curve->GetTarget().mode == SpeedCurveTarget::Mode::CRUISE &&
       current_cost_result->total_cost + 1e1 < last_cost_result->total_cost) ||
      (current_curve->GetTarget().mode == SpeedCurveTarget::Mode::FOLLOW &&
       current_cost_result->total_cost + 1e4 < last_cost_result->total_cost)) {
    ADEBUG << "current curve has samller total_cost, use current";
    return true;
  }

  if (current_curve->GetTarget().mode == SpeedCurveTarget::Mode::UNKNOWN &&
      current_cost_result->total_cost < last_cost_result->total_cost) {
    if (current_cost_result->low_speed_cost + 1e4 <
        last_cost_result->low_speed_cost) {
      ADEBUG << "current curve has samller low_speed_cost, use current";
      return true;
    }

    if (current_cost_result->over_speed_cost + 1.0 <
        last_cost_result->over_speed_cost) {
      ADEBUG << "current curve has samller over_speed_cost, use current";
      return true;
    }

    if (current_cost_result->parallel_drive_cost + 1e4 <
        last_cost_result->parallel_drive_cost) {
      ADEBUG << "current curve has samller parallel_drive_cost, use current";
      return true;
    }

    if (current_cost_result->obstacle_greater_expected_distance_cost + 1.0e4 <
        last_cost_result->obstacle_greater_expected_distance_cost) {
      ADEBUG << "current curve has samller "
                "obstacle_greater_expected_distance_cost, use current";
      return true;
    }

    if (current_cost_result->obstacle_less_expected_distance_cost + 1.0e4 <
        last_cost_result->obstacle_less_expected_distance_cost) {
      ADEBUG << "current curve samller obstacle_less_expected_distance_cost "
                "cost, use current";
      return true;
    }
  }

  return false;
}

}  // namespace planning
}  // namespace TL
