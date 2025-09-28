
/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file highway_speed_curve_evaluator.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/lane_change_ongoing_evaluator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>

#include "common/file/log.h"
#include "common/math/double_type.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_curve.h"
#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreater;
using common::math::double_type::DefinitelyLessEqual;

LaneChangeOngoingEvaluator::LaneChangeOngoingEvaluator(
    const SpeedEvaluatorConfig& config)
    : SpeedEvaluator(config),
      config_(config.lane_change_ongoing_evaluator_config()) {
  config_.set_max_speed_limit_when_nudge_obstacle(
      TL::common::math::ConvertDisplaySpdToReal(
          config_.max_speed_limit_when_nudge_obstacle()));
}

void LaneChangeOngoingEvaluator::CalculateCost(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) const {
  // reset cost
  if (!cost_result->already_calculated) {
    ResetCost(cost_result);
  }

  // check whether cost_result is nullptr, whether curve is nullptr
  if (cost_result == nullptr || cost_result->curve == nullptr) {
    return;
  }

  if (!cost_result->already_calculated &&
      (cost_result->curve->DiscretizeSparsePoint() <= 0 ||
       cost_result->curve->DiscretizeDensePoint() <= 0)) {
    return;
  }

  for (const auto& speed_cost : GetSpeedCosts()) {
    if (cost_result->already_calculated && !speed_cost->Updatable()) {
      continue;
    }
    if (!speed_cost->CalculateCost(cache, reference_line_info, cost_result)) {
      return;
    }
  }

  cost_result->mode_cost = 0.0;
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
  cost_result->already_calculated = true;
}

bool LaneChangeOngoingEvaluator::CheckFallbackSpeedData(
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

void LaneChangeOngoingEvaluator::CalculateModeCost(
    const SpeedCurveCostResult* cost_result1,
    const SpeedCurveCostResult* cost_result2, double* mode_cost1,
    double* mode_cost2) {
  if (cost_result1 == nullptr || cost_result1->curve == nullptr ||
      cost_result2 == nullptr || cost_result2->curve == nullptr ||
      mode_cost1 == nullptr || mode_cost2 == nullptr) {
    return;
  }

  const auto& target1 = cost_result1->curve->GetTarget();
  const auto& target2 = cost_result2->curve->GetTarget();

  *mode_cost1 =
      std::isinf(cost_result1->total_cost) ? cost_result1->total_cost : 0.0;
  *mode_cost2 =
      std::isinf(cost_result2->total_cost) ? cost_result2->total_cost : 0.0;

  if (target1.mode == SpeedCurveTarget::Mode::FOLLOW ||
      target2.mode == SpeedCurveTarget::Mode::FOLLOW) {
    *mode_cost1 += cost_result1->non_follow_safe_distance_cost;
    *mode_cost2 += cost_result2->non_follow_safe_distance_cost;
  } else {
    *mode_cost1 += cost_result1->obstacle_safe_distance_cost;
    *mode_cost2 += cost_result2->obstacle_safe_distance_cost;
  }
  *mode_cost1 +=
      ((cost_result1->over_curvature_speed_limit_cost +
        cost_result1->over_decision_speed_limit_cost) *
           1e2 +
       cost_result1->gap_cost + cost_result1->curve_priority_cost +
       cost_result1->yield_cost + cost_result1->lateral_distance_cost +
       cost_result1->collision_cost + cost_result1->parallel_drive_cost * 1e2 +
       cost_result1->over_speed_cost);
  *mode_cost2 +=
      ((cost_result2->over_curvature_speed_limit_cost +
        cost_result2->over_decision_speed_limit_cost) *
           1e2 +
       cost_result2->gap_cost + cost_result2->curve_priority_cost +
       cost_result2->yield_cost + cost_result2->lateral_distance_cost +
       cost_result2->collision_cost + cost_result2->parallel_drive_cost * 1e2 +
       cost_result2->over_speed_cost);
}

void LaneChangeOngoingEvaluator::CalculateTotalCost(
    const SpeedCurveCostResult* cost_result1,
    const SpeedCurveCostResult* cost_result2, double* total_cost1,
    double* total_cost2) const {
  if (std::isinf(cost_result1->total_cost)) {
    *total_cost1 = cost_result1->total_cost;
  } else {
    *total_cost1 = cost_result1->obstacle_safe_distance_cost +
                   cost_result1->obstacle_less_expected_distance_cost +
                   cost_result1->gap_cost + cost_result1->accel_cost +
                   cost_result1->jerk_cost + cost_result1->stop_distance_cost +
                   cost_result1->yield_cost +
                   cost_result1->lateral_distance_cost +
                   cost_result1->collision_cost;
  }
  if (std::isinf(cost_result2->total_cost)) {
    *total_cost2 = cost_result2->total_cost;
  } else {
    *total_cost2 = cost_result2->obstacle_safe_distance_cost +
                   cost_result2->obstacle_less_expected_distance_cost +
                   cost_result2->gap_cost + cost_result2->accel_cost +
                   cost_result2->jerk_cost + cost_result2->stop_distance_cost +
                   cost_result2->yield_cost +
                   cost_result2->lateral_distance_cost +
                   cost_result2->collision_cost;
  }

  if ((cost_result1->curve->GetTarget().mode !=
           SpeedCurveTarget::Mode::FOLLOW ||
       cost_result2->curve->GetTarget().mode !=
           SpeedCurveTarget::Mode::FOLLOW) &&
      DefinitelyLessEqual(cost_result1->over_speed_cost, 0.0) &&
      DefinitelyLessEqual(cost_result2->over_speed_cost, 0.0)) {
    *total_cost1 += cost_result1->low_speed_cost;
    *total_cost2 += cost_result2->low_speed_cost;
  }
  if (DefinitelyLessEqual(cost_result1->over_speed_cost, 0.0) &&
      DefinitelyLessEqual(cost_result2->over_speed_cost, 0.0)) {
    *total_cost1 += cost_result1->obstacle_greater_expected_distance_cost;
    *total_cost2 += cost_result2->obstacle_greater_expected_distance_cost;
  }

  if (DefinitelyLessEqual(cost_result1->curve->GetMaxV(),
                          config_.max_speed_limit_when_nudge_obstacle()) &&
      DefinitelyLessEqual(cost_result2->curve->GetMaxV(),
                          config_.max_speed_limit_when_nudge_obstacle()) &&
      DefinitelyLessEqual(
          cost_result1->max_over_cruise_speed_limit_ratio,
          config_.max_over_cruise_speed_limit_ratio_when_nudge_obstacle()) &&
      DefinitelyLessEqual(
          cost_result2->max_over_cruise_speed_limit_ratio,
          config_.max_over_cruise_speed_limit_ratio_when_nudge_obstacle()) &&
      DefinitelyLessEqual(
          cost_result1->max_over_map_speed_limit_ratio,
          config_.max_over_map_speed_limit_ratio_when_nudge_obstacle()) &&
      DefinitelyLessEqual(
          cost_result2->max_over_map_speed_limit_ratio,
          config_.max_over_map_speed_limit_ratio_when_nudge_obstacle())) {
    *total_cost1 += cost_result1->parallel_drive_cost;
    *total_cost2 += cost_result2->parallel_drive_cost;
    if (DefinitelyGreater(cost_result1->parallel_drive_cost, 0.0) ||
        DefinitelyGreater(cost_result2->parallel_drive_cost, 0.0)) {
      *total_cost1 += cost_result1->safe_over_speed_cost;
      *total_cost2 += cost_result2->safe_over_speed_cost;
      return;
    }
  }
  *total_cost1 += cost_result1->over_speed_cost;
  *total_cost2 += cost_result2->over_speed_cost;
}

bool LaneChangeOngoingEvaluator::CompareCostResult(
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

bool LaneChangeOngoingEvaluator::CompareCurrentCostResult(
    const SpeedCurveCostResult* cost_result1,
    const SpeedCurveCostResult* cost_result2) const {
  if (cost_result1 == nullptr || cost_result1->curve == nullptr) {
    return false;
  }

  if (cost_result2 == nullptr || cost_result2->curve == nullptr) {
    return true;
  }

  if (cost_result1->curve->GetTarget().mode !=
      cost_result2->curve->GetTarget().mode) {
    double mode_cost1 = 0.0;
    double mode_cost2 = 0.0;
    CalculateModeCost(cost_result1, cost_result2, &mode_cost1, &mode_cost2);
    return mode_cost1 < mode_cost2;
  }

  auto total_cost1 = 0.0;
  auto total_cost2 = 0.0;
  CalculateTotalCost(cost_result1, cost_result2, &total_cost1, &total_cost2);
  return total_cost1 < total_cost2;
}

bool LaneChangeOngoingEvaluator::CompareCurrentAndLastCostResult(
    const SpeedCurveCostResult* current_cost_result,
    const SpeedCurveCostResult* last_cost_result) const {
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
    double current_mode_cost = 0.0;
    double last_mode_cost = 0.0;
    CalculateModeCost(current_cost_result, last_cost_result, &current_mode_cost,
                      &last_mode_cost);
    return current_mode_cost + 1e5 < last_mode_cost;
  }

  // if (last_cost_result->mean_follow_time_error < 0.12) {
  //   return true;
  // }

  auto last_total_cost = 0.0;
  auto current_total_cost = 0.0;
  CalculateTotalCost(current_cost_result, last_cost_result, &current_total_cost,
                     &last_total_cost);
  current_total_cost -= current_cost_result->accel_cost;
  current_total_cost -= current_cost_result->jerk_cost;
  last_total_cost -= last_cost_result->accel_cost;
  last_total_cost -= last_cost_result->jerk_cost;

  ADEBUG << "current curve mode is same as last curve";
  auto threshold = 1e5;
  if (current_curve->GetTarget().mode == SpeedCurveTarget::Mode::CRUISE) {
    threshold = 1e1;
  } else if (current_curve->GetTarget().mode ==
             SpeedCurveTarget::Mode::FOLLOW) {
    threshold = 1e4;
  }
  if (current_total_cost + threshold < last_total_cost) {
    ADEBUG << "current curve has samller total_cost, use current";
    return true;
  }

  if (current_curve->GetTarget().mode == SpeedCurveTarget::Mode::UNKNOWN &&
      current_total_cost < last_total_cost) {
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

  ADEBUG << "use last";
  return false;
}

}  // namespace planning
}  // namespace TL
