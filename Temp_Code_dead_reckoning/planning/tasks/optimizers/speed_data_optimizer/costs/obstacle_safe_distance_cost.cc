/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file obstacle_safe_distance_cost.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/costs/obstacle_safe_distance_cost.h"
#include <math.h>  // NOLINT

#include <algorithm>
#include <cmath>
#include <limits>
#include "common/file/log.h"
#include "planning/common/obstacle.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/st_obstacle_cache.h"

namespace TL {
namespace planning {
namespace {
constexpr double kMinSafeDistance = 2.0;
}

ObstacleSafeDistanceCost::ObstacleSafeDistanceCost(
    const SpeedCostConfig& config)
    : SpeedCost(config),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
  if (config.has_obstacle_safe_distance_cost_config()) {
    config_.CopyFrom(config.obstacle_safe_distance_cost_config());
  }

  for (int i = 0; i < static_cast<int>(follow_distance_cost_table_.size());
       ++i) {
    follow_distance_cost_table_.at(i) =
        FollowDistanceCostFunction(i * obstacle_distance_epsilon_);
  }

  for (int i = 0; i < static_cast<int>(overtake_distance_cost_table_.size());
       ++i) {
    overtake_distance_cost_table_.at(i) =
        OvertakeDistanceCostFunction(i * obstacle_distance_epsilon_);
  }

  half_width_ = vehicle_param_.width() / 2.0;
}

double ObstacleSafeDistanceCost::CalculateSSafeDistanceCost(
    const SpeedCurvePoint& point, const SLTObstacleCache& obstacle_cache,
    const STObstacleLocation& obstacle_location, double* min_follow_safe_cost,
    double* min_overtake_safe_cost) const {
  double safe_distance_cost = 0.0;
  const auto* obstacle = obstacle_cache.GetObstacle();
  if (obstacle == nullptr || min_follow_safe_cost == nullptr ||
      min_overtake_safe_cost == nullptr) {
    return safe_distance_cost;
  }
  *min_follow_safe_cost = 0.0;
  *min_overtake_safe_cost = 0.0;
  // calucalate obstacle location
  const auto s_lower = point.s() - vehicle_param_.back_edge_to_center();
  const auto s_upper = point.s() + vehicle_param_.front_edge_to_center();
  const auto& obstacle_info = obstacle_cache.GetObstacleInfoAtTime(point.t());
  const auto cur_obstacle_location =
      (s_upper < obstacle_info.s_lower)
          ? STObstacleLocation::ABOVE
          : ((s_lower > obstacle_info.s_upper) ? STObstacleLocation::BELOW
                                               : STObstacleLocation::CROSS);

  // calucalate safe_distance_cost
  if (obstacle_location == STObstacleLocation::ABOVE ||
      (obstacle_location != STObstacleLocation::BELOW &&
       cur_obstacle_location == STObstacleLocation::ABOVE)) {
    // 1. this means adc must follow obstacle
    // calculate real_distance
    auto real_distance = obstacle_info.s_lower - s_upper;
    // calculate safe_distance
    const auto dv = 1.05 * point.v() - obstacle_info.ds;
    auto safe_distance =
        kMinSafeDistance +
        fmin(point.v() * config_.s_follow_distance_coef_for_speed(),
             config_.max_s_follow_distance_for_speed());
    auto min_safe_distance = safe_distance;
    if (dv > 0) {
      safe_distance += dv * dv;
      min_safe_distance +=
          dv * dv *
          (obstacle_cache.GetIsFront() ? config_.front_obs_safe_dv_coef()
                                       : config_.back_obs_safe_dv_coef());
    } else if (obstacle_cache.GetIsCutIn()) {
      safe_distance =
          fmax(safe_distance + dv * obstacle_cache.GetCutInSafty(), 0.0);
      min_safe_distance = safe_distance;
    }

    // calculate safe_distance_cost
    if (safe_distance < 1e-2 || real_distance > safe_distance) {
      safe_distance_cost = 0.0;
      *min_follow_safe_cost = 0.0;
    } else if (real_distance > 0) {
      safe_distance_cost = GetFollowDistanceCost(
          (safe_distance - real_distance) / safe_distance);
      *min_follow_safe_cost = GetFollowDistanceCost(
          (min_safe_distance - real_distance) / min_safe_distance);
    } else {
      safe_distance_cost = GetFollowDistanceCost(1.0) * (1 - real_distance);
      *min_follow_safe_cost =
          GetFollowDistanceCost(1.0) * (1 - min_safe_distance);
    }

    if (GetPrintDebug()) {
      ADEBUG << "t:" << point.t() << ", id:" << obstacle->Id()
             << ", adc_s_lower:" << s_lower << ", adc_s_upper:" << s_upper
             << ", obs_s_lower:" << obstacle_info.s_lower
             << ", obs_s_upper:" << obstacle_info.s_upper
             << ", real_distance:" << real_distance
             << ", safe_distance:" << safe_distance
             << ", is_cutin:" << obstacle_cache.GetIsCutIn()
             << ", GetCutInSafty:" << obstacle_cache.GetCutInSafty()
             << ", v:" << point.v() << ", obstacle_info.ds:" << obstacle_info.ds
             << ", dv:" << dv;
    }
  } else if (obstacle_location == STObstacleLocation::BELOW ||
             (obstacle_location != STObstacleLocation::ABOVE &&
              cur_obstacle_location == STObstacleLocation::BELOW)) {
    // 2. this means adc must overtake obstacle
    // calculate real_distance
    const auto real_distance = s_lower - obstacle_info.s_upper;
    // calculate safe distance
    const auto dv = 1.05 * obstacle_info.ds - point.v();
    auto safe_distance =
        kMinSafeDistance +
        fmin(point.v() * config_.s_overtake_distance_coef_for_speed(),
             config_.max_s_overtake_distance_for_speed());
    auto min_safe_distance = safe_distance;
    if (dv > 0) {
      safe_distance += dv * dv;
      min_safe_distance +=
          dv * dv *
          (obstacle_cache.GetIsFront() ? config_.front_obs_safe_dv_coef()
                                       : config_.back_obs_safe_dv_coef());
    } else {
      safe_distance = fmax(safe_distance + dv * 6.0, 0.0);
      min_safe_distance = safe_distance;
    }
    // calculate safe_distance_cost
    if (safe_distance < 1e-2 || real_distance > safe_distance) {
      safe_distance_cost = 0.0;
      *min_overtake_safe_cost = 0.0;
    } else if (real_distance > 0) {
      safe_distance_cost = GetOvertakeDistanceCost(
          (safe_distance - real_distance) / safe_distance);
      *min_overtake_safe_cost = GetOvertakeDistanceCost(
          (min_safe_distance - real_distance) / min_safe_distance);
    } else {
      safe_distance_cost = GetOvertakeDistanceCost(1.0) * (1 - real_distance);
      *min_overtake_safe_cost =
          GetOvertakeDistanceCost(1.0) * (1 - min_safe_distance);
    }

    if (GetPrintDebug()) {
      ADEBUG << "t:" << point.t() << ", id:" << obstacle->Id()
             << ", adc_s_lower:" << s_lower << ", adc_s_upper:" << s_upper
             << ", obs_s_lower:" << obstacle_info.s_lower
             << ", obs_s_upper:" << obstacle_info.s_upper
             << ", real_distance:" << real_distance
             << ", safe_distance:" << safe_distance
             << ", is_cutin:" << obstacle_cache.GetIsCutIn()
             << ", GetCutInSafty:" << obstacle_cache.GetCutInSafty()
             << ", v:" << point.v() << ", obstacle_info.ds:" << obstacle_info.ds
             << ", dv:" << dv;
    }
  } else {
    safe_distance_cost = GetFollowDistanceCost(1.0);
    if (GetPrintDebug()) {
      ADEBUG << "t:" << point.t() << ", id:" << obstacle->Id()
             << ", adc_s_lower:" << s_lower << ", adc_s_upper:" << s_upper
             << ", obs_s_lower:" << obstacle_info.s_lower
             << ", obs_s_upper:" << obstacle_info.s_upper
             << ", is_cutin:" << obstacle_cache.GetIsCutIn()
             << ", GetCutInSafty:" << obstacle_cache.GetCutInSafty()
             << ", v:" << point.v()
             << ", obstacle_info.ds:" << obstacle_info.ds;
    }
  }

  safe_distance_cost *= obstacle_cache.GetCostWeight();
  *min_follow_safe_cost *= obstacle_cache.GetCostWeight();
  *min_overtake_safe_cost *= obstacle_cache.GetCostWeight();

  if (GetPrintDebug()) {
    ADEBUG << "s_safe_distance_cost:" << safe_distance_cost;
  }

  return safe_distance_cost;
}

double ObstacleSafeDistanceCost::CalculateLSafeDistanceCost(
    const SpeedCurvePoint& point, const SLTObstacleCache& obstacle_cache,
    SpeedCurveCostResult* cost_result) const {
  double safe_distance_cost = 0.0;
  const auto* obstacle = obstacle_cache.GetObstacle();
  if (obstacle == nullptr) {
    return safe_distance_cost;
  }

  // calucalate obstacle location
  const auto l_lower = -half_width_;
  const auto l_upper = half_width_;
  const auto& obstacle_info = obstacle_cache.GetObstacleInfoAtTime(point.t());

  auto obs_l_lower = obstacle_info.l_lower;
  auto obs_l_upper = obstacle_info.l_upper;
  if (cost_result->nudge_merge_obstacle && obstacle->GetIsMergeObstacle()) {
    obs_l_lower = std::numeric_limits<double>::lowest();
    obs_l_upper = std::numeric_limits<double>::max();
  }

  // calucalate safe_distance_cost
  if (l_upper < obs_l_lower) {
    // 1. this means adc must follow obstacle
    // calculate real_distance
    auto real_distance = obs_l_lower - l_upper;
    // calculate safe_distance
    const auto dv = 0.0 - obstacle_info.dl;
    const auto safe_distance =
        0.3 + fmax(dv, 0.0) * config_.l_follow_distance_coef_for_speed();

    // calculate safe_distance_cost
    if (safe_distance < 1e-2 || real_distance > safe_distance) {
      safe_distance_cost = 0.0;
    } else if (real_distance > 0) {
      safe_distance_cost = GetFollowDistanceCost(
          (safe_distance - real_distance) / safe_distance);
    } else {
      safe_distance_cost = GetFollowDistanceCost(1.0) * (1 - real_distance);
    }

    if (GetPrintDebug()) {
      ADEBUG << "t:" << point.t() << ", id:" << obstacle->Id()
             << ", adc_l_lower:" << l_lower << ", adc_l_upper:" << l_upper
             << ", obs_l_lower:" << obs_l_lower
             << ", obs_l_upper:" << obs_l_upper
             << ", real_distance:" << real_distance
             << ", safe_distance:" << safe_distance
             << ", obstacle_info.dl:" << obstacle_info.dl << ", dv:" << dv;
    }

  } else if (l_lower > obs_l_upper) {
    // 2. this means adc must overtake obstacle
    // calculate real_distance
    const auto real_distance = l_lower - obs_l_upper;
    // calculate safe distance
    const auto dv = 1.05 * obstacle_info.dl - 0.0;
    const auto safe_distance =
        0.3 + fmax(dv, 0.0) * config_.l_overtake_distance_coef_for_speed();

    // calculate safe_distance_cost
    if (safe_distance < 1e-2 || real_distance > safe_distance) {
      safe_distance_cost = 0.0;
    } else if (real_distance > 0) {
      safe_distance_cost = GetOvertakeDistanceCost(
          (safe_distance - real_distance) / safe_distance);
    } else {
      safe_distance_cost = GetOvertakeDistanceCost(1.0) * (1 - real_distance);
    }

    if (GetPrintDebug()) {
      ADEBUG << "t:" << point.t() << ", id:" << obstacle->Id()
             << ", adc_l_lower:" << l_lower << ", adc_l_upper:" << l_upper
             << ", obs_l_lower:" << obs_l_lower
             << ", obs_l_upper:" << obs_l_upper
             << ", real_distance:" << real_distance
             << ", safe_distance:" << safe_distance
             << ", obstacle_info.dl:" << obstacle_info.dl << ", dv:" << dv;
    }

  } else {
    safe_distance_cost = GetFollowDistanceCost(1.0);
  }

  safe_distance_cost *= obstacle_cache.GetCostWeight();

  if (GetPrintDebug()) {
    ADEBUG << "l_safe_distance_cost:" << safe_distance_cost;
  }

  return safe_distance_cost;
}

bool ObstacleSafeDistanceCost::CalculateCost(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->obstacle_safe_distance_cost =
        std::numeric_limits<double>::infinity();
    return false;
  }

  if (cost_result->curve->GetTarget().type == SpeedCurveTarget::Type::SAFE) {
    return CalculateCostForSafeCurve(cache, reference_line_info, cost_result);
  }
  return CalculateCostWithoutDecisionForOtherCurve(cache, reference_line_info,
                                                   cost_result) &&
         CalculateCostWithDecisionForOtherCurve(cache, reference_line_info,
                                                cost_result);
}

bool ObstacleSafeDistanceCost::CalculateCostForSafeCurve(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->obstacle_safe_distance_cost =
        std::numeric_limits<double>::infinity();
    return false;
  }

  const auto& basic_cache = cache.GetBasicCache();
  if (!basic_cache.GetIsForwardPath()) {
    return true;
  }

  const auto& obstacle_caches = cache.GetSafeSLTObstacleCachesWithOutDecision();

  const SLTObstacleCache* target_obstacle_cache = nullptr;
  const auto& obstacle_id = cost_result->curve->GetTarget().obstacle_id;
  for (const auto* obstacle_cache : obstacle_caches) {
    const auto* obstacle = obstacle_cache->GetObstacle();
    if (obstacle != nullptr && obstacle->Id() == obstacle_id) {
      target_obstacle_cache = obstacle_cache;
      break;
    }
  }
  if (target_obstacle_cache == nullptr) {
    return true;
  }

  const auto min_sparse_point_count =
      cost_result->curve->GetMinSparsePointCount();
  const auto& sparse_points = cost_result->curve->GetSparsePoints();

  if (static_cast<int>(cost_result->obstacle_s_safe_distance_costs.size()) <
      min_sparse_point_count) {
    cost_result->obstacle_s_safe_distance_costs.resize(min_sparse_point_count,
                                                       0.0);
  }
  if (static_cast<int>(cost_result->obstacle_l_safe_distance_costs.size()) <
      min_sparse_point_count) {
    cost_result->obstacle_l_safe_distance_costs.resize(min_sparse_point_count,
                                                       0.0);
  }
  cost_result->obstacle_safe_distance_cost = 0.0;

  auto s_safe_distance_cost = 0.0;
  auto l_safe_distance_cost = 0.0;
  auto min_follow_s_safe_cost = 0.0;
  auto min_overtake_s_safe_cost = 0.0;
  for (int i = 1; i < min_sparse_point_count; ++i) {
    const auto& point = sparse_points[i];
    auto t = point.t();
    // check if obstacle is in st graph at time t
    if (t < target_obstacle_cache->GetMinT() ||
        t > target_obstacle_cache->GetMaxT()) {
      continue;
    }

    const auto index = target_obstacle_cache->GetIndex();
    if (index < 0 || index >= cost_result->st_obstacle_locations_.size()) {
      continue;
    }

    s_safe_distance_cost = CalculateSSafeDistanceCost(
        point, *target_obstacle_cache,
        cost_result->st_obstacle_locations_.at(index), &min_follow_s_safe_cost,
        &min_overtake_s_safe_cost);
    l_safe_distance_cost =
        CalculateLSafeDistanceCost(point, *target_obstacle_cache, cost_result);

    cost_result->obstacle_s_safe_distance_costs.at(i) = s_safe_distance_cost;
    cost_result->obstacle_l_safe_distance_costs.at(i) = l_safe_distance_cost;
    const auto st_time_length_coef =
        min_overtake_s_safe_cost > min_follow_s_safe_cost
            ? 1.0
            : target_obstacle_cache->StTimeLengthCoef();
    cost_result->obstacle_safe_distance_cost +=
        config_.obstacle_cost_time_coef(i) *
        fmin(s_safe_distance_cost, l_safe_distance_cost) * st_time_length_coef;
  }
  cost_result->obstacle_safe_distance_cost /= (min_sparse_point_count - 1);
  return true;
}

bool ObstacleSafeDistanceCost::CalculateCostWithoutDecisionForOtherCurve(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->obstacle_safe_distance_cost =
        std::numeric_limits<double>::infinity();
    return false;
  }
  const auto& obstacle_caches = cache.GetSafeSLTObstacleCachesWithOutDecision();
  const auto obstacle_cache_size = obstacle_caches.size();

  const auto* follow_obstacle_cache = cache.GetFollowSLTObstacleCache();
  const auto& curve = cost_result->curve;
  const auto min_sparse_point_count = curve->GetMinSparsePointCount();
  const auto& sparse_points = curve->GetSparsePoints();
  const auto is_forward = cache.GetBasicCache().GetIsForwardPath();
  const auto& vehicle_params =
      TL::common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto min_dec_time =
      reference_line_info.vehicle_state().linear_velocity() /
      fabs(vehicle_params.max_deceleration());
  cost_result->obstacle_safe_distance_cost = 0.0;

  auto s_safe_distance_cost = 0.0;
  auto l_safe_distance_cost = 0.0;
  for (int i = 0; i < min_sparse_point_count; ++i) {
    const auto& point = sparse_points[i];

    const auto t = point.t();
    double max_safe_distance_cost = 0.0;
    double max_non_follow_safe_distance_cost = 0.0;
    double max_front_safe_distance_cost = 0.0;
    double max_front_non_follow_safe_distance_cost = 0.0;
    double min_follow_s_safe_cost = 0.0;
    double min_overtake_s_safe_cost = 0.0;

    for (std::size_t j = 0; j < obstacle_cache_size; ++j) {
      const auto* obstacle_cache = obstacle_caches.at(j);
      if (obstacle_cache == nullptr) {
        break;
      }

      const auto* obstacle = obstacle_cache->GetObstacle();
      if (obstacle == nullptr || obstacle->GetIsCrossObstacle() ||
          (is_forward && !obstacle->IsBelievable() && t > min_dec_time)) {
        break;
      }

      if (t < obstacle_cache->GetMinT() || t > obstacle_cache->GetMaxT()) {
        continue;
      }

      const auto index = obstacle_cache->GetIndex();
      if (index < 0 || index >= cost_result->st_obstacle_locations_.size()) {
        continue;
      }

      s_safe_distance_cost = CalculateSSafeDistanceCost(
          point, *obstacle_cache, cost_result->st_obstacle_locations_.at(index),
          &min_follow_s_safe_cost, &min_overtake_s_safe_cost);
      l_safe_distance_cost =
          CalculateLSafeDistanceCost(point, *obstacle_cache, cost_result);

      const auto& s_safe_distance_costs =
          obstacle_cache->GetSSafeDistanceCosts();
      if (i < static_cast<int>(s_safe_distance_costs.size())) {
        s_safe_distance_cost =
            fmax(s_safe_distance_cost, s_safe_distance_costs.at(i));
      }
      const auto& l_safe_distance_costs =
          obstacle_cache->GetLSafeDistanceCosts();
      if (i < static_cast<int>(l_safe_distance_costs.size())) {
        l_safe_distance_cost =
            fmax(l_safe_distance_cost, l_safe_distance_costs.at(i));
      }
      const auto st_time_length_coef =
          min_overtake_s_safe_cost > min_follow_s_safe_cost
              ? 1.0
              : obstacle_cache->StTimeLengthCoef();
      auto safe_distance_cost =
          fmin(s_safe_distance_cost, l_safe_distance_cost) *
          st_time_length_coef;
      max_safe_distance_cost = fmax(max_safe_distance_cost, safe_distance_cost);

      if (obstacle_cache->GetIsFront()) {
        max_front_safe_distance_cost =
            fmax(max_front_safe_distance_cost, safe_distance_cost);
      }

      if (follow_obstacle_cache != obstacle_cache) {
        max_non_follow_safe_distance_cost =
            fmax(max_non_follow_safe_distance_cost, safe_distance_cost);
        if (obstacle_cache->GetIsFront()) {
          max_front_non_follow_safe_distance_cost =
              fmax(max_front_non_follow_safe_distance_cost, safe_distance_cost);
        }
      }
    }

    cost_result->obstacle_safe_distance_cost +=
        config_.obstacle_cost_time_coef(i) * max_safe_distance_cost;
    cost_result->non_follow_safe_distance_cost +=
        config_.obstacle_cost_time_coef(i) * max_non_follow_safe_distance_cost;
    cost_result->front_obstacle_cost.obstacle_safe_distance_cost +=
        config_.obstacle_cost_time_coef(i) * max_front_safe_distance_cost;
    cost_result->front_obstacle_cost.non_follow_safe_distance_cost +=
        config_.obstacle_cost_time_coef(i) *
        max_front_non_follow_safe_distance_cost;
  }
  cost_result->obstacle_safe_distance_cost /= (min_sparse_point_count - 1);
  cost_result->non_follow_safe_distance_cost /= (min_sparse_point_count - 1);
  cost_result->front_obstacle_cost.obstacle_safe_distance_cost /=
      (min_sparse_point_count - 1);
  cost_result->front_obstacle_cost.non_follow_safe_distance_cost /=
      (min_sparse_point_count - 1);
  return true;
}

bool ObstacleSafeDistanceCost::CalculateCostWithDecisionForOtherCurve(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->obstacle_safe_distance_cost =
        std::numeric_limits<double>::infinity();
    return false;
  }

  const auto& obstacle_caches = cache.GetSafeSLTObstacleCachesWithDecision();
  const auto obstacle_cache_size = obstacle_caches.size();

  if (GetPrintDebug()) {
    ADEBUG << "obstacle_cache_size:" << obstacle_cache_size;
  }

  const auto& curve = cost_result->curve;
  const auto min_sparse_point_count = curve->GetMinSparsePointCount();
  const auto& sparse_points = curve->GetSparsePoints();
  const auto* follow_obstacle_cache = cache.GetFollowSLTObstacleCache();
  double safe_distance_cost = 0.0;

  double obstacle_safe_distance_cost = 0.0;
  double non_follow_safe_distance_cost = 0.0;
  double front_obstacle_safe_distance_cost = 0.0;
  double front_non_follow_safe_distance_cost = 0.0;

  for (int i = 0; i < min_sparse_point_count; ++i) {
    const auto& point = sparse_points[i];
    const auto t = point.t();
    const auto s_lower = point.s() - vehicle_param_.back_edge_to_center();
    const auto s_upper = point.s() + vehicle_param_.front_edge_to_center();

    double max_safe_distance_cost = 0.0;
    double max_non_follow_safe_distance_cost = 0.0;
    double max_front_safe_distance_cost = 0.0;
    double max_front_non_follow_safe_distance_cost = 0.0;

    for (std::size_t j = 0; j < obstacle_cache_size; ++j) {
      const auto* obstacle_cache = obstacle_caches.at(j);
      if (t < obstacle_cache->GetMinT() || t > obstacle_cache->GetMaxT()) {
        continue;
      }

      const auto& obstacle_info =
          obstacle_cache->GetObstacleInfoAtTime(point.t());

      const auto& LongitudinalIntention =
          obstacle_cache->GetObstacle()->GetLongitudinalIntention();
      const auto& lat_intention =
          obstacle_cache->GetObstacle()->GetLateralIntention();
      if (LongitudinalIntention == LongitudinalIntention::YIELD) {
        // 1. this means adc must follow obstacle
        // calculate real_distance
        auto real_distance = obstacle_info.s_lower - s_upper;
        // calculate safe_distance
        const auto dv = 1.05 * point.v() - obstacle_info.ds;
        auto safe_distance =
            obstacle_cache->GetMinStopDistance() +
            fmin(point.v() * config_.s_follow_distance_coef_for_speed(),
                 config_.max_s_follow_distance_for_speed());
        if (dv > 0) {
          safe_distance += dv * dv;
        }

        // calculate safe_distance_cost
        if (safe_distance < 1e-2 || real_distance > safe_distance) {
          safe_distance_cost = 0.0;
        } else if (real_distance > 0) {
          safe_distance_cost =
              GetFollowDistanceCost((safe_distance - real_distance) /
                                    safe_distance) *
              obstacle_cache->StTimeLengthCoef();
        } else {
          safe_distance_cost = GetFollowDistanceCost(1.0) *
                               (1 - real_distance) *
                               obstacle_cache->StTimeLengthCoef();
        }

        if (lat_intention == LateralIntention::MERGE) {
          // calucalate obstacle location
          const auto l_lower = -half_width_;
          const auto l_upper = half_width_;
          auto l_distance = 0.0;
          // calucalate safe_distance_cost
          if (l_upper < obstacle_info.l_lower) {
            l_distance = obstacle_info.l_lower - l_upper;
          } else if (l_lower > obstacle_info.l_upper) {
            l_distance = l_lower - obstacle_info.l_upper;
          }
          safe_distance_cost *= pow(fmax(1.2 - l_distance, 0.0), 4);
        }

        if (GetPrintDebug()) {
          ADEBUG << "t:" << point.t()
                 << ", id:" << obstacle_cache->GetObstacle()->Id()
                 << ", adc_s_lower:" << s_lower << ", adc_s_upper:" << s_upper
                 << ", obs_s_lower:" << obstacle_info.s_lower
                 << ", obs_s_upper:" << obstacle_info.s_upper
                 << ", real_distance:" << real_distance
                 << ", safe_distance:" << safe_distance
                 << ", is_cutin:" << obstacle_cache->GetIsCutIn()
                 << ", GetCutInSafty:" << obstacle_cache->GetCutInSafty()
                 << ", v:" << point.v()
                 << ", obstacle_info.ds:" << obstacle_info.ds << ", dv:" << dv;
        }
      } else if (LongitudinalIntention == LongitudinalIntention::OVERTAKE) {
        // 2. this means adc must overtake obstacle
        // calculate real_distance
        const auto real_distance = s_lower - obstacle_info.s_upper;
        // calculate safe distance
        const auto dv = 1.05 * obstacle_info.ds - point.v();
        auto safe_distance =
            obstacle_cache->GetMinStopDistance() +
            fmin(point.v() * config_.s_overtake_distance_coef_for_speed(),
                 config_.max_s_overtake_distance_for_speed());
        if (dv > 0) {
          safe_distance += dv * dv;
        }
        // calculate safe_distance_cost
        if (safe_distance < 1e-2 || real_distance > safe_distance) {
          safe_distance_cost = 0.0;
        } else if (real_distance > 0) {
          safe_distance_cost = GetOvertakeDistanceCost(
              (safe_distance - real_distance) / safe_distance);
        } else {
          safe_distance_cost =
              GetOvertakeDistanceCost(1.0) * (1 - real_distance);
        }

        if (GetPrintDebug()) {
          ADEBUG << "t:" << point.t()
                 << ", id:" << obstacle_cache->GetObstacle()->Id()
                 << ", adc_s_lower:" << s_lower << ", adc_s_upper:" << s_upper
                 << ", obs_s_lower:" << obstacle_info.s_lower
                 << ", obs_s_upper:" << obstacle_info.s_upper
                 << ", real_distance:" << real_distance
                 << ", safe_distance:" << safe_distance
                 << ", is_cutin:" << obstacle_cache->GetIsCutIn()
                 << ", GetCutInSafty:" << obstacle_cache->GetCutInSafty()
                 << ", v:" << point.v()
                 << ", obstacle_info.ds:" << obstacle_info.ds << ", dv:" << dv;
        }
      }

      max_safe_distance_cost = fmax(max_safe_distance_cost, safe_distance_cost);
      if (obstacle_cache->GetIsFront()) {
        max_front_safe_distance_cost =
            fmax(max_front_safe_distance_cost, safe_distance_cost);
      }

      if (follow_obstacle_cache != obstacle_cache) {
        max_non_follow_safe_distance_cost =
            fmax(max_non_follow_safe_distance_cost, safe_distance_cost);
        if (obstacle_cache->GetIsFront()) {
          max_front_non_follow_safe_distance_cost =
              fmax(max_front_non_follow_safe_distance_cost, safe_distance_cost);
        }
      }
    }

    obstacle_safe_distance_cost +=
        config_.obstacle_cost_time_coef(i) * max_safe_distance_cost;
    non_follow_safe_distance_cost +=
        config_.obstacle_cost_time_coef(i) * max_non_follow_safe_distance_cost;
    front_obstacle_safe_distance_cost +=
        config_.obstacle_cost_time_coef(i) * max_front_safe_distance_cost;
    front_non_follow_safe_distance_cost +=
        config_.obstacle_cost_time_coef(i) *
        max_front_non_follow_safe_distance_cost;
  }
  obstacle_safe_distance_cost /= (min_sparse_point_count - 1);
  non_follow_safe_distance_cost /= (min_sparse_point_count - 1);
  front_obstacle_safe_distance_cost /= (min_sparse_point_count - 1);
  front_non_follow_safe_distance_cost /= (min_sparse_point_count - 1);

  cost_result->obstacle_safe_distance_cost += obstacle_safe_distance_cost;
  cost_result->non_follow_safe_distance_cost += non_follow_safe_distance_cost;
  cost_result->front_obstacle_cost.obstacle_safe_distance_cost +=
      front_obstacle_safe_distance_cost;
  cost_result->front_obstacle_cost.non_follow_safe_distance_cost +=
      front_non_follow_safe_distance_cost;

  return true;
}

bool ObstacleSafeDistanceCost::CheckIsLongitudinalSafe(
    const SpeedData& speed_data, const SLTObstacleCache& obstacle_cache) const {
  STObstacleLocation st_obstacle_location = STObstacleLocation::UNKNOWN;

  const auto* obstacle = obstacle_cache.GetObstacle();
  if (obstacle == nullptr) {
    return true;
  }

  for (const auto& point : speed_data) {
    auto t = point.t();
    if (t < obstacle_cache.GetMinT() || t > obstacle_cache.GetMaxT()) {
      continue;
    }

    const auto s = point.s();
    const auto v = point.v();

    // calucalate obstacle location
    const auto s_lower = s - vehicle_param_.back_edge_to_center();
    const auto s_upper = s + vehicle_param_.front_edge_to_center();
    const auto& obstacle_info = obstacle_cache.GetObstacleInfoAtTime(t);
    st_obstacle_location =
        (s_upper < obstacle_info.s_lower)
            ? STObstacleLocation::ABOVE
            : ((s_lower > obstacle_info.s_upper) ? STObstacleLocation::BELOW
                                                 : STObstacleLocation::CROSS);

    // calucalate safe_distance_cost
    if (st_obstacle_location == STObstacleLocation::ABOVE) {
      // 1. this means adc must follow obstacle
      // calculate real_distance
      auto real_distance = obstacle_info.s_lower - s_upper;
      // calculate safe_distance
      const auto dv = 1.05 * v - obstacle_info.ds;
      auto safe_distance = kMinSafeDistance +
                           fmin(v * config_.s_follow_distance_coef_for_speed(),
                                config_.max_s_follow_distance_for_speed());
      if (dv > 0) {
        safe_distance += dv * dv;
      }
      if (real_distance < safe_distance) {
        return false;
      }
    } else if (st_obstacle_location == STObstacleLocation::BELOW) {
      // 2. this means adc must overtake obstacle
      // calculate real_distance
      const auto real_distance = s_lower - obstacle_info.s_upper;
      // calculate safe distance
      const auto dv = 1.05 * obstacle_info.ds - v;
      auto safe_distance =
          kMinSafeDistance +
          fmin(v * config_.s_overtake_distance_coef_for_speed(),
               config_.max_s_overtake_distance_for_speed());
      if (dv > 0) {
        safe_distance += dv * dv;
      }
      if (real_distance < safe_distance) {
        return false;
      }
    }
  }
  return true;
}

double ObstacleSafeDistanceCost::FollowDistanceCostFunction(
    double det_distance) const {
  return config_.obstacle_distance_cost_coef() * pow(det_distance, 2) *
         exp(6.0 * (det_distance - 0.1));
}

double ObstacleSafeDistanceCost::OvertakeDistanceCostFunction(
    double det_distance) const {
  return config_.obstacle_distance_cost_coef() * pow(det_distance, 2) *
         exp(2.0 * (det_distance - 0.1));
}

}  // namespace planning
}  // namespace TL
