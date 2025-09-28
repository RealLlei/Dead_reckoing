/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file yield_cost.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/costs/collision_cost.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include "planning/tasks/optimizers/speed_data_optimizer/caches/st_obstacle_cache.h"
#include "planning/proto/speed_evaluator_config.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {

CollisionCost::CollisionCost(const SpeedCostConfig& config)
    : SpeedCost(config),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
  if (config.has_collision_cost_config()) {
    config_.CopyFrom(config.collision_cost_config());
  }
  half_width_ = vehicle_param_.width() / 2.0;
}

bool CollisionCost::CalculateCost(const SpeedCache& cache,
                                  const ReferenceLineInfo& reference_line_info,
                                  SpeedCurveCostResult* cost_result) const {
  if (cost_result != nullptr) {
    cost_result->st_obstacle_locations_.assign(
        cache.GetSLTObstacleCaches().size(), STObstacleLocation::UNKNOWN);
  }

  CalculateCostWithCrossCheck(cache, reference_line_info, cost_result);

  if (config_.enable_safe_check()) {
    return CalculateCostWithSafeCheck(cache, reference_line_info, cost_result);
  }

  if (config_.enable_continue_check()) {
    return CalculateCostWithContinueCheck(cache, reference_line_info,
                                          cost_result);
  }

  return config_.use_collision_risk()
             ? CalculateCostWithRisk(cache, reference_line_info, cost_result)
             : CalculateCostWithoutRisk(cache, reference_line_info,
                                        cost_result);
}

bool CollisionCost::CalculateCostWithRisk(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->collision_cost = std::numeric_limits<double>::infinity();
    return false;
  }
  const auto is_forward = cache.GetBasicCache().GetIsForwardPath();
  const auto& vehicle_params =
      TL::common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto min_dec_time =
      reference_line_info.vehicle_state().linear_velocity() /
      fabs(vehicle_params.max_deceleration());
  const auto collision_risk_speed_lower_threshold =
      config_.collision_risk_speed_lower_threshold();
  const auto collision_risk_speed_upper_threshold =
      config_.collision_risk_speed_upper_threshold();

  // check collision with dynamic obstacle
  const auto& basic_cache = cache.GetBasicCache();

  const auto& dense_points = cost_result->curve->GetDensePoints();
  const auto min_dense_point_count =
      cost_result->curve->GetMinDensePointCount();
  for (const auto* obstacle_cache : cache.GetSafeSTObstacleCaches()) {
    if (obstacle_cache == nullptr) {
      continue;
    }

    const auto* obstacle = obstacle_cache->GetObstacle();
    if (obstacle == nullptr) {
      continue;
    }

    const auto index = obstacle_cache->GetSLTIndex();
    if (index < 0 || index >= cost_result->st_obstacle_locations_.size()) {
      continue;
    }
    auto& st_obstacle_location = cost_result->st_obstacle_locations_.at(index);

    auto is_front_obstacle = obstacle->PerceptionSLBoundary().end_s() >
                             reference_line_info.AdcSlBoundary().end_s();
    const auto ignore_collision =
        !is_front_obstacle && !config_.consider_back_obstacle();

    for (int j = 1; j < min_dense_point_count; ++j) {
      const auto t = dense_points[j].t();
      const auto s = dense_points[j].s();
      const auto v = dense_points[j].v();

      // check if obstacle is in st graph at time t
      if (t < obstacle_cache->GetMinT() || t > obstacle_cache->GetMaxT()) {
        continue;
      }
      if (is_forward && !obstacle->IsBelievable() && t > min_dec_time) {
        break;
      }

      // calucalate s_upper, s_lower, obstacle_v
      const auto& obstacle_info = obstacle_cache->GetObstacleInfoAtTime(t);

      if (st_obstacle_location == STObstacleLocation::UNKNOWN) {
        if (s < obstacle_info.s_lower) {
          st_obstacle_location = STObstacleLocation::ABOVE;
        } else if (s > obstacle_info.s_upper) {
          st_obstacle_location = STObstacleLocation::BELOW;
        } else {
          st_obstacle_location = STObstacleLocation::CROSS;
        }
      }

      if (!obstacle_cache->GetEnableCollisionCheck()) {
        break;
      }

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

      // when adc has leave low road right area, ignore obstacle behind adc
      if (ignore_collision ||
          (s > basic_cache.GetLowRoadRightEndS() && s_upper < s)) {
        break;
      }

      if (s < s_lower) {
        is_front_obstacle = true;
      } else if (s > s_upper) {
        is_front_obstacle = false;
      } else if (is_front_obstacle &&
                 v + collision_risk_speed_lower_threshold < obstacle_info.v) {
        cost_result->collision_cost +=
            config_.collision_risk_base_cost() +
            fmax(v + collision_risk_speed_upper_threshold - obstacle_info.v,
                 0.0) *
                config_.collision_risk_cost_coef();
        break;
      } else {
        cost_result->collision_cost = std::numeric_limits<double>::infinity();
        if (obstacle_cache->GetIsFront()) {
          cost_result->front_obstacle_cost.collision_cost =
              std::numeric_limits<double>::infinity();
        }

        if (std::isinf(cost_result->collision_cost) &&
            std::isinf(cost_result->front_obstacle_cost.collision_cost)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool CollisionCost::CalculateCostWithoutRisk(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->collision_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  // check collision with dynamic obstacle
  const auto& basic_cache = cache.GetBasicCache();

  const auto& dense_points = cost_result->curve->GetDensePoints();
  const auto min_dense_point_count =
      cost_result->curve->GetMinDensePointCount();
  for (const auto* obstacle_cache : cache.GetSafeSTObstacleCaches()) {
    const auto* obstacle = obstacle_cache->GetObstacle();
    if (obstacle == nullptr) {
      continue;
    }

    for (int j = 1; j < min_dense_point_count; ++j) {
      const auto t = dense_points[j].t();
      const auto s = dense_points[j].s();
      const auto v = dense_points[j].v();

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
      // when adc has leave low road right area, ignore obstacle behind adc
      if (s > basic_cache.GetLowRoadRightEndS() && s_upper < s) {
        break;
      }

      if (s > s_lower && s < s_upper) {
        cost_result->collision_cost = std::numeric_limits<double>::infinity();
        return false;
      }
    }
  }
  return true;
}

bool CollisionCost::CalculateCostWithSafeCheck(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->collision_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  const auto& basic_cache = cache.GetBasicCache();
  const auto& obstacle_caches = cache.GetSafeSLTObstacleCachesWithOutDecision();
  const auto& curve = cost_result->curve;
  const auto min_dense_point_count = curve->GetMinDensePointCount();
  const auto& dense_points = curve->GetDensePoints();
  if (dense_points.empty()) {
    return false;
  }
  for (const auto* obstacle_cache : obstacle_caches) {
    if (obstacle_cache == nullptr) {
      continue;
    }

    const auto min_obs_l = obstacle_cache->GetMinL();
    const auto max_obs_l = obstacle_cache->GetMaxL();

    const auto index = obstacle_cache->GetIndex();
    if (index < 0 || index >= cost_result->st_obstacle_locations_.size()) {
      continue;
    }
    auto& st_obstacle_location = cost_result->st_obstacle_locations_.at(index);
    const auto& obstacle_start_v =
        obstacle_cache->GetObstacleInfoAtTime(dense_points.at(0).t()).ds;
    const auto init_v = dense_points.at(0).v();
    const auto back_obs_s_buffer =
        init_v > obstacle_start_v
            ? fmax(2.0,
                   vehicle_param_.length() + (obstacle_start_v - init_v) * 0.8)
            : fmax(2.0, obstacle_start_v * 0.2);
    const auto fixed_s_buffer =
        obstacle_cache->GetIsFront() ? 2.0 : back_obs_s_buffer;
    for (int j = 0; j < min_dense_point_count; ++j) {
      const auto& point = dense_points[j];
      if (point.t() < obstacle_cache->GetMinT() ||
          point.t() > obstacle_cache->GetMaxT()) {
        continue;
      }

      const auto& obstacle_info =
          obstacle_cache->GetObstacleInfoAtTime(point.t());
      const auto fixed_l_buffer = 0.3;
      auto s_lower =
          point.s() - vehicle_param_.back_edge_to_center() - fixed_s_buffer;
      auto s_upper =
          point.s() + vehicle_param_.front_edge_to_center() + fixed_s_buffer;
      auto l_lower = -half_width_ - fixed_l_buffer;
      auto l_upper = half_width_ + fixed_l_buffer;

      if (point.s() > basic_cache.GetLowRoadRightEndS() &&
          s_lower > obstacle_info.s_upper) {
        if (GetPrintDebug()) {
          AERROR << "id:" << obstacle_cache->GetId() << ", s:" << point.s()
                 << ", GetLowRoadRightEndS:"
                 << basic_cache.GetLowRoadRightEndS();
        }
        break;
      }

      if (st_obstacle_location == STObstacleLocation::UNKNOWN &&
          l_lower < obstacle_info.l_upper && l_upper > obstacle_info.l_lower) {
        if (s_upper < obstacle_info.s_lower) {
          st_obstacle_location = STObstacleLocation::ABOVE;
        } else if (s_lower > obstacle_info.s_upper) {
          st_obstacle_location = STObstacleLocation::BELOW;
        } else {
          st_obstacle_location = STObstacleLocation::CROSS;
        }
      }

      if (point.v() > obstacle_info.ds) {
        s_upper += (point.v() - obstacle_info.ds) * 0.8;
      } else if (point.v() < obstacle_info.ds) {
        s_lower -= (obstacle_info.ds - point.v()) * 0.8;
      }

      if (0.0 > obstacle_info.dl) {
        l_upper +=
            fmin(obstacle_info.l_lower - min_obs_l, -obstacle_info.dl * 1.0);
      } else if (0.0 < obstacle_info.dl) {
        l_lower -=
            fmin(max_obs_l - obstacle_info.l_upper, obstacle_info.dl * 1.0);
      }

      if (GetPrintDebug()) {
        AERROR << "id:" << obstacle_cache->GetId() << ", t:" << point.t()
               << ", s_lower:" << s_lower << ", s_upper:" << s_upper
               << ", l_lower:" << l_lower << ", l_upper:" << l_upper
               << ",  obstacle_info.s_lower:" << obstacle_info.s_lower
               << ",  obstacle_info.s_upper:" << obstacle_info.s_upper
               << ",  obstacle_info.l_lower:" << obstacle_info.l_lower
               << ",  obstacle_info.l_upper:" << obstacle_info.l_upper
               << ",  point.v:" << point.v()
               << ",  obstacle_info.ds:" << obstacle_info.ds;
      }

      if (obstacle_cache->GetIsIgnoreCollision()) {
        continue;
      }

      if (!config_.enable_high_road_right_safe_check() &&
          point.s() < basic_cache.GetHighRoadRightEndS() &&
          s_lower > obstacle_info.s_upper) {
        continue;
      }

      if (s_lower < obstacle_info.s_upper && s_upper > obstacle_info.s_lower &&
          l_lower < obstacle_info.l_upper && l_upper > obstacle_info.l_lower) {
        if (GetPrintDebug()) {
          AERROR << "collision happen";
        }
        cost_result->collision_cost = std::numeric_limits<double>::infinity();
        return false;
      }
    }
  }

  return true;
}

bool CollisionCost::CalculateCostWithContinueCheck(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->collision_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  const auto& basic_cache = cache.GetBasicCache();
  const auto& obstacle_caches = cache.GetSafeSLTObstacleCachesWithOutDecision();
  const auto& curve = cost_result->curve;
  const auto min_dense_point_count = curve->GetMinDensePointCount();
  const auto& dense_points = curve->GetDensePoints();
  for (const auto* obstacle_cache : obstacle_caches) {
    if (obstacle_cache == nullptr) {
      continue;
    }

    const auto index = obstacle_cache->GetIndex();
    if (index < 0 || index >= cost_result->st_obstacle_locations_.size()) {
      continue;
    }
    auto& st_obstacle_location = cost_result->st_obstacle_locations_.at(index);

    for (int j = 0; j < min_dense_point_count; ++j) {
      const auto& point = dense_points[j];
      if (point.t() < obstacle_cache->GetMinT() ||
          point.t() > obstacle_cache->GetMaxT()) {
        continue;
      }

      const auto& obstacle_info =
          obstacle_cache->GetObstacleInfoAtTime(point.t());
      auto s_lower = point.s() - vehicle_param_.back_edge_to_center();
      auto s_upper = point.s() + vehicle_param_.front_edge_to_center();
      auto l_lower = -half_width_;
      auto l_upper = half_width_;

      if (point.s() > basic_cache.GetLowRoadRightEndS() &&
          s_lower > obstacle_info.s_upper) {
        if (GetPrintDebug()) {
          ADEBUG << "id:" << obstacle_cache->GetId() << ", s:" << point.s()
                 << ", GetLowRoadRightEndS:"
                 << basic_cache.GetLowRoadRightEndS();
        }
        break;
      }

      if (s_lower < obstacle_info.s_upper && s_upper > obstacle_info.s_lower &&
          l_lower < obstacle_info.l_upper && l_upper > obstacle_info.l_lower) {
        if (GetPrintDebug()) {
          ADEBUG << "collision happen";
        }
        cost_result->collision_cost = std::numeric_limits<double>::infinity();
        return false;
      }

      if (st_obstacle_location == STObstacleLocation::UNKNOWN &&
          l_lower < obstacle_info.l_upper && l_upper > obstacle_info.l_lower) {
        if (s_upper < obstacle_info.s_lower) {
          st_obstacle_location = STObstacleLocation::ABOVE;
        } else if (s_lower > obstacle_info.s_upper) {
          st_obstacle_location = STObstacleLocation::BELOW;
        } else {
          st_obstacle_location = STObstacleLocation::CROSS;
        }
      }

      const auto* obstacle = obstacle_cache->GetObstacle();
      if (cost_result->nudge_merge_obstacle || obstacle == nullptr ||
          !obstacle->GetIsMergeObstacle() || s_lower > obstacle_info.s_upper) {
        continue;
      }

      // if (s_lower < obstacle_info.s_upper && s_upper > obstacle_info.s_lower &&
      //     point.v() - obstacle_info.ds < config_.continue_speed_threshold()) {
      //   cost_result->nudge_merge_obstacle = true;
      //   continue;
      // }

      if (s_upper < obstacle_info.s_lower && point.v() > obstacle_info.ds &&
          (obstacle_info.s_lower - s_upper) / (point.v() - obstacle_info.ds) >
              config_.continue_ttc_threshold()) {
        cost_result->nudge_merge_obstacle = true;
        continue;
      }
    }
  }

  return true;
}

bool CollisionCost::CalculateCostWithCrossCheck(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->collision_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  const auto& obstacle_caches = cache.GetCrossSLTObstacleCaches();
  const auto& curve = cost_result->curve;
  const auto min_dense_point_count = curve->GetMinDensePointCount();
  const auto& dense_points = curve->GetDensePoints();
  for (const auto* obstacle_cache : obstacle_caches) {
    if (obstacle_cache == nullptr || obstacle_cache->GetObstacle() == nullptr) {
      continue;
    }

    bool s_collision = false;
    auto s_collision_start_time = std::numeric_limits<double>::max();
    auto s_collision_end_time = std::numeric_limits<double>::lowest();
    bool l_collision = false;
    auto l_collision_start_time = std::numeric_limits<double>::max();
    auto l_collision_end_time = std::numeric_limits<double>::lowest();

    for (int j = 0; j < min_dense_point_count; ++j) {
      const auto& point = dense_points[j];
      if (point.t() < obstacle_cache->GetMinT() ||
          point.t() > obstacle_cache->GetMaxT()) {
        continue;
      }

      const auto& obstacle_info =
          obstacle_cache->GetObstacleInfoAtTime(point.t());

      const auto fixed_s_buffer = 1.0;
      const auto fixed_l_buffer = 0.5;
      auto s_lower =
          point.s() - vehicle_param_.back_edge_to_center() - fixed_s_buffer;
      auto s_upper =
          point.s() + vehicle_param_.front_edge_to_center() + fixed_s_buffer;
      auto l_lower = -half_width_ - fixed_l_buffer;
      auto l_upper = half_width_ + fixed_l_buffer;

      if (!s_collision && s_lower < obstacle_info.s_upper &&
          s_upper > obstacle_info.s_lower) {
        s_collision = true;
        s_collision_start_time = point.t();
      } else if (s_collision && s_collision_end_time < 0.0 &&
                 (s_lower > obstacle_info.s_upper ||
                  s_upper < obstacle_info.s_lower)) {
        s_collision_end_time = point.t();
      }

      if (!l_collision && l_lower < obstacle_info.l_upper &&
          l_upper > obstacle_info.l_lower) {
        l_collision = true;
        l_collision_start_time = point.t();
      } else if (l_collision && l_collision_end_time < 0.0 &&
                 (l_lower > obstacle_info.l_upper ||
                  l_upper < obstacle_info.l_lower)) {
        l_collision_end_time = point.t();
      }
    }

    if (!s_collision || !l_collision) {
      continue;
    }

    if (s_collision && s_collision_end_time < 0.0) {
      s_collision_end_time = std::numeric_limits<double>::max();
    }
    if (l_collision && l_collision_end_time < 0.0) {
      l_collision_end_time = std::numeric_limits<double>::max();
    }

    const auto collision_start_time =
        fmax(s_collision_start_time, l_collision_start_time);
    const auto collision_end_time =
        fmin(s_collision_end_time, l_collision_end_time);

    if (GetPrintDebug()) {
      AERROR << "id:" << obstacle_cache->GetId()
             << ", s_collision_start_time:" << s_collision_start_time
             << ", s_collision_end_time:" << s_collision_end_time
             << ", l_collision_start_time:" << l_collision_start_time
             << ", l_collision_end_time:" << l_collision_end_time
             << ", collision_start_time:" << collision_start_time
             << ", collision_end_time:" << collision_end_time;
    }

    if (collision_start_time > collision_end_time) {
      // no collision
      cost_result->collision_cost += fmax(
          0.0, 1e12 - pow(collision_start_time - collision_end_time, 4) * 1e11);
    } else if (collision_start_time > 0.0) {
      // collision
      cost_result->collision_cost +=
          1e12 + pow(fmax(5.0 - collision_start_time, 0.0), 2.0) * 1e11;
    }
  }

  return true;
}

}  // namespace planning
}  // namespace TL
