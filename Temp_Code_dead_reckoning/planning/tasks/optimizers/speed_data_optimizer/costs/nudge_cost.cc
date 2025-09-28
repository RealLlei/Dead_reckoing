/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file nudge_cost.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/costs/nudge_cost.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include "common/util/macros.h"
#include "planning/proto/speed_evaluator_config.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {

NudgeCost::NudgeCost(const SpeedCostConfig& config) : SpeedCost(config) {
  if (config.has_nudge_cost_config()) {
    config_.CopyFrom(config.nudge_cost_config());
  }

  for (int i = 0; i < static_cast<int>(big_car_distance_cost_table_.size());
       ++i) {
    const double distance = i * big_car_distance_epsilon_;
    big_car_distance_cost_table_.at(i) = BigCarDistanceCostFunction(distance);
  }

  for (int i = 0; i < static_cast<int>(obstacle_distance_cost_table_.size());
       ++i) {
    const double obstacle_distance = i * obstacle_distance_epsilon_;
    obstacle_distance_cost_table_.at(i) =
        ObstacleDistanceCostFunction(obstacle_distance);
  }

  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
}

bool NudgeCost::CalculateCost(const SpeedCache& cache,
                              const ReferenceLineInfo& reference_line_info,
                              SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->parallel_drive_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  // do not consider to nudge paralle obstacle until the lane change and lane merge is
  // finished
  if (cache.GetBasicCache().GetHasLowRoadRight() ||
      !cache.GetBasicCache().GetIsForwardPath() ||
      cache.GetFollowSTObstacleCache() != nullptr) {
    return true;
  }

  TL::common::math::Vec2d t_point;
  const auto min_sparse_point_count =
      cost_result->curve->GetMinSparsePointCount();
  const auto& sparse_points = cost_result->curve->GetSparsePoints();

  for (int i = 1; i < min_sparse_point_count; ++i) {
    const auto& point = sparse_points[i];
    const auto t = point.t();
    const auto s = point.s();
    const auto v = point.v();

    // calculate big_car_parallel_drive_cost
    double max_parallel_drive_cost = 0.0;
    for (const auto& nudge_obstacle_cache : cache.GetNudgeObstacleCaches()) {
      if (t < nudge_obstacle_cache.GetMinT() ||
          t > nudge_obstacle_cache.GetMaxT() ||
          nudge_obstacle_cache.GetTargetNudgeState() ==
              SpeedCacheConfig::IGNORE) {
        continue;
      }
      const auto& nudge_obstacle_info =
          nudge_obstacle_cache.GetObstacleInfoAtTime(t);
      const auto& hexagon = nudge_obstacle_cache.GetHexagonAtTime(t);
      if (hexagon == nullptr) {
        continue;
      }

      const auto s_lower = s - vehicle_param_.back_edge_to_center();
      const auto s_upper = s + vehicle_param_.front_edge_to_center();
      auto delta_s = 0.0;
      if (s_lower > nudge_obstacle_info.s_upper) {
        delta_s = s_lower - nudge_obstacle_info.s_upper;
      } else if (s_upper < nudge_obstacle_info.s_lower) {
        delta_s = nudge_obstacle_info.s_lower - s_upper;
      }

      if ((nudge_obstacle_cache.GetTargetNudgeState() ==
               SpeedCacheConfig::OVERTAKE &&
           delta_s < hexagon->max_x() && v < nudge_obstacle_info.ds) ||
          (nudge_obstacle_cache.GetTargetNudgeState() ==
               SpeedCacheConfig::FOLLOW &&
           delta_s < hexagon->max_x() && v > nudge_obstacle_info.ds)) {
        const auto cost = GetBigCarDistanceCost(hexagon->max_y());
        if (cost > max_parallel_drive_cost) {
          max_parallel_drive_cost = cost;
        }
        continue;
      }

      t_point.set_x(delta_s);
      t_point.set_y(v - nudge_obstacle_info.ds);
      if (!hexagon->IsPointIn(t_point)) {
        continue;
      }

      // loop six line of hexagon to find the min dis to these lines.
      double dis_to_hexagon_line = std::numeric_limits<double>::max();
      for (const auto& line : hexagon->line_segments()) {
        dis_to_hexagon_line =
            fmin(line.DistanceTo(t_point), dis_to_hexagon_line);
      }

      // calculate the cost.
      const auto cost = GetBigCarDistanceCost(dis_to_hexagon_line);
      if (cost > max_parallel_drive_cost) {
        max_parallel_drive_cost = cost;
      }
    }
    cost_result->parallel_drive_cost += max_parallel_drive_cost;
  }
  cost_result->parallel_drive_cost /= (min_sparse_point_count - 1);
  return true;
}

double NudgeCost::CalculateFollowTimeCost(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    const double follow_time, const double user_follow_time) const {
  double follow_time_cost =
      fabs(user_follow_time - follow_time) * config_.follow_time_cost_coef();

  // do not consider to nudge parallel obstacle until the lane change and lane merge is
  // finished
  if (reference_line_info.IsChangeLanePath() ||
      !cache.GetBasicCache().GetIsForwardPath() ||
      cache.GetFollowSTObstacleCache() == nullptr) {
    return follow_time_cost;
  }

  double parallel_drive_cost = 0.0;
  const auto& follow_obstacle_cache = cache.GetFollowSTObstacleCache();
  const auto& follow_obstacle_info =
      follow_obstacle_cache->GetObstacleInfoAtTime(
          FLAGS_trajectory_time_length);

  for (const auto& nudge_obstacle_cache : cache.GetNudgeObstacleCaches()) {
    const auto t = FLAGS_trajectory_time_length;
    if (t < nudge_obstacle_cache.GetMinT() ||
        t > nudge_obstacle_cache.GetMaxT()) {
      continue;
    }
    const auto& nudge_obstacle_info =
        nudge_obstacle_cache.GetObstacleInfoAtTime(t);
    const auto& hexagon = nudge_obstacle_cache.GetHexagonAtTime(t);
    if (hexagon == nullptr) {
      continue;
    }

    TL::common::math::Vec2d t_point;
    const auto expected_distance = follow_obstacle_cache->GetMinStopDistance() +
                                   follow_time * follow_obstacle_info.v;
    const auto s = follow_obstacle_info.s_lower - expected_distance;
    const auto s_lower = s - vehicle_param_.back_edge_to_center();
    const auto s_upper = s + vehicle_param_.front_edge_to_center();
    auto delta_s = 0.0;
    if (s_lower > nudge_obstacle_info.s_upper) {
      delta_s = s_lower - nudge_obstacle_info.s_upper;
    } else if (s_upper < nudge_obstacle_info.s_lower) {
      delta_s = nudge_obstacle_info.s_lower - s_upper;
    }
    const auto ds = follow_obstacle_info.v;
    if ((nudge_obstacle_cache.GetTargetNudgeState() ==
             SpeedCacheConfig::OVERTAKE &&
         delta_s < hexagon->max_x() && ds < nudge_obstacle_info.ds) ||
        (nudge_obstacle_cache.GetTargetNudgeState() ==
             SpeedCacheConfig::FOLLOW &&
         delta_s < hexagon->max_x() && ds > nudge_obstacle_info.ds)) {
      parallel_drive_cost =
          fmax(parallel_drive_cost, GetBigCarDistanceCost(hexagon->max_y()));
      continue;
    }

    t_point.set_x(delta_s);
    t_point.set_y(follow_obstacle_info.v - nudge_obstacle_info.ds);

    if (!hexagon->IsPointIn(t_point)) {
      continue;
    }

    // loop six line of hexagon to find the min dis to these lines.
    double dis_to_hexagon_line = std::numeric_limits<double>::max();
    for (const auto& line : hexagon->line_segments()) {
      dis_to_hexagon_line = fmin(line.DistanceTo(t_point), dis_to_hexagon_line);
    }

    // calculate the cost.
    parallel_drive_cost =
        fmax(parallel_drive_cost, GetBigCarDistanceCost(dis_to_hexagon_line));
  }
  follow_time_cost += parallel_drive_cost;
  return follow_time_cost;
}

double NudgeCost::BigCarDistanceCostFunction(const double distance) const {
  const double p_a = config_.hexagon_cost_sigmoid_a();
  const double p_b = config_.hexagon_cost_sigmoid_b();
  return 1 / (1 + exp(-p_a * (distance - p_b))) *
         config_.hexagon_cost_sigmoid_factor();
}

double NudgeCost::ObstacleDistanceCostFunction(double det_distance) const {
  return config_.obstacle_distance_cost_coef() * pow(det_distance, 2) *
         exp(2 * (det_distance - 0.1));
}

}  // namespace planning
}  // namespace TL
