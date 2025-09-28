/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file obstacle_expected_distance_cost.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/costs/obstacle_expected_distance_cost.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include "common/math/linear_interpolation.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_curve.h"

namespace TL {
namespace planning {

ObstacleExpectedDistanceCost::ObstacleExpectedDistanceCost(  // NOLINT
    const SpeedCostConfig& config)
    : SpeedCost(config),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
  if (config.has_obstacle_expected_distance_cost_config()) {
    config_.CopyFrom(config.obstacle_expected_distance_cost_config());
  }

  LoadLessExpectedDistanceCostCoefTable();
  LoadGreaterExpectedDistanceCostCoefTable();
  LoadLessExpectedDistanceCostTable();
  LoadTTCCostTable();
}

void ObstacleExpectedDistanceCost::LoadLessExpectedDistanceCostCoefTable() {
  std::vector<double> speeds;
  std::vector<double> coefs;
  const auto& calibration_infos =
      config_.less_expected_distance_cost_coef_calibration_table()
          .calibration_info();
  speeds.reserve(calibration_infos.size());
  coefs.reserve(calibration_infos.size());
  for (const auto& calibration_info : calibration_infos) {
    speeds.emplace_back(calibration_info.speed());
    coefs.emplace_back(calibration_info.coef());
  }
  for (int i = 0;
       i < static_cast<int>(greater_expected_distance_cost_coef_table_.size());
       ++i) {
    less_expected_distance_cost_coef_table_.at(i) =
        common::math::InterpolationOne(i * speed_epsilon_, speeds, coefs);
  }
}

void ObstacleExpectedDistanceCost::LoadGreaterExpectedDistanceCostCoefTable() {
  std::vector<double> speeds;
  std::vector<double> coefs;
  const auto& calibration_infos =
      config_.greater_expected_distance_cost_coef_calibration_table()
          .calibration_info();
  speeds.reserve(calibration_infos.size());
  coefs.reserve(calibration_infos.size());
  for (const auto& calibration_info : calibration_infos) {
    speeds.emplace_back(calibration_info.speed());
    coefs.emplace_back(calibration_info.coef());
  }
  for (int i = 0;
       i < static_cast<int>(greater_expected_distance_cost_coef_table_.size());
       ++i) {
    greater_expected_distance_cost_coef_table_.at(i) =
        common::math::InterpolationOne(i * speed_epsilon_, speeds, coefs);
  }
}

void ObstacleExpectedDistanceCost::LoadLessExpectedDistanceCostTable() {
  const auto& base_cost_coef = config_.base_less_expected_distance_cost_coef();
  const auto& extra_cost_coefs =
      config_.extra_less_expected_distance_cost_coef();

  less_expected_distance_cost_tables_.assign(extra_cost_coefs.size(), {});
  for (int i = 0; i < extra_cost_coefs.size(); ++i) {
    const auto& extra_cost_coef = extra_cost_coefs.at(i);
    less_expected_distance_cost_tables_.at(i).first =
        extra_cost_coef.follow_time();
    auto& cost_table = less_expected_distance_cost_tables_.at(i).second;
    for (int j = 0; j < static_cast<int>(cost_table.size()); ++j) {
      const auto less_expected_distance_error =
          (j * expected_distance_error_epsilon_);
      cost_table.at(j) =
          base_cost_coef.a() *
          pow(less_expected_distance_error, base_cost_coef.b()) *
          exp(base_cost_coef.c() *
              (less_expected_distance_error - base_cost_coef.d()));
      if (less_expected_distance_error > extra_cost_coef.d()) {
        cost_table.at(j) +=
            extra_cost_coef.a() *
            pow(less_expected_distance_error - extra_cost_coef.d(),
                extra_cost_coef.b()) *
            exp(extra_cost_coef.c() *
                (less_expected_distance_error - extra_cost_coef.d()));
      }
    }
  }
}

void ObstacleExpectedDistanceCost::LoadTTCCostTable() {
  for (int i = 0; i < static_cast<int>(ttc_cost_table_.size()); ++i) {
    const auto ttc = 10.0 - (i * ttc_epsilon_);
    if (ttc < 5.0) {
      ttc_cost_table_.at(i) = 1e4 * pow(ttc, 3) * exp(1.5 * (ttc - 5.0));
    } else {
      ttc_cost_table_.at(i) = 1e4 * pow(ttc, 3) * exp(4.5 * (ttc - 5.0));
    }
  }

  for (int i = 0; i < static_cast<int>(ttc_cost_coef_table_.size()); ++i) {
    const auto speed = i * speed_epsilon_;
    if (speed < 10.0) {
      ttc_cost_coef_table_.at(i) = exp(1.5 * (speed - 10.0));
    } else {
      ttc_cost_coef_table_.at(i) = 1.0;
    }
  }
}

void ObstacleExpectedDistanceCost::SetFollowTime(const double follow_time) {
  double min_dis = std::numeric_limits<double>::max();
  for (const auto& [time, table] : less_expected_distance_cost_tables_) {
    const auto dis = fabs(follow_time - time);
    if (dis < min_dis) {
      min_dis = dis;
      current_less_expected_distance_cost_table_ = &table;
    }
  }
}

bool ObstacleExpectedDistanceCost::CalculateCost(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->obstacle_less_expected_distance_cost =
        std::numeric_limits<double>::infinity();
    cost_result->obstacle_greater_expected_distance_cost =
        std::numeric_limits<double>::infinity();
    return false;
  }

  if (cost_result == nullptr || cost_result->curve == nullptr ||
      !cache.GetBasicCache().GetIsForwardPath() ||
      cost_result->curve->GetTarget().mode == SpeedCurveTarget::Mode::STOP) {
    return true;
  }

  cost_result->obstacle_less_expected_distance_cost = 0.0;
  cost_result->obstacle_greater_expected_distance_cost = 0.0;
  const auto& curve = cost_result->curve;
  if (curve->GetTarget().mode == SpeedCurveTarget::Mode::STOP ||
      curve->GetTarget().mode == SpeedCurveTarget::Mode::STOP_TO_STANDSTILL) {
    return true;
  }

  const auto* obstacle_cache = cache.GetFollowSLTObstacleCache();
  if (obstacle_cache == nullptr) {
    return true;
  }

  const auto min_sparse_point_count = curve->GetMinSparsePointCount();
  const auto& sparse_points = curve->GetSparsePoints();
  const auto temp_v = fmax(sparse_points.front().v(), 1.0);

  const auto& follow_times = cache.GetFollowTimes();
  const auto& set_follow_time = cache.GetFollowTime();
  if (follow_times.size() < min_sparse_point_count) {
    return false;
  }

  if (sparse_points.empty()) {
    return false;
  }

  const auto ttc_buffer = 1.0 + sparse_points[0].v() * 0.2;
  const auto less_expected_distance_cost_coef =
      GetLessExpectedDistanceCostCoefAtSpeed(sparse_points[0].v());
  const auto greater_expected_distance_cost_coef =
      GetGreaterExpectedDistanceCostCoefAtSpeed(sparse_points[0].v());

  for (int i = 0; i < min_sparse_point_count; ++i) {
    const auto& point = sparse_points[i];
    auto t = point.t();
    auto s = point.s();
    auto v = point.v();

    // check if obstacle is in st graph at time t
    if (t < obstacle_cache->GetMinT() || t > obstacle_cache->GetMaxT()) {
      continue;
    }

    // calucalate s_upper, s_lower, obstacle_v
    const auto& obstacle_info = obstacle_cache->GetObstacleInfoAtTime(t);

    // calculate real_distance
    auto real_distance =
        obstacle_info.s_lower - (s + vehicle_param_.front_edge_to_center());

    // calculate ttc
    const auto ds = real_distance - ttc_buffer;
    const auto dv = v - obstacle_info.ds;
    auto ttc = 0.0;
    if (ds > 0.0) {
      ttc = dv <= 0.0 ? std::numeric_limits<double>::infinity() : ds / dv;
    }
    ttc = fmin(10.0, ttc);

    // calculate less_follow_time_error
    const auto less_expected_distance =
        obstacle_cache->GetMinFollowDistance() +
        fmax(fmin(set_follow_time, follow_times.at(i)) * v, 0.0);
    const auto less_follow_time_error =
        (real_distance < less_expected_distance)
            ? (less_expected_distance - real_distance) / temp_v
            : 0.0;

    // calculate greater_follow_time_error
    const auto greater_expected_distance =
        obstacle_cache->GetMinFollowDistance() +
        fmax(fmax(set_follow_time, follow_times.at(i)) * v, 0.0);
    const auto greater_follow_time_error =
        (real_distance > greater_expected_distance)
            ? (real_distance - greater_expected_distance) / temp_v
            : 0.0;

    // calculate obstacle_less_expected_distance_cost
    const auto less_expected_distance_cost =
        (less_expected_distance_cost_coef *
             GetLessExpectedDistanceCost(less_follow_time_error) +
         GetTTCCost(v, ttc)) *
        config_.obstacle_cost_time_coef(i);

    // calculate obstacle_greater_expected_distance_cost
    const auto greater_expected_distance_cost =
        greater_follow_time_error * greater_follow_time_error *
        greater_expected_distance_cost_coef *
        config_.obstacle_cost_time_coef(i);

    if (GetPrintDebug()) {
      ADEBUG << "t:" << t << ", real_distance:" << real_distance
             << ", less_expected_distance:" << less_expected_distance
             << ", greater_expected_distance:" << greater_expected_distance
             << ", follow_time:" << follow_times.at(i)
             << ", less_follow_time_error:" << less_follow_time_error
             << ", less_expected_distance_cost:" << less_expected_distance_cost
             << ", greater_follow_time_error:" << greater_follow_time_error
             << ", greater_expected_distance_cost:"
             << greater_expected_distance_cost << ", ttc:" << ttc
             << ", ttc_cost:" << GetTTCCost(v, ttc) << ", GetMinFollowDistance:"
             << obstacle_cache->GetMinFollowDistance();
    }

    cost_result->obstacle_less_expected_distance_cost +=
        less_expected_distance_cost;
    cost_result->obstacle_greater_expected_distance_cost +=
        greater_expected_distance_cost;
    cost_result->mean_less_follow_time_error += less_follow_time_error;
    cost_result->mean_greater_follow_time_error += greater_follow_time_error;
    if (obstacle_cache->GetIsFront()) {
      cost_result->front_obstacle_cost.obstacle_less_expected_distance_cost +=
          less_expected_distance_cost;
      cost_result->front_obstacle_cost
          .obstacle_greater_expected_distance_cost +=
          greater_expected_distance_cost;
      cost_result->front_obstacle_cost.mean_less_follow_time_error +=
          less_follow_time_error;
      cost_result->front_obstacle_cost.mean_greater_follow_time_error +=
          greater_follow_time_error;
    }
  }
  cost_result->obstacle_less_expected_distance_cost /= min_sparse_point_count;
  cost_result->obstacle_greater_expected_distance_cost /=
      min_sparse_point_count;
  cost_result->mean_less_follow_time_error /= min_sparse_point_count;
  cost_result->mean_greater_follow_time_error /= min_sparse_point_count;
  cost_result->front_obstacle_cost.obstacle_less_expected_distance_cost /=
      min_sparse_point_count;
  cost_result->front_obstacle_cost.obstacle_greater_expected_distance_cost /=
      min_sparse_point_count;
  cost_result->front_obstacle_cost.mean_less_follow_time_error /=
      min_sparse_point_count;
  cost_result->front_obstacle_cost.mean_greater_follow_time_error /=
      min_sparse_point_count;
  return true;
}

}  // namespace planning
}  // namespace TL
