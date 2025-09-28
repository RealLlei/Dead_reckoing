/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file accel_cost.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/costs/accel_cost.h"

#include <algorithm>
#include <limits>

#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreaterEqual;
using common::math::double_type::DefinitelyLessEqual;

AccelCost::AccelCost(const SpeedCostConfig& config) : SpeedCost(config) {
  if (config.has_accel_cost_config()) {
    config_.CopyFrom(config.accel_cost_config());
  }
  std::vector<double> speeds;
  std::vector<double> max_comfort_accels;
  std::vector<double> min_comfort_accels;
  if (config_.has_comfort_accel_calibration_table() &&
      config_.comfort_accel_calibration_table().calibration_info_size() > 1) {
    const auto calibration_info_size =
        config_.comfort_accel_calibration_table().calibration_info_size();
    speeds.reserve(calibration_info_size);
    max_comfort_accels.reserve(calibration_info_size);
    min_comfort_accels.reserve(calibration_info_size);
    for (const auto& calibration_info :
         config_.comfort_accel_calibration_table().calibration_info()) {
      speeds.emplace_back(calibration_info.speed());
      max_comfort_accels.emplace_back(calibration_info.max_accel());
      min_comfort_accels.emplace_back(calibration_info.min_accel());
    }
  } else {
    speeds = {0.0, FLAGS_planning_upper_speed_limit};
    max_comfort_accels = {1.0, 1.0};
    min_comfort_accels = {-1.0, -1.0};
  }

  for (int i = 0; i < static_cast<int>(comfort_accel_table_.size()); ++i) {
    const double v = i * speed_epsilon_;
    comfort_accel_table_.at(i).first =
        common::math::InterpolationOne(v, speeds, max_comfort_accels);
    comfort_accel_table_.at(i).second =
        common::math::InterpolationOne(v, speeds, min_comfort_accels);
  }

  for (int i = 0; i < static_cast<int>(accel_cost_table_.size()); ++i) {
    const auto speed = i * speed_epsilon_;
    auto& table = accel_cost_table_.at(i);
    for (int j = 0; j < static_cast<int>(table.size()); ++j) {
      table.at(j) =
          AccelCostFunction(speed, (j - accel_shift_) * accel_epsilon_);
    }
  }

  for (int i = 0; i < static_cast<int>(jerk_coef_table_.size()); ++i) {
    const auto jerk = (i - jerk_shift_) * jerk_epsilon_;
    if (jerk < 0.2) {
      jerk_coef_table_.at(i) = 1.0;
    } else {
      jerk_coef_table_.at(i) = exp(20.0 * (0.2 - jerk));
    }
  }
}

bool AccelCost::CalculateCost(const SpeedCache& cache,
                              const ReferenceLineInfo& reference_line_info,
                              SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  UNUSED(cache);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->accel_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  const auto& basic_cache = cache.GetBasicCache();
  const auto is_follow_curve =
      cost_result->curve->GetTarget().mode == SpeedCurveTarget::Mode::FOLLOW;

  cost_result->accel_cost = 0.0;
  const auto curve_dense_point_count =
      cost_result->curve->GetCurveDensePointCount();
  const auto& dense_points = cost_result->curve->GetDensePoints();

  auto first_point_valid = true;
  if (!dense_points.empty()) {
    const auto& point = dense_points.front();
    const auto& accel_limit = basic_cache.GetAccelLimit(point.v());
    first_point_valid = DefinitelyGreaterEqual(point.a(), accel_limit.first) &&
                        DefinitelyLessEqual(point.a(), accel_limit.second);
  }

  for (int i = 1; i < curve_dense_point_count; ++i) {
    const auto& point = dense_points[i];

    const auto& accel_limit = basic_cache.GetAccelLimit(point.v());
    if (first_point_valid &&
        (point.a() < accel_limit.first || point.a() > accel_limit.second)) {
      cost_result->accel_cost = std::numeric_limits<double>::infinity();
      return false;
    }
    cost_result->accel_cost +=
        GetAccelCost(point.v(),
                     is_follow_curve
                         ? fmin(point.a(), GetComfortAccel(point.v()).first)
                         : point.a()) *
        GetJerkCoef(point.j());
  }
  cost_result->accel_cost /= curve_dense_point_count;
  return true;
}

double AccelCost::AccelCostFunction(const double speed,
                                    const double accel) const {
  const auto& comfort_accel = GetComfortAccel(speed);
  const double weight_accel_positive =
      config_.max_comfort_cost() * 1.0 / pow(comfort_accel.first, 2);
  const double weight_accel_negetive =
      config_.max_comfort_cost() * 100.0 / pow(comfort_accel.second, 2);
  if (accel > config_.max_sensitive_accel()) {
    return weight_accel_positive * pow(accel, 2) *
           exp(16 * (config_.max_sensitive_accel() - comfort_accel.first));
  }
  if (accel > comfort_accel.first) {
    return weight_accel_positive * pow(accel, 2) *
           exp(16 * (accel - comfort_accel.first));
  }
  if (accel > 0) {
    return weight_accel_positive * pow(accel, 2) *
           exp(3 * (accel - comfort_accel.first));
  }
  if (accel > comfort_accel.second) {
    return weight_accel_negetive * pow(accel, 2) *
           exp(5.0 * (comfort_accel.second - accel));
  }
  if (accel > config_.min_sensitive_accel()) {
    return weight_accel_negetive * pow(accel, 2) *
           exp(3.5 * (comfort_accel.second - accel));
  }
  return weight_accel_negetive * pow(accel, 2) *
         exp(2 * (comfort_accel.second - config_.min_sensitive_accel()));
}

}  // namespace planning
}  // namespace TL
