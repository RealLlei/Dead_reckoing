/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file jerk_cost.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/costs/jerk_cost.h"

#include <algorithm>
#include <cmath>

#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreaterEqual;
using common::math::double_type::DefinitelyLessEqual;

JerkCost::JerkCost(const SpeedCostConfig& config) : SpeedCost(config) {
  if (config.has_jerk_cost_config()) {
    config_.CopyFrom(config.jerk_cost_config());
  }
  std::vector<double> speeds;
  std::vector<double> max_comfort_jerks;
  std::vector<double> min_comfort_jerks;
  if (config_.has_comfort_jerk_calibration_table() &&
      config_.comfort_jerk_calibration_table().calibration_info_size() > 1) {
    const auto calibration_info_size =
        config_.comfort_jerk_calibration_table().calibration_info_size();
    speeds.reserve(calibration_info_size);
    max_comfort_jerks.reserve(calibration_info_size);
    min_comfort_jerks.reserve(calibration_info_size);
    for (const auto& calibration_info :
         config_.comfort_jerk_calibration_table().calibration_info()) {
      speeds.emplace_back(calibration_info.speed());
      max_comfort_jerks.emplace_back(calibration_info.max_jerk());
      min_comfort_jerks.emplace_back(calibration_info.min_jerk());
    }
  } else {
    speeds = {0.0, FLAGS_planning_upper_speed_limit};
    max_comfort_jerks = {1.0, 1.0};
    min_comfort_jerks = {-1.0, -1.0};
  }

  for (int i = 0; i < static_cast<int>(comfort_jerk_table_.size()); ++i) {
    const double v = i * speed_epsilon_;
    comfort_jerk_table_.at(i).first =
        common::math::InterpolationOne(v, speeds, max_comfort_jerks);
    comfort_jerk_table_.at(i).second =
        common::math::InterpolationOne(v, speeds, min_comfort_jerks);
  }

  for (int i = 0; i < static_cast<int>(jerk_cost_table_.size()); ++i) {
    const auto speed = i * speed_epsilon_;
    auto& table = jerk_cost_table_.at(i);
    for (int j = 0; j < static_cast<int>(table.size()); ++j) {
      table.at(j) = JerkCostFunction(speed, (j - jerk_shift_) * jerk_epsilon_);
    }
  }
}

bool JerkCost::CalculateCost(const SpeedCache& cache,
                             const ReferenceLineInfo& reference_line_info,
                             SpeedCurveCostResult* cost_result) const {
  UNUSED(cache);
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->jerk_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  const auto curve_dense_point_count =
      cost_result->curve->GetCurveDensePointCount();
  const auto& dense_points = cost_result->curve->GetDensePoints();
  cost_result->jerk_cost = 0.0;

  const auto& basic_cache = cache.GetBasicCache();
  auto first_point_valid = true;
  if (!dense_points.empty()) {
    const auto& point = dense_points.front();
    const auto& jerk_limit = basic_cache.GetJerkLimit(point.v());
    first_point_valid = DefinitelyGreaterEqual(point.j(), jerk_limit.first) &&
                        DefinitelyLessEqual(point.j(), jerk_limit.second);
  }

  for (int i = 0; i < curve_dense_point_count; ++i) {
    const auto& point = dense_points[i];

    const auto& jerk_limit = basic_cache.GetJerkLimit(point.v());
    if (first_point_valid &&
        (point.j() < jerk_limit.first || point.j() > jerk_limit.second)) {
      cost_result->jerk_cost = std::numeric_limits<double>::infinity();
      return false;
    }

    if (point.a() < 0.0 && point.j() > 0.0) {
      cost_result->jerk_cost += GetJerkCost(point.v(), point.j()) * 1e-5;
    } else {
      cost_result->jerk_cost += GetJerkCost(point.v(), point.j());
    }
  }
  cost_result->jerk_cost /= curve_dense_point_count;
  return true;
}

double JerkCost::JerkCostFunction(const double speed, const double jerk) const {
  const auto& comfort_jerk = GetComfortJerk(speed);
  const double weight_jerk_positive =
      config_.max_comfort_cost() * 0.1 / pow(comfort_jerk.first, 2) / exp(0.2);
  const double weight_jerk_negetive =
      config_.max_comfort_cost() / pow(comfort_jerk.second, 2) / exp(0.2);
  if (jerk > config_.max_sensitive_jerk()) {
    return weight_jerk_positive * pow(jerk, 2) *
           exp(3 * (config_.max_sensitive_jerk() - comfort_jerk.first));
  }
  if (jerk > 0) {
    return weight_jerk_positive * pow(jerk, 2) *
           exp(3 * (jerk - comfort_jerk.first));
  }
  if (jerk > config_.min_sensitive_jerk()) {
    return weight_jerk_negetive * pow(jerk, 2) *
           exp(3 * (comfort_jerk.second - jerk));
  }
  return weight_jerk_negetive * pow(jerk, 2) *
         exp(3 * (comfort_jerk.second - config_.min_sensitive_jerk()));
}

}  // namespace planning
}  // namespace TL
