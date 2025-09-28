/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file stop_distance_cost.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/costs/stop_distance_cost.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"
#include "planning/common/reference_line_info.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreater;
using common::math::double_type::DefinitelyLess;

StopDistanceCost::StopDistanceCost(const SpeedCostConfig& config)
    : SpeedCost(config) {
  if (config.has_stop_distance_cost_config()) {
    config_.CopyFrom(config.stop_distance_cost_config());
  }
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();

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
}

bool StopDistanceCost::CalculateCost(
    const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
    SpeedCurveCostResult* cost_result) const {
  UNUSED(reference_line_info);
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->stop_distance_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  if (cost_result->curve->GetTarget().mode == SpeedCurveTarget::Mode::STOP) {
    return CalculateCostForStopCurve(cache, cost_result);
  }
  return CalculateCostForNonStopCurve(cache, cost_result);
}

bool StopDistanceCost::CalculateCostForStopCurve(
    const SpeedCache& cache, SpeedCurveCostResult* cost_result) const {
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->stop_distance_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  if (cache.GetBasicCache().GetIsStopPrefinish() &&
      cost_result->curve->LastSelected()) {
    return true;
  }

  const auto& curve = cost_result->curve;
  const auto time_length = curve->GetTimeLength();
  cost_result->stop_distance_cost = 0.0;
  const auto& point_count = curve->GetCurveDensePointCount();
  const auto& dense_points = curve->GetDensePoints();
  const auto stop_low_speed_threshold = config_.stop_low_speed_threshold();
  if (dense_points.empty()) {
    return false;
  }
  int low_acc_cnt = 0;
  auto find_low_speed = false;
  const auto stop_low_speed_time_threshold =
      config_.stop_low_speed_time_threshold() +
      fmax(20.0 - dense_points.front().v(), 0.0) * 0.05;
  for (int i = 0; i < point_count; ++i) {
    const auto& point = dense_points.at(i);
    if (point.t() > time_length) {
      break;
    }
    if (point.v() < stop_low_speed_threshold && !find_low_speed) {
      find_low_speed = true;
      cost_result->stop_distance_cost +=
          fmax(time_length - point.t() - stop_low_speed_time_threshold, 0.0) *
          config_.stop_low_speed_time_cost_coef();
    }
    if (i < 1) {
      continue;
    }
    // 对于停车曲线，长时间[0, -0.3]的减速度是不是可以认为不合适的
    const auto& pre_point = dense_points.at(i - 1);
    if (DefinitelyGreater(pre_point.a(), config_.stop_low_acc_threshold()) &&
        DefinitelyGreater(point.a(), config_.stop_low_acc_threshold()) &&
        DefinitelyLess(point.a(), 0.0) && DefinitelyLess(pre_point.a(), 0.0)) {
      low_acc_cnt++;
    }
  }
  double low_acc_time = low_acc_cnt * curve->GetDensePointInterval();
  cost_result->stop_distance_cost +=
      fmax(low_acc_time - cache.GetBasicCache().GetStopSlideTimeThreshold(),
           0.0) *
      config_.stop_low_acc_time_cost_coef();

  if (CheckUnreasonableSpeed(cost_result->curve)) {
    cost_result->stop_distance_cost += config_.unreasonable_speed_cost();
  }
  return true;
}

bool StopDistanceCost::CalculateCostForNonStopCurve(
    const SpeedCache& cache, SpeedCurveCostResult* cost_result) const {
  if (cost_result == nullptr) {
    return false;
  }

  if (cost_result->curve == nullptr) {
    cost_result->stop_distance_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  const auto& basic_cache = cache.GetBasicCache();
  const auto stop_s =
      IsMergeStop()
          ? std::fmax(cache.GetBasicCache().GetHighRoadRightEndS(), 0.0)
          : basic_cache.GetRealStopS();

  const auto& dense_points = cost_result->curve->GetDensePoints();
  if (dense_points.empty()) {
    cost_result->stop_distance_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  const auto start_v = dense_points.front().v();
  const auto comfort_decel = fmin(GetComfortAccel(start_v).second, -0.1);
  const auto preview_time = fmax(2.0, start_v / fabs(comfort_decel));
  auto preview_index = static_cast<int>(
      preview_time / fmax(cost_result->curve->GetDensePointInterval(), 0.01));
  preview_index =
      std::min(preview_index, cost_result->curve->GetMinDensePointCount() - 1);
  if (preview_index < 0 || preview_index >= dense_points.size()) {
    cost_result->stop_distance_cost = std::numeric_limits<double>::infinity();
    return false;
  }

  const auto& end_point = dense_points.at(preview_index);
  const auto end_v = end_point.v();
  const auto final_s = end_point.s();
  if (final_s >= stop_s) {
    cost_result->stop_distance_cost = std::numeric_limits<double>::infinity();
    return false;
  }
  // calculate decel needed
  const auto decel = -0.5 * pow(end_v, 2) / (stop_s - final_s);
  const auto max_decel = vehicle_param_.max_deceleration();
  if (decel < max_decel) {
    cost_result->stop_distance_cost = std::numeric_limits<double>::infinity();
  } else if (decel < comfort_decel) {
    cost_result->stop_distance_cost =
        pow(comfort_decel - decel, 2) * config_.stop_accel_cost_coef();
  }
  return true;
}

bool StopDistanceCost::CheckUnreasonableSpeed(
    const std::shared_ptr<SpeedCurve>& curve) {
  if (curve == nullptr) {
    return true;
  }

  const auto& peek_v = curve->GetPeekV();
  return std::any_of(peek_v.begin(), peek_v.end(),
                     [](const auto& v) { return v.first && v.second < 0.5; });
}

}  // namespace planning
}  // namespace TL
