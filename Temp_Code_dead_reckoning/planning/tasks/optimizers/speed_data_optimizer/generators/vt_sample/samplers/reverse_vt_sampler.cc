/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file reverse_normal_vt_sampler.cc
 **/
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/samplers/reverse_vt_sampler.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <utility>

#include "common/configs/config_gflags.h"
#include "common/math/double_type.h"
#include "common/util/point_factory.h"
#include "planning/common/planning_gflags.h"

namespace TL {
namespace planning {

using TL::common::math::double_type::AlmostEqual;
using TL::common::math::double_type::DefinitelyGreaterEqual;
using TL::common::math::double_type::DefinitelyLess;

ReverseVtSampler::ReverseVtSampler(const VtSamplerConfig& sampler_config,
                                   const SpeedCurveConfig& curve_config)
    : VtSampler(sampler_config, curve_config) {
  if (sampler_config.has_reverse_vt_sampler_config()) {
    config_.CopyFrom(sampler_config.reverse_vt_sampler_config());
  }

  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  config_.set_max_accel(
      fmin(config_.max_accel(), vehicle_param.max_acceleration()));
  config_.set_min_accel(
      fmax(config_.min_accel(), vehicle_param.max_deceleration()));

  quadratic_vt_curves_.reserve(config_.max_quadratic_curve_count());
  for (int i = 0; i < config_.max_quadratic_curve_count(); ++i) {
    quadratic_vt_curves_.emplace_back(
        std::make_shared<QuadraticVTCurve>(curve_config));
  }
  normal_curve_cost_results_.assign(config_.max_quadratic_curve_count(), {});

  sample_accels_.reserve(config_.sample_accel_size());
  for (const auto& accel : config_.sample_accel()) {
    sample_accels_.push_back(accel);
  }
}

void ReverseVtSampler::SampleGuideCurves(
    const common::TrajectoryPoint& init_point, const SpeedCache& cache) {
  UNUSED(init_point);
  UNUSED(cache);
  guide_curve_count_ = 0;
}

void ReverseVtSampler::SampleSafeCurves(
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const SLTObstacleCache& obstacle_cache, const bool& set_extend,
    const std::vector<double>& sample_time_extend_time) {
  UNUSED(init_point);
  UNUSED(cache);
  UNUSED(obstacle_cache);
  UNUSED(set_extend);
  UNUSED(sample_time_extend_time);
  safe_curve_count_ = 0;
}

void ReverseVtSampler::SampleNormalCurves(
    const common::TrajectoryPoint& init_point,
    const ReferenceLineInfo& reference_line_info, const SpeedCache& cache,
    const bool& set_extend,
    const std::vector<double>& sample_time_extend_time) {
  UNUSED(set_extend);
  UNUSED(sample_time_extend_time);
  UNUSED(reference_line_info);
  quadratic_vt_curve_count_ = 0;

  // sample cruise curves
  SampleCruiseCurves(init_point, cache);

  // sample stop curves
  SampleStopCurves(init_point, cache);

  // sample unknown curves
  SampleUnknownCurves(init_point, cache);

  normal_curve_count_ =
      std::min(quadratic_vt_curve_count_, normal_curve_cost_results_.size());
  for (std::size_t i = 0; i < normal_curve_count_; ++i) {
    auto& normal_curve_cost_result = normal_curve_cost_results_.at(i);
    normal_curve_cost_result.already_calculated = false;
    normal_curve_cost_result.curve = quadratic_vt_curves_.at(i);
  }
}

bool ReverseVtSampler::AddLastCurve(
    const std::shared_ptr<SpeedCurve>& last_curve, const SpeedCache& cache) {
  UNUSED(last_curve);
  UNUSED(cache);
  return false;
}

bool ReverseVtSampler::GenerateFallbackSpeedData(
    const common::TrajectoryPoint& init_point,
    const ReferenceLineInfo& reference_line_info, const SpeedCache& cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator, SpeedData* speed_data) {
  UNUSED(reference_line_info);
  UNUSED(cache);
  if (speed_data == nullptr || evaluator == nullptr) {
    return false;
  }

  speed_data->clear();
  const auto a = fmin(config_.min_accel() * 3.0, -1e-5);
  const auto unit_t = FLAGS_trajectory_time_resolution;
  const auto max_t = fabs(init_point.v() / a);
  const auto max_s = fabs(0.5 * pow(init_point.v(), 2) / a);
  const auto t_count =
      static_cast<int>(std::round(FLAGS_trajectory_time_length / unit_t));
  for (int i = 0.0; i <= t_count; ++i) {
    const auto t = i * unit_t;
    speed_data->emplace_back(common::util::PointFactory::ToSpeedPoint(
        t < max_t ? (init_point.v() * t + 0.5 * a * pow(t, 2)) : max_s, t,
        fmax(init_point.v() + a * t, 0.0), a, 0.0));
  }
  return true;
}

void ReverseVtSampler::SampleCruiseCurves(
    const common::TrajectoryPoint& init_point, const SpeedCache& cache) {
  // if adc dose not need cruise curve, return
  // sample cruise
  const auto& speed_limit_cache = cache.GetSpeedLimitCache();
  ADEBUG << "[cruise_speed]" << speed_limit_cache.cruise_target_speed();
  SpeedCurveTarget target;
  target.mode = SpeedCurveTarget::Mode::CRUISE;
  target.type = SpeedCurveTarget::Type::NORMAL;

  int curve_count = 0;
  constexpr auto kSpeedCount = 10;
  const auto unit_v = speed_limit_cache.cruise_target_speed() / kSpeedCount;
  for (int i = 0; i <= kSpeedCount; ++i) {
    target.speed = i * unit_v;
    curve_count += SampleCurveForSpeed(init_point, target.speed, target);
  }
  ADEBUG << "sample cruise curve count:" << curve_count;
}

void ReverseVtSampler::SampleStopCurves(
    const common::TrajectoryPoint& init_point, const SpeedCache& cache) {
  // if adc dose not need stop curve, return
  if (!CheckNeedStopCurve(init_point.v(),
                          cache.GetBasicCache().GetExpectedStopS())) {
    return;
  }

  // sample stop curves
  SpeedCurveTarget target;
  target.mode = SpeedCurveTarget::Mode::STOP;
  target.type = SpeedCurveTarget::Type::NORMAL;
  target.speed = 0.0;

  auto stop_s = fmax(fabs(0.5 * pow(init_point.v(), 2) / config_.min_accel()),
                     cache.GetBasicCache().GetExpectedStopS());
  const auto curve_count = SampleCurveForLength(init_point, stop_s, target);
  ADEBUG << "sample stop curve count:" << curve_count;
}

void ReverseVtSampler::SampleUnknownCurves(
    const common::TrajectoryPoint& init_point, const SpeedCache& cache) {
  min_obstacle_st_boundary_ = std::numeric_limits<double>::infinity();
  for (const auto* obstacle_cache : cache.GetSafeSTObstacleCaches()) {
    const auto* obstacle = obstacle_cache->GetObstacle();
    if (obstacle == nullptr) {
      continue;
    }
    min_obstacle_st_boundary_ =
        fmin(min_obstacle_st_boundary_, obstacle->path_st_boundary().min_s());
  }

  // sample unknown curves
  if (!std::isinf(min_obstacle_st_boundary_)) {
    SpeedCurveTarget target;
    target.mode = SpeedCurveTarget::Mode::UNKNOWN;
    target.type = SpeedCurveTarget::Type::NORMAL;
    target.speed = 0.0;
    const auto curve_count = SampleCurveForLength(
        init_point, fmax(min_obstacle_st_boundary_, 0.0), target);
    ADEBUG << "sample unknown curve count:" << curve_count;
  }
}

int ReverseVtSampler::SampleCurveForSpeed(
    const common::TrajectoryPoint& init_point, const double end_v,
    const SpeedCurveTarget& target) {
  const auto curve_count_before_sample = quadratic_vt_curve_count_;

  std::vector<std::pair<double, double>> params(1);
  for (const auto a : sample_accels_) {
    const auto t = (AlmostEqual(init_point.v(), end_v) && AlmostEqual(a, 0.0))
                       ? 1.0
                       : (end_v - init_point.v()) / a;

    if (!std::isfinite(t) || DefinitelyLess(t, 0.0)) {
      continue;
    }

    params[0].first = t;
    params[0].second = a;
    if (quadratic_vt_curve_count_ >= quadratic_vt_curves_.size() ||
        !quadratic_vt_curves_[quadratic_vt_curve_count_++]
             ->InitPiecewiseAccelCurve(init_point, params, 0.0, target)) {
      continue;
    }

    for (const auto& rule : rules_) {
      if (!rule->ApplyRule(
              quadratic_vt_curves_[quadratic_vt_curve_count_ - 1])) {
        --quadratic_vt_curve_count_;
        break;
      }
    }
  }

  return static_cast<int>(quadratic_vt_curve_count_ -
                          curve_count_before_sample);
}

int ReverseVtSampler::SampleCurveForLength(
    const common::TrajectoryPoint& init_point, const double end_s,
    const SpeedCurveTarget& target) {
  const auto curve_count_before_sample = quadratic_vt_curve_count_;

  std::vector<std::pair<double, double>> params(2);
  for (const auto a : sample_accels_) {
    if (DefinitelyGreaterEqual(a, 0.0)) {
      continue;
    }

    const auto t2 = -init_point.v() / a;
    const auto s2 = -0.5 * pow(init_point.v(), 2) / a;
    const auto t1 =
        AlmostEqual(end_s, s2) ? 0.0 : (end_s - s2) / init_point.v();
    if (!std::isfinite(t1) || !std::isfinite(t2) || DefinitelyLess(t1, 0.0) ||
        DefinitelyLess(t2, 0.0)) {
      continue;
    }
    params[0].first = t1;
    params[0].second = 0.0;
    params[1].first = t2;
    params[1].second = a;

    if (quadratic_vt_curve_count_ >= quadratic_vt_curves_.size() ||
        !quadratic_vt_curves_[quadratic_vt_curve_count_++]
             ->InitPiecewiseAccelCurve(
                 init_point, params, config_.stop_curve_end_accel(), target)) {
      continue;
    }
    for (const auto& rule : rules_) {
      if (!rule->ApplyRule(
              quadratic_vt_curves_[quadratic_vt_curve_count_ - 1])) {
        --quadratic_vt_curve_count_;
        break;
      }
    }
  }

  return static_cast<int>(quadratic_vt_curve_count_ -
                          curve_count_before_sample);
}

bool ReverseVtSampler::CheckNeedStopCurve(const double start_v,
                                          const double stop_s) {
  static constexpr double kEpsilon = 1e-6;
  return !std::isinf(stop_s) &&
         (-0.5 * start_v * start_v / (stop_s + kEpsilon) <
          config_.average_accel_threshold_for_stop_curve());
}

}  // namespace planning
}  // namespace TL
