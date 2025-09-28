/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file forward_normal_vt_sampler.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/samplers/forward_vt_sampler.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <numeric>
#include <type_traits>
#include <utility>
#include <vector>

#include "common/file/log.h"
#include "common/math/angle.h"
#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/st_obstacle_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreaterEqual;
using common::math::double_type::DefinitelyLess;
using common::math::double_type::DefinitelyLessEqual;
using common::math::double_type::IsZero;

ForwardVtSampler::ForwardVtSampler(const VtSamplerConfig& sampler_config,
                                   const SpeedCurveConfig& curve_config)
    : VtSampler(sampler_config, curve_config),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
  if (sampler_config.has_forward_vt_sampler_config()) {
    config_.CopyFrom(sampler_config.forward_vt_sampler_config());
  }

  quadratic_vt_curves_.reserve(config_.max_quadratic_curve_count());
  for (int i = 0; i < config_.max_quadratic_curve_count(); ++i) {
    quadratic_vt_curves_.emplace_back(
        std::make_shared<QuadraticVTCurve>(curve_config));
  }
  cubic_vt_curves_.reserve(config_.max_cubic_curve_count());
  for (int i = 0; i < config_.max_cubic_curve_count(); ++i) {
    cubic_vt_curves_.emplace_back(std::make_shared<CubicVTCurve>(curve_config));
  }
  quartic_vt_curves_.reserve(config_.max_quartic_curve_count());
  for (int i = 0; i < config_.max_quartic_curve_count(); ++i) {
    quartic_vt_curves_.emplace_back(
        std::make_shared<QuarticVTCurve>(curve_config));
  }
  const auto max_curve_count = config_.max_quadratic_curve_count() +
                               config_.max_cubic_curve_count() +
                               config_.max_quartic_curve_count();
  guide_curve_cost_results_.assign(max_curve_count, {});
  safe_curve_cost_results_.assign(max_curve_count, {});
  normal_curve_cost_results_.assign(max_curve_count, {});
  merge_stop_curve_cost_results_.assign(max_curve_count, {});
  max_curve_count_ = max_curve_count;
  SampleStopTime();
  SampleCruiseTime();
  SampleFollowTime();
  SampleUnknownTime();

  sample_accels_.reserve(config_.sample_accel_size());
  for (const auto& accel : config_.sample_accel()) {
    sample_accels_.push_back(accel);
  }

  sample_jerks_.reserve(config_.sample_jerk_size());
  for (const auto& jerk : config_.sample_jerk()) {
    sample_jerks_.push_back(jerk);
  }
}

std::size_t ForwardVtSampler::CollectCurves(
    std::vector<SpeedCurveCostResult>* const curve_cost_results) {
  if (curve_cost_results == nullptr) {
    return 0;
  }

  const auto curve_count =
      std::min(quadratic_vt_curve_count_ + cubic_vt_curve_count_ +
                   quartic_vt_curve_count_,
               curve_cost_results->size());

  // add quadratic_vt_curve to curves_
  std::size_t curve_index = 0;
  for (std::size_t i = start_quadratic_vt_curve_index_;
       i < quadratic_vt_curve_count_; ++i) {
    if (curve_index >= curve_count) {
      break;
    }
    auto& curve_cost_result = curve_cost_results->at(curve_index++);
    curve_cost_result.already_calculated = false;
    curve_cost_result.curve = quadratic_vt_curves_.at(i);
  }

  // add cubic_vt_curves_ to curves_
  for (std::size_t i = start_cubic_vt_curve_index_; i < cubic_vt_curve_count_;
       ++i) {
    if (curve_index >= curve_count) {
      break;
    }
    auto& curve_cost_result = curve_cost_results->at(curve_index++);
    curve_cost_result.already_calculated = false;
    curve_cost_result.curve = cubic_vt_curves_.at(i);
  }

  // add quartic_vt_curves_ to curves_
  for (std::size_t i = start_quartic_vt_curve_index_;
       i < quartic_vt_curve_count_; ++i) {
    if (curve_index >= curve_count) {
      break;
    }
    auto& curve_cost_result = curve_cost_results->at(curve_index++);
    curve_cost_result.already_calculated = false;
    curve_cost_result.curve = quartic_vt_curves_.at(i);
  }

  return curve_index;
}

void ForwardVtSampler::SampleGuideCurves(
    const common::TrajectoryPoint& init_point, const SpeedCache& cache) {

  const auto& accel_limit =
      cache.GetBasicCache().GetAccelLimit(fabs(init_point.v()));
  config_.set_min_accel(accel_limit.first);
  config_.set_max_accel(accel_limit.second);

  for (const auto& rule : rules_) {
    rule->Update(init_point, cache);
  }

  quadratic_vt_curve_count_ = 0;
  quartic_vt_curve_count_ = 0;
  cubic_vt_curve_count_ = 0;
  guide_curve_count_ = 0;
  start_quadratic_vt_curve_index_ = 0;
  start_quartic_vt_curve_index_ = 0;
  start_cubic_vt_curve_index_ = 0;

  // smaple unknown curve indices
  SpeedCurveTarget target;
  target.mode = SpeedCurveTarget::Mode::UNKNOWN;
  target.type = SpeedCurveTarget::Type::GUIDE;

  const auto cruise_target_speed =
      cache.GetSpeedLimitCache().cruise_target_speed();

  if (cubic_vt_curve_count_ == 0) {
    for (const auto t : sample_cruise_times_) {
      const auto v_lower = init_point.v() + config_.min_accel() * t;
      const auto v_upper = init_point.v() + config_.max_accel() * t;
      if (cruise_target_speed < v_lower || cruise_target_speed > v_upper) {
        continue;
      }
      target.speed = cruise_target_speed;
      SampleCubicVTCurveForEndTimeSpeedAccel(init_point, t, cruise_target_speed,
                                             0.0, target);
    }
  }

  for (const auto t : sample_unknown_times_) {
    const auto v_lower = fmax(0.0, init_point.v() + config_.min_accel() * t);
    const auto v_upper =
        fmin(cruise_target_speed, init_point.v() + config_.max_accel() * t);
    if (v_lower > v_upper) {
      continue;
    }
    const auto interval =
        (v_upper - v_lower) / (config_.low_speed_unknown_v_count() - 1);
    for (int i = 0; i < config_.low_speed_unknown_v_count(); i++) {
      const auto v = v_lower + i * interval;
      target.speed = v;
      SampleCubicVTCurveForEndTimeSpeedAccel(init_point, t, v, 0.0, target);
    }
  }

  for (const auto t : sample_unknown_times_) {
    const auto v_lower = cruise_target_speed;
    const auto v_upper =
        fmin(cruise_target_speed * (1.0 + config_.over_speed_ratio()),
             init_point.v() + config_.max_accel() * t);
    if (v_lower > v_upper) {
      continue;
    }
    const auto interval =
        (v_upper - v_lower) / (config_.over_speed_unknown_v_count() - 1);
    for (int i = 0; i < config_.over_speed_unknown_v_count(); i++) {
      const auto v = v_lower + i * interval;
      target.speed = v;
      SampleCubicVTCurveForEndTimeSpeedAccel(init_point, t, v, 0.0, target);
      ADEBUG << "t:" << t << ", v:" << v
             << ", cruise_target_speed:" << cruise_target_speed;
    }
  }

  for (const auto t : sample_dense_unknown_times_) {
    const auto v_lower = fmax(0.0, init_point.v() + config_.min_accel() * t);
    const auto v_upper =
        fmin(cruise_target_speed, init_point.v() + config_.max_accel() * t);
    if (v_lower > v_upper) {
      continue;
    }
    const auto interval =
        (v_upper - v_lower) / (config_.low_speed_unknown_v_count() - 1);
    for (int i = 0; i < config_.low_speed_unknown_v_count(); i++) {
      const auto v = v_lower + i * interval;
      target.speed = v;
      SampleCubicVTCurveForEndTimeSpeedAccel(init_point, t, v, 0.0, target);
      ADEBUG << "t:" << t << ", v:" << v
             << ", cruise_target_speed:" << cruise_target_speed;
      break;
    }
  }

  // collect curves
  guide_curve_count_ = CollectCurves(&guide_curve_cost_results_);
}

void ForwardVtSampler::SampleSafeCurves(
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const SLTObstacleCache& obstacle_cache, const bool& set_extend,
    const std::vector<double>& sample_time_extend_time) {
  start_quadratic_vt_curve_index_ = quadratic_vt_curve_count_;
  start_quartic_vt_curve_index_ = quartic_vt_curve_count_;
  start_cubic_vt_curve_index_ = cubic_vt_curve_count_;
  safe_curve_count_ = 0;

  SpeedCurveTarget target;
  target.mode = SpeedCurveTarget::Mode::FOLLOW;
  target.type = SpeedCurveTarget::Type::SAFE;

  const auto* obstacle = obstacle_cache.GetObstacle();
  if (obstacle == nullptr) {
    return;
  }

  target.obstacle_id = obstacle->Id();

  const auto& end_obstacle_info =
      obstacle_cache.GetObstacleInfoAtTime(obstacle_cache.GetMaxT());
  auto sample_times = sample_follow_times_;
  if (set_extend) {
    sample_times = sample_time_extend_time;
  }
  for (const auto& t : sample_times) {
    if (DefinitelyLess(t, obstacle_cache.GetMinT())) {
      continue;
    }

    auto end_s = 0.0;
    auto end_v = 0.0;
    auto end_a = 0.0;
    if (DefinitelyLessEqual(t, obstacle_cache.GetMaxT())) {
      const auto& obstacle_info = obstacle_cache.GetObstacleInfoAtTime(t);
      end_s = obstacle_info.s_lower;
      end_v = obstacle_info.ds;
      end_a = obstacle_info.dds;
    } else {
      const auto dt = t - obstacle_cache.GetMaxT();
      end_s = end_obstacle_info.s_lower + end_obstacle_info.ds * dt +
              0.5 * end_obstacle_info.dds * dt * dt;
      end_v = end_obstacle_info.ds;
      end_a = end_obstacle_info.dds;
    }

    if (DefinitelyLessEqual(end_v, 0.0)) {
      end_a = 0.0;
    }
    double dynamic_time_gap =
        cache.GetFollowTime() -
        fLD::FLAGS_time_gap_relative_velocity_coef * (end_v - init_point.v()) -
        fLD::FLAGS_time_gap_front_accel_coef * end_a * 0.0;
    dynamic_time_gap = std::max(1.0, std::min(dynamic_time_gap, 2.0));
    if (FLAGS_use_dynamic_time_gap) {
      end_s = end_s - end_v * dynamic_time_gap -
              obstacle_cache.GetMinFollowDistance() -
              vehicle_param_.front_edge_to_center();
    } else {
      end_s = end_s - end_v * cache.GetFollowTime() -
              obstacle_cache.GetMinFollowDistance() -
              vehicle_param_.front_edge_to_center();
    }
    SampleQuarticVTCurveForEndTimeLengthSpeedAccel(
        init_point, t, fmax(end_s, 0.0), fmax(end_v, 0.0), end_a, target);
    ADEBUG << "end_t:" << t << ", end_s:" << end_s << ", end_v:" << end_v
           << ", end_a:" << end_a;
  }

  // collect curves
  safe_curve_count_ = CollectCurves(&safe_curve_cost_results_);
  quadratic_vt_curve_count_ = start_quadratic_vt_curve_index_;
  quartic_vt_curve_count_ = start_quartic_vt_curve_index_;
  cubic_vt_curve_count_ = start_cubic_vt_curve_index_;
}

void ForwardVtSampler::SampleNormalCurves(
    const common::TrajectoryPoint& init_point,
    const ReferenceLineInfo& reference_line_info, const SpeedCache& cache,
    const bool& set_extend,
    const std::vector<double>& sample_time_extend_time) {
  start_quadratic_vt_curve_index_ = quadratic_vt_curve_count_;
  start_quartic_vt_curve_index_ = quartic_vt_curve_count_;
  start_cubic_vt_curve_index_ = cubic_vt_curve_count_;
  normal_curve_count_ = 0.0;

  // sample stop curves
  SampleStopCurves(init_point, cache);

  // sample cruise curves
  SampleCruiseCurves(init_point, cache);

  // sample follow curves
  SampleFollowCurves(init_point, reference_line_info, cache, set_extend,
                     sample_time_extend_time);

  // collect curves
  normal_curve_count_ = CollectCurves(&normal_curve_cost_results_);
  quadratic_vt_curve_count_ = start_quadratic_vt_curve_index_;
  quartic_vt_curve_count_ = start_quartic_vt_curve_index_;
  cubic_vt_curve_count_ = start_cubic_vt_curve_index_;
}

bool ForwardVtSampler::AddLastCurve(
    const std::shared_ptr<SpeedCurve>& last_curve, const SpeedCache& cache) {
  if (last_curve == nullptr) {
    return false;
  }

  const auto& last_target = last_curve->GetTarget();

  if (last_target.mode == SpeedCurveTarget::Mode::CRUISE &&
      fabs(last_target.speed -
           cache.GetSpeedLimitCache().cruise_target_speed()) > 0.1) {
    ADEBUG << "last curve is cruise, but cruise target speed changed";
    return false;
  }

  if (last_target.mode == SpeedCurveTarget::Mode::FOLLOW) {
    const auto& obstacle_info =
        cache.GetFollowSTObstacleCache()->GetObstacleInfoAtTime(
            last_curve->GetEndTime() - last_curve->GetStartTime());
    if (fabs(obstacle_info.v - last_curve->GetEndV()) > 2.0) {
      ADEBUG << "last curve is follow, but follow speed changed";
      return false;
    }
  }

  if (last_target.mode == SpeedCurveTarget::Mode::UNKNOWN ||
      last_target.mode == SpeedCurveTarget::Mode::STOP) {
    ADEBUG << "last curve is unknown / stop, but dose not need unknown / stop "
              "curve";
    return false;
  }

  for (const auto& rule : rules_) {
    if (!rule->ApplyRule(last_curve)) {
      ADEBUG << "last curve not satisfied rules";
      return false;
    }
  }
  ADEBUG << "add last curve";
  return true;
}

bool ForwardVtSampler::GenerateFallbackSpeedData(
    const common::TrajectoryPoint& init_point,
    const ReferenceLineInfo& reference_line_info, const SpeedCache& cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator, SpeedData* speed_data) {
  if (speed_data == nullptr || evaluator == nullptr) {
    return false;
  }

  const auto& speed_limit_cache = cache.GetSpeedLimitCache();
  // const auto& vehicle_param =
  //     common::VehicleConfigHelper::GetConfig().vehicle_param();

  // sample acce
  speed_data->clear();
  if (GenerateAccelerateFallbackSpeedData(
          init_point, 0.0, speed_limit_cache.cruise_target_speed(),
          config_.max_accel(), config_.max_jerk(), speed_data) &&
      evaluator->CheckFallbackSpeedData(cache, reference_line_info,
                                        *speed_data)) {
    ADEBUG << "use accelerate fallback speed data";
    return true;
  }

  // sample dece
  speed_data->clear();
  if (GenerateDecelerateFallbackSpeedData(init_point, cache, speed_data)) {
    ADEBUG << "use decelerate fallback speed data";
    return true;
  }
  return false;
}

void ForwardVtSampler::SampleStopCurves(
    const common::TrajectoryPoint& init_point, const SpeedCache& cache) {
  // if adc dose not need stop curve, return
  if (!CheckNeedStopCurve(cache)) {
    return;
  }

  // sample stop curves
  const auto& basic_cache = cache.GetBasicCache();
  SpeedCurveTarget target;
  target.mode = SpeedCurveTarget::Mode::STOP;
  target.type = SpeedCurveTarget::Type::NORMAL;
  target.speed = config_.stop_curve_end_speed();

  const auto expected_stop_s = basic_cache.GetExpectedStopS() -
                               pow(config_.stop_curve_end_speed(), 2) /
                                   fabs(2.0 * config_.stop_curve_end_accel());
  const auto end_s = fmax(expected_stop_s, 0.0);
  const auto end_v = config_.stop_curve_end_speed();
  const auto end_a = config_.stop_curve_end_accel();

  // 1. sample stop curve around last frame stop time
  const auto stop_time =
      GetStopTime() - cache.GetBasicCache().GetTimeFromLastFrame();
  int stop_curve_count = 0;
  if (!std::isinf(stop_time) && stop_time > 0.0) {
    for (int i = -1; i <= 1; i++) {
      stop_curve_count += SampleQuarticVTCurveForEndTimeLengthSpeedAccel(
          init_point, stop_time + i * 0.1, end_s, end_v, end_a, target);
      ADEBUG << "end_t:" << (stop_time + i * 0.1) << ", end_s:" << end_s
             << ", end_v:" << end_v << ", end_a:" << end_a
             << " stop_curve_count: " << stop_curve_count;
    }
  }
  if (stop_curve_count > 0) {
    return;
  }

  // 2. second sample stop curve
  for (const auto& t : sample_stop_times_) {
    stop_curve_count += SampleQuarticVTCurveForEndTimeLengthSpeedAccel(
        init_point, t, end_s, end_v, end_a, target);
    ADEBUG << "end_t:" << t << ", end_s:" << end_s << ", end_v:" << end_v
           << ", end_a:" << end_a << " stop_curve_count: " << stop_curve_count;
  }
  if (stop_curve_count > 0) {
    return;
  }

  // 3. sample const decel stop curve
  SampleConstDecelStopCurve(init_point, end_s, basic_cache.GetFarthestStopS(),
                            target);
}

void ForwardVtSampler::SampleMergeStopCurves(
    const common::TrajectoryPoint& init_point,
    const ReferenceLineInfo& reference_line_info, const SpeedCache& cache) {
  quadratic_vt_curve_count_ = 0;
  quartic_vt_curve_count_ = 0;
  cubic_vt_curve_count_ = 0;
  merge_stop_curve_count_ = 0.0;

  // if adc dose not need stop curve, return
  const auto& basic_cache = cache.GetBasicCache();
  const auto merge_stop_s = basic_cache.GetHighRoadRightEndS();

  ADEBUG << "merge_stop_s:" << merge_stop_s << ", "
         << reference_line_info.path_data().discretized_path().Length();
  if (merge_stop_s > basic_cache.GetExpectedStopS() ||
      merge_stop_s >
          reference_line_info.path_data().discretized_path().Length()) {
    return;
  }

  // sample stop curves
  SpeedCurveTarget target;
  target.mode = SpeedCurveTarget::Mode::STOP;
  target.type = SpeedCurveTarget::Type::NORMAL;
  target.speed = config_.stop_curve_end_speed();

  for (const auto& t : sample_stop_times_) {
    const auto end_s = fmax(merge_stop_s, 0.0);
    const auto end_v = config_.stop_curve_end_speed();
    const auto end_a = config_.stop_curve_end_accel();
    SampleQuarticVTCurveForEndTimeLengthSpeedAccel(init_point, t, end_s, end_v,
                                                   end_a, target);
    ADEBUG << "end_t:" << t << ", end_s:" << end_s << ", end_v:" << end_v
           << ", end_a:" << end_a;
  }

  // collect curves
  merge_stop_curve_count_ = CollectCurves(&merge_stop_curve_cost_results_);
}

void ForwardVtSampler::SampleCruiseCurves(
    const common::TrajectoryPoint& init_point, const SpeedCache& cache) {
  // if adc dose not need cruise curve, return
  if (!CheckNeedCruiseCurve(cache)) {
    return;
  }

  const auto cruise_target_speed =
      cache.GetSpeedLimitCache().cruise_target_speed();
  ADEBUG << "[cruise_speed]" << cruise_target_speed;

  SpeedCurveTarget target;
  target.mode = SpeedCurveTarget::Mode::CRUISE;
  target.speed = cruise_target_speed;

  if (fabs(init_point.v() - cruise_target_speed) < 2.0) {
    int cruise_curve_count = 0;
    for (const auto& end_t : sample_cruise_times_) {
      cruise_curve_count += SampleCubicVTCurveForEndTimeSpeedAccel(
          init_point, end_t, cruise_target_speed, 0.0, target);
    }
    ADEBUG << "cruise_curve_count:" << cruise_curve_count;
  }
}

void ForwardVtSampler::SampleFollowCurves(
    const common::TrajectoryPoint& init_point,
    const ReferenceLineInfo& reference_line_info, const SpeedCache& cache,
    const bool& set_extend,
    const std::vector<double>& sample_time_extend_time) {
  const auto* obstacle_cache = cache.GetFollowSLTObstacleCache();
  if (obstacle_cache == nullptr || !cache.GetNeedFollowCurve()) {
    return;
  }

  const auto& end_obstacle_info =
      obstacle_cache->GetObstacleInfoAtTime(obstacle_cache->GetMaxT());

  SpeedCurveTarget target;
  target.mode = SpeedCurveTarget::Mode::FOLLOW;
  target.type = SpeedCurveTarget::Type::NORMAL;
  auto sample_times = sample_follow_times_;
  if (set_extend && sample_time_extend_time.empty()) {
    return;
  }
  if (set_extend && !sample_time_extend_time.empty()) {
    sample_times = sample_time_extend_time;
  }
  for (const auto& t : sample_times) {
    if (DefinitelyLess(t, obstacle_cache->GetMinT())) {
      continue;
    }

    auto end_s = 0.0;
    auto end_v = 0.0;
    auto end_a = 0.0;
    if (DefinitelyLessEqual(t, obstacle_cache->GetMaxT())) {
      const auto& obstacle_info = obstacle_cache->GetObstacleInfoAtTime(t);
      end_s = obstacle_info.s_lower;
      end_v = obstacle_info.ds;
      end_a = obstacle_info.dds;
    } else {
      const auto dt = t - obstacle_cache->GetMaxT();
      end_s = end_obstacle_info.s_lower + end_obstacle_info.ds * dt +
              0.5 * end_obstacle_info.dds * dt * dt;
      end_v = end_obstacle_info.ds;
      end_a = end_obstacle_info.dds;
    }

    if (DefinitelyLessEqual(end_v, 0.0)) {
      end_a = 0.0;
    }
    double dynamic_time_gap =
        reference_line_info.GetLonCtrlTime() -
        fLD::FLAGS_time_gap_relative_velocity_coef * (end_v - init_point.v()) -
        fLD::FLAGS_time_gap_front_accel_coef * end_a * 0.0;
    dynamic_time_gap = std::max(1.0, std::min(dynamic_time_gap, 2.0));
    if (FLAGS_use_dynamic_time_gap) {
      end_s = end_s - end_v * dynamic_time_gap -
              obstacle_cache->GetMinFollowDistance() -
              vehicle_param_.front_edge_to_center();
    } else {
      end_s = end_s - end_v * reference_line_info.GetLonCtrlTime() -
              obstacle_cache->GetMinFollowDistance() -
              vehicle_param_.front_edge_to_center();
    }
    SampleQuarticVTCurveForEndTimeLengthSpeedAccel(
        init_point, t, fmax(end_s, 0.0), fmax(end_v, 0.0), end_a, target);
    ADEBUG << "end_t:" << t << ", end_s:" << end_s << ", end_v:" << end_v
           << ", end_a:" << end_a;
  }
}

int ForwardVtSampler::SampleQuadraticVTCurveForEndTimeSpeedAccel(
    const common::TrajectoryPoint& init_point, const double t,
    const double end_v, const double end_a, const SpeedCurveTarget& target) {
  const auto quadratic_vt_curve_count_before_sample = quadratic_vt_curve_count_;
  const auto t_count = target.mode == SpeedCurveTarget::Mode::CRUISE ? 20 : 8;
  const auto unit_t = t / t_count;

  std::vector<std::pair<double, double>> params(3);
  if (target.mode == SpeedCurveTarget::Mode::STOP) {
    params.emplace_back(fabs(end_v / end_a), 0.0);
  }
  auto& t1 = params[0].first;
  auto& j1 = params[0].second;
  auto& t2 = params[1].first;
  auto& t3 = params[2].first;
  auto& j3 = params[2].second;

  for (int i1 = 0; i1 <= t_count; ++i1) {
    for (int i2 = 0; i2 <= t_count - i1; ++i2) {
      t1 = i1 * unit_t;
      t2 = i2 * unit_t;
      t3 = (t_count - i1 - i2) * unit_t;
      const auto a1 = t1;
      const auto b1 = t3;
      const auto c1 = end_a - init_point.a();
      const auto a2 = 0.5 * t1 * t1 + t1 * t2 + t1 * t3;
      const auto b2 = 0.5 * t3 * t3;
      const auto c2 = (end_v - init_point.v()) - init_point.a() * t;
      const auto temp = b1 * a2 - b2 * a1;
      if (IsZero(temp)) {
        continue;
      }
      j1 = (b1 * c2 - b2 * c1) / temp;
      j3 = (c1 * a2 - c2 * a1) / temp;
      if (quadratic_vt_curve_count_ >= quadratic_vt_curves_.size() ||
          !quadratic_vt_curves_[quadratic_vt_curve_count_++]
               ->InitPiecewiseJerkCurve(init_point, params, target)) {
        --quadratic_vt_curve_count_;
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
  }
  return static_cast<int>(quadratic_vt_curve_count_ -
                          quadratic_vt_curve_count_before_sample);
}

int ForwardVtSampler::SampleQuarticVTCurveForEndTimeLengthSpeedAccel(
    const common::TrajectoryPoint& init_point, const double end_t,
    const double end_s, const double end_v, const double end_a,
    const SpeedCurveTarget& target) {
  if (quartic_vt_curve_count_ >= quartic_vt_curves_.size() ||
      DefinitelyLessEqual(end_t, 0.0)) {
    return 0;
  }

  if (!quartic_vt_curves_[quartic_vt_curve_count_++]->InitST(
          init_point, end_t, end_s, end_v, end_a, target)) {
    --quartic_vt_curve_count_;
    return 0;
  }
  for (const auto& rule : rules_) {
    if (!rule->ApplyRule(quartic_vt_curves_[quartic_vt_curve_count_ - 1])) {
      --quartic_vt_curve_count_;
      return 0;
    }
  }
  return 1;
}

int ForwardVtSampler::SampleQuadraticVTCurveForStartJerkEndJerk(
    const common::TrajectoryPoint& init_point,
    const std::vector<double>& middle_accels,
    const std::vector<double>& start_jerks,
    const std::vector<double>& end_jerks, const SpeedCurveTarget& target) {
  const auto quadratic_vt_curve_count_before_sample = quadratic_vt_curve_count_;

  std::vector<std::pair<double, double>> params(3);
  auto& t1 = params[0].first;
  auto& j1 = params[0].second;
  auto& t2 = params[1].first;
  auto& t3 = params[2].first;
  auto& j3 = params[2].second;

  static std::vector<double> middle_times{0.0, 1.5, 3.0, 4.5};
  for (const auto& middle_accel : middle_accels) {
    for (const auto& start_jerk : start_jerks) {
      j1 = start_jerk;
      const auto da = middle_accel - init_point.a();
      t1 = (IsZero(da) && IsZero(start_jerk)) ? 0.0 : (da / start_jerk);
      if (!std::isfinite(t1) || DefinitelyLess(t1, 0.0)) {
        continue;
      }

      for (const auto& end_jerk : end_jerks) {
        j3 = end_jerk;
        for (const auto& middle_time : middle_times) {
          t2 = middle_time;
          auto a2 = init_point.a() + j1 * t1;
          if (!IsZero(j3)) {
            t3 = -a2 / j3;
          } else if (IsZero(a2)) {
            t3 = 0.0;
          } else {
            continue;
          }

          if (!DefinitelyGreaterEqual(t3, 0)) {
            continue;
          }

          if (quadratic_vt_curve_count_ >= quadratic_vt_curves_.size() ||
              !quadratic_vt_curves_[quadratic_vt_curve_count_++]
                   ->InitPiecewiseJerkCurve(init_point, params, target)) {
            --quadratic_vt_curve_count_;
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
      }
    }
  }
  return static_cast<int>(quadratic_vt_curve_count_ -
                          quadratic_vt_curve_count_before_sample);
}

int ForwardVtSampler::SampleCubicVTCurveForEndTimeSpeedAccel(
    const common::TrajectoryPoint& init_point, double end_t, double end_v,
    double end_a, const SpeedCurveTarget& target) {
  if (cubic_vt_curve_count_ >= cubic_vt_curves_.size()) {
    return 0;
  }

  if (!cubic_vt_curves_[cubic_vt_curve_count_++]->Init(init_point, end_t, end_v,
                                                       end_a, target)) {
    --cubic_vt_curve_count_;
    return 0;
  }
  for (const auto& rule : rules_) {
    if (!rule->ApplyRule(cubic_vt_curves_[cubic_vt_curve_count_ - 1])) {
      --cubic_vt_curve_count_;
      return 0;
    }
  }
  return 1;
}

int ForwardVtSampler::SampleConstDecelStopCurve(
    const common::TrajectoryPoint& init_point, double end_s,
    double farthest_end_s, const SpeedCurveTarget& target) {
  static constexpr double kMinStopDistance = 0.1;
  if (quadratic_vt_curve_count_ >= quadratic_vt_curves_.size() ||
      DefinitelyLessEqual(init_point.v(), 0.0) ||
      (end_s < kMinStopDistance && farthest_end_s < kMinStopDistance)) {
    return 0;
  }

  const auto min_a =
      fmax(-pow(init_point.v(), 2) / (2.0 * end_s), config_.min_accel());
  const auto max_a =
      fmax(fmin(-pow(init_point.v(), 2) / (2.0 * farthest_end_s), -0.5),
           config_.min_accel());
  std::vector<std::pair<double, double>> params(2);
  if (init_point.a() > max_a) {
    const auto a = max_a;
    params[0].first = (a - init_point.a()) / config_.min_jerk();
    params[0].second = config_.min_jerk();
    const auto end_v = init_point.v() + (init_point.a() + params[0].second / 2 *
                                                              params[0].first) *
                                            params[0].first;
    if (common::math::double_type::DefinitelyLess(end_v, 0.0)) {
      const auto tmp = sqrt(init_point.a() * init_point.a() -
                            2 * params[0].second * init_point.v());
      const auto t1 = (-init_point.a() + tmp) / params[0].second;
      const auto t2 = (-init_point.a() - tmp) / params[0].second;
      params[0].first = t1 > 0.0 ? t1 : t2;
      params[1].first = 0.0;
      params[1].second = 0.0;
    } else {
      params[1].first = -end_v / a;
      params[1].second = 0.0;
    }
    if (quadratic_vt_curve_count_ >= quadratic_vt_curves_.size() ||
        !quadratic_vt_curves_[quadratic_vt_curve_count_++]
             ->InitPiecewiseJerkCurve(init_point, params, target)) {
      return 0;
    }
  } else {
    params[0].first = 0.0;
    params[0].second = 0.0;
    params[1].first = fabs(init_point.v() / init_point.a());
    params[1].second = init_point.a();
    if (quadratic_vt_curve_count_ >= quadratic_vt_curves_.size() ||
        !quadratic_vt_curves_[quadratic_vt_curve_count_++]
             ->InitPiecewiseAccelCurve(init_point, params, params[0].second,
                                       target)) {
      return 0;
    }
  }

  for (const auto& rule : rules_) {
    if (!rule->ApplyRule(quadratic_vt_curves_[quadratic_vt_curve_count_ - 1])) {
      --quadratic_vt_curve_count_;
      return 0;
    }
  }
  return 1;
}

std::vector<double> ForwardVtSampler::SampleCruiseTime(
    const double start_time) const {
  std::vector<double> sample_times;
  double dt = 0.0;
  while (true) {
    if (dt < 1.0) {
      dt += 0.2;
    } else if (dt < 3.0) {
      dt += 0.4;
    } else if (dt < 7.0) {
      dt += 0.6;
    } else if (dt < 20.0) {
      dt += 2.0;
    } else {
      dt += 3.0;
    }
    const auto t = start_time + dt;
    if (DefinitelyGreaterEqual(t, config_.max_curve_time_length())) {
      break;
    }
    sample_times.emplace_back(t);
  }
  return sample_times;
}

void ForwardVtSampler::SampleStopTime() {
  const auto t_count = 100;
  const auto unit_t = 0.5;
  sample_stop_times_.reserve(t_count);
  for (int i = 1; i <= t_count; ++i) {
    sample_stop_times_.emplace_back(i * unit_t);
  }
}

void ForwardVtSampler::SampleCruiseTime() {
  const auto t_count = 50;
  const auto unit_t = 0.25;
  sample_cruise_times_.reserve(t_count);
  for (int i = 1; i <= t_count; ++i) {
    sample_cruise_times_.emplace_back(i * unit_t);
  }
}

void ForwardVtSampler::SampleFollowTime() {
  const auto t_count = 100;
  const auto unit_t = 0.2;
  sample_follow_times_.reserve(t_count);
  for (int i = 1; i <= t_count; ++i) {
    sample_follow_times_.emplace_back(i * unit_t);
  }
}

void ForwardVtSampler::SampleUnknownTime() {
  const auto t_count = 7;
  const auto unit_t = 1.0;
  sample_unknown_times_.reserve(t_count);
  for (int i = 1; i <= t_count; ++i) {
    sample_unknown_times_.emplace_back(i * unit_t);
  }

  const auto dense_t_count = 70;
  const auto dense_unit_t = 0.1;
  sample_dense_unknown_times_.reserve(dense_t_count);
  for (int i = 1; i <= dense_t_count; ++i) {
    sample_dense_unknown_times_.emplace_back(i * dense_unit_t);
  }
}

}  // namespace planning
}  // namespace TL
