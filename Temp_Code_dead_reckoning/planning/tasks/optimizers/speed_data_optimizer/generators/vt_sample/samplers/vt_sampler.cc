/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_optimizer.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/samplers/vt_sampler.h"

#include <cstddef>
#include <limits>
#include <numeric>

#include "common/configs/config_gflags.h"
#include "common/math/double_type.h"
#include "common/util/point_factory.h"
#include "planning/common/util/common.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/rules/base_limit_rule.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/rules/stop_comfort_rule.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreaterEqual;

VtSampler::VtSampler(const VtSamplerConfig& sampler_config,
                     const SpeedCurveConfig& curve_config)
    : config_(sampler_config) {
  UNUSED(curve_config);
  rules_.emplace_back(std::make_shared<BaseLimitRule>(sampler_config));
}

bool VtSampler::GenerateAccelerateFallbackSpeedData(
    const common::TrajectoryPoint& init_point, const double min_v,
    const double max_v, const double max_acce, const double max_jerk,
    SpeedData* const speed_data) {
  std::vector<std::vector<double>> vec_vec_state;
  if (init_point.v() > max_v ||
      !TL::planning::util::GetStateAtMaxJerk(
          init_point.v(), init_point.a(), min_v, max_v, max_acce, max_jerk,
          FLAGS_trajectory_time_length, FLAGS_trajectory_time_resolution, 0,
          &vec_vec_state)) {
    return false;
  }
  const auto& vec_t = vec_vec_state[0];
  const auto& vec_s = vec_vec_state[1];
  const auto& vec_v = vec_vec_state[2];
  const auto& vec_a = vec_vec_state[3];

  for (std::size_t i = 0; i < vec_t.size(); ++i) {
    speed_data->emplace_back(common::util::PointFactory::ToSpeedPoint(
        vec_s[i], vec_t[i], vec_v[i], vec_a[i],
        FLAGS_longitudinal_jerk_lower_bound));
  }
  return true;
}

bool VtSampler::GenerateDecelerateFallbackSpeedData(
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    SpeedData* speed_data) {
  if (speed_data == nullptr) {
    return false;
  }
  const auto max_step = static_cast<int>(std::round(
      FLAGS_trajectory_time_length / FLAGS_trajectory_time_resolution));

  auto s = 0.0;
  auto v = init_point.v();
  auto a = init_point.a();
  for (int i = 0; i < max_step; ++i) {
    const auto& accel_limit = cache.GetBasicCache().GetSmoothedAccelLimit(v);
    const auto& jerk_limit = cache.GetBasicCache().GetSmoothedJerkLimit(v);
    if (i == 0) {
      speed_data->emplace_back(common::util::PointFactory::ToSpeedPoint(
          s, i * FLAGS_trajectory_time_resolution, v, a,
          FLAGS_longitudinal_jerk_lower_bound));
      continue;
    }
    const auto next_a = common::math::Clamp(
        a + jerk_limit.first * FLAGS_trajectory_time_resolution,
        accel_limit.first, accel_limit.second);
    const auto next_v =
        v + (next_a + a) * FLAGS_trajectory_time_resolution * 0.5;
    if (next_v < 0.0) {
      v = 0.0;
    } else {
      s = s + v * FLAGS_trajectory_time_resolution +
          (2.0 * a + next_a) * pow(FLAGS_trajectory_time_resolution, 2.0) / 6.0;
      v = next_v;
    }
    a = next_a;
    speed_data->emplace_back(common::util::PointFactory::ToSpeedPoint(
        s, i * FLAGS_trajectory_time_resolution, v, a,
        FLAGS_longitudinal_jerk_lower_bound));
  }
  return true;
}

bool VtSampler::GenerateConstDecelSpeedData(const double speed,
                                            const double stop_decel,
                                            const double standstill_decel,
                                            SpeedData* const speed_data) {
  if (DefinitelyGreaterEqual(stop_decel, 0.0) ||
      DefinitelyGreaterEqual(standstill_decel, 0.0)) {
    return false;
  }

  const auto stop_t = fabs(speed / stop_decel);
  const auto stop_s = speed * stop_t / 2;

  const auto t_count = static_cast<int>(
      round(FLAGS_trajectory_time_length / FLAGS_trajectory_time_resolution));
  for (int i = 0; i <= t_count; ++i) {
    const auto t = i * FLAGS_trajectory_time_resolution;
    if (t < stop_t) {
      speed_data->emplace_back(common::util::PointFactory::ToSpeedPoint(
          speed * t - fabs(stop_decel * t * t / 2.0), t,
          speed - fabs(stop_decel * t), stop_decel, 0.0));
    } else {
      speed_data->emplace_back(common::util::PointFactory::ToSpeedPoint(
          stop_s, t, 0.0, standstill_decel, 0.0));
    }
  }
  return true;
}

}  // namespace planning
}  // namespace TL
