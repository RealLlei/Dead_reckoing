/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  at_sampler.cc
 */
#include "planning/tasks/deciders/obstacles_decider/sampler/at_sampler.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/file/log.h"
#include "planning/math/curve1d/quadratic_polynomial_curve1d.h"

namespace TL::planning {
namespace {
constexpr double kSampleTimeLength = 7.0;
constexpr double kEpsilon = 1.0e-3;
}  // namespace

AtSampler::AtSampler(
    const ObstaclesDeciderConfig_AtSamplerConfig& at_sampler_config) {
  at_sampling_strategies_.clear();
  if (at_sampler_config.has_enable_debug()) {
    enable_debug_ = at_sampler_config.enable_debug();
  }
  for (const auto sample_accel : at_sampler_config.sample_accel()) {
    for (const auto sample_time : at_sampler_config.sample_time()) {
      const double jerk = -sample_accel / std::fmax(kEpsilon, sample_time);
      const auto sample_first_segment = std::make_pair(sample_time, jerk);
      const auto sample_second_segment =
          std::make_pair(std::fmax(kSampleTimeLength - sample_time, 0.0), 0.0);
      std::vector<std::pair<double, double>> at_sampling_strategy;
      at_sampling_strategy.emplace_back(sample_first_segment);
      at_sampling_strategy.emplace_back(sample_second_segment);
      at_sampling_strategies_.emplace_back(at_sampling_strategy);
    }
  }
}

void AtSampler::Init(const std::array<double, 3>& init_lon_state) {
  init_lon_state_ = init_lon_state;
}

void AtSampler::GenerateLongitudinalTrajectoryBundle(
    std::vector<std::shared_ptr<QuadraticVTCurve>>* const vt_curves) {
  if (vt_curves == nullptr) {
    return;
  }
  vt_curves->clear();
  const double init_v = init_lon_state_.at(1);

  // 两个变量没有实际用处，仅仅是为了构造二次速度曲线的入参
  SpeedCurveTarget target;
  SpeedCurveConfig config;
  vt_curves->reserve(at_sampling_strategies_.size());
  for (const auto& at_sampling_strategy : at_sampling_strategies_) {
    // 每种采样策略包含两段速递采样
    if (at_sampling_strategy.size() != 2) {
      continue;
    }
    auto first_speed_segment = at_sampling_strategy.at(0);
    auto second_speed_segment = at_sampling_strategy.at(1);

    // 初始加速度为负数时，计算第一段速度曲线速度为0的时间点，若时间在第一段速度曲线时间内，则车辆已停止，后续时间不可继续减速
    const double init_accel =
        -first_speed_segment.first * first_speed_segment.second;
    if (init_accel < 0.0) {
      // 构造第一段速度曲线
      const double init_jerk = first_speed_segment.second;
      const auto first_segment_speed_curve =
          QuadraticPolynomialCurve1d({init_v, init_accel, 0.5 * init_jerk});
      // 二次多项式求根
      std::vector<double> time_to_stop;
      if (first_segment_speed_curve.Solve(&time_to_stop) &&
          !time_to_stop.empty()) {
        const double first_speed_segment_time = first_speed_segment.first;
        if (time_to_stop[0] > 0.0 &&
            time_to_stop[0] < first_speed_segment_time) {
          first_speed_segment.first = time_to_stop[0];
          first_speed_segment.second =
              -init_accel / std::fmax(kEpsilon, time_to_stop[0]);
          second_speed_segment.first =
              std::fmax(kSampleTimeLength - time_to_stop[0], 0.0);
        }
      }
    }

    std::shared_ptr<QuadraticVTCurve> vt_curve =
        std::make_shared<QuadraticVTCurve>(config);
    common::TrajectoryPoint init_point;
    init_point.set_v(init_v);
    init_point.set_a(-first_speed_segment.first * first_speed_segment.second);
    ADEBUG << "first vt segment t:" << first_speed_segment.first
           << ", jerk:" << first_speed_segment.second
           << ", second vt segment t:" << second_speed_segment.first
           << ", jerk:" << second_speed_segment.second
           << ", init point v:" << init_point.v()
           << ", init point a:" << init_point.a();
    if (vt_curve->InitPiecewiseJerkCurve(init_point, at_sampling_strategy,
                                         target)) {
      if (enable_debug_) {
        constexpr int debug_steps = 10;
        for (int i = 0; i < debug_steps; ++i) {
          ADEBUG << "Now is " << i << "s"
                 << ", v:" << vt_curve->GetPoint(i).v()
                 << ", a:" << vt_curve->GetPoint(i).a()
                 << ", j:" << vt_curve->GetPoint(i).j()
                 << ", s:" << vt_curve->GetPoint(i).s();
        }
      }
      vt_curves->push_back(std::move(vt_curve));
    }
  }
}

}  // namespace TL::planning
