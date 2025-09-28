/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file stop_comfort_rule.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/rules/stop_comfort_rule.h"
#include "common/util/macros.h"

namespace TL {
namespace planning {

StopComfortRule::StopComfortRule(const VtSamplerConfig& config)
    : VtSampleRule(config) {
  if (config.has_forward_vt_sampler_config()) {
    const auto& forward_vt_sampler_config = config.forward_vt_sampler_config();
    average_accel_threshold_for_stop_curve_ =
        forward_vt_sampler_config.average_accel_threshold_for_stop_curve();
    stop_curve_end_accel_ = forward_vt_sampler_config.stop_curve_end_accel();
  } else if (config.has_reverse_vt_sampler_config()) {
    const auto& reverse_vt_sampler_config = config.reverse_vt_sampler_config();
    average_accel_threshold_for_stop_curve_ =
        reverse_vt_sampler_config.average_accel_threshold_for_stop_curve();
    stop_curve_end_accel_ = reverse_vt_sampler_config.stop_curve_end_accel();
  }
}

bool StopComfortRule::ApplyRule(
    const std::shared_ptr<SpeedCurve>& curve) const {
  if (curve->GetTarget().mode != SpeedCurveTarget::Mode::STOP) {
    return true;
  }

  // check average accel
  const auto average_accel =
      -0.5 * pow(curve->GetStartV(), 2) / fmax(curve->GetEndS(), 1e-5);

  return average_accel < average_accel_threshold_for_stop_curve_;
}

void StopComfortRule::Update(const common::TrajectoryPoint& init_point,
                             const SpeedCache& cache) {
  UNUSED(init_point);
  UNUSED(cache);
}

}  // namespace planning
}  // namespace TL
