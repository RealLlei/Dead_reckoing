/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file base_limit_rule.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/rules/base_limit_rule.h"
#include <cmath>

#include "common/configs/vehicle_config_helper.h"
#include "common/math/double_type.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_curve.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreater;
using common::math::double_type::DefinitelyLess;

BaseLimitRule::BaseLimitRule(const VtSamplerConfig& config)
    : VtSampleRule(config),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
  if (config.has_forward_vt_sampler_config()) {
    const auto& forward_vt_sampler_config = config.forward_vt_sampler_config();
    max_curve_time_length_ = forward_vt_sampler_config.max_curve_time_length();
    min_accel_ = forward_vt_sampler_config.min_accel();
    max_accel_ = forward_vt_sampler_config.max_accel();
    min_jerk_ = forward_vt_sampler_config.min_jerk();
    max_jerk_ = forward_vt_sampler_config.max_jerk();
  } else if (config.has_reverse_vt_sampler_config()) {
    const auto& reverse_vt_sampler_config = config.reverse_vt_sampler_config();
    max_curve_time_length_ = reverse_vt_sampler_config.max_curve_time_length();
    min_accel_ = reverse_vt_sampler_config.min_accel();
    max_accel_ = reverse_vt_sampler_config.max_accel();
    min_jerk_ = reverse_vt_sampler_config.min_jerk();
    max_jerk_ = reverse_vt_sampler_config.max_jerk();
  }
}

bool BaseLimitRule::ApplyRule(const std::shared_ptr<SpeedCurve>& curve) const {
  // curve time length must less then max_curve_time_length
  if (curve->GetTimeLength() > max_curve_time_length_) {
    return false;
  }

  // curve speed must > 0
  if (curve->GetMinV() < -1e-6) {
    return false;
  }

  // jerk must in [min_jerk_,  min_jerk_]
  if (DefinitelyGreater(curve->GetMaxJerk(), max_jerk_) ||
      DefinitelyLess(curve->GetMinJerk(), min_jerk_)) {
    return false;
  }

  // accel must in [min_accel_, max_accel_]
  if ((curve->GetTarget().mode != SpeedCurveTarget::Mode::FOLLOW &&
       DefinitelyGreater(curve->GetMaxAccel(), max_accel_) &&
       DefinitelyLess(curve->GetStartAccel(), curve->GetMaxAccel())) ||
      (DefinitelyLess(curve->GetMinAccel(), min_accel_) &&
       DefinitelyGreater(curve->GetStartAccel(), curve->GetMinAccel()))) {
    return false;
  }

  return true;
}

void BaseLimitRule::Update(const common::TrajectoryPoint& init_point,
                           const SpeedCache& cache) {
  const auto& accel_limit =
      cache.GetBasicCache().GetAccelLimit(fabs(init_point.v()));
  min_accel_ = accel_limit.first;
  max_accel_ = accel_limit.second;
}

}  // namespace planning
}  // namespace TL
