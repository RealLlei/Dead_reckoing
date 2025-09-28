/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file base_limit_rule.h
 **/

#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/rules/vt_sample_rule.h"

namespace TL {
namespace planning {

/**
 * @class BaseLimitRule
 * @brief this class is used to filter curves
 */
class BaseLimitRule : public VtSampleRule {
 public:
  /**
   * @brief Construct a new Base Limit Rule object
   * 
   * @param config vt sample config
   */
  explicit BaseLimitRule(const VtSamplerConfig& config);
  ~BaseLimitRule() override = default;

  [[nodiscard]] bool ApplyRule(
      const std::shared_ptr<SpeedCurve>& curve) const override;

  void Update(const common::TrajectoryPoint& init_point,
              const SpeedCache& cache) override;

 private:
  const TL::common::VehicleParam& vehicle_param_;
  double max_curve_time_length_ = 0;
  double min_accel_ = 0.0;
  double max_accel_ = 0.0;
  double min_jerk_ = 0.0;
  double max_jerk_ = 0.0;
};

}  // namespace planning
}  // namespace TL
