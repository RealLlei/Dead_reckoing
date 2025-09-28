/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file stop_comfort_rule.h
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
 * @class StopLimitRule
 * @brief this class is used to filter curves
 */
class StopComfortRule : public VtSampleRule {
 public:
  /**
   * @brief Construct a new Stop Comfort Rule object
   *
   * @param config vt sample config
   */
  explicit StopComfortRule(const VtSamplerConfig& config);
  ~StopComfortRule() override = default;

  [[nodiscard]] bool ApplyRule(
      const std::shared_ptr<SpeedCurve>& curve) const override;

  void Update(const common::TrajectoryPoint& init_point,
              const SpeedCache& cache) override;

 private:
  double average_accel_threshold_for_stop_curve_ = 0.0;
  double stop_curve_end_accel_ = 0.0;
};

}  // namespace planning
}  // namespace TL
