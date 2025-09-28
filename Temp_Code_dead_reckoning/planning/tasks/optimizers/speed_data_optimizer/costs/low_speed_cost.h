/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file low_speed_cost.h
 **/

#pragma once

#include <limits>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"

namespace TL {
namespace planning {

/**
 * @class LowSpeedCost
 * @brief This class defines the low speed cost calculation method
 */
class LowSpeedCost : public SpeedCost {
 public:
  explicit LowSpeedCost(const SpeedCostConfig& config);
  ~LowSpeedCost() override = default;

  /**
   * @brief Calculate speed curve cost
   * 
   * @param cache speed cache
   * @param reference_line_info current reference line info
   * @param cost_result speed cost result
   * @return true this curve can be preserved
   * @return false this curve should be deleted
   */
  bool CalculateCost(const SpeedCache& cache,
                     const ReferenceLineInfo& reference_line_info,
                     SpeedCurveCostResult* cost_result) const override;

 private:
  //
  common::VehicleParam vehicle_param_;
  // config
  LowSpeedCostConfig config_;
};

}  // namespace planning
}  // namespace TL
