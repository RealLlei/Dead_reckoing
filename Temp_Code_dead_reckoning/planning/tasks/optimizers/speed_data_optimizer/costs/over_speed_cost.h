/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file over_speed_cost.h
 **/

#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"

namespace TL {
namespace planning {

/**
 * @class VtSampleCost
 * @brief This class defines the over speed cost calculation method
 */
class OverSpeedCost : public SpeedCost {
 public:
  explicit OverSpeedCost(const SpeedCostConfig& config);
  ~OverSpeedCost() override = default;

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
  // config
  OverSpeedCostConfig config_;
};

}  // namespace planning
}  // namespace TL
