/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file curve_priority_cost.h
 **/

#pragma once

#include <limits>

#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"

namespace TL {
namespace planning {

/**
 * @class CurvePriorityCost
 * @brief This class defines the curve_priority cost calculation method
 */
class CurvePriorityCost : public SpeedCost {
 public:
  explicit CurvePriorityCost(const SpeedCostConfig& config);
  ~CurvePriorityCost() override = default;

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
  CurvePriorityCostConfig config_;
};

}  // namespace planning
}  // namespace TL
