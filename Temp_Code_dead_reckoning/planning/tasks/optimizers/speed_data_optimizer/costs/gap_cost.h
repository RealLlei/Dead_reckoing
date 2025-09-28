/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file gap_cost.h
 **/

#pragma once

#include <limits>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"

namespace TL {
namespace planning {

/**
 * @class VtSampleCost
 * @brief This class defines the gap cost calculation method
 */
class GapCost : public SpeedCost {
 public:
  explicit GapCost(const SpeedCostConfig& config);
  ~GapCost() override = default;

    bool Updatable() const override { return true; }

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
  GapCostConfig config_;
  common::VehicleParam vehicle_param_;
};

}  // namespace planning
}  // namespace TL
