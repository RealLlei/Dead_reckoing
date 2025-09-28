/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file yield_cost.h
 **/

#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"

namespace TL {
namespace planning {

/**
 * @class YieldCost
 * @brief This class defines the vt_sample cost calculation method
 */
class CollisionCost : public SpeedCost {
 public:
  explicit CollisionCost(const SpeedCostConfig& config);
  ~CollisionCost() override = default;

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

  bool CalculateCostWithRisk(const SpeedCache& cache,
                             const ReferenceLineInfo& reference_line_info,
                             SpeedCurveCostResult* cost_result) const;

  bool CalculateCostWithoutRisk(const SpeedCache& cache,
                                const ReferenceLineInfo& reference_line_info,
                                SpeedCurveCostResult* cost_result) const;

  bool CalculateCostWithSafeCheck(const SpeedCache& cache,
                                  const ReferenceLineInfo& reference_line_info,
                                  SpeedCurveCostResult* cost_result) const;

  bool CalculateCostWithContinueCheck(
      const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
      SpeedCurveCostResult* cost_result) const;

  bool CalculateCostWithCrossCheck(const SpeedCache& cache,
                                   const ReferenceLineInfo& reference_line_info,
                                   SpeedCurveCostResult* cost_result) const;

 private:
  // config
  CollisionCostConfig config_;
  common::VehicleParam vehicle_param_;
  double half_width_ = 0.0;
};

}  // namespace planning
}  // namespace TL
