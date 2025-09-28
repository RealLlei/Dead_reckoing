/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file highway_speed_curve_evaluator.h
 **/

#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator.h"

#include "planning/proto/st_drivable_boundary.pb.h"
#include "planning/proto/task_config.pb.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

/**
 * @class HighwaySpeedEvaluator
 * @brief This class defines the highway speed curve evaluation method
 */
class alignas(CACHELINE_SIZE) LaneKeepEvaluator : public SpeedEvaluator {
 public:
  explicit LaneKeepEvaluator(const SpeedEvaluatorConfig& config);

  /**
   * @brief Calculate speed curve cost
   * 
   * @param cache speed cache
   * @param reference_line_info current reference line info
   * @param cost_result speed curve cost result
   */
  void CalculateCost(const SpeedCache& cache,
                     const ReferenceLineInfo& reference_line_info,
                     SpeedCurveCostResult* cost_result) const override;

  /**
   * @brief Check whether fallback speed data is valid
   * 
   * @param cache speed cache
   * @param reference_line_info current reference line info
   * @param speed_data speed data
   * @return true 
   * @return false 
   */
  bool CheckFallbackSpeedData(const SpeedCache& cache,
                              const ReferenceLineInfo& reference_line_info,
                              const SpeedData& speed_data) const override;

  /**
   * @brief Compare two current curve cost result
   *
   * @param cost_result1
   * @param cost_result2
   * @return true cost_result1 < cost_result2
   * @return false cost_result1 >= cost_result2
   */
  bool CompareCostResult(
      const SpeedCurveCostResult* cost_result1,
      const SpeedCurveCostResult* cost_result2) const override;

 private:
  /**
   * @brief Calculate mode cost
   * 
   * @param cost_result1 
   * @param cost_result2 
   * @param mode_cost1 
   * @param mode_cost2 
   */
  static void CalculateModeCost(const SpeedCurveCostResult* cost_result1,
                                const SpeedCurveCostResult* cost_result2,
                                double* mode_cost1, double* mode_cost2);

  /**
   * @brief Calculate total cost
   * 
   * @param cost_result1 
   * @param cost_result2 
   * @param total_cost1 
   * @param total_cost2 
   */
  void CalculateTotalCost(const SpeedCurveCostResult* cost_result1,
                          const SpeedCurveCostResult* cost_result2,
                          double* total_cost1, double* total_cost2) const;

  bool CompareCurrentCostResult(const SpeedCurveCostResult* cost_result1,
                                const SpeedCurveCostResult* cost_result2) const;

  bool CompareCurrentAndLastCostResult(
      const SpeedCurveCostResult* current_cost_result,
      const SpeedCurveCostResult* last_cost_result) const;

  LaneKeepEvaluatorConfig config_;
};

}  // namespace planning
}  // namespace TL
