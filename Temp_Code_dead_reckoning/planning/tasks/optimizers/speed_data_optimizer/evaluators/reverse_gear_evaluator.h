/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_cost.h
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
 * @class ReverseSpeedEvaluator
 * @brief This class defines the reverse speed curve evaluation method
 */
class alignas(CACHELINE_SIZE) ReverseGearEvaluator : public SpeedEvaluator {
 public:
  explicit ReverseGearEvaluator(const SpeedEvaluatorConfig& config);

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
  static bool CompareCurrentCostResult(
      const SpeedCurveCostResult* cost_result1,
      const SpeedCurveCostResult* cost_result2);

  static bool CompareCurrentAndLastCostResult(
      const SpeedCurveCostResult* current_cost_result,
      const SpeedCurveCostResult* last_cost_result);

  ReverseGearEvaluatorConfig config_;
};

}  // namespace planning
}  // namespace TL
