/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_curve_evaluator.h
 **/

#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"

namespace TL {
namespace planning {

/**
 * @class SpeedEvaluator
 * @brief This class defines the speed curve evaluation method
 */
class SpeedEvaluator {
 public:
  explicit SpeedEvaluator(const SpeedEvaluatorConfig& config);
  virtual ~SpeedEvaluator() = default;

  /**
   * @brief Get the Speed Costs object
   * 
   * @return const std::vector<std::shared_ptr<SpeedCost>>& 
   */
  const std::vector<std::shared_ptr<SpeedCost>>& GetSpeedCosts() const {
    return speed_costs_;
  }

  /**
   * @brief Set the Merge Stop object
   * 
   */
  void SetMergeStop(bool is_merge_stop);

  /**
   * @brief Calculate speed curve cost
   * 
   * @param cache speed cache
   * @param reference_line_info current reference line info
   * @param cost_result speed curve cost result
   */
  virtual void CalculateCost(const SpeedCache& cache,
                             const ReferenceLineInfo& reference_line_info,
                             SpeedCurveCostResult* cost_result) const = 0;

  /**
   * @brief Check whether fallback speed data is valid
   * 
   * @param cache speed cache
   * @param reference_line_info current reference line info
   * @param speed_data speed data
   * @return true 
   * @return false 
   */
  virtual bool CheckFallbackSpeedData(
      const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
      const SpeedData& speed_data) const = 0;

  /**
   * @brief Compare two speed curve cost result
   *
   * @param cost_result1
   * @param cost_result2
   * @return true cost_result1 < cost_result2
   * @return false cost_result1 >= cost_result2
   */
  virtual bool CompareCostResult(
      const SpeedCurveCostResult* cost_result1,
      const SpeedCurveCostResult* cost_result2) const = 0;

  /**
   * @brief Reset to front obstacle cost
   * 
   * @param cost_result 
   */
  static void ResetToFrontCost(SpeedCurveCostResult* cost_result);
  /**
   * @brief 
   * 
   * @param cache 
   * @param reference_line_info 
   * @param cost_result 
   * @return true 
   * @return false 
   */
  bool CalculateMergeStopDistanceCost(
      const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
      SpeedCurveCostResult* cost_result);

 protected:
  /**
   * @brief Reset cost
   *
   * @param cost_result speed curve cost result
   */
  static void ResetCost(SpeedCurveCostResult* cost_result);

  SpeedEvaluatorConfig speed_evaluator_config_;
  std::vector<std::shared_ptr<SpeedCost>> speed_costs_;
  std::shared_ptr<SpeedCost> stop_distance_cost_evaluator_ = nullptr;
};

}  // namespace planning
}  // namespace TL
