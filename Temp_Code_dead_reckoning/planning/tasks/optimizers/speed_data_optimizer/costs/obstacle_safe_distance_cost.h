/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file obstacle_safe_distance_cost.h
 **/

#pragma once

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "common/math/math_utils.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/slt_obstacle_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_limit_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"

namespace TL {
namespace planning {

/**
 * @class ObstacleSafeDistanceCost
 * @brief This class defines the vt_sample cost calculation method
 */
class ObstacleSafeDistanceCost : public SpeedCost {
 public:
  explicit ObstacleSafeDistanceCost(const SpeedCostConfig& config);
  ~ObstacleSafeDistanceCost() override = default;

  /**
   * @brief Whether the cost can be update after calculation
   * 
   * @return true 
   * @return false 
   */
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

  bool CheckIsLongitudinalSafe(const SpeedData& speed_data,
                               const SLTObstacleCache& obstacle_cache) const;

 private:
  double CalculateSSafeDistanceCost(const SpeedCurvePoint& point,
                                    const SLTObstacleCache& obstacle_cache,
                                    const STObstacleLocation& obstacle_location,
                                    double* min_follow_safe_cost,
                                    double* min_overtake_safe_cost) const;

  double CalculateLSafeDistanceCost(const SpeedCurvePoint& point,
                                    const SLTObstacleCache& obstacle_cache,
                                    SpeedCurveCostResult* cost_result) const;

  /**
   * @brief Calculate cost for safe curve
   * 
   * @param cache 
   * @param reference_line_info 
   * @param cost_result 
   * @return true 
   * @return false 
   */
  bool CalculateCostForSafeCurve(const SpeedCache& cache,
                                 const ReferenceLineInfo& reference_line_info,
                                 SpeedCurveCostResult* cost_result) const;

  /**
   * @brief Calculate cost for obstacle without decision for other curve
   * 
   * @param cache 
   * @param reference_line_info 
   * @param cost_result 
   * @return true 
   * @return false 
   */
  bool CalculateCostWithoutDecisionForOtherCurve(
      const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
      SpeedCurveCostResult* cost_result) const;

  /**
   * @brief Calculate cost for obstacle with decision other curve
   * 
   * @param cache 
   * @param reference_line_info 
   * @param cost_result 
   * @return true 
   * @return false 
   */
  bool CalculateCostWithDecisionForOtherCurve(
      const SpeedCache& cache, const ReferenceLineInfo& reference_line_info,
      SpeedCurveCostResult* cost_result) const;

  /**
   * @brief Get follow distance cost from table
   *
   * @param obstacle_distance obstacle distance = (exptected_distance -
   * real_distance) / exptected_distance
   * @return double
   */
  [[nodiscard]] double GetFollowDistanceCost(
      const double obstacle_distance) const {
    const auto index = common::math::Clamp(
        static_cast<int>(obstacle_distance / obstacle_distance_epsilon_), 0,
        static_cast<int>(follow_distance_cost_table_.size() - 1));
    return follow_distance_cost_table_.at(index);
  }

  /**
   * @brief Get overtake distance cost from table
   *
   * @param obstacle_distance obstacle distance = (exptected_distance -
   * real_distance) / exptected_distance
   * @return double
   */
  [[nodiscard]] double GetOvertakeDistanceCost(
      const double obstacle_distance) const {
    const auto index = common::math::Clamp(
        static_cast<int>(obstacle_distance / obstacle_distance_epsilon_), 0,
        static_cast<int>(overtake_distance_cost_table_.size() - 1));
    return overtake_distance_cost_table_.at(index);
  }

  /**
   * @brief Calculate follow distance cost
   *
   * @param obstacle_distance obstacle distance = (exptected_distance -
   * real_distance) / exptected_distance
   * @param config vt sample config
   */
  [[nodiscard]] double FollowDistanceCostFunction(double distance) const;

  /**
   * @brief Calculate overtake distance cost
   *
   * @param obstacle_distance obstacle distance = (exptected_distance -
   * real_distance) / exptected_distance
   * @param config vt sample config
   */
  [[nodiscard]] double OvertakeDistanceCostFunction(double distance) const;

  // config
  ObstacleSafeDistanceCostConfig config_;
  // obstacle distance table
  const double obstacle_distance_epsilon_ = 0.001;
  std::array<double, 1000> follow_distance_cost_table_ = {};
  std::array<double, 1000> overtake_distance_cost_table_ = {};

  common::VehicleParam vehicle_param_;
  double half_width_ = 0.0;
};

}  // namespace planning
}  // namespace TL
