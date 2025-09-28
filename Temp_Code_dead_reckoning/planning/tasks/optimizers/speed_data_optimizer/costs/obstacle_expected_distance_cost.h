/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file obstacle_expected_distance_cost.h
 **/

#pragma once

#include <array>
#include <limits>
#include <utility>
#include <vector>

#include "common/math/math_utils.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"

namespace TL {
namespace planning {

/**
 * @class VtSampleCost
 * @brief This class defines the obstacle expected distance cost calculation method
 */
class ObstacleExpectedDistanceCost : public SpeedCost {
 public:
  explicit ObstacleExpectedDistanceCost(const SpeedCostConfig& config);
  ~ObstacleExpectedDistanceCost() override = default;

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

  void SetFollowTime(double follow_time);

 private:
  void LoadLessExpectedDistanceCostCoefTable();
  void LoadGreaterExpectedDistanceCostCoefTable();
  void LoadLessExpectedDistanceCostTable();
  void LoadTTCCostTable();

  [[nodiscard]] double GetLessExpectedDistanceCostCoefAtSpeed(
      const double speed) const {
    const auto index = common::math::Clamp(
        static_cast<int>(speed / speed_epsilon_), 0,
        static_cast<int>(less_expected_distance_cost_coef_table_.size() - 1));
    return less_expected_distance_cost_coef_table_.at(index);
  }

  [[nodiscard]] double GetGreaterExpectedDistanceCostCoefAtSpeed(
      const double speed) const {
    const auto index = common::math::Clamp(
        static_cast<int>(speed / speed_epsilon_), 0,
        static_cast<int>(greater_expected_distance_cost_coef_table_.size() -
                         1));
    return greater_expected_distance_cost_coef_table_.at(index);
  }

  [[nodiscard]] double GetLessExpectedDistanceCost(
      const double less_expected_distance_error) const {
    const auto index = common::math::Clamp(
        static_cast<int>(less_expected_distance_error /
                         expected_distance_error_epsilon_),
        0,
        static_cast<int>(current_less_expected_distance_cost_table_->size() -
                         1));
    return current_less_expected_distance_cost_table_->at(index);
  }

  [[nodiscard]] double GetTTCCost(const double speed, const double ttc) const {
    const auto speed_index =
        common::math::Clamp(static_cast<int>(speed / speed_epsilon_), 0,
                            static_cast<int>(ttc_cost_coef_table_.size() - 1));

    const auto ttc_index =
        common::math::Clamp(static_cast<int>(ttc / ttc_epsilon_), 0,
                            static_cast<int>(ttc_cost_table_.size() - 1));
    return ttc_cost_coef_table_.at(speed_index) * ttc_cost_table_.at(ttc_index);
  }

  // config
  ObstacleExpectedDistanceCostConfig config_;

  // vehicle param
  common::VehicleParam vehicle_param_;

  // expected distance cost coef table
  const double speed_epsilon_ = 0.1;
  std::array<double, 500> less_expected_distance_cost_coef_table_ = {};
  std::array<double, 500> greater_expected_distance_cost_coef_table_ = {};

  // less expected distance cost table
  const double expected_distance_error_epsilon_ = 0.01;
  std::vector<std::pair<double, std::array<double, 1000>>>
      less_expected_distance_cost_tables_;
  const std::array<double, 1000>* current_less_expected_distance_cost_table_;

  // ttc cost table
  const double ttc_epsilon_ = 0.01;
  std::array<double, 500> ttc_cost_coef_table_ = {};
  std::array<double, 1000> ttc_cost_table_ = {};
};

}  // namespace planning
}  // namespace TL
