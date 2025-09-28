/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file accel_cost.h
 **/

#pragma once

#include <limits>
#include <utility>
#include <vector>

#include "common/math/math_utils.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"

namespace TL {
namespace planning {

/**
 * @class AccelCost
 * @brief This class defines the accel cost calculation method
 */
class AccelCost : public SpeedCost {
 public:
  explicit AccelCost(const SpeedCostConfig& config);
  ~AccelCost() override = default;

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
  /**
   * @brief Get comfort accel from table
   *
   * @param speed adc speed
   * @return const std::pair<double, double>&
   */
  const std::pair<double, double>& GetComfortAccel(const double speed) const {
    const auto index =
        common::math::Clamp(static_cast<int>(speed / speed_epsilon_), 0,
                            static_cast<int>(comfort_accel_table_.size() - 1));
    return comfort_accel_table_.at(index);
  }

  /**
   * @brief Calculate accel cost
   *
   * @param speed adc speed
   * @param accel adc accel
   * @return double
   */
  double AccelCostFunction(double speed, double accel) const;

  /**
   * @brief Get accel cost from table
   *
   * @param speed adc speed
   * @param accel adc accel
   * @return double
   */
  double GetAccelCost(const double speed, const double accel) const {
    const auto speed_index =
        common::math::Clamp(static_cast<int>(speed / speed_epsilon_), 0,
                            static_cast<int>(accel_cost_table_.size() - 1));
    const auto& table = accel_cost_table_.at(speed_index);
    const auto accel_index = common::math::Clamp(
        static_cast<int>(accel / accel_epsilon_) + accel_shift_, 0,
        static_cast<int>(table.size() - 1));
    return table[accel_index];
  }

  /**
   * @brief Get jerk coef from table
   *
   * @param jerk adc jerk
   * @return double
   */
  double GetJerkCoef(const double jerk) const {
    const auto jerk_index = common::math::Clamp(
        static_cast<int>(jerk / jerk_epsilon_) + jerk_shift_, 0,
        static_cast<int>(jerk_coef_table_.size() - 1));
    return jerk_coef_table_.at(jerk_index);
  }

  // config
  AccelCostConfig config_;
  // cost table
  const double speed_epsilon_ = 0.5;
  const int accel_shift_ = 500;
  const double accel_epsilon_ = 0.01;
  const int jerk_shift_ = 500;
  const double jerk_epsilon_ = 0.01;
  std::array<std::array<double, 1000>, 80> accel_cost_table_ = {};
  std::array<std::pair<double, double>, 80> comfort_accel_table_ = {};
  std::array<double, 1000> jerk_coef_table_ = {};
};

}  // namespace planning
}  // namespace TL
