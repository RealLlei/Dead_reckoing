/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_cost.h
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
 * @class JerkCost
 * @brief This class defines the jerk cost calculation method
 */
class JerkCost : public SpeedCost {
 public:
  explicit JerkCost(const SpeedCostConfig& config);
  ~JerkCost() override = default;

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
   * @brief Get comfort jerk from table
   *
   * @param speed adc speed
   * @return const std::pair<double, double>&
   */
  const std::pair<double, double>& GetComfortJerk(const double speed) const {
    const auto index =
        common::math::Clamp(static_cast<int>(speed / speed_epsilon_), 0,
                            static_cast<int>(comfort_jerk_table_.size() - 1));
    return comfort_jerk_table_.at(index);
  }

  double JerkCostFunction(double speed, double jerk) const;

  /**
   * @brief Get jerk cost from table
   *
   * @param speed adc speed
   * @param jerk adc jerk
   * @return double
   */
  double GetJerkCost(const double speed, const double jerk) const {
    const auto speed_index =
        common::math::Clamp(static_cast<int>(speed / speed_epsilon_), 0,
                            static_cast<int>(jerk_cost_table_.size() - 1));
    const auto& table = jerk_cost_table_.at(speed_index);
    const auto jerk_index = common::math::Clamp(
        static_cast<int>(jerk / jerk_epsilon_) + jerk_shift_, 0,
        static_cast<int>(table.size() - 1));
    return table[jerk_index];
  }

  // config
  JerkCostConfig config_;
  // cost table
  const double speed_epsilon_ = 0.5;
  const int jerk_shift_ = 100;
  const double jerk_epsilon_ = 0.1;
  std::array<std::array<double, 200>, 80> jerk_cost_table_ = {};
  std::array<std::pair<double, double>, 80> comfort_jerk_table_ = {};
};

}  // namespace planning
}  // namespace TL
