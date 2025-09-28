/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file stop_distance_cost.h
 **/

#pragma once

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"

namespace TL {
namespace planning {

/**
 * @class VtSampleCost
 * @brief This class defines the stop distance cost calculation method
 */
class StopDistanceCost : public SpeedCost {
 public:
  explicit StopDistanceCost(const SpeedCostConfig& config);
  ~StopDistanceCost() override = default;

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
   * @brief Calculate speed curve cost
   * 
   * @param cost_result speed cost result
   * @return true this curve can be preserved
   * @return false this curve should be deleted
   */
  bool CalculateCostForStopCurve(const SpeedCache& cache,
                                 SpeedCurveCostResult* cost_result) const;

  /**
   * @brief Calculate speed curve cost
   * 
   * @param cache speed cache
   * @param cost_result speed cost result
   * @return true this curve can be preserved
   * @return false this curve should be deleted
   */
  bool CalculateCostForNonStopCurve(const SpeedCache& cache,
                                    SpeedCurveCostResult* cost_result) const;

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

  static bool CheckUnreasonableSpeed(const std::shared_ptr<SpeedCurve>& curve);

  //
  common::VehicleParam vehicle_param_;
  // config
  StopDistanceCostConfig config_;
  // cost table
  const double speed_epsilon_ = 0.5;
  std::array<std::pair<double, double>, 80> comfort_accel_table_ = {};
};

}  // namespace planning
}  // namespace TL
