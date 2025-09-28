/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file nudge_cost.h
 **/

#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"

namespace TL {
namespace planning {

/**
 * @class VtSampleCost
 * @brief This class defines the vt_sample cost calculation method
 */
class NudgeCost : public SpeedCost {
 public:
  explicit NudgeCost(const SpeedCostConfig& config);
  ~NudgeCost() override = default;

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

  double CalculateFollowTimeCost(const SpeedCache& cache,
                                 const ReferenceLineInfo& reference_line_info,
                                 double follow_time,
                                 double user_follow_time) const;

 private:
  // void CalculateNudgeSafeCost(const SpeedCache& cache,
  //                             const ReferenceLineInfo& reference_line_info,
  //                             SpeedCurveCostResult* cost_result) const;

  double BigCarDistanceCostFunction(double distance) const;

  /**
   * @brief Calculate obstacle distance cost
   *
   * @param obstacle_distance obstacle distance = (exptected_distance -
   * real_distance) / exptected_distance
   * @param config vt sample config
   */
  double ObstacleDistanceCostFunction(double distance) const;

  /**
   * @brief Get big car distance cost from table
   *
   * @param distance big car distance
   * @return double
   */
  double GetBigCarDistanceCost(const double distance) const {
    const auto index = common::math::Clamp(
        static_cast<int>(distance / big_car_distance_epsilon_), 0,
        static_cast<int>(big_car_distance_cost_table_.size() - 1));
    return big_car_distance_cost_table_.at(index);
  }

  /**
   * @brief Get obstacle distancle cost from table
   *
   * @param obstacle_distance obstacle distance = (exptected_distance -
   * real_distance) / exptected_distance
   * @return double
   */
  double GetObstacleDistanceCost(const double obstacle_distance) const {
    const auto index = common::math::Clamp(
        static_cast<int>(obstacle_distance / obstacle_distance_epsilon_), 0,
        static_cast<int>(obstacle_distance_cost_table_.size() - 1));
    return obstacle_distance_cost_table_.at(index);
  }

  // config
  NudgeCostConfig config_;
  // big car cost table
  const double big_car_distance_epsilon_ = 0.1;
  std::array<double, 200> big_car_distance_cost_table_ = {};

  // obstacle distance table
  const double obstacle_distance_epsilon_ = 0.01;
  std::array<double, 100> obstacle_distance_cost_table_ = {};

  common::VehicleParam vehicle_param_;
};

}  // namespace planning
}  // namespace TL
