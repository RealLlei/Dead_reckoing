/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_cost.h
 **/

#pragma once

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/st_obstacle_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_curve.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

struct ObstacleCost {
  // collision cost
  double collision_cost = 0.0;
  // non follow safe distance cost
  double non_follow_safe_distance_cost = 0.0;
  // obstacle safe distance cost
  double obstacle_safe_distance_cost = 0.0;
  // obstacle less expected distance_cost
  double obstacle_less_expected_distance_cost = 0.0;
  // obstacle greater expected distance_cost
  double obstacle_greater_expected_distance_cost = 0.0;
  // lateral distance cost
  double lateral_distance_cost = 0.0;
  // yield cost
  double yield_cost = 0.0;
  // mean less follow time error
  double mean_less_follow_time_error = 0.0;
  // mean greater follow time error
  double mean_greater_follow_time_error = 0.0;
};

/**
 * @brief speed cost result
 */
struct alignas(CACHELINE_SIZE) SpeedCurveCostResult {
  // curve pointer
  std::shared_ptr<SpeedCurve> curve = nullptr;
  //
  bool already_calculated = false;
  // non follow safe distance cost
  double non_follow_safe_distance_cost = 0.0;
  // obstacle safe distance cost
  double obstacle_safe_distance_cost = 0.0;
  // obstacle safe distance cost at every time
  std::vector<double> obstacle_s_safe_distance_costs = {};
  // obstacle safe distance cost at every time
  std::vector<double> obstacle_l_safe_distance_costs = {};
  // lateral distance cost
  double lateral_distance_cost = 0.0;
  // accel cost
  double accel_cost = 0.0;
  // jerk cost
  double jerk_cost = 0.0;
  // low speed cost
  double low_speed_cost = 0.0;
  // over curvature speed limit cost
  double over_curvature_speed_limit_cost = 0.0;
  // over decision speed limit cost
  double over_decision_speed_limit_cost = 0.0;
  // over map speed limit cost
  // double over_map_speed_limit_cost = 0.0;
  // // over cruise speed limit cost
  // double over_cruise_speed_limit_cost = 0.0;
  // over comfortable speed limit cost
  double over_comfortable_speed_limit_cost = 0.0;
  // over ramp speed limit cost
  // double over_ramp_speed_limit_cost = 0.0;
  // // over tunnel speed limit cost
  // double over_tunnel_speed_limit_cost = 0.0;
  // over critical speed limit cost
  double over_critical_speed_limit_cost = 0.0;
  // over tunnel speed limit cost
  double over_nudge_speed_limit_cost = 0.0;
  // over speed cost
  double over_speed_cost = 0.0;
  // over speed cost for safe
  double safe_over_speed_cost = 0.0;
  // stop distance cost
  double stop_distance_cost = 0.0;
  // gap cost
  double gap_cost = 0.0;
  // parallel_drive_cost
  double parallel_drive_cost = 0.0;
  // obstacle less expected distance_cost
  double obstacle_less_expected_distance_cost = 0.0;
  // obstacle greater expected distance_cost
  double obstacle_greater_expected_distance_cost = 0.0;
  // curve priority cost
  double curve_priority_cost = 0.0;
  // yield cost
  double yield_cost = 0.0;
  // collision cost
  double collision_cost = 0.0;
  // mode cost
  double mode_cost = 0.0;
  // total cost
  double total_cost = 0.0;
  // mean follow time error
  double mean_less_follow_time_error = 0.0;
  // mean follow time error
  double mean_greater_follow_time_error = 0.0;
  // max_over cruise speed limit ratio
  double max_over_cruise_speed_limit_ratio = 0.0;
  // max over map speed limit ratio
  double max_over_map_speed_limit_ratio = 0.0;
  ObstacleCost front_obstacle_cost;
  // init st obstacle location
  std::vector<STObstacleLocation> st_obstacle_locations_;
  // nudge merge obstacle in continue scenario
  bool nudge_merge_obstacle = false;
};

std::string DebugString(const SpeedCurveCostResult& cost_result);

/**
 * @class SpeedCost
 * @brief This class defines the speed cost calculation method
 */
class alignas(CACHELINE_SIZE) SpeedCost {
 public:
  explicit SpeedCost(const SpeedCostConfig& config);
  virtual ~SpeedCost() = default;

  /**
   * @brief Whether the cost can be update after calculation
   * 
   * @return true 
   * @return false 
   */
  virtual bool Updatable() const { return false; }

  void SetPrintDebug(const bool print_debug) { print_debug_ = print_debug; }

  bool GetPrintDebug() const { return print_debug_; }

  /**
   * @brief Set the Is Merge Stop object
   * 
   * @param is_merge_stop 
   */
  void SetIsMergeStop(const bool is_merge_stop) {
    is_merge_stop_ = is_merge_stop;
  }

  /**
   * @brief 
   * 
   * @return true 
   * @return false 
   */
  bool IsMergeStop() const { return is_merge_stop_; }

  /**
   * @brief 
   * 
   * @return const SpeedCostConfig::SpeedCostType& 
   */
  const SpeedCostConfig::SpeedCostType& CostType() const {
    return speed_cost_type_;
  }

  /**
   * @brief Calculate speed curve cost
   * 
   * @param cache speed cache
   * @param reference_line_info current reference line info
   * @param cost_result speed cost result
   * @return true this curve can be preserved
   * @return false this curve should be deleted
   */
  virtual bool CalculateCost(const SpeedCache& cache,
                             const ReferenceLineInfo& reference_line_info,
                             SpeedCurveCostResult* cost_result) const = 0;

 private:
  bool print_debug_ = false;
  bool is_merge_stop_ = false;
  SpeedCostConfig::SpeedCostType speed_cost_type_;
};

}  // namespace planning
}  // namespace TL
