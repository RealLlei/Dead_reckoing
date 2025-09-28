/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_cost.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace TL {
namespace planning {

SpeedCost::SpeedCost(const SpeedCostConfig& config)
    : speed_cost_type_(config.speed_cost_type()) {}

std::string DebugString(const SpeedCurveCostResult& cost_result) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(3) << "non_follow_safe_distance_cost:"
     << cost_result.non_follow_safe_distance_cost
     << ", obstacle_safe_distance_cost:"
     << cost_result.obstacle_safe_distance_cost
     << ", obstacle_less_expected_distance_cost:"
     << cost_result.obstacle_less_expected_distance_cost
     << ", obstacle_greater_expected_distance_cost:"
     << cost_result.obstacle_greater_expected_distance_cost
     << ", lateral_distance_cost:" << cost_result.lateral_distance_cost
     << ", over_curvature_speed_limit_cost:"
     << cost_result.over_curvature_speed_limit_cost
     << ", over_comfortable_speed_limit_cost:"
     << cost_result.over_comfortable_speed_limit_cost
     << ", over_decision_speed_limit_cost:"
     << cost_result.over_decision_speed_limit_cost
     << ", over_critical_speed_limit_cost:"
     << cost_result.over_critical_speed_limit_cost
     << ", over_nudge_speed_limit_cost:"
     << cost_result.over_nudge_speed_limit_cost
     << ", over_speed_cost:" << cost_result.over_speed_cost
     << ", safe_over_speed_cost:" << cost_result.safe_over_speed_cost
     << ", low_speed_cost:" << cost_result.low_speed_cost
     << ", accel_cost:" << cost_result.accel_cost
     << ", jerk_cost:" << cost_result.jerk_cost
     << ", stop_distance_cost:" << cost_result.stop_distance_cost
     << ", parallel_drive_cost:" << cost_result.parallel_drive_cost
     << ", gap_cost:" << cost_result.gap_cost
     << ", yield_cost:" << cost_result.yield_cost
     << ", collision_cost:" << cost_result.collision_cost
     << ", max_over_cruise_speed_limit_ratio:"
     << cost_result.max_over_cruise_speed_limit_ratio
     << ", max_over_map_speed_limit_ratio:"
     << cost_result.max_over_map_speed_limit_ratio
     << ", curve_priority_cost:" << cost_result.curve_priority_cost
     << ", mean_less_follow_time_error:"
     << cost_result.mean_less_follow_time_error
     << ", mean_greater_follow_time_error:"
     << cost_result.mean_greater_follow_time_error
     << ", nudge_merge_obstacle:" << cost_result.nudge_merge_obstacle
     << ", mode_cost:" << cost_result.mode_cost
     << ", total_cost:" << cost_result.total_cost << std::endl;
  return ss.str();
}

}  // namespace planning
}  // namespace TL
