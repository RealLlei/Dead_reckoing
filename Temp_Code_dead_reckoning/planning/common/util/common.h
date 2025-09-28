#ifndef PLANNING_COMMON_UTIL_COMMON_H
#define PLANNING_COMMON_UTIL_COMMON_H

/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "common/math/double_type.h"
#include "planning/common/frame.h"
#include "planning/common/reference_line_info.h"

namespace TL {
namespace planning {
namespace util {

int BuildStopDecision(const std::string& stop_wall_id, double stop_line_s,
                      double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, Frame* frame,
                      ReferenceLineInfo* reference_line_info);

int BuildStopDecision(const std::string& stop_wall_id,
                      const std::string& lane_id, double lane_s,
                      double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, Frame* frame,
                      ReferenceLineInfo* reference_line_info);

/**
 * @func calculate all state of vehicle with min jerk action.
 * @param init_v
 * @param init_a
 * @param min_v
 * @param max_v
 * @param max_dece
 * @param jerk_min
 * @param vec_vec_state
 * @param max_size
 * @param delta_size
 * @param start_s
 * @return 1: the init state normal; 0: the init state abnormal.
 */
bool GetStateAtMinJerk(double init_v, double init_a, double min_v, double max_v,
                       double max_dece, double jerk_min, double max_size,
                       double delta_size, double start_s,
                       std::vector<std::vector<double>>* vec_vec_state);

/**
 * @func calculate all state of vehicle with max jerk action.
 * @param init_v
 * @param init_a
 * @param min_v
 * @param max_v
 * @param max_acce
 * @param jerk_max
 * @param vec_vec_state
 * @param max_size
 * @param delta_size
 * @param start_s
 * @return 1: the init state normal; 0: the init state abnormal.
 */
bool GetStateAtMaxJerk(double init_v, double init_a, double min_v, double max_v,
                       double max_acce, double jerk_max, double max_size,
                       double delta_size, double start_s,
                       std::vector<std::vector<double>>* vec_vec_state);

inline double GetSpeedLimitWithDeceMaxAtS(
    double temp_s, const std::vector<std::vector<double>>& vec_vec_state) {
  if (temp_s <= vec_vec_state.at(1).back()) {
    size_t num_s = std::lower_bound(vec_vec_state.at(1).begin(),
                                    vec_vec_state.at(1).end(), temp_s) -
                   vec_vec_state.at(1).begin();
    return vec_vec_state.at(2).at(num_s);
  }
  return 0;
}

inline double GetSpeedLimitWithDeceMaxAtT(
    double temp_t, const std::vector<std::vector<double>>& vec_vec_state) {
  if (vec_vec_state.size() == 4 && !vec_vec_state.at(2).empty()) {
    if (temp_t <= vec_vec_state.at(0).back()) {
      size_t num_s = std::lower_bound(vec_vec_state.at(0).begin(),
                                      vec_vec_state.at(0).end(), temp_t) -
                     vec_vec_state.at(0).begin();
      if (num_s < vec_vec_state.at(2).size()) {
        return vec_vec_state.at(2).at(num_s);
      }
      return vec_vec_state.at(2).back();
    }
    return vec_vec_state.at(2).back();
  }
  return 0;
}

inline double GetSminWithDeceMaxAtT(
    double temp_t, const std::vector<std::vector<double>>& vec_vec_state) {
  if (vec_vec_state.size() == 4 && !vec_vec_state.at(1).empty()) {
    if (temp_t <= vec_vec_state.at(0).back()) {
      size_t num_s = std::upper_bound(vec_vec_state.at(0).begin(),
                                      vec_vec_state.at(0).end(), temp_t) -
                     vec_vec_state.at(0).begin();
      if (num_s < vec_vec_state.at(1).size()) {
        return vec_vec_state.at(1).at(num_s);
      }

      return vec_vec_state.at(1).back();
    }
    return vec_vec_state.at(1).back();
  }
  return 0;
}

//  lp: determinate if current reference line is same to previous reference
//  line.
bool IsSameReferenceLine(const Frame& frame_curr,
                         const ReferenceLineInfo& reference_line_info,
                         const ReferenceLine& ref_prev);

/**
 * @brief get lane change backward safe distance 
 * 
 * @param obstacle_speed_coefficient obstacle speed coefficient 
 * @param ego_speed_coefficient ego speed coefficient
 * @param obstacle_speed obstacle speed
 * @param ego_speed ego speed
 * @return double distance
 */
inline double LaneChangeBackWardSafeDistance(
    const double obstacle_speed_coefficient, const double ego_speed_coefficient,
    const double obstacle_speed, const double ego_speed) {
  static constexpr double kPlusDistance = 4.9;
  static constexpr double kHalf = 0.5;
  static constexpr double kDec = 1.0;
  double obs_slowing_down_distance = 0.0;
  if (common::math::double_type::Compare(obstacle_speed, ego_speed) > 0) {
    obs_slowing_down_distance =
        kHalf * std::pow(obstacle_speed - ego_speed, 2) / kDec;
  }
  const double delta_speed = std::fmax(obstacle_speed - ego_speed, 0.0);
  return (obstacle_speed_coefficient - ego_speed_coefficient) * obstacle_speed +
         ego_speed_coefficient * delta_speed + kPlusDistance +
         obs_slowing_down_distance;
}

/**
   * @brief Is road curved section
   * 
   * @param curr_waypoint 
   * @return true 
   * @return false 
   */
bool IsRoadCurvedSection(const hdmap::LaneWaypoint& curr_waypoint);

/**
 * @brief Is normal turn
 * 
 * @param curr_waypoint 
 * @return true 
 * @return false 
 */
bool IsNormalTurn(const hdmap::LaneWaypoint& curr_waypoint);

/**
 * @brief Is normal turn
 * 
 * @param curr_waypoint 
 * @param is_left_normal_turn 
 * @param is_right_normal_turn 
 * @return true 
 * @return false 
 */
bool IsNormalTurn(const hdmap::LaneWaypoint& curr_waypoint,
                  bool* is_left_normal_turn, bool* is_right_normal_turn);
/**
 * @brief 
 * 
 * @param lane 
 * @param pnc_map 
 * @return true 
 * @return false 
 */
bool IsOutermostLane(const TL::hdmap::LaneInfoConstPtr& lane,
                     const std::shared_ptr<hdmap::PncMap>& pnc_map);

/**
 * Calculate the speed limit between two path points.
 *
 * @param path_point_1 the first path point
 * @param path_point_2 the second path point
 *
 * @return the speed limit between the two path points
 *
 * @throws None
 */
double GetDkappaSpeedLimit(const common::PathPoint& pre_point,
                           const common::PathPoint& point);

}  // namespace util
}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_UTIL_COMMON_H
