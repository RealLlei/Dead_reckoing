/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  obstacle_selector.h
 */
#pragma once

#include <cstdint>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "planning/common/reference_line_info.h"
#include "planning/tasks/deciders/obstacles_decider/obstacle_selector/cutin_intention_estimation.h"

namespace TL::planning {
class ObstacleSelector {
 public:
  ObstacleSelector()
      : vehicle_param_(
            common::VehicleConfigHelper::GetConfig().vehicle_param()) {}

  ~ObstacleSelector() = default;

  /**
   * @brief Process.
   * @param alongside_obstacles 
   * @param nearest_cutin_obs_id 
   * @param follow_obs_id 
   * @param reference_line_info 
   * @param decision_obstacles 
   * @param dangerous_obstacles 
   * @return cost值.
   */
  void Process(const std::unordered_set<int32_t>& alongside_obstacles,
               const std::string& nearest_cutin_obs_id,
               const std::string& follow_obs_id,
               const std::unordered_set<int32_t>& cutin_obstacles,
               ReferenceLineInfo* reference_line_info,
               std::unordered_set<std::string>* decision_obstacles,
               std::unordered_set<std::string>* dangerous_obstacles);
  /**
   * @brief GetMapInfo.
   * @param check_s 
   * @param reference_line_info 
   * @param lane_left_width 
   * @param lane_right_width 
   * @param left_neighbor_lane_width 
   * @param right_neighbor_lane_width 
   * @return true: success.
   */
  static bool GetMapInfo(const double& check_s,
                         const ReferenceLineInfo& reference_line_info,
                         double* lane_left_width, double* lane_right_width,
                         double* left_neighbor_lane_width,
                         double* right_neighbor_lane_width);

 private:
  /**
   * @brief GetTTC.
   * @param obstacle 
   * @param reference_line_info 
   * @return ttc.
   */
  static double GetTTC(const Obstacle& obstacle,
                       const ReferenceLineInfo& reference_line_info);

  /**
   * @brief ClearCache.
   * @param reference_line_info 
   */
  void ClearCache(const ReferenceLineInfo& reference_line_info);
  /**
   * @brief Debug.
   * @param reference_line_info 
   */
  void Debug(ReferenceLineInfo* reference_line_info);
  /**
   * @brief IsStableObstacle.
   * @param obstacle
   * @param stable_size 
   * @return true: stable.
   */
  static bool IsStableObstacle(const Obstacle& obstacle, int stable_size);

  /**
   * @brief DoLateralDecision.
   * @param obstacle 
   * @param is_left_obstacle 
   * @param is_right_obstacle 
   * @param reference_line_info 
   */
  static void DoLateralDecision(const Obstacle& obstacle,
                                const bool& is_left_obstacle,
                                const bool& is_right_obstacle,
                                ReferenceLineInfo* reference_line_info);

  /**
   * @brief DoStaticObstacleDecision.
   * @param obstacle 
   * @param is_left_obstacle 
   * @param is_right_obstacle 
   * @param reference_line_info 
   */
  static void DoStaticObstacleDecision(const Obstacle& obstacle,
                                       const bool& is_left_obstacle,
                                       const bool& is_right_obstacle,
                                       ReferenceLineInfo* reference_line_info);
  /**
   * @brief IsRoadRoi.
   * @param obstacle 
   * @param reference_line_info 
   * @param is_overlap_lane 
   * @param is_left_obstacle 
   * @param is_right_obstacle 
   * @param is_away_from_lane
   * @return true: in roi.
   */
  bool IsRoadRoi(const Obstacle& obstacle,
                 const ReferenceLineInfo& reference_line_info,
                 bool* is_overlap_lane_obstacle, bool* is_left_obstacle,
                 bool* is_right_obstacle, bool* is_away_from_lane);
  /**
   * @brief 按照交付要求，路口虚拟车道线自车附近的障碍物caution
   * @param reference_line_info 
   */
  static void VirtualLaneAeraObsCaution(ReferenceLineInfo* reference_line_info);

  /**
   * @brief IsContinuousCone
   * @param unknow_obs 
   * @param reference_line_info 
   */
  bool IsContinuousCone(const Obstacle& unknow_obs,
                        const ReferenceLineInfo& reference_line_info);

  std::unordered_set<std::string> rule_base_decision_obs_ids_;
  std::unordered_set<int32_t> last_frame_overlap_lane_obs_ids_;
  std::unordered_map<std::string, int> overlap_lane_count_;

  TL::common::VehicleParam vehicle_param_;
};
}  // namespace TL::planning
