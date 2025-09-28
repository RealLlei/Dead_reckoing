/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/*
 * @file
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "planning/common/path/discretized_path.h"
#include "planning/open_space/coarse_path_generator/node3d.h"
#include "planning/open_space/coarse_path_generator/path_generator.h"
#include "planning/open_space/coarse_path_generator/shape/reeds_shepp_path.h"

#include "planning/common/open_space_info.h"
#include "planning/open_space/coarse_path_generator/shape/geometry_path.h"
#include "planning/proto/planner_open_space_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"

constexpr double kEpsilon = 1.0e-3;

namespace TL {
namespace planning {

class HybridAStar : public PathGenerator {
 public:
  explicit HybridAStar(const WarmStartConfig& warm_start_config);
  ~HybridAStar() override = default;

  /**
 * @brief Use hybird a star algorithm to plan a feasible path with kinemic
 * constrain
 * 
 * @param atomic_early_stop_flag 
 * @param start_point 
 * @param end_point 
 * @param xy_bounds 
 * @param obstacles_segments_vec 
 * @param dest_region_with_angle 
 * @param path_search_strategy 
 * @param result 
 * @return true 
 * @return false 
 */
  bool Plan(const std::atomic<bool>& atomic_early_stop_flag,
            const common::PathPoint& start_point,
            const common::PathPoint& end_point,
            const std::vector<double>& xy_bounds,
            const std::vector<std::pair<common::math::LineSegment2d, double>>&
                obstacles_segments_vec,
            const DestRegionWithAng& dest_region_with_angle,
            const PathSearchStrategy& path_search_strategy,
            PathGeneratorResult* result) override;

  /**
   * @brief 
   * 
   * @param reeds_shepp_path 
   * @return double 
   */
  double CaculateRsPathSteerMargion(
      const std::shared_ptr<ReedSheppPath>& reeds_shepp_path);

  /**
   * @brief 
   * 
   * @param reeds_shepp_path 
   * @param node_ptr 
   * @return double 
   */
  double EvaluateRsPathCost(
      const std::shared_ptr<ReedSheppPath>& reeds_shepp_path,
      const std::shared_ptr<Node3d>& node_ptr);

 private:
  void InitParam();

  /**
   * @brief Analytic expansion can reach target or not
   *
   * @param current_node current expand node
   * @param obstacles_segments_vec obstacles which described as line segment
   * @return true reach the target
   * @return false failed in reaching target
   */
  bool AnalyticExpansion(
      const std::shared_ptr<Node3d>& current_node,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec);
  /**
   * @brief check Reeds Shepp path collision and validity
   *
   * @param current_node
   * @param reeds_shepp_to_end reeds shepp path to end
   * @param obstacles_segments_vec obstacles which described as line segment
   * @return true collision free and valid reeds sheep path
   * @return false
   */
  bool RSPCheck(
      const std::shared_ptr<Node3d>& current_node,
      const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec);

  /**
   * @brief check Reeds Shepp shift counts
   *
   * @param current_node
   * @param reeds_shepp_to_end reeds shepp path to end
   * @param rs_shift_times 
   * @param rs_seg_length 
   */
  static void RSShiftTimes(
      const std::shared_ptr<Node3d>& current_node,
      const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end,
      int* rs_shift_times, std::vector<double>* rs_seg_length = nullptr);

  /**
   * @brief load the whole RSP as nodes and add to the close set
   *
   * @param reeds_shepp_to_end reeds shepp path to end
   * @param current_node current expand node
   * @return std::shared_ptr<Node3d>
   */
  std::shared_ptr<Node3d> LoadRSPinCS(
      const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end,
      const std::shared_ptr<Node3d>& current_node);

  /**
 * @brief 
 * 
 * @param current_node 
 * @param next_node_index 
 * @param is_collison_free_exploration 
 * @return std::shared_ptr<Node3d> 
 */
  std::shared_ptr<Node3d> Next_node_generator(
      const std::shared_ptr<Node3d>& current_node, size_t next_node_index,
      bool is_collison_free_exploration = false);

  /**
 * @brief 
 * 
 * @param current_node 
 * @param next_node 
 * @param distance_to_obstalce 
 * @param is_collison_free_exploration 
 */
  void CalculateNodeCost(const std::shared_ptr<Node3d>& current_node,
                         const std::shared_ptr<Node3d>& next_node,
                         double distance_to_obstalce,
                         bool is_collison_free_exploration = false);

  /**
   * @brief caculate path cost from start to next node
   *
   * @param current_node parrent node of next node
   * @param next_node child node of current node
   * @param double description path cost
   */
  double PathCost(const std::shared_ptr<Node3d>& current_node,
                  const std::shared_ptr<Node3d>& next_node,
                  double distance_to_obstalce) const;

  /**
  * @brief Caculate path length cost
  * 
  * @param path_length 
  * @return double
  */
  double CalculatePathLengthCost(const double path_length) const {
    constexpr double kA = -15;
    constexpr double kB = 20;
    return warm_start_config_.path_length_penalty() /
           (1 + std::exp(kA + kB * path_length));
  }

  /**
   * @brief caculate cost to reference line
   *
   * @param x
   * @param y 
   * @param step_size 
   * @param reference_line current reference line
   * @return current cost to reference line
   */
  double GetReferenceLineCost(double x, double y, double step_size,
                              const ReferenceLine& reference_line) const;

  /**
  * @brief Get the Result object
  * 
  * @param result 
  * @param is_collison_free_exploration_path 
  * @return true 
  * @return false 
  */
  bool GetResult(PathGeneratorResult* result,
                 bool is_collison_free_exploration_path = false);
  bool CalculateParkingPrefinishCondition(
      const std::shared_ptr<Node3d>& current_node);
  /**
 * @brief 
 * 
 * @param start_point 
 * @param end_point 
 */
  void NodeGenerationParameterDecision(const common::PathPoint& start_point,
                                       const common::PathPoint& end_point);

  /**
 * @brief cut off strategy at forward direction
 * 
 * @param node extend node 
 * @param steer steer angle
 * @return true need cut off 
 * @return false 
 */
  inline bool ForwardSteerActionCutoff(const std::shared_ptr<Node3d>& node,
                                       const double steer) const {
    return node->GetY() < max_y_cut_off_ &&
           steer * path_search_strategy_.cut_off_strategy > kEpsilon;
  }

  /**
  * @brief cut off strategy at backward direction
  * 
  * @param node extend node 
  * @param steer steer angle
  * @return true need cut off 
  * @return false 
  */
  inline bool BackwardSteerActionCutoff(const std::shared_ptr<Node3d>& node,
                                        const double steer) const {
    return node->GetY() < max_y_cut_off_ &&
           steer * path_search_strategy_.cut_off_strategy < -1.0 * kEpsilon;
  }

  /**
  * @brief plan direction from start or end decision
  * 
  * @param start_point 
  * @param dest_region_with_angle 
  */
  void PlanDirectionDecision(const common::PathPoint& start_point,
                             const DestRegionWithAng& dest_region_with_angle);

  /**
   * @brief  reverse path generator result
   * 
   * @param result 
   */
  static void ReversePathGeneratorResult(PathGeneratorResult* const result) {
    if (result == nullptr) {
      AERROR << "ReverseHybridAStarResult input check fails";
      return;
    }
    std::reverse(result->x.begin(), result->x.end());
    std::reverse(result->y.begin(), result->y.end());
    std::reverse(result->phi.begin(), result->phi.end());
  }

  /**
 * @brief 
 * 
 * @param start_point 
 * @param obstacles_segments_vec 
 * @param forced_path_direction
 * @param result 
 * @param intermediate_point_ptr 
 * @param has_search_extension_path 
 * @return true 
 * @return false 
 */
  bool GenerateLocalExtensionPath(
      const common::PathPoint& start_point,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec,
      int* forced_path_direction, PathGeneratorResult* result,
      common::PathPoint* intermediate_point_ptr,
      bool* has_search_extension_path);

  size_t next_node_num_ = 0;
  double step_size_ = 0.0;
  double rs_step_size_ = 0.0;
  double max_y_cut_off_ = 0.0;

  std::shared_ptr<Node3d> start_node_;
  std::shared_ptr<Node3d> end_node_;
  std::vector<double> target_region_;
  std::shared_ptr<Node3d> final_node_;
  DestRegionWithAng dest_region_with_angle_;
  std::vector<double> steer_seq_;
  bool is_valid_end_pose_ = true;

  struct cmp {
    bool operator()(const std::pair<std::string, double>& left,
                    const std::pair<std::string, double>& right) const {
      return left.second >= right.second;
    }
  };

  struct node_cmp {
    bool operator()(const std::shared_ptr<Node3d>& left,
                    const std::shared_ptr<Node3d>& right) const {
      return left->GetPathCost() >= right->GetPathCost();
    }
  };

  std::priority_queue<std::shared_ptr<Node3d>,
                      std::vector<std::shared_ptr<Node3d>>, node_cmp>
      end_node_pq_;
  double cur_steer_angle_ = 0.0;

  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> open_set_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> close_set_;
  std::unique_ptr<ReedShepp> reed_shepp_generator_;
  double lateral_rs_steer_angle_margin_ = M_PI_2;
  int explored_failure_times_ = 0;
  int explored_overtime_times_ = 0;
  PathSearchStrategy path_search_strategy_;
  int forced_path_direction_ = 0;
};

}  // namespace planning
}  // namespace TL
