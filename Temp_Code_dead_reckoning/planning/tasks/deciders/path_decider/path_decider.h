/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>

#include "planning/common/path/path_data.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/task.h"
#include "planning/proto/planning_config.pb.h"

namespace TL {
namespace planning {

class PathDecider : public Task {
 public:
  PathDecider(const TaskConfig& config,
              const std::shared_ptr<DependencyInjector>& injector);

  /**
   * @brief Execute
   * 
   * @param frame 
   * @param reference_line_info 
   * @return TL::common::Status 
   */
  TL::common::Status Execute(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

 private:
  /**
   * @brief Process
   * 
   * @param reference_line_info 
   * @param path_data 
   * @param path_decision 
   * @return TL::common::Status 
   */
  TL::common::Status Process(const ReferenceLineInfo* reference_line_info,
                                const PathData& path_data,
                                PathDecision* path_decision);

  /**
   * @brief Make Object Filter Decision
   * 
   * @param path_decision
   * @param filter_box_id_type  
   * @return true 
   * @return false 
   */
  void MakeObjectFilterDecision(
      const PathDecision& path_decision,
      std::unordered_map<uint32_t, perception::PerceptionObstacle_Type>*
          filter_box_id_type);

  /**
   * @brief Make Object Decision
   * 
   * @param path_data 
   * @param blocking_obstacle_id 
   * @param path_decision 
   * @return true 
   * @return false 
   */
  bool MakeObjectDecision(
      const PathData& path_data, const std::string& blocking_obstacle_id,
      PathDecision* path_decision,
      const std::unordered_map<uint32_t, perception::PerceptionObstacle_Type>&
          filter_box_id_type);

  void PlotCollisionBufferBox();
  /**
   * @brief Make Static Obstacle Decision
   * 
   * @param path_data 
   * @param blocking_obstacle_id 
   * @param path_decision 
   * @return true 
   * @return false 
   */
  bool MakeStaticObstacleDecision(
      const PathData& path_data, const std::string& blocking_obstacle_id,
      PathDecision* path_decision,
      const std::unordered_map<uint32_t, perception::PerceptionObstacle_Type>&
          filter_box_id_type);

  void MakeFreeSpaceSegmentDecision(PathDecision* path_decision);

  /**
   * @brief Generate Object Stop Decision
   * 
   * @param obstacle 
   * @param collision_edge_s if forward, is front edge s else is end edge s
   * @param kappa collision path point kappa abs max
   * @return ObjectStop 
   */
  ObjectStop GenerateObjectStopDecision(const Obstacle& obstacle,
                                        double collision_edge_s,
                                        const double* kappa = nullptr) const;

  /**
   * @brief Is adc overlapping with obstacle?
   *
   * @param obs obstacle
   * @param collision path point index if adc overlaps with obstacle; otherwise -1.0
   * @return true adc overlaps with obstacle
   * @return false 
   */
  bool IsADCOverlappingWithObstacle(
      const Obstacle& obs, size_t* collision_path_point_index = nullptr) const;
  /**
   * @brief Is adc Precise bound overlapping with obstacle?
   *
   * @param obs obstacle
   * @param collision path point index if adc Precise overlaps with obstacle; otherwise -1.0
   * @return true adc overlaps with obstacle
   * @return false 
   */
  bool IsADCOverlappingWithObstaclePrecise(
      const Obstacle& obs, size_t* collision_path_point_index = nullptr) const;

  /**
   * @brief Get adc l buffer for static obstacle collision detection
   * 
   * @param obs 
   * @param is_avp_mode 
   * @return double 
   */
  double GetADCLBuffer(const Obstacle& obs, bool is_avp_mode) const;

  double GetStopDistance(const Obstacle& obs,
                         const double* kappa = nullptr) const;

 private:
  PathDeciderConfig path_decider_config_;
};

}  // namespace planning
}  // namespace TL
