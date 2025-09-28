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

/**
 * @file
 **/

#pragma once

#include <limits>
#include <string>

#include "planning/common/indexed_list.h"
#include "planning/common/obstacle.h"
#include "proto/planning/decision.pb.h"

namespace TL {
namespace planning {

/**
 * @class PathDecision
 *
 * @brief PathDecision represents all obstacle decisions on one path.
 */
class PathDecision {
 public:
  PathDecision() = default;

  Obstacle* AddObstacle(const Obstacle& obstacle);

  Obstacle* AddEmptyObstacleById(const std::string& obstacle_id);

  const IndexedList<std::string, Obstacle>& obstacles() const;

  bool AddLateralDecision(const std::string& tag, const std::string& object_id,
                          const ObjectDecisionType& decision);
  bool AddLongitudinalDecision(const std::string& tag,
                               const std::string& object_id,
                               const ObjectDecisionType& decision);

  const Obstacle* Find(const std::string& object_id) const;

  const perception::PerceptionObstacle* FindPerceptionObstacle(
      int32_t perception_obstacle_id) const;

  const perception::PerceptionObstacle* FindPerceptionObstacle(
      const std::string& perception_obstacle_id) const;

  IndexedList<std::string, Obstacle>& GetObstacles() {return obstacles_;}

  Obstacle* Find(const std::string& object_id);

  void SetSTBoundary(const std::string& id, const STBoundary& boundary);
  void EraseStBoundaries();

  MainStop main_stop() const { return main_stop_; }

  double stop_reference_line_s() const {
    return is_forward_path_ ? stop_reference_line_s_
                            : reverse_stop_reference_line_s_;
  }

  bool MergeWithMainStop(const ObjectStop& obj_stop, const std::string& obj_id,
                         const ReferenceLine& reference_line,
                         const SLBoundary& adc_sl_boundary,
                         bool is_forward_path);

 private:
  IndexedList<std::string, Obstacle> obstacles_;
  MainStop main_stop_;
  bool is_forward_path_ = true;
  double stop_reference_line_s_ = std::numeric_limits<double>::max();
  double reverse_stop_reference_line_s_ = std::numeric_limits<double>::lowest();
};

}  // namespace planning
}  // namespace TL
