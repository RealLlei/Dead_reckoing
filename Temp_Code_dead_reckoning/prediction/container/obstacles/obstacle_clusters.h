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

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/util/util.h"
#include "map/hdmap/hdmap_common.h"
#include "proto/prediction/feature.pb.h"

namespace TL {
namespace prediction {

class ObstacleClusters {
 public:
  /**
   * @brief Constructor
   */
  ObstacleClusters() = default;
  /**
   * @brief Remove all lane graphs
   */
  void Init();

  /**
   * @brief Obtain a lane graph given a lane info and s
   * @param lane start s
   * @param lane total length
   * @param if consider lane split ahead
   * @param lane info
   * @return a corresponding lane graph
   */
  static LaneGraph GetLaneGraph(
      double start_s, double lane_l, double length, bool consider_lane_split,
      bool in_forward_direction,
      const std::shared_ptr<const TL::hdmap::LaneInfo>& lane_info_ptr);

  /**
   * @brief Obtain a lane graph given a lane info and s, but don't
   *        memorize it.
   * @param lane start s
   * @param lane total length
   * @param if the obstacle is on lane
   * @param lane info
   * @return a corresponding lane graph
   */
  static LaneGraph GetLaneGraphWithoutMemorizing(
      double start_s, double lane_l, double length, bool is_on_lane,
      const std::shared_ptr<const TL::hdmap::LaneInfo>& lane_info_ptr);

  /**
   * @brief Get the nearest obstacle on lane sequence at s
   * @param Lane sequence
   * @param s offset in the first lane of the lane sequence
   * @param the forward obstacle on lane
   * @return If the forward obstacle is found
   */
  bool ForwardNearbyObstacle(const LaneSequence& lane_sequence, double s,
                             LaneObstacle* lane_obstacle);

  /**
   * @brief Add an obstacle into clusters
   * @param obstacle id
   * @param lane id
   * @param lane s
   * @param lane l
   */
  void AddObstacle(int obstacle_id, const std::string& lane_id, double lane_s,
                   double lane_l, double speed);

  /**
   * @brief Sort lane obstacles by lane s
   */
  void SortObstacles();

  /**
   * @brief Get the forward nearest obstacle on lane sequence at s
   * @param Lane sequence
   * @param s offset in the first lane of the lane sequence
   * @param the forward obstacle on lane
   * @return If the forward obstacle is found
   */
  bool ForwardNearbyObstacle(const LaneSequence& lane_sequence, int obstacle_id,
                             double obstacle_s, double obstacle_l,
                             NearbyObstacle* nearby_obstacle_ptr);

  /**
   * @brief Get the backward nearest obstacle on lane sequence at s
   * @param Lane sequence
   * @param s offset in the first lane of the lane sequence
   * @param the forward obstacle on lane
   * @return If the backward obstacle is found
   */
  bool BackwardNearbyObstacle(const LaneSequence& lane_sequence,
                              int obstacle_id, double obstacle_s,
                              double obstacle_l,
                              NearbyObstacle* nearby_obstacle_ptr);

  /**
   * @brief Query stop sign by lane ID
   * @param lane ID
   * @return the stop sign
   */
  StopSign QueryStopSignByLaneId(const std::string& lane_id);

  std::unordered_map<std::string, std::vector<LaneObstacle>>&
  GetLaneObstacles() {
    return lane_obstacles_;
  }

 private:
  std::unordered_map<std::string, std::vector<LaneObstacle>> lane_obstacles_;
  std::unordered_map<std::string, StopSign> lane_id_stop_sign_map_;
};

}  // namespace prediction
}  // namespace TL
