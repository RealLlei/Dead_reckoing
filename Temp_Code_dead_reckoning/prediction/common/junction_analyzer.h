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

#pragma once

#include <deque>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "planning/prediction/common/prediction_map.h"
#include "proto/prediction/feature.pb.h"

namespace TL {
namespace prediction {

class JunctionLaneSequence : public std::deque<std::string> {
 public:
  using BaseJunctionLaneSeq = std::deque<std::string>;
  JunctionExit::JunctionTurnType turn_type_;
};

class JunctionAnalyzer {
 public:
  /**
   * @brief Initialize by junction ID, if junction id differs from prev cycle
   * @param junction ID
   */
  void Init(const std::string& junction_id);

  /**
   * @brief Clear all stored data
   */
  void Clear();

  /**
   * @brief Get junction ID
   * @return Junction ID
   */
  const std::string& GetJunctionId();

  /**
   * @brief Compute junction range
   * @return Junction range
   */
  double ComputeJunctionRange();

  /**
   * @brief Get junction feature starting from start_lane_id
   * @param start lane ID
   * @return junction
   */
  const JunctionFeature& GetJunctionFeature(const std::string& start_lane_id);

  /**
   * @brief Get junction feature starting from start_lane_ids
   * @param start lane IDs
   * @return junction
   */
  JunctionFeature GetJunctionFeature(
      const std::vector<std::string>& start_lane_ids);

  void SetJunctionFeature(Feature* feature_ptr);

  std::list<JunctionLaneSequence> GetEntryLaneSequenceList(
      const std::string& lane_id);

  bool IsEntryLane(const std::string& lane_id);

  static void SetExitFeature(JunctionExit* junction_exit_ptr,
                             const JunctionLaneSequence& lane_seq,
                             JunctionExit::JunctionTurnType turn_type);

  void SetExitFeature(JunctionExit::JunctionTurnType turn_type, int index,
                      const std::string& road_section_id,
                      const std::deque<JunctionExit>& exits,
                      JunctionFeature* latest_junction_feature);

 private:
  /**
   * @brief Set all junction exits in the hashtable junction_exits_
   */
  void SetAllJunctionExits();

  void SetJunctionInfo();

  static void GetJunctionSequence(
      const std::shared_ptr<const hdmap::LaneInfo>& lane_info_ptr,
      const std::unordered_map<std::string, std::deque<JunctionExit>>&
          junction_exit_groups,
      std::list<JunctionLaneSequence>* junc_lane_seq_list);

  static void GetJunctionSequenceBackward(
      const std::shared_ptr<const hdmap::LaneInfo>& lane_info_ptr,
      const std::unordered_map<std::string, std::deque<std::string>>&
          junction_entry_groups,
      std::list<JunctionLaneSequence>* junc_lane_seq_list);

  /**
   * @brief Get all filtered junction exits associated to start lane ID
   * @param start lane ID
   * @return Filtered junction exits
   */
  std::vector<JunctionExit> GetJunctionExits(const std::string& start_lane_id);

  /**
   * @brief Determine if a lane with lane_id is an exit lane of this junction
   * @param lane ID
   * @return If a lane with lane_id is an exit lane of this junction
   */
  bool IsExitLane(const std::string& lane_id);

 private:
  // junction_info pointer associated to the input junction_id
  std::string junction_id_;
  std::shared_ptr<const TL::hdmap::JunctionInfo> junction_info_ptr_;
  // Hashtable: exit_lane_id -> junction_exit
  std::unordered_map<std::string, JunctionExit> junction_exits_;
  // Hashtable: start_lane_id -> junction_feature
  std::unordered_map<std::string, JunctionFeature> junction_features_;

  // Oredered exit road_section and it's heading, anti-clockwise
  std::vector<std::pair<std::string, double>> exit_road_section_heading_;
  // Hashtable: road_section_id -> junction_exit
  std::unordered_map<std::string, std::deque<JunctionExit>> exit_groups_;
  // Hashtable: road_section_id -> junction_entry_id
  std::unordered_map<std::string, std::deque<std::string>> entry_groups_;

  // exit road_section -> exit lane id -> lane seq list
  std::unordered_map<
      std::string,
      std::unordered_map<std::string, std::list<JunctionLaneSequence>>>
      exit_road_section_lane_seq_map_;

  // entry lane id -> lane seq list
  std::unordered_map<std::string, std::list<JunctionLaneSequence>>
      entry_lane_lane_seq_map_;

  // exit lane id
  std::unordered_map<std::string, std::vector<JunctionLaneSequence>>
      noturn_lane_seq_list_map_;
};

}  // namespace prediction
}  // namespace TL
