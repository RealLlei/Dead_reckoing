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

#include "planning/prediction/common/junction_analyzer.h"
#include <algorithm>
#include <limits>
#include <queue>
#include <stack>
#include <unordered_set>
#include <utility>
#include "planning/prediction/common/prediction_gflags.h"
#include "planning/prediction/common/prediction_util.h"

namespace TL {
namespace prediction {

using TL::common::PointENU;
using TL::hdmap::JunctionInfo;
using TL::hdmap::LaneInfo;
using ConstLaneInfoPtr = std::shared_ptr<const LaneInfo>;

void JunctionAnalyzer::Init(const std::string& junction_id) {
  if (junction_id_ == junction_id) {
    return;
  }
  Clear();

  junction_id_ = junction_id;
  SetJunctionInfo();
  // SetAllJunctionExits();
}

void JunctionAnalyzer::Clear() {
  // Clear all data
  junction_id_.clear();
  junction_exits_.clear();
  junction_features_.clear();

  exit_groups_.clear();
  entry_groups_.clear();
  exit_road_section_heading_.clear();
  exit_road_section_lane_seq_map_.clear();
  entry_lane_lane_seq_map_.clear();
  noturn_lane_seq_list_map_.clear();
}

void JunctionAnalyzer::SetJunctionInfo() {
  const auto& junction_info_ptr = PredictionMap::JunctionById(junction_id_);
  if (nullptr == junction_info_ptr) {
    return;
  }

  ADEBUG << "Build junction " << junction_id_ << " info...";

  std::unordered_map<std::string, std::unordered_set<std::string>>
      junction_exit_groups;
  std::unordered_map<std::string, std::unordered_set<std::string>>
      junction_entry_groups;

  for (const auto& overlap_id : junction_info_ptr->junction().overlap_id()) {
    auto overlap_info_ptr = PredictionMap::OverlapById(overlap_id.id());
    if (overlap_info_ptr == nullptr) {
      continue;
    }

    for (const auto& object : overlap_info_ptr->overlap().object()) {
      if (object.has_lane_overlap_info()) {
        const std::string& lane_id = object.id().id();
        auto lane_info_ptr = PredictionMap::LaneById(lane_id);
        if (lane_info_ptr == nullptr) {
          continue;
        }

        if (std::isgreater(object.lane_overlap_info().start_s(), 0.0)) {
          std::string road_section_id =
              PredictionMap::GetRoadSectionCombinedId(lane_info_ptr);
          if (!road_section_id.empty()) {
            auto entry_group = junction_entry_groups.find(road_section_id);
            if (entry_group != junction_entry_groups.end()) {
              entry_group->second.insert(lane_id);
            } else {
              std::unordered_set<std::string> new_entry_group;
              new_entry_group.insert(lane_id);
              junction_entry_groups.emplace(road_section_id, new_entry_group);
            }
          }
        } else {
          // 如果junction lane的 predecessor lane不在junction，
          // 那么predecessor lane可能是junction的一个入口
          for (const auto& predecessor_id :
               lane_info_ptr->lane().predecessor_id()) {
            const auto& predecessor_lane_id = predecessor_id.id();
            const auto predecessor_lane_ptr =
                PredictionMap::LaneById(predecessor_lane_id);
            if (nullptr == predecessor_lane_ptr ||
                PredictionMap::IsLaneInJunction(predecessor_lane_ptr,
                                                junction_id_)) {
              continue;
            }
            std::string road_section_id =
                PredictionMap::GetRoadSectionCombinedId(predecessor_lane_ptr);
            if (road_section_id.empty()) {
              continue;
            }
            auto entry_group = junction_entry_groups.find(road_section_id);
            if (entry_group != junction_entry_groups.end()) {
              entry_group->second.insert(predecessor_lane_id);
            } else {
              std::unordered_set<std::string> new_entry_group;
              new_entry_group.insert(predecessor_lane_id);
              junction_entry_groups.emplace(road_section_id, new_entry_group);
            }
          }
        }

        if (std::isless(object.lane_overlap_info().end_s(),
                        lane_info_ptr->overall_length() - 0.1)) {
          std::string road_section_id =
              PredictionMap::GetRoadSectionCombinedId(lane_info_ptr);
          if (!road_section_id.empty()) {
            auto exit_group = junction_exit_groups.find(road_section_id);
            if (exit_group != junction_exit_groups.end()) {
              exit_group->second.insert(lane_id);
            } else {
              std::unordered_set<std::string> new_exit_group;
              new_exit_group.insert(lane_id);
              junction_exit_groups.emplace(road_section_id, new_exit_group);
            }
          }
        } else {
          // 如果junction lane的 successor lane不在junction,
          // 那么successor lane可能是junction的一个出口
          for (const auto& successor_id :
               lane_info_ptr->lane().successor_id()) {
            const auto& successor_lane_id = successor_id.id();
            auto successor_lane_ptr =
                PredictionMap::LaneById(successor_lane_id);
            if (nullptr == successor_lane_ptr ||
                PredictionMap::IsLaneInJunction(successor_lane_ptr,
                                                junction_id_)) {
              continue;
            }
            std::string road_section_id =
                PredictionMap::GetRoadSectionCombinedId(successor_lane_ptr);
            if (road_section_id.empty()) {
              continue;
            }
            auto exit_group = junction_exit_groups.find(road_section_id);
            if (exit_group != junction_exit_groups.end()) {
              exit_group->second.insert(successor_lane_id);
            } else {
              std::unordered_set<std::string> new_exit_group;
              new_exit_group.insert(successor_lane_id);
              junction_exit_groups.emplace(road_section_id, new_exit_group);
            }
          }
        }
      }
    }
  }
  std::unordered_set<std::string> error_group_set;
  // 如果entry lane的 successor lane所在的road section和其他entry lane的scetion是同一个，
  // 将这个entry group去掉，并且将这个successor lane放到对应的entry group；
  for (const auto& entry_group : junction_entry_groups) {
    for (const auto& entry_lane_id : entry_group.second) {
      auto entry_lane_ptr = PredictionMap::LaneById(entry_lane_id);
      if (nullptr == entry_lane_ptr) {
        continue;
      }
      for (const auto& successor_id : entry_lane_ptr->lane().successor_id()) {
        const auto& successor_lane_id = successor_id.id();
        auto successor_lane_ptr = PredictionMap::LaneById(successor_lane_id);
        if (nullptr == successor_lane_ptr) {
          continue;
        }
        std::string road_section_id =
            PredictionMap::GetRoadSectionCombinedId(successor_lane_ptr);
        if (road_section_id.empty()) {
          continue;
        }
        auto x = junction_entry_groups.find(road_section_id);
        if (x == junction_entry_groups.end()) {
          continue;
        }
        x->second.insert(successor_lane_id);
        error_group_set.insert(entry_group.first);
        break;
      }
    }
  }
  for (const auto& entry_group : junction_entry_groups) {
    if (error_group_set.find(entry_group.first) != error_group_set.end()) {
      continue;
    }
    for (const auto& entry_lane_id : entry_group.second) {
      auto entry_lane_ptr = PredictionMap::LaneById(entry_lane_id);
      if (nullptr == entry_lane_ptr) {
        continue;
      }
      for (const auto& predecessor_id :
           entry_lane_ptr->lane().predecessor_id()) {
        const auto& predecessor_lane_id = predecessor_id.id();
        auto predecessor_lane_ptr =
            PredictionMap::LaneById(predecessor_lane_id);
        if (nullptr == predecessor_lane_ptr) {
          continue;
        }
        std::string road_section_id =
            PredictionMap::GetRoadSectionCombinedId(predecessor_lane_ptr);
        if (road_section_id.empty()) {
          continue;
        }
        if (error_group_set.find(road_section_id) != error_group_set.end()) {
          break;
        }
        auto x = junction_entry_groups.find(road_section_id);
        if (x == junction_entry_groups.end()) {
          continue;
        }
        x->second.insert(predecessor_lane_id);
        error_group_set.insert(entry_group.first);
        break;
      }
    }
  }
  // 把处理好的入口，从左到右放入junction_entry_groups_
  for (const auto& entry_group : junction_entry_groups) {
    if (error_group_set.find(entry_group.first) != error_group_set.end()) {
      continue;
    }
    if (entry_group.second.empty()) {
      continue;
    }
    auto section_ptr =
        PredictionMap::GetLaneSection(*entry_group.second.begin());
    if (nullptr == section_ptr || section_ptr->lane_id().empty()) {
      continue;
    }
    bool is_from_left2right =
        PredictionMap::IsSectionLaneFromLeft2Right(*section_ptr);
    std::deque<std::string> new_entry_group;

    for (const auto& section_lane_id : section_ptr->lane_id()) {
      if (entry_group.second.find(section_lane_id.id()) !=
          entry_group.second.end()) {
        if (is_from_left2right) {
          new_entry_group.push_back(section_lane_id.id());
        } else {
          new_entry_group.push_front(section_lane_id.id());
        }
      }
    }
    entry_groups_.emplace(entry_group.first, new_entry_group);
  }

  error_group_set.clear();
  // 如果exit lane的 predecessor lane所在的road section和其他exit lane的scetion是同一个，
  // 将这个exit group去掉，并且将这个predecessor lane放到对应的exit group；
  for (const auto& exit_group : junction_exit_groups) {
    for (const auto& exit_lane_id : exit_group.second) {
      auto exit_lane_ptr = PredictionMap::LaneById(exit_lane_id);
      if (nullptr == exit_lane_ptr) {
        continue;
      }
      for (const auto& predecessor_id :
           exit_lane_ptr->lane().predecessor_id()) {
        const auto& predecessor_lane_id = predecessor_id.id();
        auto predecessor_lane_ptr =
            PredictionMap::LaneById(predecessor_lane_id);
        if (nullptr == predecessor_lane_ptr) {
          continue;
        }
        const auto& road_section_id =
            PredictionMap::GetRoadSectionCombinedId(predecessor_lane_ptr);
        auto x = junction_exit_groups.find(road_section_id);
        if (x == junction_exit_groups.end()) {
          continue;
        }
        x->second.insert(predecessor_lane_id);
        error_group_set.insert(exit_group.first);
        break;
      }
    }
  }
  for (const auto& exit_group : junction_exit_groups) {
    if (error_group_set.find(exit_group.first) != error_group_set.end()) {
      continue;
    }
    for (const auto& exit_lane_id : exit_group.second) {
      auto exit_lane_ptr = PredictionMap::LaneById(exit_lane_id);
      if (nullptr == exit_lane_ptr) {
        continue;
      }
      for (const auto& successor_id : exit_lane_ptr->lane().successor_id()) {
        const auto& successor_lane_id = successor_id.id();
        auto successor_lane_ptr = PredictionMap::LaneById(successor_lane_id);
        if (nullptr == successor_lane_ptr) {
          continue;
        }
        const auto& road_section_id =
            PredictionMap::GetRoadSectionCombinedId(successor_lane_ptr);
        if (error_group_set.find(road_section_id) != error_group_set.end()) {
          break;
        }
        auto x = junction_entry_groups.find(road_section_id);
        if (x == junction_entry_groups.end()) {
          continue;
        }
        x->second.insert(successor_lane_id);
        error_group_set.insert(exit_group.first);
        break;
      }
    }
  }
  // 把处理好的出口，从左到右放入junction_exit_groups_
  for (const auto& exit_group : junction_exit_groups) {
    if (error_group_set.find(exit_group.first) != error_group_set.end()) {
      continue;
    }
    if (exit_group.second.empty()) {
      continue;
    }
    auto section_ptr =
        PredictionMap::GetLaneSection(*exit_group.second.begin());
    if (nullptr == section_ptr || section_ptr->lane_id().empty()) {
      continue;
    }

    bool is_from_left2right =
        PredictionMap::IsSectionLaneFromLeft2Right(*section_ptr);

    int first_exit_lane_idx = 0;
    int last_exit_lane_idx = section_ptr->lane_id_size() - 1;
    for (int i = 0; i < section_ptr->lane_id_size(); ++i) {
      if (exit_group.second.find(section_ptr->lane_id(i).id()) ==
          exit_group.second.end()) {
        continue;
      }
      first_exit_lane_idx = i;
      break;
    }
    for (int i = section_ptr->lane_id_size() - 1; i >= 0; --i) {
      if (exit_group.second.find(section_ptr->lane_id(i).id()) ==
          exit_group.second.end()) {
        continue;
      }
      last_exit_lane_idx = i;
      break;
    }

    std::deque<JunctionExit> new_exit_group;
    for (int i = first_exit_lane_idx; i <= last_exit_lane_idx; ++i) {
      const auto& section_lane_id = section_ptr->lane_id(i);
      const auto section_lane_ptr =
          PredictionMap::LaneById(section_lane_id.id());
      if (nullptr == section_lane_ptr) {
        continue;
      }

      double s = 0.0;
      for (const auto& overlap : section_lane_ptr->overlaps()) {
        bool valid_overlap = false;
        for (const auto& object : overlap->overlap().object()) {
          if (object.id().id() == junction_id_) {
            valid_overlap = true;
            break;
          }
        }
        for (const auto& object : overlap->overlap().object()) {
          if (valid_overlap && object.has_lane_overlap_info()) {
            s = object.lane_overlap_info().end_s();
            break;
          }
        }
      }

      PointENU position = section_lane_ptr->GetSmoothPoint(s);
      JunctionExit junction_exit;
      junction_exit.mutable_exit_position()->set_x(position.x());
      junction_exit.mutable_exit_position()->set_y(position.y());
      junction_exit.set_exit_heading(section_lane_ptr->Heading(s));
      junction_exit.set_exit_width(section_lane_ptr->GetWidth(s));
      junction_exit.set_exit_lane_id(section_lane_id.id());
      const auto& road_section_id =
          PredictionMap::GetRoadSectionCombinedId(section_lane_ptr);
      junction_exit.set_road_section_id(road_section_id);
      if (is_from_left2right) {
        new_exit_group.push_back(junction_exit);
      } else {
        new_exit_group.push_front(junction_exit);
      }
    }
    exit_groups_.emplace(exit_group.first, new_exit_group);
  }

  exit_road_section_heading_.reserve(exit_groups_.size());
  for (const auto& exit_group : exit_groups_) {
    exit_road_section_heading_.emplace_back(
        exit_group.first,
        common::math::NormalizeAngle(exit_group.second.at(0).exit_heading()));
  }
  std::sort(
      exit_road_section_heading_.begin(), exit_road_section_heading_.end(),
      [&](std::pair<std::string, double>& a,
          std::pair<std::string, double>& b) { return a.second < b.second; });

  ADEBUG << "Exit road section -- Heading ";
  for (const auto& x : exit_road_section_heading_) {
    ADEBUG << "  " << x.first << " -- " << x.second * 180.0 / M_PI;
  }

  std::unordered_map<std::string, std::list<JunctionLaneSequence>>
      tmp_section_lane_seq_map;

  JunctionLaneSequence tmp_junction_lane_seq;
  std::list<JunctionLaneSequence> tmp_junc_lane_seq_list;
  for (const auto& exit_group : exit_groups_) {
    std::unordered_map<std::string, std::list<JunctionLaneSequence>>
        tmp_exit_lane_lane_seq_map;
    std::list<JunctionLaneSequence> section_junc_lane_seq_lists;
    for (const auto& exit : exit_group.second) {
      const auto exit_lane_ptr = PredictionMap::LaneById(exit.exit_lane_id());
      if (nullptr == exit_lane_ptr) {
        continue;
      }
      tmp_junction_lane_seq.push_back(exit_lane_ptr->lane().id().id());
      GetJunctionSequenceBackward(exit_lane_ptr, entry_groups_,
                                  &tmp_junc_lane_seq_list);
      for (auto& tmp_junc_lane_seq : tmp_junc_lane_seq_list) {
        auto turn_type = JunctionExit::NO_TURN;
        for (const auto& lane_id : tmp_junc_lane_seq) {
          const auto lane_info = PredictionMap::LaneById(lane_id);
          if (nullptr == lane_info) {
            continue;
          }
          if (lane_info->lane().turn() == hdmap::Lane_LaneTurn_LEFT_TURN) {
            turn_type = JunctionExit::LEFT_TURN;
          } else if (lane_info->lane().turn() ==
                     hdmap::Lane_LaneTurn_RIGHT_TURN) {
            turn_type = JunctionExit::RIGHT_TURN;
          } else if (lane_info->lane().turn() == hdmap::Lane_LaneTurn_U_TURN) {
            turn_type = JunctionExit::U_TURN;
          }
        }
        tmp_junc_lane_seq.turn_type_ = turn_type;
        section_junc_lane_seq_lists.push_back(tmp_junc_lane_seq);
      }

      tmp_exit_lane_lane_seq_map.emplace(exit.exit_lane_id(),
                                         tmp_junc_lane_seq_list);
      tmp_junction_lane_seq.clear();
      tmp_junc_lane_seq_list.clear();
    }
    exit_road_section_lane_seq_map_.emplace(exit_group.first,
                                            tmp_exit_lane_lane_seq_map);
    tmp_section_lane_seq_map.emplace(exit_group.first,
                                     section_junc_lane_seq_lists);
  }

  ADEBUG << "\n";
  ADEBUG << "Exit_section_lane_seq_map";
  for (const auto& tmp_section_lane_seq : exit_road_section_lane_seq_map_) {
    ADEBUG << "  section id " << tmp_section_lane_seq.first;
    for (const auto& tmp_lane_lane_seq : tmp_section_lane_seq.second) {
      ADEBUG << "    lane id " << tmp_lane_lane_seq.first;
      for (const auto& tmp_lane_seq : tmp_lane_lane_seq.second) {
        std::string seq_str;
        for (const auto& tmp_lane_id : tmp_lane_seq) {
          seq_str += tmp_lane_id + "-";
        }
        ADEBUG << "      lane seq " << seq_str
               << " turn type: " << tmp_lane_seq.turn_type_;
      }
    }
  }

  tmp_junction_lane_seq.clear();
  tmp_junc_lane_seq_list.clear();

  for (const auto& entry_group : entry_groups_) {
    std::list<JunctionLaneSequence> section_junc_lane_seq_lists;

    for (const auto& entry_lane_id : entry_group.second) {
      const auto entry_lane_ptr = PredictionMap::LaneById(entry_lane_id);
      if (nullptr == entry_lane_ptr) {
        continue;
      }

      tmp_junction_lane_seq.push_back(entry_lane_ptr->lane().id().id());
      GetJunctionSequence(entry_lane_ptr, exit_groups_,
                          &tmp_junc_lane_seq_list);

      for (auto& tmp_junc_lane_seq : tmp_junc_lane_seq_list) {
        auto turn_type = JunctionExit::NO_TURN;
        for (const auto& lane_id : tmp_junc_lane_seq) {
          const auto lane_info = PredictionMap::LaneById(lane_id);
          if (nullptr == lane_info) {
            continue;
          }
          if (lane_info->lane().turn() == hdmap::Lane_LaneTurn_LEFT_TURN) {
            turn_type = JunctionExit::LEFT_TURN;
          } else if (lane_info->lane().turn() ==
                     hdmap::Lane_LaneTurn_RIGHT_TURN) {
            turn_type = JunctionExit::RIGHT_TURN;
          } else if (lane_info->lane().turn() == hdmap::Lane_LaneTurn_U_TURN) {
            turn_type = JunctionExit::U_TURN;
          }
        }
        tmp_junc_lane_seq.turn_type_ = turn_type;
        section_junc_lane_seq_lists.push_back(tmp_junc_lane_seq);
      }

      entry_lane_lane_seq_map_.emplace(entry_lane_id, tmp_junc_lane_seq_list);
      tmp_junction_lane_seq.clear();
      tmp_junc_lane_seq_list.clear();
    }
  }

  ADEBUG << "\n";
  ADEBUG << "Entry_lane_lane_seq_map";
  for (const auto& entry_lane_lane_seq : entry_lane_lane_seq_map_) {
    ADEBUG << "  lane id: " << entry_lane_lane_seq.first;
    for (const auto& tmp_lane_seq : entry_lane_lane_seq.second) {
      std::string seq_str;
      for (const auto& tmp_lane_id : tmp_lane_seq) {
        seq_str += tmp_lane_id + "-";
      }
      ADEBUG << "    lane seq " << seq_str
             << " turn type: " << tmp_lane_seq.turn_type_;
    }
  }

  for (const auto& tmp : tmp_section_lane_seq_map) {
    std::vector<JunctionLaneSequence> tmp_junction_lane_seqs;
    bool has_no_turn_seq = false;
    const auto& exit_section_lane_seq = tmp.second;
    for (const auto& lane_seq : exit_section_lane_seq) {
      if (JunctionExit::NO_TURN == lane_seq.turn_type_) {
        tmp_junction_lane_seqs.emplace_back(lane_seq);
        has_no_turn_seq = true;
      }
    }
    if (!has_no_turn_seq) {
      continue;
    }
    noturn_lane_seq_list_map_.emplace(tmp.first, tmp_junction_lane_seqs);
  }
  ADEBUG << "\n";
  ADEBUG << "Junction Entry Info:";
  for (const auto& entry_group : entry_groups_) {
    ADEBUG << "  road section id: " << entry_group.first;
    for (const auto& entry_lane_id : entry_group.second) {
      ADEBUG << "    entry lane id: " << entry_lane_id;
    }
  }

  ADEBUG << "\n";
  ADEBUG << "Junction Exit Info:";
  for (const auto& exit_group : exit_groups_) {
    ADEBUG << "  road section id: " << exit_group.first;
    for (const auto& exit_lane_id : exit_group.second) {
      ADEBUG << "    exit lane id: " << exit_lane_id.exit_lane_id();
    }
  }
}

void JunctionAnalyzer::GetJunctionSequence(
    const std::shared_ptr<const LaneInfo>& lane_info_ptr,
    const std::unordered_map<std::string, std::deque<JunctionExit>>&
        junction_exit_groups,
    std::list<JunctionLaneSequence>* junc_lane_seq_list) {
  if (nullptr == lane_info_ptr) {
    return;
  }

  std::stack<std::pair<std::shared_ptr<const LaneInfo>, int>> lane_info_st;
  std::stack<JunctionLaneSequence> lane_seq_st;

  lane_info_st.emplace(lane_info_ptr, 1);
  JunctionLaneSequence tmp_seq;
  tmp_seq.emplace_back(lane_info_ptr->id().id());
  lane_seq_st.emplace(std::move(tmp_seq));

  int h = 0;
  while (!lane_info_st.empty()) {
    auto tmp_lane_info = lane_info_st.top();
    lane_info_st.pop();
    auto lane_seq = lane_seq_st.top();
    lane_seq_st.pop();

    if (nullptr == tmp_lane_info.first) {
      break;
    }
    const auto& road_section_id =
        PredictionMap::GetRoadSectionCombinedId(tmp_lane_info.first);
    if (junction_exit_groups.find(road_section_id) !=
        junction_exit_groups.end()) {
      junc_lane_seq_list->emplace_back(lane_seq);
      continue;
    }
    h = std::max(h, tmp_lane_info.second);
    if (h >= 20) {
      return;
    }
    for (int i = tmp_lane_info.first->lane().successor_id().size() - 1; i >= 0;
         --i) {
      auto successor_lane_ptr = PredictionMap::LaneById(
          tmp_lane_info.first->lane().successor_id().at(i).id());

      if (nullptr == successor_lane_ptr) {
        continue;
      }
      lane_info_st.emplace(successor_lane_ptr, tmp_lane_info.second + 1);
      auto tmp_lane_seq = lane_seq;
      tmp_lane_seq.emplace_back(successor_lane_ptr->id().id());
      lane_seq_st.emplace(std::move(tmp_lane_seq));
    }
  }
}

void JunctionAnalyzer::GetJunctionSequenceBackward(
    const std::shared_ptr<const LaneInfo>& lane_info_ptr,
    const std::unordered_map<std::string, std::deque<std::string>>&
        junction_entry_groups,
    std::list<JunctionLaneSequence>* junc_lane_seq_list) {
  if (nullptr == lane_info_ptr) {
    return;
  }

  std::stack<std::pair<std::shared_ptr<const LaneInfo>, int>> lane_info_st;
  std::stack<JunctionLaneSequence> lane_seq_st;

  lane_info_st.emplace(lane_info_ptr, 1);
  JunctionLaneSequence tmp_seq;
  tmp_seq.emplace_back(lane_info_ptr->id().id());
  lane_seq_st.emplace(tmp_seq);

  int h = 0;
  while (!lane_info_st.empty()) {
    auto tmp_lane_info = lane_info_st.top();
    lane_info_st.pop();
    auto lane_seq = lane_seq_st.top();
    lane_seq_st.pop();

    if (nullptr == tmp_lane_info.first) {
      break;
    }
    const auto& road_section_id =
        PredictionMap::GetRoadSectionCombinedId(tmp_lane_info.first);
    if (junction_entry_groups.find(road_section_id) !=
        junction_entry_groups.end()) {
      junc_lane_seq_list->emplace_back(lane_seq);
      continue;
    }
    h = std::max(h, tmp_lane_info.second);
    if (h >= 20) {
      return;
    }
    for (auto iter = tmp_lane_info.first->lane().predecessor_id().rbegin();
         iter != tmp_lane_info.first->lane().predecessor_id().rend(); ++iter) {
      auto predecessor_lane_ptr = PredictionMap::LaneById(iter->id());
      if (nullptr == predecessor_lane_ptr) {
        continue;
      }
      lane_info_st.push({predecessor_lane_ptr, tmp_lane_info.second + 1});
      auto tmp_lane_seq = lane_seq;
      tmp_lane_seq.emplace_front(predecessor_lane_ptr->id().id());
      lane_seq_st.push(tmp_lane_seq);
    }
  }
}

void JunctionAnalyzer::SetAllJunctionExits() {
  const auto& junction_info_ptr = PredictionMap::JunctionById(junction_id_);
  if (nullptr == junction_info_ptr) {
    return;
  }
  // Go through everything that the junction overlaps with.
  for (const auto& overlap_id : junction_info_ptr->junction().overlap_id()) {
    auto overlap_info_ptr = PredictionMap::OverlapById(overlap_id.id());
    if (overlap_info_ptr == nullptr) {
      continue;
    }
    // Find the lane-segments that are overlapping, yet also extends out of
    // the junction area. Those are the junction-exit-lanes.
    for (const auto& object : overlap_info_ptr->overlap().object()) {
      if (object.has_lane_overlap_info()) {
        const std::string& lane_id = object.id().id();
        auto lane_info_ptr = PredictionMap::LaneById(lane_id);
        if (lane_info_ptr == nullptr) {
          continue;
        }
        double s = object.lane_overlap_info().end_s();
        if (s + FLAGS_junction_exit_lane_threshold <
            lane_info_ptr->total_length()) {
          JunctionExit junction_exit;
          PointENU position = lane_info_ptr->GetSmoothPoint(s);
          junction_exit.set_exit_lane_id(lane_id);
          junction_exit.mutable_exit_position()->set_x(position.x());
          junction_exit.mutable_exit_position()->set_y(position.y());
          junction_exit.set_exit_heading(lane_info_ptr->Heading(s));
          junction_exit.set_exit_width(lane_info_ptr->GetWidth(s));
          // add junction_exit to hashtable
          junction_exits_[lane_id] = junction_exit;
        }
      }
    }
  }
}

std::vector<JunctionExit> JunctionAnalyzer::GetJunctionExits(
    const std::string& start_lane_id) {
  // TODO(hongyi) make this a gflag
  int max_search_level = 6;

  std::vector<JunctionExit> junction_exits;
  std::queue<std::pair<ConstLaneInfoPtr, int>> lane_info_queue;
  auto lane_info_ptr = PredictionMap::LaneById(start_lane_id);
  if (lane_info_ptr != nullptr) {
    lane_info_queue.emplace(lane_info_ptr, 0);
  }

  std::unordered_set<std::string> visited_exit_lanes;
  // Perform a BFS to find all exit lanes that can be connected through
  // this start_lane_id.
  while (!lane_info_queue.empty()) {
    ConstLaneInfoPtr curr_lane = lane_info_queue.front().first;
    int level = lane_info_queue.front().second;
    lane_info_queue.pop();
    const std::string& curr_lane_id = curr_lane->id().id();
    // Stop if this is already an exit lane.
    if (IsExitLane(curr_lane_id) &&
        visited_exit_lanes.find(curr_lane_id) == visited_exit_lanes.end()) {
      junction_exits.push_back(junction_exits_[curr_lane_id]);
      visited_exit_lanes.insert(curr_lane_id);
      continue;
    }
    // Stop if reached max-search-level.
    if (level >= max_search_level) {
      continue;
    }
    for (const auto& succ_lane_id : curr_lane->lane().successor_id()) {
      ConstLaneInfoPtr succ_lane_ptr =
          PredictionMap::LaneById(succ_lane_id.id());
      lane_info_queue.emplace(succ_lane_ptr, level + 1);
    }
  }
  return junction_exits;
}

const JunctionFeature& JunctionAnalyzer::GetJunctionFeature(
    const std::string& start_lane_id) {
  if (junction_features_.find(start_lane_id) != junction_features_.end()) {
    return junction_features_[start_lane_id];
  }
  JunctionFeature junction_feature;
  junction_feature.set_junction_id(GetJunctionId());
  junction_feature.set_junction_range(ComputeJunctionRange());
  // Find all junction-exit-lanes that are successors of the start_lane_id.
  std::vector<JunctionExit> junction_exits = GetJunctionExits(start_lane_id);

  for (const auto& junction_exit : junction_exits) {
    junction_feature.add_junction_exit()->CopyFrom(junction_exit);
  }
  junction_feature.mutable_enter_lane()->set_lane_id(start_lane_id);
  junction_feature.add_start_lane_id(start_lane_id);
  junction_features_[start_lane_id] = junction_feature;
  return junction_features_[start_lane_id];
}

JunctionFeature JunctionAnalyzer::GetJunctionFeature(
    const std::vector<std::string>& start_lane_ids) {
  JunctionFeature merged_junction_feature;
  bool initialized = false;
  std::unordered_map<std::string, JunctionExit> junction_exits_map;
  for (const std::string& start_lane_id : start_lane_ids) {
    JunctionFeature junction_feature = GetJunctionFeature(start_lane_id);
    if (!initialized) {
      merged_junction_feature.set_junction_id(junction_feature.junction_id());
      merged_junction_feature.set_junction_range(
          junction_feature.junction_range());
      initialized = true;
    }
    for (const JunctionExit& junction_exit : junction_feature.junction_exit()) {
      if (junction_exits_map.find(junction_exit.exit_lane_id()) ==
          junction_exits_map.end()) {
        junction_exits_map[junction_exit.exit_lane_id()] = junction_exit;
      }
    }
  }
  for (const auto& exit : junction_exits_map) {
    merged_junction_feature.add_start_lane_id(exit.first);
    merged_junction_feature.add_junction_exit()->CopyFrom(exit.second);
  }
  return merged_junction_feature;
}

void JunctionAnalyzer::SetExitFeature(
    JunctionExit::JunctionTurnType turn_type, int index,
    const std::string& road_section_id, const std::deque<JunctionExit>& exits,
    JunctionFeature* latest_junction_feature) {
  if (nullptr == latest_junction_feature || index < 0 ||
      index >= exits.size()) {
    return;
  }

  const auto& exit_id = exits.at(index).exit_lane_id();
  // 找到出口road section lane seq
  const auto& section_lane_seq_map =
      exit_road_section_lane_seq_map_.find(road_section_id);
  bool is_find_seq = false;
  if (section_lane_seq_map != exit_road_section_lane_seq_map_.end()) {
    const auto& lane_lane_seq_list = section_lane_seq_map->second.find(exit_id);
    if (lane_lane_seq_list != section_lane_seq_map->second.end()) {
      for (const auto& lane_lane_seq : lane_lane_seq_list->second) {
        if (turn_type != lane_lane_seq.turn_type_) {
          continue;
        }
        JunctionExit* junction_exit_ptr =
            latest_junction_feature->add_junction_exit();
        SetExitFeature(junction_exit_ptr, lane_lane_seq, turn_type);
        is_find_seq = true;
        break;
      }
      // 没找到塞no_turn seq
      if (!is_find_seq) {
        for (const auto& lane_lane_seq : lane_lane_seq_list->second) {
          if (JunctionExit::NO_TURN != lane_lane_seq.turn_type_) {
            continue;
          }
          JunctionExit* junction_exit_ptr =
              latest_junction_feature->add_junction_exit();
          SetExitFeature(junction_exit_ptr, lane_lane_seq, turn_type);
          is_find_seq = true;
          break;
        }
      }
    }
  }
  if (!is_find_seq) {
    JunctionExit* junction_exit_ptr =
        latest_junction_feature->add_junction_exit();
    junction_exit_ptr->CopyFrom(exits.at(index));
    junction_exit_ptr->set_turn_type(turn_type);
  }
}

void JunctionAnalyzer::SetExitFeature(
    JunctionExit* junction_exit_ptr, const JunctionLaneSequence& lane_seq,
    JunctionExit::JunctionTurnType turn_type) {
  if (nullptr == junction_exit_ptr) {
    return;
  }
  for (const auto& lane_id : lane_seq) {
    const auto lane = PredictionMap::LaneById(lane_id);
    if (nullptr == lane) {
      continue;
    }
    junction_exit_ptr->add_exit_lane_sequence(lane_id);
  }
  const auto& exit_lane_id = lane_seq.back();
  const auto exit_lane = PredictionMap::LaneById(exit_lane_id);
  if (nullptr == exit_lane) {
    return;
  }
  junction_exit_ptr->set_exit_lane_id(exit_lane_id);
  const auto& road_section_id =
      PredictionMap::GetRoadSectionCombinedId(exit_lane);
  junction_exit_ptr->set_road_section_id(road_section_id);
  double s = 0.0;
  PointENU position = exit_lane->GetSmoothPoint(s);
  junction_exit_ptr->mutable_exit_position()->set_x(position.x());
  junction_exit_ptr->mutable_exit_position()->set_y(position.y());
  junction_exit_ptr->set_exit_heading(exit_lane->Heading(s));
  junction_exit_ptr->set_exit_width(exit_lane->GetWidth(s));
  junction_exit_ptr->set_turn_type(turn_type);
}

void JunctionAnalyzer::SetJunctionFeature(Feature* feature_ptr) {
  if (nullptr == feature_ptr || exit_road_section_heading_.empty()) {
    return;
  }

  // 将角度最接近0的出口塞入
  JunctionFeature* latest_junction_feature =
      feature_ptr->mutable_junction_feature();
  const auto& section_id_1 = exit_road_section_heading_.at(0).first;
  const auto& exits_1 = exit_groups_.at(section_id_1);
  for (int j = 0; j < exits_1.size(); ++j) {
    SetExitFeature(JunctionExit::NO_TURN, j, section_id_1, exits_1,
                   latest_junction_feature);
  }

  // 由于路口内使用QC，DBN junction predictor不再用
  // 只需要判断为路口场景，且有出口
  return;

  // 车辆即将进入路口，还在entry lane上
  std::list<JunctionLaneSequence> junc_lane_seq_list;
  std::list<std::string> tmp_entry_lane_ids;
  for (const auto& current_lane_feature :
       feature_ptr->lane().current_lane_feature()) {
    const auto& current_lane_id = current_lane_feature.lane_id();

    std::list<JunctionLaneSequence> tmp_junc_lane_seq_list =
        GetEntryLaneSequenceList(current_lane_id);

    for (const auto& lane_seq : tmp_junc_lane_seq_list) {
      junc_lane_seq_list.emplace_back(lane_seq);
    }
    if (!tmp_junc_lane_seq_list.empty()) {
      tmp_entry_lane_ids.emplace_back(current_lane_id);
    }
  }
  if (!junc_lane_seq_list.empty()) {
    // 车辆当前所在lane是entry lane，那么直接可以确定出口
    for (const auto& lane_seq : junc_lane_seq_list) {
      JunctionExit* junction_exit_ptr =
          latest_junction_feature->add_junction_exit();
      SetExitFeature(junction_exit_ptr, lane_seq, lane_seq.turn_type_);
    }
    // 观察到车辆出口
    if (latest_junction_feature->entry_lane_ids().empty()) {
      for (const auto& current_lane_feature :
           feature_ptr->lane().current_lane_feature()) {
        latest_junction_feature->add_entry_lane_ids(
            current_lane_feature.lane_id());
      }
    }

    latest_junction_feature->mutable_entry_lane_ids()->Clear();

    for (const auto& lane_id : tmp_entry_lane_ids) {
      latest_junction_feature->add_entry_lane_ids(lane_id);
    }

    ADEBUG << "Obstacle [" << feature_ptr->id() << "] is in entry lanes!";
    return;
  }

  // 判断车辆是否在直行lane seq上
  for (const auto& lane_seq_list_pair : noturn_lane_seq_list_map_) {
    const auto& lane_seq_list = lane_seq_list_pair.second;
    const auto& exit_lane_id = lane_seq_list.back().back();
    const auto exit_lane = PredictionMap::LaneById(exit_lane_id);
    if (nullptr == exit_lane) {
      continue;
    }
    const auto& road_section_id =
        PredictionMap::GetRoadSectionCombinedId(exit_lane);

    std::unordered_set<std::string> exist_exit_lane_ids;
    for (const auto& lane_seq : lane_seq_list) {
      for (const auto& current_lane_feature :
           feature_ptr->lane().current_lane_feature()) {
        const auto& current_lane_id = current_lane_feature.lane_id();
        if (std::any_of(lane_seq.begin(), lane_seq.end(),
                        [current_lane_id](const auto& tmp_lane_id) {
                          return current_lane_id == tmp_lane_id;
                        })) {
          // 获取出口的road section
          const auto& exit_group = exit_groups_.find(road_section_id);
          if (exit_group == exit_groups_.end()) {
            continue;
          }

          for (int j = 0; j < exit_group->second.size(); ++j) {
            if (lane_seq.back() == exit_group->second.at(j).exit_lane_id()) {
              // 找lane seq左边的出口
              if (j > 0) {
                const auto& left_exit_id =
                    exit_group->second.at(j - 1).exit_lane_id();
                if (exist_exit_lane_ids.find(left_exit_id) ==
                    exist_exit_lane_ids.end()) {
                  SetExitFeature(lane_seq.turn_type_, j + 1, road_section_id,
                                 exit_group->second, latest_junction_feature);
                  exist_exit_lane_ids.emplace(left_exit_id);
                }
              }

              // 找lane seq对应的出口
              if (exist_exit_lane_ids.find(lane_seq.back()) ==
                  exist_exit_lane_ids.end()) {
                JunctionExit* junction_exit_ptr =
                    latest_junction_feature->add_junction_exit();
                SetExitFeature(junction_exit_ptr, lane_seq,
                               lane_seq.turn_type_);
                exist_exit_lane_ids.emplace(lane_seq.back());
              }

              // 找lane seq右边的出口
              if (j < exit_group->second.size() - 1) {
                const auto& right_exit_id =
                    exit_group->second.at(j + 1).exit_lane_id();
                if (exist_exit_lane_ids.find(right_exit_id) ==
                    exist_exit_lane_ids.end()) {
                  SetExitFeature(lane_seq.turn_type_, j + 1, road_section_id,
                                 exit_group->second, latest_junction_feature);
                  exist_exit_lane_ids.emplace(right_exit_id);
                }
              }
            }
          }
          break;
        }
      }
    }
  }

  if (!latest_junction_feature->junction_exit().empty()) {
    ADEBUG << "Obstacle [" << feature_ptr->id() << "] is in no_turn lane seq!";
    return;
  }

  // 如果观察到了车辆从哪个入口进来的, 那么就根据lane seq确定出口, 并将lane seq对应出口的左右出口也作为出口考虑
  if (!latest_junction_feature->entry_lane_ids().empty()) {
    // 获取以入口lane为开始的lane seq
    for (const auto& lane_id : latest_junction_feature->entry_lane_ids()) {
      std::list<JunctionLaneSequence> tmp_junc_lane_seq_list =
          GetEntryLaneSequenceList(lane_id);
      for (const auto& lane_seq : tmp_junc_lane_seq_list) {
        if (lane_seq.turn_type_ == JunctionExit::NO_TURN) {
          int tmp_idx = std::max(0, static_cast<int>(lane_seq.size()) - 2);
          const auto& tmp_lane_id = lane_seq.at(tmp_idx);
          const auto tmp_lane = PredictionMap::LaneById(tmp_lane_id);
          if (nullptr == tmp_lane) {
            continue;
          }
          double heading_diff =
              common::math::AngleDiff(tmp_lane->headings().back(),
                                      feature_ptr->velocity_heading()) *
              180.0 / M_PI;
          if (std::fabs(heading_diff) > 15.0) {
            continue;
          }
        }

        junc_lane_seq_list.emplace_back(lane_seq);
      }
    }

    std::unordered_set<std::string> exist_exit_lane_ids;
    for (const auto& lane_seq : junc_lane_seq_list) {
      const auto tmp_lane = PredictionMap::LaneById(lane_seq.back());
      if (nullptr == tmp_lane) {
        continue;
      }
      const auto& road_section_id =
          PredictionMap::GetRoadSectionCombinedId(tmp_lane);

      // 获取出口的road section
      const auto& exit_group = exit_groups_.find(road_section_id);
      if (exit_group == exit_groups_.end()) {
        continue;
      }

      for (int i = 0; i < exit_group->second.size(); ++i) {
        if (lane_seq.back() == exit_group->second.at(i).exit_lane_id()) {
          // 找lane seq左边的出口
          if (i > 0) {
            const auto& left_exit_id =
                exit_group->second.at(i - 1).exit_lane_id();
            if (exist_exit_lane_ids.find(left_exit_id) ==
                exist_exit_lane_ids.end()) {
              SetExitFeature(lane_seq.turn_type_, i - 1, road_section_id,
                             exit_group->second, latest_junction_feature);
              exist_exit_lane_ids.emplace(left_exit_id);
            }
          }

          // 找lane seq对应的出口
          if (exist_exit_lane_ids.find(lane_seq.back()) ==
              exist_exit_lane_ids.end()) {
            JunctionExit* junction_exit_ptr =
                latest_junction_feature->add_junction_exit();
            SetExitFeature(junction_exit_ptr, lane_seq, lane_seq.turn_type_);
            exist_exit_lane_ids.emplace(lane_seq.back());
          }

          // 找lane seq右边的出口
          if (i < exit_group->second.size() - 1) {
            const auto& right_exit_id =
                exit_group->second.at(i + 1).exit_lane_id();
            if (exist_exit_lane_ids.find(right_exit_id) ==
                exist_exit_lane_ids.end()) {
              SetExitFeature(lane_seq.turn_type_, i + 1, road_section_id,
                             exit_group->second, latest_junction_feature);
              exist_exit_lane_ids.emplace(right_exit_id);
            }
          }
        }
      }
    }

    if (!latest_junction_feature->junction_exit().empty()) {
      ADEBUG << "Obstacle [" << feature_ptr->id()
             << "] get exits according to observed entry lanes!";
      return;
    }
  }

  // 如果obs在junction的边角内
  bool is_in_corner = false;
  common::Point2D obs_p;
  obs_p.set_x(feature_ptr->position().x());
  obs_p.set_y(feature_ptr->position().y());
  for (const auto& iter : exit_road_section_lane_seq_map_) {
    for (const auto& iter2 : iter.second) {
      const auto& seq_list = iter2.second;
      for (const auto& seq : seq_list) {
        if (seq.turn_type_ != JunctionExit::RIGHT_TURN) {
          continue;
        }
        const auto lane_info = PredictionMap::LaneById(seq.back());
        if (nullptr == lane_info || lane_info->GetRightBoundPoints().empty()) {
          continue;
        }
        const auto& p1 = lane_info->GetRightBoundPoints().front();
        double heading1 = lane_info->Heading(0.0);

        common::Point2D start_p;
        start_p.set_x(p1.x());
        start_p.set_y(p1.y());
        common::Point2D end_p;
        end_p.set_x(p1.x() + std::cos(heading1));
        end_p.set_y(p1.y() + std::sin(heading1));

        double res = prediction_util::point_on_line_side(start_p, end_p, obs_p);
        if (std::isgreater(res, 0.0)) {
          continue;
        }
        const auto lane_info_2 = PredictionMap::LaneById(seq.front());
        if (nullptr == lane_info_2) {
          continue;
        }
        if (nullptr == lane_info_2 ||
            lane_info_2->GetLeftBoundPoints().empty()) {
          continue;
        }
        const auto& p2 = lane_info_2->GetLeftBoundPoints().back();
        double heading =
            lane_info_2->Heading(lane_info_2->total_length() - 0.001);
        start_p.set_x(p2.x());
        start_p.set_y(p2.y());
        end_p.set_x(p2.x() + std::cos(heading));
        end_p.set_y(p2.y() + std::sin(heading));

        res = prediction_util::point_on_line_side(start_p, end_p, obs_p);
        if (std::isgreater(res, 0.0)) {
          continue;
        }

        is_in_corner = true;

        double heading_diff =
            common::math::AngleDiff(heading1, feature_ptr->velocity_heading());
        heading_diff =
            common::math::NormalizeAngle(heading_diff) * 180.0 / M_PI;
        if (heading_diff < -5.0 || heading_diff > 90.0) {
          continue;
        }

        std::string road_section_id = iter.first;
        const auto& exits = exit_groups_.at(road_section_id);

        if (exits.size() > 1) {
          SetExitFeature(JunctionExit::RIGHT_TURN,
                         static_cast<int>(exits.size()) - 2, road_section_id,
                         exits, latest_junction_feature);
        }
        SetExitFeature(JunctionExit::RIGHT_TURN,
                       static_cast<int>(exits.size()) - 1, road_section_id,
                       exits, latest_junction_feature);

        break;
      }
    }
  }
  if (!latest_junction_feature->junction_exit().empty() || is_in_corner) {
    ADEBUG << "Obstacle [" << feature_ptr->id() << "] is in cornor!";
    return;
  }

  for (int i = 0; i < exit_road_section_heading_.size(); ++i) {
    const auto& section_id_1 = exit_road_section_heading_.at(i).first;
    const auto& exits_1 = exit_groups_.at(section_id_1);
    const auto& right_exit = exits_1.back();
    double right_exit_heading = right_exit.exit_heading();
    const auto& right_lane_info =
        PredictionMap::LaneById(right_exit.exit_lane_id());
    if (nullptr == right_lane_info) {
      continue;
    }
    const auto& p1 = right_lane_info->GetRightBoundPoints().front();
    common::Point2D start_p;
    start_p.set_x(p1.x());
    start_p.set_y(p1.y());
    common::Point2D end_p;
    end_p.set_x(p1.x() + std::cos(right_exit_heading));
    end_p.set_y(p1.y() + std::sin(right_exit_heading));

    double res = prediction_util::point_on_line_side(start_p, end_p, obs_p);
    if (std::isgreater(res, 0.0)) {
      continue;
    }

    int prev_idx = i - 1;
    if (prev_idx < 0) {
      prev_idx = static_cast<int>(exit_road_section_heading_.size()) - 1;
    }
    const auto& section_id_2 = exit_road_section_heading_.at(prev_idx).first;

    const auto& exits_2 = exit_groups_.at(section_id_2);
    const auto& right_exit_2 = exits_2.back();

    double right_exit_heading_2 = right_exit_2.exit_heading();
    const auto& right_lane_info_2 =
        PredictionMap::LaneById(right_exit_2.exit_lane_id());
    if (nullptr == right_lane_info_2) {
      continue;
    }
    const auto& p2 = right_lane_info_2->GetLeftBoundPoints().front();
    start_p.set_x(p2.x());
    start_p.set_y(p2.y());
    end_p.set_x(p2.x() + std::cos(right_exit_heading_2));
    end_p.set_y(p2.y() + std::sin(right_exit_heading_2));

    res = prediction_util::point_on_line_side(start_p, end_p, obs_p);
    if (std::isgreater(res, 0.0)) {
      continue;
    }

    double heading_diff = common::math::AngleDiff(
        right_exit_heading_2, feature_ptr->velocity_heading());
    heading_diff = common::math::NormalizeAngle(heading_diff) * 180.0 / M_PI;
    if (heading_diff < -10.0 || heading_diff > 90.0) {
      continue;
    }

    if (exits_2.size() > 1) {
      SetExitFeature(JunctionExit::RIGHT_TURN,
                     static_cast<int>(exits_2.size()) - 2, section_id_2,
                     exits_2, latest_junction_feature);
    }
    SetExitFeature(JunctionExit::RIGHT_TURN,
                   static_cast<int>(exits_2.size()) - 1, section_id_2, exits_2,
                   latest_junction_feature);
  }
  if (!latest_junction_feature->junction_exit().empty()) {
    ADEBUG << "Obstacle [" << feature_ptr->id() << "] is in cornor(2)!";
    return;
  }

  // 如果obs方向在两个出口方向之间
  for (int i = 0; i < exit_road_section_heading_.size(); ++i) {
    double heading1 = exit_road_section_heading_.at(i).second;
    int next_idx = i + 1;
    if (exit_road_section_heading_.size() == next_idx) {
      next_idx = 0;
    }
    double heading2 = exit_road_section_heading_.at(next_idx).second;
    double velocity_heading = feature_ptr->velocity_heading();

    if (!common::math::AngleInRange(velocity_heading, heading1, heading2)) {
      continue;
    }
    const auto& section_id_1 = exit_road_section_heading_.at(i).first;
    const auto& exits_1 = exit_groups_.at(section_id_1);
    for (int j = 0; j < exits_1.size(); ++j) {
      SetExitFeature(JunctionExit::NO_TURN, j, section_id_1, exits_1,
                     latest_junction_feature);
    }

    const auto& section_id_2 = exit_road_section_heading_.at(next_idx).first;
    const auto& exits_2 = exit_groups_.at(section_id_2);
    for (int j = 0; j < exits_2.size(); ++j) {
      SetExitFeature(JunctionExit::LEFT_TURN, j, section_id_2, exits_2,
                     latest_junction_feature);
    }
  }

  ADEBUG << "Obstacle [" << feature_ptr->id()
         << "] get exits according heading diff!";
}

std::list<JunctionLaneSequence> JunctionAnalyzer::GetEntryLaneSequenceList(
    const std::string& lane_id) {
  auto iter = entry_lane_lane_seq_map_.find(lane_id);
  if (iter == entry_lane_lane_seq_map_.end()) {
    return {};  // 如果未找到车道 ID，返回一个空列表
  }

  // 返回与车道 ID 相关联的列表
  return iter->second;
}

bool JunctionAnalyzer::IsEntryLane(const std::string& lane_id) {
  const auto iter = entry_lane_lane_seq_map_.find(lane_id);
  return iter != entry_lane_lane_seq_map_.end();
}

bool JunctionAnalyzer::IsExitLane(const std::string& lane_id) {
  return junction_exits_.find(lane_id) != junction_exits_.end();
}

const std::string& JunctionAnalyzer::GetJunctionId() {
  return junction_id_;
}

double JunctionAnalyzer::ComputeJunctionRange() {
  const auto& junction_info_ptr = PredictionMap::JunctionById(junction_id_);
  if (nullptr == junction_info_ptr) {
    return 0.0;
  }

  if (!junction_info_ptr->junction().has_polygon() ||
      junction_info_ptr->junction().polygon().point_size() < 3) {
    AERROR << "Junction [" << GetJunctionId()
           << "] has not enough polygon points to compute range";
    return FLAGS_defualt_junction_range;
  }
  double x_min = std::numeric_limits<double>::infinity();
  double x_max = -std::numeric_limits<double>::infinity();
  double y_min = std::numeric_limits<double>::infinity();
  double y_max = -std::numeric_limits<double>::infinity();
  for (const auto& point : junction_info_ptr->junction().polygon().point()) {
    x_min = std::min(x_min, point.x());
    x_max = std::max(x_max, point.x());
    y_min = std::min(y_min, point.y());
    y_max = std::max(y_max, point.y());
  }
  double dx = std::abs(x_max - x_min);
  double dy = std::abs(y_max - y_min);
  double range = std::sqrt(dx * dx + dy * dy);
  return range;
}

}  // namespace prediction
}  // namespace TL
