/*
 * Copyright (c) 2022 TL Technologies Co., Ltd. All rights reserved.
 */

#include "planning/tasks/deciders/obstacles_decider/conflict_resolution/behavior_conflict_resolution.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_set>
#include <vector>

#include "planning/common/path_decision.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/planning/sl_boundary.pb.h"

namespace TL {
namespace planning {

void BehaviorConflictResolution::Process(
    ReferenceLineInfo* reference_line_info,
    const std::unordered_map<std::string, ObstacleInfo>& obstacles_info,
    const std::pair<uint32_t, InteractiveDecision>& current_behavior_info) {
  behavior_conflict_decision_vec_.clear();
  auto* const path_decision = reference_line_info->path_decision();
  const auto& adc_iter = obstacles_info.find("adc");
  if (adc_iter == obstacles_info.end()) {
    return;
  }
  const double adc_end_s = adc_iter->second.end_s;

  GetKeepDecisionID(current_behavior_info, obstacles_info, path_decision,
                    adc_end_s);

  RemoveHistoryID(obstacles_info, path_decision, adc_end_s,
                  current_behavior_info);

  // history behavior
  last_adc_obs_behavior_pair_ = current_behavior_info;
}

void BehaviorConflictResolution::GetKeepDecisionID(
    const std::pair<uint32_t, InteractiveDecision>& current_behavior_info,
    const std::unordered_map<std::string, ObstacleInfo>& obstacles_info,
    PathDecision* path_decision, double adc_end_s) {
  UNUSED(path_decision);
  const uint32_t current_perception_id = current_behavior_info.first;
  const auto last_perception_id = last_adc_obs_behavior_pair_.first;
  const auto& last_decision = last_adc_obs_behavior_pair_.second.second.first;
  ADEBUG << "last_id=" << last_adc_obs_behavior_pair_.second.first;
  ADEBUG << "last_decision=" << last_decision.DebugString();
  if (current_perception_id == last_perception_id) {
    return;
  }
  if (!last_decision.has_yield()) {
    return;
  }

  for (const auto& obs : obstacles_info) {
    if (obs.second.perception_id == last_perception_id) {
      // last_id is still cutin in current cycle
      if (obs.second.init_behavior == ObstacleBehavior::CutIn &&
          obs.second.start_s < adc_end_s) {
        ADEBUG << "id=" << obs.first
               << " first appears in keep_decision_obs_ids_";
        keep_decision_obs_ids_.insert({obs.first, obs.second.perception_id});
      }
    }
  }
}

void BehaviorConflictResolution::RemoveHistoryID(
    const std::unordered_map<std::string, ObstacleInfo>& obstacles_info,
    PathDecision* path_decision, double adc_end_s,
    const std::pair<uint32_t, InteractiveDecision>& current_behavior_info) {
  UNUSED(path_decision);
  std::unordered_map<std::string, uint32_t> remove_history_obs_ids;
  for (const auto& keep_id : keep_decision_obs_ids_) {
    // keep_id is interactive_obs again
    if (keep_id.second == current_behavior_info.first) {
      remove_history_obs_ids.insert(keep_id);
      continue;
    }

    bool is_keep_id_in_current_obstacles_info = false;
    for (const auto& obs : obstacles_info) {
      if (obs.second.perception_id == keep_id.second) {
        is_keep_id_in_current_obstacles_info = true;
        bool is_obs_in_range =
            (obs.second.init_behavior == ObstacleBehavior::CutIn) ||
            (obs.second.init_behavior == ObstacleBehavior::Alongside_Left ||
             obs.second.init_behavior == ObstacleBehavior::Alongside_Right);
        if (obs.second.start_s < adc_end_s && is_obs_in_range) {
          ADEBUG << "id=" << obs.first
                 << " keeps decision in keep_decision_obs_ids_";
        } else {
          // keep_id is in the front of adc or out of lateral range
          remove_history_obs_ids.insert({obs.first, obs.second.perception_id});
        }
      }
    }

    if (!is_keep_id_in_current_obstacles_info) {
      remove_history_obs_ids.insert(keep_id);
    }
  }

  for (const auto& obs_id : remove_history_obs_ids) {
    keep_decision_obs_ids_.erase(obs_id.first);
  }

  for (const auto& obs_id : keep_decision_obs_ids_) {
    ADEBUG << "keep_decision_obs_id=" << obs_id.first;
    ObjectDecisionType decision;
    decision.mutable_yield();
    InteractiveDecision adc_obs_behavior_pair = {
        obs_id.first,
        {decision,
         {InteractiveObsBehavior::ConstantSpeed,
          InteractiveObsBehavior::ConstantSpeed}}};
    behavior_conflict_decision_vec_.emplace_back(adc_obs_behavior_pair);
  }
}

void BehaviorConflictResolution::ClearCache() {
  last_adc_obs_behavior_pair_ = {};
  keep_decision_obs_ids_.clear();
}

}  // namespace planning
}  // namespace TL
