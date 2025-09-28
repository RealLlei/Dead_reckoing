/*
 * Copyright (c) 2022 TL Technologies Co., Ltd. All rights reserved.
 */

#pragma once
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "planning/common/reference_line_info.h"
#include "planning/tasks/deciders/obstacles_decider/obstacle_selector/obstacles_info_updater.h"
#include "planning/proto/planning_config.pb.h"

namespace TL {
namespace planning {

using InteractiveDecision = std::pair<
    std::string,
    std::pair<ObjectDecisionType, std::pair<InteractiveObsBehavior::Behavior,
                                            InteractiveObsBehavior::Behavior>>>;

class BehaviorConflictResolution {
 public:
  BehaviorConflictResolution() = default;

  void Process(
      ReferenceLineInfo* reference_line_info,
      const std::unordered_map<std::string, ObstacleInfo>& obstacles_info,
      const std::pair<uint32_t, InteractiveDecision>& current_behavior_info);

  void GetKeepDecisionID(
      const std::pair<uint32_t, InteractiveDecision>& current_behavior_info,
      const std::unordered_map<std::string, ObstacleInfo>& obstacles_info,
      PathDecision* path_decision, double adc_end_s);

  void RemoveHistoryID(
      const std::unordered_map<std::string, ObstacleInfo>& obstacles_info,
      PathDecision* path_decision, double adc_end_s,
      const std::pair<uint32_t, InteractiveDecision>& current_behavior_info);

  void ClearCache();

  const std::vector<InteractiveDecision>& GetBehaviorConflictDecision() const {
    return behavior_conflict_decision_vec_;
  }

 private:
  std::pair<uint32_t, InteractiveDecision> last_adc_obs_behavior_pair_ = {};
  std::unordered_map<std::string, uint32_t> keep_decision_obs_ids_ = {};
  std::vector<InteractiveDecision> behavior_conflict_decision_vec_;
};

}  // namespace planning
}  // namespace TL
