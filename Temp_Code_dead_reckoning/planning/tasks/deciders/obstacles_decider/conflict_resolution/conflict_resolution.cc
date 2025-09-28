/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  conflict_resolution.cc
 */

#include "planning/tasks/deciders/obstacles_decider/conflict_resolution/conflict_resolution.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include "planning/common/obstacle.h"
#include "planning/common/path_decision.h"

namespace TL::planning {

namespace {
constexpr int kMaxCounter = 20;
constexpr int kLateralDistance = 1.0;
}  // namespace

ConflictResolution::ConflictResolution(const bool enable_debug)
    : enable_debug_(enable_debug) {}

void ConflictResolution::Debug(ReferenceLineInfo* const reference_line_info) {
  auto* path_decision = reference_line_info->path_decision();
  if (path_decision == nullptr) {
    return;
  }
  auto* avoid_alongside_debug = reference_line_info->mutable_debug()
                                    ->mutable_avoid_alongside_decider_info();
  for (const auto* const obs : path_decision->obstacles().Items()) {
    if (obs == nullptr) {
      continue;
    }
    for (const auto& counter_info : history_decision_nudge_counter_info_) {
      const auto& history_obs_perception_id = counter_info.first.second;
      const auto& history_obs_prediction_id = counter_info.first.first;
      if (history_obs_perception_id != obs->PerceptionId()) {
        continue;
      }
      auto* decision_info =
          avoid_alongside_debug->mutable_decision_info()->Add();
      decision_info->set_id(history_obs_prediction_id);
      auto* tags = decision_info->mutable_decision_tags();
      for (const auto& tag : obs->decider_tags()) {
        tags->Add()->assign(tag);
      }
    }
  }
  for (const auto& counter_info : history_decision_nudge_counter_info_) {
    ADEBUG << "prediction id: " << counter_info.first.first
           << ", perception id: " << counter_info.first.second
           << ", behavior: " << counter_info.second.first
           << ", times: " << counter_info.second.second;
  }
}

void ConflictResolution::StableDecisionInfomation(
    ReferenceLineInfo* const reference_line_info) {
  auto* path_decision = reference_line_info->path_decision();
  if (path_decision == nullptr) {
    return;
  }
  ObjectDecisionType object_decision;
  ObjectNudge* object_nudge_ptr = object_decision.mutable_nudge();
  object_nudge_ptr->set_distance_l(0.1);
  std::set<ObsPredictionAndPerceptionId> remove_history_obs;
  for (const auto& history_decision : history_decision_nudge_counter_info_) {
    const auto& perception_id = history_decision.first.second;
    const auto& prediction_id = history_decision.first.first;
    const auto& obstacle_id = history_decision.first;
    const auto& decision_info = history_decision.second;
    // 1.比较变化次数，次数小于阈值，2.纵向有重叠的避让状态且横向距离小于1m 稳定决策，大于等于做变更并清除历史信息
    const Obstacle* obstacle =
        reference_line_info->path_decision()->obstacles().Find(prediction_id);
    const bool is_s_overlap_with_ego =
        obstacle != nullptr &&
        obstacle->PerceptionSLBoundary().start_s() <
            reference_line_info->AdcSlBoundary().end_s() &&
        obstacle->PerceptionSLBoundary().end_s() >
            reference_line_info->AdcSlBoundary().start_s();
    bool is_l_close_to_ego = false;
    if (obstacle != nullptr) {
      const auto obs_start_l = obstacle->PerceptionSLBoundary().start_l();
      const auto obs_end_l = obstacle->PerceptionSLBoundary().end_l();
      const bool is_left_obstacle =
          obs_start_l + obs_end_l >
          reference_line_info->AdcSlBoundary().start_l() +
              reference_line_info->AdcSlBoundary().end_l();
      is_l_close_to_ego =
          is_left_obstacle
              ? (obstacle->PerceptionSLBoundary().start_l() -
                     reference_line_info->AdcSlBoundary().end_l() <
                 kLateralDistance)
              : (reference_line_info->AdcSlBoundary().start_l() - obs_end_l <
                 kLateralDistance);
    }
    if (decision_info.second < kMaxCounter ||
        (is_s_overlap_with_ego && is_l_close_to_ego)) {
      for (const auto* obstacle :
           reference_line_info->path_decision()->obstacles().Items()) {
        if (obstacle == nullptr) {
          continue;
        }
        if (obstacle->PerceptionId() != perception_id) {
          continue;
        }
        if (obstacle->IsLateralIgnore() || obstacle->IsLongitudinalIgnore()) {
          continue;
        }
        if (decision_info.first == LEFT_NUDGE) {
          if (obstacle->IsStatic()) {
            object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
            path_decision->AddLateralDecision(
                "obstacles_decider/static-left-nudge-stable", obstacle->Id(),
                object_decision);
          } else {
            object_nudge_ptr->set_type(ObjectNudge::DYNAMIC_LEFT_NUDGE);
            path_decision->AddLateralDecision(
                "obstacles_decider/dynamic-left-nudge-stable", obstacle->Id(),
                object_decision);
          }
        }

        if (decision_info.first == RIGHT_NUDGE) {
          if (obstacle->IsStatic()) {
            object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
            path_decision->AddLateralDecision(
                "obstacles_decider/static-right-nudge-stable", obstacle->Id(),
                object_decision);
          } else {
            object_nudge_ptr->set_type(ObjectNudge::DYNAMIC_RIGHT_NUDGE);
            path_decision->AddLateralDecision(
                "obstacles_decider/dynamic-right-nudge-stable", obstacle->Id(),
                object_decision);
          }
        }
      }
    } else {
      remove_history_obs.emplace(obstacle_id);
    }
  }

  for (const auto& obs : remove_history_obs) {
    history_decision_nudge_counter_info_.erase(obs);
  }
}

void ConflictResolution::MaintainingHistoricalInfomation(
    const ReferenceLineInfo& reference_line_info,
    const ObsPredictionAndPerceptionId& obstacle_id,
    const BehaviorType& behavior_type) {
  UNUSED(reference_line_info);
  // 历史上第一次出现决策
  if (history_decision_nudge_counter_info_.find(obstacle_id) ==
      history_decision_nudge_counter_info_.end()) {
    if (behavior_type == LEFT_NUDGE) {
      history_decision_nudge_counter_info_[obstacle_id].first = LEFT_NUDGE;
      history_decision_nudge_counter_info_[obstacle_id].second = 0;
    } else if (behavior_type == RIGHT_NUDGE) {
      history_decision_nudge_counter_info_[obstacle_id].first = RIGHT_NUDGE;
      history_decision_nudge_counter_info_[obstacle_id].second = 0;
    }
  } else {
    // 历史上有过决策
    // 历史上的决策是left nudge
    if (history_decision_nudge_counter_info_[obstacle_id].first == LEFT_NUDGE) {
      if (behavior_type == LEFT_NUDGE) {
        history_decision_nudge_counter_info_[obstacle_id].second = 0;
      } else {
        history_decision_nudge_counter_info_[obstacle_id].second = std::min(
            history_decision_nudge_counter_info_[obstacle_id].second + 1,
            kMaxCounter);
      }
    }

    // 历史上的决策是right nudge
    if (history_decision_nudge_counter_info_[obstacle_id].first ==
        RIGHT_NUDGE) {
      if (behavior_type == RIGHT_NUDGE) {
        history_decision_nudge_counter_info_[obstacle_id].second = 0;
      } else {
        history_decision_nudge_counter_info_[obstacle_id].second = std::min(
            history_decision_nudge_counter_info_[obstacle_id].second + 1,
            kMaxCounter);
      }
    }
  }
}

void ConflictResolution::ClearCache() {
  history_decision_nudge_counter_info_.clear();
}

void ConflictResolution::Process(ReferenceLineInfo* const reference_line_info) {
  if (reference_line_info == nullptr) {
    return;
  }
  auto* path_decision = reference_line_info->path_decision();
  if (path_decision == nullptr) {
    return;
  }

  // static obs change to dynamic obs and vice versa
  for (const auto& history_decision : history_decision_nudge_counter_info_) {
    const auto& obstacle_id = history_decision.first;
    const auto& prediction_id = history_decision.first.first;
    // 障碍物消失，次数+1
    if (reference_line_info->path_decision()->obstacles().Find(prediction_id) ==
        nullptr) {
      history_decision_nudge_counter_info_[obstacle_id].second =
          std::min(history_decision_nudge_counter_info_[obstacle_id].second + 1,
                   kMaxCounter);
    }
  }

  for (const auto& obstacle : path_decision->obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }
    const auto prediction_id = obstacle->Id();
    const auto perception_id = obstacle->PerceptionId();
    const auto obstacle_id = std::make_pair(prediction_id, perception_id);
    BehaviorType behavior_type = UNKNOW;
    if (obstacle->LateralDecision().has_nudge()) {
      if ((obstacle->LateralDecision().nudge().type() ==
               ObjectNudge::DYNAMIC_LEFT_NUDGE ||
           obstacle->LateralDecision().nudge().type() ==
               ObjectNudge::LEFT_NUDGE)) {
        behavior_type = LEFT_NUDGE;
      }
      if (obstacle->LateralDecision().nudge().type() ==
              ObjectNudge::DYNAMIC_RIGHT_NUDGE ||
          obstacle->LateralDecision().nudge().type() ==
              ObjectNudge::RIGHT_NUDGE) {
        behavior_type = RIGHT_NUDGE;
      }
    }

    // 维护历史量
    MaintainingHistoricalInfomation(*reference_line_info, obstacle_id,
                                    behavior_type);
  }
  // 更新决策信息
  StableDecisionInfomation(reference_line_info);

  // debug 信息
  if (enable_debug_) {
    Debug(reference_line_info);
  }
}
};  // namespace TL::planning
