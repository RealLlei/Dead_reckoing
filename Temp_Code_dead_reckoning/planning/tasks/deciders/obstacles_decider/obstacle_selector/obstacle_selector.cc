/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  obstacle_selector.cc
 */

#include "planning/tasks/deciders/obstacles_decider/obstacle_selector/obstacle_selector.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include "common/file/log.h"
#include "common/util/util.h"
#include "planning/common/obstacle.h"
#include "planning/common/obstacle_blocking_analyzer.h"

namespace TL::planning {

namespace {
constexpr double kDecisionObstaclesSDist = 70.0;
constexpr double kDecisionObstaclesLDist = 7.0;
constexpr double kDecisionObstaclesLOverlapDist = 1.0;
constexpr double kDecisionObstaclesLOverlapNeighborLaneDist = 0.25;
constexpr double kVirtualNeighborExtendWidth = 3.5;
constexpr double kCutinAngle = 5.0 / 180.0 * M_PI;
constexpr double kCutinLateralSpeed = 1.0;
constexpr double kCutoutLateralSpeed = 0.5;
// 此处按照交付要求，稳定存在的障碍物判定改为6帧
constexpr int kStableDecisionObstacleFrameSize = 6;
constexpr int kStableDangerousObstacleFrameSize = 3;
constexpr double kTotalBehind = 20.0;
constexpr double kAdcLowSpeed = 0.5;
constexpr double kIgnoreTTC = 6.5;
constexpr double kDangerousObstacleIgnoreTTC = 5.0;
constexpr double kBeforeObsDist = 8.0;
// 此参数产品要求，大车距离主车一侧车道线距离大于1m，不下发决策
constexpr double kIgnoreLateralDist = 1.0;
// 此处按照交付设计要求，路口虚拟车道线附近的障碍物caution
constexpr double kVirtualAeraCautionFrontDist = 80.0;
constexpr double kVirtualAeraCautionBackDist = 8.0;
constexpr double kVirtualAeraCautionLateralDist = 5.0;
constexpr double kContinuousConeDist = 50.0;
// 增加静态障碍物避障条件,解决LCC模式下远端车道线不稳的问题.
// 1.前方30内
// 2.ttc < 3
constexpr double kStaticObstacleNudgeDist = 30.0;
constexpr double kStaticObstacleNudgeTtc = 3.0;
}  // namespace

void ObstacleSelector::DoLateralDecision(
    const Obstacle& obstacle, const bool& is_left_obstacle,
    const bool& is_right_obstacle,
    ReferenceLineInfo* const reference_line_info) {
  if (reference_line_info == nullptr) {
    return;
  }

  ObjectDecisionType object_decision;
  auto* path_decision = reference_line_info->path_decision();
  if (path_decision == nullptr) {
    return;
  }
  ObjectNudge* object_nudge_ptr = object_decision.mutable_nudge();
  object_nudge_ptr->set_distance_l(0.1);
  if (is_right_obstacle) {
    ADEBUG << "decision_obstacle id:" << obstacle.Id()
           << " is need left nudge!";
    object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
    path_decision->AddLateralDecision("obstacles_decider/static-left-nudge",
                                      obstacle.Id(), object_decision);
  } else if (is_left_obstacle) {
    ADEBUG << "decision_obstacle id:" << obstacle.Id()
           << " is need right nudge!";
    object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
    path_decision->AddLateralDecision("obstacles_decider/static-right-nudge",
                                      obstacle.Id(), object_decision);
  }
}

void ObstacleSelector::DoStaticObstacleDecision(
    const Obstacle& obstacle, const bool& is_left_obstacle,
    const bool& is_right_obstacle,
    ReferenceLineInfo* const reference_line_info) {
  if (reference_line_info == nullptr) {
    return;
  }

  DoLateralDecision(obstacle, is_left_obstacle, is_right_obstacle,
                    reference_line_info);

  ADEBUG << "###static decision_obstacle id:" << obstacle.Id()
         << " is need yield!";
  ObjectDecisionType object_decision;
  auto* path_decision = reference_line_info->path_decision();
  if (path_decision == nullptr) {
    return;
  }
  auto* fence_point = object_decision.mutable_yield()->mutable_fence_point();
  fence_point->set_x(obstacle.PerceptionBoundingBox().center_x());
  fence_point->set_y(obstacle.PerceptionBoundingBox().center_y());
  object_decision.mutable_overtake()->set_fence_heading(
      obstacle.PerceptionBoundingBox().heading());
  path_decision->AddLongitudinalDecision("obstacles_decider/-yield",
                                         obstacle.Id(), object_decision);
}

bool ObstacleSelector::IsStableObstacle(const Obstacle& obstacle,
                                        const int stable_size) {
  if (!obstacle.TrackStatus().has_is_tracking()) {
    return false;
  }
  if (!obstacle.TrackStatus().is_tracking()) {
    return false;
  }
  if (obstacle.TrackStatus().tracking_counter() < stable_size &&
      obstacle.TrackStatus().motion_tracking_counter() < stable_size) {
    return false;
  }
  return true;
}

bool ObstacleSelector::IsContinuousCone(
    const Obstacle& unknow_obs, const ReferenceLineInfo& reference_line_info) {
  if (unknow_obs.Perception().sub_type() !=
      perception::PerceptionObstacle::ST_TRAFFICCONE) {
    return false;
  }
  const auto& obstacles =
      reference_line_info.path_decision().obstacles().Items();
  return std::any_of(
      obstacles.begin(), obstacles.end(), [&](const Obstacle* const& obs) {
        if (obs == nullptr) {
          return false;
        }
        if (obs->Perception().sub_type() !=
            perception::PerceptionObstacle::ST_TRAFFICCONE) {
          return false;
        }
        if (last_frame_overlap_lane_obs_ids_.find(obs->PerceptionId()) ==
            last_frame_overlap_lane_obs_ids_.end()) {
          return false;
        }
        return unknow_obs.PerceptionSLBoundary().start_s() -
                       obs->PerceptionSLBoundary().end_s() <
                   kContinuousConeDist &&
               unknow_obs.PerceptionSLBoundary().start_s() -
                       obs->PerceptionSLBoundary().end_s() >
                   0.0;
      });
}

bool ObstacleSelector::GetMapInfo(const double& check_s,
                                  const ReferenceLineInfo& reference_line_info,
                                  double* const lane_left_width,
                                  double* const lane_right_width,
                                  double* const left_neighbor_lane_width,
                                  double* const right_neighbor_lane_width) {
  if (left_neighbor_lane_width == nullptr ||
      right_neighbor_lane_width == nullptr) {
    return false;
  }
  if (!reference_line_info.reference_line().GetLaneWidth(
          check_s, lane_left_width, lane_right_width)) {
    ADEBUG << "check_s: " << check_s << " can`t get lane width";
    return false;
  }

  hdmap::Id left_neighbor_lane_id;
  hdmap::Id right_neighbor_lane_id;
  if (reference_line_info.GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftForward, check_s,
          &left_neighbor_lane_id, left_neighbor_lane_width)) {
    ADEBUG << "Extend left forward neighbor lane.";
  } else if (reference_line_info.GetNeighborLaneInfo(
                 ReferenceLineInfo::LaneType::LeftReverse, check_s,
                 &left_neighbor_lane_id, left_neighbor_lane_width)) {
    ADEBUG << "Extend left reverse neighbor lane.";
  } else {
    *left_neighbor_lane_width = kVirtualNeighborExtendWidth;
    ADEBUG << "There is no left neighbor lane.";
  }
  if (reference_line_info.GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::RightForward, check_s,
          &right_neighbor_lane_id, right_neighbor_lane_width)) {
    ADEBUG << "Extend right forward neighbor lane.";
  } else if (reference_line_info.GetNeighborLaneInfo(
                 ReferenceLineInfo::LaneType::RightReverse, check_s,
                 &right_neighbor_lane_id, right_neighbor_lane_width)) {
    ADEBUG << "Extend right reverse neighbor lane.";
  } else {
    *right_neighbor_lane_width = kVirtualNeighborExtendWidth;
    ADEBUG << "There is no right neighbor lane.";
  }
  return true;
}

double ObstacleSelector::GetTTC(const Obstacle& obstacle,
                                const ReferenceLineInfo& reference_line_info) {
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_start_s = reference_line_info.AdcSlBoundary().start_s();
  const double obs_start_s = obstacle.PerceptionSLBoundary().start_s();
  const double obs_end_s = obstacle.PerceptionSLBoundary().end_s();
  const double obs_distance_to_adc =
      GetDistanceBetweenADCAndObstacle(&reference_line_info, &obstacle);
  const bool is_front_obstacle =
      adc_start_s + adc_end_s < obs_start_s + obs_end_s;
  double ttc = 0.0;
  if (is_front_obstacle) {
    ttc = common::util::IsFloatEqual(
              obstacle.speed(),
              reference_line_info.vehicle_state().linear_velocity())
              ? std::numeric_limits<double>::max()
              : obs_distance_to_adc /
                    (reference_line_info.vehicle_state().linear_velocity() -
                     obstacle.speed());
  } else {
    ttc = common::util::IsFloatEqual(
              obstacle.speed(),
              reference_line_info.vehicle_state().linear_velocity())
              ? std::numeric_limits<double>::max()
              : obs_distance_to_adc /
                    (obstacle.speed() -
                     reference_line_info.vehicle_state().linear_velocity());
  }
  return ttc;
}

bool ObstacleSelector::IsRoadRoi(const Obstacle& obstacle,
                                 const ReferenceLineInfo& reference_line_info,
                                 bool* const is_overlap_lane_obstacle,
                                 bool* const is_left_obstacle,
                                 bool* const is_right_obstacle,
                                 bool* is_away_from_lane) {
  if (is_overlap_lane_obstacle == nullptr || is_left_obstacle == nullptr ||
      is_right_obstacle == nullptr || is_away_from_lane == nullptr) {
    return false;
  }

  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_start_s = reference_line_info.AdcSlBoundary().start_s();
  const double adc_start_l = reference_line_info.AdcSlBoundary().start_l();
  const double adc_end_l = reference_line_info.AdcSlBoundary().end_l();
  const double obs_start_s = obstacle.PerceptionSLBoundary().start_s();
  const double obs_end_s = obstacle.PerceptionSLBoundary().end_s();
  const double obs_start_l = obstacle.PerceptionSLBoundary().start_l();
  const double obs_end_l = obstacle.PerceptionSLBoundary().end_l();
  // bound随速
  const double ttc = GetTTC(obstacle, reference_line_info);
  const bool out_ttc_condition = (ttc < 0.0 || ttc > kIgnoreTTC);
  const bool out_s_condition =
      obs_end_s < adc_start_s - kTotalBehind - obstacle.speed() ||
      adc_end_s + kDecisionObstaclesSDist +
              reference_line_info.vehicle_state().linear_velocity() <
          obs_start_s;
  const bool out_l_condition =
      adc_start_l - kDecisionObstaclesLDist > obs_end_l ||
      adc_end_l + kDecisionObstaclesLDist < obs_start_l;
  if ((out_s_condition && out_ttc_condition) || out_l_condition) {
    ADEBUG << "obs id: " << obstacle.Id() << " out s l bound."
           << " out_s_condition: " << out_s_condition
           << ", out_ttc_condition: " << out_ttc_condition
           << ", out_l_condition: " << out_l_condition;
    return false;
  }
  const bool l_overlap = obs_start_l < adc_end_l && obs_end_l > adc_start_l;
  if (adc_start_s > obs_end_s && l_overlap) {
    return false;
  }
  double lane_left_width = 1.75;
  double lane_right_width = 1.75;
  double right_neighbor_lane_width = 3.5;
  double left_neighbor_lane_width = 3.5;
  if (!GetMapInfo(0.5 * (obs_start_s + obs_end_s), reference_line_info,
                  &lane_left_width, &lane_right_width,
                  &left_neighbor_lane_width, &right_neighbor_lane_width)) {
    ADEBUG << "can`t GetMapInfo.";
    return false;
  }

  // 此处修改为产品要求，大车距离主车一侧车道线距离大于100cm，不下发决策
  *is_away_from_lane = obs_start_l > kIgnoreLateralDist + lane_left_width ||
                       obs_end_l < -lane_right_width - kIgnoreLateralDist;

  const bool last_frame_not_overlap_lane =
      last_frame_overlap_lane_obs_ids_.find(obstacle.PerceptionId()) ==
      last_frame_overlap_lane_obs_ids_.end();
  double stable_overlap_distance = 0.0;
  if (!last_frame_not_overlap_lane) {
    stable_overlap_distance = 0.1;
    if (obstacle.Perception().sub_type() ==
        perception::PerceptionObstacle::ST_TRAFFICCONE) {
      stable_overlap_distance = 0.25;
    }
  }
  if (obstacle.IsStatic()) {
    const double vehicle_width = vehicle_param_.width();
    *is_left_obstacle = obs_start_l + obs_end_l > 0.0 &&
                        obs_start_l + lane_right_width - vehicle_width >
                            FLAGS_nonstatic_obstacle_nudge_l_buffer &&
                        obs_start_l < lane_left_width + stable_overlap_distance;
    ADEBUG << "obs id: " << obstacle.Id() << ", obs_start_l: " << obs_start_l
           << ", lane_right_width: " << lane_right_width
           << ", vehicle_width: " << vehicle_width
           << ", FLAGS: " << FLAGS_nonstatic_obstacle_nudge_l_buffer
           << ", obs_start_l: " << obs_start_l
           << ", lane_left_width: " << lane_left_width
           << ", stable_overlap_distance: " << stable_overlap_distance;

    *is_right_obstacle =
        obs_start_l + obs_end_l < 0.0 &&
        lane_left_width - obs_end_l - vehicle_width >
            FLAGS_nonstatic_obstacle_nudge_l_buffer &&
        obs_end_l > -lane_right_width - stable_overlap_distance;
    *is_overlap_lane_obstacle = *is_right_obstacle || *is_left_obstacle;
  } else {
    *is_right_obstacle =
        obs_start_l > -lane_right_width - right_neighbor_lane_width -
                          kDecisionObstaclesLOverlapNeighborLaneDist &&
        obs_end_l < -lane_right_width + kDecisionObstaclesLOverlapDist;
    *is_left_obstacle =
        obs_end_l < lane_left_width + left_neighbor_lane_width +
                        kDecisionObstaclesLOverlapNeighborLaneDist &&
        obs_start_l > lane_left_width - kDecisionObstaclesLOverlapDist;
    *is_overlap_lane_obstacle =
        (obs_end_l > -lane_right_width - stable_overlap_distance &&
         obs_end_l < -lane_right_width + kDecisionObstaclesLOverlapDist) ||
        (obs_start_l < lane_left_width + stable_overlap_distance &&
         obs_start_l > lane_left_width - kDecisionObstaclesLOverlapDist);
  }

  if (!*is_right_obstacle && !*is_left_obstacle) {
    ADEBUG << "right_obstacle: " << *is_right_obstacle
           << ", left_obstacle: " << *is_left_obstacle;
  }

  ADEBUG << "obs_start_l: " << obs_start_l << ", obs_end_l: " << obs_end_l
         << ", lane_right_width: " << -lane_right_width
         << ", lane_left_width: " << lane_left_width
         << ", stable_overlap_distance: " << stable_overlap_distance
         << ", kDecisionObstaclesLOverlapDist: "
         << kDecisionObstaclesLOverlapDist;
  if (*is_overlap_lane_obstacle) {
    last_frame_overlap_lane_obs_ids_.emplace(obstacle.PerceptionId());
  }
  return true;
}

void ObstacleSelector::ClearCache(
    const ReferenceLineInfo& reference_line_info) {
  rule_base_decision_obs_ids_.clear();
  std::unordered_set<int32_t> disappear_obs;
  for (const auto& last_obs_id : last_frame_overlap_lane_obs_ids_) {
    bool is_real_disappear = true;
    for (const auto* obstacle :
         reference_line_info.path_decision().obstacles().Items()) {
      if (obstacle == nullptr) {
        continue;
      }
      if (obstacle->PerceptionId() == last_obs_id) {
        is_real_disappear = false;
        break;
      }
    }
    if (is_real_disappear) {
      disappear_obs.emplace(last_obs_id);
    }
  }
  for (const auto& obs_id : disappear_obs) {
    last_frame_overlap_lane_obs_ids_.erase(obs_id);
  }
  std::unordered_set<std::string> disappear_overlap_obs;
  for (const auto& overlap_info : overlap_lane_count_) {
    const auto& histoty_obstacle_id = overlap_info.first;
    const bool disappear = reference_line_info.path_decision().obstacles().Find(
                               histoty_obstacle_id) == nullptr;
    if (disappear) {
      disappear_overlap_obs.emplace(histoty_obstacle_id);
    }
  }
  for (const auto& id : disappear_overlap_obs) {
    overlap_lane_count_.erase(id);
  }
}

void ObstacleSelector::Debug(ReferenceLineInfo* const reference_line_info) {
  auto* avoid_alongside_debug = reference_line_info->mutable_debug()
                                    ->mutable_avoid_alongside_decider_info();
  for (const auto& debug_obs_id : rule_base_decision_obs_ids_) {
    const auto* obs =
        reference_line_info->path_decision()->obstacles().Find(debug_obs_id);
    if (obs == nullptr) {
      continue;
    }

    auto* decision_info = avoid_alongside_debug->mutable_decision_info()->Add();
    decision_info->set_id(debug_obs_id);
    auto* tags = decision_info->mutable_decision_tags();
    for (const auto& tag : obs->decider_tags()) {
      tags->Add()->assign(tag);
    }
  }
}

void ObstacleSelector::VirtualLaneAeraObsCaution(
    ReferenceLineInfo* const reference_line_info) {
  if (reference_line_info == nullptr) {
    return;
  }
  ObjectDecisionType object_decision;
  object_decision.mutable_caution();
  auto lane_points =
      reference_line_info->reference_line()
          .GetReferencePoint(reference_line_info->AdcSlBoundary().end_s())
          .lane_waypoints();
  if (lane_points.empty()) {
    return;
  }

  const auto& lane_point = lane_points.front();
  const bool is_in_virtual_aera =
      lane_point.lane != nullptr &&
      (lane_point.lane->lane().left_boundary().virtual_() ||
       lane_point.lane->lane().left_boundary().virtual_());
  if (!is_in_virtual_aera) {
    return;
  }
  const double adc_start_s = reference_line_info->AdcSlBoundary().start_s();
  const double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
  const double adc_start_l = reference_line_info->AdcSlBoundary().start_l();
  const double adc_end_l = reference_line_info->AdcSlBoundary().end_l();
  if (reference_line_info->path_decision() == nullptr) {
    return;
  }
  for (const auto& obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }
    const bool s_caution_range = obstacle->PerceptionSLBoundary().start_s() <
                                     adc_end_s + kVirtualAeraCautionFrontDist &&
                                 obstacle->PerceptionSLBoundary().end_s() >
                                     adc_start_s - kVirtualAeraCautionBackDist;
    const bool l_caution_range =
        obstacle->PerceptionSLBoundary().start_l() <
            adc_end_l + kVirtualAeraCautionLateralDist &&
        obstacle->PerceptionSLBoundary().end_l() >
            adc_start_l - kVirtualAeraCautionLateralDist;
    if (s_caution_range && l_caution_range) {
      ADEBUG << "###obs id:" << obstacle->Id()
             << " in virtual lane aera, caution!";
      reference_line_info->path_decision()->AddLateralDecision(
          "obstacles_decider/caution", obstacle->Id(), object_decision);
    }
  }
}

void ObstacleSelector::Process(
    const std::unordered_set<int32_t>& alongside_obstacles,
    const std::string& nearest_cutin_obs_id, const std::string& follow_obs_id,
    const std::unordered_set<int32_t>& cutin_obstacles,
    ReferenceLineInfo* const reference_line_info,
    std::unordered_set<std::string>* const decision_obstacles,
    std::unordered_set<std::string>* const dangerous_obstacles) {
  if (reference_line_info == nullptr || decision_obstacles == nullptr ||
      dangerous_obstacles == nullptr) {
    return;
  }
  ClearCache(*reference_line_info);
  VirtualLaneAeraObsCaution(reference_line_info);
  auto* path_decision = reference_line_info->path_decision();
  if (reference_line_info->vehicle_state().linear_velocity() < kAdcLowSpeed) {
    return;
  }

  const auto* const cut_in_obs =
      reference_line_info->path_decision()->obstacles().Find(
          nearest_cutin_obs_id);
  const auto* const follow_obs =
      reference_line_info->path_decision()->obstacles().Find(follow_obs_id);
  ADEBUG << "nearest_cutin_obs_id: {" << nearest_cutin_obs_id
         << "} , follow obs id: {" << follow_obs_id << "}";

  for (const auto& obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    // CIPV前的障碍物忽略
    if (obstacle == nullptr) {
      continue;
    }
    if (obstacle->IsLateralIgnore() || obstacle->IsLongitudinalIgnore()) {
      ADEBUG << "obs id: " << obstacle->Id() << " , ignore.";
      for (const auto& tag : obstacle->decider_tags()) {
        ADEBUG << "tag: " << tag;
      }
      continue;
    }
    if (!IsStableObstacle(*obstacle, kStableDangerousObstacleFrameSize)) {
      ADEBUG << "obs id: " << obstacle->Id() << " , !IsStableObstacle.";
      continue;
    }
    // FOLLOW前的障碍物忽略
    if (follow_obs != nullptr &&
        obstacle->PerceptionSLBoundary().start_s() >
            follow_obs->PerceptionSLBoundary().end_s()) {
      ADEBUG << "obs id: " << obstacle->Id() << " , before follow.";
      continue;
    }
    // CUTIN前的障碍物忽略
    if (cut_in_obs != nullptr &&
        obstacle->PerceptionSLBoundary().start_s() >
            cut_in_obs->PerceptionSLBoundary().start_s()) {
      ADEBUG << "obs id: " << obstacle->Id() << " , before cutin.";
      continue;
    }
    bool is_overlap_lane_obstacle = false;
    bool is_left_obstacle = false;
    bool is_right_obstacle = false;
    bool is_away_from_lane = false;
    const double ttc = GetTTC(*obstacle, *reference_line_info);
    if (IsRoadRoi(*obstacle, *reference_line_info, &is_overlap_lane_obstacle,
                  &is_left_obstacle, &is_right_obstacle, &is_away_from_lane)) {
      if (!obstacle->IsStatic()) {
        ObjectDecisionType object_decision;
        object_decision.mutable_caution();
        path_decision->AddLateralDecision("obstacles_decider/caution",
                                          obstacle->Id(), object_decision);
      }
      if (ttc < kDangerousObstacleIgnoreTTC && ttc > 0) {
        ADEBUG << "obs id: " << obstacle->Id() << ", ttc: " << ttc;
        dangerous_obstacles->emplace(obstacle->Id());
      }
    } else {
      ADEBUG << "obs id: " << obstacle->Id() << " , out roi.";
      continue;
    }
    const bool is_continuous_cone =
        IsContinuousCone(*obstacle, *reference_line_info);
    // 最新交付要求，连续锥桶不再校验稳定性
    if (!is_continuous_cone &&
        !IsStableObstacle(*obstacle, kStableDecisionObstacleFrameSize)) {
      ADEBUG << "obs id: " << obstacle->Id() << " , !IsStableObstacle.";
      continue;
    }
    if (!is_left_obstacle && !is_right_obstacle) {
      ADEBUG << "obs id: " << obstacle->Id()
             << " , !is_left_obstacle && !is_right_obstacle.";
      continue;
    }
    // 产品要求：连续压线5帧，认为压线
    // 按照最新的交付要求，连续压线判定改为3帧，连续锥桶不再进行连续压线的判定
    constexpr int kOverlapLaneCount = 3;
    if (is_continuous_cone && is_overlap_lane_obstacle) {
      overlap_lane_count_[obstacle->Id()] = kOverlapLaneCount;
    } else if (is_overlap_lane_obstacle) {
      overlap_lane_count_[obstacle->Id()] =
          std::min(kOverlapLaneCount, overlap_lane_count_[obstacle->Id()] + 1);
    } else {
      overlap_lane_count_.erase(obstacle->Id());
    }
    is_overlap_lane_obstacle =
        overlap_lane_count_[obstacle->Id()] >= kOverlapLaneCount;
    ADEBUG << "obs id: " << obstacle->Id()
           << ", overlap count: " << overlap_lane_count_[obstacle->Id()];
    // 产品要求：静态障碍物只处理前方压线的
    if (obstacle->IsStatic() && is_overlap_lane_obstacle &&
        obstacle->PerceptionSLBoundary().end_s() >
            reference_line_info->AdcSlBoundary().start_s() &&
        (obstacle->PerceptionSLBoundary().end_s() <
             reference_line_info->AdcSlBoundary().start_s() +
                 kStaticObstacleNudgeDist ||
         (ttc < kStaticObstacleNudgeTtc && ttc > 0.0))) {
      DoStaticObstacleDecision(*obstacle, is_left_obstacle, is_right_obstacle,
                               reference_line_info);
      rule_base_decision_obs_ids_.emplace(obstacle->Id());
      ADEBUG << "obs id: " << obstacle->Id() << " , is static.";
      continue;
    }
    constexpr double kCrowdSpeed = 6.5;
    if (reference_line_info->vehicle_state().linear_velocity() < kCrowdSpeed) {
      continue;
    }

    // ttc 不再范围之内的跳过
    const bool has_s_overlap =
        obstacle->PerceptionSLBoundary().start_s() <
            reference_line_info->AdcSlBoundary().end_s() + kBeforeObsDist &&
        obstacle->PerceptionSLBoundary().end_s() >
            reference_line_info->AdcSlBoundary().start_s();
    if ((ttc < 0.0 || ttc > kIgnoreTTC) && !has_s_overlap) {
      ADEBUG << "obs id: " << obstacle->Id() << ", ttc: {" << ttc << "}";
      continue;
    }
    // 后方及前方20m+随速外不进行决策
    constexpr double kNearDistance = 12.0;
    constexpr double kNearTime = 0.8;
    if (obstacle->PerceptionSLBoundary().start_s() -
            reference_line_info->AdcSlBoundary().end_s() >
        kNearDistance +
            kNearTime *
                reference_line_info->vehicle_state().linear_velocity()) {
      ADEBUG << "obs id: " << obstacle->Id() << ", not near!";
      continue;
    }
    const bool is_back_obs = obstacle->PerceptionSLBoundary().end_s() <
                             reference_line_info->AdcSlBoundary().start_s();
    // 产品要求：后方忽略ttc改为1.0s
    constexpr double kBackObsIgnoreTTC = 1.0;
    if (is_back_obs && (ttc > kBackObsIgnoreTTC || ttc < 0.0)) {
      ADEBUG << "obs id: " << obstacle->Id() << ", is back!"
             << " ttc: " << ttc;
      continue;
    }

    // 预测给出cutin的轨迹时不做决策
    const bool is_cyclist = obstacle->Perception().type() ==
                            ::TL::perception::PerceptionObstacle_Type::
                                PerceptionObstacle_Type_CYCLIST;
    const bool pred_cutin_obs =
        alongside_obstacles.find(obstacle->PerceptionId()) ==
            alongside_obstacles.end() ||
        cutin_obstacles.find(obstacle->PerceptionId()) != cutin_obstacles.end();
    if (pred_cutin_obs && !is_cyclist) {
      ADEBUG << "obs id: " << obstacle->Id() << " , prediction cut in."
             << ", sub type: " << obstacle->Perception().sub_type()
             << ", type:" << obstacle->Perception().type();
      continue;
    }
    if (obstacle->IsCrossReferenceLine()) {
      ADEBUG << "obs id: " << obstacle->Id() << " , prediction cross.";
      continue;
    }
    const double obs_start_s = obstacle->PerceptionSLBoundary().start_s();
    const double obs_end_s = obstacle->PerceptionSLBoundary().end_s();
    if (!obstacle->Trajectory().trajectory_point().empty()) {
      // 切入角度大的障碍物不做决策
      const double ref_heading = common::math::NormalizeAngle(
          reference_line_info->reference_line()
              .GetReferencePoint((obs_start_s + obs_end_s) * 0.5)
              .heading());
      const double obstacle_heading = common::math::NormalizeAngle(
          obstacle->Trajectory().trajectory_point(0).path_point().theta());
      const double heading_difference =
          common::math::NormalizeAngle(obstacle_heading - ref_heading);
      if (std::fabs(heading_difference) > kCutinAngle) {
        ADEBUG << "obs id: " << obstacle->Id()
               << ", heading_difference: " << heading_difference / M_PI * 180;
        continue;
      }
      // 侧向速度大的障碍物不做决策
      const double lateral_speed =
          obstacle->speed() * std::sin(heading_difference);
      if (std::fabs(lateral_speed) > kCutinLateralSpeed) {
        ADEBUG << "obs id: " << obstacle->Id()
               << ", lateral_speed: " << lateral_speed;
        continue;
      }
      // 切出车辆不做决策
      if ((lateral_speed > kCutoutLateralSpeed && is_left_obstacle) ||
          (lateral_speed < -kCutoutLateralSpeed && is_right_obstacle)) {
        ADEBUG << "obs id: " << obstacle->Id() << "cut out!"
               << ", lateral_speed: {" << lateral_speed << "}";
        continue;
      }
    }
    if (is_away_from_lane) {
      ADEBUG << "obs id: " << obstacle->Id() << ", is_away_from_lane";
      continue;
    }
    const bool is_decision_type = obstacle->IsOversizedVehicle();
    // 近距离压线车增加横向避让
    constexpr double kNearOverlapDist = 1.0;
    const bool is_near_dist =
        (reference_line_info->AdcSlBoundary().end_s() < obs_end_s) &&
        (obs_start_s - reference_line_info->AdcSlBoundary().end_s() <
         kNearOverlapDist);
    ADEBUG << "obs id: " << obstacle->Id() << ", is_near_dist: " << is_near_dist
           << ", is_overlap_lane_obstacle: " << is_overlap_lane_obstacle
           << ", obs_end_s: " << obs_end_s << ", obs_start_s: " << obs_start_s
           << ", adc_end_s: " << reference_line_info->AdcSlBoundary().end_s();
    if (is_decision_type && is_overlap_lane_obstacle && is_near_dist) {
      ObjectDecisionType object_decision;
      object_decision.mutable_caution();
      ObjectNudge* object_nudge_ptr = object_decision.mutable_nudge();
      object_nudge_ptr->set_distance_l(0.1);
      if (is_right_obstacle) {
        object_nudge_ptr->set_type(ObjectNudge::DYNAMIC_LEFT_NUDGE);
        path_decision->AddLateralDecision(
            "obstacles_decider/dynamic-left-nudge", obstacle->Id(),
            object_decision);
      } else if (is_left_obstacle) {
        object_nudge_ptr->set_type(ObjectNudge::DYNAMIC_RIGHT_NUDGE);
        path_decision->AddLateralDecision(
            "obstacles_decider/dynamic-right-nudge", obstacle->Id(),
            object_decision);
      }
      continue;
    }

    // 压线车及大车做决策
    if (!is_decision_type && !is_overlap_lane_obstacle) {
      ADEBUG << "obs id: " << obstacle->Id()
             << ", is_decision_type: " << is_decision_type
             << ", is_overlap_lane_obstacle: " << is_overlap_lane_obstacle;
      continue;
    }
    decision_obstacles->emplace(obstacle->Id());
  }
  Debug(reference_line_info);
}

};  // namespace TL::planning
