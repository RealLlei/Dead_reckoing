/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file obstacle_intention_processor.cc
 **/

#include "planning/tasks/deciders/st_bounds_decider/obstacle_filter.h"

#include <algorithm>
#include <limits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "common/file/log.h"
#include "common/math/vec2d.h"
#include "common/util/util.h"
#include "map/hdmap/path.h"
#include "planning/common/obstacle.h"
#include "planning/common/util/common.h"
#include "planning/localview/local_view.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/planning/decision.pb.h"

namespace TL {
namespace planning {

using common::math::NormalizeAngle;

void ObstacleFilter::Process(const Frame& frame,
                             ReferenceLineInfo* const reference_line_info) {
  if (reference_line_info == nullptr) {
    reverse_obs_map_.clear();
    return;
  }
  IgnoreObstaclesOnVirtualLane(frame, reference_line_info);
  IgnoreOutLanePedAndBicycle(frame, reference_line_info);
}

void ObstacleFilter::IgnoreObstaclesOnVirtualLane(
    const Frame& frame, ReferenceLineInfo* const reference_line_info) {
  if (reference_line_info == nullptr ||
      !frame.local_view().HasFunctionManagerIn() ||
      frame.local_view().GetFunctionManagerIn() == nullptr ||
      !frame.local_view().HasFunctionManagerOut() ||
      frame.local_view().GetFunctionManagerOut() == nullptr) {
    reverse_obs_map_.clear();
    return;
  }
  const auto& fct_in = frame.local_view().GetFunctionManagerIn();
  const auto& fct_out = frame.local_view().GetFunctionManagerOut();
  if (fct_in == nullptr || fct_out == nullptr ||
      fct_in->ta_pilot_mode() == functionmanager::AVP ||
      (fct_out->fsm_state() == functionmanager::PERCEPTION_TYPE &&
       fct_out->perception_sub_state() ==
           functionmanager::PerceptionSubState::CRUISE_TYPE)) {
    reverse_obs_map_.clear();
    return;
  }
  auto* const path_block_obs = reference_line_info->GetBlockingObstacle();
  const auto adc_front_s = reference_line_info->AdcSlBoundary().end_s();
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }
    const auto is_path_block_obs =
        path_block_obs != nullptr && path_block_obs->Id() == obstacle->Id();
    if (obstacle->IsVirtual() ||
        (obstacle->HasLongitudinalDecision() &&
         obstacle->LongitudinalDecision().has_ignore()) ||
        (obstacle->path_st_boundary().IsEmpty() && !is_path_block_obs) ||
        obstacle->PerceptionSLBoundary().start_s() < adc_front_s) {
      continue;
    }
    if (fct_in->ta_pilot_mode() == functionmanager::NCP ||
        fct_in->ta_pilot_mode() == functionmanager::NNP &&
            reference_line_info->reference_line().IsOnLane(
                obstacle->PerceptionSLBoundary())) {
      auto& obs_info = reverse_obs_map_.count(obstacle->PerceptionId()) > 0
                           ? reverse_obs_map_.at(obstacle->PerceptionId())
                           : reverse_obs_map_[(obstacle->PerceptionId())];
      obs_info.check_cnt++;
      obs_info.ignore = false;
      continue;
    }

    const auto obs_locate_ref_heading =
        reference_line_info->reference_line()
            .GetNearestReferencePoint(
                (obstacle->PerceptionSLBoundary().start_s() +
                 obstacle->PerceptionSLBoundary().end_s()) /
                2.0)
            .heading();
    const auto theta_diff = fabs(NormalizeAngle(obstacle->Perception().theta() -
                                                obs_locate_ref_heading));
    const auto reverse_obs = theta_diff > kReverseObsHeading;
    if (!reverse_obs && !obstacle->IsStatic()) {
      continue;
    }
    const auto& obs_locate_lane = reference_line_info->LocateLaneInfo(
        obstacle->PerceptionSLBoundary().end_s());
    if (obs_locate_lane == nullptr) {
      continue;
    }
    auto& obs_info = reverse_obs_map_.count(obstacle->PerceptionId()) > 0
                         ? reverse_obs_map_.at(obstacle->PerceptionId())
                         : reverse_obs_map_[(obstacle->PerceptionId())];
    if (obs_info.check_cnt > 0 && !obs_info.ignore) {
      // 选中了就不再忽略了
      continue;
    }
    const auto& lanes = reference_line_info->reference_line().GetLaneSegments(
        adc_front_s, fmin(obstacle->PerceptionSLBoundary().start_s(),
                          reference_line_info->reference_line().Length()));
    // 都在虚拟线内，保守点
    const auto has_normal_lane_between_adc_and_obs = std::any_of(
        lanes.cbegin(), lanes.cend(), [&](const hdmap::LaneSegment& lane) {
          return (!lane.lane->lane().left_boundary().virtual_() ||
                  !lane.lane->lane().right_boundary().virtual_());
        });

    const auto obs_on_virtual_lane =
        obs_locate_lane->lane().left_boundary().virtual_() &&
        obs_locate_lane->lane().right_boundary().virtual_();
    auto ignore = false;
    obs_info.check_cnt++;
    obs_info.dis = obstacle->PerceptionSLBoundary().start_s() - adc_front_s;
    obs_info.cal_dec =
        -pow(reference_line_info->vehicle_state().linear_velocity(), 2) /
        (fmax(obs_info.dis, 0.01));
    obs_info.on_virtual_lane = obs_on_virtual_lane;
    obs_info.theta_diff = theta_diff;
    const auto& perception_type = obstacle->Perception().type();
    const auto need_caution =
        !has_normal_lane_between_adc_and_obs &&
        (perception_type == perception::PerceptionObstacle::PEDESTRIAN ||
         perception_type == perception::PerceptionObstacle::CYCLIST ||
         perception_type == perception::PerceptionObstacle::BICYCLE);
    const auto dis = obstacle->PerceptionSLBoundary().start_s() - adc_front_s;
    if (!reverse_obs) {
      ignore = obstacle->IsStatic() && has_normal_lane_between_adc_and_obs
                   ? obs_on_virtual_lane
                         ? (obs_info.cal_dec > kVirtualLaneIgnoreDec &&
                            obs_info.check_cnt < kVirtualLaneCheckCnt)
                         : false
                   : false;
    } else {
      if (obstacle->IsStatic() || need_caution) {
        ignore = obs_on_virtual_lane
                     ? (obs_info.cal_dec > kVirtualLaneIgnoreDec &&
                        obs_info.check_cnt < kVirtualLaneCheckCnt)
                     : (obs_info.check_cnt < kNormalLaneCheckCnt &&
                        obs_info.cal_dec > kNormalLaneIgnoreDec);
      } else {
        ignore = obs_on_virtual_lane
                     ? (obs_info.cal_dec > kVirtualLaneIgnoreDec ||
                        obs_info.check_cnt < kVirtualLaneCheckCnt)
                     : (obs_info.check_cnt < kNormalLaneCheckCnt &&
                        obs_info.cal_dec > kNormalLaneIgnoreDec);
      }
    }
    if (ignore) {
      ignore = reverse_obs ? dis > kMaxReverseConsiderDistance
                           : dis > kMaxNormalConsiderDistance;
    }
    obs_info.ignore = ignore;
    ADEBUG << " ID : " << obstacle->PerceptionId()
           << " obs_info check_cnt : " << obs_info.check_cnt
           << "obs_info.cal_dec : " << obs_info.cal_dec
           << " theta_diff : " << theta_diff
           << "obs_on_virtual_lane : " << obs_on_virtual_lane
           << " ignore : " << ignore;

    if (!ignore) {
      continue;
    }
    auto* obs_ptr = reference_line_info->path_decision()->Find(obstacle->Id());
    if (obs_ptr == nullptr) {
      continue;
    }
    ObjectDecisionType ignore_decision;
    ignore_decision.mutable_ignore();
    obs_ptr->SetLongitudinalDecision("st_obs_fillter_reverse", ignore_decision);
  }
  for (auto it = reverse_obs_map_.begin(); it != reverse_obs_map_.end();) {
    if (reference_line_info->path_decision()->FindPerceptionObstacle(
            it->first) == nullptr) {
      it = reverse_obs_map_.erase(it);
    } else {
      it++;
    }
  }
}

void ObstacleFilter::IgnoreOutLanePedAndBicycle(
    const Frame& frame, ReferenceLineInfo* reference_line_info) {
  if (reference_line_info == nullptr ||
      !frame.local_view().HasFunctionManagerIn() ||
      frame.local_view().GetFunctionManagerIn() == nullptr ||
      !frame.local_view().HasFunctionManagerOut() ||
      frame.local_view().GetFunctionManagerOut() == nullptr) {
    return;
  }
  const auto& fct_in = frame.local_view().GetFunctionManagerIn();
  const auto& fct_out = frame.local_view().GetFunctionManagerOut();
  if (fct_in == nullptr || fct_out == nullptr ||
      fct_in->ta_pilot_mode() == functionmanager::AVP ||
      (fct_out->fsm_state() == functionmanager::PERCEPTION_TYPE &&
       fct_out->perception_sub_state() ==
           functionmanager::PerceptionSubState::CRUISE_TYPE)) {
    return;
  }
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }
    if (obstacle->IsVirtual() ||
        (obstacle->HasLongitudinalDecision() &&
         obstacle->LongitudinalDecision().has_ignore()) ||
        (obstacle->Perception().type() !=
             perception::PerceptionObstacle::PEDESTRIAN &&
         obstacle->Perception().type() !=
             perception::PerceptionObstacle::CYCLIST &&
         obstacle->Perception().type() !=
             perception::PerceptionObstacle::BICYCLE) ||
        obstacle->path_st_boundary().IsEmpty()) {
      continue;
    }
    const auto is_on_lane = reference_line_info->reference_line().IsOnLane(
        obstacle->PerceptionSLBoundary());
    if (is_on_lane) {
      continue;
    }
    if (!reference_line_info->IsChangeLanePath()) {
      // 不换道，忽略当前车道的
      auto* obs_ptr =
          reference_line_info->path_decision()->Find(obstacle->Id());
      if (obs_ptr == nullptr) {
        continue;
      }
      ObjectDecisionType ignore_decision;
      ignore_decision.mutable_ignore();
      obs_ptr->SetLongitudinalDecision("st_obs_fillter_not_on_lane",
                                       ignore_decision);
      continue;
    }
    // 换道，忽略自车车道和目标车道的
    auto ignore = false;
    const auto adc_middle_s = (reference_line_info->AdcSlBoundary().start_s() +
                               reference_line_info->AdcSlBoundary().end_s()) /
                              2;
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    reference_line_info->reference_line().map_path().GetLaneWidth(
        adc_middle_s, &lane_left_width, &lane_right_width);
    const auto adc_left_lane_change =
        reference_line_info->AdcSlBoundary().end_l() > lane_left_width;
    const auto adc_right_lane_change =
        -lane_right_width > reference_line_info->AdcSlBoundary().start_l();

    const auto obs_middles = (obstacle->PerceptionSLBoundary().start_s() +
                              obstacle->PerceptionSLBoundary().end_s()) /
                             2;
    if (!adc_right_lane_change && !adc_left_lane_change) {
      continue;
    }
    const auto lane_type = adc_right_lane_change
                               ? ReferenceLineInfo::LaneType::LeftForward
                               : ReferenceLineInfo::LaneType::RightForward;
    const auto& neighbor_lane =
        reference_line_info->GetNeighborLaneInfo(lane_type, obs_middles);
    ignore =
        neighbor_lane == nullptr ||
        !neighbor_lane->IsOnLane(Vec2d{obstacle->Perception().position().x(),
                                       obstacle->Perception().position().y()});
    if (ignore) {
      auto* obs_ptr =
          reference_line_info->path_decision()->Find(obstacle->Id());
      if (obs_ptr == nullptr) {
        continue;
      }
      ObjectDecisionType ignore_decision;
      ignore_decision.mutable_ignore();
      obs_ptr->SetLongitudinalDecision("st_obs_fillter_not_on_lane",
                                       ignore_decision);
      continue;
    }
  }
}

}  // namespace planning
}  // namespace TL
