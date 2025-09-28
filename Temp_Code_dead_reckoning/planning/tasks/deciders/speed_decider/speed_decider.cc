/******************************************************************************
 * Copyright 2017 The TL Authors. All Rights Reserved.
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

#include "planning/tasks/deciders/speed_decider/speed_decider.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "absl/strings/match.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"
#include "common/time/clock.h"
#include "planning/common/frame.h"
#include "planning/common/obstacle.h"
#include "planning/common/path_decision.h"
#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/utils/st_gap_estimator.h"
#include "planning/proto/reference_line_smoother_config.pb.h"
#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/planning/decision.pb.h"

namespace TL {
namespace planning {

using TL::common::Clock;
using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::VehicleConfigHelper;
using TL::common::math::Vec2d;
using TL::perception::PerceptionObstacle;

SpeedDecider::SpeedDecider(const TaskConfig& config,
                           const std::shared_ptr<DependencyInjector>& injector)
    : Task(config, injector) {
  if (config_.has_speed_decider_config()) {
    speed_decider_config_.CopyFrom(config_.speed_decider_config());
  }

  for (int i = 0; i < static_cast<int>(follow_distance_limit_table_.size());
       ++i) {
    follow_distance_limit_table_.at(i) =
        kFollowDistanceMin + kFollowDistanceIndex * i;
  }
  for (int i = 0; i < static_cast<int>(overtake_distance_limit_table_.size());
       ++i) {
    overtake_distance_limit_table_.at(i) =
        kOvertakeDistanceMax - kOvertakeDistanceIndex * i;
  }
  vehicle_param_ = VehicleConfigHelper::GetConfig().vehicle_param();
}

common::Status SpeedDecider::Execute(Frame* frame,
                                     ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  init_point_ = frame_->PlanningStartPoint();
  if (!reference_line_info->path_data().frenet_frame_path().is_forward_path()) {
    init_point_.set_v(std::fabs(init_point_.v()));
    init_point_.set_a(-init_point_.a());
  }
  adc_sl_boundary_ = reference_line_info_->AdcSlBoundary();
  reference_line_ = &reference_line_info_->reference_line();
  is_forward_path_ =
      reference_line_info->path_data().frenet_frame_path().is_forward_path();

  if (!MakeObjectDecision(reference_line_info->speed_data(),
                          reference_line_info->path_decision())
           .ok()) {
    const std::string msg = "Get object decision by speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_SPEEDDECIDER_ERROR, msg);
  }
  if (reference_line_info != nullptr &&
      reference_line_info->speed_data().GetIsFallback()) {
    frame->SetIsStopFallback(true);
  }
  switch (speed_decider_config_.is_fall_back_type()) {
    case SpeedDeciderConfig::IS_FALL_BACK_ST:
      frame->SetIsSpeedFallback(IsFallbackWithST(reference_line_info_));
      break;
    case SpeedDeciderConfig::IS_FALL_BACK_TTC:
      frame->SetIsSpeedFallback(IsFallbackWithTTC(reference_line_info_, frame));
      break;
    default:
      AERROR << "IS_FALL_BACK type not defined";
      break;
  }

  return Status::OK();
}

SpeedDecider::STLocation SpeedDecider::GetSTLocation(
    const PathDecision* const path_decision, const SpeedData& speed_profile,
    const STBoundary& st_boundary) {
  if (st_boundary.IsEmpty()) {
    return BELOW;
  }

  STLocation st_location = BELOW;
  bool st_position_set = false;
  const double start_t = st_boundary.min_t();
  const double end_t = st_boundary.max_t();
  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    const STPoint curr_st(speed_profile[i].s(), speed_profile[i].t());
    const STPoint next_st(speed_profile[i + 1].s(), speed_profile[i + 1].t());
    if (curr_st.t() < start_t && next_st.t() < start_t) {
      continue;
    }
    if (curr_st.t() > end_t) {
      break;
    }

    if (!FLAGS_use_st_drivable_boundary) {
      common::math::LineSegment2d speed_line(curr_st, next_st);
      if (st_boundary.HasOverlap(speed_line)) {
        ADEBUG << "speed profile cross st_boundaries.";
        st_location = CROSS;

        if (!FLAGS_use_st_drivable_boundary) {
          if (st_boundary.boundary_type() ==
              STBoundary::BoundaryType::KEEP_CLEAR) {
            if (!CheckKeepClearCrossable(path_decision, speed_profile,
                                         st_boundary)) {
              st_location = BELOW;
            }
          }
        }
        break;
      }
    }

    // note: st_position can be calculated by checking two st points once
    //       but we need iterate all st points to make sure there is no CROSS
    if (!st_position_set) {
      if (start_t < next_st.t() && curr_st.t() < end_t) {
        STPoint bd_point_front = st_boundary.upper_points().front();
        double side = common::math::CrossProd(bd_point_front, curr_st, next_st);
        st_location = side < 0.0 ? ABOVE : BELOW;
        st_position_set = true;
      }
    }
  }
  return st_location;
}

bool SpeedDecider::CheckKeepClearCrossable(
    const PathDecision* const path_decision, const SpeedData& speed_profile,
    const STBoundary& keep_clear_st_boundary) {
  UNUSED(path_decision);
  bool keep_clear_crossable = true;

  const auto& last_speed_point = speed_profile.back();
  double last_speed_point_v = 0.0;
  if (last_speed_point.has_v()) {
    last_speed_point_v = last_speed_point.v();
  } else {
    const size_t len = speed_profile.size();
    if (len > 1) {
      const auto& last_2nd_speed_point = speed_profile[len - 2];
      last_speed_point_v = (last_speed_point.s() - last_2nd_speed_point.s()) /
                           (last_speed_point.t() - last_2nd_speed_point.t());
    }
  }
  static constexpr double kKeepClearSlowSpeed = 2.5;  // m/s
  ADEBUG << "last_speed_point_s[" << last_speed_point.s()
         << "] st_boundary.max_s[" << keep_clear_st_boundary.max_s()
         << "] last_speed_point_v[" << last_speed_point_v << "]";
  if (last_speed_point.s() <= keep_clear_st_boundary.max_s() &&
      last_speed_point_v < kKeepClearSlowSpeed) {
    keep_clear_crossable = false;
  }
  return keep_clear_crossable;
}

bool SpeedDecider::CheckKeepClearBlocked(
    const PathDecision* const path_decision,
    const Obstacle& keep_clear_obstacle) {
  bool keep_clear_blocked = false;

  // check if overlap with other stop wall
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    if (obstacle->Id() == keep_clear_obstacle.Id()) {
      continue;
    }
    const double obstacle_start_s = obstacle->PerceptionSLBoundary().start_s();
    const double adc_length =
        VehicleConfigHelper::GetConfig().vehicle_param().length();
    const double distance =
        obstacle_start_s - keep_clear_obstacle.PerceptionSLBoundary().end_s();

    if (obstacle->IsBlockingObstacle() && distance > 0 &&
        distance < (adc_length / 2)) {
      keep_clear_blocked = true;
      break;
    }
  }
  return keep_clear_blocked;
}

bool SpeedDecider::IsFollowTooClose(const Obstacle& obstacle) const {
  if (!obstacle.IsBlockingObstacle()) {
    return false;
  }

  if (obstacle.path_st_boundary().min_t() > 0.0) {
    return false;
  }
  const double obs_speed = obstacle.speed();
  const double ego_speed = init_point_.v();
  if (obs_speed > ego_speed) {
    return false;
  }
  const double distance =
      obstacle.path_st_boundary().min_s() - FLAGS_min_stop_distance_obstacle;
  TL::common::VehicleParam vehicle_param;
  vehicle_param.CopyFrom(
      TL::common::VehicleConfigHelper::GetConfig().vehicle_param());
  const double lane_follow_max_decel = fabs(vehicle_param.max_deceleration());
  const double lane_change_max_decel = fabs(vehicle_param.max_deceleration());
  auto* planning_status = injector_->planning_context()
                              ->mutable_planning_status()
                              ->mutable_change_lane();
  double distance_numerator = std::pow((ego_speed - obs_speed), 2) * 0.5;
  double distance_denominator = lane_follow_max_decel;
  if (planning_status->has_status() &&
      planning_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
    distance_denominator = lane_change_max_decel;
  }
  return distance < distance_numerator / distance_denominator;
}

Status SpeedDecider::MakeObjectDecision(
    const SpeedData& speed_profile, PathDecision* const path_decision) const {
  if (speed_profile.size() < 2) {
    const std::string msg = "dp_st_graph failed to get speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_SPEEDDECIDER_ERROR, msg);
  }

  for (const auto* obstacle : path_decision->obstacles().Items()) {
    auto* mutable_obstacle = path_decision->Find(obstacle->Id());
    const auto& boundary = mutable_obstacle->path_st_boundary();

    if (boundary.IsEmpty() || boundary.max_s() < 0.0 ||
        boundary.max_t() < 0.0 ||
        boundary.min_t() >= speed_profile.back().t()) {
      AppendIgnoreDecision(mutable_obstacle);
      continue;
    }
    if (obstacle->HasLongitudinalDecision()) {
      AppendIgnoreDecision(mutable_obstacle);
      continue;
    }

    // for Virtual obstacle, skip if center point NOT "on lane"
    if (obstacle->IsVirtual()) {
      const auto& obstacle_box = obstacle->PerceptionBoundingBox();
      if (!reference_line_->IsOnLane(obstacle_box.center())) {
        continue;
      }
    }

    // always STOP for pedestrian
    if (CheckStopForPedestrian(*mutable_obstacle)) {
      ObjectDecisionType stop_decision;
      if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                             -FLAGS_min_stop_distance_obstacle)) {
        mutable_obstacle->AddLongitudinalDecision("dp_st_graph/pedestrian",
                                                  stop_decision);
      }
      continue;
    }

    auto location = GetSTLocation(path_decision, speed_profile, boundary);

    if (!FLAGS_use_st_drivable_boundary) {
      if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
        if (CheckKeepClearBlocked(path_decision, *obstacle)) {
          location = BELOW;
        }
      }
    }

    switch (location) {
      case BELOW:
        if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*mutable_obstacle, &stop_decision, 0.0)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/keep_clear",
                                                      stop_decision);
          }
        } else if (CheckIsFollow(*obstacle, boundary)) {
          // stop for low_speed decelerating
          // if (IsFollowTooClose(*mutable_obstacle)) {
          //   ObjectDecisionType stop_decision;
          //   if (CreateStopDecision(*mutable_obstacle, &stop_decision,
          //                          -FLAGS_min_stop_distance_obstacle)) {
          //     mutable_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
          //                                               stop_decision);
          //   }
          // } else {  // high speed or low speed accelerating
          // FOLLOW decision
          ObjectDecisionType follow_decision;
          if (CreateFollowDecision(*mutable_obstacle, &follow_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                      follow_decision);
          }
          // }
        } else {
          // YIELD decision
          ObjectDecisionType yield_decision;
          if (CreateYieldDecision(*mutable_obstacle, &yield_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                      yield_decision);
          }
        }
        break;
      case ABOVE:
        if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
          ObjectDecisionType ignore;
          ignore.mutable_ignore();
          mutable_obstacle->AddLongitudinalDecision("dp_st_graph", ignore);
        } else {
          // OVERTAKE decision
          ObjectDecisionType overtake_decision;
          if (CreateOvertakeDecision(*mutable_obstacle, &overtake_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/overtake",
                                                      overtake_decision);
          }
        }
        break;
      case CROSS:
        if (mutable_obstacle->IsBlockingObstacle()) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                 -FLAGS_min_stop_distance_obstacle)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/cross",
                                                      stop_decision);
          }
          const std::string msg =
              absl::StrCat("Failed to find a solution for crossing obstacle: ",
                           mutable_obstacle->Id());
          AERROR << msg;
          return Status(ErrorCode::PLANNER_CRUISING_SPEEDDECIDER_ERROR, msg);
        }
        break;
      default:
        AERROR << "Unknown position:" << location;
    }
    AppendIgnoreDecision(mutable_obstacle);
  }

  return Status::OK();
}

void SpeedDecider::AppendIgnoreDecision(Obstacle* obstacle) {
  ObjectDecisionType ignore_decision;
  ignore_decision.mutable_ignore();
  if (!obstacle->HasLongitudinalDecision()) {
    obstacle->AddLongitudinalDecision("dp_st_graph", ignore_decision);
  }
  if (!obstacle->HasLateralDecision()) {
    obstacle->AddLateralDecision("dp_st_graph", ignore_decision);
  }
}

bool SpeedDecider::CreateStopDecision(const Obstacle& obstacle,
                                      ObjectDecisionType* const stop_decision,
                                      double stop_distance) const {
  const auto& boundary = obstacle.path_st_boundary();

  // TODO(all): this is a bug! Cannot mix reference s and path s!
  // Replace boundary.min_s() with computed reference line s
  // fence is set according to reference line s.
  double fence_s =
      is_forward_path_
          ? adc_sl_boundary_.end_s() + boundary.min_s() + stop_distance
          : adc_sl_boundary_.start_s() - boundary.min_s() - stop_distance;
  if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
    fence_s = is_forward_path_ ? obstacle.PerceptionSLBoundary().start_s()
                               : obstacle.PerceptionSLBoundary().end_s();
  }
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if ((is_forward_path_ && main_stop_s < fence_s) ||
      (!is_forward_path_ && main_stop_s > fence_s)) {
    ADEBUG << "Stop fence is further away, ignore.";
    return false;
  }

  const auto fence_point = reference_line_->GetReferencePoint(fence_s);

  // set STOP decision
  auto* stop = stop_decision->mutable_stop();
  stop->set_distance_s(stop_distance);
  auto* stop_point = stop->mutable_stop_point();
  stop_point->set_x(fence_point.x());
  stop_point->set_y(fence_point.y());
  stop_point->set_z(0.0);
  stop->set_stop_heading(fence_point.heading());

  if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
    stop->set_reason_code(StopReasonCode::STOP_REASON_CLEAR_ZONE);
  }

  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
  ADEBUG << "STOP: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

bool SpeedDecider::CreateFollowDecision(
    const Obstacle& obstacle, ObjectDecisionType* const follow_decision) const {
  const double follow_speed = init_point_.v();
  const double follow_distance_s =
      -StGapEstimator::EstimateProperFollowingGap(follow_speed);

  const auto& boundary = obstacle.path_st_boundary();
  const double reference_s =
      is_forward_path_
          ? adc_sl_boundary_.end_s() + boundary.min_s() + follow_distance_s
          : adc_sl_boundary_.start_s() - boundary.min_s() - follow_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if ((is_forward_path_ && main_stop_s < reference_s) ||
      (!is_forward_path_ && main_stop_s > reference_s)) {
    ADEBUG << "Follow reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_s);

  // set FOLLOW decision
  auto* follow = follow_decision->mutable_follow();
  follow->set_distance_s(follow_distance_s);
  auto* fence_point = follow->mutable_fence_point();
  fence_point->set_x(ref_point.x());
  fence_point->set_y(ref_point.y());
  fence_point->set_z(0.0);
  follow->set_fence_heading(ref_point.heading());

  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();

  ADEBUG << "FOLLOW: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

bool SpeedDecider::CreateYieldDecision(
    const Obstacle& obstacle, ObjectDecisionType* const yield_decision) const {
  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
  double yield_distance = StGapEstimator::EstimateProperYieldingGap();

  const auto& obstacle_boundary = obstacle.path_st_boundary();
  const double yield_distance_s =
      std::max(-obstacle_boundary.min_s(), -yield_distance);

  const double reference_line_fence_s =
      is_forward_path_ ? adc_sl_boundary_.end_s() + obstacle_boundary.min_s() +
                             yield_distance_s
                       : adc_sl_boundary_.start_s() -
                             obstacle_boundary.min_s() - yield_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if ((is_forward_path_ && main_stop_s < reference_line_fence_s) ||
      (!is_forward_path_ && main_stop_s > reference_line_fence_s)) {
    ADEBUG << "Yield reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  // set YIELD decision
  auto* yield = yield_decision->mutable_yield();
  yield->set_distance_s(yield_distance_s);
  yield->mutable_fence_point()->set_x(ref_point.x());
  yield->mutable_fence_point()->set_y(ref_point.y());
  yield->mutable_fence_point()->set_z(0.0);
  yield->set_fence_heading(ref_point.heading());

  ADEBUG << "YIELD: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

bool SpeedDecider::CreateOvertakeDecision(
    const Obstacle& obstacle,
    ObjectDecisionType* const overtake_decision) const {
  const auto& velocity = obstacle.Perception().velocity();
  const double obstacle_speed =
      common::math::Vec2d::CreateUnitVec2d(init_point_.path_point().theta())
          .InnerProd(Vec2d(velocity.x(), velocity.y()));

  const double overtake_distance_s =
      StGapEstimator::EstimateProperOvertakingGap(obstacle_speed,
                                                  init_point_.v());

  const auto& boundary = obstacle.path_st_boundary();
  const double reference_line_fence_s =
      is_forward_path_
          ? adc_sl_boundary_.end_s() + boundary.min_s() + overtake_distance_s
          : adc_sl_boundary_.start_s() - boundary.min_s() - overtake_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if ((is_forward_path_ && main_stop_s < reference_line_fence_s) ||
      (!is_forward_path_ && main_stop_s > reference_line_fence_s)) {
    ADEBUG << "Overtake reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  // set OVERTAKE decision
  auto* overtake = overtake_decision->mutable_overtake();
  overtake->set_distance_s(overtake_distance_s);
  overtake->mutable_fence_point()->set_x(ref_point.x());
  overtake->mutable_fence_point()->set_y(ref_point.y());
  overtake->mutable_fence_point()->set_z(0.0);
  overtake->set_fence_heading(ref_point.heading());

  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
  ADEBUG << "OVERTAKE: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

bool SpeedDecider::CheckIsFollow(const Obstacle& obstacle,
                                 const STBoundary& boundary) {
  UNUSED(obstacle);
  // const double obstacle_l_distance =
  //     std::min(std::fabs(obstacle.PerceptionSLBoundary().start_l()),
  //              std::fabs(obstacle.PerceptionSLBoundary().end_l()));
  // if (obstacle_l_distance > FLAGS_follow_min_obs_lateral_distance) {
  //   return false;
  // }
  if (obstacle.IsStatic()) {
    return false;
  }
  // move towards adc and unmove obs
  if (boundary.bottom_left_point().s() >= boundary.bottom_right_point().s()) {
    return false;
  }

  static constexpr double kFollowTimeEpsilon = 1e-3;
  if (boundary.max_t() < kFollowTimeEpsilon) {
    return false;
  }

  // cross lane but be moving to different direction
  if (boundary.max_t() - boundary.min_t() < FLAGS_follow_min_time_sec) {
    return false;
  }

  return true;
}

bool SpeedDecider::CheckStopForPedestrian(const Obstacle& obstacle) const {
  const auto& perception_obstacle = obstacle.Perception();
  if (perception_obstacle.type() != PerceptionObstacle::PEDESTRIAN) {
    return false;
  }

  const bool is_forward_path =
      reference_line_info_->path_data().frenet_frame_path().is_forward_path();
  const auto& obstacle_sl_boundary = obstacle.PerceptionSLBoundary();
  if ((is_forward_path &&
       obstacle_sl_boundary.end_s() < adc_sl_boundary_.start_s()) ||
      (!is_forward_path &&
       obstacle_sl_boundary.start_s() > adc_sl_boundary_.end_s())) {
    return false;
  }

  // read pedestrian stop time from PlanningContext
  auto* mutable_speed_decider_status = injector_->planning_context()
                                           ->mutable_planning_status()
                                           ->mutable_speed_decider();
  std::unordered_map<std::string, double> stop_time_map;
  for (const auto& pedestrian_stop_time :
       mutable_speed_decider_status->pedestrian_stop_time()) {
    stop_time_map[pedestrian_stop_time.obstacle_id()] =
        pedestrian_stop_time.stop_timestamp_sec();
  }

  const std::string& obstacle_id = obstacle.Id();

  // update stop timestamp on static pedestrian for watch timer
  // check on stop timer for static pedestrians
  static constexpr double kSDistanceStartTimer = 10.0;
  static constexpr double kMaxStopSpeed = 0.3;
  static constexpr double kPedestrianStopTimeout = 4.0;

  bool result = true;
  if (obstacle.path_st_boundary().min_s() < kSDistanceStartTimer) {
    const auto obstacle_speed = std::hypot(perception_obstacle.velocity().x(),
                                           perception_obstacle.velocity().y());
    if (obstacle_speed > kMaxStopSpeed) {
      stop_time_map.erase(obstacle_id);
    } else {
      if (stop_time_map.count(obstacle_id) == 0) {
        // add timestamp
        stop_time_map[obstacle_id] = Clock::NowInSeconds();
        ADEBUG << "add timestamp: obstacle_id[" << obstacle_id << "] timestamp["
               << Clock::NowInSeconds() << "]";
      } else {
        // check timeout
        double stop_timer = Clock::NowInSeconds() - stop_time_map[obstacle_id];
        ADEBUG << "stop_timer: obstacle_id[" << obstacle_id << "] stop_timer["
               << stop_timer << "]";
        if (stop_timer >= kPedestrianStopTimeout) {
          result = false;
        }
      }
    }
  }

  // write pedestrian stop time to PlanningContext
  mutable_speed_decider_status->mutable_pedestrian_stop_time()->Clear();
  for (const auto& stop_time : stop_time_map) {
    auto* pedestrian_stop_time =
        mutable_speed_decider_status->add_pedestrian_stop_time();
    pedestrian_stop_time->set_obstacle_id(stop_time.first);
    pedestrian_stop_time->set_stop_timestamp_sec(stop_time.second);
  }
  return result;
}

bool SpeedDecider::IsFallbackWithTTC(
    const ReferenceLineInfo* reference_line_info, Frame* frame) const {
  if (reference_line_info == nullptr ||
      reference_line_info->speed_data().empty() || frame == nullptr) {
    return false;
  }
  if (IfCurvatureSpeedDangerous(reference_line_info, frame)) {
    return true;
  }
  const auto& path_decision = reference_line_info->path_decision();

  const auto& obstacles = path_decision.obstacles().Items();
  const auto& intention_obstacles =
      reference_line_info->speed_data().GetSpeedDeciderIntentionID();
  frame->SetIsCIPVSpeedFallback(false);
  if (std::any_of(obstacles.begin(), obstacles.end(),
                  [&](const auto* obstacle) {
                    return obstacle != nullptr &&
                           (intention_obstacles.find(obstacle->Id()) !=
                                intention_obstacles.end() ||
                            obstacle->IsStatic()) &&
                           absl::StrContains(frame->GetObstacleInfo().GetCIPV(),
                                             obstacle->Id()) &&
                           CalculateTTC(*obstacle, reference_line_info, frame);
                  })) {
    frame->SetIsCIPVSpeedFallback(true);
    return true;
  }
  return std::any_of(
      obstacles.begin(), obstacles.end(), [&](const auto* obstacle) {
        return obstacle != nullptr &&
               !absl::StrContains(frame->GetObstacleInfo().GetCIPV(),
                                  obstacle->Id()) &&
               (intention_obstacles.find(obstacle->Id()) !=
                    intention_obstacles.end() ||
                obstacle->IsStatic()) &&
               CalculateTTC(*obstacle, reference_line_info, frame);
      });
}

bool SpeedDecider::CalculateTTC(const Obstacle& obstacle,
                                const ReferenceLineInfo* reference_line_info,
                                const Frame* frame) const {
  if (frame == nullptr) {
    return false;
  }
  return obstacle.IsStatic()
             ? (frame->LonStopObsId() == obstacle.Id() &&
                CalculateTTCWithStaticObstacle(obstacle, reference_line_info))
             : CalculateTTCWithDynamicObstacle(obstacle, reference_line_info,
                                               frame);
}

bool SpeedDecider::IfCurvatureSpeedDangerous(
    const ReferenceLineInfo* reference_line_info, const Frame* frame) const {
  if (reference_line_info == nullptr || frame == nullptr ||
      frame->vehicle_state().driving_mode() !=
          soc::Chassis::COMPLETE_AUTO_DRIVE ||
      reference_line_info->speed_data().size() <= kDisSatisFactionPoint) {
    return false;
  }
  size_t dissatisfaction_point = 0;
  const auto& speed_data = reference_line_info->speed_data();
  const auto& path_data = reference_line_info->path_data();
  for (int i = 1; i < reference_line_info->speed_data().size(); i++) {
    if (speed_data.empty() || path_data.Empty()) {
      break;
    }
    const auto& cur_speed_point = speed_data.at(i);
    const auto& pre_speed_point = speed_data.at(i - 1);
    const auto interval_time = cur_speed_point.t() - pre_speed_point.t();
    const auto& cur_path_point =
        path_data.GetPathPointWithPathS(cur_speed_point.s());
    const auto& pre_path_point =
        path_data.GetPathPointWithPathS(pre_speed_point.s());

    // 1.Calculate the current steering wheel angle
    const auto& max_steering_wheel_speed =
        common::math::InterpolationOne(
            cur_speed_point.v(),
            speed_decider_config_.steer_wheel_speed_segment().vehicle_speed(),
            speed_decider_config_.steer_wheel_speed_segment()
                .steering_wheel_speed_limit_segment()) /
        180 * M_PI * 1.5;
    const auto& max_steering_wheel_angle =
        common::math::InterpolationOne(
            cur_speed_point.v(),
            speed_decider_config_.steer_wheel_speed_segment().vehicle_speed(),
            speed_decider_config_.steer_wheel_speed_segment()
                .steering_wheel_angle_limit()) /
        180 * M_PI;

    const auto cur_steering_wheel_angle =
        CalculateCurSteeringWheelAngle(cur_path_point.kappa());
    const auto pre_steering_wheel_angle =
        CalculateCurSteeringWheelAngle(pre_path_point.kappa());

    // 2.Compared to path horizontal position
    const auto pre_max_steer_wheel_angle =
        pre_steering_wheel_angle + interval_time * max_steering_wheel_speed;
    const auto pre_min_steer_wheel_angle =
        pre_steering_wheel_angle - interval_time * max_steering_wheel_speed;

    if (cur_steering_wheel_angle > pre_max_steer_wheel_angle ||
        cur_steering_wheel_angle < pre_min_steer_wheel_angle ||
        max_steering_wheel_angle < cur_steering_wheel_angle ||
        -max_steering_wheel_angle > cur_steering_wheel_angle) {
      dissatisfaction_point++;
    }
    if (dissatisfaction_point > kDisSatisFactionPoint) {
      return true;
    }
  }
  return false;
}

double SpeedDecider::CalculateCurSteeringWheelAngle(const double kappa) const {
  const double wheel_angle = atan(vehicle_param_.wheel_base() * kappa);
  double steer_angle = vehicle_param_.steer_ratio() * wheel_angle;
  return steer_angle;
}

bool SpeedDecider::CalculateTTCWithStaticObstacle(
    const Obstacle& obstacle,
    const ReferenceLineInfo* reference_line_info) const {
  if (!obstacle.LongitudinalDecision().has_stop() ||
      reference_line_info == nullptr || obstacle.path_st_boundary().IsEmpty() ||
      init_point_.v() < 10) {
    return false;
  }

  const auto& speed_data = reference_line_info->speed_data();
  if (speed_data.empty()) {
    return false;
  }
  if (obstacle.IsCone() &&
      TL::common::math::double_type::DefinitelyLessEqual(
          speed_data.at(0).a(), -1.0)) {
    return true;
  }
  bool ttc_is_dangerous = false;
  double s_upper = 0.0;
  double s_lower = 0.0;
  obstacle.path_st_boundary().GetBoundarySRange(
      obstacle.path_st_boundary().min_t(), &s_upper, &s_lower);
  s_lower -= obstacle.LongitudinalDecision().stop().distance_s();

  for (const auto& speed_point : speed_data) {
    if (speed_point.t() > 3.0) {
      break;
    }

    auto ttc = 0.0;
    const auto ttc_buffer = fmin(3.0, 0.1 + init_point_.v() * 0.05);
    const auto dv = fmax(speed_point.v(), 0);
    const auto ds = s_lower - speed_point.s() - ttc_buffer;
    const auto ttc_a = -dv * dv / (2 * ds);
    if (TL::common::math::double_type::DefinitelyGreater(ds, 0.0)) {
      ttc = dv <= 0.0 ? std::numeric_limits<double>::infinity() : ds / dv;
    }
    ttc = fmin(10.0, ttc);
    if (ttc <= kTTCRange && ttc_a < -3.5) {
      ttc_is_dangerous = true;
      break;
    }
  }

  return ttc_is_dangerous;
}

bool SpeedDecider::CalculateTTCWithDynamicObstacle(
    const Obstacle& obstacle, const ReferenceLineInfo* reference_line_info,
    const Frame* frame) const {
  const auto& speed_data = reference_line_info->speed_data();
  if (reference_line_info->speed_data().empty() || frame == nullptr ||
      init_point_.v() < 10) {
    return false;
  }
  if (obstacle.Id() == reference_line_info->GetLonFollowObsId() &&
      CalculateExtremeDecIsDangerous(obstacle, reference_line_info)) {
    return true;
  }
  bool ttc_is_dangerous = false;
  const auto& init_adc_v = init_point_.v();
  const auto ttc_buffer = fmin(3.0, 0.1 + init_adc_v * 0.05);

  const auto& obs_st_lower_points = obstacle.path_st_boundary().lower_points();
  const auto& obs_st_upper_points = obstacle.path_st_boundary().upper_points();
  const auto& init_obs_s = (obstacle.PerceptionSLBoundary().start_s() +
                            obstacle.PerceptionSLBoundary().end_s()) /
                           2;
  const auto& init_adc_s = (reference_line_info->AdcSlBoundary().start_s() +
                            reference_line_info->AdcSlBoundary().end_s()) /
                           2;
  // ignore 车辆有路权且障碍物在车辆后方
  if (TL::common::math::double_type::DefinitelyLessEqual(
          speed_data.GetLowRoadRightEndS(), speed_data.at(0).s()) &&
      TL::common::math::double_type::DefinitelyGreaterEqual(init_adc_s,
                                                               init_obs_s)) {
    return false;
  }

  for (size_t i = 0; i < obs_st_lower_points.size(); i++) {
    if (TL::common::math::double_type::DefinitelyGreater(
            obs_st_lower_points.at(i).t(), 2)) {
      break;
    }
    common::SpeedPoint speed_point;
    speed_data.EvaluateByTime(obs_st_lower_points.at(i).t(), &speed_point);
    const auto& adc_s = speed_point.s();
    const auto& obs_v =
        obstacle.GetPointAtTime(obs_st_lower_points.at(i).t()).v();
    const auto& adc_v = speed_point.v();
    auto dv = adc_v - obs_v;
    if (TL::common::math::double_type::IsZero(dv)) {
      dv = 0.001;
    }

    auto ttc = 0.0;
    // 分类考虑adc与障碍物之间的距离关系
    if (obs_st_upper_points.at(i).s() >= adc_s &&
        obs_st_lower_points.at(i).s() <= adc_s) {
      ttc_is_dangerous = true;

    } else if (adc_s < obs_st_lower_points.at(i).s()) {
      if (TL::common::math::double_type::DefinitelyLessEqual(adc_v + 3,
                                                                obs_v)) {
        continue;
      }
      const auto ds = obs_st_lower_points.at(i).s() - adc_s - ttc_buffer;

      if (TL::common::math::double_type::DefinitelyGreater(ds, 0.0)) {
        ttc = dv <= 0.0 ? std::numeric_limits<double>::infinity() : ds / dv;
      }
      ttc = fmin(10.0, ttc);
      if (ttc <= kTTCRange &&
          TL::common::math::double_type::DefinitelyLessEqual(init_point_.a(),
                                                                -3.5)) {
        ttc_is_dangerous = true;
        break;
      }

    } else {
      if (TL::common::math::double_type::DefinitelyLessEqual(
              speed_data.GetLowRoadRightEndS(), adc_s)) {
        break;
      }
      const auto ds = adc_s - obs_st_upper_points.at(i).s() - ttc_buffer;
      if (TL::common::math::double_type::DefinitelyGreater(ds, 0.0)) {
        ttc = dv >= 0.0 ? std::numeric_limits<double>::infinity() : ds / -dv;
      }
      ttc = fmin(10.0, ttc);
      if (ttc <= kTTCRange) {
        ttc_is_dangerous = true;
        break;
      }
    }
    if (ttc_is_dangerous) {
      break;
    }
  }
  return ttc_is_dangerous;
}

bool SpeedDecider::IsFallbackWithST(
    const ReferenceLineInfo* reference_line_info) const {
  if (reference_line_info == nullptr) {
    return true;
  }
  if (reference_line_info->speed_data().empty()) {
    return true;
  }
  const auto& speed_data = reference_line_info->speed_data();
  const auto& path_decision = reference_line_info->path_decision();
  // const auto& path_date = reference_line_info->path_data();
  static constexpr double kBigCarFollowMin = 1.5;
  static constexpr double kBigCarOvertakeMin = 2.0;
  std::vector<std::pair<double, double>> st_drivable(
      static_cast<int>(FLAGS_trajectory_time_length));
  GetSDrivableBoundary(speed_data, &st_drivable);

  /*
  if(fabs(frame_->vehicle_state().timestamp()-1690445946.768320799)<1e-1){
    ADEBUG<<"timestamptest";
  }*/
  // 不可以const auto * const path_decision=reference_line_info->path_decision();
  // 不可以用这个 const PathDecision&类型 path_decision=reference_line_info->path_decision();
  if (reference_line_info->path_decision().obstacles().Items().empty()) {
    return false;
  }
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    if (obstacle == nullptr) {
      return false;
    }
    const auto* mutable_obstacle = path_decision.Find(obstacle->Id());
    if (mutable_obstacle == nullptr) {
      return false;
    }
    double fallback_follow_min = GetFollowDistanceLimit(speed_data.front().v());
    ADEBUG << "isfallback" << fallback_follow_min << " "
           << speed_data.front().v();
    double fallback_overtake_min =
        GetOvertakeDistanceLimit(speed_data.front().v());
    ADEBUG << "isfallback" << fallback_overtake_min << " "
           << speed_data.front().v();
    if (mutable_obstacle->IsOversizedVehicle()) {
      fallback_follow_min += kBigCarFollowMin;
      fallback_overtake_min += kBigCarOvertakeMin;
    }

    // const auto& boundary = mutable_obstacle->path_st_boundary();
    // std::vector<STPoint> fallback_follow_points = boundary.lower_points();
    // std::vector<STPoint> fallback_overtake_points = boundary.upper_points();
    // 跟车过近
    if (mutable_obstacle->LongitudinalDecision().has_follow() &&
        (FallBackFollow(reference_line_info->speed_data(),
                        mutable_obstacle->path_st_boundary().lower_points(),
                        fallback_follow_min, mutable_obstacle->Trajectory(),
                        st_drivable))) {
      AERROR << "fallback_follow_distance_limit";
      return true;
    }

    // 超车过近

    if (mutable_obstacle->LongitudinalDecision().has_overtake() &&
        (FallBackOvertake(reference_line_info->speed_data(),
                          mutable_obstacle->path_st_boundary().upper_points(),
                          fallback_overtake_min, st_drivable))) {
      AERROR << "fallback_overtake_distance_limit";
      return true;
    }
    // 针对锥桶急减速
    if (obstacle->LongitudinalDecision().has_stop() &&
        obstacle->Perception().sub_type() ==
            perception::PerceptionObstacle::ST_TRAFFICCONE &&
        (std::any_of(speed_data.begin(), speed_data.end(),
                     [&](const auto& speed_point) {
                       return speed_point.a() < kConeDec;
                     }))) {
      return true;
    }

    // 横纵向过近
    /*if (FallBackLatlon(mutable_obstacle->GetTrajectoryEnvelope(),
                       path_date.frenet_frame_path())) {
      AERROR << "fallback_lon&lat_limit";
      return true;
    }*/
  }
  // 过急减速
  if (std::any_of(speed_data.begin(), speed_data.end(),
                  [&](const auto& speed_point) {
                    return speed_point.a() <
                           speed_decider_config_.fallback_decel_threshold();
                  })) {
    AERROR << "fallback_decel_limit";

    return true;
  }
  // 过急加速
  if (std::any_of(speed_data.begin(), speed_data.end(),
                  [&](const auto& speed_point) {
                    return speed_point.a() >
                           speed_decider_config_.fallback_accel_threshold();
                  })) {
    AERROR << "fallback_accel_limit";

    return true;
  }

  return false;
}

bool SpeedDecider::FallBackOvertake(  // NOLINT
    const SpeedData& speed_data,
    const std::vector<STPoint>& fallback_overtake_points,
    double fallback_overtake_min,
    const std::vector<std::pair<double, double>>& st_drivable) const {
  if (speed_data.empty() || st_drivable.empty()) {
    return false;
  }
  for (const auto& fallback_overtake_point : fallback_overtake_points) {
    auto index = static_cast<int>(fallback_overtake_point.t() /
                                  FLAGS_trajectory_time_resolution);
    if (index < 0) {
      continue;
    }
    if (index >= static_cast<int>(speed_data.size()) ||
        (TL::common::math::double_type::DefinitelyGreater(
            fallback_overtake_point.t(), 2))) {
      break;
    }
    double overtake_s = speed_data.at(index).s();
    ADEBUG << "overtake_distance_ds"
           << overtake_s - fallback_overtake_point.s();
    const auto t_index = common::math::Clamp(
        static_cast<int>(fallback_overtake_point.t() / kDrivableBoundarydt), 0,
        static_cast<int>(st_drivable.size() - 1));
    if (TL::common::math::double_type::DefinitelyGreaterEqual(
            (overtake_s - fallback_overtake_point.s()),
            fallback_overtake_min) ||
        TL::common::math::double_type::DefinitelyGreater(
            st_drivable.at(t_index).second, fallback_overtake_point.s())) {
      continue;
    }
    ADEBUG << "fallback_overtake_distance_limit"
           << overtake_s - fallback_overtake_point.s() << "<"
           << fallback_overtake_min << "overtake_s" << overtake_s << " "
           << fallback_overtake_point.s();
    return true;
  }
  return false;
}

/*bool SpeedDecider::FallBackLatlon(
    const std::vector<ObsPointDescription>& trajectory_envelope,
    const FrenetFramePath& frenet_frame_path) const {
  if (frenet_frame_path.empty()) {
    return false;
  }
  const auto& car_sl = frenet_frame_path.at(0);
  const double reardis = 5;
  const double l_limit = 0.7;
  const double s_limit = 2;
  double ob_min_l = 0;
  for (const auto& envelope : trajectory_envelope) {
    ADEBUG << "lateral and longitudinal_test";
    if ((!reference_line_info_->IsChangeLanePath() &&
         envelope.center_p.s() + reardis < car_sl.s()) ||
        (TL::common::math::double_type::DefinitelyGreater(envelope.time,
                                                              1))) {
      break;
    }
    if (envelope.center_p.l() > car_sl.l()) {
      ob_min_l = fmin(envelope.low_left_p.l(), envelope.low_right_p.l());
    } else {
      ob_min_l = fmax(envelope.upper_left_p.l(), envelope.upper_right_p.l());
    }
    auto ob_s = envelope.center_p.s();
    for (size_t i = 0; i + 1 < frenet_frame_path.size(); i++) {
      if ((ob_s - frenet_frame_path.at(i).s()) *
              (ob_s - frenet_frame_path.at(i + 1).s()) <
          0) {
        // 障碍物在车辆参考线位置的映射
        ob_s = frenet_frame_path.at(i).s();
        break;
      }
    }
    auto l_de_min = fabs(car_sl.l() - ob_min_l);
    auto s_de_min = fabs(car_sl.s() - ob_s);
    ADEBUG << "l_de_min" << l_de_min << "s_de_min" << s_de_min;
    if (l_de_min < l_limit && s_de_min < s_limit) {
      AERROR << "fallback_lateral and longitudinal too close"
             << "l_de_min" << l_de_min << "s_de_min" << s_de_min;
      return true;
    }
  }
  return false;
}*/

bool SpeedDecider::FallBackFollow(  // NOLINT
    const SpeedData& speed_data,
    const std::vector<STPoint>& fallback_follow_points,
    double fallback_follow_min, const prediction::Trajectory& trajectory,
    const std::vector<std::pair<double, double>>& st_drivable) const {
  if (speed_data.empty() || st_drivable.empty()) {
    return false;
  }

  for (size_t i = 0; i < fallback_follow_points.size(); i++) {
    auto index = static_cast<int>(fallback_follow_points.at(i).t() /
                                  FLAGS_trajectory_time_resolution);
    if (trajectory.trajectory_point().empty()) {
      return false;
    }
    if (index < 0) {
      continue;
    }
    if (index >= static_cast<int>(speed_data.size()) ||
        (TL::common::math::double_type::DefinitelyGreater(
            fallback_follow_points.at(i).t(), 2))) {
      break;
    }

    double follow_s = speed_data.at(index).s();
    auto follow_distance_ds = fallback_follow_points.at(i).s() - follow_s;
    ADEBUG << "follow_distance_ds" << follow_distance_ds;
    const auto& obtraj_v = trajectory.trajectory_point(0).v();
    if ((obtraj_v - kFollowspeedds > speed_data.front().v() &&
         obtraj_v > kFollowspeedds)) {
      break;
    }
    const auto t_index =
        common::math::Clamp(static_cast<int>(fallback_follow_points.at(i).t() /
                                             kDrivableBoundarydt),
                            0, static_cast<int>(st_drivable.size() - 1));

    if (TL::common::math::double_type::DefinitelyGreaterEqual(
            follow_distance_ds, fallback_follow_min) ||
        TL::common::math::double_type::DefinitelyLess(
            st_drivable.at(t_index).first, fallback_follow_points.at(i).s())) {
      ADEBUG << "is_can_break"
             << "  "
             << "st_drivable.at(i).first =  " << st_drivable.at(t_index).first
             << "t_index= " << t_index << "fallback_follow_points.at(i).s()= "
             << fallback_follow_points.at(i).s() << "follow_s= " << follow_s
             << "i=" << i;
      continue;
    }

    // double safe_a = kBreakDec;
    // double safe_t = fallback_follow_points.at(i).t();
    // auto safe_break_s = speed_data.front().s() +
    //                     speed_data.front().v() * safe_t +
    //                     0.5 * safe_a * pow(safe_t, 2) + fallback_follow_min;
    // if (fallback_follow_points.at(i).s() > safe_break_s) {
    //   // dis_limit_safe < (fallback_follow_min)
    //   ADEBUG << "is_can_break"
    //          << " "
    //          << "safe_break_s" << safe_break_s << "< "
    //          << "fallback_follow_points[i].s()"
    //          << fallback_follow_points.at(i).s();
    //   break;
    // }

    ADEBUG << "follow_distance_ds: " << follow_distance_ds << "<"
           << "fallback_follow_min: " << fallback_follow_min
           << "fallback_follow_points[i].s(): "
           << " " << fallback_follow_points.at(i).s()
           << "follow_s: " << follow_s << "t_index: " << t_index
           << "st_drivable.at(t_index).first" << st_drivable.at(t_index).first;

    return true;
  }
  return false;
}

void SpeedDecider::SetSDrivableBoundary(  // NOLINT
    const SpeedData& speed_data,
    std::vector<std::pair<double, double>>* const st_drivable) const {
  if (speed_data.empty() || st_drivable == nullptr) {
    return;
  }
  double t = 0;
  const double cur_v = fabs(speed_data.at(0).v());
  const double dec_time = fabs(cur_v / kBreakDec);
  const double acc_time = fabs(kMaxv - cur_v) / kAcc;
  size_t st_drivable_size = st_drivable->size();

  for (size_t i = 0; i < st_drivable_size; ++i) {
    if (t < dec_time) {
      st_drivable->at(i).first =
          speed_data.at(0).s() + 0.5 * kBreakDec * t * t + cur_v * t;
    } else {
      st_drivable->at(i).first = speed_data.at(0).s() + dec_time * 0.5 * cur_v;
    }
    if (st_drivable->at(i).first < 1e-3) {
      st_drivable->at(i).first = 0.0;
    }

    if (t < acc_time) {
      st_drivable->at(i).second =
          speed_data.at(0).s() + 0.5 * kAcc * t * t + cur_v * t;
    } else {
      st_drivable->at(i).second = speed_data.at(0).s() +
                                  0.5 * acc_time * (cur_v + kMaxv) +
                                  kMaxv * (t - acc_time);
    }
    t += kDrivableBoundarydt;
  }
}

void SpeedDecider::GetSDrivableBoundary(
    const SpeedData& speed_data,
    std::vector<std::pair<double, double>>* const st_drivable) const {
  if (speed_data.empty() || st_drivable == nullptr) {
    return;
  }
  // for (int i = 0; i < FLAGS_trajectory_time_length; ++i) {
  //   st_drivable->at(i) = {std::numeric_limits<double>::min(),
  //                         std::numeric_limits<double>::max()};
  // }
  st_drivable->assign(
      st_drivable->size(),
      {std::numeric_limits<double>::min(), std::numeric_limits<double>::max()});
  SetSDrivableBoundary(speed_data, st_drivable);
}

bool SpeedDecider::CalculateExtremeDecIsDangerous(
    const Obstacle& obstacle, const ReferenceLineInfo* reference_line_info) {
  if (reference_line_info == nullptr ||
      obstacle.path_st_boundary().lower_points().size() < 2) {
    return false;
  }
  const auto& speed_data = reference_line_info->speed_data();
  const auto size = std::min(obstacle.path_st_boundary().lower_points().size(),
                             obstacle.path_st_boundary().speed_points().size());
  for (size_t i = 0; i < size; ++i) {
    const auto& cur_obs_point =
        obstacle.path_st_boundary().lower_points().at(i);
    if (TL::common::math::double_type::DefinitelyGreater(cur_obs_point.t(),
                                                            3)) {
      break;
    }
    common::SpeedPoint speed_point;
    speed_data.EvaluateByTime(cur_obs_point.t(), &speed_point);
    const auto& adc_s = speed_point.s();
    const auto& adc_v = speed_point.v();
    const auto& obs_s = cur_obs_point.s();
    const auto& obs_v = obstacle.path_st_boundary().speed_points().at(i).s();
    const auto relative_speed = adc_v - obs_v;
    const auto relative_distance = obs_s - adc_s;
    if (relative_speed < 0 || relative_distance < 0 || obs_s < 0 || obs_v < 0) {
      continue;
    }
    const auto required_deceleration =
        relative_speed * relative_speed / (2 * relative_distance);
    if (TL::common::math::double_type::DefinitelyGreater(
            required_deceleration, kExtremeDec)) {
      return true;
    }
  }

  return false;
}
}  // namespace planning
}  // namespace TL
