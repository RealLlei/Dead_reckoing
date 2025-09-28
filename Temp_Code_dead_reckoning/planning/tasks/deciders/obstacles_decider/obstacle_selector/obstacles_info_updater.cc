/*
 * Copyright (c) 2022 TL Technologies Co., Ltd. All rights reserved.
 */

#include "planning/tasks/deciders/obstacles_decider/obstacle_selector/obstacles_info_updater.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "common/math/double_type.h"
#include "planning/common/obstacle_blocking_analyzer.h"
#include "planning/common/path_decision.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/planning/sl_boundary.pb.h"

namespace TL {
namespace planning {
namespace {
constexpr double kEpsilon = 0.0000001;
constexpr double kHalf = 0.5;
constexpr uint32_t kObsMinTrackingCnt = 3;
constexpr double kObsInNeighborLaneRatio = 0.1;
constexpr double kAdcMergeFirstObsArrivalTimeGap = 3.0;
constexpr double kInterObsSpeedMinThd = 5.0;
}  // namespace

using TL::common::math::double_type::DefinitelyGreaterEqual;

void ObstaclesInfo::GetObsInitialBehavior(
    const ReferenceLineInfo& reference_line_info) {
  ObsInfoClear();
  const PathDecision& path_decision = reference_line_info.path_decision();
  const auto& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  const double adc_end_s = adc_sl_boundary.end_s();
  ADEBUG << "adc_end_s=" << adc_end_s;
  const double adc_start_s = adc_sl_boundary.start_s();
  const double adc_speed =
      reference_line_info.vehicle_state().linear_velocity();
  ADEBUG << "adc_speed=" << adc_speed;
  const ObstacleInfo adc_info = {adc_start_s,
                                 adc_end_s,
                                 adc_sl_boundary.start_l(),
                                 adc_sl_boundary.end_l(),
                                 adc_speed,
                                 ObstacleBehavior::Unknown,
                                 -1};
  obstacles_behavior_info_.emplace("adc", adc_info);

  double lane_left_width = FLAGS_default_reference_line_width * kHalf;
  double lane_right_width = lane_left_width;
  reference_line_info.reference_line().GetLaneWidth(adc_end_s, &lane_left_width,
                                                    &lane_right_width);
  ADEBUG << "lane_left_width=" << lane_left_width;
  ADEBUG << "lane_right_width=" << lane_right_width;

  const auto& adc_map_common_info =
      reference_line_info.reference_line().GetAdcMapCommonInfo();
  // adc_distance_to_merging_end_s_ =
  //     adc_map_common_info.adc_distance_to_merging_end;
  // ADEBUG << "adc_distance_to_merging_end=" << adc_distance_to_merging_end_s_;
  // if (!std::isinf(adc_distance_to_merging_end_s_)) {
  //   adc_in_dying_road_ = true;
  // }
  // use merge info from pnc_map
  if (adc_map_common_info.vehicle_in_merge_lane &&
      adc_map_common_info.dis_first_merge_point > kEpsilon) {
    adc_in_dying_road_ = adc_map_common_info.vehicle_in_merge_lane;
    ADEBUG << std::boolalpha << "vehicle_in_merge_lane=" << adc_in_dying_road_;
    adc_distance_to_merging_end_s_ = adc_map_common_info.dis_first_merge_point;
    ADEBUG << "dis_first_merge_point=" << adc_distance_to_merging_end_s_;
  }

  std::pair<std::string, double> nearest_back_cutin_obs = {"", 0.0};
  for (const auto* obs : path_decision.obstacles().Items()) {
    ADEBUG << "obs_id=" << obs->Id();
    ADEBUG << "obs_speed=" << obs->speed();
    if (ObstacleHasInvalidBoudary(obs)) {
      continue;
    }
    const auto& obs_sl = obs->PerceptionSLBoundary();
    const bool obs_is_front = obs_sl.end_s() > adc_start_s;
    if (obs_is_front) {
      adc_front_obs_.emplace_back(obs->Id());
    }
    if (ObstacleHasInvalidTrackingTime(obs)) {
      continue;
    }
    const bool is_vehicle =
        obs->Perception().type() == perception::PerceptionObstacle::VEHICLE ||
        obs->Perception().type() == perception::PerceptionObstacle::UNKNOWN ||
        obs->Perception().type() ==
            perception::PerceptionObstacle::UNKNOWN_MOVABLE;
    if (!is_vehicle) {
      ADEBUG << "obs_is_not_vehicle";
      continue;
    }

    // set front cross obs
    if (obs_is_front && SetCrossObs(obs, reference_line_info)) {
      continue;
    }

    const auto& trajectory_envelope = obs->GetTrajectoryEnvelope();
    bool is_forward = false;
    bool is_going_to_follow_adc_lane = false;
    if (!trajectory_envelope.empty() &&
        DefinitelyGreaterEqual(trajectory_envelope.back().center_p.s(),
                               trajectory_envelope.front().center_p.s())) {
      is_forward = true;
      is_going_to_follow_adc_lane =
          IsObsGoingToFollowAdcLane(reference_line_info, trajectory_envelope);
    }
    ADEBUG << std::boolalpha
           << "is_going_to_follow_adc_lane=" << is_going_to_follow_adc_lane;

    const bool is_blocking = IsBlockingDrivingPathObstacle(
        reference_line_info.reference_line(), obs);
    ADEBUG << std::boolalpha << "is_blocking=" << is_blocking;

    // const bool is_onlane =
    //     reference_line_info.reference_line().IsOnLane(obs_sl);
    // ADEBUG << std::boolalpha << "is_onlane=" << is_onlane;
    const double obs_start_s = obs->PerceptionSLBoundary().start_s();
    // set merge
    if (adc_in_dying_road_ &&
        obs_start_s < adc_distance_to_merging_end_s_ + adc_end_s) {
      // there may be multiple merges!!!
      if (obs->Intent() == prediction::ObstacleIntent::MERGING &&
          !is_blocking) {
        if (obs_is_front) {
          neighbor_front_merge_obs_.emplace_back(obs->Id(), obs_start_s);
        } else {
          neighbor_back_merge_obs_.emplace_back(obs->Id(), obs_start_s);
        }
        ADEBUG << "obs intent is merging";
      }
    }
    if (obs_is_front) {
      if (is_going_to_follow_adc_lane) {
        front_follow_adc_lane_obs_.emplace_back(obs->Id());
      }
      // set front on lane obs
      if (SetBlockingObs(obs, adc_end_s, is_forward, is_blocking,
                         is_going_to_follow_adc_lane)) {
        if (obs_start_s < nearest_blocking_obs_.second) {
          nearest_blocking_obs_ = {obs->Id(), obs_start_s};
        }
        continue;
      }
      // set front cut_in obs
      if (SetCutInObs(obs, is_blocking, is_going_to_follow_adc_lane)) {
        front_cut_in_obs_.emplace_back(obs->Id());
        adc_front_valid_obstacles_.emplace_back(obs->Id());
        cutin_obs_perception_ids_.emplace(obs->PerceptionId());
        if (obs_start_s < nearest_front_cutin_obs_.second) {
          nearest_front_cutin_obs_ = {obs->Id(), obs_start_s};
        }
        continue;
      }
    } else if (adc_in_dying_road_) {
      // set back cut_in obs
      if (SetCutInObs(obs, is_blocking, is_going_to_follow_adc_lane)) {
        cutin_obs_perception_ids_.emplace(obs->PerceptionId());
        if (obs_start_s > nearest_back_cutin_obs.second) {
          nearest_back_cutin_obs = {obs->Id(), obs_start_s};
        }
        continue;
      }
    }
    SetAlongsideObs(obs, is_blocking, lane_left_width, lane_right_width);
  }
  // if (adc_in_dying_road_) {
  //   SelectInteractiveObs(adc_end_s, adc_speed);
  // }
  // ObsInfoDebug();
}

bool ObstaclesInfo::SetCrossObs(const Obstacle* obs,
                                const ReferenceLineInfo& reference_line_info) {
  const double heading_difference =
      ObsHeadingDifferenceFromAdc(obs, &reference_line_info);
  if ((heading_difference < (M_PI_2 + M_PI_4) &&
       heading_difference > (M_PI_4)) &&
      obs->IsCrossReferenceLine()) {
    cross_obs_.emplace_back(obs->Id());
    adc_front_valid_obstacles_.emplace_back(obs->Id());

    const auto& obs_sl = obs->PerceptionSLBoundary();
    const ObstacleInfo obs_info = {obs_sl.start_s(),   obs_sl.end_s(),
                                   obs_sl.start_l(),   obs_sl.end_l(),
                                   obs->speed(),       ObstacleBehavior::Cross,
                                   obs->PerceptionId()};
    obstacles_behavior_info_.emplace(obs->Id(), obs_info);
    ADEBUG << "cross_obs_=" << obs->Id();
    return true;
  }
  return false;
}

bool ObstaclesInfo::SetBlockingObs(const Obstacle* obs, const double adc_s,
                                   bool is_forward, const bool is_blocking,
                                   const bool is_going_to_follow_adc_lane) {
  const auto& obs_sl = obs->PerceptionSLBoundary();
  is_forward = obs->IsStatic() ? true : is_forward;
  if (is_forward && is_blocking) {
    ADEBUG << "front_onlane_obs";
    adc_front_valid_obstacles_.emplace_back(obs->Id());

    const ObstacleInfo obs_info = {obs_sl.start_s(),   obs_sl.end_s(),
                                   obs_sl.start_l(),   obs_sl.end_l(),
                                   obs->speed(),       ObstacleBehavior::Follow,
                                   obs->PerceptionId()};
    obstacles_behavior_info_.emplace(obs->Id(), obs_info);
    const double obs_start_s = obs_sl.start_s();
    // cipv should fulfil obs_start_s > adc_end_s, cipv is totally in front of adc
    if (obs_start_s > adc_s && obs_start_s < cipv_.second &&
        is_going_to_follow_adc_lane) {
      ADEBUG << "cipv";
      cipv_ = {obs->Id(), obs_start_s};
    }
    return true;
  }
  return false;
}

bool ObstaclesInfo::SetCutInObs(const Obstacle* obs, const bool is_blocking,
                                const bool is_going_to_follow_adc_lane) {
  if (!obs->IsStatic() && !obs->reference_line_st_boundary().IsEmpty()) {
    if (!is_blocking && is_going_to_follow_adc_lane) {
      ADEBUG << "cut_in_obs_";
      const auto& obs_sl = obs->PerceptionSLBoundary();
      const ObstacleInfo obs_info = {
          obs_sl.start_s(),   obs_sl.end_s(), obs_sl.start_l(),
          obs_sl.end_l(),     obs->speed(),   ObstacleBehavior::CutIn,
          obs->PerceptionId()};
      obstacles_behavior_info_.emplace(obs->Id(), obs_info);
      return true;
    }
  }
  return false;
}

bool ObstaclesInfo::IsObsGoingToFollowAdcLane(
    const ReferenceLineInfo& reference_line_info,
    const std::vector<ObsPointDescription>& trajectory_envelope) {
  const double ref_length = reference_line_info.reference_line().Length();
  ADEBUG << "ref_line_length=" << ref_length;
  const std::pair<double, double> traj_back =
      GetTrajFarthestPoint(ref_length, trajectory_envelope);
  common::SLPoint ref_point;
  ref_point.set_s(traj_back.first);
  ref_point.set_l(traj_back.second);
  const bool is_going_to_follow_adc_lane =
      reference_line_info.reference_line().IsOnLane(ref_point);
  return is_going_to_follow_adc_lane;
}

std::pair<double, double> ObstaclesInfo::GetTrajFarthestPoint(
    double ref_length,
    const std::vector<ObsPointDescription>& trajectory_envelope) {
  double traj_back_s = trajectory_envelope.back().center_p.s();
  double traj_back_l = trajectory_envelope.back().center_p.l();
  ADEBUG << "obs_front_center_s=" << trajectory_envelope.front().center_p.s();
  ADEBUG << "obs_back_center_s=" << trajectory_envelope.back().center_p.s();
  if (ref_length < traj_back_s) {
    for (const auto& envelope : trajectory_envelope) {
      if (envelope.center_p.s() < ref_length) {
        traj_back_s = envelope.center_p.s();
        traj_back_l = envelope.center_p.l();
      } else {
        break;
      }
    }
  }
  ADEBUG << "traj_back_s=" << traj_back_s << ", traj_back_l=" << traj_back_l;
  return {traj_back_s, traj_back_l};
}

void ObstaclesInfo::GetObsFrontCIPV(std::vector<std::string>* const ids,
                                    const IndexedObstacles& obstacles) {
  if (cipv_.first.empty()) {
    return;
  }
  const auto* cipv_ptr = obstacles.Find(cipv_.first);
  if (cipv_ptr == nullptr) {
    return;
  }
  ADEBUG << "cipv: " << cipv_.first;
  const double cipv_s = cipv_ptr->PerceptionSLBoundary().end_s();
  for (const auto& obs_id : adc_front_obs_) {
    ADEBUG << "adc_front_obs_id=" << obs_id;
    const auto* obs_ptr = obstacles.Find(obs_id);
    if (obs_ptr == nullptr) {
      continue;
    }
    if (cipv_ptr->PerceptionId() == obs_ptr->PerceptionId()) {
      continue;
    }
    if (obs_ptr->PerceptionSLBoundary().start_s() > cipv_s) {
      ids->emplace_back(obs_id);
      ADEBUG << "adc_front_obs_id_not_cipv=" << obs_id;
    } else if (!obs_ptr->reference_line_st_boundary().IsEmpty()) {
      // min_s means the s of nearest point
      if (obs_ptr->reference_line_st_boundary().min_s() > cipv_s &&
          obs_id != interactive_obs_[0].first &&
          obs_id != interactive_obs_[1].first) {
        ids->emplace_back(obs_id);
        ADEBUG << "adc_front_obs_id_not_cipv and not interactive=" << obs_id;
      }
    }
  }
}

bool ObstaclesInfo::ObstacleHasInvalidBoudary(const Obstacle* obs) {
  if (obs == nullptr) {
    return true;
  }
  if (obs->IsVirtual()) {
    return true;
  }
  if (obs->HasLongitudinalDecision() &&
      obs->LongitudinalDecision().has_ignore() && obs->HasLateralDecision() &&
      obs->LateralDecision().has_ignore()) {
    return true;
  }
  const auto& obs_sl_boundary = obs->PerceptionSLBoundary();
  return (!obs_sl_boundary.has_end_l() || !obs_sl_boundary.has_start_l() ||
          !obs_sl_boundary.has_start_s() || !obs_sl_boundary.has_end_s());
}

bool ObstaclesInfo::ObstacleHasInvalidTrackingTime(const Obstacle* obs) {
  // static obstacle also has tracking_time
  if (!obs->TrackStatus().has_is_tracking() ||
      !obs->TrackStatus().is_tracking() ||
      !obs->TrackStatus().has_tracking_counter() ||
      obs->TrackStatus().tracking_counter() < kObsMinTrackingCnt) {
    ADEBUG << "obs_tracking_time is not long";
    return true;
  }
  return false;
}

double ObstaclesInfo::ObsHeadingDifferenceFromAdc(
    const Obstacle* obstacle, const ReferenceLineInfo* reference_line_info) {
  // obstacle->IsStatic()
  if (!obstacle->HasTrajectory()) {
    return 0.0;
  }
  double obstacle_moving_direction =
      obstacle->Trajectory().trajectory_point(0).path_point().theta();
  const int obs_traj_point_size =
      obstacle->Trajectory().trajectory_point_size();
  // get middle point theta
  if (obs_traj_point_size >= 1) {
    obstacle_moving_direction = obstacle->Trajectory()
                                    .trajectory_point(obs_traj_point_size / 2)
                                    .path_point()
                                    .theta();
  }
  const auto& vehicle_state = reference_line_info->vehicle_state();
  double vehicle_moving_direction = vehicle_state.heading();
  if (vehicle_state.gear() == soc::Chassis::GEAR_REVERSE) {
    vehicle_moving_direction =
        common::math::NormalizeAngle(vehicle_moving_direction + M_PI);
  }
  const double heading_difference = std::fabs(common::math::NormalizeAngle(
      obstacle_moving_direction - vehicle_moving_direction));
  ADEBUG << "heading_difference=" << heading_difference;
  return heading_difference;
}

void ObstaclesInfo::SetAlongsideObs(const Obstacle* obstacle,
                                    const bool is_blocking,
                                    const double lane_left_width,
                                    const double lane_right_width) {
  const auto& obs_sl_boundary = obstacle->PerceptionSLBoundary();
  const double left_neighbor_lane_width = lane_left_width + lane_right_width;
  const double right_neighbor_lane_width = left_neighbor_lane_width;
  const double obs_start_l = obs_sl_boundary.start_l();
  const double obs_end_l = obs_sl_boundary.end_l();
  ADEBUG << "obs_start_l()=" << obs_start_l;
  ADEBUG << "obs_end_l()=" << obs_end_l;
  const double obs_l = (obs_start_l + obs_end_l) * kHalf;
  if (!is_blocking) {
    // obs_sl_boundary.start_l() > lane_left_width
    if (obs_l > 0.0 &&
        obs_end_l < lane_left_width + left_neighbor_lane_width *
                                          (1 + kObsInNeighborLaneRatio)) {
      ADEBUG << "ObstaclePosition::LEFT";
      alongside_obs_perception_ids_.emplace(obstacle->PerceptionId());

      const auto& obs_sl = obstacle->PerceptionSLBoundary();
      const ObstacleInfo obs_info = {
          obs_sl.start_s(),        obs_sl.end_s(),
          obs_sl.start_l(),        obs_sl.end_l(),
          obstacle->speed(),       ObstacleBehavior::Alongside_Left,
          obstacle->PerceptionId()};
      obstacles_behavior_info_.emplace(obstacle->Id(), obs_info);
    }
    // obs_sl_boundary.end_l() < -lane_right_width
    if (obs_l < 0.0 &&
        obs_start_l > -lane_right_width - (right_neighbor_lane_width *
                                           (1 + kObsInNeighborLaneRatio))) {
      ADEBUG << "ObstaclePosition::RIGHT";
      alongside_obs_perception_ids_.emplace(obstacle->PerceptionId());

      const auto& obs_sl = obstacle->PerceptionSLBoundary();
      const ObstacleInfo obs_info = {
          obs_sl.start_s(),        obs_sl.end_s(),
          obs_sl.start_l(),        obs_sl.end_l(),
          obstacle->speed(),       ObstacleBehavior::Alongside_Right,
          obstacle->PerceptionId()};
      obstacles_behavior_info_.emplace(obstacle->Id(), obs_info);
    }
  }
}

void ObstaclesInfo::SelectInteractiveObs(const double adc_end_s,
                                         const double adc_speed) {
  // judge whether adc has follow_obs in dying road
  bool adc_has_follow_obs_in_dying_road = false;
  if (!nearest_blocking_obs_.first.empty()) {
    const auto& obs_iter =
        obstacles_behavior_info_.find(nearest_blocking_obs_.first);
    if (obs_iter != obstacles_behavior_info_.end()) {
      const double obs_s = obs_iter->second.start_s;
      const double obs_l =
          (obs_iter->second.start_l + obs_iter->second.end_l) * kHalf;
      ADEBUG << "nearest_blocking_obs_s=" << obs_s;
      ADEBUG << "nearest_blocking_obs_l=" << obs_l;
      if (obs_s < adc_distance_to_merging_end_s_ + adc_end_s) {
        ADEBUG << "adc has follow_obs in dying road";
        adc_has_follow_obs_in_dying_road = true;
      }
    }
  }

  // the nearest obs to merge point
  std::string nearest_obs_to_merge_point_id;
  double nearest_obs_to_merge_point_s = 0.0;
  if (!neighbor_front_merge_obs_.empty()) {
    // sort according s
    sort(neighbor_front_merge_obs_.begin(), neighbor_front_merge_obs_.end(),
         [](std::pair<std::string, double>& left,
            std::pair<std::string, double>& right) {
           return left.second > right.second;
         });

    const auto& first_obs = neighbor_front_merge_obs_.front();
    nearest_obs_to_merge_point_id = first_obs.first;
    nearest_obs_to_merge_point_s = first_obs.second;
  }
  ADEBUG << "nearest_obs_to_merge_point_id=" << nearest_obs_to_merge_point_id;
  ADEBUG << "nearest_obs_to_merge_point_s=" << nearest_obs_to_merge_point_s;

  // the nearest obs out of merge point
  std::string nearest_obs_out_of_merge_point_id;
  double nearest_obs_out_of_merge_point_s = 0.0;
  for (const auto& obs_id : front_follow_adc_lane_obs_) {
    const auto& obs_iter = obstacles_behavior_info_.find(obs_id);
    if (obs_iter != obstacles_behavior_info_.end()) {
      const double obs_s = obs_iter->second.start_s;
      if (obs_s > adc_distance_to_merging_end_s_ + adc_end_s &&
          obs_s < nearest_obs_out_of_merge_point_s) {
        nearest_obs_out_of_merge_point_id = obs_id;
        nearest_obs_out_of_merge_point_s = obs_s;
      }
    }
  }
  ADEBUG << "nearest_obs_out_of_merge_point_id="
         << nearest_obs_out_of_merge_point_id;
  ADEBUG << "nearest_obs_out_of_merge_point_s="
         << nearest_obs_out_of_merge_point_s;

  // adc leading_obs
  if (!nearest_blocking_obs_.first.empty()) {
    adc_leading_obs_id_ = nearest_blocking_obs_.first;
  }
  ADEBUG << "nearest_blocking_obs_=" << nearest_blocking_obs_.first;

  // judge whether adc can overtake the nearest obs to merge point
  double nearest_inter_obs_to_merge_point_s =
      std::numeric_limits<double>::infinity();
  if (!nearest_obs_to_merge_point_id.empty()) {
    ADEBUG << "nearest_obs_to_merge_point_id=" << nearest_obs_to_merge_point_id;
    const auto& nearest_obs_to_merge_point =
        obstacles_behavior_info_.find(nearest_obs_to_merge_point_id);
    if (nearest_obs_to_merge_point != obstacles_behavior_info_.end()) {
      const double obs_arrival_time =
          (adc_distance_to_merging_end_s_ + adc_end_s -
           nearest_obs_to_merge_point->second.end_s) /
          std::fmax(nearest_obs_to_merge_point->second.speed, kEpsilon);
      double adc_arrival_time =
          adc_distance_to_merging_end_s_ / std::fmax(adc_speed, kEpsilon);
      // adc_arrival_time base on the speed of adc_leading_obs
      const auto& adc_leading_obs =
          obstacles_behavior_info_.find(adc_leading_obs_id_);
      if (adc_has_follow_obs_in_dying_road &&
          adc_leading_obs != obstacles_behavior_info_.end()) {
        double adc_leading_arrival_time =
            (adc_distance_to_merging_end_s_ + adc_end_s -
             adc_leading_obs->second.end_s) /
            std::fmax(adc_leading_obs->second.speed, kEpsilon);
        double delta_t =
            (adc_leading_obs->second.end_s - adc_end_s) /
            std::fmax(kHalf * (adc_leading_obs->second.speed + adc_speed),
                      kEpsilon);
        adc_arrival_time = adc_leading_arrival_time + delta_t;
      }
      ADEBUG << "obs_arrival_time=" << obs_arrival_time;
      ADEBUG << "adc_arrival_time=" << adc_arrival_time;
      if (adc_arrival_time + kAdcMergeFirstObsArrivalTimeGap <
          obs_arrival_time) {
        interactive_obs_[0].first = nearest_obs_to_merge_point_id;
        nearest_inter_obs_to_merge_point_s = nearest_obs_to_merge_point_s;
        if (!nearest_obs_out_of_merge_point_id.empty()) {
          ADEBUG
              << "inter_leading_obs_id_ is nearest_obs_out_of_merge_point_id";
          interactive_obs_[0].second = nearest_obs_out_of_merge_point_id;
        }
        ADEBUG << "adc can overtake the nearest obs to merge point with a "
                  "high probability";
      }
    }
  }

  bool has_nearest_neighbor_front_merge_obs = false;
  double nearest_neighbor_front_merge_obs_s =
      std::numeric_limits<double>::infinity();
  for (auto it = neighbor_front_merge_obs_.rbegin();
       it != neighbor_front_merge_obs_.rend(); ++it) {
    if (!has_nearest_neighbor_front_merge_obs &&
        (*it).second < nearest_inter_obs_to_merge_point_s) {
      interactive_obs_[1].first = (*it).first;
      ADEBUG << "nearest neighbor_front_merge_obs_="
             << interactive_obs_[1].first;
      has_nearest_neighbor_front_merge_obs = true;
      nearest_neighbor_front_merge_obs_s = (*it).second;
    }
    if (has_nearest_neighbor_front_merge_obs &&
        (*it).second > nearest_neighbor_front_merge_obs_s) {
      interactive_obs_[1].second = (*it).first;
      ADEBUG << "leading_id of nearest neighbor_front_merge_obs_="
             << interactive_obs_[1].second;
      break;
    }
  }

  // if adc in dying road, yield the front_cutin_obs
  // and select nearest_back_cutin_id_ as interactive_obs_id_
  if (!neighbor_back_merge_obs_.empty()) {
    // sort according s
    sort(neighbor_back_merge_obs_.begin(), neighbor_back_merge_obs_.end(),
         [](std::pair<std::string, double>& left,
            std::pair<std::string, double>& right) {
           return left.second > right.second;
         });

    interactive_obs_[2].first = neighbor_back_merge_obs_.front().first;
    if (!neighbor_front_merge_obs_.empty()) {
      interactive_obs_[2].second = neighbor_front_merge_obs_.back().first;
      ADEBUG << "inter_leading_obs_id_ is nearest_front_cutin_obs_";
    } else if (!nearest_obs_out_of_merge_point_id.empty()) {
      ADEBUG << "inter_leading_obs_id_ is nearest_obs_out_of_merge_point_id";
      interactive_obs_[2].second = nearest_obs_out_of_merge_point_id;
    }
  }

  std::string first_interactive_id;
  for (auto& interactive_ob : interactive_obs_) {
    if (!interactive_ob.first.empty()) {
      first_interactive_id = interactive_ob.first;
      break;
    }
  }
  // disenable when the roads are crowded
  if (!first_interactive_id.empty()) {
    const auto& obs_iter = obstacles_behavior_info_.find(first_interactive_id);
    if (obs_iter != obstacles_behavior_info_.end()) {
      const double obs_v = obs_iter->second.speed;
      ADEBUG << "inter_obs_id=" << first_interactive_id
             << ", inter_obs_v=" << obs_v;
      if (obs_v < kInterObsSpeedMinThd) {
        ADEBUG << "cancel behavior_evaluator because of low speed of "
                  "interactive_obs";
        interactive_obs_ = {};
      }
    }
  }
}

void ObstaclesInfo::ObsInfoClear() {
  adc_front_valid_obstacles_.clear();
  adc_front_obs_.clear();
  front_cut_in_obs_.clear();
  cross_obs_.clear();
  front_follow_adc_lane_obs_.clear();
  neighbor_front_merge_obs_.clear();
  neighbor_back_merge_obs_.clear();

  cipv_ = {"", std::numeric_limits<double>::infinity()};
  nearest_front_cutin_obs_ = {"", std::numeric_limits<double>::infinity()};
  nearest_blocking_obs_ = {"", std::numeric_limits<double>::infinity()};

  adc_in_dying_road_ = false;
  adc_distance_to_merging_end_s_ = std::numeric_limits<double>::infinity();

  obstacles_behavior_info_.clear();
  interactive_obs_ = {};
  adc_leading_obs_id_.clear();

  alongside_obs_perception_ids_.clear();
  cutin_obs_perception_ids_.clear();
}

void ObstaclesInfo::ObsInfoDebug() {
  ADEBUG << "==========";
  ADEBUG << "adc_front_valid_obstacles_:";
  for (const auto& obs_id : adc_front_valid_obstacles_) {
    ADEBUG << obs_id;
  }
  ADEBUG << "cipv_id_=" << cipv_.first;
  ADEBUG << "nearest_blocking_obs_=" << nearest_blocking_obs_.first;
  ADEBUG << "front_cut_in_obs_:";
  for (const auto& obs_id : front_cut_in_obs_) {
    ADEBUG << obs_id;
  }
  ADEBUG << "nearest_front_cutin_obs_=" << nearest_front_cutin_obs_.first;
  ADEBUG << "cross_obs_:";
  for (const auto& obs_id : cross_obs_) {
    ADEBUG << obs_id;
  }
  ADEBUG << "alongside_obs_:";
  for (const auto& obs_id : alongside_obs_perception_ids_) {
    ADEBUG << obs_id;
  }
  ADEBUG << std::boolalpha << "adc_in_dying_road_=" << adc_in_dying_road_;
  if (adc_in_dying_road_) {
    ADEBUG << "adc_distance_to_merging_end_s_="
           << adc_distance_to_merging_end_s_;
    ADEBUG << "neighbor_front_merge_obs_:";
    for (const auto& obs_id : neighbor_front_merge_obs_) {
      ADEBUG << obs_id.first;
    }
    ADEBUG << "neighbor_back_merge_obs_:";
    for (const auto& obs_id : neighbor_back_merge_obs_) {
      ADEBUG << obs_id.first;
    }

    if (!interactive_obs_[0].first.empty() ||
        !interactive_obs_[1].first.empty() ||
        !interactive_obs_[2].first.empty()) {
      ADEBUG << "adc_leading_obs_id_=" << adc_leading_obs_id_;

      ADEBUG << "nearest_obs_out_of_merge_point_id_="
             << interactive_obs_[0].second;
      ADEBUG << "nearest_obs_to_merge_point_id_=" << interactive_obs_[0].first;
      ADEBUG << "nearest_front_obs_leading_id_=" << interactive_obs_[1].second;
      ADEBUG << "nearest_front_obs_id_=" << interactive_obs_[1].first;
      ADEBUG << "nearest_back_obs_leading_id_=" << interactive_obs_[2].second;
      ADEBUG << "nearest_back_obs_id_=" << interactive_obs_[2].first;
    }
  }
}

}  // namespace planning
}  // namespace TL
