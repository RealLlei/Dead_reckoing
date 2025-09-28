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

/**
 * @file
 **/

#include "planning/common/reference_line_info.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <ios>

#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/file/log.h"
#include "common/math/math_utils.h"
#include "common/time/clock.h"
#include "common/util/point_factory.h"
#include "common/util/util.h"
#include "map/hdmap/hdmap_common.h"
#include "map/hdmap/hdmap_util.h"
#include "map/hdmap/path.h"
#include "planning/common/path_boundary.h"
#include "planning/common/planning_gflags.h"

#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/planning/planning_status.pb.h"
#include "proto/planning/sl_boundary.pb.h"
#include "proto/prediction/feature.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {

// if lateral length of adc head near the current lane boundary > lane_merge_finish_l_threshold,
// this means adc start lane merge
// constexpr double kLaneMergeStartLThreshold = 0.1;
// if lateral length of adc head entering the neighbor lane > lane_merge_finish_l_threshold,
// this means adc finish lane merge
// constexpr double kLaneMergeFinishLThreshold = 0.5;

using TL::common::EngageAdvice;
using TL::common::VehicleConfigHelper;
using TL::common::VehicleSignal;
using TL::common::math::Box2d;
using TL::common::math::Vec2d;
using TL::common::util::PointFactory;
using TL::soc::Chassis;

std::unordered_map<std::string, bool>
    ReferenceLineInfo::junction_right_of_way_map_;

ReferenceLineInfo::ReferenceLineInfo(
    const std::shared_ptr<common::VehicleState>& vehicle_state_ptr,
    common::TrajectoryPoint adc_planning_point,
    const std::shared_ptr<ReferenceLine>& reference_line,
    hdmap::RouteSegments segments)
    : vehicle_state_ptr_(vehicle_state_ptr),
      adc_planning_point_(std::move(adc_planning_point)),
      reference_line_(reference_line),
      lanes_(std::move(segments)) {
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  max_acceleration_ = veh_param.max_acceleration();
  max_deceleration_ = veh_param.max_deceleration();
}

bool ReferenceLineInfo::Init(
    const std::vector<const std::shared_ptr<Obstacle>*>& obstacles,
    const TL::functionmanager::FunctionManagerIn& fct_manager_input) {
  const auto& fct_input = fct_manager_input.fct_nnp_in();
  const auto& param = VehicleConfigHelper::GetConfig().vehicle_param();
  // stitching point
  const auto& path_point = adc_planning_point_.path_point();
  Vec2d position(path_point.x(), path_point.y());
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);
  Vec2d center(position + vec_to_center.rotate(path_point.theta()));
  Box2d box(center, path_point.theta(), param.length(), param.width());
  // realtime vehicle position
  Vec2d vehicle_position(vehicle_state_ptr_->x(), vehicle_state_ptr_->y());
  Vec2d vehicle_center(vehicle_position +
                       vec_to_center.rotate(vehicle_state_ptr_->heading()));
  Box2d vehicle_box(vehicle_center, vehicle_state_ptr_->heading(),
                    param.length(), param.width());

  if (!reference_line_->GetSLBoundary(box, &adc_sl_boundary_)) {
    AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
    return false;
  }

  if (!reference_line_->XYToSL(position, &adc_sl_point_)) {
    AERROR << "Failed to get ADC SLPoint from position";
    return false;
  }

  InitFirstOverlaps();

  if (adc_sl_boundary_.end_s() < 0 ||
      adc_sl_boundary_.start_s() > reference_line_->Length()) {
    AWARN << "Vehicle SL " << adc_sl_boundary_.ShortDebugString()
          << " is not on reference line:[0, " << reference_line_->Length()
          << "]";
  }
  const double kOutOfReferenceLineL =
      (fct_manager_input.has_ta_pilot_mode() &&
       fct_manager_input.ta_pilot_mode() == functionmanager::TaPilotMode::AVP)
          ? 30.0
          : 10.0;  // in meters
  if (adc_sl_boundary_.start_l() > kOutOfReferenceLineL ||
      adc_sl_boundary_.end_l() < -kOutOfReferenceLineL) {
    AERROR << "adc_start_l:" << adc_sl_boundary_.start_l()
           << "   adc_end_l:" << adc_sl_boundary_.end_l()
           << "   threshold:" << kOutOfReferenceLineL
           << "   Ego vehicle is too far away from reference line.";
    return false;
  }
  is_on_reference_line_ = reference_line_->IsOnLane(adc_sl_boundary_);
  if (!AddObstacles(obstacles)) {
    AERROR << "Failed to add obstacles to reference line";
    return false;
  }

  const auto& map_path = reference_line_->map_path();
  for (const auto& speed_bump : map_path.speed_bump_overlaps()) {
    // -1 and + 1.0 are added to make sure it can be sampled.
    reference_line_->AddSpeedLimit(speed_bump.start_s - 1.0,
                                   speed_bump.end_s + 1.0,
                                   FLAGS_speed_bump_speed_limit);
  }
  if (fct_input.has_longitud_ctrl_cruise_speedms()) {
    SetCruiseSpeed(fct_input.longitud_ctrl_cruise_speedms());
    SetLatticeCruiseSpeed(fct_input.longitud_ctrl_cruise_speedms());
  } else {
    SetCruiseSpeed(FLAGS_planning_upper_speed_limit);
    SetLatticeCruiseSpeed(FLAGS_planning_upper_speed_limit);
  }
  if (fct_input.has_longitud_ctrl_time()) {
    SetLonCtrlTime(fct_input.longitud_ctrl_time());
  } else {
    SetLonCtrlTime(1.5);
  }
  if (fct_input.has_longitud_ctrl_min_dis()) {
    SetLonCtrlMinDis(fct_input.longitud_ctrl_min_dis());
  } else {
    SetLonCtrlMinDis(8.0);
  }

  // set lattice planning target speed limit;

  vehicle_signal_.Clear();

  return true;
}

bool ReferenceLineInfo::GetProjection() {
  const auto& param = VehicleConfigHelper::GetConfig().vehicle_param();
  // stitching point
  const auto& path_point = adc_planning_point_.path_point();
  Vec2d position(path_point.x(), path_point.y());
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);
  Vec2d center(position + vec_to_center.rotate(path_point.theta()));
  Box2d box(center, path_point.theta(), param.length(), param.width());
  // realtime vehicle position
  Vec2d vehicle_position(vehicle_state_ptr_->x(), vehicle_state_ptr_->y());

  adc_sl_boundary_.clear_boundary_point();
  if (!reference_line_->GetSLBoundary(box, &adc_sl_boundary_)) {
    AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
    return false;
  }
  if (!reference_line_->XYToSL(position, &adc_sl_point_)) {
    AERROR << "Failed to get ADC SLPoint from position";
    return false;
  }

  first_encounter_overlaps_.clear();
  InitFirstOverlaps();

  if (adc_sl_boundary_.end_s() < 0 ||
      adc_sl_boundary_.start_s() > reference_line_->Length()) {
    AWARN << "Vehicle SL " << adc_sl_boundary_.ShortDebugString()
          << " is not on reference line:[0, " << reference_line_->Length()
          << "]";
  }

  static constexpr double kOutOfReferenceLineL = 10.0;  // in meters
  if (adc_sl_boundary_.start_l() > kOutOfReferenceLineL ||
      adc_sl_boundary_.end_l() < -kOutOfReferenceLineL) {
    AERROR << "adc_start_l:" << adc_sl_boundary_.start_l()
           << "   adc_end_l:" << adc_sl_boundary_.end_l()
           << "   threshold:" << kOutOfReferenceLineL
           << "   Ego vehicle is too far away from reference line.";
    return false;
  }

  is_on_reference_line_ = reference_line_->IsOnLane(adc_sl_boundary_);

  const auto& obstacles = path_decision_.obstacles().Items();
  if (FLAGS_use_multi_thread_to_add_obstacles) {
    common::sub_thread_name = "_planning";
    auto lamAddObstacle = [&](const auto* obstacle) {
      auto* mutable_obstacle = path_decision_.Find(obstacle->Id());
      if (mutable_obstacle == nullptr ||
          !GetObstacleProjection(mutable_obstacle)) {
        AERROR << "Failed to add obstacle " << obstacle->Id();
      }
    };
    TL::common::thread::ThreadPool::Instance()->ForEach(
        obstacles.begin(), obstacles.end(), lamAddObstacle);
  } else {
    for (const auto* obstacle : obstacles) {
      if (obstacle == nullptr) {
        continue;
      }
      auto* mutable_obstacle = path_decision_.Find(obstacle->Id());
      if (mutable_obstacle == nullptr ||
          !GetObstacleProjection(mutable_obstacle)) {
        AERROR << "Failed to get obstacle projection" << obstacle->Id();
      }
    }
  }

  return true;
}

const std::vector<PathData>& ReferenceLineInfo::GetCandidatePathData() const {
  return candidate_path_data_;
}

void ReferenceLineInfo::SetCandidatePathData(
    std::vector<PathData>&& candidate_path_data) {
  candidate_path_data_ = std::move(candidate_path_data);
}

const std::vector<PathBoundary>& ReferenceLineInfo::GetCandidatePathBoundaries()
    const {
  return candidate_path_boundaries_;
}

void ReferenceLineInfo::SetCandidatePathBoundaries(
    std::vector<PathBoundary>&& path_boundaries) {
  candidate_path_boundaries_ = std::move(path_boundaries);
}

hdmap::LaneInfoConstPtr ReferenceLineInfo::LocateLaneInfo(
    const double s) const {
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  reference_line_->GetLaneFromS(s, &lanes);
  if (lanes.empty()) {
    AWARN << "cannot get any lane using s";
    return nullptr;
  }

  return lanes.front();
}

bool ReferenceLineInfo::GetNeighborLaneInfo(
    const ReferenceLineInfo::LaneType lane_type, const double s,
    hdmap::Id* ptr_lane_id, double* ptr_lane_width) const {
  auto ptr_lane_info = LocateLaneInfo(s);
  if (ptr_lane_info == nullptr) {
    return false;
  }

  switch (lane_type) {
    case LaneType::LeftForward: {
      if (ptr_lane_info->lane().left_neighbor_forward_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().left_neighbor_forward_lane_id(0);
      break;
    }
    case LaneType::LeftReverse: {
      if (ptr_lane_info->lane().left_neighbor_reverse_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().left_neighbor_reverse_lane_id(0);
      break;
    }
    case LaneType::RightForward: {
      if (ptr_lane_info->lane().right_neighbor_forward_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().right_neighbor_forward_lane_id(0);
      break;
    }
    case LaneType::RightReverse: {
      if (ptr_lane_info->lane().right_neighbor_reverse_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().right_neighbor_reverse_lane_id(0);
      break;
    }
    default:
      ACHECK(false);
  }
  auto ptr_neighbor_lane =
      hdmap::HDMapUtil::MapForPlanning().GetLaneById(*ptr_lane_id);
  if (ptr_neighbor_lane == nullptr) {
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(s);

  double neighbor_s = 0.0;
  double neighbor_l = 0.0;
  if (!ptr_neighbor_lane->GetProjection({ref_point.x(), ref_point.y()},
                                        &neighbor_s, &neighbor_l)) {
    return false;
  }

  *ptr_lane_width = ptr_neighbor_lane->GetWidth(neighbor_s);
  return true;
}

hdmap::LaneInfoConstPtr ReferenceLineInfo::GetNeighborLaneInfo(
    ReferenceLineInfo::LaneType lane_type, double s) const {
  const auto ptr_lane_info = LocateLaneInfo(s);
  if (ptr_lane_info == nullptr) {
    return nullptr;
  }

  switch (lane_type) {
    case LaneType::LeftForward:
      if (!ptr_lane_info->lane().left_neighbor_forward_lane_id().empty()) {
        return hdmap::HDMapUtil::MapForPlanning().GetLaneById(
            ptr_lane_info->lane().left_neighbor_forward_lane_id(0));
      }
      break;
    case LaneType::LeftReverse:
      if (!ptr_lane_info->lane().left_neighbor_reverse_lane_id().empty()) {
        return hdmap::HDMapUtil::MapForPlanning().GetLaneById(
            ptr_lane_info->lane().left_neighbor_reverse_lane_id(0));
      }
      break;
    case LaneType::RightForward:
      if (!ptr_lane_info->lane().right_neighbor_forward_lane_id().empty()) {
        return hdmap::HDMapUtil::MapForPlanning().GetLaneById(
            ptr_lane_info->lane().right_neighbor_forward_lane_id(0));
      }
      break;
    case LaneType::RightReverse:
      if (!ptr_lane_info->lane().right_neighbor_reverse_lane_id().empty()) {
        return hdmap::HDMapUtil::MapForPlanning().GetLaneById(
            ptr_lane_info->lane().right_neighbor_reverse_lane_id(0));
      }
      break;
    default:
      break;
  }
  return nullptr;
}

bool ReferenceLineInfo::GetFirstOverlap(
    const std::vector<hdmap::PathOverlap>& path_overlaps,
    hdmap::PathOverlap* path_overlap) {
  CHECK_NOTNULL(path_overlap);
  const double start_s = adc_sl_boundary_.end_s();
  static constexpr double kMaxOverlapRange = 500.0;
  double overlap_min_s = kMaxOverlapRange;

  auto overlap_min_s_iter = path_overlaps.end();
  for (auto iter = path_overlaps.begin(); iter != path_overlaps.end(); ++iter) {
    if (iter->end_s < start_s) {
      continue;
    }
    if (overlap_min_s > iter->start_s) {
      overlap_min_s_iter = iter;
      overlap_min_s = iter->start_s;
    }
  }

  // Ensure that the path_overlaps is not empty.
  if (overlap_min_s_iter != path_overlaps.end()) {
    *path_overlap = *overlap_min_s_iter;
  }

  return overlap_min_s < kMaxOverlapRange;
}

void ReferenceLineInfo::InitFirstOverlaps() {
  const auto& map_path = reference_line_->map_path();
  // clear_zone
  hdmap::PathOverlap clear_area_overlap;
  if (GetFirstOverlap(map_path.clear_area_overlaps(), &clear_area_overlap)) {
    first_encounter_overlaps_.emplace_back(CLEAR_AREA, clear_area_overlap);
  }

  // crosswalk
  hdmap::PathOverlap crosswalk_overlap;
  if (GetFirstOverlap(map_path.crosswalk_overlaps(), &crosswalk_overlap)) {
    first_encounter_overlaps_.emplace_back(CROSSWALK, crosswalk_overlap);
  }

  // pnc_junction
  hdmap::PathOverlap pnc_junction_overlap;
  if (GetFirstOverlap(map_path.pnc_junction_overlaps(),
                      &pnc_junction_overlap)) {
    first_encounter_overlaps_.emplace_back(PNC_JUNCTION, pnc_junction_overlap);
  }

  // signal
  hdmap::PathOverlap signal_overlap;
  if (GetFirstOverlap(map_path.signal_overlaps(), &signal_overlap)) {
    first_encounter_overlaps_.emplace_back(SIGNAL, signal_overlap);
  }

  // stop_sign
  hdmap::PathOverlap stop_sign_overlap;
  if (GetFirstOverlap(map_path.stop_sign_overlaps(), &stop_sign_overlap)) {
    first_encounter_overlaps_.emplace_back(STOP_SIGN, stop_sign_overlap);
  }

  // yield_sign
  hdmap::PathOverlap yield_sign_overlap;
  if (GetFirstOverlap(map_path.yield_sign_overlaps(), &yield_sign_overlap)) {
    first_encounter_overlaps_.emplace_back(YIELD_SIGN, yield_sign_overlap);
  }

  // sort by start_s
  if (!first_encounter_overlaps_.empty()) {
    std::sort(first_encounter_overlaps_.begin(),
              first_encounter_overlaps_.end(),
              [](const std::pair<OverlapType, hdmap::PathOverlap>& a,
                 const std::pair<OverlapType, hdmap::PathOverlap>& b) {
                return a.second.start_s < b.second.start_s;
              });
  }
}

bool WithinOverlap(const hdmap::PathOverlap& overlap, double s) {
  static constexpr double kEpsilon = 1e-2;
  return overlap.start_s - kEpsilon <= s && s <= overlap.end_s + kEpsilon;
}

void ReferenceLineInfo::SetJunctionRightOfWay(const double junction_s,
                                              const bool is_protected) const {
  for (const auto& overlap : reference_line_->map_path().junction_overlaps()) {
    if (WithinOverlap(overlap, junction_s)) {
      junction_right_of_way_map_[overlap.object_id] = is_protected;
    }
  }
}

ADCTrajectory::RightOfWayStatus ReferenceLineInfo::GetRightOfWayStatus() const {
  for (const auto& overlap : reference_line_->map_path().junction_overlaps()) {
    if (overlap.end_s < adc_sl_boundary_.start_s()) {
      junction_right_of_way_map_.erase(overlap.object_id);
    } else if (WithinOverlap(overlap, adc_sl_boundary_.end_s())) {
      auto is_protected = junction_right_of_way_map_[overlap.object_id];
      if (is_protected) {
        return ADCTrajectory::PROTECTED;
      }
    }
  }
  return ADCTrajectory::UNPROTECTED;
}

const hdmap::RouteSegments& ReferenceLineInfo::Lanes() const {
  return lanes_;
}

hdmap::RouteSegments* ReferenceLineInfo::GetMutableLanes() {
  return &lanes_;
}

std::list<hdmap::Id> ReferenceLineInfo::TargetLaneId() const {
  std::list<hdmap::Id> lane_ids;
  for (const auto& lane_seg : lanes_) {
    lane_ids.push_back(lane_seg.lane->id());
  }
  return lane_ids;
}

PathDecision* ReferenceLineInfo::path_decision() {
  return &path_decision_;
}

const PathDecision& ReferenceLineInfo::path_decision() const {
  return path_decision_;
}

const ReferenceLine& ReferenceLineInfo::reference_line() const {
  return *reference_line_;
}

std::shared_ptr<ReferenceLine> ReferenceLineInfo::mutable_reference_line() {
  return reference_line_;
}

void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory& trajectory) {
  discretized_trajectory_ = trajectory;
}

bool ReferenceLineInfo::AddObstacleHelper(
    const std::shared_ptr<Obstacle>& obstacle) {
  return AddObstacle(obstacle) != nullptr;
}

// AddObstacle is thread safe
bool ReferenceLineInfo::GetObstacleProjection(
    Obstacle* const mutable_obstacle) {
  if (mutable_obstacle == nullptr) {
    AERROR << "failed to add obstacle";
    return false;
  }

  SLBoundary perception_sl;
  if (!reference_line_->GetSLBoundary(mutable_obstacle->PerceptionBoundingBox(),
                                      &perception_sl)) {
    AERROR << "Failed to get sl boundary for obstacle: "
           << mutable_obstacle->Id();
    return false;
  }
  mutable_obstacle->SetPerceptionSlBoundary(perception_sl);
  mutable_obstacle->CheckLaneBlocking(reference_line_);
  if (mutable_obstacle->IsLaneBlocking()) {
    ADEBUG << "obstacle [" << mutable_obstacle->Id() << "] is lane blocking.";
  } else {
    ADEBUG << "obstacle [" << mutable_obstacle->Id()
           << "] is NOT lane blocking.";
  }

  {
    // lp: system rotate theta
    auto SystemRotate = [](double x, double y, double theta) {
      return Vec2d{x * cos(theta) - y * sin(theta),
                   x * sin(theta) + y * cos(theta)};
    };

    double half_length =
        mutable_obstacle->PerceptionBoundingBox().length() / 2.0;
    double half_width = mutable_obstacle->PerceptionBoundingBox().width() / 2.0;
    double theta = mutable_obstacle->PerceptionBoundingBox().heading();
    Vec2d vpoint;
    SLPoint sl_p;
    Vec2d center_p(mutable_obstacle->PerceptionBoundingBox().center_x(),
                   mutable_obstacle->PerceptionBoundingBox().center_y());
    std::vector<ObsPointDescription> trajectory_envelope;

    auto SetSLExtreme = [](const ObsPointDescription& obs_point,
                           Obstacle* const obstacle) {
      double s_min = fmin(obs_point.low_left_p.s(), obs_point.low_right_p.s());
      s_min = fmin(obs_point.upper_left_p.s(), s_min);
      s_min = fmin(obs_point.upper_right_p.s(), s_min);

      double s_max = fmax(obs_point.low_left_p.s(), obs_point.low_right_p.s());
      s_max = fmax(obs_point.upper_left_p.s(), s_min);
      s_max = fmax(obs_point.upper_right_p.s(), s_min);

      double l_max = fmax(obs_point.low_left_p.l(), obs_point.low_right_p.l());
      l_max = fmax(obs_point.upper_left_p.l(), l_max);
      l_max = fmax(obs_point.upper_right_p.l(), l_max);

      double l_min = fmin(obs_point.low_left_p.l(), obs_point.low_right_p.l());
      l_min = fmin(obs_point.upper_left_p.l(), l_min);
      l_min = fmin(obs_point.upper_right_p.l(), l_min);

      obstacle->SetTrajMaxS(fmax(s_max, obstacle->GetTrajMaxS()));
      obstacle->SetTrajMinS(fmin(s_min, obstacle->GetTrajMinS()));
      obstacle->SetTrajMaxL(fmax(l_max, obstacle->GetTrajMaxL()));
      obstacle->SetTrajMinL(fmin(l_min, obstacle->GetTrajMinL()));
    };

    // lp: static obstacle project
    if (mutable_obstacle->IsStatic()) {
      ObsPointDescription obs_point;
      const auto nums = static_cast<uint>(
          FLAGS_trajectory_time_length / FLAGS_speed_planning_delta_time + 1);

      int index_s = 0;
      vpoint = SystemRotate(-half_length, -half_width, theta);
      vpoint += center_p;
      reference_line().XYToSL(vpoint, &sl_p, &index_s);
      obs_point.low_left_p.CopyFrom(sl_p);

      vpoint = SystemRotate(half_length, -half_width, theta);
      vpoint += center_p;
      reference_line().XYToSL(vpoint, &sl_p);
      obs_point.low_right_p.CopyFrom(sl_p);

      vpoint = SystemRotate(-half_length, half_width, theta);
      vpoint += center_p;
      reference_line().XYToSL(vpoint, &sl_p);
      obs_point.upper_left_p.CopyFrom(sl_p);

      vpoint = SystemRotate(half_length, half_width, theta);
      vpoint += center_p;
      reference_line().XYToSL(vpoint, &sl_p);
      obs_point.upper_right_p.CopyFrom(sl_p);

      obs_point.center_p.set_s(
          (obs_point.low_left_p.s() + obs_point.low_right_p.s() +
           obs_point.upper_left_p.s() + obs_point.upper_right_p.s()) /
          4);
      obs_point.center_p.set_l(
          (obs_point.low_left_p.l() + obs_point.low_right_p.l() +
           obs_point.upper_left_p.l() + obs_point.upper_right_p.l()) /
          4);
      trajectory_envelope.resize(nums, obs_point);
      for (size_t i = 0; i < nums; ++i) {
        trajectory_envelope[i].time =
            static_cast<double>(i) * FLAGS_speed_planning_delta_time;
      }
      SetSLExtreme(obs_point, mutable_obstacle);
    } else {
      // lp: dynamic obstacle project
      double radius1d = half_length * 2.0 * 2.0;

      trajectory_envelope.resize(
          mutable_obstacle->Trajectory().trajectory_point_size());
      // PERF_BLOCK_START();
      size_t counter_i = 0;
      for (const auto& traj :
           mutable_obstacle->Trajectory().trajectory_point()) {
        auto& obs_point = trajectory_envelope[counter_i++];
        obs_point.time = traj.relative_time() - FLAGS_perception_compensation;
        theta = traj.path_point().theta();
        center_p.set_x(traj.path_point().x());
        center_p.set_y(traj.path_point().y());

        int index_s = 0;
        vpoint = SystemRotate(-half_length, -half_width, theta);
        vpoint += center_p;

        reference_line().XYToSL(vpoint, &obs_point.low_left_p, &index_s);

        vpoint = SystemRotate(half_length, -half_width, theta);
        vpoint += center_p;
        reference_line().XYToSL(vpoint, &obs_point.low_right_p, nullptr,
                                radius1d, index_s);

        vpoint = SystemRotate(-half_length, half_width, theta);
        vpoint += center_p;
        reference_line().XYToSL(vpoint, &obs_point.upper_left_p, nullptr,
                                radius1d, index_s);

        vpoint = SystemRotate(half_length, half_width, theta);
        vpoint += center_p;
        reference_line().XYToSL(vpoint, &obs_point.upper_right_p, nullptr,
                                radius1d, index_s);

        obs_point.center_p.set_s(
            (obs_point.low_left_p.s() + obs_point.low_right_p.s() +
             obs_point.upper_left_p.s() + obs_point.upper_right_p.s()) /
            4);
        obs_point.center_p.set_l(
            (obs_point.low_left_p.l() + obs_point.low_right_p.l() +
             obs_point.upper_left_p.l() + obs_point.upper_right_p.l()) /
            4);
        SetSLExtreme(obs_point, mutable_obstacle);
        // if (obs_point.center_p.s() >
        //     vehicle_state_ptr_->linear_velocity() * FLAGS_trajectory_time_length +
        //         adc_sl_boundary_.start_s())
        //   break;
      }
    }
    mutable_obstacle->SetTrajectoryEnvelope(std::move(trajectory_envelope));
  }
  //   for (auto& point : mutable_obstacle->GetTrajectoryEnvelope()) {
  //     ADEBUG << "point time:" << point.time << "  min_l:"
  //            << fmin(fmin(point.low_left_p.l(), point.low_right_p.l()),
  //                    fmin(point.upper_left_p.l(), point.upper_right_p.l()))
  //            << "  max_l:"
  //            << fmax(fmax(point.low_left_p.l(), point.low_right_p.l()),
  //                    fmax(point.upper_left_p.l(), point.upper_right_p.l()))
  //            << "  ct_s:" << point.center_p.s() << "   l:" <<
  //            point.center_p.l()
  //            << "  ll_s:" << point.low_left_p.s()
  //            << "   l:" << point.low_left_p.l()
  //            << "  lr_s:" << point.low_right_p.s()
  //            << "   l:" << point.low_right_p.l()
  //            << "  rl_s:" << point.upper_left_p.s()
  //            << "   l:" << point.upper_left_p.l()
  //            << "  rr_s:" << point.upper_right_p.s()
  //            << "   l:" << point.upper_right_p.l();
  //   }

  if (IsIrrelevantObstacle(*mutable_obstacle)) {
    ObjectDecisionType ignore;
    ignore.mutable_ignore();
    mutable_obstacle->AddLateralDecision("reference_line_filter", ignore);
    mutable_obstacle->AddLongitudinalDecision("reference_line_filter", ignore);
    ADEBUG << "NO build reference line st boundary. id:"
           << mutable_obstacle->Id();
  } else {
    mutable_obstacle->EraseReferenceLineStBoundary();
    mutable_obstacle->BuildReferenceLineStBoundary(*reference_line_,
                                                   adc_sl_boundary_.start_s());

    ADEBUG << "build reference line st boundary. id:" << mutable_obstacle->Id()
           << " reference line st boundary: t["
           << mutable_obstacle->reference_line_st_boundary().min_t() << ", "
           << mutable_obstacle->reference_line_st_boundary().max_t() << "] s["
           << mutable_obstacle->reference_line_st_boundary().min_s() << ", "
           << mutable_obstacle->reference_line_st_boundary().max_s() << "]";
  }
  return true;
}

// AddObstacle is thread safe
Obstacle* ReferenceLineInfo::AddObstacle(
    const std::shared_ptr<Obstacle>& obstacle) {
  if (obstacle == nullptr) {
    AERROR << "The provided obstacle is nullptr";
    return nullptr;
  }
  Obstacle* mutable_obstacle = nullptr;
  {
    std::lock_guard<std::mutex> guard(lock_add_obs_);
    mutable_obstacle = path_decision_.AddEmptyObstacleById(obstacle->Id());
  }
  if (mutable_obstacle == nullptr) {
    AERROR << "failed to add obstacle " << obstacle->Id();
    return nullptr;
  }
  *mutable_obstacle = *obstacle;

  if (!GetObstacleProjection(mutable_obstacle)) {
    AERROR << "failed to get obstacle projection" << obstacle->Id();
  }
  return mutable_obstacle;
}

bool ReferenceLineInfo::AddObstacles(
    const std::vector<const std::shared_ptr<Obstacle>*>& obstacles) {
  if (FLAGS_use_multi_thread_to_add_obstacles) {
    common::sub_thread_name = "_planning";
    auto lamAddObstacle = [&](auto& obstacle) {
      return static_cast<bool>(AddObstacle(*obstacle));
    };
    TL::common::thread::ThreadPool::Instance()->ForEach(
        obstacles.begin(), obstacles.end(), lamAddObstacle);
  } else {
    for (const auto* obstacle : obstacles) {
      if (AddObstacle(*obstacle) == nullptr) {
        AERROR << "Failed to add obstacle " << (*obstacle)->Id();
        return false;
      }
    }
  }

  return true;
}

bool ReferenceLineInfo::IsIrrelevantObstacle(const Obstacle& obstacle) {
  if (obstacle.Priority() == prediction::ObstaclePriority::IGNORE) {
    ADEBUG << "Find ignore obstacle " << obstacle.Id();
    return true;
  }
  if (obstacle.IsCautionLevelObstacle()) {
    return false;
  }
  // 规控过滤感知提供的减速带bbox
  if (obstacle.Perception().has_sub_type() &&
      obstacle.Perception().sub_type() ==
          TL::perception::PerceptionObstacle::ST_SPEEDBUMP) {
    return true;
  }
  // if adc is on the road, and obstacle behind adc, ignore
  const auto& obstacle_boundary = obstacle.PerceptionSLBoundary();
  if (obstacle_boundary.start_s() > reference_line_->Length()) {
    ADEBUG << "obstacle boundary end_s: " << obstacle_boundary.end_s()
           << ", > ref lin length: " << reference_line_->Length();
    return true;
  }
  if (is_on_reference_line_ && !IsChangeLanePath() &&
      (obstacle_boundary.start_s() > adc_sl_boundary_.end_s() &&
       IsHistoryTrace()) &&
      (reference_line_->IsOnLane(obstacle_boundary) ||
       obstacle_boundary.end_s() < 0.0)) {  // if obstacle is far backward
    ADEBUG << "obstacle boundary end_s: " << obstacle_boundary.end_s()
           << ", < adc_sl_boundary_.end_s(): " << adc_sl_boundary_.end_s();
    return true;
  }
  return false;
}

const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return discretized_trajectory_;
}

void ReferenceLineInfo::SetLatticeStopPoint(const StopPoint& stop_point) {
  planning_target_.mutable_stop_point()->CopyFrom(stop_point);
}

void ReferenceLineInfo::SetLatticeCruiseSpeed(double speed) {
  planning_target_.set_cruise_speed(speed);
}

bool ReferenceLineInfo::IsStartFrom(
    const ReferenceLineInfo& previous_reference_line_info) const {
  if (reference_line_->reference_points().empty()) {
    return false;
  }
  auto start_point = reference_line_->reference_points().front();
  const auto& prev_reference_line =
      previous_reference_line_info.reference_line();
  common::SLPoint sl_point;
  prev_reference_line.XYToSL(start_point, &sl_point);
  return previous_reference_line_info.reference_line_->IsOnLane(sl_point);
}

const PathData& ReferenceLineInfo::path_data() const {
  return path_data_;
}

const PathData& ReferenceLineInfo::fallback_path_data() const {
  return fallback_path_data_;
}

const SpeedData& ReferenceLineInfo::speed_data() const {
  return speed_data_;
}

PathData* ReferenceLineInfo::mutable_path_data() {
  return &path_data_;
}

PathData* ReferenceLineInfo::mutable_fallback_path_data() {
  return &fallback_path_data_;
}

SpeedData* ReferenceLineInfo::mutable_speed_data() {
  return &speed_data_;
}

const RSSInfo& ReferenceLineInfo::rss_info() const {
  return rss_info_;
}

RSSInfo* ReferenceLineInfo::mutable_rss_info() {
  return &rss_info_;
}

bool ReferenceLineInfo::CombinePathAndSpeedProfile(
    const double relative_time, const double start_s,
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  ACHECK(ptr_discretized_trajectory != nullptr);
  // use varied resolution to reduce data load but also provide enough data
  // point for control module
  const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;
  const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;
  const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;
  const auto unit_t = 0.01;
  const auto total_size =
      static_cast<int>(std::round(speed_data_.TotalTime() / unit_t));

  const auto dense_size = static_cast<int>(std::round(kDenseTimeSec / unit_t));
  const auto dense_time_step =
      static_cast<int>(std::round(kDenseTimeResoltuion / unit_t));
  const auto spare_time_step =
      static_cast<int>(std::round(kSparseTimeResolution / unit_t));

  if (path_data_.discretized_path().empty()) {
    AERROR << "path data is empty";
    return false;
  }

  if (speed_data_.empty()) {
    AERROR << "speed profile is empty";
    return false;
  }
  for (int i = 0; i < total_size;
       i += (i < dense_size ? dense_time_step : spare_time_step)) {
    const auto cur_rel_time = i * unit_t;
    common::SpeedPoint speed_point;
    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_data_.discretized_path().Length()) {
      break;
    }
    common::PathPoint path_point =
        path_data_.GetPathPointWithPathS(speed_point.s());
    path_point.set_s(path_point.s() + start_s);

    const auto point_relative_time = speed_point.t() + relative_time;
    auto* trajectory_point =
        ptr_discretized_trajectory->AppendTrajectoryPoint(point_relative_time);
    if (trajectory_point == nullptr) {
      continue;
    }

    trajectory_point->mutable_path_point()->Swap(&path_point);
    trajectory_point->set_v(speed_point.v());
    trajectory_point->set_a(speed_point.a());
    trajectory_point->set_da(speed_point.da());
    trajectory_point->set_relative_time(point_relative_time);
  }
  return true;
}

void ReferenceLineInfo::SetDrivable(bool drivable) {
  is_drivable_ = drivable;
}

bool ReferenceLineInfo::IsDrivable() const {
  return is_drivable_;
}

bool ReferenceLineInfo::IsChangeLanePath() const {
  return !Lanes().IsOnSegment();
}

bool ReferenceLineInfo::IsNeighborLanePath() const {
  return Lanes().IsNeighborSegment();
}

std::string ReferenceLineInfo::PathSpeedDebugString() const {
  return absl::StrCat("path_data:", path_data_.DebugString(),
                      "speed_data:", speed_data_.DebugString());
}

void ReferenceLineInfo::SetTurnSignalBasedOnLaneTurnType(
    common::VehicleSignal* vehicle_signal) const {
  CHECK_NOTNULL(vehicle_signal);
  if (vehicle_signal->has_turn_signal() &&
      vehicle_signal->turn_signal() != VehicleSignal::TURN_NONE) {
    return;
  }
  vehicle_signal->set_turn_signal(VehicleSignal::TURN_NONE);

  // Set turn signal based on lane-change.
  if (IsChangeLanePath()) {
    if (Lanes().PreviousAction() == routing::ChangeLaneType::LEFT) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
    } else if (Lanes().PreviousAction() == routing::ChangeLaneType::RIGHT) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
    }
    return;
  }

  // Set turn signal based on lane-borrow.
  if (absl::StrContains(path_data_.path_label(), "left")) {
    vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
    return;
  }
  if (absl::StrContains(path_data_.path_label(), "right")) {
    vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
    return;
  }

  // Set turn signal based on lane's turn type.
  double route_s = 0.0;
  const double adc_s = adc_sl_boundary_.end_s();
  for (const auto& seg : Lanes()) {
    if (route_s > adc_s + FLAGS_turn_signal_distance) {
      break;
    }
    route_s += seg.end_s - seg.start_s;
    if (route_s < adc_s) {
      continue;
    }
    const auto& turn = seg.lane->lane().turn();
    if (turn == hdmap::Lane::LEFT_TURN) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
      break;
    }
    if (turn == hdmap::Lane::RIGHT_TURN) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
      break;
    }
    if (turn == hdmap::Lane::U_TURN) {
      // check left or right by geometry.
      auto start_xy =
          PointFactory::ToVec2d(seg.lane->GetSmoothPoint(seg.start_s));
      auto middle_xy = PointFactory::ToVec2d(
          seg.lane->GetSmoothPoint((seg.start_s + seg.end_s) / 2.0));
      auto end_xy = PointFactory::ToVec2d(seg.lane->GetSmoothPoint(seg.end_s));
      auto start_to_middle = middle_xy - start_xy;
      auto start_to_end = end_xy - start_xy;
      if (start_to_middle.CrossProd(start_to_end) < 0) {
        vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
      } else {
        vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
      }
      break;
    }
  }
}

void ReferenceLineInfo::SetTurnSignal(
    const VehicleSignal::TurnSignal& turn_signal) {
  vehicle_signal_.set_turn_signal(turn_signal);
}

void ReferenceLineInfo::SetEmergencyLight() {
  vehicle_signal_.set_emergency_light(true);
}

void ReferenceLineInfo::ExportVehicleSignal(
    common::VehicleSignal* vehicle_signal,
    const functionmanager::TaPilotMode ta_pilot_mode) const {
  if (vehicle_signal == nullptr || vehicle_state_ptr_ == nullptr) {
    return;
  }
  const auto driving_mode = vehicle_state_ptr_->driving_mode();
  if (ta_pilot_mode == functionmanager::ADAS &&
      (driving_mode == TL::soc::Chassis::COMPLETE_MANUAL ||
       driving_mode == TL::soc::Chassis::AUTO_SPEED_ONLY)) {
    return;
  }
  *vehicle_signal = vehicle_signal_;
  SetTurnSignalBasedOnLaneTurnType(vehicle_signal);
}

bool ReferenceLineInfo::ReachedDestination() const {
  static constexpr double kDestinationDeltaS = 0.05;
  const double distance_destination = SDistanceToDestination();
  return distance_destination <= kDestinationDeltaS;
}

double ReferenceLineInfo::SDistanceToDestination(
    const bool solution_through /*= false*/) const {
  double res = std::numeric_limits<double>::max();
  const auto* dest_ptr = path_decision_.Find(FLAGS_destination_obstacle_id);
  if (dest_ptr == nullptr) {
    return res;
  }
  if (!solution_through) {
    if (!dest_ptr->LongitudinalDecision().has_stop()) {
      return res;
    }
    if (!reference_line_->IsOnLane(
            dest_ptr->PerceptionBoundingBox().center())) {
      return res;
    }
    const double stop_s = dest_ptr->PerceptionSLBoundary().start_s() +
                          dest_ptr->LongitudinalDecision().stop().distance_s();
    return stop_s - adc_sl_boundary_.end_s();
  }
  if (!dest_ptr->PerceptionSLBoundary().has_start_s()) {
    return res;
  }
  double tmp_distance =
      dest_ptr->PerceptionSLBoundary().start_s() - adc_sl_boundary_.end_s();
  return tmp_distance < 0.0 ? res : tmp_distance;
}

void ReferenceLineInfo::ExportDecision(
    DecisionResult* decision_result, PlanningContext* planning_context,
    const functionmanager::TaPilotMode ta_pilot_mode) const {
  MakeDecision(decision_result, planning_context);
  ExportVehicleSignal(decision_result->mutable_vehicle_signal(), ta_pilot_mode);
  auto* main_decision = decision_result->mutable_main_decision();
  if (main_decision->has_stop()) {
    main_decision->mutable_stop()->set_change_lane_type(
        Lanes().PreviousAction());
  } else if (main_decision->has_cruise()) {
    main_decision->mutable_cruise()->set_change_lane_type(
        Lanes().PreviousAction());
  }
}

void ReferenceLineInfo::MakeDecision(DecisionResult* decision_result,
                                     PlanningContext* planning_context) const {
  CHECK_NOTNULL(decision_result);
  decision_result->Clear();

  // cruise by default
  decision_result->mutable_main_decision()->mutable_cruise();

  // check stop decision
  int error_code = MakeMainStopDecision(decision_result);
  if (error_code < 0) {
    MakeEStopDecision(decision_result);
  }
  MakeMainMissionCompleteDecision(decision_result, planning_context);
  SetObjectDecisions(decision_result->mutable_object_decision());
}

void ReferenceLineInfo::MakeMainMissionCompleteDecision(
    DecisionResult* decision_result, PlanningContext* planning_context) const {
  if (!decision_result->main_decision().has_stop()) {
    return;
  }
  auto main_stop = decision_result->main_decision().stop();
  if (main_stop.reason_code() != STOP_REASON_DESTINATION &&
      main_stop.reason_code() != STOP_REASON_PULL_OVER) {
    return;
  }
  const auto& fsm_state =
      planning_context->planning_status().function_manager_out().fsm_state();
  if (fsm_state == functionmanager::MachineStateType::APA_TYPE ||
      fsm_state == functionmanager::MachineStateType::HISTORY_TRACE_TYPE ||
      fsm_state == functionmanager::MachineStateType::HDMAP_AVP_TYPE) {
    bool isParkingFinished =
        planning_context->planning_status()
            .function_manager_out()
            .avp_fct_out()
            .parking_status() == functionmanager::AvpFctOut::MISSIONFINISHED;

    if (isParkingFinished) {
      decision_result->mutable_main_decision()->mutable_mission_complete();
      ADEBUG << "avp mission complete";
    }
  } else {
    const auto& adc_pos = adc_planning_point_.path_point();
    if (common::util::DistanceXY(adc_pos, main_stop.stop_point()) >
        FLAGS_destination_check_distance) {
      return;
    }

    auto* mission_complete =
        decision_result->mutable_main_decision()->mutable_mission_complete();
    if (ReachedDestination()) {
      planning_context->mutable_planning_status()
          ->mutable_destination()
          ->set_has_passed_destination(true);
    } else {
      mission_complete->mutable_stop_point()->CopyFrom(main_stop.stop_point());
      mission_complete->set_stop_heading(main_stop.stop_heading());
    }
  }
}

int ReferenceLineInfo::MakeMainStopDecision(
    DecisionResult* decision_result) const {
  double min_stop_line_s = std::numeric_limits<double>::infinity();
  const Obstacle* stop_obstacle = nullptr;
  const ObjectStop* stop_decision = nullptr;

  for (const auto* obstacle : path_decision_.obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }
    const auto& object_decision = obstacle->LongitudinalDecision();
    if (!object_decision.has_stop()) {
      continue;
    }

    TL::common::PointENU stop_point = object_decision.stop().stop_point();
    common::SLPoint stop_line_sl;
    reference_line_->XYToSL(stop_point, &stop_line_sl);

    double stop_line_s = stop_line_sl.s();
    if (stop_line_s < 0 || stop_line_s > reference_line_->Length()) {
      AERROR << "Ignore object:" << obstacle->Id() << " fence route_s["
             << stop_line_s << "] not in range[0, " << reference_line_->Length()
             << "]";
      continue;
    }

    // check stop_line_s vs adc_s
    if (stop_line_s < min_stop_line_s) {
      min_stop_line_s = stop_line_s;
      stop_obstacle = obstacle;
      stop_decision = &(object_decision.stop());
    }
  }

  if (stop_obstacle != nullptr) {
    MainStop* main_stop =
        decision_result->mutable_main_decision()->mutable_stop();
    main_stop->set_reason_code(stop_decision->reason_code());
    main_stop->set_reason("stop by " + stop_obstacle->Id());
    main_stop->mutable_stop_point()->set_x(stop_decision->stop_point().x());
    main_stop->mutable_stop_point()->set_y(stop_decision->stop_point().y());
    main_stop->set_stop_heading(stop_decision->stop_heading());

    ADEBUG << " main stop obstacle id:" << stop_obstacle->Id()
           << " stop_line_s:" << min_stop_line_s << " stop_point: ("
           << stop_decision->stop_point().x() << stop_decision->stop_point().y()
           << " ) stop_heading: " << stop_decision->stop_heading();

    return 1;
  }

  return 0;
}

void ReferenceLineInfo::SetObjectDecisions(
    ObjectDecisions* object_decisions) const {
  for (const auto* const obstacle : path_decision_.obstacles().Items()) {
    if (obstacle == nullptr || !obstacle->HasNonIgnoreDecision()) {
      continue;
    }
    auto* object_decision = object_decisions->add_decision();

    object_decision->set_id(obstacle->Id());
    object_decision->set_perception_id(obstacle->PerceptionId());
    if (obstacle->HasLateralDecision() && !obstacle->IsLateralIgnore()) {
      object_decision->add_object_decision()->CopyFrom(
          obstacle->LateralDecision());
    }
    if (obstacle->HasLongitudinalDecision() &&
        !obstacle->IsLongitudinalIgnore()) {
      object_decision->add_object_decision()->CopyFrom(
          obstacle->LongitudinalDecision());
    }
  }
}

void ReferenceLineInfo::ExportEngageAdvice(
    EngageAdvice* engage_advice, PlanningContext* planning_context) const {
  static EngageAdvice prev_advice;
  static constexpr double kMaxAngleDiff = M_PI / 6.0;

  bool engage = false;
  if (!IsDrivable()) {
    prev_advice.set_reason("Reference line not drivable");
  } else if (!is_on_reference_line_) {
    const auto& scenario_type =
        planning_context->planning_status().scenario().scenario_type();
    if (scenario_type == ScenarioStatus::PARK_AND_GO || IsChangeLanePath()) {
      // note: when is_on_reference_line_ is FALSE
      //   (1) always engage while in PARK_AND_GO scenario
      //   (2) engage when "ChangeLanePath" is picked as Drivable ref line
      //       where most likely ADC not OnLane yet
      engage = true;
    } else {
      prev_advice.set_reason("Not on reference line");
    }
  } else {
    // check heading
    auto ref_point =
        reference_line_->GetReferencePoint(adc_sl_boundary_.end_s());
    if (common::math::AngleDiff(vehicle_state_ptr_->heading(),
                                ref_point.heading()) < kMaxAngleDiff) {
      engage = true;
    } else {
      prev_advice.set_reason("Vehicle heading is not aligned");
    }
  }

  if (engage) {
    if (vehicle_state_ptr_->driving_mode() !=
        Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
      // READY_TO_ENGAGE when in non-AUTO mode
      prev_advice.set_advice(EngageAdvice::READY_TO_ENGAGE);
    } else {
      // KEEP_ENGAGED when in AUTO mode
      prev_advice.set_advice(EngageAdvice::KEEP_ENGAGED);
    }
    prev_advice.clear_reason();
  } else {
    if (prev_advice.advice() != EngageAdvice::DISALLOW_ENGAGE) {
      prev_advice.set_advice(EngageAdvice::PREPARE_DISENGAGE);
    }
  }
  engage_advice->CopyFrom(prev_advice);
}

void ReferenceLineInfo::MakeEStopDecision(
    DecisionResult* decision_result) const {
  decision_result->Clear();

  MainEmergencyStop* main_estop =
      decision_result->mutable_main_decision()->mutable_estop();
  main_estop->set_reason_code(MainEmergencyStop::ESTOP_REASON_INTERNAL_ERR);
  main_estop->set_reason("estop reason to be added");
  main_estop->mutable_cruise_to_stop();

  // set object decisions
  ObjectDecisions* object_decisions =
      decision_result->mutable_object_decision();
  for (const auto* const obstacle : path_decision_.obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }
    auto* object_decision = object_decisions->add_decision();
    object_decision->set_id(obstacle->Id());
    object_decision->set_perception_id(obstacle->PerceptionId());
    object_decision->add_object_decision()->mutable_avoid();
  }
}

hdmap::Lane::LaneTurn ReferenceLineInfo::GetPathTurnType(const double s) const {
  const double forward_buffer = 30.0;
  double route_s = 0.0;
  for (const auto& seg : Lanes()) {
    if (route_s > s + forward_buffer) {
      break;
    }
    route_s += seg.end_s - seg.start_s;
    if (route_s < s) {
      continue;
    }
    const auto& turn_type = seg.lane->lane().turn();
    if (turn_type == hdmap::Lane::LEFT_TURN ||
        turn_type == hdmap::Lane::RIGHT_TURN ||
        turn_type == hdmap::Lane::U_TURN) {
      return turn_type;
    }
  }

  return hdmap::Lane::NO_TURN;
}

bool ReferenceLineInfo::GetIntersectionRightofWayStatus(
    const hdmap::PathOverlap& pnc_junction_overlap) const {
  // if (GetPathTurnType(pnc_junction_overlap.start_s) != hdmap::Lane::NO_TURN) {
  //   return false;
  // }

  // // TODO(all): iterate exits of intersection to check/compare speed-limit
  // return true;
  return GetPathTurnType(pnc_junction_overlap.start_s) == hdmap::Lane::NO_TURN;
}

int ReferenceLineInfo::GetPnCJunction(
    const double s, hdmap::PathOverlap* pnc_junction_overlap) const {
  CHECK_NOTNULL(pnc_junction_overlap);
  const std::vector<hdmap::PathOverlap>& pnc_junction_overlaps =
      reference_line_->map_path().pnc_junction_overlaps();

  static constexpr double kError = 1.0;  // meter
  for (const auto& overlap : pnc_junction_overlaps) {
    if (s >= overlap.start_s - kError && s <= overlap.end_s + kError) {
      *pnc_junction_overlap = overlap;
      return 1;
    }
  }
  return 0;
}

void ReferenceLineInfo::SetBlockingObstacle(
    const std::string& blocking_obstacle_id) {
  blocking_obstacle_ = path_decision_.Find(blocking_obstacle_id);
}

std::vector<common::SLPoint> ReferenceLineInfo::GetAllStopDecisionSLPoint()
    const {
  std::vector<common::SLPoint> result;
  for (const auto* obstacle : path_decision_.obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }
    const auto& object_decision = obstacle->LongitudinalDecision();
    if (!object_decision.has_stop()) {
      continue;
    }
    TL::common::PointENU stop_point = object_decision.stop().stop_point();
    common::SLPoint stop_line_sl;
    reference_line_->XYToSL(stop_point, &stop_line_sl);
    if (stop_line_sl.s() <= 0 ||
        stop_line_sl.s() >= reference_line_->Length()) {
      continue;
    }
    result.push_back(stop_line_sl);
  }

  // sort by s
  if (!result.empty()) {
    std::sort(result.begin(), result.end(),
              [](const common::SLPoint& a, const common::SLPoint& b) {
                return a.s() < b.s();
              });
  }

  return result;
}

}  // namespace planning
}  // namespace TL
