/******************************************************************************
 * Copyright 2019 The TL Authors. All Rights Reserved.
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
#include "planning/tasks/optimizers/open_space_path_partition/open_space_path_partition.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "common/math/box2d.h"
#include "common/math/double_type.h"
#include "common/math/line_segment2d.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/time/clock.h"
#include "planning/common/frame.h"
#include "planning/common/open_space_info.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/planning_gflags.h"
#include "proto/common/error_code.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/perception_parking_lot.pb.h"
#include "proto/planning/planning_internal.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

using common::math::double_type::Compare;  // NOLINT
using common::math::double_type::DefinitelyGreaterEqual;
using common::math::double_type::DefinitelyLess;
using TL::common::Clock;  // NOLINT
using TL::common::ErrorCode;
using TL::common::PathPoint;
using TL::common::Status;
using TL::common::TrajectoryPoint;  // NOLINT
using TL::common::math::Box2d;      // NOLINT
using TL::common::math::NormalizeAngle;
using TL::common::math::Polygon2d;
using TL::common::math::Vec2d;

OpenSpacePathPartition::OpenSpacePathPartition(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : PathOptimizer(config, injector),
      open_space_path_partition_config_(
          config_.open_space_path_partition_config()),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
  ADEBUG << "OpenSpacePathPartition Init";
}

Status OpenSpacePathPartition::Reset() {
  history_path_ = PartitionedPath();
  is_warm_start_ = false;
  frozen_near_end_time_ = TL::common::Clock::NowInSeconds();
  frozen_time_ = TL::common::Clock::NowInSeconds();
  yaw_track_abnormal_start_time_ = TL::common::Clock::NowInSeconds();
  is_mirror_fold_ = false;
  AINFO << "Path Partition is reseted";
  return Status::OK();
}

void OpenSpacePathPartition::SetStopPath(
    const soc::Chassis::GearPosition& pub_gear,
    const PartitionedPath& reserved_partitioned_paths,
    PartitionedPath* const partitioned_paths_ptr) {
  std::vector<PathGearPair> rest_path_set;
  if (!reserved_partitioned_paths.path_set.empty()) {
    const auto path_idx = reserved_partitioned_paths.path_idx;
    rest_path_set.assign(reserved_partitioned_paths.path_set.begin() +
                             static_cast<int>(path_idx + 1),
                         reserved_partitioned_paths.path_set.end());
  }
  const auto last_gear = pub_gear == soc::Chassis::GEAR_NEUTRAL
                             ? soc::Chassis::GEAR_PARKING
                             : pub_gear;
  ResetPartitionPath(partitioned_paths_ptr);
  partitioned_paths_ptr->path_set.emplace_back();
  partitioned_paths_ptr->path_set.back().first.GenerateStopPath(
      start_point_.x(), start_point_.y(), start_point_.theta(),
      start_point_.kappa());
  partitioned_paths_ptr->path_set.back().second = last_gear;
  partitioned_paths_ptr->path_set.insert(partitioned_paths_ptr->path_set.end(),
                                         rest_path_set.begin(),
                                         rest_path_set.end());
}

Status OpenSpacePathPartition::Process() {
  auto* open_space_info_ptr = frame_->mutable_open_space_info();
  auto* partitioned_paths_ptr =
      open_space_info_ptr->mutable_partitioned_paths();
  auto* chosen_partitioned_path_ptr =
      open_space_info_ptr->mutable_chosen_partitioned_path();
  UpdateParam();
  path_decision_debug_.clear();
  OpenSpacePathDecision openspace_path_decision = OpenSpacePathDecision::UNKOWN;
  UpdatePathDecision(&openspace_path_decision, partitioned_paths_ptr);
  if (task_finish_status_ == planning_internal::OpenSpaceDebug::OVER_TIME) {
    const std::string msg = "task finish status is over time";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_PATHPARTITION_ERROR, msg);
  }
  *(frame_->mutable_open_space_info()->mutable_chosen_partitioned_path_idx()) =
      std::make_pair(partitioned_paths_ptr->path_idx,
                     partitioned_paths_ptr->point_idx);
  switch (openspace_path_decision) {
    case OpenSpacePathDecision::TASK_FINISH: {
      static constexpr double kAlmostCompleted = 11;
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_avp_to_hmi()
          ->set_park_bar_percent(kAlmostCompleted);
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_avp_to_hmi()
          ->set_nns_distance(0);
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_function_manager_out()
          ->mutable_avp_fct_out()
          ->set_parking_status(
              TL::functionmanager::AvpFctOut::MISSIONFINISHED);
      frame_->mutable_open_space_info()->set_is_stop_path(true);
      path_decision_debug_ += " stop for task finished ";
      break;
    }
    case OpenSpacePathDecision::PREPARE_FINISH: {
      static constexpr double kPrepareCompleted = 10;
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_avp_to_hmi()
          ->set_park_bar_percent(kPrepareCompleted);
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_avp_to_hmi()
          ->set_nns_distance(1);
      path_decision_debug_ += " stop for prepare finish";
      break;
    }
    case OpenSpacePathDecision::TRACK_ABNORMAL: {
      OpenSpaceInfo::UpdateReplanStatus(
          TL::planning::OpenSpaceStatus::TRACK_ABNORMAL,
          injector_->planning_context()
              ->mutable_planning_status()
              ->mutable_open_space());
      frame_->mutable_open_space_info()->set_is_stop_path(true);
      path_decision_debug_ += " track abnormal ";
      break;
    }
    case OpenSpacePathDecision::NO_VALID_PATH: {
      OpenSpaceInfo::UpdateReplanStatus(
          TL::planning::OpenSpaceStatus::NO_VALID_PATH,
          injector_->planning_context()
              ->mutable_planning_status()
              ->mutable_open_space());
      frame_->mutable_open_space_info()->set_is_stop_path(true);
      path_decision_debug_ += " stop for no valid path ";
      break;
    }
    case OpenSpacePathDecision::CHOOSE_HISTORY_PATH: {
      path_decision_debug_ += " choose history path ";
      UpdateHistoryPath(*partitioned_paths_ptr);
      UpdateParkDisplay(partitioned_paths_ptr);
      break;
    }
    case OpenSpacePathDecision::CHOOSE_NEW_PATH: {
      path_decision_debug_ += " choose new path ";
      UpdateHistoryPath(*partitioned_paths_ptr);
      UpdateParkDisplay(partitioned_paths_ptr);
      break;
    }
    default:
      return Status(ErrorCode::PLANNER_PARKING_PATHPARTITION_ERROR,
                    "error partition");
  }

  AdjustRelativeS(
      partitioned_paths_ptr->path_set, partitioned_paths_ptr->path_idx,
      partitioned_paths_ptr->point_idx, chosen_partitioned_path_ptr);
  UpdateInfoForPreFinishCondition(openspace_path_decision,
                                  chosen_partitioned_path_ptr);
  UpdateStatusBasedPartitionResult();

  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_avp_to_hmi()
      ->set_is_mirror_fold(is_mirror_fold_);

  if (FLAGS_enable_record_debug) {
    if (nullptr != chosen_partitioned_path_ptr) {
      path_decision_debug_ +=
          soc::Chassis::GearPosition_Name(chosen_partitioned_path_ptr->second);
    }
    auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug();
    *(ptr_debug->mutable_planning_data()
          ->mutable_open_space()
          ->mutable_path_decision()) = path_decision_debug_;
    frame_->mutable_open_space_info()->sync_debug_instance();
  }
  ADEBUG << "path_decision " << path_decision_debug_;
  if (!injector_->planning_context()
           ->planning_status()
           .open_space()
           .has_replan() ||
      (injector_->planning_context()->planning_status().open_space().replan() &
       static_cast<uint32_t>(OpenSpaceStatus::YAW_TRACK_ABNORMAL)) == 0U) {
    // reset time cnt
    yaw_track_abnormal_start_time_ = TL::common::Clock::NowInSeconds();
  }
  return Status::OK();
}

void OpenSpacePathPartition::UpdateParam() {
  const common::VehicleState& vehicle_state = frame_->vehicle_state();
  ego_x_ = vehicle_state.x();
  ego_y_ = vehicle_state.y();
  ego_theta_ = vehicle_state.heading();
  start_point_ = frame_->PlanningStartPoint().path_point();
  const auto& previous_frame = injector_->frame_history()->Latest();
  if (nullptr == previous_frame ||
      previous_frame->vehicle_state().gear() != vehicle_state.gear()) {
    is_warm_start_ = false;
    frozen_near_end_time_ = TL::common::Clock::NowInSeconds();
  }
  is_warm_start_ = is_warm_start_ || !frame_->IsVehicleStandStill();

  // fct pause
  const bool fct_pause =
      previous_frame != nullptr &&
      previous_frame->local_view().HasFunctionManagerIn() &&
      previous_frame->local_view()
              .GetFunctionManagerIn()
              ->fct_avp_in()
              .sys_run_state() == functionmanager::AvpFctIn::PAUSE;

  // const bool stop_by_plan =
  //     frame_->local_view().GetGuardTriggeredFlag() ||
  //     (previous_frame != nullptr &&
  //      previous_frame->open_space_info().speed_task_interactive_stage() !=
  //          AvpSpeedPlanCollisionInfo::INIT);
  // TODO(jyw): Temporarily don't distinguish between ultrasonic braking
  // and control response timeout
  const bool stop_by_plan =
      (previous_frame != nullptr &&
       previous_frame->open_space_info().speed_task_interactive_stage() !=
           AvpSpeedPlanCollisionInfo::INIT);
  if (stop_by_plan || fct_pause || !frame_->IsVehicleStandStill()) {
    frozen_near_end_time_ = TL::common::Clock::NowInSeconds();
  }
}

void OpenSpacePathPartition::UpdateStatusBasedPartitionResult() {
  auto* open_space_info_ptr = frame_->mutable_open_space_info();
  auto* partitioned_paths_ptr =
      open_space_info_ptr->mutable_partitioned_paths();
  auto* chosen_partitioned_path_ptr =
      open_space_info_ptr->mutable_chosen_partitioned_path();
  if (nullptr != partitioned_paths_ptr &&
      partitioned_paths_ptr->path_set.size() >
          partitioned_paths_ptr->path_idx) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_open_space()
        ->set_current_part_path_length(
            partitioned_paths_ptr->path_set[partitioned_paths_ptr->path_idx]
                .first.Length());
  }
  if (nullptr == chosen_partitioned_path_ptr ||
      chosen_partitioned_path_ptr->first.empty() ||
      chosen_partitioned_path_ptr->first.back().s() >
          open_space_path_partition_config_
              .rough_longitudinal_offset_to_midpoint()) {
    frozen_near_end_time_ = TL::common::Clock::NowInSeconds();
  }
  const bool path_state_gear_same =
      (nullptr == chosen_partitioned_path_ptr) ||
      chosen_partitioned_path_ptr->second == frame_->vehicle_state().gear();
  if (!path_state_gear_same) {
    // avoid to trigger dynamic replan when the path state is changed
    return;
  }
  // trigger dynamic replan
  bool need_dynamic_plan = false;
  const auto park_scenario_type =
      frame_->open_space_info()
          .open_space_path_info()
          .open_space_env_structured_info.parking_scenario_type;
  switch (park_scenario_type) {
    case (LEFT_VERTICAL_PARKING_IN):
    case (RIGHT_VERTICAL_PARKING_IN):
    case (LEFT_OBLIQUE_PARKING_IN):
    case (RIGHT_OBLIQUE_PARKING_IN): {
      const auto gear_tmp =
          frame_->open_space_info()
                  .open_space_path_info()
                  .open_space_env_structured_info.is_parking_inwards
              ? soc::Chassis_GearPosition_GEAR_DRIVE
              : soc::Chassis_GearPosition_GEAR_REVERSE;
      if (frame_->vehicle_state().gear() == gear_tmp &&
          (partitioned_paths_ptr->path_set.size() >
           partitioned_paths_ptr->path_idx + 1) &&
          !frame_->IsVehicleStandStill()) {
        need_dynamic_plan = true;
      }
      break;
    }
    case (LEFT_LATERAL_PARKING_IN):
    case (RIGHT_LATERAL_PARKING_IN): {
      const auto* ptr_last_frame = injector_->frame_history()->Latest();
      if ((ptr_last_frame != nullptr &&
           ptr_last_frame->open_space_info()
               .replan_triggered_by_speed_plan()) ||
          frame_->local_view().GetGuardTriggeredFlag()) {
        ADEBUG << " history has collision with obs, skip dynamic replan";
        // if triggerd by speed plan, need to replan(by search) instead of dynamic replan(geometry)
        break;
      }
      if (frame_->vehicle_state().gear() ==
              soc::Chassis_GearPosition_GEAR_REVERSE &&
          (partitioned_paths_ptr->path_set.size() >
           partitioned_paths_ptr->path_idx + 2)) {
        need_dynamic_plan = true;
      }
      break;
    }
    case (LEFT_VERTICAL_PARKING_OUT):
    case (RIGHT_VERTICAL_PARKING_OUT):
    case (LEFT_OBLIQUE_PARKING_OUT):
    case (RIGHT_OBLIQUE_PARKING_OUT): {
      if (frame_->vehicle_state().gear() ==
              soc::Chassis_GearPosition_GEAR_DRIVE &&
          (partitioned_paths_ptr->path_set.size() >
           partitioned_paths_ptr->path_idx + 1) &&
          !frame_->IsVehicleStandStill()) {
        need_dynamic_plan = true;
      }
      break;
    }
    case (LEFT_LATERAL_PARKING_OUT):
    case (RIGHT_LATERAL_PARKING_OUT): {
      static constexpr double kDynamicReplanDist = 1.0;
      // ToDo(gsb): add collision free constraint, consider kappa differ constraint
      if (frame_->vehicle_state().gear() ==
              soc::Chassis_GearPosition_GEAR_DRIVE &&
          (partitioned_paths_ptr->path_set.size() >
           partitioned_paths_ptr->path_idx + 1) &&
          frame_->open_space_info().get_adc_bottom_dist() >=
              kDynamicReplanDist) {
        need_dynamic_plan = true;
      }
      break;
    }
    default: {
      ADEBUG << " In other secnario, fast geometry is not used";
      break;
    }
  }
  if (need_dynamic_plan) {
    ADEBUG << " trigger dynamic replan once";
    OpenSpaceInfo::UpdateReplanStatus(
        TL::planning::OpenSpaceStatus::DYNAMIC_REPLAN,
        injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_open_space());
  }
  if (nullptr == chosen_partitioned_path_ptr ||
      chosen_partitioned_path_ptr->first.empty() ||
      chosen_partitioned_path_ptr->first.back().s() >
          open_space_path_partition_config_
              .rough_longitudinal_offset_to_midpoint()) {
    frozen_near_end_time_ = TL::common::Clock::NowInSeconds();
  }
}

void OpenSpacePathPartition::UpdatePathDecision(
    OpenSpacePathDecision* const openspace_path_decision_ptr,
    PartitionedPath* const choose_path_ptr) {
  auto pub_gear = frame_->local_view().GetChassis()->gear_location();
  const auto& previous_frame = injector_->frame_history()->Latest();
  if (nullptr != previous_frame) {
    if (previous_frame->local_view().HasADCTrajectory()) {
      pub_gear = previous_frame->local_view().GetADCTrajectory()->gear();
    } else if (previous_frame->local_view().HasADCTrajectoryGuard()) {
      pub_gear = previous_frame->local_view().GetADCTrajectoryGuard()->gear();
    }
  }
  is_mirror_fold_ =
      is_mirror_fold_ ||
      IsMirrorFold(frame_->open_space_info().open_space_path_info());
  // path match
  AlternativePath alternative_path;
  OpenSpacePathDeciderStatus open_space_path_decider_status;
  const auto& open_space_info = frame_->open_space_info();
  PartitionedPath path_result = open_space_info.path_result();
  // decide choose which path
  // check history path is valid or not
  PartitionedPath history_path_partition = history_path_;
  uint32_t executable_status = 0;
  // check if history path is valid and match
  bool is_history_path_match_suc =
      PathMatch(&history_path_partition, &executable_status);
  if (!is_history_path_match_suc && nullptr != previous_frame &&
      !previous_frame->open_space_info().is_on_open_space_trajectory()) {
    executable_status = 0;
    is_history_path_match_suc =
        GetLastCyclePubPath(previous_frame, &history_path_partition) &&
        PathMatch(&history_path_partition, &executable_status);
    history_path_partition.path_type =
        planning_internal::PathUpdateStatus::CRUISE_PATH;
  }
  // compare history path and new path
  if (is_history_path_match_suc) {
    if ((previous_frame != nullptr &&
         previous_frame->open_space_info().replan_triggered_by_speed_plan()) ||
        frame_->local_view().GetGuardTriggeredFlag()) {
      executable_status += COLLISION_RISK;
    }
    open_space_path_decider_status.executable_status = executable_status;
    open_space_path_decider_status.gear_shift_num =
        gear_shift_.second + history_path_partition.path_set.size() -
        history_path_partition.path_idx - 1;
    UpdateCollisionDistance(history_path_partition,
                            &open_space_path_decider_status.collision_distance);
    alternative_path.emplace_back(history_path_partition,
                                  open_space_path_decider_status);
  }
  executable_status = 0;
  const bool is_path_result_match_suc =
      PathMatch(&path_result, &executable_status);
  if (is_path_result_match_suc) {
    open_space_path_decider_status.Reset();
    UpdatePathExcutableStatus(path_result, &executable_status);
    open_space_path_decider_status.executable_status = executable_status;
    const auto& path_set = path_result.path_set;
    open_space_path_decider_status.gear_shift_num =
        gear_shift_.first == path_set.front().second
            ? gear_shift_.second + path_set.size() - 1
            : gear_shift_.second + path_set.size();
    UpdateCollisionDistance(path_result,
                            &open_space_path_decider_status.collision_distance);
    alternative_path.emplace_back(path_result, open_space_path_decider_status);
  }
  open_space_path_decider_status.Reset();
  // choose path by PathDecider
  int chosen_path_set_idx = PathDecider(alternative_path);
  PartitionedPath reserved_partitioned_paths;
  if (chosen_path_set_idx < 0 ||
      chosen_path_set_idx >= static_cast<int>(alternative_path.size())) {
    *openspace_path_decision_ptr = OpenSpacePathDecision::NO_VALID_PATH;
    if (!alternative_path.empty()) {
      open_space_path_decider_status = alternative_path.back().second;
      reserved_partitioned_paths = alternative_path.front().first;
    }
    SetStopPath(pub_gear, reserved_partitioned_paths, choose_path_ptr);
  } else {
    *choose_path_ptr = alternative_path[chosen_path_set_idx].first;
    open_space_path_decider_status =
        alternative_path[chosen_path_set_idx].second;
    reserved_partitioned_paths = *choose_path_ptr;
    // Add debug info
    if (is_history_path_match_suc && 0 == chosen_path_set_idx) {
      *openspace_path_decision_ptr = OpenSpacePathDecision::CHOOSE_HISTORY_PATH;
    } else {
      *openspace_path_decision_ptr = OpenSpacePathDecision::CHOOSE_NEW_PATH;
    }
  }
  if (!GetEndPointSLInCurrentPath(choose_path_ptr->path_set,
                                  choose_path_ptr->path_idx, &end_point_sl_)) {
    end_point_sl_.set_l(INFINITY);
    ADEBUG << "get end pose SLpoint failed";
  }
  if (IsTaskFinish(*choose_path_ptr)) {
    pub_gear = soc::Chassis::GEAR_PARKING;
    SetStopPath(pub_gear, reserved_partitioned_paths, choose_path_ptr);
    *openspace_path_decision_ptr = OpenSpacePathDecision::TASK_FINISH;
    return;
  }
  if (IsTrackAbnormal()) {
    SetStopPath(pub_gear, reserved_partitioned_paths, choose_path_ptr);
    *openspace_path_decision_ptr = OpenSpacePathDecision::TRACK_ABNORMAL;
    return;
  }
  if (task_finish_status_ ==
      planning_internal::OpenSpaceDebug::PREFINISH_BRAKING) {
    ADEBUG << "prefinish condition satisfied";
    *openspace_path_decision_ptr = OpenSpacePathDecision::PREPARE_FINISH;
    return;
  }
  // update replan based on path decision
  uint32_t replan_status = 0;
  GetReplanStatusBasedExcutableStatus(open_space_path_decider_status,
                                      &replan_status);
  if (replan_status > 0) {
    OpenSpaceInfo::UpdateReplanStatus(
        static_cast<OpenSpaceStatus::Replan>(replan_status),
        injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_open_space());
  }
}

bool OpenSpacePathPartition::GetLastCyclePubPath(
    const Frame* const previous_frame,
    PartitionedPath* const partition_path_ptr) {
  if (nullptr == partition_path_ptr || nullptr == previous_frame ||
      !previous_frame->local_view().HasADCTrajectory() ||
      previous_frame->local_view()
          .GetADCTrajectory()
          ->trajectory_point()
          .empty()) {
    return false;
  }
  const auto last_pub_traj_points =
      previous_frame->local_view().GetADCTrajectory()->trajectory_point();
  std::vector<common::PathPoint> last_pub_path;
  last_pub_path.reserve(last_pub_traj_points.size());
  last_pub_path.emplace_back(last_pub_traj_points[0].path_point());
  for (int i = 1; i < last_pub_traj_points.size(); ++i) {
    if (common::math::double_type::DefinitelyLessEqual(
            last_pub_traj_points[i].path_point().s(),
            last_pub_traj_points[i - 1].path_point().s())) {
      break;
    }
    last_pub_path.emplace_back(last_pub_traj_points[i].path_point());
  }
  if (last_pub_path.size() < 2) {
    return false;
  }
  ResetPartitionPath(partition_path_ptr);
  partition_path_ptr->path_set.emplace_back(
      DiscretizedPath(last_pub_path),
      previous_frame->local_view().GetADCTrajectory()->gear());
  return true;
}

bool OpenSpacePathPartition::PathMatch(
    PartitionedPath* const partition_path_ptr,
    uint32_t* const executable_status_ptr) {
  if (nullptr == partition_path_ptr || nullptr == executable_status_ptr ||
      partition_path_ptr->path_set.empty()) {
    return false;
  }
  size_t path_idx = partition_path_ptr->path_idx;
  size_t path_point_idx = partition_path_ptr->point_idx;
  if (GetPathMatchIdx(*partition_path_ptr, path_idx, &path_point_idx)) {
    UpdatePathMatchIdx(path_idx, path_point_idx, partition_path_ptr,
                       executable_status_ptr);
    const auto& match_path =
        partition_path_ptr->path_set[partition_path_ptr->path_idx].first;
    const auto& match_point = match_path[partition_path_ptr->point_idx];
    const auto& end_point = match_path.back();
    auto* open_space_info_ptr = frame_->mutable_open_space_info();
    CalculatePoseError(ego_x_, ego_y_, ego_theta_, match_point.x(),
                       match_point.y(), match_point.theta(),
                       open_space_info_ptr->mutable_vehicle_follow_error());
    CalculatePoseError(
        ego_x_, ego_y_, ego_theta_, end_point.x(), end_point.y(),
        end_point.theta(),
        open_space_info_ptr->mutable_vehicle_to_current_end_error());
    return true;
  }
  return false;
}

int32_t OpenSpacePathPartition::PathDecider(
    const AlternativePath& alternative_path) {
  // init
  if (FLAGS_enable_record_debug) {
    // set debug info
    for (size_t i = 0; i < alternative_path.size(); ++i) {
      path_decision_debug_ += "path idx " + std::to_string(i) + " : " +
                              alternative_path[i].second.DebugString() + " \n";
    }
  }
  const auto actual_mirror_fold =
      frame_->local_view().HasChassis() &&
      frame_->local_view().GetChassis()->avm_pds_info().ddcu1_mirrorfoldst() ==
          0x1;
  if (is_mirror_fold_ && !actual_mirror_fold &&
      IsVerticalParkOut(
          frame_->open_space_info()
              .open_space_path_info()
              .open_space_env_structured_info.parking_scenario_type)) {
    path_decision_debug_ += " stop for not detect mirror fold ";
    AERROR << "not detect mirror fold";
    return -1;
  }
  if (alternative_path.empty()) {
    return -1;
  }
  if (alternative_path.size() < 2) {
    // adjusting only has short path to launch
    const auto& executable_status =
        alternative_path.front().second.executable_status;
    return (executable_status == EXCUTABLE ||
            executable_status == TOO_SHORT_TO_LAUNCH ||
            executable_status == COLLISION_RISK)
               ? 0
               : -1;
  }
  // Currently only support two alternative paths
  const auto& history_path = alternative_path.front();
  const auto& new_path = alternative_path.back();
  // check excutable status
  // temp fix geometry adjust bug for rolling steering wheel
  if (new_path.first.path_type ==
          planning_internal::PathUpdateStatus::GEOMETRY_ADJUST &&
      history_path.first.path_type ==
          planning_internal::PathUpdateStatus::GEOMETRY_ADJUST &&
      !is_warm_start_) {
    ADEBUG << "both geometry adjust path, choose old path avoiding gear "
              "change frequently";
    return 0;
  }
  const auto& previous_frame = injector_->frame_history()->Latest();
  static constexpr double kEpsilon = 1e-3;
  if (previous_frame != nullptr &&
      previous_frame->open_space_info()
          .speed_plan_collision_info()
          .is_use_middle_buffer() &&
      previous_frame->open_space_info().partitioned_paths().path_type ==
          planning_internal::PathUpdateStatus::SEARCH_EXTENSION_PATH &&
      previous_frame->open_space_info().partitioned_paths().path_idx == 0 &&
      previous_frame->open_space_info()
              .speed_plan_collision_info()
              .collision_distance() < kEpsilon &&
      previous_frame->open_space_info()
              .speed_plan_collision_info()
              .collision_type() != AvpSpeedPlanCollisionInfo::NO_COLLISION) {
    AINFO << "speed can not adpat to extension path, choose old path avoiding "
             "kappa change ";
    return 0;
  }
  const bool is_history_executable =
      history_path.second.executable_status == EXCUTABLE;
  const bool is_new_path_executable =
      new_path.second.executable_status == EXCUTABLE;
  if (is_mirror_fold_) {
    return 0;
  }
  if (!is_history_executable && !is_new_path_executable) {
    // if both path are not executable, judge if second path is short path
    if (new_path.second.executable_status == TOO_SHORT_TO_LAUNCH) {
      ADEBUG
          << "both path are unexecutable, choose new short path instead stop";
      return 1;
    }
    // no valid path to choose
    return -1;
  }
  if (is_history_executable && is_new_path_executable) {
    if (new_path.second.gear_shift_num <= history_path.second.gear_shift_num) {
      return 1;
    }
    if (new_path.second.gear_shift_num > FLAGS_apa_gear_shift_limit) {
      // new path will gear shift over limit
      return 0;
    }
    if ((new_path.first.replan_status &
         static_cast<uint32_t>(OpenSpaceStatus::TARGET_UPDATE)) != 0U) {
      return (frame_->open_space_info()
                  .open_space_path_info()
                  .open_space_env_structured_info.is_in_nns_adjust_scenario &&
              new_path.second.gear_shift_num > 1)
                 ? 0
                 : 1;
    }
    if (IsUseSpeedWarnReplan(new_path.first.replan_status)) {
      return 1;
    }
    if (new_path.first.replan_status ==
            static_cast<uint32_t>(OpenSpaceStatus::TARGET_UPDATE_SLIGHTLY) ||
        new_path.first.replan_status ==
            static_cast<uint32_t>(OpenSpaceStatus::TARGET_UPDATE_SLIGHTLY) +
                static_cast<uint32_t>(OpenSpaceStatus::DYNAMIC_REPLAN) ||
        new_path.first.replan_status ==
            static_cast<uint32_t>(OpenSpaceStatus::REPLAN_FOR_SPEED_WARN) ||
        new_path.first.replan_status ==
            static_cast<uint32_t>(OpenSpaceStatus::REPLAN_FOR_SPEED_WARN) +
                static_cast<uint32_t>(OpenSpaceStatus::DYNAMIC_REPLAN) ||
        new_path.first.replan_status ==
            static_cast<uint32_t>(OpenSpaceStatus::REPLAN_FOR_SPEED_WARN) +
                static_cast<uint32_t>(
                    OpenSpaceStatus::TARGET_UPDATE_SLIGHTLY) ||
        new_path.first.replan_status ==
            static_cast<uint32_t>(OpenSpaceStatus::REPLAN_FOR_SPEED_WARN) +
                static_cast<uint32_t>(OpenSpaceStatus::TARGET_UPDATE_SLIGHTLY) +
                static_cast<uint32_t>(OpenSpaceStatus::DYNAMIC_REPLAN)) {
      // return gear shift less one
      return new_path.second.gear_shift_num <=
                     history_path.second.gear_shift_num
                 ? 1
                 : 0;
    }
    // return safer one
    return history_path.second.collision_distance >
                   new_path.second.collision_distance + 0.1
               ? 0
               : 1;
  }
  // choose excutable path
  return is_new_path_executable ? 1 : 0;
}

bool OpenSpacePathPartition::IsUseSpeedWarnReplan(
    const uint32_t replan_status) {
  if ((replan_status &
       static_cast<uint32_t>(OpenSpaceStatus::REPLAN_FOR_SPEED_WARN)) == 0U) {
    return false;
  }
  const auto& open_space_path_info =
      frame_->open_space_info().open_space_path_info();
  const auto& park_scenario_type =
      open_space_path_info.open_space_env_structured_info.parking_scenario_type;
  const bool is_vertical_park_in =
      park_scenario_type == ParkingScenarioType::LEFT_VERTICAL_PARKING_IN ||
      park_scenario_type == ParkingScenarioType::RIGHT_VERTICAL_PARKING_IN ||
      park_scenario_type == ParkingScenarioType::LEFT_OBLIQUE_PARKING_IN ||
      park_scenario_type == ParkingScenarioType::RIGHT_OBLIQUE_PARKING_IN;
  if (!is_vertical_park_in ||
      (open_space_path_info.open_space_env_structured_info
           .parking_scenario_diffculty_type &
       NARROW_SPOT_SCENARIO) == 0) {
    return false;
  }
  auto adc_point = Vec2d(ego_x_, ego_y_);
  adc_point -= open_space_path_info.origin;
  adc_point.SelfRotate(-open_space_path_info.rotate_angle);
  return adc_point.y() < common::math::kMathEpsilon;
}

void OpenSpacePathPartition::CalculatePoseError(
    const double cur_x, const double cur_y, const double cur_theta,
    const double target_x, const double target_y, const double target_theta,
    planning_internal::VehicleFollowError* const pose_error) {
  const Vec2d point_to_ego(cur_x - target_x, cur_y - target_y);
  const double diff_angle =
      std::fabs(NormalizeAngle(cur_theta - point_to_ego.Angle()));
  const double diff_distance = std::hypot(target_x - cur_x, target_y - cur_y);
  const double lateral_offset = std::fabs(diff_distance * std::sin(diff_angle));
  const double longitudinal_offset =
      std::fabs(diff_distance * std::cos(diff_angle));
  pose_error->set_angle_offset(
      std::fabs(NormalizeAngle(cur_theta - target_theta)));
  pose_error->set_lateral_offset(lateral_offset);
  pose_error->set_longitudinal_offset(longitudinal_offset);
}

bool OpenSpacePathPartition::GetPathMatchIdx(
    const PartitionedPath& partition_path, const size_t path_idx,
    size_t* path_point_idx_ptr) {
  bool match_suc = false;
  if (nullptr == path_point_idx_ptr) {
    AERROR << "GetPathMatchIdx input check error";
    return match_suc;
  }
  // const size_t path_idx = *path_idx_ptr;
  // currentlt only match path point idx
  // find the first matchable point with current start point in partition_path
  if (path_idx < partition_path.path_set.size()) {
    const auto& path_pair = partition_path.path_set[path_idx];
    const auto& path = path_pair.first;
    const auto& path_gear = path_pair.second;
    for (size_t i = 0; i < path.size(); ++i) {
      if (PointMatch(path[i], path_gear)) {
        *path_point_idx_ptr = i;
        match_suc = true;
        break;
      }
    }
    // if can not find matchable by using point match
    // using box match to find the nearest point with current start point
    if (!match_suc) {
      const auto start_point_polygon = common::math::Polygon2d(
          common::VehicleConfigHelper::GetBoundingBox(start_point_));
      std::pair<double, int> largest_iou_point(0.0, 0);
      for (size_t i = 0; i < path.size(); ++i) {
        double iou = start_point_polygon.ComputeIoU(
            Polygon2d(common::VehicleConfigHelper::GetBoundingBox(path[i])));
        if (iou > largest_iou_point.first) {
          largest_iou_point = {iou, i};
        }
      }
      if (largest_iou_point.first >
          open_space_path_partition_config_.point_match_iou_threshold()) {
        // *path_idx_ptr = path_idx;
        *path_point_idx_ptr = largest_iou_point.second;
        match_suc = true;
      }
    }
  }
  return match_suc;
}

bool OpenSpacePathPartition::PointMatch(
    const common::PathPoint& path_point,
    const soc::Chassis::GearPosition& gear) {
  Vec2d tracking_vector(path_point.x() - start_point_.x(),
                        path_point.y() - start_point_.y());
  const double distance = tracking_vector.Length();
  const double path_point_heading =
      gear == soc::Chassis::GEAR_DRIVE
          ? path_point.theta()
          : NormalizeAngle(path_point.theta() + M_PI);
  const double head_track_difference =
      std::fabs(NormalizeAngle(tracking_vector.Angle() - path_point_heading));
  return distance < open_space_path_partition_config_.distance_search_range() &&
         head_track_difference <
             open_space_path_partition_config_.heading_track_range();
}

void OpenSpacePathPartition::UpdatePathMatchIdx(
    const size_t match_path_idx, const size_t match_point_idx,
    PartitionedPath* const partition_path_ptr,
    uint32_t* const executable_status_ptr) {
  partition_path_ptr->path_idx = match_path_idx;
  partition_path_ptr->point_idx = match_point_idx;
  partition_path_ptr->path_shift = false;
  const bool has_next_path =
      match_path_idx + 1 < partition_path_ptr->path_set.size();
  if (!has_next_path || !frame_->IsVehicleStandStill()) {
    ADEBUG << "it is last path or adc is moving, can not change next path";
    return;
  }
  if (partition_path_ptr->path_set[match_path_idx + 1].second ==
      frame_->vehicle_state().gear()) {
    ADEBUG << "next path is same gear as previous";
    return;
  }
  const auto& partitioned_path_set = partition_path_ptr->path_set;
  PathPoint project_path_point;
  ProjectOnPath(
      partition_path_ptr->path_set[partition_path_ptr->path_idx].first,
      partition_path_ptr->point_idx, start_point_, &project_path_point);
  const double lon_dis =
      fabs(partitioned_path_set[match_path_idx].first.back().s() -
           project_path_point.s());
  // init
  size_t next_path_idx = partition_path_ptr->path_idx + 1;
  size_t next_point_idx = 0;
  if (AbleToGearShift(*partition_path_ptr, lon_dis, &next_path_idx,
                      &next_point_idx, executable_status_ptr)) {
    partition_path_ptr->path_idx = next_path_idx;
    partition_path_ptr->point_idx = next_point_idx;
    partition_path_ptr->path_shift = next_path_idx > match_path_idx;
  }
}

bool OpenSpacePathPartition::AbleToGearShift(
    const PartitionedPath& partition_path,
    const double lon_dis_to_gear_shift_point, size_t* const next_path_idx_ptr,
    size_t* const next_path_point_idx_ptr,
    uint32_t* const executable_status_ptr) {
  if (lon_dis_to_gear_shift_point >
      open_space_path_partition_config_
          .rough_longitudinal_offset_to_midpoint()) {
    ADEBUG << "Adc is far away from gear shift point";
    return false;
  }
  if (nullptr == next_path_idx_ptr || nullptr == next_path_point_idx_ptr ||
      nullptr == executable_status_ptr) {
    AERROR << "Input has null";
    return false;
  }
  auto gear_shift_longitudinal_threshold = SetGearShiftDis();
  if (is_warm_start_ &&
      lon_dis_to_gear_shift_point < gear_shift_longitudinal_threshold) {
    if (!GetPathMatchIdx(partition_path, *next_path_idx_ptr,
                         next_path_point_idx_ptr)) {
      *next_path_idx_ptr = partition_path.path_idx + 1;
      *next_path_point_idx_ptr = 0;
    }
    // check yaw error if shift path
    const auto& next_match_path_pair =
        partition_path.path_set[*next_path_idx_ptr];
    PathPoint project_path_point(start_point_);
    ProjectOnPath(next_match_path_pair.first, *next_path_point_idx_ptr,
                  start_point_, &project_path_point);
    const double yaw_track_abnormal_period =
        common::Clock::NowInSeconds() - yaw_track_abnormal_start_time_;
    const bool is_yaw_track_abnormal =
        yaw_track_abnormal_period < open_space_path_partition_config_
                                        .yaw_error_replan_time_threshold() &&
        IsYawTrackAbnormal(project_path_point, next_match_path_pair.second);
    if (is_yaw_track_abnormal) {
      *executable_status_ptr += LARGE_YAW_ERROR_IN_GEAR_SHIFT;
      return false;
    }
  } else {
    double frozen_duration =
        TL::common::Clock::NowInSeconds() - frozen_near_end_time_;
    const double time_threshold =
        is_warm_start_ ? open_space_path_partition_config_
                             .warm_start_response_time_threshold()
                       : open_space_path_partition_config_
                             .cold_start_response_time_threshold();
    ADEBUG << " frozen_duration " << frozen_duration;
    if (frozen_duration > time_threshold) {
      *executable_status_ptr += TOO_SHORT_TO_LAUNCH;
    }
    return false;
  }
  return is_warm_start_;
}

void OpenSpacePathPartition::ProjectOnPath(const DiscretizedPath& path,
                                           size_t next_path_point_idx,
                                           const common::PathPoint& curr_p,
                                           common::PathPoint* project_point) {
  if (nullptr == project_point) {
    return;
  }
  if (path.empty()) {
    *project_point = curr_p;
    return;
  }
  if (next_path_point_idx + 1 >= path.size()) {
    *project_point = path.back();
    return;
  }
  if (next_path_point_idx > 0) {
    static constexpr double kEpison = 1e-10;
    const auto& pre_p = path[next_path_point_idx - 1];
    const auto& next_p = path[next_path_point_idx];

    Vec2d pre_curr(curr_p.x() - pre_p.x(), curr_p.y() - pre_p.y());
    Vec2d pre_next(next_p.x() - pre_p.x(), next_p.y() - pre_p.y());

    auto ratio = pre_curr.InnerProd(pre_next) /
                 (pre_next.Length() * pre_next.Length() + kEpison);
    *project_point = TL::common::math::InterpolateUsingLinearApproximation(
        pre_p, next_p, pre_p.s() + ratio * pre_next.Length());
    return;
  }
  *project_point = path.at(next_path_point_idx);
}

size_t OpenSpacePathPartition::GetCompletePathSize(
    const std::vector<PathGearPair>& partitioned_paths) {
  size_t num = 0;
  for (const auto& path_pair : partitioned_paths) {
    num += path_pair.first.size();
  }
  num = partitioned_paths.size() < 2 ? num : num + 1 - partitioned_paths.size();
  return num;
}

void OpenSpacePathPartition::UpdateParkDisplay(PartitionedPath* const path) {
  if (path == nullptr) {
    ADEBUG << "no path , can not update park bar";
    return;
  }
  if (injector_->planning_context()
          ->planning_status()
          .avp_status()
          .parking_type() == planning::AVPStatus::NNS_ADJUST) {
    return;
  }
  auto last_bar = injector_->planning_context()
                      ->planning_status()
                      .avp_to_hmi()
                      .park_bar_percent();
  ADEBUG << "last bar " << last_bar;
  // reset park bar when change ntp_to_avp
  if (path->path_type == planning_internal::PathUpdateStatus::TRACE_PATH ||
      path->path_type == planning_internal::PathUpdateStatus::CRUISE_PATH) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_avp_to_hmi()
        ->set_park_bar_percent(last_bar);
    return;
  }
  // Only gear shift can reset display
  bool park_bar_reset_flag = false;
  if (nullptr != injector_->frame_history() &&
      nullptr != injector_->frame_history()->Latest()) {
    if (injector_->frame_history()->Latest()->local_view().HasADCTrajectory()) {
      park_bar_reset_flag =
          injector_->frame_history()
              ->Latest()
              ->local_view()
              .GetADCTrajectory()
              ->gear() != path->path_set.at(path->path_idx).second;
    } else if (injector_->frame_history()
                   ->Latest()
                   ->local_view()
                   .HasADCTrajectoryGuard()) {
      park_bar_reset_flag =
          injector_->frame_history()
              ->Latest()
              ->local_view()
              .GetADCTrajectoryGuard()
              ->gear() != path->path_set.at(path->path_idx).second;
    }
  }
  auto current_point_index = static_cast<double>(path->point_idx);
  auto current_total_size =
      static_cast<double>(path->path_set.at(path->path_idx).first.size());
  static constexpr double kEpsilon = 1e-6;
  if (current_total_size > kEpsilon) {
    // Only display same gear path percent to HMI
    // Considering partition path replan and whether new path has gear changing
    auto* avp_to_hmi = injector_->planning_context()
                           ->mutable_planning_status()
                           ->mutable_avp_to_hmi();
    double curr_gear_percent = current_point_index / current_total_size;
    double curr_rest_dist =
        fabs(path->path_set.at(path->path_idx).first.back().s() -
             path->path_set.at(path->path_idx).first.at(path->point_idx).s());
    static constexpr int kMultiplier = 10;
    static constexpr int kMultiplierForCm = 100;
    // send 100x as cm
    auto dist_value =
        std::max(0.0, std::floor(curr_rest_dist * kMultiplierForCm));
    auto curr_percent_bar = std::max(
        1, static_cast<int>(std::ceil(curr_gear_percent * kMultiplier)));
    // park_bar can only increase
    int percent_bar =
        park_bar_reset_flag
            ? 0
            : (curr_percent_bar > last_bar ? curr_percent_bar : last_bar);
    avp_to_hmi->set_park_bar_percent(
        curr_rest_dist - 0.1 < kEpsilon ? 11 : percent_bar);
    // rest distance display true distance during parking
    // Using nns_distance to represent rest distance
    avp_to_hmi->set_nns_distance(static_cast<int32_t>(dist_value));
  }
}

bool OpenSpacePathPartition::IsAddStartPointToPath(
    const common::PathPoint& pre_p, const common::PathPoint& curr_p,
    const common::PathPoint& next_p, common::PathPoint* const proj_p) {
  static constexpr double kEpison = 1e-10;
  Vec2d pre_curr(curr_p.x() - pre_p.x(), curr_p.y() - pre_p.y());
  Vec2d pre_next(next_p.x() - pre_p.x(), next_p.y() - pre_p.y());

  auto ratio = pre_curr.InnerProd(pre_next) /
               (pre_next.Length() * pre_next.Length() + kEpison);
  *proj_p = TL::common::math::InterpolateUsingLinearApproximation(
      pre_p, next_p, pre_p.s() + ratio * pre_next.Length());

  return DefinitelyLess(ratio, 1.0) && DefinitelyGreaterEqual(ratio, 0.0);
}

void OpenSpacePathPartition::AdjustRelativeS(
    const std::vector<PathGearPair>& partitioned_paths,
    const size_t current_path_index, const size_t closest_path_point_index,
    PathGearPair* const chosen_partitioned_path) {
  if (chosen_partitioned_path == nullptr) {
    AERROR << "input nullptr";
    return;
  }

  if (partitioned_paths.empty() ||
      partitioned_paths.size() <= current_path_index ||
      partitioned_paths.at(current_path_index).first.size() <=
          closest_path_point_index) {
    chosen_partitioned_path->second = frame_->vehicle_state().gear();
    chosen_partitioned_path->first.GenerateStopPath(
        start_point_.x(), start_point_.y(), start_point_.theta(),
        start_point_.kappa());
    AERROR << "invalid partitioned path";
    return;
  }

  // Reassign relative time and relative s to have the closest point as origin
  // point
  chosen_partitioned_path->second =
      partitioned_paths.at(current_path_index).second;
  auto& des_path = chosen_partitioned_path->first;
  const auto& curr_gear = chosen_partitioned_path->second;
  const auto& src_path = partitioned_paths[current_path_index].first;
  bool is_gear_changed = false;
  if (nullptr != injector_->frame_history() &&
      nullptr != injector_->frame_history()->Latest() &&
      injector_->frame_history()->Latest()->local_view().HasADCTrajectory() &&
      injector_->frame_history()
              ->Latest()
              ->local_view()
              .GetADCTrajectory()
              ->gear() != curr_gear) {
    is_gear_changed = true;
  }
  bool is_add_start_point = false;
  PathPoint project_path_point;
  if (closest_path_point_index > 0) {
    is_add_start_point = IsAddStartPointToPath(
        src_path[closest_path_point_index - 1], start_point_,
        src_path[closest_path_point_index], &project_path_point);
  }

  frame_->mutable_open_space_info()->set_is_gear_changed(is_gear_changed);

  if (closest_path_point_index > 0 && is_add_start_point) {
    size_t partition_size = src_path.size() - closest_path_point_index;
    des_path.resize(partition_size + 1);

    Vec2d pre(project_path_point.x(), project_path_point.y());
    Vec2d curr(src_path[closest_path_point_index].x(),
               src_path[closest_path_point_index].y());
    double offset_s =
        pre.DistanceTo(curr) - src_path[closest_path_point_index].s();

    des_path[0] = project_path_point;
    des_path[0].set_s(0.0);
    for (size_t i = 1; i < partition_size + 1; i++) {
      PathPoint& path_point = des_path[i];
      path_point = src_path[closest_path_point_index + i - 1];
      path_point.set_s(path_point.s() + offset_s);
    }
  } else {
    double offset_s = -src_path[closest_path_point_index].s();
    auto partition_size = src_path.size() - closest_path_point_index;
    des_path.resize(partition_size);

    for (int i = 0; i < partition_size; ++i) {
      PathPoint& path_point = des_path[i];
      path_point = src_path[closest_path_point_index + i];
      path_point.set_s(path_point.s() + offset_s);
    }
  }
}

bool OpenSpacePathPartition::IsTrackAbnormal() {
  planning_internal::VehicleFollowError error;
  CalculatePoseError(ego_x_, ego_y_, ego_theta_, start_point_.x(),
                     start_point_.y(), start_point_.theta(), &error);
  const double distance_threshold =
      open_space_path_partition_config_.distance_search_range();
  if (error.longitudinal_offset() > distance_threshold ||
      error.lateral_offset() > distance_threshold) {
    *(frame_->mutable_open_space_info()->mutable_vehicle_follow_error()) =
        error;
    return true;
  }
  return false;
}

bool OpenSpacePathPartition::IsYawTrackAbnormal(
    const common::PathPoint& track_point,
    const soc::Chassis::GearPosition& gear) {
  bool is_yaw_track_abnomal = false;
  double yaw_track_error = track_point.theta() - ego_theta_;
  int next_path_direction = gear == soc::Chassis::GEAR_DRIVE ? 1 : -1;
  const bool is_convergence_trend =
      common::math::double_type::DefinitelyGreater(
          yaw_track_error * track_point.kappa() * next_path_direction, 0.0);
  if (is_convergence_trend) {
    is_yaw_track_abnomal =
        fabs(yaw_track_error) > open_space_path_partition_config_
                                    .convergence_trend_yaw_error_threshold();
  } else {
    is_yaw_track_abnomal =
        fabs(yaw_track_error) >
        open_space_path_partition_config_.divergent_trend_yaw_error_threshold();
  }
  ADEBUG << "is_yaw_track_abnomal " << is_yaw_track_abnomal
         << " yaw_track_error " << yaw_track_error;
  return is_yaw_track_abnomal;
}

bool OpenSpacePathPartition::IsTaskFinish(const PartitionedPath& chosen_path) {
  std::string msg;
  const auto& veh_gear = frame_->vehicle_state().gear();
  const bool is_adc_gear_valid =
      (veh_gear == TL::soc::Chassis::GEAR_DRIVE ||
       veh_gear == TL::soc::Chassis::GEAR_REVERSE);
  if (is_adc_gear_valid && veh_gear != gear_shift_.first) {
    if (gear_shift_.first != TL::soc::Chassis::GEAR_PARKING) {
      ++gear_shift_.second;
    }
    gear_shift_.first = veh_gear;
  }
  bool is_task_finish = false;
  const AVPStatus::ParkingType& parking_type = injector_->planning_context()
                                                   ->planning_status()
                                                   .avp_status()
                                                   .parking_type();

  switch (parking_type) {
    case TL::planning::AVPStatus::PARKING_IN:
    case TL::planning::AVPStatus::TEST_CONTROL_MODE: {
      if (!is_veh_reach_destination_) {
        AdcStatus adc_status;
        GetAdcStatus(frame_->vehicle_state(), chosen_path, &adc_status);
        UpdateFinishStatusBasedOnStatus(adc_status);
        UpdateReplanInfoBasedOnStatus(adc_status);
        if (IsEndReplanTriggered()) {
          task_finish_status_ =
              TL::planning_internal::OpenSpaceDebug::LARGE_ANGLE;
        }
        ADEBUG << adc_status.DebugString();
        if (FinishCheck() == SUCCESS) {
          is_veh_reach_destination_ = true;
        }
      }
      if (is_veh_reach_destination_) {
        msg = "Vehicle is near to destination, Parking finished";
        is_task_finish = true;
      }
      break;
    }
    case TL::planning::AVPStatus::PARKING_OUT_FRONT:
    case TL::planning::AVPStatus::PARKING_OUT_BACK:
    case TL::planning::AVPStatus::PARKING_OUT_LEFT:
    case TL::planning::AVPStatus::PARKING_OUT_RIGHT: {
      if (is_veh_reach_destination_ ||
          IsSatisfyParkOutFinishCondition(frame_->vehicle_state(),
                                          frame_->open_space_info()
                                              .open_space_path_info()
                                              .dest_region_with_angle)) {
        msg =
            "Vehicle is satisfy park out finish condition, Parking "
            "finished";
        is_veh_reach_destination_ = true;
        is_task_finish = true;
      }
      break;
    }
    case TL::planning::AVPStatus::PARKING_OUT_NNS: {
      is_veh_reach_destination_ = is_veh_reach_destination_ || IsVehOnRoad();
      break;
    }
    case TL::planning::AVPStatus::NNS_ADJUST: {
      is_veh_reach_destination_ = false;
      is_task_finish = false;
    }
    default: {
      msg = "error parking type";
      break;
    }
  }

  frame_->mutable_open_space_info()->set_destination_reached(
      is_veh_reach_destination_);

  if (FLAGS_enable_record_debug) {
    auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug();
    ptr_debug->mutable_planning_data()->mutable_open_space()->set_finish_status(
        task_finish_status_);
  }

  return is_task_finish;
}

void OpenSpacePathPartition::GetAdcStatus(
    const common::VehicleState& vehicle_state,
    const PartitionedPath& chosen_path, AdcStatus* const adc_status_ptr) {
  if (nullptr == adc_status_ptr) {
    AERROR << "adc status input check error.";
    return;
  }
  const auto end_pose_enu = TaskTargetPose(chosen_path);
  if (frame_->IsVehicleStandStill()) {
    adc_status_ptr->is_stand_still = true;
  } else {
    frozen_time_ = TL::common::Clock::NowInSeconds();
  }
  constexpr double kAlmostStandStillSpd = 0.3;
  adc_status_ptr->is_almost_stand_still =
      fabs(vehicle_state.linear_velocity()) <= kAlmostStandStillSpd;

  adc_status_ptr->is_collision_near_target =
      IsCollisionNearTarget(vehicle_state, end_pose_enu);

  adc_status_ptr->is_reach_wheel_mask = IsReachWheelMask();

  IsBlockByCurbOrCar(end_pose_enu, &adc_status_ptr->is_block_by_curb,
                     &adc_status_ptr->is_block_by_car,
                     &adc_status_ptr->is_block_by_other_fs);

  const auto target_id =
      frame_->local_view().GetParkingLotOutArray()->opt_parking_seq();
  const auto& parking_lots =
      frame_->local_view().GetParkingLotOutArray()->parking_lots();
  adc_status_ptr->is_uss_spot = false;
  for (const auto& parking_lot : parking_lots) {
    if (parking_lot.parking_seq() == target_id) {
      adc_status_ptr->is_uss_spot =
          parking_lot.sensor_type() == perception::ParkingLotOut::USS;
      break;
    }
  }

  GetAdcPosStatus(vehicle_state, end_pose_enu, adc_status_ptr);
  GetAdcHeadingStatus(vehicle_state, end_pose_enu, adc_status_ptr);

  const auto& park_scenario_type =
      frame_->open_space_info()
          .open_space_path_info()
          .open_space_env_structured_info.parking_scenario_type;
  adc_status_ptr->is_lateral_park_in =
      park_scenario_type == ParkingScenarioType::LEFT_LATERAL_PARKING_IN ||
      park_scenario_type == ParkingScenarioType::RIGHT_LATERAL_PARKING_IN;

  adc_status_ptr->frozen_duration =
      TL::common::Clock::NowInSeconds() - frozen_time_;
  static constexpr double plan_cycle = 0.1;
  adc_status_ptr->is_over_time =
      adc_status_ptr->frozen_duration >=
      plan_cycle *
          open_space_path_partition_config_.destination_long_time_count();
  adc_status_ptr->is_blocked_over_time =
      adc_status_ptr->frozen_duration >
      open_space_path_partition_config_.early_blocked_replan_time_threshold();

  adc_status_ptr->is_execute_last_part_path =
      IsAdcExecuteLastPartPath(chosen_path);
}

void OpenSpacePathPartition::GetAdcPosStatus(
    const common::VehicleState& vehicle_state,
    const common::PathPoint& end_pose_enu, AdcStatus* const adc_status_ptr) {
  if (nullptr == adc_status_ptr) {
    AERROR << "adc pos status input check error.";
    return;
  }

  const auto target_to_adc = Vec2d(vehicle_state.x() - end_pose_enu.x(),
                                   vehicle_state.y() - end_pose_enu.y());
  const auto& parking_lot_enu = frame_->get_parking_lot_vertices();
  adc_status_ptr->dist_error = target_to_adc.Length();

  const AVPStatus::ParkingType& parking_type = injector_->planning_context()
                                                   ->planning_status()
                                                   .avp_status()
                                                   .parking_type();
  static constexpr double kLonReachThresholdSlack = 0.1;
  const double distance_threshold =
      open_space_path_partition_config_
          .is_near_destination_distance_threshold();
  const double lon_distance_threshold =
      (adc_status_ptr->is_block_by_other_fs || adc_status_ptr->is_uss_spot)
          ? distance_threshold + kLonReachThresholdSlack
          : distance_threshold;
  adc_status_ptr->is_distance_reach =
      adc_status_ptr->dist_error < distance_threshold;

  if (parking_type == planning::AVPStatus::PARKING_IN &&
      parking_lot_enu.size() > 3) {
    // check over control
    const auto target_unit_vec = Vec2d::CreateUnitVec2d(
        (parking_lot_enu[0] - parking_lot_enu[1]).Angle());

    adc_status_ptr->lon_error = target_unit_vec.InnerProd(target_to_adc);
    adc_status_ptr->lat_error = target_unit_vec.CrossProd(target_to_adc);
    const double long_error_lower_bound =
        std::min(-1 * distance_threshold,
                 target_unit_vec.InnerProd(
                     0.5 * (parking_lot_enu[1] + parking_lot_enu[2]) -
                     Vec2d(end_pose_enu.x(), end_pose_enu.y())));
    auto isInRange = [](double val, double lower, double upper) {
      return val < upper && val > lower;
    };

    adc_status_ptr->is_lon_reach =
        isInRange(adc_status_ptr->lon_error, long_error_lower_bound,
                  lon_distance_threshold);
    adc_status_ptr->is_lat_reach = isInRange(
        adc_status_ptr->lat_error, -1 * distance_threshold, distance_threshold);

    if (adc_status_ptr->is_reach_wheel_mask ||
        adc_status_ptr->is_block_by_car || adc_status_ptr->is_block_by_curb) {
      adc_status_ptr->is_lon_reach = adc_status_ptr->lon_error <
                                     config_.open_space_path_partition_config()
                                         .is_earily_finish_distance_threshold();
    }

    if (!adc_status_ptr->is_distance_reach) {
      adc_status_ptr->is_distance_reach =
          adc_status_ptr->is_lon_reach && adc_status_ptr->is_lat_reach;
    }
  }
}

void OpenSpacePathPartition::GetAdcHeadingStatus(
    const common::VehicleState& vehicle_state,
    const common::PathPoint& end_pose_enu, AdcStatus* const adc_status_ptr) {
  if (nullptr == adc_status_ptr) {
    AERROR << "adc heading status input check error.";
    return;
  }

  adc_status_ptr->heading_error = std::fabs(
      common::math::AngleDiff(vehicle_state.heading(), end_pose_enu.theta()));
  const auto last_bar = injector_->planning_context()
                            ->planning_status()
                            .avp_to_hmi()
                            .park_bar_percent();
  static constexpr int32_t kPrefinshThrehold = 9;
  // if almost finish use larger threhold
  const double angle_diff_threhold =
      last_bar >= kPrefinshThrehold ? open_space_path_partition_config_
                                          .is_near_destination_theta_threshold()
                                    : open_space_path_partition_config_
                                          .is_earily_finish_theta_threshold();
  adc_status_ptr->is_heading_reach =
      adc_status_ptr->heading_error < angle_diff_threhold;
}

void OpenSpacePathPartition::UpdateFinishStatusBasedOnStatus(
    const AdcStatus& adc_status) {
  task_finish_status_ = planning_internal::OpenSpaceDebug::UNKNOWN;

  if (adc_status.is_collision_near_target) {
    ADEBUG << "finish immediate due to collison";
    task_finish_status_ = planning_internal::OpenSpaceDebug::COLLISION_FINISH;
    return;
  }

  if (adc_status.is_reach_wheel_mask) {
    const auto& park_scenario_type =
        frame_->open_space_info()
            .open_space_path_info()
            .open_space_env_structured_info.parking_scenario_type;
    bool is_vertical_park_in =
        (park_scenario_type == ParkingScenarioType::LEFT_OBLIQUE_PARKING_IN ||
         park_scenario_type == ParkingScenarioType::RIGHT_OBLIQUE_PARKING_IN ||
         park_scenario_type == ParkingScenarioType::LEFT_VERTICAL_PARKING_IN ||
         park_scenario_type == ParkingScenarioType::RIGHT_VERTICAL_PARKING_IN);
    if (!is_vertical_park_in ||
        (is_vertical_park_in && adc_status.is_lat_reach &&
         adc_status.is_execute_last_part_path)) {
      task_finish_status_ = planning_internal::OpenSpaceDebug::REACH_WHEEL_MASK;
      return;
    }
  }

  if (adc_status.is_block_by_car && adc_status.is_lon_reach &&
      adc_status.is_lat_reach && adc_status.is_execute_last_part_path) {
    task_finish_status_ =
        planning_internal::OpenSpaceDebug::BLOCK_BY_CAR_IN_SPOT;
    return;
  }

  if (!adc_status.is_stand_still) {
    task_finish_status_ = planning_internal::OpenSpaceDebug::VEHICEL_MOVING;
    if (!adc_status.is_lateral_park_in || !adc_status.is_almost_stand_still) {
      return;
    }
    const bool is_heading_reach =
        adc_status.heading_error <=
        open_space_path_partition_config_.is_earily_finish_theta_threshold();
    const bool is_pose_reach = adc_status.is_distance_reach && is_heading_reach;
    const bool is_path_pass_through_target =
        fabs(end_point_sl_.l()) < open_space_path_partition_config_
                                      .is_near_destination_distance_threshold();
    if (is_veh_prefinish_brake_saftisfied_ ||
        (is_pose_reach && is_path_pass_through_target)) {
      ADEBUG << "veh satisfy prefinish condition, brake now";
      is_veh_prefinish_brake_saftisfied_ = true;
      task_finish_status_ =
          planning_internal::OpenSpaceDebug::PREFINISH_BRAKING;
    }
    return;
  }

  if (adc_status.is_block_by_curb && adc_status.is_lon_reach &&
      adc_status.is_lat_reach && adc_status.is_execute_last_part_path) {
    task_finish_status_ =
        planning_internal::OpenSpaceDebug::BLOCK_BY_CURB_IN_SPOT;
    return;
  }

  if (is_veh_prefinish_brake_saftisfied_) {
    ADEBUG << "veh already prefinish braked, reach destination";
    task_finish_status_ = planning_internal::OpenSpaceDebug::REACH_TARGET;
    return;
  }

  if (!adc_status.is_heading_reach) {
    if (adc_status.is_distance_reach && adc_status.is_execute_last_part_path &&
        adc_status.is_over_time) {
      task_finish_status_ = planning_internal::OpenSpaceDebug::OVER_TIME;
      return;
    }
    task_finish_status_ = planning_internal::OpenSpaceDebug::LARGE_ANGLE;
    return;
  }

  if (!adc_status.is_distance_reach || !adc_status.is_execute_last_part_path ||
      !is_warm_start_) {
    task_finish_status_ = planning_internal::OpenSpaceDebug::FAR_AWAY;
    return;
  }

  task_finish_status_ = planning_internal::OpenSpaceDebug::REACH_TARGET;
}

void OpenSpacePathPartition::UpdateReplanInfoBasedOnStatus(
    const AdcStatus& adc_status) {
  if (task_finish_status_ ==
      TL::planning_internal::OpenSpaceDebug::PREFINISH_BRAKING) {
    return;
  }

  if (task_finish_status_ == planning_internal::OpenSpaceDebug::REACH_TARGET &&
      is_veh_prefinish_brake_saftisfied_) {
    // reset prefinishbrake state
    is_veh_prefinish_brake_saftisfied_ = false;
    return;
  }

  if (FinishCheck() != SUCCESS) {
    if (!adc_status.is_heading_reach && adc_status.is_distance_reach &&
        adc_status.is_stand_still && is_warm_start_ &&
        adc_status.is_execute_last_part_path && !adc_status.is_over_time) {
      OpenSpaceInfo::UpdateReplanStatus(
          TL::planning::OpenSpaceStatus::END_ANGLE_UNREACHABLE,
          injector_->planning_context()
              ->mutable_planning_status()
              ->mutable_open_space());
    }
    return;
  }

  if (adc_status.is_blocked_over_time) {
    return;
  }

  const AVPStatus::ParkingType& parking_type = injector_->planning_context()
                                                   ->planning_status()
                                                   .avp_status()
                                                   .parking_type();
  if (parking_type != planning::AVPStatus::PARKING_IN) {
    return;
  }

  if (!adc_status.is_reach_wheel_mask && !adc_status.is_block_by_curb &&
      !adc_status.is_block_by_car) {
    return;
  }
  constexpr double kLatDisThreshold = 0.02;
  const double angle_diff_threshold =
      adc_status.lat_error < kLatDisThreshold
          ? open_space_path_partition_config_.is_earily_finish_theta_threshold()
          : open_space_path_partition_config_
                .is_precisely_arrive_theta_threshold();
  const bool is_heading_reach = adc_status.heading_error < angle_diff_threshold;

  if (!is_heading_reach) {
    AINFO << "Not precisely arrived yet, replan again.";

    OpenSpaceInfo::UpdateReplanStatus(
        TL::planning::OpenSpaceStatus::END_ANGLE_UNREACHABLE,
        injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_open_space());
  }
}

bool OpenSpacePathPartition::IsEndReplanTriggered() {
  const auto replan_status =
      injector_->planning_context()->planning_status().open_space().replan();
  return (replan_status &
          static_cast<uint32_t>(
              TL::planning::OpenSpaceStatus::END_ANGLE_UNREACHABLE)) != 0U;
}

bool OpenSpacePathPartition::IsCollisionNearTarget(
    const common::VehicleState& vehicle_state,
    const common::PathPoint& end_pose_enu) {
  const auto& park_scenario_type =
      frame_->open_space_info()
          .open_space_path_info()
          .open_space_env_structured_info.parking_scenario_type;
  const bool need_check_wheel_mask =
      (park_scenario_type == ParkingScenarioType::LEFT_OBLIQUE_PARKING_IN ||
       park_scenario_type == ParkingScenarioType::RIGHT_OBLIQUE_PARKING_IN ||
       park_scenario_type == ParkingScenarioType::LEFT_VERTICAL_PARKING_IN ||
       park_scenario_type == ParkingScenarioType::RIGHT_VERTICAL_PARKING_IN);
  // check vehicle is in slot and collsion with obs
  static constexpr double valid_speed_limit = 0.05;
  if (need_check_wheel_mask &&
      vehicle_state.pose().linear_velocity_vrf().y() > valid_speed_limit &&
      vehicle_state.gear() ==
          TL::soc::Chassis_GearPosition::Chassis_GearPosition_GEAR_REVERSE) {
    // vehicle polygon
    const auto vehicle_polygon =
        common::VehicleConfigHelper::GetPolygon2dWithBuffer(
            vehicle_state.x(), vehicle_state.y(), vehicle_state.heading());

    const auto ideal_vehicle_polygon =
        common::VehicleConfigHelper::GetPolygon2dWithBuffer(
            end_pose_enu.x(), end_pose_enu.y(), end_pose_enu.theta());
    const double iou = vehicle_polygon.ComputeIoU(ideal_vehicle_polygon);

    ADEBUG << "vehicle has reverse speed compare to the gear, and the overlap "
              "iou reach : "
           << iou << " iou threshold "
           << open_space_path_partition_config_.reach_ideal_pose_threshold();
    return iou > open_space_path_partition_config_.reach_ideal_pose_threshold();
  }
  return false;
}

void OpenSpacePathPartition::IsBlockByCurbOrCar(
    const common::PathPoint& end_pose_enu, bool* const is_block_by_curb,
    bool* const is_block_by_car, bool* const is_block_by_other_fs) {
  if (nullptr == is_block_by_curb || nullptr == is_block_by_car ||
      nullptr == is_block_by_other_fs) {
    return;
  }
  *is_block_by_curb = false;
  *is_block_by_car = false;
  *is_block_by_other_fs = false;
  const auto& park_scenario_type =
      frame_->open_space_info()
          .open_space_path_info()
          .open_space_env_structured_info.parking_scenario_type;
  bool is_vertical_park_in =
      (park_scenario_type == ParkingScenarioType::LEFT_OBLIQUE_PARKING_IN ||
       park_scenario_type == ParkingScenarioType::RIGHT_OBLIQUE_PARKING_IN ||
       park_scenario_type == ParkingScenarioType::LEFT_VERTICAL_PARKING_IN ||
       park_scenario_type == ParkingScenarioType::RIGHT_VERTICAL_PARKING_IN);
  const auto& previous_frame = injector_->frame_history()->Latest();
  if (!is_vertical_park_in || nullptr == previous_frame ||
      !previous_frame->open_space_info().replan_triggered_by_speed_plan() ||
      !frame_->IsVehicleStandStill()) {
    return;
  }
  const auto& open_space_path_info =
      frame_->open_space_info().open_space_path_info();
  // default adc point is rear_axis center
  auto adc_point = Vec2d(ego_x_, ego_y_);
  if (open_space_path_info.open_space_env_structured_info.is_parking_inwards) {
    adc_point +=
        vehicle_param_.wheel_base() * Vec2d::CreateUnitVec2d(ego_theta_);
  }
  adc_point -= open_space_path_info.origin;
  adc_point.SelfRotate(-open_space_path_info.rotate_angle);
  if (adc_point.y() > common::math::kMathEpsilon) {
    return;
  }
  Polygon2d block_area_enu;
  GetBlockArea(end_pose_enu, &block_area_enu);
  for (const auto& free_space_item :
       previous_frame->local_view().GetFreeSpaceOutArray()->freespace_out()) {
    const int points_num = free_space_item.freespace_keypoint_size();
    for (int i = 0; i + 1 < points_num; i++) {
      const Vec2d start_point{free_space_item.freespace_keypoint().at(i).x(),
                              free_space_item.freespace_keypoint().at(i).y()};
      const Vec2d end_point{free_space_item.freespace_keypoint().at(i + 1).x(),
                            free_space_item.freespace_keypoint().at(i + 1).y()};
      const common::math::LineSegment2d fs_seg(start_point, end_point);
      if (block_area_enu.HasOverlap(fs_seg)) {
        switch (free_space_item.cls()) {
          case perception::FreeSpaceOut::CURBSTONE: {
            *is_block_by_curb = true;
            return;
          }
          case perception::FreeSpaceOut::VEHICLE: {
            *is_block_by_car = true;
            return;
          }
          case perception::FreeSpaceOut::OTHER_CLASS: {
            *is_block_by_other_fs = true;
            return;
          }
          default:
            break;
        }
      }
    }
  }
}

void OpenSpacePathPartition::GetBlockArea(
    const common::PathPoint& end_pose_enu,
    common::math::Polygon2d* const block_area_ptr) {
  if (nullptr == block_area_ptr) {
    return;
  }
  const auto adc_bottom =
      Vec2d(ego_x_, ego_y_) + vehicle_param_.back_edge_to_center() *
                                  Vec2d::CreateUnitVec2d(ego_theta_ + M_PI);
  const auto adc_top =
      Vec2d(ego_x_, ego_y_) + vehicle_param_.front_edge_to_center() *
                                  Vec2d::CreateUnitVec2d(ego_theta_);
  const Vec2d end_point(end_pose_enu.x(), end_pose_enu.y());
  const auto& is_parking_inwards =
      frame_->open_space_info()
          .open_space_path_info()
          .open_space_env_structured_info.is_parking_inwards;
  const auto end_pose_axle =
      is_parking_inwards
          ? common::math::LineSegment2d(
                end_point +
                    0.5 * vehicle_param_.width() *
                        Vec2d::CreateUnitVec2d(end_pose_enu.theta() - M_PI_2),
                end_point +
                    0.5 * vehicle_param_.width() *
                        Vec2d::CreateUnitVec2d(end_pose_enu.theta() + M_PI_2))
          : common::math::LineSegment2d(
                end_point +
                    0.5 * vehicle_param_.width() *
                        Vec2d::CreateUnitVec2d(end_pose_enu.theta() + M_PI_2),
                end_point +
                    0.5 * vehicle_param_.width() *
                        Vec2d::CreateUnitVec2d(end_pose_enu.theta() - M_PI_2));
  const double dist_to_end_axle =
      end_pose_axle.ProductOntoUnit(is_parking_inwards ? adc_top : adc_bottom);
  constexpr double block_area_length = 0.5;
  std::vector<Vec2d> block_area_points(4);
  block_area_points[0] =
      end_pose_axle.start() +
      dist_to_end_axle * Vec2d::CreateUnitVec2d(
                             is_parking_inwards ? end_pose_enu.theta() + M_PI
                                                : end_pose_enu.theta());
  block_area_points[3] =
      end_pose_axle.end() +
      dist_to_end_axle * Vec2d::CreateUnitVec2d(
                             is_parking_inwards ? end_pose_enu.theta() + M_PI
                                                : end_pose_enu.theta());
  block_area_points[1] =
      block_area_points[0] +
      block_area_length * Vec2d::CreateUnitVec2d(
                              is_parking_inwards ? end_pose_enu.theta()
                                                 : end_pose_enu.theta() + M_PI);
  block_area_points[2] =
      block_area_points[3] +
      block_area_length * Vec2d::CreateUnitVec2d(
                              is_parking_inwards ? end_pose_enu.theta()
                                                 : end_pose_enu.theta() + M_PI);
  *block_area_ptr = Polygon2d(block_area_points);
  ADEBUG << "block area judge polygon: " << block_area_ptr->DebugString();
}

bool OpenSpacePathPartition::IsReachWheelMask() {
  const auto& previous_frame = injector_->frame_history()->Latest();
  return previous_frame != nullptr &&
         frame_->open_space_info().is_consider_wheel_mask() &&
         previous_frame->open_space_info().is_stop_near_wheel_mask();
}

bool OpenSpacePathPartition::IsSatisfyParkOutFinishCondition(
    const common::VehicleState& vehicle_state,
    const DestRegionWithAng& dest_region_with_angle) {
  const auto& parking_lot_enu = frame_->get_parking_lot_vertices();
  auto base_line =
      common::math::LineSegment2d(parking_lot_enu[0], parking_lot_enu[3]);
  double base_angle = 0.5 * (std::get<1>(dest_region_with_angle) +
                             std::get<2>(dest_region_with_angle));
  const auto& park_scenario_type =
      frame_->open_space_info()
          .open_space_path_info()
          .open_space_env_structured_info.parking_scenario_type;
  switch (park_scenario_type) {
    case (LEFT_VERTICAL_PARKING_OUT):
    case (LEFT_OBLIQUE_PARKING_OUT): {
      base_line =
          common::math::LineSegment2d(parking_lot_enu[1], parking_lot_enu[0]);
      break;
    }
    case (RIGHT_VERTICAL_PARKING_OUT):
    case (RIGHT_OBLIQUE_PARKING_OUT): {
      base_line =
          common::math::LineSegment2d(parking_lot_enu[3], parking_lot_enu[2]);
      break;
    }
    case (LEFT_LATERAL_PARKING_OUT):
    case (RIGHT_LATERAL_PARKING_OUT): {
      base_line =
          common::math::LineSegment2d(parking_lot_enu[0], parking_lot_enu[3]);
      break;
    }
    default: {
      break;
    }
  }
  bool isParkingOutFinish = false;
  auto finish_status = planning_internal::OpenSpaceDebug::UNKNOWN;
  if (!frame_->IsVehicleStandStill()) {
    AINFO << "vehicle is still moving";
    finish_status = planning_internal::OpenSpaceDebug::VEHICEL_MOVING;
  } else {
    isParkingOutFinish =
        IsVehicleReachDestinationZone(vehicle_state, dest_region_with_angle);
    if (isParkingOutFinish) {
      finish_status = planning_internal::OpenSpaceDebug::REACH_TARGET;
    } else {
      AINFO << "vehicle is not reach destination zone";
      // check vehicle is satisfy other finish condition
      // case 1: gear shift over difine times and angle diff is less than deg
      // threshold
      // case 2: veh rear axis is in base line and angle diff is less
      // than deg threshold
      double ang_diff = std::fabs(
          common::math::AngleDiff(vehicle_state.heading(), base_angle));
      Vec2d vehicle_pos{vehicle_state.x(), vehicle_state.y()};
      bool vehout = (base_line.end() - base_line.start())
                        .CrossProd(vehicle_pos - base_line.start()) >
                    -common::math::kMathEpsilon;
      finish_status = planning_internal::OpenSpaceDebug::FAR_AWAY;
      if (ang_diff > FLAGS_park_out_early_stop_angle_limit) {
        finish_status = planning_internal::OpenSpaceDebug::LARGE_ANGLE;
      } else if (vehout) {
        finish_status = planning_internal::OpenSpaceDebug::OUT_OF_PARK_LOT;
        isParkingOutFinish = true;
      }
      ADEBUG << "ang_diff " << ang_diff << " veh pose is out of slot "
             << vehout;
    }
  }
  task_finish_status_ = finish_status;
  return isParkingOutFinish;
}

bool OpenSpacePathPartition::IsVehicleReachDestinationZone(
    const common::VehicleState& vehicle_state,
    const DestRegionWithAng& dest_region_with_angle) {
  Vec2d vehicle_position(vehicle_state.x(), vehicle_state.y());
  const auto& destregion_polygon = std::get<0>(dest_region_with_angle);
  const double destregion_fromangle = std::get<1>(dest_region_with_angle);
  const double destregion_toangle = std::get<2>(dest_region_with_angle);

  if (destregion_polygon.num_points() < 3) {
    AERROR << "dest region is not valid";
    return false;
  }
  const bool position_in_destregion =
      destregion_polygon.DistanceTo(vehicle_position) <
      open_space_path_partition_config_
          .is_near_destination_distance_threshold();
  bool heading_in_destregion = false;
  double theta_to_destregion = 0.0;
  if (!common::math::AngleInRange(vehicle_state.heading(), destregion_fromangle,
                                  destregion_toangle)) {
    theta_to_destregion =
        std::max(std::fabs(common::math::AngleDiff(vehicle_state.heading(),
                                                   destregion_fromangle)),
                 std::fabs(common::math::AngleDiff(vehicle_state.heading(),
                                                   destregion_toangle)));
  }
  heading_in_destregion =
      theta_to_destregion <
      open_space_path_partition_config_.is_near_destination_theta_threshold();

  return (frame_->IsVehicleStandStill() && position_in_destregion &&
          heading_in_destregion);
}

bool OpenSpacePathPartition::IsVehOnRoad() {
  // if veh is on road and heading is parallel to road, set parking out
  // finished
  double vehicle_s = 0.0;
  double vehicle_l = 0.0;
  const auto vehicle_state = frame_->vehicle_state();
  auto nearby_path =
      frame_->reference_line_info().front().reference_line().GetMapPath();
  if (!nearby_path.GetProjection({vehicle_state.pose().position().x(),
                                  vehicle_state.pose().position().y()},
                                 &vehicle_s, &vehicle_l)) {
    AERROR << "GetProjection failed.";
    return false;
  }
  const auto& path_point = nearby_path.GetSmoothPoint(vehicle_s);
  double diff_angle = common::math::NormalizeAngle(
      fabs(vehicle_state.pose().heading() - path_point.heading()));
  ADEBUG << "diff_angle = " << diff_angle << " vehicle_l " << vehicle_l;
  // TODO(jyw): need optimize, set threshold based on road environment
  static constexpr double kMaxDiffAngle = M_PI / 8.0;
  static constexpr double kMaxDiffL = 0.5;
  return diff_angle < kMaxDiffAngle && fabs(vehicle_l) < kMaxDiffL;
}

void OpenSpacePathPartition::UpdatePathExcutableStatus(
    const PartitionedPath& path, uint32_t* const executable_status_ptr) {
  if (nullptr == executable_status_ptr) {
    return;
  }
  if (IsPathTooShortToBrake(path)) {
    *executable_status_ptr += TOO_SHORT_TO_BRAKE;
  }
  if (IsPathSteerRateLarge(path)) {
    *executable_status_ptr += LARGE_STEER_RATE;
  }
}

void OpenSpacePathPartition::UpdateCollisionDistance(
    const PartitionedPath& path, double* const collision_distance_ptr) {
  if (nullptr == collision_distance_ptr) {
    return;
  }
  *collision_distance_ptr = INFINITY;
  const auto& match_path = path.path_set[path.path_idx].first;
  for (size_t i = path.point_idx; i < match_path.size(); ++i) {
    const auto& path_point = match_path[i];
    if (common::math::CheckCollisionWithVehiclePolygon2d(
            path_point.x(), path_point.y(), path_point.theta(),
            frame_->open_space_info()
                .open_space_path_info()
                .obstacles_segments_vec)) {
      *collision_distance_ptr = path_point.s() - match_path[path.point_idx].s();
      break;
    }
  }
}

bool OpenSpacePathPartition::IsPathTooShortToBrake(
    const PartitionedPath& path) {
  if (path.path_set.empty()) {
    return false;
  }
  const auto& match_path = path.path_set[path.path_idx].first;
  if (match_path.empty() || path.point_idx + 1 >= match_path.size()) {
    return false;
  }
  if (frame_->IsVehicleStandStill()) {
    return false;
  }
  static constexpr double kLonDisThreshold = 0.1;
  const auto& path_gear = path.path_set[path.path_idx].second;
  const auto& vehicle_state = frame_->local_view().GetVehicleState();
  const double path_left_length = (path_gear != vehicle_state->gear())
                                      ? 0.0
                                      : kLonDisThreshold +
                                            match_path.back().s() -
                                            match_path[path.point_idx].s();
  const double spd = fabs(vehicle_state->linear_velocity());
  const bool unable_to_stop =
      2 * abs(FLAGS_pause_brake_acceleration) * path_left_length < spd * spd;
  ADEBUG << "spd " << spd << " path_left_length " << path_left_length;
  return unable_to_stop;
}

bool OpenSpacePathPartition::IsPathSteerRateLarge(const PartitionedPath& path) {
  if (path.path_set.empty()) {
    return false;
  }
  const auto& match_path = path.path_set[path.path_idx].first;
  const auto& path_gear = path.path_set[path.path_idx].second;
  if (match_path.empty() || path.point_idx + 1 >= match_path.size()) {
    return false;
  }
  if (frame_->IsVehicleStandStill()) {
    return false;
  }
  const auto& vehicle_state = frame_->vehicle_state();
  double cur_v = abs(vehicle_state.linear_velocity());
  // assume adc will excute path with cur spd
  common::VehicleParam vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  auto kappa_to_steer = [&](double kappa) {
    return atan(kappa * vehicle_param.wheel_base()) *
           vehicle_param.steer_ratio();
  };

  auto steer_to_kappa = [&](double steer) {
    return tan(steer / vehicle_param.steer_ratio()) /
           vehicle_param.wheel_base();
  };
  const double w =
      common::math::InterpolationOne(
          cur_v,
          vehicle_param_.steer_wheel_speed_segment().vehicle_speed_segment(),
          vehicle_param_.steer_wheel_speed_segment()
              .steering_wheel_speed_limit_segment()) /
      180.0 * M_PI;
  // yaw boundary
  double yaw_lower{vehicle_state.heading()};
  double yaw_upper{vehicle_state.heading()};
  // steer boundary
  double steer_lower{kappa_to_steer(vehicle_state.kappa())};
  double steer_upper{steer_lower};
  static constexpr double kLookAheadDis = 1.0;
  // 10 deg is not acceptable
  // TODO(jyw): move it to config
  static constexpr double kYawErrorThreshold = 10.0 / 180.0 * M_PI;
  double dis = 0;
  size_t idx = path.point_idx + 1;
  while (dis < kLookAheadDis && idx + 1 < match_path.size()) {
    const auto& prev_point = match_path[idx - 1];
    const auto& cur_point = match_path[idx];
    const double delta_s = cur_point.s() - prev_point.s();
    const double delta_t = delta_s / cur_v;
    const double delta_steer = w * delta_t;
    steer_lower = std::max(steer_lower - delta_steer,
                           -1 * vehicle_param_.max_steer_angle());
    steer_upper =
        std::min(steer_upper + delta_steer, vehicle_param_.max_steer_angle());
    if (path_gear == soc::Chassis::GEAR_DRIVE) {
      yaw_lower += steer_to_kappa(steer_lower) * delta_s;
      yaw_upper += steer_to_kappa(steer_upper) * delta_s;
    } else {
      yaw_lower -= steer_to_kappa(steer_upper) * delta_s;
      yaw_upper -= steer_to_kappa(steer_lower) * delta_s;
    }

    if (yaw_lower - cur_point.theta() > kYawErrorThreshold ||
        cur_point.theta() - yaw_upper > kYawErrorThreshold) {
      ADEBUG << "cur point theta " << cur_point.theta() << " " << yaw_lower
             << " " << yaw_upper;
      ADEBUG << "steer " << kappa_to_steer(vehicle_state.kappa()) << " "
             << steer_lower << " " << steer_upper;
      ADEBUG << "kappa " << steer_to_kappa(steer_lower) << " "
             << steer_to_kappa(steer_upper);
      ADEBUG << "delta_s " << delta_s << " delta_steer  " << delta_steer;
      ADEBUG << "dis " << dis << " idx " << idx << " path.point_idx "
             << path.point_idx;
      return true;
    }
    dis += delta_s;
    ++idx;
  }
  return false;
}

void OpenSpacePathPartition::UpdateInfoForPreFinishCondition(
    const OpenSpacePathDecision& openspace_path_decision,
    PathGearPair* const chosen_partitioned_path) {
  if (nullptr == chosen_partitioned_path) {
    AERROR << "current_partitioned_path is nullptr";
    return;
  }
  const auto& park_scenario_type =
      frame_->open_space_info()
          .open_space_path_info()
          .open_space_env_structured_info.parking_scenario_type;
  const bool is_lateral_park_in =
      park_scenario_type == ParkingScenarioType::LEFT_LATERAL_PARKING_IN ||
      park_scenario_type == ParkingScenarioType::RIGHT_LATERAL_PARKING_IN;
  if (!is_lateral_park_in) {
    return;
  }
  if (openspace_path_decision != OpenSpacePathDecision::PREPARE_FINISH &&
      fabs(end_point_sl_.l()) > open_space_path_partition_config_
                                    .is_near_destination_distance_threshold()) {
    ADEBUG << "current path is not pass through target areas";
    return;
  }
  // set spd limit info if path pass through target areas
  for (const auto& point : chosen_partitioned_path->first) {
    if (point.s() > end_point_sl_.s()) {
      frame_->mutable_open_space_info()
          ->mutable_spd_limit_points()
          ->emplace_back(point);
    }
  }
  // clip path if adc in pre finish status
#ifdef FOR_BAIDU_SIMULATION
  static constexpr double kPreFinishClipLength = -0.1;  // meters
#else
  static constexpr double kPreFinishClipLength = 0.1;  // meters
#endif
  if (openspace_path_decision != OpenSpacePathDecision::PREPARE_FINISH) {
    return;
  }
  if (!is_clipped_) {
    clipped_point_ = chosen_partitioned_path->first.back();
    for (const auto& point : chosen_partitioned_path->first) {
      if (point.s() > kPreFinishClipLength) {
        clipped_point_ = point;
        break;
      }
    }
  }
  auto iter = chosen_partitioned_path->first.begin();
  bool is_reach_clipped = false;
  while (iter != chosen_partitioned_path->first.end()) {
    if (is_reach_clipped) {
      iter = chosen_partitioned_path->first.erase(iter);
    } else {
      is_reach_clipped = DiscretizedPath::IsSamePoint(*iter, clipped_point_);
      iter++;
    }
  }
  if (!is_reach_clipped) {
    chosen_partitioned_path->first.GenerateStopPath(
        chosen_partitioned_path->first.begin()->x(),
        chosen_partitioned_path->first.begin()->y(),
        chosen_partitioned_path->first.begin()->theta(),
        chosen_partitioned_path->first.begin()->kappa());
    ADEBUG << "failed to clip path, generate stop path instead";
  }
  is_clipped_ = true;
}

bool OpenSpacePathPartition::GetEndPointSLInCurrentPath(
    const std::vector<PathGearPair>& partitioned_path, const size_t path_idx,
    common::SLPoint* const end_point_sl_ptr) {
  if (nullptr == end_point_sl_ptr) {
    return false;
  }
  static constexpr double kEps = 1e-3;
  if (partitioned_path.empty() || partitioned_path.back().first.empty() ||
      partitioned_path.back().first.back().s() < kEps) {
    ADEBUG << "partition_path is not valid";
    return false;
  }
  const auto& current_partitioned_path = partitioned_path.at(path_idx);
  const auto& end_pose = partitioned_path.back().first.back();
  if (!current_partitioned_path.first.XYToSL(end_pose.x(), end_pose.y(),
                                             end_point_sl_ptr)) {
    ADEBUG << "end pose projection failed";
    return false;
  }
  return true;
}

common::PathPoint OpenSpacePathPartition::TaskTargetPose(
    const PartitionedPath& chosen_path) {
  // openspace dose not ensure that adc reaches target pose precisely (reach target region).
  // while checking task finished by evaluating the difference between adc pose and path final pose
  static const double kEps = 1e-3;
  // ignore trace path
  const bool is_apa_path =
      chosen_path.path_type !=
          planning_internal::PathUpdateStatus::TRACE_PATH &&
      chosen_path.path_type != planning_internal::PathUpdateStatus::CRUISE_PATH;
  const bool has_valid_path =
      is_apa_path && !chosen_path.path_set.empty() &&
      !chosen_path.path_set.back().first.empty() &&
      chosen_path.path_set.back().first.back().s() > kEps;
  return has_valid_path
             ? chosen_path.path_set.back().first.back()
             : frame_->open_space_info().open_space_path_info().end_point;
}

bool OpenSpacePathPartition::IsAdcExecuteLastPartPath(
    const PartitionedPath& chosen_path) {
  if (chosen_path.path_set.empty()) {
    return false;
  }
  const int path_size = static_cast<int>(chosen_path.path_set.size());
  const int path_idx = static_cast<int>(chosen_path.path_idx);
  const auto path_gear = chosen_path.path_set.at(path_idx).second;
  return (path_idx + 1 == path_size) &&
         frame_->vehicle_state().gear() == path_gear;
}

bool OpenSpacePathPartition::IsMirrorFold(
    const TL::planning::OpenSpacePathInfo& open_space_path_info) {
  if (!frame_->local_view().HasFunctionManagerIn() ||
      frame_->local_view()
              .GetFunctionManagerIn()
              ->fct_avp_in()
              .sys_run_state() == functionmanager::AvpFctIn::PARKSTART) {
    return false;
  }
  const auto& open_space_env_structured_info =
      open_space_path_info.open_space_env_structured_info;
  const auto& park_scenario_type =
      open_space_env_structured_info.parking_scenario_type;
  const bool is_narrow_spot_scenario =
      (open_space_path_info.open_space_env_structured_info
           .parking_scenario_diffculty_type &
       NARROW_SPOT_SCENARIO) != 0;
  if (IsVerticalParkOut(park_scenario_type) &&
      (FLAGS_force_mirror_fold || is_narrow_spot_scenario)) {
    return true;
  }
  const auto& previous_frame = injector_->frame_history()->Latest();
  if (!FLAGS_force_mirror_fold &&
      (!is_narrow_spot_scenario ||
       (previous_frame != nullptr &&
        !IsAdcExecuteLastPartPath(
            previous_frame->open_space_info().partitioned_paths())) ||
       (!open_space_env_structured_info.is_parking_inwards &&
        frame_->vehicle_state().gear() != soc::Chassis::GEAR_REVERSE) ||
       (open_space_env_structured_info.is_parking_inwards &&
        frame_->vehicle_state().gear() != soc::Chassis::GEAR_DRIVE))) {
    return false;
  }
  const bool is_vertical_park_in =
      park_scenario_type == LEFT_VERTICAL_PARKING_IN ||
      park_scenario_type == RIGHT_VERTICAL_PARKING_IN ||
      park_scenario_type == LEFT_OBLIQUE_PARKING_IN ||
      park_scenario_type == RIGHT_OBLIQUE_PARKING_IN;
  if (!is_vertical_park_in) {
    return false;
  }
  auto adc_point = Vec2d(ego_x_, ego_y_);
  adc_point -= open_space_path_info.origin;
  adc_point.SelfRotate(-open_space_path_info.rotate_angle);
  return adc_point.y() < (open_space_env_structured_info.is_parking_inwards
                              ? -config_.open_space_path_partition_config()
                                     .fold_mirror_depth_threshold()
                              : config_.open_space_path_partition_config()
                                    .fold_mirror_depth_threshold());
}

double OpenSpacePathPartition::SetGearShiftDis() {
  auto gear_shift_dist =
      open_space_path_partition_config_.longitudinal_offset_to_midpoint();
  static constexpr double kExtraLongitudinalOffset = 0.3;
  common::math::Vec2d vehicle_rear_axis_center{frame_->vehicle_state().x(),
                                               frame_->vehicle_state().y()};
  const auto& vehicle_param =
      TL::common::VehicleConfigHelper::GetConfig().vehicle_param();
  common::math::Vec2d vehicle_geometric_center =
      vehicle_rear_axis_center + (vehicle_param.front_edge_to_center() -
                                  vehicle_param.back_edge_to_center()) /
                                     2 *
                                     common::math::Vec2d::CreateUnitVec2d(
                                         frame_->vehicle_state().heading());
  common::math::Box2d ego_box(vehicle_geometric_center,
                              frame_->vehicle_state().heading(),
                              vehicle_param.length(), vehicle_param.width());
  if (!frame_->open_space_info().speed_bump_segments().empty()) {
    for (const auto& speed_bump_seg :
         frame_->open_space_info().speed_bump_segments()) {
      if (ego_box.HasOverlap(speed_bump_seg)) {
        gear_shift_dist = kExtraLongitudinalOffset;
        AINFO << "has speed bump nearby, slack gear shift threshold";
        break;
      }
    }
  }
  return gear_shift_dist;
}

}  // namespace planning
}  // namespace TL
