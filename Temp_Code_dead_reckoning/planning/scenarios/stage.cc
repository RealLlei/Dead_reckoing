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

/**
 * @file
 **/

#include "planning/scenarios/stage.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>

#include "absl/strings/match.h"
#include "common/file/log.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/time/clock.h"
#include "common/util/point_factory.h"
#include "glog/logging.h"
#include "planning/common/path/fallback_path.h"
#include "planning/common/planning_context.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/speed_profile_generator.h"
#include "planning/common/trajectory/publishable_trajectory.h"
#include "planning/common/util/common.h"
#include "planning/reference_line_info_decider/common/reference_line_info_decider_common.h"
#include "planning/tasks/deciders/lane_change_decider/lane_change_decider.h"
#include "planning/tasks/optimizers/open_space_path_generation/open_space_path_provider.h"
#include "planning/tasks/task.h"
#include "planning/tasks/task_factory.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/planning/planning_status.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {
namespace scenario {

using TL::common::Clock;
using TL::common::ErrorCode;
using TL::common::SLPoint;
using TL::common::Status;
using TL::common::TrajectoryPoint;
using TL::common::util::PointFactory;

namespace {
constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
}  // namespace

Stage::Stage(const ScenarioConfig::StageConfig& config,
             const std::shared_ptr<DependencyInjector>& injector)
    : config_(config),
      next_stage_(config_.stage_type()),
      name_(ScenarioStatus::StageType_Name(config_.stage_type())),
      injector_(injector) {
  // set stage_type in PlanningContext
  injector->planning_context()
      ->mutable_planning_status()
      ->mutable_scenario()
      ->set_stage_type(stage_type());

  std::unordered_map<TaskConfig::TaskType, const TaskConfig*, std::hash<int>>
      config_map;
  for (const auto& task_config : config_.task_config()) {
    config_map[task_config.task_type()] = &task_config;
  }
  for (int i = 0; i < config_.task_type_size(); ++i) {
    auto task_type = config_.task_type(i);
    ACHECK(config_map.find(task_type) != config_map.end())
        << "Task: " << TaskConfig::TaskType_Name(task_type)
        << " used but not configured";
    auto iter = tasks_.find(task_type);
    if (iter == tasks_.end()) {
      auto ptr = TaskFactory::CreateTask(*config_map[task_type], injector_);
      task_list_.push_back(ptr.get());
      tasks_[task_type] = std::move(ptr);
    } else {
      task_list_.push_back(iter->second.get());
    }
  }

  use_joint_optizimizer_ =
      std::any_of(task_list_.begin(), task_list_.end(), [&](const auto* task) {
        return task != nullptr &&
               absl::StrContains(task->Name(), "JOINT_OPTIMIZER");
      });
}

const std::string& Stage::Name() const {
  return name_;
}

Task* Stage::FindTask(TaskConfig::TaskType task_type) const {
  auto iter = tasks_.find(task_type);
  if (iter == tasks_.end()) {
    return nullptr;
  }
  return iter->second.get();
}

common::Status Stage::ExecuteTaskOnOpenSpace(Frame* frame,
                                             const bool init_open_space) {
  if (nullptr == frame) {
    const std::string msg = "frame is nullptr!";
    AERROR << msg;
    return Status(ErrorCode::CORE_PLANNING_PARKINGPARKINGSTAGE_ERROR, msg);
  }

  auto ret = common::Status::OK();
  auto trajectory_type = ADCTrajectory::NORMAL;
  auto parking_type = injector_->planning_context()
                          ->planning_status()
                          .avp_status()
                          .parking_type();
  bool is_straight_path = (parking_type == AVPStatus::DIRECT_FORWARD) ||
                          (parking_type == AVPStatus::DIRECT_BACKWARD);
  const bool in_pre_plan =
      frame->local_view()
          .GetFunctionManagerIn()
          ->fct_avp_in()
          .sys_run_state() == functionmanager::AvpFctIn::PARKSTART;
  for (auto* task : task_list_) {
    if (!absl::StrContains(task->Name(), "OPEN_SPACE")) {
      continue;
    }
    const auto start_timestamp = common::Clock::NowInSeconds();
    if (is_straight_path) {
      if (task->Name() == "OPEN_SPACE_ROI_DECIDER" ||
          task->Name() == "OPEN_SPACE_PATH_PROVIDER" ||
          task->Name() == "OPEN_SPACE_PATH_PARTITION") {
        continue;
      }
    } else if (task->Name() == "OPEN_SPACE_STRAIGHT_PATH") {
      continue;
    }
    if (init_open_space) {
      ret = task->Reset();
      if (ret != common::Status::OK()) {
        return ret;
      }
    }
    ret = task->Execute(frame);

    const auto end_timestamp = Clock::NowInSeconds();
    double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    if (absl::StrContains(task->Name(), "OPEN_SPACE_PATH_PROVIDER")) {
      const auto* provider_task = dynamic_cast<OpenSpacePathProvider*>(task);
      time_diff_ms += provider_task->GetSubThreadCostTimeMs();
    }

    ADEBUG << "task[" << task->Name() << "] time spent: " << time_diff_ms
           << " ms.";
    RecordOpenSpaceTaskConsumeTime(task->Name(), time_diff_ms,
                                   frame->mutable_open_space_info());
    if (!ret.ok()) {
      AERROR << "Failed to run tasks[" << task->Name()
             << "], Error message: " << ret.error_message();
      GeneratePauseTrajectory(frame);
      trajectory_type = ADCTrajectory::SHORT_PATH;
      frame->mutable_reference_line_info()->front().set_trajectory_type(
          trajectory_type);
      return in_pre_plan ? common::Status::OK() : ret;
    }
  }

  if (injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_open_space()
          ->current_part_path_length() < FLAGS_openspace_short_path_limit) {
    trajectory_type = ADCTrajectory::SHORT_PATH;
  }

  const auto& trajectory =
      frame->open_space_info().speed_optimizer_trajectory().first;
  const auto& gear =
      frame->open_space_info().speed_optimizer_trajectory().second;

  PublishableTrajectory publishable_trajectory(
      frame->open_space_info().global_publish_timestamp(), trajectory);
  auto publishable_traj_and_gear =
      std::make_pair(std::move(publishable_trajectory), gear);
  *(frame->mutable_open_space_info()->mutable_publishable_trajectory_data()) =
      std::move(publishable_traj_and_gear);
  frame->SetTargetGear(gear);
  frame->mutable_reference_line_info()->front().set_trajectory_type(
      trajectory_type);
  return ret;
}

Stage::StageStatus Stage::FinishScenario() {
  next_stage_ = ScenarioStatus::NO_STAGE;
  return Stage::FINISHED;
}

void Stage::RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                            const std::string& name,
                            const double time_diff_ms) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }
  if (reference_line_info == nullptr) {
    AERROR << "Reference line info is null.";
    return;
  }

  auto* ptr_latency_stats = reference_line_info->mutable_latency_stats();

  auto* ptr_stats = ptr_latency_stats->add_task_stats();
  ptr_stats->set_name(name);
  ptr_stats->set_time_ms(time_diff_ms);
}

double Stage::PauseACC(const common::VehicleState& vehicle_state) {
  double brake_acceleration = 0;
  switch (vehicle_state.gear()) {
    case soc::Chassis::GEAR_DRIVE:
      brake_acceleration = (1 == common::math::double_type::Compare(
                                     vehicle_state.linear_velocity(), 0.0))
                               ? -FLAGS_pause_brake_acceleration
                               : 0.0;
      break;
    case soc::Chassis::GEAR_REVERSE:
      brake_acceleration = (-1 == common::math::double_type::Compare(
                                      vehicle_state.linear_velocity(), 0.0))
                               ? FLAGS_pause_brake_acceleration
                               : 0.0;
      break;
    default:
      break;
  }
  return brake_acceleration;
}

void Stage::GeneratePauseTrajectory(Frame* frame) {
  DiscretizedTrajectory trajectory;
  const auto& vehicle_state = *(injector_->vehicle_state());
  const auto& start_point = frame->PlanningStartPoint();
  auto last_gear = vehicle_state.gear();
  if (nullptr != injector_ && nullptr != injector_->frame_history() &&
      nullptr != injector_->frame_history()->Latest()) {
    last_gear = injector_->frame_history()->Latest()->GetTargetGear();
  }
  last_gear = last_gear == soc::Chassis::GEAR_NEUTRAL
                  ? soc::Chassis::GEAR_PARKING
                  : last_gear;
  double brake_acceleration = PauseACC(vehicle_state);
  trajectory.SetStopTrajectory(
      start_point.path_point().x(), start_point.path_point().y(),
      start_point.path_point().theta(), start_point.path_point().kappa(),
      start_point.relative_time(), brake_acceleration);
  PublishableTrajectory publishable_trajectory(
      frame->open_space_info().global_publish_timestamp(), trajectory);
  auto publishable_traj_and_gear =
      std::make_pair(std::move(publishable_trajectory), last_gear);
  if (frame->local_view()
              .GetFunctionManagerIn()
              ->fct_avp_in()
              .sys_run_state() == functionmanager::AvpFctIn_SysRunState_PAUSE &&
      (injector_->frame_history()->Latest() != nullptr)) {
    *(frame->mutable_open_space_info()) =
        injector_->frame_history()->Latest()->open_space_info();
  }
  *(frame->mutable_open_space_info()->mutable_publishable_trajectory_data()) =
      std::move(publishable_traj_and_gear);

  frame->SetTargetGear(last_gear);
}

bool Stage::GeneratePauseTrajectory(
    const common::TrajectoryPoint& planning_start_point, Frame* frame) {
  bool is_set_trajectory_done = false;
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    if (!reference_line_info.IsDrivable()) {
      AERROR << "The generated path is not drivable";
      continue;
    }
    auto& vehicle_state = *(injector_->vehicle_state());
    double brake_acceleration = PauseACC(vehicle_state);
    DiscretizedTrajectory trajectory;
    auto last_gear = vehicle_state.gear();
    if (nullptr != injector_ && nullptr != injector_->frame_history() &&
        nullptr != injector_->frame_history()->Latest()) {
      last_gear = injector_->frame_history()->Latest()->GetTargetGear();
    }
    last_gear = last_gear == soc::Chassis::GEAR_NEUTRAL
                    ? soc::Chassis::GEAR_PARKING
                    : last_gear;
    trajectory.SetStopTrajectory(planning_start_point.path_point().x(),
                                 planning_start_point.path_point().y(),
                                 planning_start_point.path_point().theta(),
                                 planning_start_point.path_point().kappa(),
                                 planning_start_point.relative_time(),
                                 brake_acceleration);
    reference_line_info.SetTrajectory(trajectory);
    reference_line_info.SetDrivable(true);
    frame->SetTargetGear(last_gear);
    is_set_trajectory_done = true;
    return is_set_trajectory_done;
  }
  return is_set_trajectory_done;
}

void Stage::RecordObstacleDebugInfo(ReferenceLineInfo* reference_line_info) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }
  auto* ptr_debug = reference_line_info->mutable_debug();

  auto* const path_decision = reference_line_info->path_decision();
  for (const auto* const obstacle : path_decision->obstacles().Items()) {
    auto* obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();
    obstacle_debug->set_id(obstacle->Id());
    obstacle_debug->mutable_sl_boundary()->CopyFrom(
        obstacle->PerceptionSLBoundary());
    const auto& decider_tags = obstacle->decider_tags();
    const auto& decisions = obstacle->decisions();
    if (decider_tags.size() != decisions.size()) {
      AERROR << "decider_tags size: " << decider_tags.size()
             << " different from decisions size:" << decisions.size();
    }
    for (size_t i = 0; i < decider_tags.size(); ++i) {
      auto* decision_tag = obstacle_debug->add_decision_tag();
      decision_tag->set_decider_tag(decider_tags[i]);
      decision_tag->mutable_decision()->CopyFrom(decisions[i]);
    }
  }
}

void Stage::ResetFilter() {
  smooth_mean_dx_ = 0.0;
  smooth_mean_dy_ = 0.0;
  smooth_mean_dtheta_ = 0.0;
  adjust_stitch_dx_filter_.Reset();
  adjust_stitch_dy_filter_.Reset();
  adjust_stitch_dtheta_filter_.Reset();
}

bool Stage::AdjustStitchingTrajectoryPoints(
    const Frame* const frame, const ReferenceLineInfo& reference_line_info) {
  auto* sti_traj_points = frame->GetStitchingTrajectoryPointsPtr();
  if (sti_traj_points == nullptr || sti_traj_points->empty()) {
    ResetFilter();
    return false;
  }

  const auto init_point_iter = sti_traj_points->rbegin();
  const auto* last_ref_line =
      injector_->frame_history()->Latest()->DriveReferenceLineInfo();

  common::SLPoint cur_state_to_last_ref;
  common::SLPoint cur_state_to_cur_ref;
  if (!init_point_iter->has_path_point() ||
      !reference_line_info.reference_line().XYToSL(
          common::math::Vec2d(init_point_iter->path_point().x(),
                              init_point_iter->path_point().y()),
          &cur_state_to_cur_ref) ||
      !last_ref_line->reference_line().XYToSL(
          common::math::Vec2d(init_point_iter->path_point().x(),
                              init_point_iter->path_point().y()),
          &cur_state_to_last_ref)) {
    ResetFilter();
    return false;
  }
  const auto& cur_ref_path_point =
      reference_line_info.reference_line().map_path().GetSmoothPoint(
          cur_state_to_cur_ref.s());
  const auto& last_ref_path_point =
      last_ref_line->reference_line().map_path().GetSmoothPoint(
          cur_state_to_last_ref.s());

  const double last_heading_error =
      init_point_iter->path_point().theta() - last_ref_path_point.heading();
  const double cur_heading_error =
      init_point_iter->path_point().theta() - cur_ref_path_point.heading();
  const bool is_need_rotate =
      cur_state_to_last_ref.l() * (cur_heading_error - last_heading_error) > 0;
  const bool is_need_transform =
      std::abs(cur_state_to_cur_ref.l()) > std::abs(cur_state_to_last_ref.l());

  common::math::Vec2d transform(
      cur_ref_path_point.x() - last_ref_path_point.x(),
      cur_ref_path_point.y() - last_ref_path_point.y());
  const auto rotate = common::math::AngleDiff(last_ref_path_point.heading(),
                                              cur_ref_path_point.heading());

  static constexpr double transform_threshold = 0.5;
  static constexpr double rotate_threshold = M_PI / 36.0;
  static constexpr double l_threshold = 1.0;

  if (transform.Length() < transform_threshold &&
      fabs(rotate) < rotate_threshold &&
      fabs(cur_state_to_cur_ref.l()) < l_threshold) {
    smooth_mean_dx_ = adjust_stitch_dx_filter_.UpdateForAll(transform.x());
    smooth_mean_dy_ = adjust_stitch_dy_filter_.UpdateForAll(transform.y());
    smooth_mean_dtheta_ = adjust_stitch_dtheta_filter_.UpdateForAll(rotate);
  }
  common::math::Vec2d smooth_transform(smooth_mean_dx_, smooth_mean_dy_);
  const double clamp_length =
      common::math::Clamp(smooth_transform.Length(), 0.0, transform_threshold);
  smooth_transform.Normalize();
  smooth_transform *= clamp_length;
  double mean_dtheta = common::math::Clamp(smooth_mean_dtheta_,
                                           -rotate_threshold, rotate_threshold);
  ADEBUG << "transform.x(): " << transform.x()
         << ", transform.y(): " << transform.y()
         << ", smooth x: " << smooth_transform.x()
         << ", smooth y: " << smooth_transform.y() << ", theta diff: "
         << common::math::AngleDiff(last_ref_path_point.heading(),
                                    cur_ref_path_point.heading())
         << ", smooth theta diff: " << mean_dtheta;

  for (auto& sti_traj_point : *sti_traj_points) {
    if (is_need_transform || is_need_rotate) {
      double x_new = last_ref_path_point.x();
      double y_new = last_ref_path_point.y();
      x_new += is_need_transform ? smooth_transform.x() : 0.0;
      y_new += is_need_transform ? smooth_transform.y() : 0.0;
      mean_dtheta = is_need_rotate ? mean_dtheta : 0.0;
      auto delta_x = sti_traj_point.path_point().x() - last_ref_path_point.x();
      auto delta_y = sti_traj_point.path_point().y() - last_ref_path_point.y();
      Eigen::Vector2d rotated_point = common::math::RotateVector2d(
          {delta_x, delta_y}, common::math::NormalizeAngle(mean_dtheta));
      x_new += rotated_point.x();
      y_new += rotated_point.y();
      sti_traj_point.mutable_path_point()->set_x(x_new);
      sti_traj_point.mutable_path_point()->set_y(y_new);
      sti_traj_point.mutable_path_point()->set_theta(
          common::math::NormalizeAngle(sti_traj_point.path_point().theta() +
                                       mean_dtheta));
    }
  }

  return true;
}

std::pair<Stage::StageStatus, common::Status> Stage::ExecuteTaskOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  bool has_drivable_reference_line = false;

  ADEBUG << "Number of reference lines:\t"
         << frame->mutable_reference_line_info()->size();

  unsigned int count = 0;
  auto cur_status = Status::OK();
  RepeatedPtrField<TrajectoryPoint> sti_traj_points_temp;
  if (frame->GetStitchingTrajectoryPointsPtr() != nullptr) {
    sti_traj_points_temp = *frame->GetStitchingTrajectoryPointsPtr();
  }

  const bool enable_smooth_select_reference_line =
      FLAGS_enable_smooth_select_reference_line &&
      frame->local_view().HasFunctionManagerOut() &&
      frame->local_view().GetFunctionManagerOut()->has_hdmap_sub_state() &&
      frame->local_view().GetFunctionManagerOut()->hdmap_sub_state() ==
          functionmanager::HdmapSubState::MAP_FUSION_TYPE;

  const bool is_sti_traj_points_valid = !sti_traj_points_temp.empty();
  const double plan_on_refer_start_time = Clock::NowInSeconds();
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    if (enable_smooth_select_reference_line) {
      reference_line_info.GetProjection();
    }

    // TODO(SHU): need refactor
    if (count++ == frame->mutable_reference_line_info()->size()) {
      break;
    }
    if (!reference_line_info.IsDrivable()) {
      continue;
    }
    if (has_drivable_reference_line) {
      reference_line_info.SetDrivable(false);
      ADEBUG << "Find drivable referenceline.";
      break;
    }
    ADEBUG << "No: [" << count << "] Reference Line inludes ";
    // adjust stitching traj points

    auto is_ref_line_considered = false;
    const bool fsm_state_check =
        frame->local_view().HasFunctionManagerOut() &&
        frame->local_view().GetFunctionManagerOut()->has_fsm_state() &&
        frame->local_view().GetFunctionManagerOut()->fsm_state() ==
            functionmanager::MachineStateType::HDMAP_TYPE;
    const bool frame_history_check =
        injector_ != nullptr && injector_->frame_history() != nullptr &&
        injector_->frame_history()->Latest() != nullptr &&
        injector_->frame_history()->Latest()->DriveReferenceLineInfo() !=
            nullptr &&
        injector_->frame_history()
            ->Latest()
            ->local_view()
            .HasFunctionManagerOut() &&
        injector_->frame_history()
            ->Latest()
            ->local_view()
            .GetFunctionManagerOut()
            ->has_localization_maptype();
    const bool final_map_type_check =
        fsm_state_check && frame_history_check &&
        frame->local_view()
            .GetFunctionManagerOut()
            ->has_localization_maptype() &&
        frame->local_view().GetFunctionManagerOut()->localization_maptype() ==
            injector_->frame_history()
                ->Latest()
                ->local_view()
                .GetFunctionManagerOut()
                ->localization_maptype();
    const bool driving_mode_check = frame->vehicle_state().has_driving_mode() &&
                                    (frame->vehicle_state().driving_mode() ==
                                         soc::Chassis::COMPLETE_AUTO_DRIVE ||
                                     frame->vehicle_state().driving_mode() ==
                                         soc::Chassis::AUTO_STEER_ONLY);
    constexpr static double curv_check_distance = 20.0;
    constexpr static double large_kappa_threshold = 0.01;
    constexpr static double small_kappa_threshold = 0.001;
    bool curv_check = fabs(reference_line_info.reference_line()
                               .GetNearestReferencePoint(
                                   reference_line_info.GetAdcSLPoint().s() +
                                   curv_check_distance)
                               .kappa()) < large_kappa_threshold;
    if (!last_curv_check_ && curv_check) {
      curv_check = fabs(reference_line_info.reference_line()
                            .GetNearestReferencePoint(
                                reference_line_info.GetAdcSLPoint().s())
                            .kappa()) < small_kappa_threshold;
    }
    last_curv_check_ = curv_check;
    if (is_sti_traj_points_valid && final_map_type_check &&
        driving_mode_check && curv_check) {
      if (AdjustStitchingTrajectoryPoints(frame, reference_line_info)) {
        is_ref_line_considered = true;
        auto* sti_traj_points = frame->GetStitchingTrajectoryPointsPtr();
        frame->SetPlanningStartPoint(*sti_traj_points->rbegin());
      }
    } else {
      ResetFilter();
    }

    for (const auto& id : reference_line_info.TargetLaneId()) {
      ADEBUG << id.DebugString();
    }
    ADEBUG << "IsChangeLanePath: " << reference_line_info.IsChangeLanePath();

    auto cur_status =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);

    if (cur_status.ok()) {
      if (reference_line_info.IsChangeLanePath()) {
        ADEBUG << "reference line is lane change ref."
               << "reference line info cost: " << reference_line_info.Cost();
        if (reference_line_info.Cost() < FLAGS_straight_forward_line_cost &&
            !is_lane_change_prepare_overtime_) {
          // If the path and speed optimization succeed on target lane while
          // under smart lane-change
          has_drivable_reference_line = true;
          reference_line_info.SetDrivable(true);
          LaneChangeDecider::IsFailedLaneChange(true, frame,
                                                injector_->planning_context());
          ADEBUG << "\tclear for lane change";
        } else {
          reference_line_info.SetDrivable(false);
          const std::string msg = "stage reference line info decider error";
          LaneChangeDecider::IsFailedLaneChange(
              false, frame, injector_->planning_context(), msg);
          ADEBUG << "\tlane change failed";
        }
      } else {
        ADEBUG << "reference line is NOT lane change ref.";
        has_drivable_reference_line = true;
      }
    } else {
      ADEBUG << "planning is failed.";
      reference_line_info.SetDrivable(false);
      LaneChangeDecider::IsFailedLaneChange(false, frame,
                                            injector_->planning_context(),
                                            cur_status.error_message());

      // restore StitchingTrajectoryPoints if task run fail
      if (is_ref_line_considered) {
        frame->SetStitchingTrajectoryPoints(sti_traj_points_temp);
        frame->SetPlanningStartPoint(*sti_traj_points_temp.rbegin());
      }
    }
  }
  const double plan_on_refer_end_time =
      (Clock::NowInSeconds() - plan_on_refer_start_time) * 1000;
  for (auto& reference_line_info : *(frame->mutable_reference_line_info())) {
    auto* ptr_latency_stats = reference_line_info.mutable_latency_stats();
    auto* ptr_stats = ptr_latency_stats->add_task_stats();
    ptr_stats->set_name("plan_on_reference");
    ptr_stats->set_time_ms(plan_on_refer_end_time);
  }
  bool is_success = has_drivable_reference_line;
  // 暂时屏蔽城市场景path故障检测
  // if (frame != nullptr && frame->local_view().HasFunctionManagerOut()) {
  //   const auto& fct_out = frame->local_view().GetFunctionManagerOut();
  //   if (fct_out->has_hdmap_sub_state() &&
  //       fct_out->hdmap_sub_state() !=
  //           functionmanager::HdmapSubState::LOCAL_HDMAP_TYPE) {
  //     is_success = is_success && frame->IsPathValid();
  //   }
  // }

  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_function_manager_out()
      ->mutable_nnp_fct_out()
      ->mutable_nnp_software_fault()
      ->set_plan_trajectory_success(is_success);

  DealHmiChangeLaneStatus(frame);
  ADEBUG << std::boolalpha << "is_lane_change_prepare_overtime_: "
         << is_lane_change_prepare_overtime_;
  ADEBUG << "hmi change lane status: "
         << functionmanager::HmiChangeLaneStatus_Name(
                injector_->planning_context()
                    ->mutable_planning_status()
                    ->mutable_change_lane()
                    ->hmi_change_lane_status());

  return has_drivable_reference_line
             ? std::make_pair(StageStatus::RUNNING, cur_status)
             : std::make_pair(StageStatus::ERROR, cur_status);
}

Status Stage::PlanOnReferenceLine(const TrajectoryPoint& planning_start_point,
                                  Frame* frame,
                                  ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->IsChangeLanePath()) {
    reference_line_info->AddCost(FLAGS_straight_forward_line_cost);
  }
  ADEBUG << "planning start point:" << planning_start_point.ShortDebugString();
  ADEBUG << "Current reference_line_info is IsChangeLanePath: "
         << reference_line_info->IsChangeLanePath();

  auto* mutable_ref_line =
      reference_line_info->mutable_debug()->mutable_ref_line();
  mutable_ref_line->clear_ref_points();
  mutable_ref_line->set_interval(
      reference_line_info->reference_line().GetInterval());
  const auto size = static_cast<int>(
      reference_line_info->reference_line().reference_points().size());
  mutable_ref_line->mutable_ref_points()->Reserve(size);
  for (const auto& ref_points :
       reference_line_info->reference_line().reference_points()) {
    auto* mutable_ref_point = mutable_ref_line->add_ref_points();
    mutable_ref_point->set_x(ref_points.x());
    mutable_ref_point->set_y(ref_points.y());
    mutable_ref_point->set_theta(ref_points.heading());
    mutable_ref_point->set_kappa(ref_points.kappa());
    mutable_ref_point->set_dkappa(ref_points.dkappa());
  }
  auto ret = Status::OK();
  bool path_task_ok = false;
  uint final_path_task_order = 0;
  for (auto* task : task_list_) {
    const double start_timestamp = Clock::NowInSeconds();

    ret = task->Execute(frame, reference_line_info);

    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    ADEBUG << "after task[" << task->Name()
           << "]:" << reference_line_info->PathSpeedDebugString();
    RecordDebugInfo(reference_line_info, task->Name(), time_diff_ms);
    if (!ExecuteTaskSuccessfully(ret, task, frame, &final_path_task_order,
                                 reference_line_info, &path_task_ok)) {
      AERROR << "Fail to execute final task, and need to trigger fallback "
                "trajectory.";
      break;
    }
  }

  RecordObstacleDebugInfo(reference_line_info);

  // check path and speed results for path or speed fallback
  reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
  if (!ret.ok()) {
    const std::string msg = "lane follow task error";
    AERROR << msg;
    // when change lane is failed, trigger adc reference line planning.
    if ((reference_line_info != nullptr) &&
        reference_line_info->IsChangeLanePath()) {
      const std::string msg =
          "reference line info is change lane path and failed to plan.";
      AERROR << msg;
      static constexpr double kReferenceLineInfoFailCost = 2e4;
      reference_line_info->set_trajectory_type(ADCTrajectory::UNKNOWN);
      reference_line_info->AddCost(kReferenceLineInfoFailCost);
      return ret;
    }
    // execute final fallback trajectory.
    if (!reference_line_info->IsChangeLanePath()) {
      PlanFallbackTrajectory(planning_start_point, frame, reference_line_info);
    } else {
      return ret;
    }
  }

  if (!use_joint_optizimizer_) {
    DiscretizedTrajectory trajectory;
    if (!reference_line_info->CombinePathAndSpeedProfile(
            planning_start_point.relative_time(),
            planning_start_point.path_point().s(), &trajectory)) {
      const std::string msg = "Fail to combine path and speed profile.";
      AERROR << msg;
      return Status(ErrorCode::CORE_PLANNING_LANEFOLLOWSTAGE_ERROR, msg);
    }

    for (const auto& traj : trajectory) {
      ADEBUG << FIXED << SETPRECISION(3)
             << "traj_t_out:" << traj.relative_time()
             << "  traj_s_out:" << traj.path_point().s()
             << "  traj_v_out:" << traj.v() << "  traj_a_out:" << traj.a()
             << "  traj_x_out:" << traj.path_point().x()
             << "   traj_y_out:" << traj.path_point().y()
             << "   traj_theta_out:" << traj.path_point().theta();
    }
    reference_line_info->SetTrajectory(trajectory);
    reference_line_info->SetDrivable(true);
  }

#if 0
  // determine if there is a destination on reference line.
  double dest_stop_s = -1.0;
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->LongitudinalDecision().has_stop() &&
        obstacle->LongitudinalDecision().stop().reason_code() ==
            STOP_REASON_DESTINATION) {
      SLPoint dest_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                  reference_line_info->reference_line());
      dest_stop_s = dest_sl.s();
    }
  }

  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (!obstacle->IsStatic()) {
      continue;
    }
    if (obstacle->LongitudinalDecision().has_stop()) {
      bool add_stop_obstacle_cost = false;
      if (dest_stop_s < 0.0) {
        ADEBUG << "add_stop_obstacle_cost and dest_stop_s:" << dest_stop_s;
        add_stop_obstacle_cost = true;
      } else {
        SLPoint stop_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                    reference_line_info->reference_line());
        if (stop_sl.s() < dest_stop_s) {
          ADEBUG << "add_stop_obstacle_cost: stop_sl.s()[" << stop_sl.s()
                 << "], dest_stop_s[" << dest_stop_s << "]";
          add_stop_obstacle_cost = true;
        }
      }

      // if adc reference line remain routing length behind obstacle is not
      // enough to change lane, the lane change reference line info will be the
      // best reference line info.
      if (add_stop_obstacle_cost && reference_line_info->IsChangeLanePath()) {
        auto& adc_reference_line_info = frame->reference_line_info().back();
        const double distance_to_route_end_point =
            adc_reference_line_info.reference_line()
                .GetAdcDistanceToRouteEndPoint();
        const double adc_distance_to_route_end_point =
            std::fmin(adc_reference_line_info.SDistanceToDestination(),
                      distance_to_route_end_point);
        const double distance_to_obstacle =
            GetDistanceBetweenADCAndObstacle(reference_line_info, obstacle);
        constexpr double kLimitLaneChangeDistance = 30.0;
        if (adc_distance_to_route_end_point - distance_to_obstacle <
            kLimitLaneChangeDistance) {
          add_stop_obstacle_cost = false;
          ADEBUG
              << " adc reference line info real length is not enough to change "
                 "lane.";
        }
      }

      if (add_stop_obstacle_cost) {
        static constexpr double kReferenceLineStaticObsCost = 1e3;
        ADEBUG << "add stop obstacle cost 1000, and obstacle id ["
               << obstacle->Id() << "]";
        reference_line_info->AddCost(kReferenceLineStaticObsCost);
      }
    }
  }
#endif

  // if (FLAGS_enable_trajectory_check) {
  //   if (ConstraintChecker::ValidTrajectory(trajectory) !=
  //       ConstraintChecker::Result::VALID) {
  //     const std::string msg = "Current planning trajectory is not valid.";
  //     AERROR << msg;
  //     return Status(ErrorCode::CORE_PLANNING_LANEFOLLOWSTAGE_ERROR, msg);
  //   }
  // }

  return Status(ErrorCode::OK, ret.error_message());
}

void Stage::PlanFallbackTrajectory(const TrajectoryPoint& planning_start_point,
                                   Frame* frame,
                                   ReferenceLineInfo* reference_line_info) {
  UNUSED(planning_start_point);
  // path and speed fall back
  if (reference_line_info->path_data().Empty()) {
    AERROR << "Path fallback due to algorithm failure";
    GenerateFallbackPathProfile(reference_line_info,
                                reference_line_info->mutable_path_data());
    reference_line_info->AddCost(kPathOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::PATH_FALLBACK);
  }

  static uint32_t fallback_sequence_num_prev = 0;
  static int fallback_counter = 0;
  bool get_prev_frame = injector_->frame_history()->Latest() != nullptr;
  uint32_t latest_seq = 0;
  if (get_prev_frame &&
      fallback_sequence_num_prev ==
          injector_->frame_history()->Latest()->SequenceNum()) {
    ADEBUG << "update counter.";
    latest_seq = injector_->frame_history()->Latest()->SequenceNum();
    ++fallback_counter;
  } else {
    fallback_counter = 0;
  }
  ADEBUG << "get_prev_frame:" << get_prev_frame
         << "    fallback_sequence_num_prev:" << fallback_sequence_num_prev
         << "  frame_num:" << latest_seq
         << "   fallback_counter:" << fallback_counter;
  bool is_same_ref = false;
  is_same_ref = TL::planning::util::IsSameReferenceLine(
      *frame, *reference_line_info, reference_line_info->reference_line());

  if (fallback_counter < FLAGS_speed_freeze_frame && is_same_ref &&
      RetrieveLastFrameSpeedProfile(
          reference_line_info, frame,
          reference_line_info->mutable_speed_data())) {
    AERROR << "use prev speed plan.";
  } else {
    AERROR << "Speed fallback due to algorithm failure";
    *reference_line_info->mutable_speed_data() =
        SpeedProfileGenerator::GenerateFallbackSpeed(
            injector_->ego_info(), reference_line_info->GetMaxDeceleration());
  }
  if (get_prev_frame) {
    fallback_sequence_num_prev = frame->SequenceNum();
  } else {
    fallback_sequence_num_prev = 0;
  }

  if (reference_line_info->trajectory_type() != ADCTrajectory::PATH_FALLBACK) {
    reference_line_info->AddCost(kSpeedOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
  }
}

void Stage::GenerateFallbackPathProfile(
    const ReferenceLineInfo* reference_line_info, PathData* path_data) {
  const double unit_s = 1.0;
  const auto& reference_line = reference_line_info->reference_line();

  auto adc_point = injector_->ego_info()->start_point();
  DCHECK(adc_point.has_path_point());
  const auto adc_point_x = adc_point.path_point().x();
  const auto adc_point_y = adc_point.path_point().y();

  common::SLPoint adc_point_s_l;
  if (!reference_line.XYToSL(adc_point.path_point(), &adc_point_s_l)) {
    AERROR << "Fail to project ADC to reference line when calculating path "
              "fallback. Straight forward path is generated";
    const auto adc_point_heading = adc_point.path_point().theta();
    const auto adc_point_kappa = adc_point.path_point().kappa();
    const auto adc_point_dkappa = adc_point.path_point().dkappa();
    std::vector<common::PathPoint> path_points;
    double adc_traversed_x = adc_point_x;
    double adc_traversed_y = adc_point_y;

    const double max_s = 100.0;
    const int max_s_i = std::floor(max_s / unit_s);
    for (int s_i = 0; s_i < max_s_i; ++s_i) {
      path_points.push_back(PointFactory::ToPathPoint(
          adc_traversed_x, adc_traversed_y, 0.0, s_i * unit_s,
          adc_point_heading, adc_point_kappa, adc_point_dkappa));
      adc_traversed_x += unit_s * std::cos(adc_point_heading);
      adc_traversed_y += unit_s * std::sin(adc_point_heading);
    }
    path_data->SetReferenceLine(&reference_line);
    path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
    return;
  }

  // Generate a fallback path along the reference line direction
  const auto adc_s = adc_point_s_l.s();
  const int adc_s_i = std::floor(adc_s / unit_s);
  const auto& adc_ref_point =
      reference_line.GetReferencePoint(adc_point_x, adc_point_y);
  const double dx = adc_point_x - adc_ref_point.x();
  const double dy = adc_point_y - adc_ref_point.y();

  std::vector<common::PathPoint> path_points;
  const int max_s_i = std::floor(reference_line.Length() / unit_s);
  for (int s_i = adc_s_i; s_i < max_s_i; ++s_i) {
    const auto& ref_point = reference_line.GetReferencePoint(s_i * unit_s);
    path_points.push_back(PointFactory::ToPathPoint(
        ref_point.x() + dx, ref_point.y() + dy, 0.0, s_i * unit_s - adc_s,
        ref_point.heading(), ref_point.kappa(), ref_point.dkappa()));
  }
  path_data->SetReferenceLine(&reference_line);
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
}

bool Stage::RetrieveLastFrameSpeedProfile(
    const ReferenceLineInfo* reference_line_info, const Frame* frame,
    SpeedData* const speed_data) {
  UNUSED(reference_line_info);
  const auto* ptr_last_frame = injector_->frame_history()->Latest();
  if (ptr_last_frame == nullptr) {
    AERROR << "Last frame doesn't succeed, fail to retrieve last frame speed "
              "data";
    return false;
  }

  const auto& last_frame_planned_trajectory =
      ptr_last_frame->current_frame_planned_trajectory();

  double dis_between_init_traj_min = std::numeric_limits<double>::max();
  double dis_between_init_traj_temp = 0.0;
  double x_error = 0.0;
  double y_error = 0.0;
  size_t traj_size = last_frame_planned_trajectory->trajectory_point_size();

  if (traj_size <= 1) {
    return false;
  }

  int i = 0;
  for (; i < static_cast<int>(traj_size); ++i) {
    x_error = last_frame_planned_trajectory->trajectory_point()
                  .at(i)
                  .path_point()
                  .x() -
              frame->PlanningStartPoint().path_point().x();
    y_error = last_frame_planned_trajectory->trajectory_point()
                  .at(i)
                  .path_point()
                  .y() -
              frame->PlanningStartPoint().path_point().y();
    dis_between_init_traj_temp = sqrt(x_error * x_error + y_error * y_error);
    if (dis_between_init_traj_temp < dis_between_init_traj_min) {
      dis_between_init_traj_min = dis_between_init_traj_temp;
    } else {
      break;
    }
  }

  if (static_cast<int>(traj_size) - i < FLAGS_least_size_in_reuse_speed_plan) {
    return false;
  }

  double s_start =
      last_frame_planned_trajectory->trajectory_point().at(i).path_point().s();
  double t_start =
      last_frame_planned_trajectory->trajectory_point().at(i).relative_time();
  double v_start = last_frame_planned_trajectory->trajectory_point().at(i).v();
  double a_start = last_frame_planned_trajectory->trajectory_point().at(i).a();

  double s = 0.0;
  double t = 0.0;
  double v = 0.0;
  double a = 0.0;

  for (; i < static_cast<int>(traj_size); ++i) {
    s = last_frame_planned_trajectory->trajectory_point()
            .at(i)
            .path_point()
            .s() -
        s_start + frame->PlanningStartPoint().path_point().s();
    t = last_frame_planned_trajectory->trajectory_point()
            .at(i)
            .relative_time() -
        t_start;
    v = last_frame_planned_trajectory->trajectory_point().at(i).v() - v_start +
        frame->PlanningStartPoint().v();
    a = last_frame_planned_trajectory->trajectory_point().at(i).a() - a_start +
        frame->PlanningStartPoint().a();

    speed_data->AppendSpeedPoint(
        s, t, v, a,
        last_frame_planned_trajectory->trajectory_point().at(i).da());
  }
  SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
  return true;
}

bool Stage::RetrieveLastFramePathProfile(
    const ReferenceLineInfo& reference_line_info, const Frame& frame,
    PathData* const path_data) {
  DCHECK(nullptr != path_data);
  if (injector_ == nullptr || injector_->frame_history() == nullptr) {
    AERROR << "injector_ or frame_history is nullptr.";
    return false;
  }
  const auto* ptr_last_frame = injector_->frame_history()->Latest();
  if (ptr_last_frame == nullptr) {
    AERROR << "ptr_last_frame is nullptr, retrieve path profile fail.";
    return false;
  }

  const auto& reference_line = reference_line_info.reference_line();
  const auto& last_frame_discretized_path =
      ptr_last_frame->current_frame_planned_path();
  if (last_frame_discretized_path == nullptr ||
      last_frame_discretized_path->empty()) {
    AERROR << "Last discretized path is empty, fail to retrieve last frame "
              "path data";
    return false;
  }

  path_data->SetReferenceLine(&reference_line);
  path_data->SetDiscretizedPath(*last_frame_discretized_path);
  const auto adc_frenet_frame_point =
      reference_line.GetFrenetPoint(frame.PlanningStartPoint().path_point());

  bool trim_success = path_data->LeftTrimWithRefS(adc_frenet_frame_point);
  if (!trim_success || path_data->Empty()) {
    AERROR << "Fail to trim path_data. adc_frenet_frame_point: "
           << adc_frenet_frame_point.ShortDebugString();
    return false;
  }

  return true;
}

SLPoint Stage::GetStopSL(const ObjectStop& stop_decision,
                         const ReferenceLine& reference_line) {
  SLPoint sl_point;
  reference_line.XYToSL(stop_decision.stop_point(), &sl_point);
  return sl_point;
}

bool Stage::ExecuteTaskSuccessfully(
    const Status& ret, const Task* task, const Frame* frame,
    uint* const final_path_task_order,
    ReferenceLineInfo* const reference_line_info, bool* const path_task_ok) {
  CHECK(task && frame && reference_line_info && path_task_ok);

  *path_task_ok =
      *path_task_ok || (ret.ok() && task->Name() == "RULE_BASED_STOP_DECIDER");
  if (ret.ok()) {
    return true;
  }
  AERROR << "frame_num:[" << frame->SequenceNum() << "], Failed to run tasks["
         << task->Name() << "], Error message: " << ret.error_message();
  if (reference_line_info->IsChangeLanePath()) {
    return false;
  }
  if (!*path_task_ok) {
    if (*final_path_task_order == 0) {
      for (const auto& order_task : task_list_) {
        ++(*final_path_task_order);
        if (order_task->Name() == "RULE_BASED_STOP_DECIDER") {
          break;
        }
      }
    }
    if (!reference_line_info->path_data().Empty()) {
      FallBackPath fallback_path(config_, injector_);
      if (!fallback_path.GenerateFallbackPathProfile(
              frame, reference_line_info,
              reference_line_info->mutable_path_data())) {
        AERROR << "Failed to generate path fallback.";
        return false;
      }
    } else {
      AWARN << "Trigger path fallback.";
      *path_task_ok = true;
      return true;
    }
  }
  uint current_task_order = 0;
  for (const auto& order_task : task_list_) {
    ++current_task_order;
    if (order_task->Name() == task->Name()) {
      break;
    }
  }

  if (current_task_order > *final_path_task_order) {
    AERROR << "Failed to run speed task.";
    return false;
  }
  ADEBUG << "Skip the failure in task [" << task->Name();
  return true;
}

void Stage::DealHmiChangeLaneStatus(const Frame* frame) {
  auto* mutable_change_lane_status = injector_->planning_context()
                                         ->mutable_planning_status()
                                         ->mutable_change_lane();
  is_lane_change_prepare_overtime_ = false;
  const auto& hmi_change_lane_status =
      mutable_change_lane_status->hmi_change_lane_status();
  ADEBUG << "input hmi change lane status: "
         << functionmanager::HmiChangeLaneStatus_Name(hmi_change_lane_status)
         << " current lane change status: "
         << ChangeLaneStatus::Status_Name(mutable_change_lane_status->status());
  if (mutable_change_lane_status->change_lane_reason() ==
          functionmanager::HmiChangeLaneReason::HMI_TURN_SIGNAL_SWITCH &&
      hmi_change_lane_status == functionmanager::HMI_CHANGE_LANE_START &&
      (mutable_change_lane_status->is_lane_change_on_solid_lane() ||
       !mutable_change_lane_status->target_lane_has_lane_marker())) {
    return;
  }

  if (frame->reference_line_info().size() < 2) {
    road_traffic_jams_start_timestamp_ = 0.0;
    if (hmi_change_lane_status != functionmanager::HMI_CHANGE_LANE_FAILED &&
        hmi_change_lane_status != functionmanager::HMI_CHANGE_LANE_CANCELED &&
        hmi_change_lane_status != functionmanager::HMI_CHANGE_LANE_REQUEST) {
      mutable_change_lane_status->set_hmi_change_lane_status(
          functionmanager::HMI_CHANGE_LANE_FINISHED);
    }
    return;
  }

  if (mutable_change_lane_status->change_lane_reason() ==
          functionmanager::HmiChangeLaneReason::HMI_NONE ||
      hmi_change_lane_status == functionmanager::HMI_CHANGE_LANE_START ||
      hmi_change_lane_status == functionmanager::HMI_CHANGE_LANE_REQUEST ||
      hmi_change_lane_status == functionmanager::HMI_CHANGE_LANE_CANCELED) {
    return;
  }

  mutable_change_lane_status->set_hmi_change_lane_status(
      functionmanager::HMI_CHANGE_LANE_FINISHED);

  if (mutable_change_lane_status->has_status()) {
    if (mutable_change_lane_status->status() ==
        ChangeLaneStatus::IN_CHANGE_LANE) {
      mutable_change_lane_status->set_hmi_change_lane_status(
          functionmanager::HMI_IN_CHANGE_LANE);
    }
    if (mutable_change_lane_status->status() ==
        ChangeLaneStatus::CHANGE_LANE_FAILED) {
      mutable_change_lane_status->set_hmi_change_lane_status(
          functionmanager::HMI_CHANGE_LANE_CANCELED);
    }
  }

  // if adc is in change lane preparation and coming to route end point ,
  // should be warn change lane failure.
  bool is_no_gap = false;
  if (mutable_change_lane_status->exist_lane_change_start_position()) {
    is_no_gap = true;
  } else {
    road_traffic_jams_start_timestamp_ = 0.0;
  }

  // auto& adc_reference_line =
  //     (++frame->reference_line_info().begin())->reference_line();
  // const double adc_distance_to_route_end_point =
  //     adc_reference_line.GetAdcDistanceToRouteEndPoint();
  // auto& adc_reference_line_tag = adc_reference_line.Tag();
  // const auto pose = adc_reference_line_tag.find_last_of("/");
  // if (adc_reference_line_tag.empty() || pose == std::string::npos) {
  //   AERROR << " adc reference line tag [" << adc_reference_line_tag
  //          << "] is error.";
  //   return;
  // }
  // ADEBUG << "adc_reference_line_tag: " << adc_reference_line_tag;
  // int lane_change_cnt =
  //     std::max(1, std::stoi(adc_reference_line_tag.substr(pose + 1)));
  // static constexpr double kMinChangeLaneTime = 3.0;
  // const double change_lane_limit_distance =
  //     lane_change_cnt *
  //     std::fmax(kMinChangeLaneTime *
  //                   injector_->ego_info()->vehicle_state().linear_velocity(),
  //               kHmiWarningDistanceToRouteEndPoint);

  // //  When adc is already in limit lane change, it is very impossible to
  // //  complete lane change.
  // if (is_no_gap &&
  //     common::math::double_type::Compare(adc_distance_to_route_end_point,
  //                                        change_lane_limit_distance) < 1) {
  //   mutable_change_lane_status->set_hmi_change_lane_status(
  //       functionmanager::HMI_CHANGE_LANE_CANCELED);
  //   ADEBUG << " change lane preparation causes lane change failure.";
  //   return;
  // }

  // road traffic jams time is more than 10s to send hmi lane change failure.
  if (is_no_gap &&
      common::math::double_type::IsZero(road_traffic_jams_start_timestamp_)) {
    road_traffic_jams_start_timestamp_ = Clock::NowInSeconds();
  }
  static constexpr double kRoadTrafficJamsTime = 20.0;
  const auto change_lane_reason =
      mutable_change_lane_status->change_lane_reason();
  if (is_no_gap &&
      Clock::NowInSeconds() - road_traffic_jams_start_timestamp_ >
          kRoadTrafficJamsTime &&
      change_lane_reason != functionmanager::HmiChangeLaneReason::HMI_NONE &&
      change_lane_reason !=
          functionmanager::HmiChangeLaneReason::HMI_NAVIGATION) {
    mutable_change_lane_status->set_hmi_change_lane_status(
        functionmanager::HMI_CHANGE_LANE_CANCELED);
    is_lane_change_prepare_overtime_ = true;
    ADEBUG << " change lane preparation time is more than 20s.";
  }
}

void Stage::RecordOpenSpaceTaskConsumeTime(
    const std::string& name, const double time_ms,
    OpenSpaceInfo* const open_space_info) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  if (nullptr == open_space_info) {
    AERROR << "open space info is nullptr";
    return;
  }

  auto* ptr_latency_stats = open_space_info->mutable_latency_stats();
  auto* task_ptr = ptr_latency_stats->add_task_stats();
  task_ptr->set_name(name);
  task_ptr->set_time_ms(time_ms);
}
}  // namespace scenario
}  // namespace planning
}  // namespace TL
