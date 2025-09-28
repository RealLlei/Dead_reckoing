/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "planning/tasks/optimizers/open_space_path_generation/open_space_path_provider.h"
#include "planning/tasks/deciders/open_space_decider/open_space_obstacle.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "common/file/file.h"
#include "common/math/line_segment2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/time/clock.h"
#include "planning/common/frame.h"
#include "planning/common/open_space_info.h"
#include "planning/common/path/discretized_path.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/fsm/avp_fct.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/planning/planning_internal.pb.h"
#include "proto/planning/planning_status.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

using TL::common::Clock;  // NOLINT
using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::math::Vec2d;

namespace {
constexpr double kEpsilon = 1e-3;
constexpr size_t kMaxSearchThreadNum = 4;
constexpr double kTraceAdjustBound = 30.0;
constexpr double kTraceAdjustLatDiffThreshold = 0.05;
constexpr double kTraceAdjustThetaDiffThreshold = 0.02;
constexpr double kTraceAdjustTargetS = 8.0;
constexpr double kNNSAdjustMaxDec = 0.2;
constexpr double kNNSAdjustMinVel = 0.5;
}  // namespace

OpenSpacePathProvider::OpenSpacePathProvider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : PathOptimizer(config, injector),
      open_space_thread_manager_(kMaxSearchThreadNum,
                                 config_.open_space_path_provider_config()) {
  ACHECK(common::GetProtoFromFile(FLAGS_vehicle_model_config_filename,
                                  &vehicle_model_config_))
      << "Failed to load vehicle model config file "
      << FLAGS_vehicle_model_config_filename;
}

// OpenSpacePathProvider::~OpenSpacePathProvider() {
// Stop();
// }

// void OpenSpacePathProvider::Stop() {
// is_generation_thread_stop_.store(true);
// open_space_cv_.notify_all();
// if (task_future_.valid()) {
//   task_future_.get();
// }
// }

Status OpenSpacePathProvider::Reset() {
  // data_ready_.store(false);
  // thread_start_time_.store(TL::common::Clock::NowInNanoseconds());
  OpenSpaceInfo::ResetReplanStatus(injector_->planning_context()
                                       ->mutable_planning_status()
                                       ->mutable_open_space());
  optimizer_debug_.Clear();
  optimizer_multi_debugs_.clear();
  splice_path_data_ = PathGearPair();
  replan_status_ = 0;
  is_reach_precise_target_ = false;
  is_entered_special_domain_ = false;
  thread_cost_time_ = 0.0;
  thread_start_time_ = common::Clock::NowInSeconds();
  no_valid_path_start_time_ = common::Clock::NowInSeconds();
  plan_thread_status_ = PlanThreadStatus::OFF;
  init_adc_point_ = nullptr;
  AINFO << "path provider is reseted";
  return Status::OK();
}

Status OpenSpacePathProvider::Process() {
  AINFO << "enter AVP path provider";
  CHECK_NOTNULL(injector_);
  const auto parking_type = injector_->planning_context()
                                ->planning_status()
                                .avp_status()
                                .parking_type();
  if (frame_->local_view()
              .GetFunctionManagerIn()
              ->fct_avp_in()
              .sys_run_state() == functionmanager::AvpFctIn::PARKSTART &&
      parking_type != TL::planning::AVPStatus_ParkingType_NNS_ADJUST) {
    PrePlan();
    if (FLAGS_enable_record_debug) {
      auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug();
      ptr_debug->mutable_planning_data()
          ->mutable_open_space()
          ->clear_multi_search_info();
      for (auto& optimizer_debug : optimizer_multi_debugs_) {
        const auto& optimizer_search_debug = optimizer_debug.second;
        auto* search_info = ptr_debug->mutable_planning_data()
                                ->mutable_open_space()
                                ->add_multi_search_info();
        search_info->mutable_warm_start_path()->MergeFrom(
            optimizer_search_debug.warm_start_path());
        search_info->mutable_xy_boundary()->MergeFrom(
            optimizer_search_debug.xy_boundary());
        search_info->mutable_trajectory_stitching_point()->MergeFrom(
            optimizer_search_debug.trajectory_stitching_point());
        search_info->mutable_obstacles()->MergeFrom(
            optimizer_search_debug.obstacles());
        search_info->mutable_roi_shift_point()->MergeFrom(
            optimizer_search_debug.roi_shift_point());
        search_info->set_park_id(optimizer_debug.first);
      }
      ADEBUG << "succeed to get "
             << ptr_debug->mutable_planning_data()
                    ->mutable_open_space()
                    ->multi_search_info()
                    .size()
             << " search result(s)";
      frame_->mutable_open_space_info()->sync_debug_instance();
    }
    return Status::OK();
  }
  auto status = PreCheck();
  if (status != Status::OK()) {
    return status;
  }
  return PlanningOnPathThread();
}

TL::common::Status OpenSpacePathProvider::PreCheck() {
  double left_extra_buffer = 0.0;
  double right_extra_buffer = 0.0;
  double left_extra_buffer_for_low_fs = 0.0;
  double right_extra_buffer_for_low_fs = 0.0;
  const auto& parking_scenario_type =
      frame_->open_space_info()
          .open_space_path_info()
          .open_space_env_structured_info.parking_scenario_type;
  if (nullptr == init_adc_point_) {
    const auto vehicle_state = frame_->vehicle_state();
    init_adc_point_ = std::make_unique<common::PathPoint>();
    init_adc_point_->set_x(vehicle_state.x());
    init_adc_point_->set_y(vehicle_state.y());
    init_adc_point_->set_theta(vehicle_state.heading());
    if (parking_scenario_type ==
        ParkingScenarioType::LEFT_LATERAL_PARKING_OUT) {
      right_extra_buffer += config_.open_space_path_provider_config()
                                .lat_spot_park_out_bottom_distance_threshold();
      right_extra_buffer_for_low_fs +=
          config_.open_space_path_provider_config()
              .lat_spot_park_out_bottom_distance_threshold_for_low_fs();
    } else if (parking_scenario_type ==
               ParkingScenarioType::RIGHT_LATERAL_PARKING_OUT) {
      left_extra_buffer += config_.open_space_path_provider_config()
                               .lat_spot_park_out_bottom_distance_threshold();
      left_extra_buffer_for_low_fs +=
          config_.open_space_path_provider_config()
              .lat_spot_park_out_bottom_distance_threshold_for_low_fs();
    }
    /* ---------PreCheck for parking-out adjust for Low fs segements obs -------*/
    // 1.Sepreate Low fs obs_segement
    // 2.Set diff threshold for diff precheck
    // 3.Ensure only Low fs obs_segement can be used by small buffer
    if (left_extra_buffer > kEpsilon || right_extra_buffer > kEpsilon) {
      const auto adc_polygon =
          common::VehicleConfigHelper::GetPolygon2dWithBuffer(
              init_adc_point_->x(), init_adc_point_->y(),
              init_adc_point_->theta(), 0.0, 0.0, left_extra_buffer,
              right_extra_buffer);
      const auto adc_polygon_for_low_fs =
          common::VehicleConfigHelper::GetPolygon2dWithBuffer(
              init_adc_point_->x(), init_adc_point_->y(),
              init_adc_point_->theta(), 0.0, 0.0, left_extra_buffer_for_low_fs,
              right_extra_buffer_for_low_fs);

      // Origin check low_fs method
      // for (const auto& obs : frame_->open_space_info()
      //                            .open_space_path_info()
      //                            .obstacles_segments_vec) {
      //   if (obs.second < kEpsilon) {
      //     continue;
      //   }
      //   // time and space complexity needs to be checked and review
      //   if (adc_polygon.HasOverlap(obs.first)) {
      //     // using lamda expression to judge if coll-obs is low_fs_obs
      //     auto low_fs_seg = frame_->open_space_info()
      //                           .open_space_path_info()
      //                           .low_fs_obstacles_segments_vec;
      //     auto it = std::find_if(
      //         low_fs_seg.begin(), low_fs_seg.end(),
      //         [&obs](const std::pair<common::math::LineSegment2d, double>&
      //                    another) { return obs.first == another.first; });
      //     if (it == low_fs_seg.end() ||
      //         adc_polygon_for_low_fs.HasOverlap(obs.first)) {
      //       ADEBUG << "Precheck fail";
      //       return Status(ErrorCode::PLANNER_PARKING_PATHPROVIDER_ERROR,
      //                     "Lower space is too small to finish "
      //                     "task");
      //     }
      //   }
      // }

      // Improved method
      for (const auto& low_obs : frame_->open_space_info()
                                     .open_space_path_info()
                                     .low_fs_obstacles_segments_vec) {
        if (adc_polygon_for_low_fs.HasOverlap(low_obs.first)) {
          ADEBUG << "Precheck fail";
          return Status(ErrorCode::PLANNER_PARKING_PATHPROVIDER_ERROR,
                        "Lower space is too small to finish "
                        "task");
        }
      }
      for (const auto& obs : frame_->open_space_info()
                                 .open_space_path_info()
                                 .obstacles_segments_vec) {
        // easy-check for whether fs is low_fs_segment
        if (obs.second <= kBtmLowFsBuffer + kEpsilon) {
          continue;
        }
        if (adc_polygon.HasOverlap(obs.first)) {
          ADEBUG << "Precheck fail";
          return Status(ErrorCode::PLANNER_PARKING_PATHPROVIDER_ERROR,
                        "Lower space is too small to finish "
                        "task");
        }
      }
    }
  }
  return Status::OK();
}

Status OpenSpacePathProvider::PlanningOnPathThread() {
  size_t update_path_segment_size = 0;
  auto update_path_status = planning_internal::PathUpdateStatus::NONE;
  PartitionedPath optimized_path;
  UpdatePathStatus(&optimized_path, &update_path_segment_size,
                   &update_path_status);
  ADEBUG << "Output path type: "
         << planning_internal::PathUpdateStatus::PathType_Name(
                optimized_path.path_type);

  // sync to debug
  if (FLAGS_enable_record_debug) {
    auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug();
    // {
    // std::lock_guard<std::mutex> lock(open_space_mutex_);
    ptr_debug->mutable_planning_data()->mutable_open_space()->MergeFrom(
        optimizer_debug_);
    // }
    ptr_debug->mutable_planning_data()
        ->mutable_open_space()
        ->mutable_path_update_status()
        ->set_path_gear_shift_time(update_path_segment_size);
    ptr_debug->mutable_planning_data()
        ->mutable_open_space()
        ->mutable_path_update_status()
        ->set_update_status(update_path_status);
    frame_->mutable_open_space_info()->sync_debug_instance();
  }
  std::string update_path_msg =
      planning_internal::PathUpdateStatus::UpdateStatus_Name(
          update_path_status);
  switch (update_path_status) {
    case planning_internal::PathUpdateStatus::SUCCESS:
    case planning_internal::PathUpdateStatus::PATH_SEGMENT_OVER_LIMIT: {
      *(frame_->mutable_open_space_info()->mutable_path_result()) =
          optimized_path;
      break;
    }
    case planning_internal::PathUpdateStatus::NONE:
    case planning_internal::PathUpdateStatus::SEARCH_FAILED:
    case planning_internal::PathUpdateStatus::OPTIMIZE_FAILED:
    case planning_internal::PathUpdateStatus::START_POINT_MISMATCH:
    case planning_internal::PathUpdateStatus::WAIT_RESULT: {
      break;
    }
    case planning_internal::PathUpdateStatus::OVER_TIME: {
      // std::lock_guard<std::mutex> lock(open_space_mutex_);
      update_path_msg += open_space_path_output_.error_msg;
      AERROR << update_path_msg;
      return Status(ErrorCode::PLANNER_PARKING_PATHPROVIDER_ERROR,
                    update_path_msg);
    }
    default:
      break;
  }
  ADEBUG << "update_path_msg " << update_path_msg;
  return Status(ErrorCode::OK, update_path_msg);
}

void OpenSpacePathProvider::UpdatePathStatus(
    PartitionedPath* const optimized_path_ptr,
    size_t* const update_path_segment_size_ptr,
    planning_internal::PathUpdateStatus::UpdateStatus* const path_status_ptr) {
  if (nullptr == optimized_path_ptr ||
      nullptr == update_path_segment_size_ptr || nullptr == path_status_ptr) {
    return;
  }
  optimized_path_ptr->path_set.clear();
  *update_path_segment_size_ptr = 0;
  *path_status_ptr = planning_internal::PathUpdateStatus::NONE;
  UpdateCurTaskReplanStatus();
  const auto thread_plan_time =
      common::Clock::NowInSeconds() - thread_start_time_;
  const auto no_valid_path_time =
      common::Clock::NowInSeconds() - no_valid_path_start_time_;
  const auto is_time_over_limit = [&](const double time) {
    const auto scenario_diffculty_type =
        frame_->open_space_info()
            .open_space_path_info()
            .open_space_env_structured_info.parking_scenario_diffculty_type;
    const double time_limit =
        ((scenario_diffculty_type & DEADEND_SCENARIO) != 0 ||
         (scenario_diffculty_type & NARROW_PASSAGE_SCENARIO) != 0)
            ? config_.open_space_path_provider_config()
                  .dead_end_scenario_path_generate_max_time()
            : config_.open_space_path_provider_config()
                  .path_generate_max_time();
    return time > time_limit;
  };
  switch (plan_thread_status_) {
    case OFF: {
      if (replan_status_ > 0) {
        UpdateReplanInfo();
        *path_status_ptr = planning_internal::PathUpdateStatus::WAIT_RESULT;
      }
      // reset time when not replanning
      thread_start_time_ = common::Clock::NowInSeconds();
      no_valid_path_start_time_ = common::Clock::NowInSeconds();
      break;
    }
    case RUNNING: {
      *path_status_ptr = planning_internal::PathUpdateStatus::WAIT_RESULT;
      // update result
      auto get_target_success = open_space_thread_manager_.GetTargetOutput(
          &open_space_path_output_, &optimizer_debug_);
      if (get_target_success && open_space_path_output_.has_smoothed &&
          open_space_path_output_.error_msg.empty()) {
        plan_thread_status_ = PlanThreadStatus::OFF;
        thread_cost_time_ = thread_plan_time;

        is_reach_precise_target_ =
            !open_space_path_output_.partitioned_path.empty() &&
            !open_space_path_output_.partitioned_path.back().first.empty() &&
            DiscretizedPath::IsSamePoint(
                open_space_path_input_.end_pose,
                open_space_path_output_.partitioned_path.back().first.back());
        if (CheckPathValid(open_space_path_output_.partitioned_path,
                           update_path_segment_size_ptr, path_status_ptr)) {
          optimized_path_ptr->path_set =
              open_space_path_output_.partitioned_path;
          optimized_path_ptr->path_type = open_space_path_output_.path_type;
          optimized_path_ptr->replan_status =
              open_space_path_output_.replan_status;
          ADEBUG << "path type: "
                 << planning_internal::PathUpdateStatus::PathType_Name(
                        optimized_path_ptr->path_type);
          auto splice_path = splice_path_data_.first;
          const auto& splice_gear = splice_path_data_.second;
          injector_->planning_context()
              ->mutable_planning_status()
              ->mutable_open_space()
              ->set_is_reach_precise_target(is_reach_precise_target_);
          if (splice_path.size() > 1) {
            if (splice_gear != optimized_path_ptr->path_set.front().second) {
              optimized_path_ptr->path_set.insert(
                  optimized_path_ptr->path_set.begin(), splice_path_data_);
            } else {
              splice_path.pop_back();
              const double init_s = splice_path.front().s();
              for (auto& point : splice_path) {
                point.set_s(point.s() - init_s);
              }
              const auto& splice_end_point = splice_path.back();
              const auto& optimize_path_front_point =
                  optimized_path_ptr->path_set.front().first.front();
              const double acc_s =
                  splice_end_point.s() +
                  std::hypot(
                      splice_end_point.x() - optimize_path_front_point.x(),
                      splice_end_point.y() - optimize_path_front_point.y());

              for (auto& point : optimized_path_ptr->path_set.front().first) {
                point.set_s(point.s() + acc_s);
              }
              optimized_path_ptr->path_set.front().first.insert(
                  optimized_path_ptr->path_set.front().first.begin(),
                  splice_path.begin(), splice_path.end());
            }
          }
        } else {
          if ((open_space_path_output_.replan_status &
               static_cast<uint32_t>(OpenSpaceStatus::TRACE_REPLAN)) != 0) {
            replan_status_ |=
                static_cast<uint32_t>(OpenSpaceStatus::TRACE_REPLAN);
          }
          if (replan_status_ > 0) {
            UpdateReplanInfo();
          }
        }
      } else if (get_target_success &&
                 !open_space_path_output_.error_msg.empty()) {
        *path_status_ptr = planning_internal::PathUpdateStatus::SEARCH_FAILED;
        plan_thread_status_ = PlanThreadStatus::OFF;
        if (replan_status_ > 0) {
          UpdateReplanInfo();
        }
      }
      if (!frame_->IsVehicleStandStill() || HasValidHistoryPath() ||
          (*path_status_ptr == planning_internal::PathUpdateStatus::SUCCESS)) {
        no_valid_path_start_time_ = common::Clock::NowInSeconds();
      } else if (is_time_over_limit(no_valid_path_time)) {
        *path_status_ptr = planning_internal::PathUpdateStatus::OVER_TIME;
      }
      break;
    }

    default:
      break;
  }
}

bool OpenSpacePathProvider::HasValidHistoryPath() {
  const auto& previous_frame = injector_->frame_history()->Latest();
  return nullptr != previous_frame &&
         previous_frame->open_space_info().is_partitioned_paths_valid();
}

bool OpenSpacePathProvider::CheckPathValid(
    const std::vector<PathGearPair>& optimized_path,
    size_t* const update_path_segment_size_ptr,
    planning_internal::PathUpdateStatus::UpdateStatus* const path_status_ptr) {
  bool ret = false;
  const auto& splice_path = splice_path_data_.first;
  const auto& start_point = frame_->PlanningStartPoint().path_point();
  bool is_start_point_match = !optimized_path.empty();
  if (splice_path.size() == 1) {
    // loc will have error if adc is standstill
    double kDistanceBuffer = 0.02;
    double kAngleBuffer = 0.02;
    is_start_point_match =
        is_start_point_match &&
        DiscretizedPath::IsSamePoint(splice_path.back(),
                                     optimized_path.front().first.front(),
                                     kDistanceBuffer, kAngleBuffer);
  } else {
    is_start_point_match =
        is_start_point_match && splice_path.IsPointIn(start_point) &&
        DiscretizedPath::IsSamePoint(splice_path.back(),
                                     optimized_path.front().first.front());
  }
  if (!is_start_point_match) {
    *path_status_ptr =
        planning_internal::PathUpdateStatus::START_POINT_MISMATCH;
    ret = false;
  } else {
    const size_t trajectory_segment_size = optimized_path.size();
    *update_path_segment_size_ptr = trajectory_segment_size;
    if (trajectory_segment_size >= FLAGS_apa_gear_shift_limit) {
      *path_status_ptr =
          planning_internal::PathUpdateStatus::PATH_SEGMENT_OVER_LIMIT;
    } else {
      *path_status_ptr = planning_internal::PathUpdateStatus::SUCCESS;
    }
    ret = true;
  }
  return ret;
}

// void OpenSpacePathProvider::GeneratePathThread() {
//  CHECK_NOTNULL(injector_);
//  while (!is_generation_thread_stop_) {
//    OpenSpacePathInput open_space_path_input;
//    OpenSpacePathOutput open_space_path_output;
//    {
//      std::unique_lock<std::mutex> lck(open_space_mutex_);
//      while (!is_generation_thread_stop_ && !data_ready_) {
//        open_space_cv_.wait(lck);
//      }
//      if (is_generation_thread_stop_) {
//        break;
//      }
//      ADEBUG << "download optimizer needed data";
//      open_space_path_input = open_space_path_input_;
//      open_space_path_output = open_space_path_output_;
//    }
//    plan_thread_status_.store(PlanThreadStatus::RUNNING);
//    path_generator_ptr_->Plan(false, open_space_path_input,
//                              &open_space_path_output);
//    path_smoother_ptr_->Smooth(open_space_path_input, &open_space_path_output);
//
//    {
//      std::lock_guard<std::mutex> lock(open_space_mutex_);
//      open_space_path_output_ = open_space_path_output;
//      if (FLAGS_enable_record_debug) {
//        path_generator_ptr_->UpdateDebugInfo(&optimizer_debug_);
//        path_smoother_ptr_->UpdateDebugInfo(&optimizer_debug_);
//      }
//    }
//    data_ready_.store(false);
//    if (open_space_path_output.error_msg.empty()) {
//      const bool is_same_point =
//          !open_space_path_output.partitioned_path.empty() &&
//          !open_space_path_output.partitioned_path.back().first.empty() &&
//          DiscretizedPath::IsSamePoint(
//              open_space_path_input.end_pose,
//              open_space_path_output.partitioned_path.back().first.back());
//      is_reach_precise_target_.store(is_same_point);
//      plan_thread_status_.store(PlanThreadStatus::UPDATE_SUC);
//      AINFO << "---- openspace_trajetory is_ok ------ ";
//    } else {
//      plan_thread_status_.store(PlanThreadStatus::UPDATE_FAIL);
//      AERROR << "----- openspace_trajetory is_fail ---- ";
//    }
//  }
// }

void OpenSpacePathProvider::UpdateReplanInfo() {
  const auto& vehicle_state = frame_->vehicle_state();
  const auto& start_point = frame_->PlanningStartPoint().path_point();
  auto& splice_path = splice_path_data_.first;
  auto& splice_gear = splice_path_data_.second;
  const auto& previous_frame = injector_->frame_history()->Latest();
  ResetSpliceTraj();
  if (nullptr != previous_frame &&
      previous_frame->local_view().HasADCTrajectory()) {
    double replan_time_estimation =
        config_.open_space_path_provider_config().replan_time_estimation();
    // TODO(lsy): align spd param
    if (frame_->open_space_info()
            .open_space_path_info()
            .open_space_env_structured_info.is_in_nns_adjust_scenario) {
      replan_time_estimation =
          std::max(replan_time_estimation,
                   (vehicle_state.linear_velocity() - kNNSAdjustMinVel) /
                       kNNSAdjustMaxDec);
    }
    static constexpr double kMinDisDiff = 1e-5;
    static constexpr double kMinSpliceLength = 0.2;
    const DiscretizedTrajectory last_pub_traj(
        previous_frame->local_view().GetADCTrajectory()->trajectory_point());
    const int match_idx = static_cast<int>(last_pub_traj.QueryNearestPoint(
        Vec2d(start_point.x(), start_point.y())));
    for (int i = match_idx; i < static_cast<int>(last_pub_traj.size()); ++i) {
      const auto& point_i = last_pub_traj[i];
      splice_path.emplace_back(point_i.path_point());
      const double time_diff =
          point_i.relative_time() - last_pub_traj[match_idx].relative_time();
      if (time_diff > replan_time_estimation) {
        break;
      }
      if (i > match_idx) {
        const auto& point_i_1 = last_pub_traj[i - 1];
        const double s_diff =
            point_i.path_point().s() - point_i_1.path_point().s();
        // if nearby two points to close, find next point in last_pub_traj
        if (fabs(s_diff) < kMinDisDiff) {
          splice_path.pop_back();
          break;
        }
      }
    }
    // if splice_path is not valid, clear
    if (splice_path.empty() || !splice_path.IsPointIn(start_point) ||
        fabs(splice_path.back().s() - splice_path.front().s()) <
            kMinSpliceLength) {
      splice_path.clear();
    }
    // set splice_path gear informations
    splice_gear = previous_frame->local_view().GetADCTrajectory()->gear();
  }
  // in case search failed due to start point on cruise path
  if (frame_->open_space_info()
          .open_space_path_info()
          .open_space_env_structured_info.is_in_nns_adjust_scenario &&
      frame_->IsVehicleStandStill()) {
    ResetSpliceTraj();
  }
  if (splice_path.empty()) {
    splice_path.emplace_back(start_point);
    splice_gear = vehicle_state.gear();
    const double step_estimation =
        vehicle_state.linear_velocity() *
        config_.open_space_path_provider_config().replan_time_estimation();
    if (nullptr != previous_frame &&
        !previous_frame->open_space_info().is_on_open_space_trajectory() &&
        !previous_frame->reference_line_info().empty() &&
        !frame_->IsVehicleStandStill()) {
      const auto& cruise_path = previous_frame->reference_line_info()
                                    .front()
                                    .path_data()
                                    .discretized_path();
      static constexpr double kSpliceStep = 0.2;
      double splice_length = kSpliceStep;
      common::SLPoint start_point_sl;
      while (splice_length <= step_estimation) {
        if (!cruise_path.XYToSL(start_point.x(), start_point.y(),
                                &start_point_sl)) {
          break;
        }
        splice_path.emplace_back(
            cruise_path.Evaluate(start_point_sl.s() + splice_length));
        splice_length += kSpliceStep;
      }
    }
  }
  // {
  //  std::lock_guard<std::mutex> lock(open_space_mutex_);
  for (const auto& open_space_path_info_pair :
       frame_->open_space_info().open_space_path_info_map()) {
    if (open_space_path_info_pair.first !=
        frame_->open_space_info().open_space_path_info_id()) {
      continue;
    }
    LoadOptimizerData(open_space_path_info_pair.second, &open_space_path_input_,
                      &open_space_path_output_);
    open_space_path_input_.path_id =
        static_cast<int>(open_space_path_info_pair.first);
  }
  // }

  thread_start_time_ = common::Clock::NowInSeconds();
  open_space_thread_manager_.TargetPlan(open_space_path_input_);
  plan_thread_status_ = PlanThreadStatus::RUNNING;

  // data_ready_.store(true);
  // open_space_cv_.notify_all();
}

void OpenSpacePathProvider::LoadOptimizerData(
    const OpenSpacePathInfo& open_space_path_info,
    OpenSpacePathInput* const open_space_path_input,
    OpenSpacePathOutput* const open_space_path_output) {
  if (nullptr == open_space_path_input || nullptr == open_space_path_output) {
    return;
  }
  // reset output
  open_space_path_output->Reset();
  open_space_path_input->replan_status = replan_status_;
  open_space_path_input->rotate_angle = open_space_path_info.rotate_angle;
  open_space_path_input->end_pose = open_space_path_info.end_point;
  open_space_path_input->translate_origin = open_space_path_info.origin;
  open_space_path_input->obstacles_segments_vec =
      open_space_path_info.obstacles_segments_vec;
  open_space_path_input->xy_bounds = open_space_path_info.roi_xy_boundary;
  const auto& splice_path = splice_path_data_.first;
  auto splice_gear = frame_->vehicle_state().gear();
  auto start_point = frame_->PlanningStartPoint().path_point();
  if (!splice_path.empty()) {
    open_space_path_input->start_point = splice_path.back();
    splice_gear = splice_path_data_.second;
  } else {
    open_space_path_input->start_point =
        frame_->PlanningStartPoint().path_point();
  }
  open_space_path_input->dest_region_with_angle =
      open_space_path_info.dest_region_with_angle;
  SetWarmStartPath(start_point, open_space_path_info.trace_path,
                   &(open_space_path_input->warm_start_path));
  SetPathStrategy(open_space_path_info, splice_path.size(),
                  open_space_path_input->start_point, splice_gear,
                  open_space_path_input->dest_region_with_angle,
                  open_space_path_info.trace_path,
                  &(open_space_path_input->path_strategy));
  if (open_space_path_input->path_strategy.path_search_strategy
          .trace_adjust_search_strategy.is_trace_adjust) {
    auto iter = open_space_path_input->obstacles_segments_vec.begin();
    while (iter != open_space_path_input->obstacles_segments_vec.end()) {
      if (iter->second < kEpsilon) {
        iter = open_space_path_input->obstacles_segments_vec.erase(iter);
      } else {
        ++iter;
      }
    }
  }
  AINFO << open_space_path_input->path_strategy.path_search_strategy
               .DebugString();
}

void OpenSpacePathProvider::SetWarmStartPath(
    const common::PathPoint& start_point, const DiscretizedPath& apa_trace_path,
    DiscretizedPath* const warm_start_path_ptr) const {
  if (nullptr == warm_start_path_ptr) {
    return;
  }
  warm_start_path_ptr->clear();
  // 初始时刻或者在使用历史路径过程中，使用倒车路径
  if ((replan_status_ & static_cast<uint32_t>(OpenSpaceStatus::TRACE_REPLAN)) ==
          0U &&
      (replan_status_ &
       static_cast<uint32_t>(OpenSpaceStatus::NO_VALID_PATH)) == 0U) {
    return;
  }
  common::SLPoint start_point_sl;
  if (apa_trace_path.empty() ||
      !apa_trace_path.XYToSL(start_point.x(), start_point.y(),
                             &start_point_sl)) {
    // initialize sl_point by sl_start_point
    ADEBUG << "Error init apa trace path " << apa_trace_path.size();
    return;
  }
  constexpr double kLonDiffThreshold = 0.5;
  // use lambda expression to find the first point
  // whose s greater than start_point_sl.s() in apa_trace_path
  auto iter =
      std::find_if(apa_trace_path.begin(), apa_trace_path.end(),
                   [&start_point_sl](const common::PathPoint& path_point) {
                     return start_point_sl.s() < path_point.s();
                   });
  warm_start_path_ptr->assign(iter, apa_trace_path.end());
  if (!warm_start_path_ptr->empty()) {
    warm_start_path_ptr->front() = start_point;
    warm_start_path_ptr->front().set_s(0.0);
    for (size_t i = 1; i < warm_start_path_ptr->size(); ++i) {
      const auto& prev_point = warm_start_path_ptr->at(i - 1);
      auto& cur_point = warm_start_path_ptr->at(i);
      // calculate s in warm_start_path
      cur_point.set_s(prev_point.s() +
                      std::hypot(cur_point.x() - prev_point.x(),
                                 cur_point.y() - prev_point.y()));
    }
    if (warm_start_path_ptr->back().s() - warm_start_path_ptr->front().s() <
        kLonDiffThreshold) {
      // too short to use
      ADEBUG << "trace path is too short to use";
      warm_start_path_ptr->clear();
    }
  }
}

// if has TRACE_PATH and is valid judge by current start point
// use as warm_start_path

void OpenSpacePathProvider::UpdateCurTaskReplanStatus() {
  replan_status_ = 0;
  const auto& previous_frame = injector_->frame_history()->Latest();
  const bool reach_dest =
      previous_frame != nullptr &&
      previous_frame->open_space_info().destination_reached();
  if (reach_dest) {
    return;
  }
  replan_status_ =
      injector_->planning_context()->planning_status().open_space().replan();

  if (replan_status_ > 0) {
    std::string replan_reason;
    for (int i = OpenSpaceStatus::Replan_MIN; i <= OpenSpaceStatus::Replan_MAX;
         ++i) {
      if (OpenSpaceStatus::Replan_IsValid(i) &&
          (replan_status_ & static_cast<uint32_t>(i)) != 0U) {
        replan_reason += OpenSpaceStatus_Replan_Name(i);
        replan_reason += ";";
      }
    }
    *(frame_->mutable_open_space_info()->mutable_replan_reason()) =
        replan_reason;
    AINFO << "replan_status_ " << replan_status_
          << " replan_reason: " << replan_reason;
    // reset replan status
    OpenSpaceInfo::ResetReplanStatus(injector_->planning_context()
                                         ->mutable_planning_status()
                                         ->mutable_open_space());
  }
}

void OpenSpacePathProvider::SetPathStrategy(
    const OpenSpacePathInfo& open_space_path_info,
    const size_t splice_path_size, const common::PathPoint& start_point,
    const soc::Chassis::GearPosition& start_gear,
    const DestRegionWithAng& dest_region_with_angle,
    const DiscretizedPath& trace_path, PathStrategy* const path_strategy) {
  if (nullptr == path_strategy) {
    AERROR << "path_strategy is nullptr";
    return;
  }
  auto* path_search_strategy_ptr = &(path_strategy->path_search_strategy);
  const auto& open_space_env_structured_info =
      open_space_path_info.open_space_env_structured_info;
  // reset
  path_strategy->init_moving_direction = 0;
  path_strategy->disable_search = false;
  path_search_strategy_ptr->Reset();
  // decided by data loaded in provider, set path_search_strategy
  // force gear shift: default 0, no limit
  ForceInitDirectionStrategy(open_space_env_structured_info, splice_path_size,
                             start_gear,
                             &(path_search_strategy_ptr->init_path_direction));
  path_search_strategy_ptr->enable_init_kappa_cost = splice_path_size > 1;
  // has valid path: limit_init_steer_margin
  path_search_strategy_ptr->limit_init_steer_margin = HasValidHistoryPath();

  // is plan from start point to end point
  PlanDirectionDecision(start_point, dest_region_with_angle,
                        open_space_env_structured_info.parking_scenario_type,
                        &(path_search_strategy_ptr->is_plan_from_start),
                        &(path_search_strategy_ptr->space_structure),
                        &(path_search_strategy_ptr->park_direction));

  // cut off strategy: default no cut off
  SteerCutoffStrategy(open_space_env_structured_info,
                      path_search_strategy_ptr->is_plan_from_start,
                      &(path_search_strategy_ptr->cut_off_strategy));

  // use_larger_curvature: default false
  KappaStrategy(&(path_search_strategy_ptr->use_larger_curvature));

  // init moving direction strategy
  InitMoveDirectionStrategy(splice_path_size, start_gear,
                            &(path_strategy->init_moving_direction));
  // use geometry planer strategy
  UseGeometryStrategy(open_space_env_structured_info.parking_scenario_type,
                      path_search_strategy_ptr->is_plan_from_start,
                      path_strategy->init_moving_direction,
                      &(path_search_strategy_ptr->use_geometry_strategy));
  // local collision free search strategy
  LocalCollisionFreeSearchStrategy(
      path_search_strategy_ptr->is_plan_from_start,
      path_search_strategy_ptr->space_structure,
      &(path_search_strategy_ptr->collision_free_search_strategy));
  // use trace path strategy
  UseTracePathStrategy(start_point, open_space_env_structured_info, trace_path,
                       path_strategy);
  // dead end scenario strategy
  const auto scenario_diffculty_type =
      open_space_env_structured_info.parking_scenario_diffculty_type;
  path_search_strategy_ptr->is_dead_end_scenario =
      (scenario_diffculty_type & DEADEND_SCENARIO) != 0;
  path_search_strategy_ptr->is_narrow_passage_scenario =
      (scenario_diffculty_type & NARROW_PASSAGE_SCENARIO) != 0;
  path_search_strategy_ptr->is_nns_adjust_senario =
      open_space_env_structured_info.is_in_nns_adjust_scenario;
  path_search_strategy_ptr->reference_line =
      open_space_path_info.reference_line;
}

void OpenSpacePathProvider::PlanDirectionDecision(
    const common::PathPoint& start_point,
    const DestRegionWithAng& dest_region_with_angle,
    const ParkingScenarioType& parking_scenario_type,
    bool* const is_plan_from_start, SpaceStructure* const space_structure,
    ParkDirection* const park_direction) {
  if (nullptr == space_structure || nullptr == park_direction) {
    AERROR << "PlanDirectionDecision input check fails";
    return;
  }
  // TODO(gsb):  lateral replan consider use plan from start
  switch (parking_scenario_type) {
    case ParkingScenarioType::LEFT_VERTICAL_PARKING_IN:
    case ParkingScenarioType::RIGHT_VERTICAL_PARKING_IN:
    case ParkingScenarioType::LEFT_OBLIQUE_PARKING_IN:
    case ParkingScenarioType::RIGHT_OBLIQUE_PARKING_IN:
    case ParkingScenarioType::LEFT_LATERAL_PARKING_IN:
    case ParkingScenarioType::RIGHT_LATERAL_PARKING_IN: {
      // decide parking_in search direction
      const auto dest_region_polygon = std::get<0>(dest_region_with_angle);
      const auto start_point_polygon =
          common::VehicleConfigHelper::GetPolygon2dWithBuffer(
              start_point.x(), start_point.y(), start_point.theta());

      if (start_point_polygon.HasOverlap(dest_region_polygon)) {
        // end point has more slack area than start point
        // if has overlap meanwhile doesn't have extra space, plan from start to reduce shift number
        if (parking_scenario_type == LEFT_LATERAL_PARKING_IN ||
            parking_scenario_type == RIGHT_LATERAL_PARKING_IN) {
          // if has extra space in lateral, remain plan from end to get accurate position
          *is_plan_from_start = !HasExtraSpaceNearby();
          ADEBUG << "lateral direction is from "
                 << (*is_plan_from_start ? "start" : "end");
        } else {
          // if isn't lateral parking in, do not considering right now
          *is_plan_from_start = true;
          ADEBUG << "direct direction is from start";
        }
      } else {
        *is_plan_from_start = false;
        // has not overlap, remain plan from end to get accurate position
        ADEBUG << "direct direction is from end";
      }
      *park_direction = PARKIN;
      break;
    }
    case ParkingScenarioType::FORWARD_OBLIQUE_PARKING_OUT:
    case ParkingScenarioType::FORWARD_VERTICAL_PARKING_OUT: {
      *park_direction = FORWARDPARKOUT;
      *is_plan_from_start = true;
      break;
    }
    case ParkingScenarioType::LEFT_VERTICAL_PARKING_OUT:
    case ParkingScenarioType::LEFT_OBLIQUE_PARKING_OUT:
    case ParkingScenarioType::LEFT_LATERAL_PARKING_OUT: {
      *park_direction = LEFTPARKOUT;
      *is_plan_from_start = true;
      break;
    }
    case ParkingScenarioType::RIGHT_VERTICAL_PARKING_OUT:
    case ParkingScenarioType::RIGHT_OBLIQUE_PARKING_OUT:
    case ParkingScenarioType::RIGHT_LATERAL_PARKING_OUT: {
      *park_direction = RIGHTPARKOUT;
      *is_plan_from_start = true;
      break;
    }
    default: {
      *park_direction = NODIRECTION;
      *is_plan_from_start = true;
      ADEBUG << "plan direct direction is from start";
      break;
    }
  }
  ADEBUG << "plan direction is" << *is_plan_from_start;
  switch (parking_scenario_type) {
    case ParkingScenarioType::RIGHT_LATERAL_PARKING_OUT:
    case ParkingScenarioType::LEFT_LATERAL_PARKING_IN:
    case ParkingScenarioType::LEFT_LATERAL_PARKING_OUT:
    case ParkingScenarioType::RIGHT_LATERAL_PARKING_IN: {
      // In these scenarios, rs curvature is different from exploration
      *space_structure = SpaceStructure::LAT_PARK_LOT;
      break;
    }
    default: {
      break;
    }
  }
}

void OpenSpacePathProvider::SteerCutoffStrategy(
    const OpenSpaceEnvStructuredInfo& open_space_env_structured_info,
    const bool is_plan_from_start, int* const cut_off_strategy) {
  if (nullptr == cut_off_strategy) {
    return;
  }
  const auto& parking_scenario_type =
      open_space_env_structured_info.parking_scenario_type;
  const bool is_parking_inwards =
      open_space_env_structured_info.is_parking_inwards;
  if ((open_space_env_structured_info.parking_scenario_diffculty_type &
       NARROW_PASSAGE_SCENARIO) != 0) {
    *cut_off_strategy = 0;
    return;
  }
  switch (parking_scenario_type) {
    case ParkingScenarioType::LEFT_LATERAL_PARKING_IN: {
      *cut_off_strategy = is_plan_from_start ? -1 : 1;
      break;
    }
    case ParkingScenarioType::RIGHT_LATERAL_PARKING_IN: {
      *cut_off_strategy = is_plan_from_start ? 1 : -1;
      break;
    }
    // case ParkingScenarioType::LEFT_OBLIQUE_PARKING_IN:
    case ParkingScenarioType::LEFT_VERTICAL_PARKING_IN: {
      *cut_off_strategy = (is_parking_inwards || is_plan_from_start) ? 0 : -1;
      break;
    }
    // case ParkingScenarioType::RIGHT_OBLIQUE_PARKING_IN:
    case ParkingScenarioType::RIGHT_VERTICAL_PARKING_IN: {
      *cut_off_strategy = (is_parking_inwards || is_plan_from_start) ? 0 : 1;
      break;
    }
    default:
      *cut_off_strategy = 0;
      break;
  }
}

void OpenSpacePathProvider::UseGeometryStrategy(
    const ParkingScenarioType& parking_scenario_type,
    const bool plan_from_start, const int init_move_direction,
    GeometryStrategy* const use_geometry_strategy) {
  if (nullptr == use_geometry_strategy) {
    return;
  }
  // if start point is moving, filter geometry path with different sign
  use_geometry_strategy->consider_kappa_diff = init_move_direction != 0;
  static constexpr double kMaxRange = 1e3;
  static constexpr double later_slot_lat_range = 3.0;
  switch (parking_scenario_type) {
    case ParkingScenarioType::LEFT_VERTICAL_PARKING_IN:
    case ParkingScenarioType::LEFT_OBLIQUE_PARKING_IN:
    case ParkingScenarioType::RIGHT_VERTICAL_PARKING_IN:
    case ParkingScenarioType::RIGHT_OBLIQUE_PARKING_IN:
    case ParkingScenarioType::LEFT_VERTICAL_PARKING_OUT:
    case ParkingScenarioType::RIGHT_VERTICAL_PARKING_OUT:
    case ParkingScenarioType::LEFT_OBLIQUE_PARKING_OUT:
    case ParkingScenarioType::RIGHT_OBLIQUE_PARKING_OUT:
    case ParkingScenarioType::LEFT_LATERAL_PARKING_OUT:
    case ParkingScenarioType::RIGHT_LATERAL_PARKING_OUT: {
      use_geometry_strategy->use_geometry =
          replan_status_ ==
                  static_cast<uint32_t>(OpenSpaceStatus::DYNAMIC_REPLAN)
              ? UseGeometry::ONLY_USE
              : UseGeometry::USE_FIRST;
      use_geometry_strategy->longitudal_bound.emplace_back(-kMaxRange,
                                                           kMaxRange);
      use_geometry_strategy->use_purpose.emplace_back(
          UseGeometryPurpose::PRECISEPOSE);
      use_geometry_strategy->geometry_path_type.emplace_back(
          GeometryConnectionType::CYCLE_STRAIGHT);
      break;
    }
    case ParkingScenarioType::LEFT_LATERAL_PARKING_IN:
    case ParkingScenarioType::RIGHT_LATERAL_PARKING_IN: {
      // use geometry only if dynamic replan or dynamic replan + target update
      if ((replan_status_ == OpenSpaceStatus::DYNAMIC_REPLAN ||
           static_cast<int>(replan_status_) ==
               (static_cast<int>(OpenSpaceStatus::DYNAMIC_REPLAN) +
                static_cast<int>(OpenSpaceStatus::TARGET_UPDATE)))) {
        ADEBUG << "use geometry plan once";
        use_geometry_strategy->use_geometry = UseGeometry::ONLY_USE;
        use_geometry_strategy->longitudal_bound.emplace_back(
            -later_slot_lat_range, later_slot_lat_range);
        use_geometry_strategy->use_purpose.emplace_back(
            UseGeometryPurpose::PRECISEPOSE);
        use_geometry_strategy->geometry_path_type.emplace_back(
            GeometryConnectionType::CYCLE_STRAIGHT);
      } else if (frame_->open_space_info().entered_lateral_parking_slot(
                     false)) {
        // replan due to obstacle collision
        ADEBUG << "plan_from_start: " << plan_from_start;
        if (plan_from_start) {
          use_geometry_strategy->use_geometry = UseGeometry::USE_FIRST_LAST;
          // add precise pose type
          use_geometry_strategy->use_purpose.emplace_back(
              UseGeometryPurpose::PRECISEPOSE);
          use_geometry_strategy->longitudal_bound.emplace_back(
              -later_slot_lat_range, later_slot_lat_range);
          use_geometry_strategy->geometry_path_type.emplace_back(
              GeometryConnectionType::CYCLE_STRAIGHT);
          // add precise angle type
          use_geometry_strategy->use_purpose.emplace_back(
              UseGeometryPurpose::PRECISEANGLE);
          use_geometry_strategy->longitudal_bound.emplace_back(
              -later_slot_lat_range, later_slot_lat_range);
          use_geometry_strategy->geometry_path_type.emplace_back(
              GeometryConnectionType::CYCLE_STRAIGHT);
        } else {
          use_geometry_strategy->use_geometry = UseGeometry::USE_FIRST;
          use_geometry_strategy->use_purpose.emplace_back(
              UseGeometryPurpose::PRECISEPOSE);
          use_geometry_strategy->longitudal_bound.emplace_back(
              -later_slot_lat_range, later_slot_lat_range);
          use_geometry_strategy->geometry_path_type.emplace_back(
              GeometryConnectionType::CYCLE_STRAIGHT);
        }
      } else {
        use_geometry_strategy->use_geometry = UseGeometry::NOT_USE;
      }
      break;
    }
    case ParkingScenarioType::CONTROL_CALIBRATION_MODE: {
      use_geometry_strategy->use_geometry = UseGeometry::USE_FIRST;
      use_geometry_strategy->use_purpose.emplace_back(
          UseGeometryPurpose::PRECISEPOSE);
      use_geometry_strategy->longitudal_bound.emplace_back(-kMaxRange,
                                                           kMaxRange);
      use_geometry_strategy->geometry_path_type.emplace_back(
          GeometryConnectionType::CYCLE_STRAIGHT);
      use_geometry_strategy->use_purpose.emplace_back(
          UseGeometryPurpose::PRECISEPOSE);
      use_geometry_strategy->longitudal_bound.emplace_back(-kMaxRange,
                                                           kMaxRange);
      use_geometry_strategy->geometry_path_type.emplace_back(
          GeometryConnectionType::STRAIGHT_CYCLE);
      break;
    }
    default:
      use_geometry_strategy->use_geometry = UseGeometry::NOT_USE;
      break;
  }
}

void OpenSpacePathProvider::KappaStrategy(bool* const use_larger_curvature) {
  if (nullptr == use_larger_curvature) {
    AERROR << " check KappaStrategy is null "
           << (nullptr == use_larger_curvature);
    return;
  }
  is_entered_special_domain_ =
      frame_->open_space_info().entered_lateral_parking_slot(
          is_entered_special_domain_);
  *use_larger_curvature = is_entered_special_domain_;
}

void OpenSpacePathProvider::InitMoveDirectionStrategy(
    const size_t splice_path_size, const soc::Chassis::GearPosition& start_gear,
    int* const move_direction_strategy) {
  if (nullptr == move_direction_strategy) {
    AERROR << "InitMoveDirectionStrategy input check fails";
    return;
  }
  if (splice_path_size < 2) {
    *move_direction_strategy = 0;
  } else {
    if (start_gear == soc::Chassis::GEAR_DRIVE) {
      *move_direction_strategy = 1;
    } else if (start_gear == soc::Chassis::GEAR_REVERSE) {
      *move_direction_strategy = -1;
    } else {
      *move_direction_strategy = 0;
    }
  }
}

void OpenSpacePathProvider::LocalCollisionFreeSearchStrategy(
    const bool plan_from_start, const SpaceStructure& space_structure,
    CollisionFreeSearchStrategy* const collision_free_search_strategy) {
  if (nullptr == collision_free_search_strategy) {
    AERROR << "LocalCollisionFreeSearchStrategy input check fails";
    return;
  }
  const bool is_replan_due_to_collision =
      (replan_status_ & static_cast<uint32_t>(
                            OpenSpaceStatus::BLOCK_BY_STATIC_OBSTACLE)) != 0U ||
      (replan_status_ & static_cast<uint32_t>(OpenSpaceStatus::BLOCK_BY_USS)) !=
          0U;
  if (is_replan_due_to_collision &&
      (FLAGS_enable_change_buffer_when_gear_changed ||
       space_structure == SpaceStructure::LAT_PARK_LOT)) {
    ADEBUG << "use local collision free search once";
    collision_free_search_strategy->replan_due_to_collision = true;
    collision_free_search_strategy->collision_free_dist =
        plan_from_start ? FLAGS_avp_ego_inflated_buffer_for_checking_collision
                        : FLAGS_avp_ego_inflated_buffer_for_checking_collision +
                              config_.open_space_path_provider_config()
                                  .warm_start_config()
                                  .extra_distance_for_rs();
    const auto* ptr_last_frame = injector_->frame_history()->Latest();
    if (ptr_last_frame != nullptr) {
      collision_free_search_strategy->collision_path_point =
          ptr_last_frame->open_space_info()
              .future_collision_point()
              .path_point();
    }
  }
}

void OpenSpacePathProvider::UseTracePathStrategy(
    const common::PathPoint& start_point,
    const OpenSpaceEnvStructuredInfo& open_space_env_structured_info,
    const DiscretizedPath& trace_path, PathStrategy* const path_strategy) {
  if (nullptr == path_strategy) {
    AERROR << "UseTracePathStrategy input check fails";
    return;
  }
  auto* path_search_strategy_ptr = &(path_strategy->path_search_strategy);
  if ((replan_status_ & static_cast<uint32_t>(OpenSpaceStatus::TRACE_REPLAN)) !=
      0U) {
    bool is_lon_reach = false;
    bool is_lat_reach = false;
    GetAdcStatusOnTracePath(
        open_space_env_structured_info.parking_scenario_type, trace_path,
        start_point, &is_lon_reach, &is_lat_reach);
    if (open_space_env_structured_info.is_in_nns_adjust_scenario) {
      path_strategy->disable_search = false;
    } else {
      const auto& previous_frame = injector_->frame_history()->Latest();
      if (nullptr != previous_frame &&
          previous_frame->open_space_info().replan_triggered_by_speed_plan()) {
        path_strategy->disable_search = false;
      } else {
        path_strategy->disable_search = !is_lon_reach && is_lat_reach;
      }
    }
    path_search_strategy_ptr->trace_adjust_search_strategy.is_trace_adjust =
        !is_lon_reach && !is_lat_reach;
  }
  AINFO << "disable search " << path_strategy->disable_search;
  if (path_search_strategy_ptr->trace_adjust_search_strategy.is_trace_adjust) {
    LocalTraceAdjustSearchStrategy(
        trace_path, path_search_strategy_ptr,
        &path_search_strategy_ptr->trace_adjust_search_strategy);
  }
}

void OpenSpacePathProvider::LocalTraceAdjustSearchStrategy(
    const DiscretizedPath& trace_path,
    PathSearchStrategy* const path_search_strategy,
    TraceAdjustSearchStrategy* const trace_adjust_search_strategy) {
  if (nullptr == path_search_strategy ||
      nullptr == trace_adjust_search_strategy) {
    AERROR << "TraceAdjustSearchStrategy input check fails";
    return;
  }
  trace_adjust_search_strategy->trace_path = trace_path;
  trace_adjust_search_strategy->finish_l_threshold =
      kTraceAdjustLatDiffThreshold;
  trace_adjust_search_strategy->finish_theta_threshold =
      kTraceAdjustThetaDiffThreshold;
  trace_adjust_search_strategy->target_s = kTraceAdjustTargetS;
  trace_adjust_search_strategy->xy_bounds.resize(4);
  trace_adjust_search_strategy->xy_bounds.at(0) = -kTraceAdjustBound;
  trace_adjust_search_strategy->xy_bounds.at(1) = kTraceAdjustBound;
  trace_adjust_search_strategy->xy_bounds.at(2) = -kTraceAdjustBound;
  trace_adjust_search_strategy->xy_bounds.at(3) = kTraceAdjustBound;
  path_search_strategy->cut_off_strategy = 0;
  path_search_strategy->is_plan_from_start = true;
}

void OpenSpacePathProvider::GetAdcStatusOnTracePath(
    const ParkingScenarioType& parking_scenario_type,
    const DiscretizedPath& trace_path, const common::PathPoint& start_point,
    bool* const is_lon_reach, bool* const is_lat_reach) {
  if (nullptr == is_lon_reach || nullptr == is_lat_reach) {
    AERROR << "GetAdcStatusOnTracePath input check fails";
    return;
  }
  common::SLPoint start_point_sl;
  if (trace_path.empty() ||
      !trace_path.XYToSL(start_point.x(), start_point.y(), &start_point_sl)) {
    // initialize sl_point by sl_start_point
    ADEBUG << "Error init apa trace path " << trace_path.size();
    return;
  }
  constexpr double kLatDiffThreshold = 0.15;
  *is_lat_reach = std::fabs(start_point_sl.l()) < kLatDiffThreshold;

  const auto& previous_frame = injector_->frame_history()->Latest();
  if (nullptr != previous_frame) {
    const auto& pre_partitioned_paths =
        previous_frame->open_space_info().partitioned_paths();
    if (pre_partitioned_paths.path_type ==
            planning_internal::PathUpdateStatus::TRACE_PATH &&
        !pre_partitioned_paths.path_set.empty() &&
        pre_partitioned_paths.path_idx !=
            pre_partitioned_paths.path_set.size() - 1) {
      *is_lon_reach = false;
      return;
    }
  }
  const bool is_lateral_park_in =
      parking_scenario_type == LEFT_LATERAL_PARKING_IN ||
      parking_scenario_type == RIGHT_LATERAL_PARKING_IN;
  *is_lon_reach = (trace_path.back().s() - start_point_sl.s()) <
                  (is_lateral_park_in
                       ? FLAGS_lat_spot_enable_trace_replan_dist_threshold
                       : FLAGS_non_lat_spot_enable_trace_replan_dist_threshold);
}

void OpenSpacePathProvider::ForceInitDirectionStrategy(
    const OpenSpaceEnvStructuredInfo& open_space_env_structured_info,
    size_t splice_path_size, const soc::Chassis::GearPosition& start_gear,
    int* const init_gear_direction) const {
  if (nullptr == init_gear_direction) {
    AERROR << "ForceInitDirectionStrategy input check fails";
    return;
  }
  const bool is_gear_valid = start_gear == soc::Chassis::GEAR_DRIVE ||
                             start_gear == soc::Chassis::GEAR_REVERSE;
  if (!is_gear_valid) {
    return;
  }
  ADEBUG << "splice_path_size " << splice_path_size;
  if (splice_path_size < 2 &&
      ((replan_status_ &
        static_cast<uint32_t>(OpenSpaceStatus::YAW_TRACK_ABNORMAL)) != 0U ||
       (replan_status_ &
        static_cast<uint32_t>(OpenSpaceStatus::BLOCK_BY_USS)) != 0U ||
       (replan_status_ &
        static_cast<uint32_t>(
            TL::planning::OpenSpaceStatus::END_ANGLE_UNREACHABLE)) != 0U)) {
    ADEBUG << " need force init move direction different from current";
    *init_gear_direction = start_gear == soc::Chassis::GEAR_DRIVE ? -1 : 1;
    return;
  }
  const bool is_lateral_park_in =
      (open_space_env_structured_info.parking_scenario_type ==
           LEFT_LATERAL_PARKING_IN ||
       open_space_env_structured_info.parking_scenario_type ==
           RIGHT_LATERAL_PARKING_IN);
  if ((replan_status_ &
       static_cast<uint32_t>(OpenSpaceStatus::ENTER_SPECIAL_DOMAIN)) != 0U &&
      is_lateral_park_in) {
    ADEBUG << " need force init move direction as same with current";
    *init_gear_direction = start_gear == soc::Chassis::GEAR_DRIVE ? 1 : -1;
    return;
  }
  if ((replan_status_ &
       static_cast<uint32_t>(OpenSpaceStatus::DYNAMIC_REPLAN)) != 0U) {
    ADEBUG << " need force init move direction as same with current";
    *init_gear_direction = start_gear == soc::Chassis::GEAR_DRIVE ? 1 : -1;
    return;
  }
  if ((replan_status_ & static_cast<uint32_t>(OpenSpaceStatus::TRACE_REPLAN)) !=
          0U &&
      !frame_->IsVehicleStandStill()) {
    ADEBUG << " need force init move direction as same with current";
    *init_gear_direction = start_gear == soc::Chassis::GEAR_DRIVE ? 1 : -1;
  }
}

bool OpenSpacePathProvider::HasExtraSpaceNearby() {
  static constexpr double kLatExtensionBuffer = 1.2;
  bool is_left_empty = true;
  bool is_right_empty = true;
  std::array<common::math::Vec2d, 4> parking_lot_vertices =
      frame_->get_parking_lot_vertices();
  std::vector<Vec2d> left_polygon_points;
  std::vector<Vec2d> right_polygon_points;
  auto left_mid = (parking_lot_vertices.at(0) + parking_lot_vertices.at(1)) /
                  2;  // parking_lot left edge mid
  auto right_mid = (parking_lot_vertices.at(2) + parking_lot_vertices.at(3)) /
                   2;  // parking_lot right edge mid
  auto left_edge =
      common::math::LineSegment2d(parking_lot_vertices.at(0), left_mid);
  auto right_edge =
      common::math::LineSegment2d(parking_lot_vertices.at(3), right_mid);
  double parking_lot_enu_angle =
      (parking_lot_vertices.at(3) - parking_lot_vertices.at(0)).Angle();
  left_edge.Translate(kLatExtensionBuffer, parking_lot_enu_angle + M_PI);
  right_edge.Translate(kLatExtensionBuffer, parking_lot_enu_angle);
  common::math::Polygon2d left_polygon({left_edge.start(), left_edge.end(),
                                        left_mid, parking_lot_vertices.at(0)});
  common::math::Polygon2d right_polygon({parking_lot_vertices.at(3), right_mid,
                                         right_edge.end(), right_edge.start()});
  for (const auto& obs : frame_->open_space_info()
                             .open_space_path_info()
                             .obstacles_segments_vec) {
    if (is_left_empty && left_polygon.HasOverlap(obs.first)) {
      is_left_empty = false;
    }
    if (is_right_empty && right_polygon.HasOverlap(obs.first)) {
      is_right_empty = false;
    }
    if (!is_right_empty && !is_left_empty) {
      ADEBUG << "is_left_empty: " << is_left_empty << "  "
             << " is_right_empty: " << is_right_empty;
      return false;
    }
  }
  ADEBUG << "is_left_empty: " << is_left_empty << "  "
         << " is_right_empty: " << is_right_empty;
  return true;
}

void OpenSpacePathProvider::PrePlan() {
  if (frame_->local_view()
          .GetFunctionManagerIn()
          ->fct_avp_in()
          .sys_run_state() != functionmanager::AvpFctIn::PARKSTART) {
    return;
  }
  AERROR << "PrePlaning .............";
  UpdateCurTaskReplanStatus();
  std::vector<OpenSpacePathInput> open_space_path_inputs;
  std::vector<OpenSpacePathOutput> open_space_path_outputs;
  int i = 0;
  for (const auto& path_info_pair :
       frame_->open_space_info().open_space_path_info_map()) {
    if (path_info_pair.first < 0) {
      continue;
    }
    i++;
    open_space_path_inputs.emplace_back();
    open_space_path_outputs.emplace_back();
    LoadOptimizerData(path_info_pair.second, &open_space_path_inputs.back(),
                      &open_space_path_outputs.back());
    open_space_path_inputs.back().path_id = path_info_pair.first;
  }
  AERROR << " open space path info map size: " << i;
  open_space_thread_manager_.PrePlan(open_space_path_inputs);
  optimizer_multi_debugs_.clear();
  for (int i = 0; i < open_space_path_inputs.size(); ++i) {
    optimizer_multi_debugs_.emplace_back();
    optimizer_multi_debugs_.back().first = open_space_path_inputs[i].path_id;
    if (!open_space_thread_manager_.GetParkOutput(
            open_space_path_inputs[i].path_id, &open_space_path_outputs[i],
            &optimizer_multi_debugs_.back().second)) {
      optimizer_multi_debugs_.pop_back();
    }
  }
}

}  // namespace planning
}  // namespace TL
