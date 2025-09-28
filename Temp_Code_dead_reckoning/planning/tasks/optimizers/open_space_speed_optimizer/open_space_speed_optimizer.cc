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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <ctime>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "planning/tasks/optimizers/open_space_speed_optimizer/open_space_speed_optimizer.h"
#include "planning/tasks/optimizers/open_space_speed_optimizer/path_handle.h"
#include "planning/tasks/optimizers/open_space_speed_optimizer/st_sample_cost.h"

#include "common/configs/vehicle_config_helper.h"
#include "common/file/global_data.h"
#include "common/math/double_type.h"
#include "common/math/math_utils.h"
#include "common/status/status.h"
#include "common/thread/thread_pool.h"
#include "planning/common/frame.h"
#include "planning/common/open_space_info.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/speed/speed_data.h"
#include "planning/common/trajectory/discretized_trajectory.h"
#include "planning/common/trajectory1d/piecewise_acceleration_trajectory1d.h"
#include "planning/localview/local_view.h"
#include "planning/tasks/optimizers/open_space_speed_optimizer/st_sample_curves.h"
#include "planning/tasks/optimizers/speed_optimizer.h"

#include "planning/proto/planning_config.pb.h"
#include "proto/common/error_code.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/types.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/fsm/avp_fct.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/planning/planning_internal.pb.h"
#include "proto/planning/planning_status.pb.h"
#include "proto/planning/sl_boundary.pb.h"

namespace TL {
namespace planning {
using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::TrajectoryPoint;
using TL::common::math::double_type::DefinitelyGreater;
using TL::common::math::double_type::DefinitelyLess;

OpenSpaceSpeedOptimizer::OpenSpaceSpeedOptimizer(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : SpeedOptimizer(config, injector),
      path_handle_(config_.open_space_speed_optimizer_config()),
      trajectory_unit_t_(
          config_.open_space_speed_optimizer_config().tarjectory_unit_t()) {}

Status OpenSpaceSpeedOptimizer::Process(Frame* frame) {
  if (frame == nullptr) {
    const std::string msg = "frame is nullptr!";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_SPEED_OPTIMIZER_ERROR, msg);
  }

  st_debug_info_.Clear();
  const DiscretizedPath& complete_path =
      frame->open_space_info().chosen_partitioned_path().first;
  const Chassis::GearPosition& gear =
      frame->open_space_info().chosen_partitioned_path().second;
  TrajGearPair trajectory_gear;
  trajectory_gear.second = gear;

  InitInteractiveStage(gear);

  auto pre_check_msg = SpeedPlanPreCheck(
      complete_path, gear, frame_->open_space_info().get_is_stop_path());
  if (!pre_check_msg.empty()) {
    GenerateStopTrajectory(&trajectory_gear);
    AERROR << pre_check_msg;
    st_debug_info_.set_message(pre_check_msg);
    RecordDebug();
    return Status::OK();
  }

  auto start_point = frame->PlanningStartPoint();
  bool is_rpa_direct_mode = false;
  UpdateSpeedPlanInputInfo(gear, &start_point, &is_rpa_direct_mode);
  auto vehicle_state = frame->vehicle_state();
  DiscretizedPath candidate_path;
  std::shared_ptr<const FreeSpaceOutArray> freespace_out_array;
  if (frame->local_view().HasFreeSpaceOutArray()) {
    freespace_out_array = frame_->local_view().GetFreeSpaceOutArray();
  }
  const bool is_mirror_fold = injector_->planning_context()
                                  ->planning_status()
                                  .avp_to_hmi()
                                  .is_mirror_fold();
  auto path_handle_msg = path_handle_.Process(
      complete_path, frame_->obstacles(), freespace_out_array, vehicle_state,
      frame_->IsVehicleStandStill(), is_forward_, is_rpa_direct_mode,
      speed_bound_info_, is_mirror_fold, frame_->open_space_info(),
      &candidate_path, &interactive_stage_, frame_->mutable_open_space_info());
  // LCOV_EXCL_START
  if (!path_handle_msg.empty()) {
    AERROR << path_handle_msg;
    GenerateStopTrajectory(&trajectory_gear);
    st_debug_info_.set_message(path_handle_msg);
    RecordDebug();
    return Status::OK();
  }
  // LCOV_EXCL_STOP

  UpdateStDebugInfo(start_point.v(), start_point.a(), complete_path.Length(),
                    candidate_path.Length(), vehicle_state.linear_velocity(),
                    vehicle_state.linear_acceleration(),
                    path_handle_.GetSpeedLimits(),
                    path_handle_.GetSpeedLimitUnitS());
  if (AvpSpeedPlanCollisionInfo::INIT != interactive_stage_) {
    std::string msg = "interavtive is " +
                      AvpSpeedPlanCollisionInfo::SpeedTaskInteractiveStage_Name(
                          interactive_stage_);
    GenerateStopTrajectory(&trajectory_gear);
    st_debug_info_.set_message(msg);
    RecordDebug();
    return Status::OK();
  }

  UpdateSampleParams(start_point, is_forward_, candidate_path.Length());
  std::string msg;
  if (!GenerateTrajectory(st_sample_params_, candidate_path, &trajectory_gear,
                          &msg)) {
    // LCOV_EXCL_START
    msg = "generate trajectory failed!";
    AERROR << msg;
    st_debug_info_.set_message(msg);
    RecordDebug();
    return Status(ErrorCode::PLANNER_PARKING_SPEED_OPTIMIZER_ERROR, msg);
    // LCOV_EXCL_STOP
  }

  st_debug_info_.set_message(msg);
  RecordDebug();

  return Status::OK();
}

// LCOV_EXCL_START
Status OpenSpaceSpeedOptimizer::Reset() {  // NOLINT
  is_forward_ = true;
  interactive_stage_ = AvpSpeedPlanCollisionInfo::INIT;
  speed_bound_info_.Clear();
  last_curve_ = nullptr;
  min_costs_.clear();
  st_debug_info_.Clear();
  AINFO << " opne space optimmizer is reseted! ";
  return Status::OK();
}

// LCOV_EXCL_STOP

void OpenSpaceSpeedOptimizer::GenerateStopTrajectory(
    TrajGearPair* const trajectory_gear_ptr) {
  AINFO << "no space to move, stop now!";

  if (nullptr == trajectory_gear_ptr) {
    AERROR << "GenerateStopTrajectory init check failed";
    return;
  }

  static constexpr double kStopACC = 0.3;
  soc::Chassis::GearPosition gear = trajectory_gear_ptr->second;
  double stop_acc = -kStopACC;
  if (soc::Chassis::GEAR_REVERSE == gear) {
    stop_acc = kStopACC;
  }

  double offset_s = 0.0;
  double offset_relative_t = 0.0;
  if (!frame_->open_space_info().is_gear_changed()) {
    offset_s = frame_->PlanningStartPoint().path_point().s();
    offset_relative_t = frame_->PlanningStartPoint().relative_time();
  }

  DiscretizedTrajectory& trajectory = trajectory_gear_ptr->first;
  trajectory.clear();
  TrajectoryPoint start_p = frame_->PlanningStartPoint();
  start_p.set_v(0.0);
  start_p.set_a(stop_acc);

  if (!frame_->open_space_info().chosen_partitioned_path().first.empty()) {
    auto kappa = frame_->open_space_info()
                     .chosen_partitioned_path()
                     .first.front()
                     .kappa();
    start_p.mutable_path_point()->set_kappa(kappa);
  } else {
    start_p.mutable_path_point()->set_kappa(0.0);
  }

  common::TrajectoryPoint trajectory_point;
  for (int i = 0; i < FLAGS_publish_trajectory_points_number; ++i) {
    trajectory_point = start_p;
    trajectory_point.mutable_path_point()->set_s(
        trajectory_point.path_point().s() + offset_s);
    trajectory_point.set_relative_time(i * trajectory_unit_t_ +
                                       offset_relative_t);
    trajectory.AppendTrajectoryPoint(trajectory_point);
  }

  frame_->mutable_open_space_info()->set_speed_optimizer_trajectory(
      (*trajectory_gear_ptr));
  last_curve_.reset();
}

bool OpenSpaceSpeedOptimizer::GenerateBackUpTrajectory(
    const StSampleParams& sample_params, const DiscretizedPath& candidate_path,
    TrajGearPair* const traj_gear_ptr) {
  if (nullptr == traj_gear_ptr) {
    AERROR << "input pointer is nullptr!";
    return false;
  }

  if (!DefinitelyGreater(sample_params.end_s, kMinSampleS)) {
    AERROR << " candidate path length is not greater 0.01!";
    return false;
  }

  static constexpr double kMinThreshold = 0.01;
  const double init_v = fabs(sample_params.start_v);
  if (!DefinitelyGreater(init_v, kMinThreshold)) {
    AERROR << " init_v is not greater than 0.01!";
    return false;
  }

  const double path_length = sample_params.end_s;
  const double dec = fmin(init_v * init_v / (2.0 * path_length),
                          fabs(common::VehicleConfigHelper::GetConfig()
                                   .vehicle_param()
                                   .max_deceleration()));
  const double dec_t = init_v / dec;
  auto ptr_trajectory =
      std::make_unique<PiecewiseAccelerationTrajectory1d>(0, init_v);
  ptr_trajectory->AppendSegment(-dec, dec_t);
  std::vector<double> s;
  std::vector<double> ds;
  std::vector<double> dds;
  const size_t num_of_knots =
      static_cast<size_t>(dec_t / trajectory_unit_t_) + 1;
  s.reserve(num_of_knots);
  ds.reserve(num_of_knots);
  dds.reserve(num_of_knots);
  for (size_t i = 0; i < num_of_knots; ++i) {
    auto t = static_cast<double>(i) * trajectory_unit_t_;
    if (!DefinitelyLess(t, dec_t)) {
      s.push_back(path_length);
      ds.push_back(0.0);
      dds.push_back(dec_t);
      break;
    }
    s.push_back(ptr_trajectory->Evaluate(0, t));
    ds.push_back(ptr_trajectory->Evaluate(1, t));
    dds.push_back(ptr_trajectory->Evaluate(2, t));
  }

  SpeedData candidate_speed;
  candidate_speed.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  for (size_t i = 1; i < num_of_knots; ++i) {
    if (DefinitelyLess(s[i], s[i - 1])) {
      // LCOV_EXCL_START
      AERROR << "unexpected decreasing s in speed data at time "
             << static_cast<double>(i) * trajectory_unit_t_
             << "with total time " << dec_t << " i = " << i;
      return false;
      // LCOV_EXCL_STOP
    }
    candidate_speed.AppendSpeedPoint(
        s[i], trajectory_unit_t_ * static_cast<double>(i), ds[i], dds[i],
        (dds[i] - dds[i - 1]) / trajectory_unit_t_);
    if (DefinitelyLess(path_length, s[i])) {
      break;
    }
  }

  if (!CombinePathAndSpeed(is_forward_, candidate_path, candidate_speed,
                           &traj_gear_ptr->first)) {
    AERROR << "combine path and speed failed!";
    return false;
  }

  frame_->mutable_open_space_info()->set_speed_optimizer_trajectory(
      (*traj_gear_ptr));
  last_curve_.reset();

  return true;
}

bool OpenSpaceSpeedOptimizer::CombinePathAndSpeed(
    const bool is_forward, const DiscretizedPath& path_points,
    const SpeedData& speed_points,
    DiscretizedTrajectory* const discretized_trajectory_ptr) {
  if (nullptr == discretized_trajectory_ptr || speed_points.empty() ||
      path_points.empty()) {
    AERROR << "init check failed!";
    return false;
  }

  discretized_trajectory_ptr->clear();
  const auto trajectory_size =
      static_cast<size_t>(speed_points.TotalTime() / trajectory_unit_t_ + 1);
  double offset_s = 0.0;
  double offset_relative_t = 0.0;
  if (!frame_->open_space_info().is_gear_changed()) {
    offset_s = frame_->PlanningStartPoint().path_point().s();
    offset_relative_t = frame_->PlanningStartPoint().relative_time();
  }

  common::SpeedPoint speed_point;
  common::PathPoint path_point;
  common::TrajectoryPoint trajectory_point;
  for (size_t i = 0; i < trajectory_size + 1; i++) {
    auto t = static_cast<double>(i) * trajectory_unit_t_;
    if (i < trajectory_size) {
      if (!speed_points.EvaluateByTime(t, &speed_point)) {
        // LCOV_EXCL_START
        AERROR << "Fail to get speed point with relative time " << t;
        return false;
        // LCOV_EXCL_STOP
      }
    } else {
      // in case the i point relative t is equal (i-1) point
      speed_point = speed_points.back();
      speed_point.set_t(t);
    }

    if (DefinitelyLess(path_points.Length(), speed_point.s())) {
      break;
    }

    path_point = path_points.Evaluate(speed_point.s());
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.mutable_path_point()->set_s(
        trajectory_point.path_point().s() + offset_s);
    trajectory_point.set_relative_time(speed_point.t() + offset_relative_t);
    if (is_forward) {
      trajectory_point.set_v(speed_point.v());
      trajectory_point.set_a(speed_point.a());
    } else {
      trajectory_point.set_v(0.0 - speed_point.v());
      trajectory_point.set_a(0.0 - speed_point.a());
    }

    discretized_trajectory_ptr->AppendTrajectoryPoint(trajectory_point);
  }

  ADEBUG << "path length before combine " << path_points.Length();
  ADEBUG << "trajectory length after combine "
         << discretized_trajectory_ptr->GetSpatialLength();

  return true;
}

std::string OpenSpaceSpeedOptimizer::SpeedPlanPreCheck(
    const DiscretizedPath& path, const Chassis::GearPosition& gear,
    const bool is_stop_path) {
  std::string msg;
  if (is_stop_path ||
      (Chassis::GEAR_DRIVE != gear && Chassis::GEAR_REVERSE != gear) ||
      path.empty()) {
    std::stringstream ss;
    ss << "chosen path is sotp path: " << is_stop_path
       << " gear: " << Chassis::GearPosition_Name(gear)
       << " path is empty: " << path.empty();
    return ss.str();
  }

  return msg;
}

void OpenSpaceSpeedOptimizer::UpdateSpeedPlanInputInfo(
    const Chassis::GearPosition& gear, TrajectoryPoint* const start_point,
    bool* const is_rpa_direct_mode) {
  is_forward_ = (gear == Chassis::GEAR_DRIVE);
  if (nullptr != start_point && frame_->open_space_info().is_gear_changed()) {
    start_point->set_v(0.0);
    start_point->set_a(0.0);
    last_curve_.reset();
  }

  speed_bound_info_ = is_forward_ ? config_.open_space_speed_optimizer_config()
                                        .apa_speed_bound_info()
                                        .forward_info()
                                  : config_.open_space_speed_optimizer_config()
                                        .apa_speed_bound_info()
                                        .reverse_info();
  if (injector_ != nullptr && injector_->planning_context() != nullptr) {
    auto parking_type = injector_->planning_context()
                            ->planning_status()
                            .avp_status()
                            .parking_type();
    bool is_rpa_mode =
        !frame_->local_view().GetFunctionManagerIn()->has_fct_avp_in()
            ? false
            : frame_->local_view()
                      .GetFunctionManagerIn()
                      ->fct_avp_in()
                      .sys_mode() == functionmanager::AvpFctIn::RPA;
    // for nns adjust scenario
    bool is_nns_adjust =
        (parking_type == TL::planning::AVPStatus::NNS_ADJUST);
    if (is_nns_adjust) {
      speed_bound_info_ = is_forward_
                              ? config_.open_space_speed_optimizer_config()
                                    .nns_adjust_speed_bound_info()
                                    .forward_info()
                              : config_.open_space_speed_optimizer_config()
                                    .nns_adjust_speed_bound_info()
                                    .reverse_info();
    }
    bool is_direct_mode =
        (parking_type == TL::planning::AVPStatus::DIRECT_FORWARD) ||
        (parking_type == TL::planning::AVPStatus::DIRECT_BACKWARD);
    if (is_rpa_mode) {
      speed_bound_info_ =
          is_forward_
              ? (is_direct_mode ? config_.open_space_speed_optimizer_config()
                                      .rpa_direct_speed_bound_info()
                                      .forward_info()
                                : config_.open_space_speed_optimizer_config()
                                      .rpa_speed_bound_info()
                                      .forward_info())
              : (is_direct_mode ? config_.open_space_speed_optimizer_config()
                                      .rpa_direct_speed_bound_info()
                                      .reverse_info()
                                : config_.open_space_speed_optimizer_config()
                                      .rpa_speed_bound_info()
                                      .reverse_info());
      if (nullptr != is_rpa_direct_mode) {
        *is_rpa_direct_mode = is_direct_mode;
      }
    }
  }
}

void OpenSpaceSpeedOptimizer::UpdateSampleParams(
    const common::TrajectoryPoint& start_point, const bool is_forward,
    const double end_s, const double start_s, const double end_v) {
  double start_v = fabs(start_point.v());
  double start_acc = start_point.a();
  if (!is_forward) {
    start_acc = -start_point.a();
  }

  st_sample_params_.start_s = start_s;
  st_sample_params_.start_v = start_v;
  st_sample_params_.start_acc = start_acc;
  st_sample_params_.end_s = end_s;
  st_sample_params_.end_v = fabs(end_v);
  st_sample_params_.unit_acc =
      config_.open_space_speed_optimizer_config().sample_unit_acc();
  st_sample_params_.unit_max_v =
      config_.open_space_speed_optimizer_config().sample_unit_max_v();
  st_sample_params_.efficiency_cost =
      config_.open_space_speed_optimizer_config().efficiency_cost();
  st_sample_params_.acc_cost =
      config_.open_space_speed_optimizer_config().acc_cost();
  st_sample_params_.jerk_cost =
      config_.open_space_speed_optimizer_config().jerk_cost();
  st_sample_params_.over_speed_cost_max =
      config_.open_space_speed_optimizer_config().over_speed_cost_max();
  st_sample_params_.over_speed_cost_min =
      config_.open_space_speed_optimizer_config().over_speed_cost_min();
  st_sample_params_.diff_cost =
      config_.open_space_speed_optimizer_config().diff_cost();
  st_sample_params_.speed_bound_info = speed_bound_info_;

  if (nullptr != last_curve_) {
    auto diff_time = CalDiffTimeFromLast();
    last_curve_->UpdateOriginByDiffTime(diff_time);
  }
}

bool OpenSpaceSpeedOptimizer::SampleStCurves(
    const StSampleParams& sample_params) {
  size_t curves_size = st_sample_curves_.SampleProcess(sample_params);
  return curves_size > 0;
}

bool OpenSpaceSpeedOptimizer::GetBestCurveIdx(int* const best_idx) {
  if (best_idx == nullptr) {
    AERROR << "input pointer is nullptr!";
    return false;
  }
  static constexpr size_t kMinThreadSize = 4;
  static constexpr size_t kMaxThreadSize = 1024;

  *best_idx = -1;
  auto* st_curves = st_sample_curves_.GetMutableStSampleCurves();
  const auto st_curve_size = st_sample_curves_.GetStSampleCurveSize();
  auto thread_count = static_cast<size_t>(
      config_.open_space_speed_optimizer_config().sample_thread_size());
  thread_count =
      common::math::Clamp(thread_count, kMinThreadSize, kMaxThreadSize);
  min_costs_.reserve(thread_count);
  min_costs_.assign(thread_count, {0, std::numeric_limits<double>::max()});

  const size_t count_per_thread = st_curve_size / thread_count + 1;
  std::vector<int> thread_indexs;
  for (size_t i = 0; i < thread_count; i++) {
    thread_indexs.emplace_back(i);
  }

  TL::common::thread::ThreadPool::ForEach(
      thread_indexs.begin(), thread_indexs.end(), [&](size_t thread_index) {
        common::sub_thread_name = "_open_space_speed_planning";
        const size_t start_index = thread_index * count_per_thread;
        const size_t end_index =
            std::min(start_index + count_per_thread, st_curve_size);
        StSampleCost st_sample_cost(
            st_sample_params_, path_handle_.GetSpeedLimits(), last_curve_,
            path_handle_.GetSpeedLimitUnitS(), trajectory_unit_t_ * 2.0,
            st_sample_curves_.GetMaxSampleT());
        double min_cost = std::numeric_limits<double>::max();
        size_t min_cost_index = 0;
        for (size_t i = start_index; i < end_index && i < st_curve_size; i++) {
          auto* st_curve = &st_curves->at(i);
          double cost = st_sample_cost.CalCurveCost(st_curve);
          if (DefinitelyLess(cost, min_cost)) {
            min_cost_index = i;
            min_cost = cost;
          }
        }
        if (thread_index < min_costs_.size()) {
          min_costs_[thread_index].first = min_cost_index;
          min_costs_[thread_index].second = min_cost;
        }
      });

  std::sort(min_costs_.begin(), min_costs_.end(),
            [](const std::pair<size_t, double>& left,
               const std::pair<size_t, double>& right) {
              return left.second < right.second;
            });
  *best_idx = static_cast<int>(min_costs_.front().first);

#if 0
  for (size_t i = 0; i < st_curve_size; i++) {
    AERROR << "curve: " << i << " " << st_curves->at(i).DebugInfo();
  }
#endif

  return (*best_idx) >= 0 && (*best_idx) < static_cast<int>(st_curve_size);
}

bool OpenSpaceSpeedOptimizer::SampleTrajectory(
    const StSampleParams& sample_params, const DiscretizedPath& candidate_path,
    TrajGearPair* const traj_gear_ptr) {
  if (traj_gear_ptr == nullptr) {
    AERROR << "input pointer is nullptr!";
    return false;
  }

  if (!SampleStCurves(sample_params)) {
    AERROR << "sample st curves failed!";
    return false;
  }

  int best_index = -1;
  if (!GetBestCurveIdx(&best_index)) {
    // LCOV_EXCL_START
    AERROR << "get best speed data failed!";
    return false;
    // LCO_EXCL_STOP
  }
  ADEBUG << "sample curve size: " << st_sample_curves_.GetStSampleCurveSize()
         << " best curve idx is: " << best_index << " "
         << st_sample_curves_.GetStSampleCurves()[best_index].DebugInfo();

  const auto& best_curve = st_sample_curves_.GetStSampleCurves()[best_index];
  st_debug_info_.set_best_curve(best_curve.DebugInfo());
  if (nullptr != last_curve_) {
    st_debug_info_.set_last_curve(last_curve_->DebugInfo());
  }

  last_curve_ = best_curve.Clone();
  SpeedData candidate_speed;
  if (!best_curve.Discrete(trajectory_unit_t_, &candidate_speed)) {
    // LCOV_EXCL_START
    AERROR << "get speed data from best curve failed!";
    return false;
    // LCOV_EXCL_STOP
  }

  if (!CombinePathAndSpeed(is_forward_, candidate_path, candidate_speed,
                           &traj_gear_ptr->first)) {
    AERROR << "combine path and speed failed!";
    return false;
  }

  frame_->mutable_open_space_info()->set_speed_optimizer_trajectory(
      (*traj_gear_ptr));

  return true;
}

bool OpenSpaceSpeedOptimizer::GenerateTrajectory(
    const StSampleParams& sample_params, const DiscretizedPath& candidate_path,
    TrajGearPair* const traj_gear_ptr, std::string* const msg) {
  if (nullptr == traj_gear_ptr || nullptr == msg) {
    AERROR << "input pointer is nullptr!";
    return false;
  }

  *msg = "generate sample trajectory!";
  if (!SampleTrajectory(sample_params, candidate_path, traj_gear_ptr)) {
    *msg = "generate backup trajectory!";
    if (!GenerateBackUpTrajectory(sample_params, candidate_path,
                                  traj_gear_ptr)) {
      *msg = "sample and backup trajectory failed!";
      GenerateStopTrajectory(traj_gear_ptr);
    }
  }
  ADEBUG << *msg;

  return true;
}

void OpenSpaceSpeedOptimizer::UpdateStDebugInfo(
    const double start_v, const double start_acc, const double total_s,
    const double end_s, const double actual_v, const double actual_acc,
    const std::vector<double>& speed_limits, const double speed_limit_unit_s) {
  st_debug_info_.set_start_v(start_v);
  st_debug_info_.set_start_acc(start_acc);
  st_debug_info_.set_total_s(total_s);
  st_debug_info_.set_end_s(end_s);
  st_debug_info_.set_actual_v(actual_v);
  st_debug_info_.set_actual_acc(actual_acc);
  auto* speed_limit_points = st_debug_info_.mutable_speed_limit_points();
  for (size_t i = 0; i < speed_limits.size(); i++) {
    auto s = static_cast<double>(i) * speed_limit_unit_s;
    auto* speed_limit_p = speed_limit_points->Add();
    speed_limit_p->set_s(s);
    speed_limit_p->set_limit_v(speed_limits[i]);
  }
}

void OpenSpaceSpeedOptimizer::RecordDebug() {
  if (FLAGS_enable_record_debug) {
    auto* st_debug = frame_->mutable_open_space_info()
                         ->mutable_debug_instance()
                         ->mutable_planning_data()
                         ->mutable_open_space()
                         ->mutable_st_sample_debug();
    st_debug->CopyFrom(st_debug_info_);
  }
}

void OpenSpaceSpeedOptimizer::InitInteractiveStage(
    const Chassis::GearPosition& gear) {
  if (Chassis::GEAR_PARKING == gear) {
    interactive_stage_ = AvpSpeedPlanCollisionInfo::INIT;
  }

  if (frame_->local_view().HasFunctionManagerIn()) {
    const auto& curr_sys_run_state = frame_->local_view()
                                         .GetFunctionManagerIn()
                                         ->fct_avp_in()
                                         .sys_run_state();
    if (functionmanager::AvpFctIn::PAUSE != curr_sys_run_state &&
        AvpSpeedPlanCollisionInfo::RUNNING == interactive_stage_) {
      interactive_stage_ = AvpSpeedPlanCollisionInfo::INIT;
    }
  }

  frame_->mutable_open_space_info()
      ->mutable_speed_plan_collision_info()
      ->set_speed_task_inter_stage(interactive_stage_);
}

double OpenSpaceSpeedOptimizer::CalDiffTimeFromLast() {
  if (nullptr != injector_ && nullptr != injector_->frame_history() &&
      nullptr != injector_->frame_history()->Latest() && nullptr != frame_) {
    const auto* last_frame = injector_->frame_history()->Latest();
    return frame_->vehicle_state().timestamp() +
           frame_->PlanningStartPoint().relative_time() -
           last_frame->vehicle_state().timestamp() -
           last_frame->PlanningStartPoint().relative_time();
  }

  return 0.0;
}

}  // namespace planning
}  // namespace TL
