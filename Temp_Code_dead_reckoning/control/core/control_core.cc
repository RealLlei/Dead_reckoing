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
#include "control/core/control_core.h"

#include "absl/strings/str_cat.h"
#include "common/adapters/adapter_gflags.h"
#include "common/file/file.h"
#include "common/file/log.h"
#include "common/math/math_utils.h"
#include "common/time/clock.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "control/common/control_gflags.h"

namespace TL {
namespace control {

using TL::soc::Chassis;
using TL::common::Clock;
using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::VehicleStateProvider;
// using TL::drivers::Ultrasonic;
using TL::localization::Localization;
using TL::planning::ADCTrajectory;

ControlCore::ControlCore() {}

// : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}
bool ControlCore::Init() {
  injector_ = std::make_shared<DependencyInjector>();
  local_view_ = std::make_shared<LocalView>();
  init_time_ = Clock::NowInMicroseconds();

  AINFO << "Control init, starting ...";

  ACHECK(common::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_))
      << "Unable to load control conf file: " + FLAGS_control_conf_file;

  AINFO << "Conf file: " << FLAGS_control_conf_file << " is loaded.";

  // Init mbd control
  mbd_control_.Init();

  // initial controller agent when not using control submodules
  if (controller_agent_.Init(injector_, &control_conf_).ok()) {
    // set controller
    ADEBUG << "original control";
    // monitor_logger_buffer_.ERROR("Control init controller failed!
    // Stopping...");
    return false;
  }

  return true;
}

Status ControlCore::ProduceControlCommand(ControlCommand* control_command) {
  Status status = CheckInput(local_view_);
  // check data
  bool estop = false;
  std::string estop_reason;
  if (!status.ok()) {
    AERROR_EVERY(100) << "Control input data failed: "
                      << status.error_message();
    control_command->mutable_engage_advice()->set_advice(
        TL::common::EngageAdvice::DISALLOW_ENGAGE);
    control_command->mutable_engage_advice()->set_reason(
        status.error_message());
    estop = true;
    estop_reason = status.error_message();
  } else {
    Status status_ts = Status::OK();
    if (!FLAGS_use_sim_time) {
      status_ts = CheckTimestamp(local_view_);
      AERROR << "Not use sim time!!!";
    }
    if (!status_ts.ok()) {
      AERROR << "Input messages timeout";
      estop = true;
      status = status_ts;
      if (local_view_->chassis().driving_mode() !=
          TL::soc::Chassis::COMPLETE_AUTO_DRIVE) {
        control_command->mutable_engage_advice()->set_advice(
            TL::common::EngageAdvice::DISALLOW_ENGAGE);
        control_command->mutable_engage_advice()->set_reason(
            status.error_message());
      }
    } else if (local_view_->trajectory().total_path_length() > 5.0 || true) {
      control_command->mutable_engage_advice()->set_advice(
          TL::common::EngageAdvice::READY_TO_ENGAGE);
    } else {
      control_command->mutable_engage_advice()->set_advice(
          TL::common::EngageAdvice::PREPARE_DISENGAGE);
    }
  }

  // check estop
  estop = control_conf_.enable_persistent_estop()
              ? estop || local_view_->trajectory().estop().is_estop()
              : local_view_->trajectory().estop().is_estop();

  if (local_view_->trajectory().estop().is_estop()) {
    estop = true;
    estop_reason = "estop from planning : ";
    estop_reason += local_view_->trajectory().estop().reason();
  }

  if (local_view_->trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    estop = true;
    estop_reason = "estop for empty planning trajectory, planning headers: " +
                   local_view_->trajectory().header().ShortDebugString();
  }

  if (FLAGS_enable_gear_drive_negative_speed_protection) {
    const double kEpsilon = 0.001;
    auto first_trajectory_point = local_view_->trajectory().trajectory_point(0);
    if (local_view_->chassis().gear_location() == Chassis::GEAR_DRIVE &&
        first_trajectory_point.v() < -1 * kEpsilon) {
      estop = true;
      estop_reason = "estop for negative speed when gear_drive";
    }
  }

  if (!estop) {
    if (local_view_->chassis().driving_mode() == Chassis::COMPLETE_MANUAL) {
      controller_agent_.Reset();
      AINFO_EVERY(100) << "Reset Controllers in Manual Mode";
    }

    auto debug = control_command->mutable_debug()->mutable_input_debug();
    debug->mutable_localization_header()->CopyFrom(
        local_view_->localization().header());
    debug->mutable_canbus_header()->CopyFrom(local_view_->chassis().header());
    debug->mutable_trajectory_header()->CopyFrom(
        local_view_->trajectory().header());

    if (local_view_->trajectory().is_replan()) {
      latest_replan_trajectory_header_ = local_view_->trajectory().header();
    }

    if (latest_replan_trajectory_header_.has_seq()) {
      debug->mutable_latest_replan_trajectory_header()->CopyFrom(
          latest_replan_trajectory_header_);
    }
    // controller agent
    Status status_compute = Status::OK();
    if (FLAGS_enable_mbd_control) {
      mbd_control_.Process(local_view_, control_command);
    } else {
      status_compute = controller_agent_.ComputeControlCommand(control_command);
    }
    if (!status_compute.ok()) {
      AERROR << "Control main function failed"
             << " with localization: "
             << local_view_->localization().ShortDebugString()
             << " with chassis: " << local_view_->chassis().ShortDebugString()
             << " with trajectory: "
             << local_view_->trajectory().ShortDebugString()
             << " with cmd: " << control_command->ShortDebugString()
             << " status:" << status_compute.error_message();
      estop = true;
      estop_reason = status_compute.error_message();
      status = status_compute;
    }
  }
  // if planning set estop, then no control process triggered
  if (estop) {
    // AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
    // set Estop command
    control_command->set_acceleration(0.0314);
    control_command->mutable_header()->mutable_status()->set_msg(estop_reason);
    control_command->set_driving_mode(TL::soc::Chassis::EMERGENCY_MODE);
  }
  // check signal
  if (local_view_->trajectory().decision().has_vehicle_signal()) {
    control_command->mutable_signal()->CopyFrom(
        local_view_->trajectory().decision().vehicle_signal());
    control_command->set_driving_mode(
        TL::soc::Chassis::COMPLETE_AUTO_DRIVE);
  }
  return status;
}

bool ControlCore::RunOnce(const std::shared_ptr<LocalView>& local_view,
                          ControlCommand* const control_command) {
  const auto start_time = Clock::NowInMicroseconds();
  local_view_ = local_view;

  // set header
  control_command->mutable_header()->mutable_sensor_stamp()->set_lidar_stamp(
      local_view_->trajectory().header().sensor_stamp().lidar_stamp());
  control_command->mutable_header()->mutable_sensor_stamp()->set_camera_stamp(
      local_view_->trajectory().header().sensor_stamp().camera_stamp());
  control_command->mutable_header()->mutable_sensor_stamp()->set_radar_stamp(
      local_view_->trajectory().header().sensor_stamp().radar_stamp());

  if (FLAGS_use_localization_timestamp_for_control) {  // use localization timestamp.
    static std::atomic<uint64_t> seq = {0};
    control_command->mutable_header()->set_frame_id("control");
    control_command->mutable_header()->set_data_stamp(
        local_view_->localization().header().data_stamp() +
        FLAGS_set_localization_control_time_gap);
    control_command->mutable_header()->set_seq(
        static_cast<unsigned int>(seq.fetch_add(1)));
  } else {
    common::util::FillHeader("control", control_command);
  }

  Status status = ProduceControlCommand(control_command);
  AERROR_IF(!status.ok()) << "Failed to produce control command:"
                          << status.error_message();
  AINFO << "control_cmd:" << control_command->DebugString();

  if (control_conf_.is_control_test_mode() &&
      control_conf_.control_test_duration() > 0 &&
      start_time - init_time_ > control_conf_.control_test_duration()) {
    AERROR << "Control finished testing. exit";
    return false;
  }

  // ADEBUG << control_command.ShortDebugString();

  const auto end_time = Clock::NowInMicroseconds();
  const double time_diff_ms = end_time - start_time;
  ADEBUG << "total control time spend: " << time_diff_ms << " ms.";

  control_command->mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  control_command->mutable_latency_stats()->set_total_time_exceeded(
      time_diff_ms > control_conf_.control_period() * 1e3);
  ADEBUG << "control cycle time is: " << time_diff_ms << " ms.";
  status.Save(control_command->mutable_header()->mutable_status());

  return true;
}

Status ControlCore::CheckInput(const std::shared_ptr<LocalView>& local_view) {
  // ADEBUG << "Received localization:"
  //       << local_view->localization().ShortDebugString();
  // ADEBUG << "Received chassis:" << local_view->chassis().ShortDebugString();

  if (!local_view->trajectory().estop().is_estop() &&
      local_view->trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    const std::string msg =
        absl::StrCat("planning has no trajectory point. planning_seq_num:",
                     local_view->trajectory().header().seq());
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, msg);
  }

  for (auto& trajectory_point :
       *local_view->mutable_trajectory()->mutable_trajectory_point()) {
    if (std::abs(trajectory_point.v()) <
            control_conf_.minimum_speed_resolution() &&
        std::abs(trajectory_point.a()) <
            control_conf_.max_acceleration_when_stopped()) {
      trajectory_point.set_v(0.0);
      trajectory_point.set_a(0.0);
    }
  }

  auto status = injector_->vehicle_state()->Update(local_view->localization(),
                                                   local_view->chassis());
  if (!status.ok()) {
    return status;
  }

  injector_->local_view()->CopyFrom(*local_view);
  return Status::OK();
}

Status ControlCore::CheckTimestamp(
    const std::shared_ptr<LocalView>& local_view) {
  if (!control_conf_.enable_input_timestamp_check() ||
      control_conf_.is_control_test_mode()) {
    ADEBUG << "Skip input timestamp check by gflags.";
    return Status::OK();
  }

  double current_timestamp = Clock::NowInSeconds();
  double localization_diff =
      current_timestamp - local_view->localization().header().data_stamp();
  if (localization_diff > (control_conf_.max_localization_miss_num() *
                           control_conf_.localization_period())) {
    AERROR << "Localization msg lost for " << SETPRECISION(6)
           << localization_diff << "s";
    // monitor_logger_buffer_.ERROR("Localization msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Localization msg timeout");
  }

  double chassis_diff =
      current_timestamp - local_view->chassis().header().data_stamp();
  if (chassis_diff >
      (control_conf_.max_chassis_miss_num() * control_conf_.chassis_period())) {
    AERROR << "Chassis msg lost for " << SETPRECISION(6) << chassis_diff << "s";
    // monitor_logger_buffer_.ERROR("Chassis msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Chassis msg timeout");
  }

  double trajectory_diff =
      current_timestamp - local_view->trajectory().header().data_stamp();
  if (trajectory_diff > (control_conf_.max_planning_miss_num() *
                         control_conf_.trajectory_period())) {
    AERROR << "Trajectory msg lost for " << SETPRECISION(6) << trajectory_diff
           << "s";
    // monitor_logger_buffer_.ERROR("Trajectory msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Trajectory msg timeout");
  }
  return Status::OK();
}
}  // namespace control
}  // namespace TL
