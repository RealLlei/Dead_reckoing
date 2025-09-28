/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  control检测demo头文件
 */

#include "control/control.h"

#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "common/adapters/adapter_gflags.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/file/file.h"
#include "common/file/log.h"
#include "common/pb2struct/pb2struct.h"
#include "common/struct2pb/struct2pb.h"
#include "common/time/clock.h"
#include "common/util/message_util.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "control/common/control_gflags.h"

using TL::soc::Chassis;
using TL::common::Clock;
using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::VehicleStateProvider;
using TL::localization::Localization;

void Control::Process() {
  // receive
  GetChassis();
  if (FLAGS_enable_teleop) {
    ProcessTeleop();
    return;
  }
  GetLocation();
  // MockChassis();
  GetEgoTrajectory();
  // process adcontrol;
  const double control_start_timestamp = TL::common::Clock::NowInSeconds();
  ProcessAdControl();
  const double control_end_timestamp = TL::common::Clock::NowInSeconds();
  const double time_diff_ms =
      (control_end_timestamp - control_start_timestamp) * 1000;
  AINFO << " *********** control use time : " << time_diff_ms;
  return;
}
void Control::ProcessTeleop() {
  /// pub
  TL::control::ControlCommand control_command_teleop;
  teleop_.SetChassis(latest_chassis_);
  if (FLAGS_enable_teleop_pid_calculate) {
    teleop_.PIDCalculateFunc();
  }
  control_command_teleop.CopyFrom(teleop_.GetControlCmd());
  PubData(control_command_teleop);
}

void Control::GetChassis() {
  Chassis chassis;
  Adsfi::HafChassisInfo hafChassis;
  std::lock_guard<std::shared_timed_mutex> readerLock(
      *mutexPtr_Chassis_DDS_8100);
  { hafChassis = *dataPtr_Chassis_DDS_8100; }
  Struct2ChassisPb(hafChassis, &chassis);
  latest_chassis_.CopyFrom(chassis);
}
void Control::GetEgoTrajectory() {
  ADCTrajectory adc_trajectory;
  Adsfi::HafEgoTrajectory hafTrajectory;
  std::lock_guard<std::shared_timed_mutex> readerLock(
      *mutexPtr_EgoTrajectory_DDS_9900);
  { hafTrajectory = *dataPtr_EgoTrajectory_DDS_9900; }
  Struct2TrajectoryPb(hafTrajectory, &adc_trajectory);
  AINFO << "traj seq : " << adc_trajectory.header().seq();
  AINFO << "traj time : " << adc_trajectory.header().DebugString();
  AINFO << "trajectoryLength : " << adc_trajectory.total_path_length();
  AINFO << "trajectoryPeriod : " << adc_trajectory.total_path_time();
  AINFO << "isReplanning : " << adc_trajectory.is_replan();
  latest_trajectory_.CopyFrom(adc_trajectory);
  // egoTrajectoryDataPtr_->isValid = false;
}
void Control::GetLocation() {
  Localization localization;
  Adsfi::HafLocation hafLocation;
  if (FLAGS_is_nnp_mode) {
    std::lock_guard<std::shared_timed_mutex> readerLock(
        *mutexPtr_Location_DDS_8000);
    { hafLocation = *dataPtr_Location_DDS_8000; }
  } else {
    // avp mode
    AINFO << "AVP_Location";
    std::lock_guard<std::shared_timed_mutex> readerLock(
        *mutexPtr_Location_DDS_8001);
    { hafLocation = *dataPtr_Location_DDS_8001; }
  }

  Struct2LocalizationPb(hafLocation, &localization);
  const double curr_time = TL::common::Clock::NowInSeconds();
  localization.mutable_header()->set_data_stamp(curr_time);
  latest_localization_.CopyFrom(localization);
}
void Control::Destroy(Adsfi::HafContext* context) {
  if (FLAGS_enable_teleop) {
    teleop_.Stop();
  }
  (void)Adsfi::HafRelease(*context);
  if (FLAGS_enable_control_zmq) {
    zmq_sender_->Stop();
  }
}
void Control::InitAdControl() {
  if (FLAGS_enable_teleop) {
    if (teleop_.Start() != 0) {
      AERROR << "Teleop start failed.";
      return;
    }
    teleop_.PrintKeycode();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    AINFO << "Teleop exit done.";
    return;
  }
  if (FLAGS_enable_control_zmq) {
    zmq_sender_ = std::make_unique<TL::common::ZMQSender>();
    zmq_sender_->Init(std::to_string(FLAGS_zmq_data_port + 1));
  }
  local_view_ = std::make_shared<TL::control::LocalView>();
  control_core_.Init();
  init_time_ = Clock::NowInMicroseconds();
  AINFO << "Control init, starting ...";
  ACHECK(
      TL::common::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_))
      << "Unable to load control conf file: " + FLAGS_control_conf_file;
  AINFO << "Conf file: " << FLAGS_control_conf_file << " is loaded.";

  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // should init_vehicle first, let car enter work status, then use status msg
  // trigger control

  AINFO << "Control default driving action is "
        << DrivingAction_Name(control_conf_.action());
  pad_msg_.set_action(control_conf_.action());
}
bool Control::ProcessAdControl() {
  {
    // TODO(SHU): to avoid redundent copy
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_->mutable_chassis()->CopyFrom(latest_chassis_);
    local_view_->mutable_trajectory()->CopyFrom(latest_trajectory_);
    local_view_->mutable_localization()->CopyFrom(latest_localization_);
    // if (pad_msg != nullptr) {
    // local_view_->mutable_pad_msg()->CopyFrom(pad_msg_);
    // }
  }
  ControlCommand control_command;
  control_core_.RunOnce(local_view_, &control_command);
  /// pub
  PubData(control_command);
  return true;
}

void Control::PubData(const ControlCommand& control_command) {
  Adsfi::ControlDebug HafControlDebug;
  std::lock_guard<std::shared_timed_mutex> writerLock(
      *mutexPtr_ControlCommand_DDS_8805);
  { Pb2StructControlCmd(control_command, dataPtr_ControlCommand_DDS_8805); }

  if (FLAGS_enable_viz) {
    SerializeProtoToString(control_command);
  } else {
    // nothing
  }
}
void Control::SerializeProtoToString(const ControlCommand& control_command) {
  std::string one;
  {
    std::lock_guard<std::shared_timed_mutex> DebugwriterLock(
        *mutexPtr_ControlDebug_DDS_9200);
    CleanControlStructDebugString(dataPtr_ControlDebug_DDS_9200);
    dataPtr_ControlDebug_DDS_9200->msg_1 = one;
  }
  control_command.SerializeToString(&one);
  static int32_t count = 0;
  if (count % 10 == 0) {
    std::vector<std::string> msg;
    msg.push_back(std::move(one));
    zmq_sender_->Process(msg);
  }
  count++;
}
