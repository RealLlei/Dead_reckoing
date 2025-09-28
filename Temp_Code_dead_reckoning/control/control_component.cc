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
#include "control/control_component.h"
#include <memory>

#include "absl/strings/str_cat.h"
#include "common/adapters/adapter_gflags.h"
#include "common/file/file.h"
#include "common/file/log.h"
// #include "common/latency_recorder/latency_recorder.h"
#include "common/math/math_utils.h"
#include "common/time/clock.h"
#include "common/util/convert_tool.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "control/common/control_gflags.h"
#include "apollo_proto/control/pad_msg.pb.h"

namespace TL {
namespace control {

using TL::common::Clock;
using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::VehicleStateProvider;
using TL::localization::Localization;
using TL::planning::ADCTrajectory;
using TL::soc::Chassis;

ControlComponent::ControlComponent() {}

bool ControlComponent::Init() {
  local_view_ = std::make_shared<LocalView>();
  init_time_ = Clock::NowInMicroseconds();
  AINFO << "Control init, starting ...";
  control_core_.Init();

  apollo::cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = FLAGS_chassis_pending_queue_size;

  chassis_reader_ = node_->CreateReader<planning::ApolloChassis>(
      chassis_reader_config, nullptr);
  ACHECK(chassis_reader_ != nullptr);

  apollo::cyber::ReaderConfig planning_reader_config;
  planning_reader_config.channel_name = FLAGS_planning_trajectory_topic;
  planning_reader_config.pending_queue_size = FLAGS_planning_pending_queue_size;

  trajectory_reader_ = node_->CreateReader<planning::ApolloADCTrajectory>(
      planning_reader_config, nullptr);
  ACHECK(trajectory_reader_ != nullptr);

  apollo::cyber::ReaderConfig localization_reader_config;
  localization_reader_config.channel_name = FLAGS_localization_topic;
  localization_reader_config.pending_queue_size =
      FLAGS_localization_pending_queue_size;

  localization_reader_ =
      node_->CreateReader<planning::ApolloLocalizationEstimate>(
          localization_reader_config, nullptr);
  ACHECK(localization_reader_ != nullptr);

  apollo::cyber::ReaderConfig pad_msg_reader_config;
  pad_msg_reader_config.channel_name = FLAGS_pad_topic;
  pad_msg_reader_config.pending_queue_size = FLAGS_pad_msg_pending_queue_size;

  pad_msg_reader_ = node_->CreateReader<apollo::control::PadMessage>(
      pad_msg_reader_config, nullptr);
  ACHECK(pad_msg_reader_ != nullptr);

  control_cmd_writer_ = node_->CreateWriter<planning::ApolloControlCommand>(
      FLAGS_control_command_topic);
  ACHECK(control_cmd_writer_ != nullptr);

  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  return true;
}

void ControlComponent::OnPad(const std::shared_ptr<PadMessage>& pad) {
  std::lock_guard<std::mutex> lock(mutex_);
  pad_msg_.CopyFrom(*pad);
  ADEBUG << "Received Pad Msg:" << pad_msg_.DebugString();
  AERROR_IF(!pad_msg_.has_action()) << "pad message check failed!";
}

void ControlComponent::OnChassis(const std::shared_ptr<Chassis>& chassis) {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_chassis_.CopyFrom(*chassis);
}

void ControlComponent::OnPlanning(
    const std::shared_ptr<ADCTrajectory>& trajectory) {
  ADEBUG << "Received chassis data: run trajectory callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_trajectory_.CopyFrom(*trajectory);
}

void ControlComponent::OnLocalization(
    const std::shared_ptr<Localization>& localization) {
  ADEBUG << "Received control data: run localization message callback. "
            "localization sequence "
         << localization->header().seq();
  std::lock_guard<std::mutex> lock(mutex_);
  latest_localization_.CopyFrom(*localization);
}

bool ControlComponent::Proc() {
  chassis_reader_->Observe();
  const auto& chassis_msg = chassis_reader_->GetLatestObserved();
  if (chassis_msg == nullptr) {
    AERROR << "Chassis msg is not ready!";
    return false;
  }
  const auto TL_chassis = planning::ConvertTool::ForwardChassis(chassis_msg);
  OnChassis(TL_chassis);

  trajectory_reader_->Observe();
  const auto& trajectory_msg = trajectory_reader_->GetLatestObserved();
  if (trajectory_msg == nullptr) {
    AERROR << "planning msg is not ready!";
    return false;
  }
  const auto TL_trajectory =
      planning::ConvertTool::ForwardADCTrajectory(trajectory_msg);
  OnPlanning(TL_trajectory);

  localization_reader_->Observe();
  const auto& localization_msg = localization_reader_->GetLatestObserved();
  if (localization_msg == nullptr) {
    AERROR << "localization msg is not ready!";
    return false;
  }
  const auto TL_location =
      planning::ConvertTool::ForwardLocalization(localization_msg);
  OnLocalization(TL_location);

  pad_msg_reader_->Observe();
  const auto& pad_msg = pad_msg_reader_->GetLatestObserved();
  if (pad_msg != nullptr) {
    const auto TL_pad =
        planning::ConvertTool::ForwardControlPadMessage(pad_msg);
    OnPad(TL_pad);
  }

  {
    // TODO(SHU): to avoid redundent copy
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_->mutable_chassis()->CopyFrom(latest_chassis_);
    local_view_->mutable_trajectory()->CopyFrom(latest_trajectory_);
    local_view_->mutable_localization()->CopyFrom(latest_localization_);
    if (pad_msg != nullptr) {
      local_view_->mutable_pad_msg()->CopyFrom(pad_msg_);
    }
  }

  if (pad_msg != nullptr) {
    ADEBUG << "pad_msg: " << pad_msg_.ShortDebugString();
    if (pad_msg_.action() == DrivingAction::RESET) {
      AINFO << "Control received RESET action!";
    }
    pad_received_ = true;
  }

  ControlCommand control_command;

  control_core_.RunOnce(local_view_, &control_command);
  AERROR << "steering_target = " << control_command.steering_target() << '\t'
         << "acceleration = " << control_command.acceleration();
  const auto apollo_control_command =
      planning::ConvertTool::ReverseControlCommand(
          std::make_shared<planning::TLControlCommand>(control_command));
  control_cmd_writer_->Write(*apollo_control_command);
  return true;
}

}  // namespace control
}  // namespace TL
