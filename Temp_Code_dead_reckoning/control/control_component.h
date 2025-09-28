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

#pragma once

#include <memory>
#include <queue>
#include <string>

#include "control/proto/preprocessor.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/soc/chassis.pb.h"

#include "common/util/convert_tool.h"
#include "common/util/macros.h"
#include "common/util/message_util.h"
#include "common/util/util.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "cyber/timer/timer.h"

#include "control/core/control_core.h"

/**
 * @namespace TL::control
 * @brief TL::control
 */
namespace TL {
namespace control {

/**
 * @class Control
 *
 * @brief control module main class, it processes localization, chassis, and
 * pad data to compute throttle, brake and steer values.
 */
class ControlComponent final : public apollo::cyber::TimerComponent {
  // friend class ControlTestBase;

 public:
  ControlComponent();
  bool Init() override;

  bool Proc() override;

 private:
  // Upon receiving pad message
  void OnPad(const std::shared_ptr<PadMessage>& pad);

  void OnChassis(const std::shared_ptr<TL::soc::Chassis>& chassis);

  void OnPlanning(
      const std::shared_ptr<TL::planning::ADCTrajectory>& trajectory);

  void OnLocalization(
      const std::shared_ptr<TL::localization::Localization>& localization);

 private:
  double init_time_;

  // TL::drivers::Ultrasonic latest_ultrasonic_;
  localization::Localization latest_localization_;
  soc::Chassis latest_chassis_;
  planning::ADCTrajectory latest_trajectory_;
  PadMessage pad_msg_;

  bool pad_received_ = false;

  unsigned int status_lost_ = 0;
  unsigned int status_sanity_check_failed_ = 0;
  unsigned int total_status_lost_ = 0;
  unsigned int total_status_sanity_check_failed_ = 0;

  std::mutex mutex_;

  std::shared_ptr<apollo::cyber::Reader<planning::ApolloChassis>>
      chassis_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::control::PadMessage>>
      pad_msg_reader_;
  std::shared_ptr<apollo::cyber::Reader<planning::ApolloLocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<apollo::cyber::Reader<planning::ApolloADCTrajectory>>
      trajectory_reader_;

  std::shared_ptr<apollo::cyber::Writer<planning::ApolloControlCommand>>
      control_cmd_writer_;

  std::shared_ptr<LocalView> local_view_;

  bool chassis_received_ = false;

  ControlCore control_core_;
};

CYBER_REGISTER_COMPONENT(ControlComponent)

}  // namespace control
}  // namespace TL
