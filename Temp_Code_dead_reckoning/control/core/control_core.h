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

#include "proto/soc/chassis.pb.h"
#include "control/proto/control_cmd.pb.h"
#include "control/proto/control_conf.pb.h"
#include "control/proto/mbd_control_conf.pb.h"
#include "control/proto/pad_msg.pb.h"
#include "control/proto/preprocessor.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/planning/planning.pb.h"

#include "common/util/macros.h"
#include "common/util/message_util.h"
#include "common/util/util.h"

#include "control/common/dependency_injector.h"
#include "control/controller/controller_agent.h"
#include "control/mbd_control/mbd_control.h"

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
class ControlCore {
  // friend class ControlTestBase;

 public:
  ControlCore();
  bool Init();

  bool RunOnce(const std::shared_ptr<LocalView>& local_view,
               ControlCommand* const control_command);

 private:
  common::Status ProduceControlCommand(ControlCommand* control_command);
  common::Status CheckInput(const std::shared_ptr<LocalView>& local_view);
  common::Status CheckTimestamp(const std::shared_ptr<LocalView>& local_view);
  common::Status CheckPad();

 private:
  double init_time_;

  // TL::drivers::Ultrasonic latest_ultrasonic_;
  PadMessage pad_msg_;

  ControllerAgent controller_agent_;

  bool pad_received_ = false;

  unsigned int status_lost_ = 0;
  unsigned int status_sanity_check_failed_ = 0;
  unsigned int total_status_lost_ = 0;
  unsigned int total_status_sanity_check_failed_ = 0;

  ControlConf control_conf_;

  std::mutex mutex_;

  std::shared_ptr<LocalView> local_view_;
  std::shared_ptr<DependencyInjector> injector_;
  MbdControl mbd_control_;
  bool chassis_received_ = false;
  TL::common::Header latest_replan_trajectory_header_;
};

}  // namespace control
}  // namespace TL
