/*
 * @LastEditors: Liu Bei
 */
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
#include <string>

#include "common/util/util.h"
#include "dead_reckoning/src/dead_reckoning_core.h"
#include "dead_reckoning/src/dead_reckoning_gflags.h"

#include "proto/control/control_cmd.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "cyber/time/time.h"

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace dead_reckoning {

class DeadReckoningComponent final : public cyber::Component<soc::Chassis> {
 public:
  DeadReckoningComponent();

  bool Init();

  bool Proc(const std::shared_ptr<soc::Chassis>& chassis);

  void Stop();

 private:
  void InitTimerAndIO();

  std::shared_ptr<cyber::Writer<apollo::localization::Localization>>
      localization_writer_;

  // Time interval of the timer, in milliseconds.
  DeadReckoningCore dead_reckoning_core_;
};

CYBER_REGISTER_COMPONENT(DeadReckoningComponent)
}  // namespace dead_reckoning
}  // namespace apollo
