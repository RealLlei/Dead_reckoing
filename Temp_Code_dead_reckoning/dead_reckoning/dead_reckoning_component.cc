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
#include "dead_reckoning/dead_reckoning_component.h"

#include "common/adapters/adapter_gflags.h"
#include "common/util/message_util.h"
#include "control/common/control_gflags.h"
#include "dead_reckoning/src/dead_reckoning_gflags.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

using apollo::common::Clock;
using apollo::common::ErrorCode;
using apollo::common::Header;
using apollo::common::Pose;
using apollo::common::util::FillHeader;
using apollo::control::ControlCommand;
using apollo::localization::Localization;
using apollo::perception::PerceptionObstacles;
using apollo::soc::Chassis;

namespace apollo {
namespace dead_reckoning {

DeadReckoningComponent::DeadReckoningComponent() {}

bool DeadReckoningComponent::Init() {
  // FLAGS_alsologtostderr = true;
  AINFO << "DeadReckoningComponent Init.";
  dead_reckoning_core_.Init();
  InitTimerAndIO();
  return true;
}

bool DeadReckoningComponent::Proc(
    const std::shared_ptr<soc::Chassis>& chassis) {
  auto localization = std::make_shared<Localization>();
  dead_reckoning_core_.RunOnce(chassis, localization);
  localization_writer_->Write(localization);
  return true;
}

void DeadReckoningComponent::InitTimerAndIO() {
  localization_writer_ =
      node_->CreateWriter<Localization>(FLAGS_localization_topic);
}

void DeadReckoningComponent::Stop() {
  dead_reckoning_core_.Stop();
}

}  // namespace dead_reckoning
}  // namespace apollo
