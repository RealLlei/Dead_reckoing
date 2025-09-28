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

#pragma once

#include <memory>

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "planning/prediction/mobileye_adapter/mobileye.h"
#include "proto/canbus/chassis_detail.pb.h"
#include "proto/drivers/imu.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace mobileye {
// NOLINTBEGIN
class MobileyeComponent final : public apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 private:
  bool InitIO();

  Mobileye mobileye_;

  //   std::shared_ptr<apollo::Reader<localization::Localization>>
  //       localization_reader_ = nullptr;
  //   std::shared_ptr<apollo::Reader<soc::ChassisDetail>> chassis_detail_reader_ =
  //       nullptr;

  //   std::shared_ptr<apollo::Writer<perception::PerceptionObstacles>> obs_writer_ =
  //       nullptr;
};

CYBER_REGISTER_COMPONENT(MobileyeComponent)
// NOLINTEND
}  // namespace mobileye
}  // namespace TL
