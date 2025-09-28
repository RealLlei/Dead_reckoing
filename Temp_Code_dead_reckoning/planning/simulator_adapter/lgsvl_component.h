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

#include "planning/simulator_adapter/lgsvl.h"
#include "proto/drivers/imu.pb.h"

namespace TL {
namespace lgsvl {

class LgsvlComponent final : public ::TL::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 private:
  bool InitIO();

  Lgsvl lgsvl_;

  // lgsvl reader & writer
  std::shared_ptr<cyber::Reader<perception::PerceptionLanes>>
      lgsvl_line_reader_ = nullptr;
  std::shared_ptr<cyber::Reader<localization::Gps>> lgsvl_gps_reader_ = nullptr;
  std::shared_ptr<::TL::cyber::Writer<localization::Localization>>
      lgsvl_localization_writer_ = nullptr;
  std::shared_ptr<cyber::Reader<drivers::gnss::Imu>> lgsvl_imu_reader_ =
      nullptr;
  std::shared_ptr<cyber::Writer<soc::ChassisDetail>> chassis_detail_writer_ =
      nullptr;
};

CYBER_REGISTER_COMPONENT(LgsvlComponent)

}  // namespace lgsvl
}  // namespace TL
