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

#include <string>

#include "common/status/status.h"
#include "proto/canbus/chassis_detail.pb.h"
#include "proto/drivers/imu.pb.h"
#include "proto/localization/gps.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/perception/perception_lane.pb.h"

namespace TL {
namespace lgsvl {
using TL::localization::Localization;
using TL::soc::ChassisDetail;

class Lgsvl {
 public:
  Lgsvl();

  /**
   * @brief module name
   */
  std::string Name() const { return "Lgsvl"; }

  /**
   * @brief module initialization function
   * @return initialization status
   */
  TL::common::Status Init();

  /**
   * @brief module start function
   * @return start status
   */
  TL::common::Status Start();

  /**
   * @brief module stop function
   */
  void Stop();

  /**
   * @brief destructor
   */
  virtual ~Lgsvl() = default;

  /**
   * @brief main logic of the lgsvl module, runs periodically
   * triggered by timer.
   */
  bool Process(std::shared_ptr<ChassisDetail> chassis_detail,
               std::shared_ptr<Localization> localization);

  void OnLgsvlPerceptionLines(const perception::PerceptionLanes& line_msg);
  void OnLgsvlGps(const localization::Gps& gps_msg);
  void OnLgsvlImu(const drivers::gnss::Imu& imu_msg);

 private:
  ChassisDetail sim_chassis_detail_;
  localization::Localization sim_localization_msg_;
  // pub sim localization msg when
  // lgsvl simulator running
  std::mutex lgsvl_mutex_;
};

}  // namespace lgsvl
}  // namespace TL
