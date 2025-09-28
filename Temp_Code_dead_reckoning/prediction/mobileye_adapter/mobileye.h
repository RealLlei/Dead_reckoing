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

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include "gflags/gflags.h"

#include "common/status/status.h"
#include "proto/canbus/chassis_detail.pb.h"
#include "proto/drivers/imu.pb.h"
#include "proto/localization/gps.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/perception/perception_lane.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

// DECLARE_double(mobileye_obs_keep_time);  // ms

namespace TL {
namespace mobileye {
// NOLINTBEGIN
using TL::common::Status;
using TL::localization::Localization;
using TL::perception::PerceptionObstacles;
using TL::soc::ChassisDetail;

class Mobileye {
 public:
  Mobileye();

  /**
   * @brief module name
   */
  std::string Name() const { return "Mobileye"; }

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
  virtual ~Mobileye() = default;

  /**
   * @brief main logic of the lgsvl module, runs periodically
   * triggered by timer.
   */
  bool Process(
      std::shared_ptr<TL::perception::PerceptionObstacles> perception_obs);

  void OnChassisDetail(const ChassisDetail& chassis_detail);
  void OnLocalization(const localization::Localization& localization_estimate);

 private:
  localization::Localization localization_;
  std::mutex mobileye_mutex_;
  std::unordered_map<int, TL::perception::PerceptionObstacle> obs_map_;
  std::unordered_map<
      TL::soc::Vis_obs_msg_1_675::Vis_obs_classification_01Type,
      TL::perception::PerceptionObstacle::Type>
      obs_type_parser_;
  perception::PerceptionObstacle raw_obstacle_;
};

// NOLINTEND
}  // namespace mobileye
}  // namespace TL
