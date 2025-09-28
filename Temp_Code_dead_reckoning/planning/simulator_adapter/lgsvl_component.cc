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

#include "planning/simulator_adapter/lgsvl_component.h"

#include "common/adapters/adapter_gflags.h"
#include "common/util/message_util.h"

namespace TL {
namespace lgsvl {

using TL::localization::Localization;
using TL::soc::ChassisDetail;

bool LgsvlComponent::Init() {
  InitIO();
  return lgsvl_.Init().ok() && lgsvl_.Start().ok();
}

bool LgsvlComponent::Proc() {
  auto chassis_detail = std::make_shared<ChassisDetail>();
  auto localization = std::make_shared<Localization>();
  if (!lgsvl_.Process(chassis_detail, localization)) {
    return false;
  }
  chassis_detail_writer_->Write(chassis_detail);
  common::util::FillHeader(node_->Name(), localization.get());
  lgsvl_localization_writer_->Write(localization);
  return true;
}

bool LgsvlComponent::InitIO() {
  lgsvl_line_reader_ = node_->CreateReader<perception::PerceptionLanes>(
      "/apollo/Laneline",
      [this](const std::shared_ptr<perception::PerceptionLanes>& lgsvl_msg) {
        ADEBUG << "Received lgsvl data: run svl callback.";
        lgsvl_.OnLgsvlPerceptionLines(*lgsvl_msg.get());
      });
  lgsvl_gps_reader_ = node_->CreateReader<localization::Gps>(
      "/apollo/sensor/gnss/odometry",
      [this](const std::shared_ptr<localization::Gps>& lgsvl_msg) {
        ADEBUG << "Received lgsvl data: run svl gps callback.";
        lgsvl_.OnLgsvlGps(*lgsvl_msg.get());
      });
  lgsvl_imu_reader_ = node_->CreateReader<drivers::gnss::Imu>(
      "/apollo/sensor/gnss/imu",
      [this](const std::shared_ptr<drivers::gnss::Imu>& lgsvl_msg) {
        ADEBUG << "Received lgsvl data: run svl gps callback.";
        lgsvl_.OnLgsvlImu(*lgsvl_msg.get());
      });

  lgsvl_localization_writer_ =
      node_->CreateWriter<Localization>(FLAGS_localization_topic);
  chassis_detail_writer_ =
      node_->CreateWriter<ChassisDetail>(FLAGS_chassis_detail_topic);

  return true;
}

}  // namespace lgsvl
}  // namespace TL
