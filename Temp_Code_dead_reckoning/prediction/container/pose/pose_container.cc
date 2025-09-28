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

#include "planning/prediction/container/pose/pose_container.h"
#include <cmath>

#include "common/configs/vehicle_config_helper.h"
#include "common/file/log.h"
#include "common/math/quaternion.h"
#include "planning/prediction/common/prediction_gflags.h"

namespace TL {
namespace prediction {

using TL::localization::Localization;
using TL::perception::PerceptionObstacle;
using Point = TL::common::Point3D;

void PoseContainer::Insert(const ::google::protobuf::Message& message) {
  Update(dynamic_cast<const Localization&>(message));
}

void PoseContainer::Update(const localization::Localization& localization) {
  if (!localization.has_header() || !localization.header().has_data_stamp() ||
      !localization.has_pose() || !localization.pose().has_position() ||
      !localization.pose().has_linear_velocity()) {
    AERROR << "Localiazation message has no timestamp, pose, position or "
              "linear velocity ["
           << localization.ShortDebugString() << "].";
    return;
  }

  if (obstacle_ptr_ == nullptr) {
    obstacle_ptr_ = std::make_unique<PerceptionObstacle>();
  }
  obstacle_ptr_->Clear();

  obstacle_ptr_->set_id(FLAGS_ego_vehicle_id);
  obstacle_ptr_->mutable_position()->set_x(localization.pose().position().x());
  obstacle_ptr_->mutable_position()->set_y(localization.pose().position().y());
  obstacle_ptr_->mutable_position()->set_z(localization.pose().position().z());

  double theta = localization.pose().heading();
  if (localization.pose().has_quaternion() &&
      localization.pose().quaternion().has_x() &&
      localization.pose().quaternion().has_y() &&
      localization.pose().quaternion().has_z() &&
      localization.pose().quaternion().has_w()) {
    double qw = localization.pose().quaternion().w();
    double qx = localization.pose().quaternion().x();
    double qy = localization.pose().quaternion().y();
    double qz = localization.pose().quaternion().z();
    theta = common::math::QuaternionToHeading(qw, qx, qy, qz);
  }
  obstacle_ptr_->set_theta(theta);

  obstacle_ptr_->mutable_velocity()->set_x(
      localization.pose().linear_velocity().x());
  obstacle_ptr_->mutable_velocity()->set_y(
      localization.pose().linear_velocity().y());
  obstacle_ptr_->mutable_velocity()->set_z(
      localization.pose().linear_velocity().z());

  obstacle_ptr_->mutable_acceleration()->set_x(0.0);
  obstacle_ptr_->mutable_acceleration()->set_y(0.0);
  obstacle_ptr_->mutable_acceleration()->set_z(0.0);

  obstacle_ptr_->set_type(type_);
  obstacle_ptr_->set_timestamp(localization.header().data_stamp());
  const auto& vehicle_config = common::VehicleConfigHelper::GetConfig();
  obstacle_ptr_->set_width(vehicle_config.vehicle_param().width());
  obstacle_ptr_->set_length(vehicle_config.vehicle_param().length());

  ADEBUG << "ADC obstacle [" << obstacle_ptr_->ShortDebugString() << "].";
}

double PoseContainer::GetTimestamp() {
  if (obstacle_ptr_ != nullptr) {
    return obstacle_ptr_->timestamp();
  }
  return 0.0;
}

localization::Localization PoseContainer::PurePerceptionTransform(
    const localization::Localization& localization) {
  localization::Localization localization_new;
  localization_new.mutable_header()->CopyFrom(localization.header());
  localization_new.mutable_pose()->mutable_position()->set_x(0.0);
  localization_new.mutable_pose()->mutable_position()->set_y(0.0);
  localization_new.mutable_pose()->mutable_position()->set_z(0.0);
  localization_new.mutable_pose()->set_heading(0.0);
  double speed = 0.0;
  if (localization.pose().has_linear_velocity() &&
      localization.pose().linear_velocity().has_x() &&
      localization.pose().linear_velocity().has_y()) {
    speed = std::hypot(localization.pose().linear_velocity().x(),
                       localization.pose().linear_velocity().y());
  }

  localization_new.mutable_pose()->mutable_linear_velocity()->set_x(speed);
  localization_new.mutable_pose()->mutable_linear_velocity()->set_y(0.0);
  localization_new.mutable_pose()->mutable_linear_velocity()->set_z(0.0);

  return localization_new;
}

const PerceptionObstacle* PoseContainer::ToPerceptionObstacle() {
  return obstacle_ptr_.get();
}

}  // namespace prediction
}  // namespace TL
