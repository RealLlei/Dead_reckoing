
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

#include "common/vehicle_state/vehicle_state_provider.h"
#include <unistd.h>

#include <cmath>

#include "absl/strings/str_cat.h"
#include "common/configs/config_gflags.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/math/linear_interpolation.h"
#include "common/math/quaternion.h"
#include "common/time/clock.h"
#include "common/util/message_util.h"
#include "common/util/util.h"

namespace TL {
namespace common {
namespace {
constexpr double kEpsilon = 1.0e-6;
}

void VehicleStateProvider::SetVehicleState(const VehicleState& vehicle_state) {
  vehicle_state_.Clear();
  vehicle_state_ = vehicle_state;
}

void VehicleStateProvider::InitFilter() {}

Status VehicleStateProvider::Update(  // NOLINT
    const localization::Localization& localization,
    const soc::Chassis& chassis) {
  vehicle_state_.Clear();
  if (!localization.has_header() || !chassis.has_header()) {
    has_header_ = false;
    AERROR << "no header! localization must have header!";
    return Status(ErrorCode::CORE_ONLANE_VEHICLESTATECHECK_ERROR,
                  "no header! localization must have header!");
  }
  has_header_ = true;
  if (!is_initialized_) {
    previous_localization_header_.set_seq(-1);
    InitFilter();
    is_initialized_ = true;
  }
  if (localization.header().frame_id() == "vrf_localization") {
    vehicle_state_.set_angular_velocity(0.0);
    vehicle_state_.set_linear_acceleration(0.0);
    vehicle_state_.set_x(0.0);
    vehicle_state_.set_y(0.0);
    vehicle_state_.set_z(0.0);
    vehicle_state_.set_heading(0.0);
    vehicle_state_.set_yaw(0.0);
  } else {

    if (localization.header().seq() == previous_localization_header_.seq()) {
      return Status::OK();
    }
    original_localization_ = localization;
    if (!ConstructExceptLinearVelocity(localization)) {
      std::string msg = absl::StrCat(
          "Fail to update because ConstructExceptLinearVelocity error.",
          "localization:\n", localization.DebugString());
      return Status(ErrorCode::CORE_ONLANE_VEHICLESTATECHECK_ERROR, msg);
    }
  }
  vehicle_state_.mutable_pose()->CopyFrom(localization.pose());
  vehicle_state_.mutable_chassis()->CopyFrom(chassis);
  vehicle_state_.mutable_localization_header()->CopyFrom(localization.header());
  if (localization.has_measurement_time()) {
    vehicle_state_.set_timestamp(localization.measurement_time());
  } else if (localization.header().has_data_stamp()) {
    vehicle_state_.set_timestamp(localization.header().data_stamp());
  } else if (chassis.has_header() && chassis.header().has_data_stamp()) {
    AERROR << "Unable to use location timestamp for vehicle state. Use chassis "
              "time instead.";
    vehicle_state_.set_timestamp(chassis.header().data_stamp());
  } else {
    AERROR << "no header in localization or chassis";
    return Status(ErrorCode::CORE_ONLANE_VEHICLESTATECHECK_ERROR,
                  "no header in localization or chassis");
  }

  if (chassis.has_gear_location()) {
    vehicle_state_.set_gear(chassis.gear_location());
  } else {
    vehicle_state_.set_gear(soc::Chassis::GEAR_NONE);
  }
  if (localization.pose().has_linear_velocity_vrf()) {
    vehicle_state_.set_linear_velocity(
        localization.pose().linear_velocity_vrf().y());
  } else if (chassis.has_speed_mps()) {
    vehicle_state_.set_linear_velocity(chassis.speed_mps());
    if (!FLAGS_reverse_heading_vehicle_state &&
        vehicle_state_.gear() == soc::Chassis::GEAR_REVERSE) {
      vehicle_state_.set_linear_velocity(
          -fabs(vehicle_state_.linear_velocity()));
    }
  }

  if (!vehicle_state_.has_angular_velocity()) {
    if (chassis.has_yaw_rate()) {
      vehicle_state_.set_angular_velocity(chassis.yaw_rate());
    } else {
      vehicle_state_.set_angular_velocity(0.0);
    }
  }

  if (std::fabs(vehicle_state_.linear_velocity()) < 0.01) {
    vehicle_state_.set_angular_velocity(0.0);
  }
  
  if (!vehicle_state_.has_linear_acceleration()) {
    if (chassis.has_imu_acc()) {
      vehicle_state_.set_linear_acceleration(chassis.imu_acc().y());
    } else {
      vehicle_state_.set_linear_acceleration(0.0);
    }
  }

  if (chassis.has_steering_percentage()) {
    vehicle_state_.set_steering_percentage(chassis.steering_percentage());
  }

  static constexpr double kLowSpeed = 5.0;
  const double steer_angle = chassis.has_steering_angle()
                                 ? chassis.steering_angle() * M_PI / 180.0
                                 : 0.0;
  const double beta_angle = EstimateBetaAngle(steer_angle, vehicle_param_);
  const double kappa = std::tan(beta_angle) / vehicle_param_.wheel_base();
  vehicle_state_.set_kappa(kappa);

  vehicle_state_.set_driving_mode(chassis.driving_mode());

  if (chassis.has_signal()) {
    vehicle_state_.mutable_sigal()->CopyFrom(chassis.signal());
  }

  if (chassis.has_steering_angle() &&
      vehicle_param_.has_steer_ratio_segment()) {
    const double linear_steer_ratio = common::math::InterpolationOne(
        chassis.steering_angle(),
        vehicle_param_.steer_ratio_segment().steer_segment(),
        vehicle_param_.steer_ratio_segment().steer_ratio_segment());
    vehicle_state_.set_linear_steer_ratio(linear_steer_ratio);
  } else {
    vehicle_state_.set_linear_steer_ratio(vehicle_param_.steer_ratio());
  }

  UpdateYawRate(localization, chassis);
  common::util::FillHeader("VehicleStateProvier", &vehicle_state_);
  return Status::OK();
}

bool VehicleStateProvider::UpdateYawRate(
    const localization::Localization& localization,  // NOLINT
    const soc::Chassis& chassis) {
  double yaw_rate_steer = 0.0;
  if (chassis.has_steering_percentage()) {
    double steering = chassis.steering_percentage();
    double delta = steering / 100 * vehicle_param_.max_steer_angle() /
                   vehicle_param_.steer_ratio();
    double l_r = vehicle_param_.wheel_base() * 0.5;
    double l_f = vehicle_param_.wheel_base() * 0.5;
    double beta = atan((l_r / (l_r + l_f)) * tan(delta));
    yaw_rate_steer = vehicle_state_.linear_velocity() / l_f * sin(beta);
  } else {
    AERROR << "there is not chassis' output of steering, and using 0.0 by "
              "default!";
  }

  double yaw_rate_esc_sensor = 0.0;
  double yaw_rate_imu_sensor = 0.0;
  if (chassis.has_yaw_rate()) {
    yaw_rate_esc_sensor = chassis.yaw_rate();
  } else {
    yaw_rate_esc_sensor = yaw_rate_steer;
    AERROR << "there is not chassis' output of esc's yaw rate, and using steer "
              "by default!";
  }
  if (vehicle_state_.pose().angular_velocity_vrf().has_z()) {
    yaw_rate_imu_sensor = vehicle_state_.pose().angular_velocity_vrf().z();
  } else {
    yaw_rate_imu_sensor = yaw_rate_steer;
    AERROR << "there is not localization' output of angular_velocity, and "
              "using steer by default!";
  }

  ADEBUG << "yaw_rate data, steer, " << yaw_rate_steer << " , esc, "
         << yaw_rate_esc_sensor << " , imu, " << yaw_rate_imu_sensor;
  return true;
}

bool VehicleStateProvider::ConstructExceptLinearVelocity(
    const localization::Localization& localization) {
  if (!localization.has_pose()) {
    AERROR << "Invalid localization input.";
    return false;
  }
  TL::common::Pose pose;
  pose.CopyFrom(localization.pose());

  if (pose.has_position()) {
    vehicle_state_.set_x(pose.position().x());
    vehicle_state_.set_y(pose.position().y());
    vehicle_state_.set_z(pose.position().z());
  }

  if (!pose.has_heading() && !pose.has_quaternion()) {
    AERROR << "localization has not heading nor orientaton.";
    return false;
  }
  if (pose.has_heading()) {
    vehicle_state_.set_heading(pose.heading());
  } else {
    const auto& orientation = pose.quaternion();
    double heading = math::QuaternionToHeading(
        orientation.w(), orientation.x(), orientation.y(), orientation.z());
    pose.set_heading(heading);
  }

  if (!pose.has_quaternion()) {
    auto orientation = math::HeadingToQuaternion(pose.heading());
    pose.mutable_quaternion()->set_w(orientation.w());
    pose.mutable_quaternion()->set_x(orientation.x());
    pose.mutable_quaternion()->set_y(orientation.y());
    pose.mutable_quaternion()->set_z(orientation.z());
  }

  if (!pose.has_linear_velocity_vrf()) {
    common::util::TransformToVRF(pose.linear_velocity(), pose.quaternion(),
                                 pose.mutable_linear_velocity_vrf());
  }
  if (!pose.has_angular_velocity_vrf()) {
    common::util::TransformToVRF(pose.angular_velocity(), pose.quaternion(),
                                 pose.mutable_angular_velocity_vrf());
  }
  vehicle_state_.set_angular_velocity(pose.angular_velocity_vrf().z());

  if (!pose.has_linear_acceleration_vrf()) {
    common::util::TransformToVRF(pose.linear_acceleration(), pose.quaternion(),
                                 pose.mutable_linear_acceleration_vrf());
  }
  vehicle_state_.set_linear_acceleration(pose.linear_acceleration_vrf().y());

  if (pose.has_euler_angles()) {
    vehicle_state_.set_roll(pose.euler_angles().x());
    vehicle_state_.set_pitch(pose.euler_angles().y());
    vehicle_state_.set_yaw(pose.euler_angles().z());
  } else {
    math::EulerAnglesZXYd euler_angle(
        pose.quaternion().w(), pose.quaternion().x(), pose.quaternion().y(),
        pose.quaternion().z());
    vehicle_state_.set_roll(euler_angle.roll());
    vehicle_state_.set_pitch(euler_angle.pitch());
    vehicle_state_.set_yaw(euler_angle.yaw());
  }

  return true;
}

double VehicleStateProvider::x() const {
  return vehicle_state_.x();
}

double VehicleStateProvider::y() const {
  return vehicle_state_.y();
}

double VehicleStateProvider::z() const {
  return vehicle_state_.z();
}

double VehicleStateProvider::roll() const {
  return vehicle_state_.roll();
}

double VehicleStateProvider::pitch() const {
  return vehicle_state_.pitch();
}

double VehicleStateProvider::yaw() const {
  return vehicle_state_.yaw();
}

double VehicleStateProvider::heading() const {
  return vehicle_state_.heading();
}

double VehicleStateProvider::kappa() const {
  return vehicle_state_.kappa();
}

double VehicleStateProvider::linear_velocity() const {
  return vehicle_state_.linear_velocity();
}

double VehicleStateProvider::angular_velocity() const {
  return vehicle_state_.angular_velocity();
}

double VehicleStateProvider::linear_acceleration() const {
  return vehicle_state_.linear_acceleration();
}

double VehicleStateProvider::gear() const {
  return vehicle_state_.gear();
}

double VehicleStateProvider::steering_percentage() const {
  return vehicle_state_.steering_percentage();
}

double VehicleStateProvider::timestamp() const {
  return vehicle_state_.timestamp();
}

const TL::common::Pose& VehicleStateProvider::pose() const {
  return vehicle_state_.pose();
}

const TL::common::Pose& VehicleStateProvider::original_pose() const {
  return original_localization_.pose();
}

void VehicleStateProvider::set_linear_velocity(const double linear_velocity) {
  vehicle_state_.set_linear_velocity(linear_velocity);
}

const VehicleState& VehicleStateProvider::vehicle_state() const {
  return vehicle_state_;
}

VehicleState* VehicleStateProvider::mutable_vehicle_state() {
  return &vehicle_state_;
}

bool VehicleStateProvider::has_header() const {
  return has_header_;
}

bool VehicleStateProvider::EstimatePassedPosition(
    const double t, double* x, double* y, double* heading,  // NOLINT
    VehicleState* const vehicle_state) {
  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  double v = vehicle_state->linear_velocity();
  // Predict distance travel vector
  if (std::fabs(vehicle_state->angular_velocity()) < 0.0001) {
    vec_distance[0] = 0.0;
    vec_distance[1] = -v * t;
  } else {
    vec_distance[0] = v / vehicle_state->angular_velocity() *
                      (1.0 - std::cos(vehicle_state->angular_velocity() * t));
    vec_distance[1] = -std::sin(vehicle_state->angular_velocity() * t) * v /
                      vehicle_state->angular_velocity();
  }
  const auto& orientation = vehicle_state->pose().quaternion();
  Eigen::Quaternion<double> quaternion(orientation.w(), orientation.x(),
                                       orientation.y(), orientation.z());
  Eigen::Vector3d pos_vec(vehicle_state->x(), vehicle_state->y(),
                          vehicle_state->z());
  const Eigen::Vector3d future_pos_3d =
      quaternion.toRotationMatrix() * vec_distance + pos_vec;
  vehicle_state->set_x(future_pos_3d.x());
  vehicle_state->set_x(future_pos_3d.y());
  vehicle_state->set_x(-vehicle_state->angular_velocity() * t +
                       vehicle_state->heading());
  return true;
}

bool VehicleStateProvider::EstimateFuturePosition(
    const double t, VehicleState* const vehicle_state) {
  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  double v = vehicle_state->linear_velocity();
  // Predict distance travel vector
  if (std::fabs(vehicle_state->angular_velocity()) < 0.0001) {
    vec_distance[0] = 0.0;
    vec_distance[1] = v * t;
  } else {
    vec_distance[0] = -v / vehicle_state->angular_velocity() *
                      (1.0 - std::cos(vehicle_state->angular_velocity() * t));
    vec_distance[1] = std::sin(vehicle_state->angular_velocity() * t) * v /
                      vehicle_state->angular_velocity();
  }
  vehicle_state->set_timestamp(vehicle_state->timestamp() + t);

  // If we have rotation information, take it into consideration.
  if (vehicle_state->pose().has_quaternion() ||
      vehicle_state->is_vehicle_reference_frame()) {
    const auto& orientation = vehicle_state->pose().quaternion();
    Eigen::Quaternion<double> quaternion(orientation.w(), orientation.x(),
                                         orientation.y(), orientation.z());
    if (vehicle_state->is_vehicle_reference_frame()) {
      quaternion = common::math::HeadingToQuaternion(0.0);
    }
    Eigen::Vector3d pos_vec(vehicle_state->x(), vehicle_state->y(),
                            vehicle_state->z());
    const Eigen::Vector3d future_pos_3d =
        quaternion.toRotationMatrix() * vec_distance + pos_vec;
    vehicle_state->set_x(future_pos_3d[0]);
    vehicle_state->set_y(future_pos_3d[1]);
    return true;
  }

  // If no valid rotation information provided from localization,
  // return the estimated future position without rotation.
  vehicle_state->set_x(vec_distance[0] + vehicle_state->x());
  vehicle_state->set_y(vec_distance[1] + vehicle_state->y());
  return true;
}

bool VehicleStateProvider::EstimateFuturePosition(
    const double t, const VehicleState* vehicle_state, double* const x,
    double* const y) {
  if (vehicle_state == nullptr || x == nullptr || y == nullptr) {
    return false;
  }
  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  const double v = vehicle_state->linear_velocity();
  // Predict distance travel vector
  if (std::fabs(vehicle_state->angular_velocity()) < 0.0001) {
    vec_distance[0] = 0.0;
    vec_distance[1] = v * t;
  } else {
    vec_distance[0] = -v / vehicle_state->angular_velocity() *
                      (1.0 - std::cos(vehicle_state->angular_velocity() * t));
    vec_distance[1] = std::sin(vehicle_state->angular_velocity() * t) * v /
                      vehicle_state->angular_velocity();
  }

  // If we have rotation information, take it into consideration.
  if (vehicle_state->pose().has_quaternion() ||
      vehicle_state->is_vehicle_reference_frame()) {
    const auto& orientation = vehicle_state->pose().quaternion();
    Eigen::Quaternion<double> quaternion(orientation.w(), orientation.x(),
                                         orientation.y(), orientation.z());
    if (vehicle_state->is_vehicle_reference_frame()) {
      quaternion = common::math::HeadingToQuaternion(0.0);
    }
    Eigen::Vector3d pos_vec(vehicle_state->x(), vehicle_state->y(),
                            vehicle_state->z());
    const Eigen::Vector3d future_pos_3d =
        quaternion.toRotationMatrix() * vec_distance + pos_vec;
    *x = future_pos_3d[0];
    *y = future_pos_3d[1];
    return true;
  }

  // If no valid rotation information provided from localization,
  // return the estimated future position without rotation.
  *x = vec_distance[0] + vehicle_state->x();
  *y = vec_distance[1] + vehicle_state->y();
  return true;
}

bool VehicleStateProvider::EstimateFutureVehiclePosition(
    const double t, const VehicleState* vehicle_state, double* const x,
    double* const y) {
  if (vehicle_state == nullptr || x == nullptr || y == nullptr) {
    return false;
  }
  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  const double v = vehicle_state->linear_velocity();
  if (std::fabs(vehicle_state->angular_velocity()) < 0.0001) {
    vec_distance[0] = 0.0;
    vec_distance[1] = v * t;
  } else {
    vec_distance[1] = v * t;
    vec_distance[0] =
        -vec_distance[1] * vec_distance[1] *
        (vehicle_state->angular_velocity() / math::Clamp(v, 0.001, 1000.0)) *
        0.5;
  }

  // If we have rotation information, take it into consideration.
  if (vehicle_state->pose().has_quaternion() ||
      vehicle_state->is_vehicle_reference_frame()) {
    const auto& orientation = vehicle_state->pose().quaternion();
    Eigen::Quaternion<double> quaternion(orientation.w(), orientation.x(),
                                         orientation.y(), orientation.z());
    if (vehicle_state->is_vehicle_reference_frame()) {
      quaternion = common::math::HeadingToQuaternion(0.0);
    }
    Eigen::Vector3d pos_vec(vehicle_state->x(), vehicle_state->y(),
                            vehicle_state->z());
    const Eigen::Vector3d future_pos_3d =
        quaternion.toRotationMatrix() * vec_distance + pos_vec;
    *x = future_pos_3d[0];
    *y = future_pos_3d[1];    
    return true;
  }  
}

bool VehicleStateProvider::EstimateFutureVehiclePosition(
    const double kappa, const double t, const VehicleState* vehicle_state,
    double* const estimate_x, double* const estimate_y) {
  if (vehicle_state == nullptr || estimate_x == nullptr ||
      estimate_y == nullptr) {
    return false;
  }
  common::Path init_points;
  const double max_arc_length =
      std::max(30.0, vehicle_state->linear_velocity() * 8.0);
  const double central_length = 150;
  const double back_length = 10;
  const double delta_distance = 1.0;
  double limit_kappa = kappa;
  if (std::abs(kappa) > 0.0001) {
    limit_kappa = kappa;
  } else {
    if (kappa > 0.0) {
      limit_kappa = 0.0001;
    } else {
      limit_kappa = -0.0001;
    }
  }
  const double radius = std::max(std::abs(1.0 / limit_kappa), 6.0);
  const double angle = vehicle_state->chassis().steering_angle();
  const double delta_angle = delta_distance / radius;
  const double pre_s = vehicle_state->linear_velocity() * t;
  const double pre_phi = vehicle_state->angular_velocity() * t;
  double s = -1.0;
  // 自车轨迹曲率较大,半径大于 3000 ;认为是直行
  int cut_idx = 0;
  if (radius > 3000.0) {
    int max_index = floor(central_length / delta_distance);
    for (int i = 0; i < max_index; i++) {
      auto* point = init_points.add_path_point();
      auto x = i * delta_distance;
      s += delta_distance;
      point->set_x(x);
      point->set_y(0.0);
      point->set_theta(0.0);
      point->set_s(s);
      point->set_kappa(0);
    }
  } else {
    // int back_max_index = floor(back_length / delta_distance);
    // for (int i = 0; i < back_max_index; i++) {
    //   auto* point = init_points.add_path_point();
    //   s += delta_distance;
    //   auto x = i * delta_distance - back_length;
    //   point->set_x(x);
    //   point->set_y(0.0);
    //   point->set_theta(0.0);
    //   point->set_s(s);
    //   point->set_kappa(1 / radius);
    // }
    // 自车前方轨迹
    double x = 0.0;
    double y = 0.0;
    double beta = 0.0;
    int phi_num = M_PI_2 / delta_angle;
    for (int i = 0; i < phi_num && s <= max_arc_length; i++) {
      double phi = i * delta_angle;
      x = radius * std::sin(phi);
      y = copysign(radius * (1 - std::cos(phi)), angle);
      beta = copysign(phi, angle);
      // cneter lane point
      if (init_points.path_point_size() < 1) {
        s += std::hypot(x - 0, y - 0);
      } else {
        auto pre_point =
            init_points.path_point().at(init_points.path_point_size() - 1);
        s += std::hypot(x - pre_point.x(), y - pre_point.y());
      }
      auto* point = init_points.add_path_point();
      // cneter lane point
      point->set_x(x);
      point->set_y(y);
      point->set_theta(beta);
      point->set_kappa(1 / radius);
      point->set_s(s);
    }
    // 裁剪
    cut_idx = init_points.path_point_size();
    if (s < central_length) {
      // cneter lane point
      auto pre_point =
          init_points.path_point().at(init_points.path_point_size() - 2);
      auto end_point =
          init_points.path_point().at(init_points.path_point_size() - 1);
      double dx = end_point.x() - pre_point.x();
      double dy = end_point.y() - pre_point.y();
      double end_phi = end_point.theta();
      while (s < central_length) {
        // cneter lane point
        auto last_point =
            init_points.path_point().at(init_points.path_point_size() - 1);
        auto* point = init_points.add_path_point();
        // cneter lane point
        point->set_x(last_point.x() + dx);
        point->set_y(last_point.y() + dy);
        point->set_theta(end_phi);
        s += std::hypot(dx, dy);
        point->set_s(s);
        point->set_kappa(0);
      }
    }
  }
  if (init_points.path_point_size() < 1) {
    return false;
  }
  int lower_idx = 0;
  for (int i = 1; i < init_points.path_point_size(); i++) {
    double phi = init_points.path_point(i).theta();
    double last_phi = init_points.path_point(i - 1).theta();
    if (pre_phi <= init_points.path_point(0).theta()) {
      lower_idx = 0;
      break;
    }
    if (pre_phi >=
        init_points.path_point(init_points.path_point_size() - 1).theta()) {
      lower_idx = init_points.path_point_size() - 2;
      break;
    }

    if (pre_phi >= last_phi && pre_phi < phi) {
      lower_idx = i - 1;
      break;
    }
  }
  const double lower_phi = init_points.path_point(lower_idx).theta();
  const double lower_x = init_points.path_point(lower_idx).x();
  const double lower_y = init_points.path_point(lower_idx).y();
  const double upper_phi = init_points.path_point(lower_idx + 1).theta();
  const double upper_x = init_points.path_point(lower_idx + 1).x();
  const double upper_y = init_points.path_point(lower_idx + 1).y();
  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  if (radius > 3000.0) {
    vec_distance[0] = 0;
    vec_distance[1] = vehicle_state->linear_velocity() * t;
  } else {
    if (lower_idx >= cut_idx) {
      vec_distance[1] = lower_x + upper_x - lower_x;
      vec_distance[0] = -(lower_y + (upper_y - lower_y));
    } else {
      vec_distance[1] = lower_x + (pre_phi - lower_phi) /
                                      (upper_phi - lower_phi) *
                                      (upper_x - lower_x);
      vec_distance[0] =
          -(lower_y + (pre_phi - lower_phi) / (upper_phi - lower_phi) *
                          (upper_y - lower_y));
    }
  }
  if (vehicle_state->pose().has_quaternion() ||
      vehicle_state->is_vehicle_reference_frame()) {
    const auto& orientation = vehicle_state->pose().quaternion();
    Eigen::Quaternion<double> quaternion(orientation.w(), orientation.x(),
                                         orientation.y(), orientation.z());
    if (vehicle_state->is_vehicle_reference_frame()) {
      quaternion = common::math::HeadingToQuaternion(0.0);
    }
    Eigen::Vector3d pos_vec(vehicle_state->x(), vehicle_state->y(),
                            vehicle_state->z());
    const Eigen::Vector3d future_pos_3d =
        quaternion.toRotationMatrix() * vec_distance + pos_vec;
    *estimate_x = future_pos_3d[0];
    *estimate_y = future_pos_3d[1];
    return true;
  }
}

void VehicleStateProvider::SetVehicleReferenceFrame(
    VehicleState* const vehicle_state) {
  vehicle_state->set_x(0.0);
  vehicle_state->set_y(0.0);
  vehicle_state->set_z(0.0);
  vehicle_state->set_heading(0.0);
  vehicle_state->set_yaw(0.0);
  vehicle_state->set_is_vehicle_reference_frame(true);
  if (vehicle_state->chassis().has_yaw_rate()) {
    vehicle_state->set_angular_velocity(vehicle_state->chassis().yaw_rate());
  } else {
    vehicle_state->set_angular_velocity(0.0);
  }
  if (vehicle_state->chassis().has_speed_mps()) {
    vehicle_state->set_linear_velocity(vehicle_state->chassis().speed_mps());
  } else {
    vehicle_state->set_linear_velocity(0.0);
  }
  if (std::fabs(vehicle_state->linear_velocity()) < 0.01) {
    vehicle_state->set_angular_velocity(0.0);
  }
  if (vehicle_state->chassis().imu_acc().has_y()) {

    vehicle_state->set_linear_acceleration(
        vehicle_state->chassis().imu_acc().y());
  } else {
    vehicle_state->set_linear_acceleration(0.0);
  }
  if (vehicle_state->chassis().header().has_data_stamp()) {
    vehicle_state->set_timestamp(
        vehicle_state->chassis().header().data_stamp());
  } else {
    vehicle_state->set_timestamp(common::Clock::NowInSeconds());
  }
}

void VehicleStateProvider::SetNewZoneVehicleState(
    std::shared_ptr<common::VehicleState>& new_vehicle_state,
    int new_utm_zone) {
  if ((new_vehicle_state->pose().utm_zone_01()) == new_utm_zone) {
    new_vehicle_state->set_x(new_vehicle_state->pose().pos_utm_01().x());
    new_vehicle_state->set_y(new_vehicle_state->pose().pos_utm_01().y());
    new_vehicle_state->set_z(new_vehicle_state->pose().pos_utm_01().z());
  } else if ((new_vehicle_state->pose().utm_zone_02()) == new_utm_zone) {
    new_vehicle_state->set_x(new_vehicle_state->pose().pos_utm_02().x());
    new_vehicle_state->set_y(new_vehicle_state->pose().pos_utm_02().y());
    new_vehicle_state->set_z(new_vehicle_state->pose().pos_utm_02().z());
  } else if (new_utm_zone != 0 && new_utm_zone != -1) {
    AERROR << " zone is error ";
  }
}

double VehicleStateProvider::EstimateBetaAngle(
    const double steer_angle, const common::VehicleParam& vehicle_param) {

  const double steer_ratio =
      fmax(vehicle_param.has_steer_ratio_segment()
               ? common::math::InterpolationOne(
                     steer_angle * 180 / M_PI,
                     vehicle_param.steer_ratio_segment().steer_segment(),
                     vehicle_param.steer_ratio_segment().steer_ratio_segment())
               : vehicle_param.steer_ratio(),
           kEpsilon);
  ADEBUG << "steer_ratio : " << steer_ratio;
  return steer_angle / steer_ratio;
}

}  // namespace common
}  // namespace TL
