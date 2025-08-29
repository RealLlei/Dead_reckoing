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
#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <deque>
#include <map>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

const double RAD_2_DEG = 180.0 / 3.1415926;
const double DEG_2_RAD = 1.0 / RAD_2_DEG;

namespace apollo {
namespace dead_reckoning {

struct ImuData {
  uint32_t seq;
  double timestamp;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_speed;
  Eigen::Quaterniond orientation_quternion;
  Eigen::Vector3d angular_pose;  // roll pitch yaw
};

struct LocalStateData {
  double timestamp;
  Eigen::Vector3d pos_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel_w;
  Eigen::Vector3d acc_w;
  Eigen::Quaterniond q_i_w;
  Eigen::Vector3d omg_i;
  Eigen::Vector3d VehicleToWorld(const Eigen::Vector3d& p_i) {
    return q_i_w * p_i + pos_w;
  }

  Eigen::Vector3d WorldToVehicle(const Eigen::Vector3d& p_w) {
    return q_i_w.inverse() * (p_w - pos_w);
  }
};

struct Point {
  float x;
  float y;
  float z;
};

struct WheelSpeedData {
  uint32_t seq;
  double timestamp;
  bool fl_wheel_speed_isvalid;
  bool fr_wheel_speed_isvalid;
  bool rl_wheel_speed_isvalid;
  bool rr_wheel_speed_isvalid;

  double fl_wheel_speed;  // m/s
  double fr_wheel_speed;
  double rl_wheel_speed;
  double rr_wheel_speed;
};

struct CounterInfoData {
  uint32_t seq;
  double timestamp;
  int driving_direction;
  bool fl_counter_info_isvalid;
  bool fr_counter_info_isvalid;
  bool rl_counter_info_isvalid;
  bool rr_counter_info_isvalid;

  double fl_counter_info;  // num
  double fr_counter_info;
  double rl_counter_info;
  double rr_counter_info;
};

struct SteerData {
  uint32_t seq;
  bool clock_wise;
  bool steer_valid;
  bool steer_rate_valid;
  double timestamp;
  double steer_angle;
  double steer_angle_speed;
  double steer_torque;
};

}  // namespace dead_reckoning
}  // namespace apollo
