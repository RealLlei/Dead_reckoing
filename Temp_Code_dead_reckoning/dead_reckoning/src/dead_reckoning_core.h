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

#include <sys/time.h>

#include <fstream>
#include <iostream>
#include <memory>

#include "../common/kalmanfilter.h"
#include "../common/structs.h"
#include "../common/util.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/filters/digital_filter.h"
#include "common/filters/digital_filter_coefficients.h"

#include "proto/control/control_cmd.pb.h"
#include "proto/dead_reckoning/dead_reckoning_conf.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/soc/chassis.pb.h"

using apollo::soc::Chassis;

// #define EM_DEBUG

const unsigned int STATE_NUMBER = 12;
const double CON_g0 = 9.794645;        // acceleration of gravity
const double MAX_STEER_ANGLE = 475.0;  // max steer wheel angle

namespace apollo {
namespace dead_reckoning {

class DeadReckoningCore {
 public:
  // Public deleted functions can give better error msg
  DeadReckoningCore() = default;
  ~DeadReckoningCore() = default;
  void RunOnce(
      const std::shared_ptr<const apollo::soc::Chassis>& chassis,
      const std::shared_ptr<apollo::localization::Localization>& localization);
  void Init();
  void ResetState();
  void Stop();

 private:
  void SetLocalState();
  bool StateEstimateEkf();
  void PublishLocalization(
      const std::shared_ptr<apollo::localization::Localization>& localization);
  void LocalInitialize();
  void LocalStateDynamics(Eigen::VectorXd* dx, const Eigen::VectorXd& x);
  void LocalPredict(const double dt);
  void LocalYawCovUpdate();
  void LocalCarSpeedCorrect();
  void LocalStateUpdate();

 private:
  double yaw_counter_;
  bool is_imu_bias_init_;
  bool is_forward_;
  bool last_is_forward_;
  bool chassis_is_update_;
  bool wheel_speed_is_valid_;
  bool wheel_counter_is_valid_;
  bool enabled_ = false;
  std::shared_ptr<const apollo::soc::Chassis> chassis_received_;
  WheelSpeedData wheel_speed_;
  WheelSpeedData wheel_counter_speed_;
  CounterInfoData wheel_counter_;
  CounterInfoData last_wheel_counter_;
  ImuData imu_data_;
  SteerData steer_;
  LocalStateData local_state_;
  double last_msg_time_;
  Eigen::VectorXd x_;    // states(vel, acc_bias, ru, gyro_bias) ru: angle_bias?
  Eigen::MatrixXd P_;    // states covariance
  Eigen::Vector3d pos_;  // position relate to original of local frame
  Eigen::Quaterniond q_;  // attitude from local frame to car frame

  KalmanFilterCorrector car_speed_corrector_;
  KalmanFilterCorrector yaw_cov_corrector_;

  double yaw_l_;               // yaw from ENU frame to local frame
  Eigen::Vector3d gyro_bias_;  // gyro bias (rad/s)
  Eigen::Vector3d acc_bias_;   // gyro bias (rad/s)

  bool is_sensor_ready_;
  bool is_car_standstill_;
  bool is_car_go_straight_;
  double car_standstill_counter_;
  Eigen::Vector3d car_standstill_omg_sum_;
  Eigen::Vector3d car_standstill_acc_sum_;
  bool is_local_init_;

  int move_direction_ = 0;

  int get_speed_model_;  // get vehicle speed modle

  double neta_forward_coef_;
  double neta_backward_coef_;
  double rear_wheel_rot_arm_;

  DeadReckoningConf dead_reckoning_conf_;
  common::VehicleParam vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

#ifdef EM_DEBUG
  std::ofstream _fout;
#endif
};

Eigen::MatrixX3d get_cross_mat(const Eigen::Vector3d& w);
Eigen::Vector3d Quaternion2EulerZyx(const Eigen::Quaterniond& q);
Eigen::Vector3d Quaternion2EulerZxy(const Eigen::Quaterniond& q);
}  // namespace dead_reckoning
}  // namespace apollo
