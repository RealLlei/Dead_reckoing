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

#include "dead_reckoning/src/dead_reckoning_core.h"
// #include <fmt/ostream.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "common/file/file.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/quaternion.h"
#include "common/util/message_util.h"
#include "common/util/util.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "dead_reckoning/src/dead_reckoning_gflags.h"

using apollo::common::Clock;
using apollo::common::math::HeadingToQuaternion;
using apollo::common::math::InterpolateUsingLinearApproximation;
using apollo::common::util::FillHeader;
using apollo::common::util::TransformToMRF;
using apollo::common::util::TransformToVRF;
using apollo::soc::Chassis;
using apollo::soc::WheelSpeed_WheelSpeedType;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace apollo {
namespace dead_reckoning {

void DeadReckoningCore::Init() {
  // get params
  AINFO << "Dead_reckoning vio init, starting ...";
  ACHECK(common::GetProtoFromFile(FLAGS_dead_reckoning_conf_file,
                                  &dead_reckoning_conf_))
      << "Unable to load dead reckoning conf file: " +
             FLAGS_dead_reckoning_conf_file;

  AINFO << "Conf file: " << FLAGS_dead_reckoning_conf_file << " is loaded.";

  neta_forward_coef_ = dead_reckoning_conf_.neta_forward_coef();
  neta_backward_coef_ = dead_reckoning_conf_.neta_backward_coef();
  rear_wheel_rot_arm_ = dead_reckoning_conf_.rear_wheel_rot_arm();

  car_speed_corrector_._len_sw = 0;
  car_speed_corrector_._chi_thr = 1.0;
  car_speed_corrector_._gain_coef.setConstant(12, 1, 1.0);
  car_speed_corrector_._gain_coef(8) = 0.0;
  car_speed_corrector_._gain_coef(11) = 0.0;
  yaw_cov_corrector_._len_sw = 0;
  yaw_cov_corrector_._chi_thr = 1.0;

  is_local_init_ = false;

  chassis_received_ = std::make_shared<Chassis>();
  x_ = Eigen::VectorXd::Zero(STATE_NUMBER);
  P_ = Eigen::MatrixXd::Identity(STATE_NUMBER, STATE_NUMBER);

  wheel_speed_.fl_wheel_speed = 0.0;
  wheel_speed_.fr_wheel_speed = 0.0;
  wheel_speed_.rl_wheel_speed = 0.0;
  wheel_speed_.rr_wheel_speed = 0.0;

  wheel_counter_speed_.fl_wheel_speed = 0.0;
  wheel_counter_speed_.fr_wheel_speed = 0.0;
  wheel_counter_speed_.rl_wheel_speed = 0.0;
  wheel_counter_speed_.rr_wheel_speed = 0.0;

  wheel_counter_.fl_counter_info = 0.0;
  wheel_counter_.fr_counter_info = 0.0;
  wheel_counter_.rl_counter_info = 0.0;
  wheel_counter_.rr_counter_info = 0.0;

  last_wheel_counter_.fl_counter_info = 0.0;
  last_wheel_counter_.fr_counter_info = 0.0;
  last_wheel_counter_.rl_counter_info = 0.0;
  last_wheel_counter_.rr_counter_info = 0.0;

  imu_data_.angular_speed(0) = 0.0;
  imu_data_.angular_speed(1) = 0.0;
  imu_data_.angular_speed(2) = 0.0;

  imu_data_.linear_acceleration(0) = 0.0;
  imu_data_.linear_acceleration(1) = 0.0;
  imu_data_.linear_acceleration(2) = 0.0;

  is_sensor_ready_ = false;
  is_car_standstill_ = false;
  is_local_init_ = false;
  chassis_is_update_ = false;
  wheel_speed_is_valid_ = false;
  wheel_counter_is_valid_ = false;
  car_standstill_counter_ = 0.0;
  car_standstill_omg_sum_ = Eigen::Vector3d::Zero();
  is_forward_ = last_is_forward_ = true;
  is_imu_bias_init_ = false;
  move_direction_ = 0;
}

void DeadReckoningCore::Stop() {
  if (enabled_) {
    enabled_ = false;
  }
}

void DeadReckoningCore::ResetState() {
  pos_.setZero();
  q_.setIdentity();
  yaw_counter_ = 0.0;
}

void DeadReckoningCore::RunOnce(
    const std::shared_ptr<const apollo::soc::Chassis>& chassis,
    const std::shared_ptr<apollo::localization::Localization>& localization) {
  // add chassis info when  simulating
  chassis_received_ = chassis;
  chassis_is_update_ =
      ((chassis_received_->header().publish_stamp() - last_msg_time_) > 1.0e-3);
  AINFO << "chassis_is_update_Info: " << FIXED << SETPRECISION(4)
        << "current_time: " << chassis_received_->header().publish_stamp()
        << "last_time: " << last_msg_time_
        << "chassis_is_update_falg: " << chassis_is_update_;
  ADEBUG << "onchassis wheel_counter" << chassis->has_wheel_counter();
  ADEBUG << "RunOnce wheel_counter" << chassis->has_wheel_counter();
  if (!StateEstimateEkf()) {
    return;
  }
  PublishLocalization(localization);

  if (chassis_is_update_) {
    last_msg_time_ = chassis_received_->header().publish_stamp();
  }
}

bool DeadReckoningCore::StateEstimateEkf() {
  double dt = (chassis_received_->header().publish_stamp() - last_msg_time_);
  // if(dt<1.0e-3){
  //       AINFO << "Dead_reckoning vio init, starting ...";
  //       chassis_is_update_ =false;
  // } else{
  //   chassis_is_update_=true;
  // }
  AINFO << "StateEstimateEkf entried: "
        << " dt: " << dt << " chassis_is_update_: " << chassis_is_update_;
  if (!chassis_is_update_) {
    imu_data_.angular_speed.setZero();
    imu_data_.linear_acceleration = {0.0, 0.0, CON_g0};
    chassis_is_update_ = false;
    wheel_speed_is_valid_ = false;
    return false;
  } else {
    AINFO << "chassis_has_yaw_rate: " << FIXED << SETPRECISION(4)
          << chassis_received_->has_yaw_rate();
    if (chassis_received_->has_yaw_rate()) {
      imu_data_.timestamp = chassis_received_->header().publish_stamp();
      // since raw_angle_rate has  non-zero bias in  z direction, this should be
      // substr

      imu_data_.angular_speed(0) = 0.0;
      imu_data_.angular_speed(1) = 0.0;
      imu_data_.angular_speed(2) = chassis_received_->yaw_rate();

      imu_data_.linear_acceleration(0) = chassis_received_->imu_acc().y();
      imu_data_.linear_acceleration(1) =
          -1.0 * chassis_received_->imu_acc().x();
      imu_data_.linear_acceleration(2) = CON_g0;
      AINFO << "imu_data_.angular_speed: " << FIXED << SETPRECISION(6)
            << imu_data_.angular_speed(0) << "; " << imu_data_.angular_speed(1)
            << "; " << imu_data_.angular_speed(2);
      AINFO << "imu_data_.linear_acceleration: " << FIXED << SETPRECISION(6)
            << imu_data_.linear_acceleration(0) << "; "
            << imu_data_.linear_acceleration(1) << "; "
            << imu_data_.linear_acceleration(2);
    } else {
      ADEBUG << "Waiting imu data...";
      return false;
    }

    if (chassis_received_->has_wheel_speed()) {
      wheel_speed_.timestamp = chassis_received_->header().publish_stamp();
      auto wheel_speed = chassis_received_->wheel_speed();
      wheel_speed_.fl_wheel_speed_isvalid = wheel_speed.is_wheel_spd_fl_valid();
      wheel_speed_.fr_wheel_speed_isvalid = wheel_speed.is_wheel_spd_fr_valid();
      wheel_speed_.rl_wheel_speed_isvalid = wheel_speed.is_wheel_spd_rl_valid();
      wheel_speed_.rr_wheel_speed_isvalid = wheel_speed.is_wheel_spd_rr_valid();

      wheel_speed_.fl_wheel_speed = wheel_speed.wheel_spd_fl();
      wheel_speed_.fr_wheel_speed = wheel_speed.wheel_spd_fr();
      wheel_speed_.rl_wheel_speed = wheel_speed.wheel_spd_rl();
      wheel_speed_.rr_wheel_speed = wheel_speed.wheel_spd_rr();
      // TODO(gongshengbo) optimize this assertation

      if ((wheel_speed.wheel_direction_rl() == soc::WheelSpeed::BACKWARD) &&
              (wheel_speed.wheel_direction_rr() != soc::WheelSpeed::FORWARD) ||
          (wheel_speed.wheel_direction_rl() != soc::WheelSpeed::FORWARD) &&
              (wheel_speed.wheel_direction_rr() == soc::WheelSpeed::BACKWARD)) {
        move_direction_ = -1;
        wheel_speed_.fl_wheel_speed = -1.0 * wheel_speed_.fl_wheel_speed;
        wheel_speed_.fr_wheel_speed = -1.0 * wheel_speed_.fr_wheel_speed;
        wheel_speed_.rl_wheel_speed = -1.0 * wheel_speed_.rl_wheel_speed;
        wheel_speed_.rr_wheel_speed = -1.0 * wheel_speed_.rr_wheel_speed;

      } else if ((wheel_speed.wheel_direction_rl() ==
                  soc::WheelSpeed::FORWARD) &&
                     (wheel_speed.wheel_direction_rr() !=
                      soc::WheelSpeed::BACKWARD) ||
                 (wheel_speed.wheel_direction_rl() !=
                  soc::WheelSpeed::BACKWARD) &&
                     (wheel_speed.wheel_direction_rr() ==
                      soc::WheelSpeed::FORWARD)) {
        move_direction_ = 1;
      } else {
        move_direction_ = 0;
      }
      //  AINFO << "wheel_direction_Info:
      //  "<<std::setprecision(2)<<wheel_speed.WheelSpeedType_IsValid()<<wheel_speed.WheelSpeedType_Name()->has_imu_raw_ang_rate();

      wheel_speed_is_valid_ = true;
    } else {
      wheel_speed_is_valid_ = false;
      ADEBUG << "Waiting wheel speed data...";
      return false;
    }
    ADEBUG << "StateEstimateEkf wheel_counter"
           << chassis_received_->has_wheel_counter();

    if (chassis_received_->has_wheel_counter()) {
      wheel_counter_.timestamp = chassis_received_->header().publish_stamp();
      auto wheel_counter = chassis_received_->wheel_counter();
      double forward_counter_coef = 1.0 / neta_forward_coef_;
      double backward_counter_coef = 1.0 / neta_backward_coef_;

      wheel_counter_is_valid_ = wheel_counter.is_wheel_cnt_fr_valid() &&
                                wheel_counter.is_wheel_cnt_fl_valid() &&
                                wheel_counter.is_wheel_cnt_rr_valid() &&
                                wheel_counter.is_wheel_cnt_rl_valid();
      wheel_counter_.fl_counter_info_isvalid =
          wheel_counter.is_wheel_cnt_fl_valid();
      wheel_counter_.fr_counter_info_isvalid =
          wheel_counter.is_wheel_cnt_fr_valid();
      wheel_counter_.rl_counter_info_isvalid =
          wheel_counter.is_wheel_cnt_rl_valid();
      wheel_counter_.rr_counter_info_isvalid =
          wheel_counter.is_wheel_cnt_rr_valid();

      wheel_counter_.fl_counter_info = wheel_counter.wheel_counter_fl();
      wheel_counter_.fr_counter_info = wheel_counter.wheel_counter_fr();
      wheel_counter_.rl_counter_info = wheel_counter.wheel_counter_rl();
      wheel_counter_.rr_counter_info = wheel_counter.wheel_counter_rr();
      double full_scale = 32767.0;
      auto lmd = [&full_scale](double a) -> double {
        if (a > 0.5 * full_scale) {
          a -= full_scale;
        } else if (a < -0.5 * full_scale) {
          a += full_scale;
        }
        return a;
      };

      AINFO << "wheel_counter_is_valid_: " << FIXED << SETPRECISION(4)
            << wheel_counter_is_valid_;

      if (wheel_counter_is_valid_) {
        if (dt >= 1.0e-3) {
          wheel_counter_speed_.fl_wheel_speed =
              lmd(fabs(wheel_counter_.fl_counter_info) -
                  fabs(last_wheel_counter_.fl_counter_info)) /
              dt;
          wheel_counter_speed_.fr_wheel_speed =
              lmd(fabs(wheel_counter_.fr_counter_info) -
                  fabs(last_wheel_counter_.fr_counter_info)) /
              dt;
          wheel_counter_speed_.rl_wheel_speed =
              lmd(fabs(wheel_counter_.rl_counter_info) -
                  fabs(last_wheel_counter_.rl_counter_info)) /
              dt;
          wheel_counter_speed_.rr_wheel_speed =
              lmd(fabs(wheel_counter_.rr_counter_info) -
                  fabs(last_wheel_counter_.rr_counter_info)) /
              dt;
        } else {
          wheel_counter_speed_.fl_wheel_speed = 0.0;
          wheel_counter_speed_.fr_wheel_speed = 0.0;
          wheel_counter_speed_.rl_wheel_speed = 0.0;
          wheel_counter_speed_.rr_wheel_speed = 0.0;
        }
        ADEBUG << "rr_wheel_speed: : " << wheel_counter_speed_.rr_wheel_speed;
        Eigen::Vector3d det_pos = Eigen::Vector3d::Zero();

        det_pos.x() = 0.5 *
                      (wheel_counter_speed_.rl_wheel_speed +
                       wheel_counter_speed_.rr_wheel_speed) *
                      dt;
        double det_yaw = (wheel_counter_speed_.rr_wheel_speed -
                          wheel_counter_speed_.rl_wheel_speed) *
                         dt;
        // TODO(huangyun): confirm the direction with EP30

        // if (wheel_counter_.rr_counter_info >= 0.0) {
        //   direction = 1.0;
        // } else {
        //   direction = -1.0;
        // }
        det_pos *= move_direction_;
        det_yaw *= move_direction_;

        double counter_coef =
            det_pos.x() >= 0.0 ? forward_counter_coef : backward_counter_coef;
        det_pos.x() *= counter_coef;
        det_yaw *= counter_coef / rear_wheel_rot_arm_;
        pos_ += q_ * det_pos;
        // if (dt>=1.0e-3){
        //    // write data into file
        //    //t, x, y, z dx, dy, dz, vx, vy, vz, ax, ay, az,   ang_rate_x,
        //    ang_rate_y, ang_rate_z
        //   std::vector<double> temp(std::vector<double>(16, 0));
        //   temp[0] = chassis_received_->header().publish_stamp();
        //   temp[1] = pos_.x();
        //   temp[2] = pos_.y();
        //   temp[3] = pos_.z();
        //   temp[4] = det_q.x();
        //   temp[5] = det_q.y();
        //   temp[6] = det_q.z();
        //   std::ofstream
        //   ofs("/home/gongshengbo/ads_hz/ad/tools/plot_trajectory/hz_vio_log/vio_log.txt",
        //   std::ios::app); for(int ii =0; ii<16;ii++){
        //       ofs<<std::setprecision(11)<<temp[ii]<<'\t';
        //   }
        //   ofs<<std::endl;
        //   ofs.close();

        // }

        yaw_counter_ += det_yaw;
        yaw_counter_ = atan2(sin(yaw_counter_), cos(yaw_counter_));
      }

      if (chassis_received_->has_steering_percentage()) {
        steer_.steer_angle =
            chassis_received_->steering_percentage() * MAX_STEER_ANGLE;
      }
      if (chassis_received_->has_steering_torque_nm()) {
        steer_.steer_torque =
            chassis_received_->steering_torque_nm() * MAX_STEER_ANGLE;
      }
      last_wheel_counter_ = wheel_counter_;
    }
  }

  Eigen::Quaterniond q_yaw_l{cos(0.5 * yaw_l_), 0.0, 0.0, sin(0.5 * yaw_l_)};

  // check the sensor whether has data first
  if (!is_sensor_ready_) {
    // once has wheel data, goon
    if (wheel_speed_is_valid_ == true) {
      is_sensor_ready_ = true;
    } else {
      return false;
    }
  }
  if (wheel_counter_is_valid_) {
    Eigen::VectorXd wheel_counter_speed(4);
    wheel_counter_speed << wheel_counter_speed_.fl_wheel_speed,
        wheel_counter_speed_.fr_wheel_speed,
        wheel_counter_speed_.rl_wheel_speed,
        wheel_counter_speed_.rr_wheel_speed;
    is_car_standstill_ =
        wheel_counter_speed.norm() < FLAGS_default_standstill_value;
    Eigen::VectorXd wheel_counter_dif(2);
    wheel_counter_dif << wheel_counter_speed_.fl_wheel_speed -
                             wheel_counter_speed_.fr_wheel_speed,
        wheel_counter_speed_.rl_wheel_speed -
            wheel_counter_speed_.rr_wheel_speed;
    is_car_go_straight_ =
        fabs(wheel_counter_speed_.rr_wheel_speed) > 10 &&
        wheel_counter_speed.norm() < FLAGS_default_standstill_value;
  }
  ADEBUG << "is_car_go_straight_ " << is_car_go_straight_;

  // local state estimate
  // state init
  if (!is_local_init_) {
    LocalInitialize();
    q_.setIdentity();
  } else {
    if (FLAGS_enable_auto_angle_rate_bias) {
      if (is_car_standstill_) {
        car_standstill_counter_ += 1.0;
        car_standstill_omg_sum_ += imu_data_.angular_speed;
        car_standstill_acc_sum_ += imu_data_.linear_acceleration;
        if (car_standstill_counter_ >= FLAGS_auto_angle_rate_bias_time) {
          gyro_bias_ +=
              1.0 *
              (car_standstill_omg_sum_ / car_standstill_counter_ - gyro_bias_);
          acc_bias_ +=
              1.0 *
              (car_standstill_acc_sum_ / car_standstill_counter_ - acc_bias_);
          acc_bias_[2] = 0.0;
          car_standstill_counter_ = 0.0;
          car_standstill_omg_sum_.setZero();
          car_standstill_acc_sum_.setZero();
          is_imu_bias_init_ = true;
          ADEBUG << "Complete auto angle rate bias estimate: " << gyro_bias_(2)
                 << " acc x " << acc_bias_(0) << " acc y " << acc_bias_(1);
        }
      } else {
        car_standstill_counter_ = 0.0;
        car_standstill_omg_sum_.setZero();
        car_standstill_acc_sum_.setZero();
      }
    } else {
      is_imu_bias_init_ = true;
      ADEBUG << "Complete angle rate bias set: "
             << FLAGS_default_angle_rate_bias_value;
    }
    if (!is_imu_bias_init_) {
      ADEBUG << "Waiting imu bias estimation... counter: "
             << car_standstill_counter_;
      q_.setIdentity();
      // return false;
    }

    Eigen::Quaterniond det_q;
    Eigen::Vector3d det_ang = (imu_data_.angular_speed - gyro_bias_) * dt;
    double theta = det_ang.norm();
    AINFO << "theta: " << FIXED << SETPRECISION(6) << theta;

    det_q.w() = cos(0.5 * theta);
    det_q.vec() = sin(0.5 * theta) * det_ang.normalized();
    q_ = q_ * det_q;

    ADEBUG << "env imu init finished!";

    // local position prediction
    LocalPredict(dt);

    // local state update
    if (wheel_speed_is_valid_) {
      ADEBUG << "call local_car_speed_correct";
      LocalCarSpeedCorrect();
    }

    static unsigned int counter = 0;
    if (++counter % 100 == 0) {
      LocalYawCovUpdate();
    }
  }

  SetLocalState();

  return true;
}

void DeadReckoningCore::SetLocalState() {
  Eigen::MatrixXd vec_g0 = Eigen::MatrixXd::Zero(3, 1);
  vec_g0(2, 0) = CON_g0;

  local_state_.timestamp = imu_data_.timestamp;
  local_state_.pos_w = pos_;
  // output  linear  vel acc  in  vehicle  framework
  Eigen::MatrixXd trans_mat = q_.toRotationMatrix().transpose();
  local_state_.vel_w = trans_mat * x_.segment(0, 3);
  local_state_.acc_w = imu_data_.linear_acceleration - acc_bias_ -
                       x_.segment(3, 3) -
                       trans_mat * vec_g0;  // local_acc x,y,z
  local_state_.q_i_w = q_;
  local_state_.omg_i = imu_data_.angular_speed - gyro_bias_;  // rad/s

  AINFO << "SetLocalState q_: " << FIXED << SETPRECISION(6) << q_.x() << "; "
        << q_.y() << "; " << q_.z() << "; " << q_.w();
}

void DeadReckoningCore::LocalInitialize() {
  const double vel_sgm = 0.3;
  const double acc_sgm = 0.03;
  const double att_sgm = 0.5 * DEG_2_RAD;
  const double gyro_bias_sgm = 0.5 * DEG_2_RAD;
  const double yaw_sgm = 0.5 * DEG_2_RAD;

  x_ = Eigen::VectorXd::Zero(STATE_NUMBER);
  P_ = Eigen::MatrixXd::Identity(STATE_NUMBER, STATE_NUMBER);
  for (int i = 0; i != 3; ++i) {
    P_(i, i) = pow(vel_sgm, 2.0);
    P_(3 + i, 3 + i) = pow(acc_sgm, 2.0);
    P_(6 + i, 6 + i) = pow(att_sgm, 2.0);
    P_(9 + i, 9 + i) = pow(gyro_bias_sgm, 2.0);
  }
  P_(8, 8) = pow(yaw_sgm, 2.0);
  pos_.setZero(3, 1);
  Eigen::VectorXd acc = imu_data_.linear_acceleration;

  acc.normalize();
  double roll = atan2(acc(1), acc(2));
  double pitch = -asin(acc(0));
  q_ = (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  yaw_l_ = 0.0;
  gyro_bias_.setZero(3, 1);
  acc_bias_.setZero(3, 1);

  is_local_init_ = true;
}

// dx =: dx/dt, x =: x_
void DeadReckoningCore::LocalStateDynamics(Eigen::VectorXd* dx,
                                           const Eigen::VectorXd& x) {
  *dx = Eigen::VectorXd::Zero(x.rows());
  Eigen::MatrixXd vec_g0 = Eigen::MatrixXd::Zero(3, 1);
  vec_g0(2, 0) = CON_g0;
  dx->segment(0, 3) = q_.toRotationMatrix() * (imu_data_.linear_acceleration -
                                               acc_bias_ - x.segment(3, 3)) -
                      vec_g0;
}

void DeadReckoningCore::LocalPredict(const double dt) {
  // state prediction
  Eigen::VectorXd dx;
  LocalStateDynamics(&dx, x_);
  AINFO << "lat velocity increament: " << dx(0) * dt
        << " , current lat velocity :" << x_(0)
        << " next lat vel:" << x_(0) + dx(0) * dt;
  AINFO << "long velocity increament: " << dx(1) * dt
        << " , current long velocity :" << x_(1)
        << " next long vel:" << x_(1) + dx(1) * dt;
  x_ = x_ + dt * dx;
  // state covariance prediction
  Eigen::MatrixXd prc_mat = Eigen::MatrixXd::Zero(x_.rows(), x_.rows());
  const double sgm_vel_prc = 0.3;
  const double sgm_acc_prc = 0.01;
  const double sgm_att_prc = 0.2 * DEG_2_RAD;
  const double sgm_gyro_bias_prc = 0.01 * DEG_2_RAD;
  for (int i = 0; i != 3; ++i) {
    prc_mat(i, i) = dt * pow(sgm_vel_prc, 2.0);
    prc_mat(3 + i, 3 + i) = dt * pow(sgm_acc_prc, 2.0);
    prc_mat(6 + i, 6 + i) = dt * pow(sgm_att_prc, 2.0);
    prc_mat(9 + i, 9 + i) = dt * pow(sgm_gyro_bias_prc, 2.0);
  }

  Eigen::MatrixXd rot_mat = q_.toRotationMatrix();
  Eigen::MatrixXd sys_mat = Eigen::MatrixXd::Zero(x_.rows(), x_.rows());
  // sys_mat.block<3,3>(0,3) = - rot_mat;
  sys_mat.block<3, 1>(0, 5) = -rot_mat.col(2);
  sys_mat.block<3, 3>(0, 6) =
      -2.0 * get_cross_mat(rot_mat * (imu_data_.linear_acceleration -
                                      acc_bias_ - x_.segment(3, 3)))
                 .block<3, 3>(0, 0);
  sys_mat.block<3, 3>(6, 9) = -0.5 * rot_mat;

  Eigen::MatrixXd disc_sys_mat =
      Eigen::MatrixXd::Identity(sys_mat.rows(), sys_mat.cols());
  disc_sys_mat += dt * sys_mat;
  P_ = disc_sys_mat * P_ * disc_sys_mat.transpose() + prc_mat;
}

void DeadReckoningCore::LocalStateUpdate() {
  const double max_gyro_bias = 7.0 * DEG_2_RAD;
  VectorXd ru = x_.segment(6, 3);
  Eigen::Quaterniond quat_error(1.0, ru(0), ru(1), ru(2));
  quat_error.normalize();
  q_ = quat_error * q_;
  AINFO << "quat_error: " << FIXED << SETPRECISION(6) << quat_error.x() << "; "
        << quat_error.y() << "; " << quat_error.z() << "; " << quat_error.w();

  gyro_bias_ += x_.segment(9, 3);
  for (unsigned int i = 0; i != 3; ++i) {
    gyro_bias_(i) =
        std::min(std::max(gyro_bias_(i), -max_gyro_bias), max_gyro_bias);
  }
  x_.block<3, 1>(6, 0) = VectorXd::Zero(3, 1);
  x_.block<3, 1>(9, 0) = VectorXd::Zero(3, 1);
}

void DeadReckoningCore::LocalYawCovUpdate() {
  const double sgm_yaw = 3.0 * DEG_2_RAD;
  Eigen::MatrixXd r_mat = Eigen::MatrixXd::Zero(1, 1);
  r_mat(0, 0) = pow(sgm_yaw, 2.0);

  Eigen::MatrixXd trans_mat = q_.toRotationMatrix();
  Eigen::VectorXd res_error = Eigen::VectorXd::Zero(1);
  Eigen::MatrixXd h_mat = Eigen::MatrixXd::Zero(1, x_.rows());
  h_mat(0, 8) = 1.0;

  yaw_cov_corrector_.AdaptiveCorrect(&x_, &P_, res_error, h_mat, r_mat);
  LocalStateUpdate();
}

void DeadReckoningCore::LocalCarSpeedCorrect() {
  Eigen::Vector3d v_car = Eigen::Vector3d::Zero(3);
  v_car(0) = (wheel_speed_.rl_wheel_speed + wheel_speed_.rr_wheel_speed) * 0.5;

  const double sgm_vel = 0.03 + 0.01 * v_car.norm();

  Eigen::MatrixXd r_mat = Eigen::MatrixXd::Zero(3, 3);
  r_mat(0, 0) = pow(sgm_vel, 2.0);
  r_mat(1, 1) = r_mat(2, 2) = r_mat(0, 0) * 0.05;

  Eigen::MatrixXd trans_mat = q_.toRotationMatrix();
  Eigen::VectorXd res_error = v_car - trans_mat.transpose() * x_.segment(0, 3);
  ADEBUG << "current_v_car," << v_car(0) << " vel_err_0: " << res_error(0)
         << " vel_err_1: " << res_error(1);

  Eigen::MatrixXd h_mat = Eigen::MatrixXd::Zero(3, x_.rows());
  h_mat.block<3, 3>(0, 0) = trans_mat.transpose();
  h_mat.block<3, 3>(0, 6) =
      2.0 * get_cross_mat(trans_mat.transpose() * x_.segment(0, 3));

  car_speed_corrector_.AdaptiveCorrect(&x_, &P_, res_error, h_mat, r_mat);
  AINFO << "AdaptiveCorrect speed, x(0) :" << x_(0) << ", x(1): " << x_(1);

  LocalStateUpdate();
}

Eigen::MatrixX3d get_cross_mat(const Eigen::Vector3d& w) {
  double wx = w(0, 0);
  double wy = w(1, 0);
  double wz = w(2, 0);
  Eigen::MatrixXd a_mat(3, 3);
  a_mat << 0.0, -wz, wy, wz, 0.0, -wx, -wy, wx, 0.0;
  return a_mat;
}

void DeadReckoningCore::PublishLocalization(
    const std::shared_ptr<apollo::localization::Localization>& localization) {
  FillHeader("DeadReckoning", localization.get());
  localization->set_measurement_time(
      chassis_received_->header().publish_stamp());
  auto* pose = localization->mutable_pose();

  Eigen::Vector3d euler = Quaternion2EulerZyx(local_state_.q_i_w);
  // range of euer(2) is (-2*M_PI, 2*M_PI), change it to (-M_PI, M_PI)
  if (euler(2) < -M_PI) {
    euler(2) = euler(2) + 2 * M_PI;
  } else if (euler(2) > M_PI) {
    euler(2) = euler(2) - 2 * M_PI;
  }
  double heading_angle_pub = euler(2) + 0.5 * M_PI;
  if (heading_angle_pub > M_PI) {
    heading_angle_pub = heading_angle_pub - 2 * M_PI;
  } else if (heading_angle_pub < -M_PI) {
    heading_angle_pub = heading_angle_pub + 2 * M_PI;
  }

  // Set position
  pose->mutable_position()->set_x(-1.0 * local_state_.pos_w(1));
  pose->mutable_position()->set_y(local_state_.pos_w(0));
  pose->mutable_position()->set_z(local_state_.pos_w(2));
  // Set heading
  pose->set_heading(heading_angle_pub);
  // Set orientation
  Eigen::Quaterniond q_rot =
      (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())) * q_;
  pose->mutable_quaternion()->set_w(q_rot.w());
  pose->mutable_quaternion()->set_x(q_rot.x());
  pose->mutable_quaternion()->set_y(q_rot.y());
  pose->mutable_quaternion()->set_z(q_rot.z());

  // Set linear_velocity_vrf
  pose->mutable_linear_velocity_vrf()->set_x(-1.0 * local_state_.vel_w(1));
  pose->mutable_linear_velocity_vrf()->set_y(local_state_.vel_w(0));
  pose->mutable_linear_velocity_vrf()->set_z(local_state_.vel_w(2));
  // Set linear_acc_vrf
  pose->mutable_linear_acceleration_vrf()->set_x(-1.0 * local_state_.acc_w(1));
  pose->mutable_linear_acceleration_vrf()->set_y(local_state_.acc_w(0));
  pose->mutable_linear_acceleration_vrf()->set_z(local_state_.acc_w(2) -
                                                 CON_g0);
  // Set angular_velocity_vrf
  pose->mutable_angular_velocity_vrf()->set_x(-1.0 * local_state_.omg_i(1));
  pose->mutable_angular_velocity_vrf()->set_y(local_state_.omg_i(0));
  pose->mutable_angular_velocity_vrf()->set_z(local_state_.omg_i(2));

  Eigen::MatrixXd trans_mat = q_.toRotationMatrix();
  // Set  linear velocity
  TransformToMRF(pose->linear_velocity_vrf(), pose->quaternion(),
                 pose->mutable_linear_velocity());
  // Set  linear acc
  TransformToMRF(pose->linear_acceleration_vrf(), pose->quaternion(),
                 pose->mutable_linear_acceleration());
  // Set angle velocity
  TransformToMRF(pose->angular_velocity_vrf(), pose->quaternion(),
                 pose->mutable_angular_velocity());
  // Set euler angle
  pose->mutable_euler_angles()->set_x(euler(0));
  pose->mutable_euler_angles()->set_y(euler(1));
  pose->mutable_euler_angles()->set_z(euler(2));

  localization->mutable_pose()->mutable_pos_utm_01()->CopyFrom(
      pose->position());
  localization->mutable_pose()->mutable_pos_utm_02()->CopyFrom(
      pose->position());
  localization->mutable_pose()->set_utm_zone_01(FLAGS_local_utm_zone_id);
  localization->mutable_pose()->set_utm_zone_02(FLAGS_local_utm_zone_id);
  localization->mutable_pose()->set_using_utm_zone(FLAGS_local_utm_zone_id);

  ADEBUG << "euler_angles: " << FIXED << SETPRECISION(6) << euler(0) << ";, "
         << euler(1) << ";, " << euler(2);

  ADEBUG << "point x," << FIXED << SETPRECISION(3) << pose->position().x()
         << ",y," << pose->position().y() << ",heading," << pose->heading()
         << " ,v_x," << pose->linear_velocity().x()
         << ", v_y: " << pose->linear_velocity().y()
         << ", a_x: " << pose->linear_acceleration().x()
         << ", a_y: " << pose->linear_acceleration().y();
  //  write data into file
  //   t, x, y, z  vx, vy, vz, ax, ay, az,   roll,  pitch , yaw,  ang_rate_x,
  //   ang_rate_y, ang_rate_z,  Quaternion1x4
  std::vector<double> temp(std::vector<double>(21, 0.0));
  temp[0] = chassis_received_->header().publish_stamp();
  temp[1] = pose->position().x();
  temp[2] = pose->position().y();
  temp[3] = pose->position().z();

  temp[4] = pose->linear_velocity().x();
  temp[5] = pose->linear_velocity().y();
  temp[6] = pose->linear_velocity().z();

  temp[7] = pose->linear_acceleration().x();
  temp[8] = pose->linear_acceleration().y();
  temp[9] = pose->linear_acceleration().z();
  // heading angle
  temp[12] = pose->heading();

  temp[13] = pose->angular_velocity().x();
  temp[14] = pose->angular_velocity().y();
  temp[15] = pose->angular_velocity().z();

  temp[16] = pose->quaternion().x();
  temp[17] = pose->quaternion().y();
  temp[18] = pose->quaternion().z();
  temp[18] = pose->quaternion().w();

  temp[19] = wheel_speed_.rr_wheel_speed;
  temp[20] = wheel_counter_speed_.rr_wheel_speed;

  std::ofstream ofs(
      "/home/liubei/"
      "dr_output_log.txt",
      std::ios::app);
  for (int ii = 0; ii < 21; ii++) {
    ofs << std::setprecision(16) << temp[ii] << '\t';
  }
  ofs << std::endl;
  ofs.close();
}

}  // namespace dead_reckoning
}  // namespace apollo
