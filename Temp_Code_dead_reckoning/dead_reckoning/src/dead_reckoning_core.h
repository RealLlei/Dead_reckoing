/******************************************************************************
 * Copyright 2017 The Magna Authors. All Rights Reserved.
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

using Magna::soc::Chassis;

// #define EM_DEBUG

const unsigned int STATE_NUMBER = 12;
const double CON_g0 = 9.794645;        // acceleration of gravity
const double MAX_STEER_ANGLE = 475.0;  // max steer wheel angle

namespace Magna {
namespace dead_reckoning {

struct DeadReckoningConfig {

    bool use_imu = false;
	double wheel_radius = 0.35;        // 轮子半径（单位：米，默认值）
     
    // 轮速校正系数（前进/后退）  
    double neta_forward_coef = 50.0;    // 前进时前轮速系数（默认值）  
    double neta_backward_coef = 50.0;   // 后退时前轮速系数（默认值）  

    // 车辆物理参数  
    double rear_wheel_rot_arm = 1.6;   // 后轮旋转臂长（单位：米，默认值） 
    double gain_yaw_count = 0.0;      // 偏航角计数器增益（默认值）
	double gain_omg_z_count = 0.0;      // 角速度z轴计数器增益（默认值）
	
    double default_standstill_value = 0.01;  // 替代FLAGS_default_standstill_value
    bool enable_auto_angle_rate_bias = true; // 替代FLAGS_enable_auto_angle_rate_bias
    int auto_angle_rate_bias_time = 100;     // 替代FLAGS_auto_angle_rate_bias_time
    double default_angle_rate_bias_value = 0.001; // 替代FLAGS_default_angle_rate_bias_value

  
};

// 1. 车轮方向枚举（替代protobuf的WheelDirection）  
enum class WheelDirection {
    FORWARD = 0,  // 前进  
    BACKWARD = 1,   // 后退  
    UNKNOWN = 2
};
// 2. 轮速数据结构体（包含四轮速度和方向）  
struct WheelSpeedData {
    uint32_t seq = 0;
    double timestamp = 0.0;
    bool fl_valid = false;  // 左前轮有效标志
    bool fr_valid = false;  // 右前轮有效标志
    bool rl_valid = false;  // 左后轮有效标志
    bool rr_valid = false;  // 右后轮有效标志
    double fl_speed = 0.0;  // 左前轮速度(m/s)
    double fr_speed = 0.0;  // 右前轮速度(m/s)
    double rl_speed = 0.0;  // 左后轮速度(m/s)
    double rr_speed = 0.0;  // 右后轮速度(m/s)
    WheelDirection rl_dir = UNKNOWN;  // 左后轮方向
    WheelDirection rr_dir = UNKNOWN;  // 右后轮方向  
    WheelDirection fl_dir = UNKNOWN;
    WheelDirection fr_dir = UNKNOWN;
};

// 3. IMU数据结构体（偏航率+加速度）  
struct ImuData {
    // 偏航率（单位：rad/s，绕z轴）  
    double yaw_rate = 0.0;
    bool has_yaw_rate = false;  // 横摆角速度数据有效性标志  
    double timestamp = 0.0;
    // 加速度（单位：m/s²，车身坐标系）  
    struct Acceleration {
        double x = 0.0;  // x轴加速度  
        double y = 0.0;  // y轴加速度  
    } acc;
    bool has_acc = false;  // 加速度有效性标志  
};
 
// 4. 底盘数据结构体（替代apollo::soc::Chassis）
struct ChassisData {
    double timestamp = 0.0;  // 时间戳
    bool has_yaw_rate = false;
    double yaw_rate = 0.0;   // 横摆角速度(rad/s)
    bool has_imu_acc = false;
    Eigen::Vector3d imu_acc = Eigen::Vector3d::Zero();  // IMU加速度
    bool has_wheel_speed = false;
    WheelSpeedData wheel_speed;  // 轮速数据
    bool has_wheel_counter = false;
    CounterInfoData wheel_counter;  // 轮计数器数据（复用现有结构体）
};
// 5. 定位结果结构体（输出，替代Localization protobuf）  
struct LocalizationOutput {
    double timestamp = 0.0;  // 结果时间戳  

    struct Position {
        double x = 0.0;  // x坐标（单位：米）  
        double y = 0.0;  // y坐标（单位：米）  
    } pos;

    double heading = 0.0;  // 航向角（单位：rad）  
    double speed = 0.0;    // 车速（单位：m/s）  
};


class DeadReckoningCore {
 public:
  // Public deleted functions can give better error msg
  DeadReckoningCore() = default;
  ~DeadReckoningCore() = default;
  
  void RunOnce(
      const std::shared_ptr<const ChassisData>& chassis,
      const std::shared_ptr<LocalizationOutput>& localization);
  


  void Init();
  void ResetState();
  void Stop();

 private:
  void SetLocalState();
  bool StateEstimateEkf();
  void PublishLocalization(
      const std::shared_ptr<Magna::localization::Localization>& localization);
  void LocalInitialize();
  void LocalStateDynamics(Eigen::VectorXd* dx, const Eigen::VectorXd& x);
  void LocalPredict(const double dt);
  void LocalYawCovUpdate();
  void LocalCarSpeedCorrect();
  void LocalStateUpdate();

 private:
  double yaw_counter_;  // 航向角 
  double last_timestamp_ = 0.0; // 上一帧时间戳

  bool is_imu_bias_init_;
  bool is_forward_;
  bool last_is_forward_;
  bool chassis_is_update_;
  bool wheel_speed_is_valid_;
  bool wheel_counter_is_valid_;
  bool enabled_ = false;

  //使用智能指针定义结构体ChassisData的成员变量chassis_received_，用于存储接收到的底盘数据。
  std::shared_ptr<const ChassisData> chassis_received_;
  
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
  DeadReckoningConfig config_;


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
}  // namespace Magna
