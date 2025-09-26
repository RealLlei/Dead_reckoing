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

using Magna::common::Clock;
using Magna::common::math::HeadingToQuaternion;
using Magna::common::math::InterpolateUsingLinearApproximation;
using Magna::common::util::FillHeader;
using Magna::common::util::TransformToMRF;
using Magna::common::util::TransformToVRF;
using Magna::soc::Chassis;
using Magna::soc::WheelSpeed_WheelSpeedType;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace Magna {
namespace dead_reckoning {

void DeadReckoningCore::Init() {

	//使用结构体赋值配置参数，取代从配置文件proto读取
	config_.use_imu = true;          // 是否使用IMU数据（示例值：true）
	config_.wheel_radius = 0.35;        // 轮子半径（示例值：0.35米）
    config_.neta_forward_coef = 19.9;    // 前进时轮速校正系数（示例值）  
    config_.neta_backward_coef = 20.62;   // 后退时轮速校正系数（示例值）  
    config_.rear_wheel_rot_arm = 1.6;    // 后轮旋转臂长（示例值：1.5米）  
	config_.gain_yaw_count = 0.0;      // 偏航角计数器增益（示例值）
	config_.gain_omg_z_count = 0.0;    // 角速度z轴计数器增益（示例值）

	//从配置结构体中获取参数值，赋给类成员变量
	neta_forward_coef_ = config_.neta_forward_coef;
    neta_backward_coef_ = config_.neta_backward_coef;
    rear_wheel_rot_arm_ = config_.rear_wheel_rot_arm;

  // 初始化车速校正器（卡尔曼滤波器参数）
    car_speed_corrector_._len_sw = 0;  // 滑动窗口长度
    car_speed_corrector_._chi_thr = 1.0;  // 卡方检验阈值（用于数据有效性判断）
    car_speed_corrector_._gain_coef.setConstant(12, 1, 1.0);  // 增益系数矩阵（12x1，初始为1）
    car_speed_corrector_._gain_coef(8) = 0.0;  // 第8个增益系数设为0
    car_speed_corrector_._gain_coef(11) = 0.0;  // 第11个增益系数设为0

    // 初始化航向角协方差校正器
    yaw_cov_corrector_._len_sw = 0;
    yaw_cov_corrector_._chi_thr = 1.0;

    // 初始化标志位（未初始化、未就绪状态）
    is_local_init_ = false;

    // 创建底盘数据智能指针
    chassis_received_ = std::make_shared<Chassis>();
    // 初始化状态向量x_（12维零向量）和协方差矩阵P_（12x12单位矩阵）
    x_ = Eigen::VectorXd::Zero(STATE_NUMBER);
    P_ = Eigen::MatrixXd::Identity(STATE_NUMBER, STATE_NUMBER);

    // 初始化轮速数据（前后左右轮速均为0）
    wheel_speed_.fl_wheel_speed = 0.0;
    wheel_speed_.fr_wheel_speed = 0.0;
    wheel_speed_.rl_wheel_speed = 0.0;
    wheel_speed_.rr_wheel_speed = 0.0;

    // 初始化基于计数器的轮速数据（均为0）
    wheel_counter_speed_.fl_wheel_speed = 0.0;
    wheel_counter_speed_.fr_wheel_speed = 0.0;
    wheel_counter_speed_.rl_wheel_speed = 0.0;
    wheel_counter_speed_.rr_wheel_speed = 0.0;

    // 初始化轮速计数器当前值和上一时刻值（均为0）
    wheel_counter_.fl_counter_info = 0.0;
    wheel_counter_.fr_counter_info = 0.0;
    wheel_counter_.rl_counter_info = 0.0;
    wheel_counter_.rr_counter_info = 0.0;

    last_wheel_counter_.fl_counter_info = 0.0;
    last_wheel_counter_.fr_counter_info = 0.0;
    last_wheel_counter_.rl_counter_info = 0.0;
    last_wheel_counter_.rr_counter_info = 0.0;

    // 初始化IMU数据（角速度、线加速度均为0，z轴加速度初始为重力加速度）
    imu_data_.angular_speed(0) = 0.0;
    imu_data_.angular_speed(1) = 0.0;
    imu_data_.angular_speed(2) = 0.0;

    imu_data_.linear_acceleration(0) = 0.0;
    imu_data_.linear_acceleration(1) = 0.0;
    imu_data_.linear_acceleration(2) = 0.0;  // 后续可能被重力加速度覆盖

    // 初始化其他标志位和状态变量
    is_sensor_ready_ = false;  // 传感器未就绪
    is_car_standstill_ = false;  // 车辆未静止
    is_local_init_ = false;  // 本地坐标系未初始化
    chassis_is_update_ = false;  // 底盘数据未更新
    wheel_speed_is_valid_ = false;  // 轮速数据无效
    wheel_counter_is_valid_ = false;  // 轮速计数器数据无效
    car_standstill_counter_ = 0.0;  // 静止计数器为0
    car_standstill_omg_sum_ = Eigen::Vector3d::Zero();  // 静止时角速度总和为0
    is_forward_ = last_is_forward_ = true;  // 初始默认前进
    is_imu_bias_init_ = false;  // IMU偏置未初始化
    move_direction_ = 0;  // 初始移动方向无效
}

void DeadReckoningCore::Stop() {
  if (enabled_) {
    enabled_ = false;
  }
}

void DeadReckoningCore::ResetState() {
  pos_.setZero();//通过Eigen::Vector3d中的setZero()函数将位置向量pos_初始化为零向量（即[0, 0, 0]）。
  q_.setIdentity();//将车辆的姿态重置为初始状态（无旋转），确保后续姿态推算从 “零旋转” 开始。
  yaw_counter_ = 0.0;//将航向角强制设置为 0.0。
}

void DeadReckoningCore::RunOnce(
    const std::shared_ptr<const ChassisData>& chassis,  // 输入改为本地结构体
    const std::shared_ptr<LocalizationOutput>& localization) {
  // add chassis info when  simulating
  chassis_received_ = chassis;
  chassis_is_update_ =
      ((chassis_received_->timestamp - last_msg_time_) > 1.0e-3);
  AINFO << "current_time: " << chassis_received_->timestamp
        << "last_time: " << last_msg_time_
        << "chassis_is_update_falg: " << chassis_is_update_;
 
  if (!StateEstimateEkf()) {
    return;
  }
  PublishLocalization(localization);

  if (chassis_is_update_) {
    last_msg_time_ = chassis_received_->timestamp;
  }
}


bool DeadReckoningCore::StateEstimateEkf() {  
    double dt = chassis_received_->timestamp - last_msg_time_;
    AINFO << "dt: " << dt << " update_flag: " << chassis_is_update_;
/*
    if (!chassis_is_update_) {  // 若底盘数据未更新
        imu_data_.angular_speed.setZero();  // IMU角速度归零
        imu_data_.linear_acceleration = { 0.0, 0.0, CON_g0 };  // IMU线加速度设为重力
        chassis_is_update_ = false;  // 重置更新标志
        wheel_speed_is_valid_ = false;  // 轮速数据无效
        return false;  // 返回失败
    }
 */
  //else {
        AINFO << "chassis_has_yaw_rate: " << chassis_received_->has_yaw_rate;  // 输出底盘是否有横摆角速度
        if (chassis_received_->has_yaw_rate) {  // 若底盘有横摆角速度
            imu_data_.timestamp = chassis_received_->timestamp;  // 更新IMU时间戳为底盘时间
            imu_data_.angular_speed(0) = 0.0;  // IMU角速度x设为0
            imu_data_.angular_speed(1) = 0.0;  // IMU角速度y设为0
            imu_data_.angular_speed(2) = chassis_received_->yaw_rate;  // IMU角速度z设为底盘横摆角速度

            imu_data_.linear_acceleration(0) = chassis_received_->imu_acc.y();  // IMU线加速度x设为底盘IMU的y向加速度
            imu_data_.linear_acceleration(1) = -1.0 * chassis_received_->imu_acc.x();  // IMU线加速度y设为底盘IMU的x向加速度的负值
            imu_data_.linear_acceleration(2) = CON_g0;  // IMU线加速度z设为重力加速度
        }
        else {  // 底盘无横摆角速度
            ADEBUG << "No yaw rate data";  // 调试日志：等待IMU数据
            return false;  // 返回失败
        }
        if (chassis_received_->has_wheel_speed) {  // 若底盘有轮速数据            
            const auto& wheel_speed = chassis_received_->wheel_speed;  // 获取轮速数据
            wheel_speed_.fl_wheel_speed = wheel_speed.fl_speed;  // 左前轮速值
            wheel_speed_.fr_wheel_speed = wheel_speed.fr_speed;  // 右前轮速值
            wheel_speed_.rl_wheel_speed = wheel_speed.rl_speed;  // 左后轮速值
            wheel_speed_.rr_wheel_speed = wheel_speed.rr_speed;  // 右后轮速值

            if ((wheel_speed.rl_dir == soc::WheelSpeed::BACKWARD) &&
                (wheel_speed.rr_dir != soc::WheelSpeed::FORWARD) ||
                (wheel_speed.rl_dir != soc::WheelSpeed::FORWARD) &&
                (wheel_speed.rr_dir == soc::WheelSpeed::BACKWARD))
            {  // 后轮方向判断为倒车
                move_direction_ = -1;  // 运动方向为-1（倒车）
                wheel_speed_.fl_wheel_speed = -1.0 * wheel_speed_.fl_wheel_speed;  // 左前轮速取负
                wheel_speed_.fr_wheel_speed = -1.0 * wheel_speed_.fr_wheel_speed;  // 右前轮速取负
                wheel_speed_.rl_wheel_speed = -1.0 * wheel_speed_.rl_wheel_speed;  // 左后轮速取负
                wheel_speed_.rr_wheel_speed = -1.0 * wheel_speed_.rr_wheel_speed;  // 右后轮速取负
            }
            else if ((wheel_speed.rl_dir == soc::WheelSpeed::FORWARD) &&
                     (wheel_speed.rr_dir != soc::WheelSpeed::BACKWARD) ||
                     (wheel_speed.rl_dir != soc::WheelSpeed::BACKWARD) &&
                     (wheel_speed.rr_dir == soc::WheelSpeed::FORWARD))
            {  // 后轮方向判断为前进
                move_direction_ = 1;  // 运动方向为1（前进）
            }
            else 
            { 
                move_direction_ = 0; 
            }  // 方向不确定，设为0
            wheel_speed_is_valid_ = true;  // 轮速数据有效
        }
        else {  // 底盘无轮速数据
            wheel_speed_is_valid_ = false;  // 轮速数据无效
            ADEBUG << "Waiting wheel speed data...";  // 调试日志：等待轮速数据
            return false;  // 返回失败
        }


        ADEBUG << "StateEstimateEkf wheel_counter" << chassis_received_->has_wheel_counter;  // 调试日志：EKF中底盘是否有轮计数器数据
        if (chassis_received_->has_wheel_counter) {  // 若底盘有轮计数器数据
            wheel_counter_.timestamp = chassis_received_->timestamp;  // 轮计数器时间戳设为底盘时间
            auto wheel_counter = chassis_received_->wheel_counter;  // 获取轮计数器数据
            double forward_counter_coef = 1.0 / neta_forward_coef_;  // 前向计数器系数（1/配置的前向系数）
            double backward_counter_coef = 1.0 / neta_backward_coef_;  // 后向计数器系数（1/配置的后向系数）

            wheel_counter_is_valid_ = wheel_counter.is_wheel_cnt_fr_valid() && 
                                      wheel_counter.is_wheel_cnt_fl_valid() && 
                                      wheel_counter.is_wheel_cnt_rr_valid() && 
                                      wheel_counter.is_wheel_cnt_rl_valid();  // 轮计数器数据有效性（四轮均有效）

            wheel_counter_.fl_counter_info_isvalid = wheel_counter.is_wheel_cnt_fl_valid();  // 左前轮计数器有效性
            wheel_counter_.fr_counter_info_isvalid = wheel_counter.is_wheel_cnt_fr_valid();  // 右前轮计数器有效性
            wheel_counter_.rl_counter_info_isvalid = wheel_counter.is_wheel_cnt_rl_valid();  // 左后轮计数器有效性
            wheel_counter_.rr_counter_info_isvalid = wheel_counter.is_wheel_cnt_rr_valid();  // 右后轮计数器有效性
            wheel_counter_.fl_counter_info = wheel_counter.wheel_counter_fl();  // 左前轮计数器值
            wheel_counter_.fr_counter_info = wheel_counter.wheel_counter_fr();  // 右前轮计数器值
            wheel_counter_.rl_counter_info = wheel_counter.wheel_counter_rl();  // 左后轮计数器值
            wheel_counter_.rr_counter_info = wheel_counter.wheel_counter_rr();  // 右后轮计数器值
            double full_scale = 32767.0;  // 计数器满量程值
            auto lmd = [&full_scale](double a) -> double { 
                if (a > 0.5 * full_scale) 
                { 
                    a -= full_scale; 
                } 
                else if (a < -0.5 * full_scale) 
                { 
                    a += full_scale; 
                } 
                return a; 
            };  // 处理计数器溢出的lambda函数（将值限制在[-16383.5,16383.5]）

            AINFO << "wheel_counter_is_valid_: " << FIXED << SETPRECISION(4) << wheel_counter_is_valid_;  // 输出轮计数器数据有效性
            if (wheel_counter_is_valid_) {  // 若轮计数器数据有效
                if (dt >= 1.0e-3) {  // 时间差>=1ms
                    wheel_counter_speed_.fl_wheel_speed = lmd(fabs(wheel_counter_.fl_counter_info) - fabs(last_wheel_counter_.fl_counter_info)) / dt;  // 左前轮计数器速度（差值/时间，经溢出处理）
                    wheel_counter_speed_.fr_wheel_speed = lmd(fabs(wheel_counter_.fr_counter_info) - fabs(last_wheel_counter_.fr_counter_info)) / dt;  // 右前轮计数器速度
                    wheel_counter_speed_.rl_wheel_speed = lmd(fabs(wheel_counter_.rl_counter_info) - fabs(last_wheel_counter_.rl_counter_info)) / dt;  // 左后轮计数器速度
                    wheel_counter_speed_.rr_wheel_speed = lmd(fabs(wheel_counter_.rr_counter_info) - fabs(last_wheel_counter_.rr_counter_info)) / dt;  // 右后轮计数器速度
                }
                else {  // 时间差<1ms
                    wheel_counter_speed_.fl_wheel_speed = 0.0;  // 左前轮计数器速度设为0
                    wheel_counter_speed_.fr_wheel_speed = 0.0;  // 右前轮计数器速度设为0
                    wheel_counter_speed_.rl_wheel_speed = 0.0;  // 左后轮计数器速度设为0
                    wheel_counter_speed_.rr_wheel_speed = 0.0;  // 右后轮计数器速度设为0
                }
                ADEBUG << "rr_wheel_speed: : " << wheel_counter_speed_.rr_wheel_speed;  // 调试日志：右后轮计数器速度
                Eigen::Vector3d det_pos = Eigen::Vector3d::Zero();  // 位置变化量初始化为0
                det_pos.x() = 0.5 * (wheel_counter_speed_.rl_wheel_speed + wheel_counter_speed_.rr_wheel_speed) * dt;  // 位置x方向变化（后轮平均速度*时间）
                double det_yaw = (wheel_counter_speed_.rr_wheel_speed - wheel_counter_speed_.rl_wheel_speed) * dt;  // 偏航角变化（左右后轮速度差*时间）
                det_pos *= move_direction_;  // 位置变化乘以运动方向（前进/后退）
                det_yaw *= move_direction_;  // 偏航变化乘以运动方向
                double counter_coef = det_pos.x() >= 0.0 ? forward_counter_coef : backward_counter_coef;  // 根据位置变化方向选择前向/后向系数
                det_pos.x() *= counter_coef;  // 位置变化乘以计数器系数
                det_yaw *= counter_coef / rear_wheel_rot_arm_;  // 偏航变化乘以系数（计数器系数/后轮旋转臂长）
                pos_ += q_ * det_pos;  // 更新位置（姿态旋转后累加位置变化）
                yaw_counter_ += det_yaw;  // 累加偏航角变化
                yaw_counter_ = atan2(sin(yaw_counter_), cos(yaw_counter_));  // 归一化偏航角到[-π,π]
            }
            if (chassis_received_->has_steering_percentage()) { steer_.steer_angle = chassis_received_->steering_percentage() * MAX_STEER_ANGLE; }  // 若有转向百分比，计算转向角（百分比*最大转向角）
            if (chassis_received_->has_steering_torque_nm()) { steer_.steer_torque = chassis_received_->steering_torque_nm() * MAX_STEER_ANGLE; }  // 若有转向扭矩，计算转向扭矩（扭矩*最大转向角）
            last_wheel_counter_ = wheel_counter_;  // 更新上一时刻轮计数器数据
        }
    //}

    Eigen::Quaterniond q_yaw_l{ cos(0.5 * yaw_l_), 0.0, 0.0, sin(0.5 * yaw_l_) };  // 偏航角对应的四元数（绕z轴旋转yaw_l_）
    if (!is_sensor_ready_) {  // 若传感器未就绪
        if (wheel_speed_is_valid_ == true) { is_sensor_ready_ = true; }  // 轮速有效则标记传感器就绪
        else { 
            return false;
        }  // 否则返回失败
    }
    if (wheel_counter_is_valid_) {  // 若轮计数器数据有效
        Eigen::VectorXd wheel_counter_speed(4);  // 四轮计数器速度向量
        wheel_counter_speed << wheel_counter_speed_.fl_wheel_speed, 
                                wheel_counter_speed_.fr_wheel_speed, 
                                wheel_counter_speed_.rl_wheel_speed, 
                                wheel_counter_speed_.rr_wheel_speed;  // 赋值四轮速度

        is_car_standstill_ = wheel_counter_speed.norm() < config_.default_standstill_value;  // 车辆静止判断（速度向量模长<阈值）
        Eigen::VectorXd wheel_counter_dif(2);  // 轮速差值向量
        wheel_counter_dif << wheel_counter_speed_.fl_wheel_speed - wheel_counter_speed_.fr_wheel_speed, wheel_counter_speed_.rl_wheel_speed - wheel_counter_speed_.rr_wheel_speed;  // 前后轮速差
        is_car_go_straight_ = fabs(wheel_counter_speed_.rr_wheel_speed) > 10 && wheel_counter_speed.norm() < config_.default_standstill_value;  // 直行判断（右后轮速绝对值>10且速度模长<阈值）
    }
    ADEBUG << "is_car_go_straight_ " << is_car_go_straight_;  // 调试日志：是否直行

    if (!is_local_init_) {  // 若局部状态未初始化
        LocalInitialize();  // 初始化局部状态
        q_.setIdentity();  // 姿态四元数设为单位矩阵
    }
    else {  // 局部状态已初始化
        if (config_.enable_auto_angle_rate_bias) {  // 若启用自动角速度偏置估计
            if (is_car_standstill_) {  // 车辆静止
                car_standstill_counter_ += 1.0;  // 静止计数器加1
                car_standstill_omg_sum_ += imu_data_.angular_speed;  // 累加IMU角速度
                car_standstill_acc_sum_ += imu_data_.linear_acceleration;  // 累加IMU线加速度
                if (car_standstill_counter_ >= FLAGS_auto_angle_rate_bias_time) {  // 静止计数达到阈值
                    gyro_bias_ += 1.0 * (car_standstill_omg_sum_ / car_standstill_counter_ - gyro_bias_);  // 更新陀螺偏置（滑动平均）
                    acc_bias_ += 1.0 * (car_standstill_acc_sum_ / car_standstill_counter_ - acc_bias_);  // 更新加速度偏置（滑动平均）
                    acc_bias_[2] = 0.0;  // z轴加速度偏置设为0
                    car_standstill_counter_ = 0.0;  // 静止计数器归零
                    car_standstill_omg_sum_.setZero();  // 角速度总和归零
                    car_standstill_acc_sum_.setZero();  // 加速度总和归零
                    is_imu_bias_init_ = true;  // IMU偏置初始化完成
                    ADEBUG << "Complete auto angle rate bias estimate: " << gyro_bias_(2) << " acc x " << acc_bias_(0) << " acc y " << acc_bias_(1);  // 输出偏置估计完成日志
                }
            }
            else {  // 车辆非静止
                car_standstill_counter_ = 0.0;  // 静止计数器归零
                car_standstill_omg_sum_.setZero();  // 角速度总和归零
                car_standstill_acc_sum_.setZero();  // 加速度总和归零
            }
        }
        else {  // 不启用自动偏置估计
            is_imu_bias_init_ = true;  // 标记IMU偏置已初始化
            ADEBUG << "Complete angle rate bias set: " << FLAGS_default_angle_rate_bias_value;  // 输出偏置设置完成日志
        }
        if (!is_imu_bias_init_) {  // 若IMU偏置未初始化
            ADEBUG << "Waiting imu bias estimation... counter: " << car_standstill_counter_;  // 调试日志：等待偏置估计
            q_.setIdentity();  // 姿态四元数设为单位矩阵
        }
        Eigen::Quaterniond det_q;  // 姿态变化四元数
        Eigen::Vector3d det_ang = (imu_data_.angular_speed - gyro_bias_) * dt;  // 角速度增量（去偏置后*时间）
        double theta = det_ang.norm();  // 角速度增量模长
        AINFO << "theta: " << FIXED << SETPRECISION(6) << theta;  // 输出角速度增量模长
        det_q.w() = cos(0.5 * theta);  // 四元数w分量（余弦半角）
        det_q.vec() = sin(0.5 * theta) * det_ang.normalized();  // 四元数向量分量（正弦半角*单位向量）
        q_ = q_ * det_q;  // 更新姿态四元数（当前姿态*变化量）
        ADEBUG << "env imu init finished!";  // 调试日志：IMU初始化完成

        LocalPredict(dt);  // 局部位置预测

        if (wheel_speed_is_valid_) {  // 轮速有效
            ADEBUG << "call local_car_speed_correct";  // 调试日志：调用车速校正
            LocalCarSpeedCorrect();  // 校正车速
        }
        static unsigned int counter = 0;  // 静态计数器
        if (++counter % 100 == 0) { LocalYawCovUpdate(); }  // 每100次调用一次偏航协方差更新
    }
    SetLocalState();  // 设置局部状态
    return true;  // 返回成功
 
}
void DeadReckoningCore::SetLocalState() {  // 设置局部状态数据
    Eigen::MatrixXd vec_g0 = Eigen::MatrixXd::Zero(3, 1);  // 重力向量
    vec_g0(2, 0) = CON_g0;  // z轴重力加速度
    local_state_.timestamp = imu_data_.timestamp;  // 局部状态时间戳设为IMU时间
    local_state_.pos_w = pos_;  // 位置赋值
    Eigen::MatrixXd trans_mat = q_.toRotationMatrix().transpose();  // 姿态旋转矩阵的转置
    local_state_.vel_w = trans_mat * x_.segment(0, 3);  // 速度（旋转矩阵转置*状态向量前3维）
    local_state_.acc_w = imu_data_.linear_acceleration - acc_bias_ - x_.segment(3, 3) - trans_mat * vec_g0;  // 加速度（IMU加速度-偏置-状态向量3-5维-旋转后的重力）
    local_state_.q_i_w = q_;  // 姿态四元数赋值
    local_state_.omg_i = imu_data_.angular_speed - gyro_bias_;  // 角速度（IMU角速度-偏置）
    AINFO << "SetLocalState q_: " << FIXED << SETPRECISION(6) << q_.x() << "; " << q_.y() << "; " << q_.z() << "; " << q_.w();  // 输出姿态四元数
}
void DeadReckoningCore::LocalInitialize() {  // 局部状态初始化
    const double vel_sgm = 0.3;  // 速度标准差
    const double acc_sgm = 0.03;  // 加速度标准差
    const double att_sgm = 0.5 * DEG_2_RAD;  // 姿态标准差（0.5度转弧度）
    const double gyro_bias_sgm = 0.5 * DEG_2_RAD;  // 陀螺偏置标准差（0.5度转弧度）
    const double yaw_sgm = 0.5 * DEG_2_RAD;  // 偏航角标准差（0.5度转弧度）
    x_ = Eigen::VectorXd::Zero(STATE_NUMBER);  // 状态向量归零
    P_ = Eigen::MatrixXd::Identity(STATE_NUMBER, STATE_NUMBER);  // 协方差矩阵初始化为单位矩阵
    for (int i = 0; i != 3; ++i) {  // 循环3次（x,y,z轴）
        P_(i, i) = pow(vel_sgm, 2.0);  // 速度协方差（标准差平方）
        P_(3 + i, 3 + i) = pow(acc_sgm, 2.0);  // 加速度协方差
        P_(6 + i, 6 + i) = pow(att_sgm, 2.0);  // 姿态协方差
        P_(9 + i, 9 + i) = pow(gyro_bias_sgm, 2.0);  // 陀螺偏置协方差
    }
    P_(8, 8) = pow(yaw_sgm, 2.0);  // 第8维（偏航相关）协方差
    pos_.setZero(3, 1);  // 位置归零
    Eigen::VectorXd acc = imu_data_.linear_acceleration;  // 获取IMU加速度
    acc.normalize();  // 加速度归一化
    double roll = atan2(acc(1), acc(2));  // 计算横滚角（atan2(acc_y, acc_z)）
    double pitch = -asin(acc(0));  // 计算俯仰角（-asin(acc_x)）
    q_ = (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));  // 姿态四元数（由横滚、俯仰角构造，偏航角0）
    yaw_l_ = 0.0;  // 局部偏航角归零
    gyro_bias_.setZero(3, 1);  // 陀螺偏置归零
    acc_bias_.setZero(3, 1);  // 加速度偏置归零
    is_local_init_ = true;  // 标记局部状态已初始化
}
void DeadReckoningCore::LocalStateDynamics(Eigen::VectorXd* dx, const Eigen::VectorXd& x) {  // 计算状态导数（用于预测）
    *dx = Eigen::VectorXd::Zero(x.rows());  // 导数向量归零
    Eigen::MatrixXd vec_g0 = Eigen::MatrixXd::Zero(3, 1);  // 重力向量
    vec_g0(2, 0) = CON_g0;  // z轴重力加速度
    dx->segment(0, 3) = q_.toRotationMatrix() * (imu_data_.linear_acceleration - acc_bias_ - x.segment(3, 3)) - vec_g0;  // 速度导数（旋转矩阵*(IMU加速度-偏置-状态3-5维) - 重力）
}
void DeadReckoningCore::LocalPredict(const double dt) {  // 局部状态预测
    Eigen::VectorXd dx;  // 状态导数
    LocalStateDynamics(&dx, x_);  // 计算导数
    AINFO << "lat velocity increament: " << dx(0) * dt << " , current lat velocity :" << x_(0) << " next lat vel:" << x_(0) + dx(0) * dt;  // 输出侧向速度增量及当前、下一时刻速度
    AINFO << "long velocity increament: " << dx(1) * dt << " , current long velocity :" << x_(1) << " next long vel:" << x_(1) + dx(1) * dt;  // 输出纵向速度增量及当前、下一时刻速度
    x_ = x_ + dt * dx;  // 状态预测（当前状态+导数*时间）
    Eigen::MatrixXd prc_mat = Eigen::MatrixXd::Zero(x_.rows(), x_.rows());  // 过程噪声协方差矩阵
    const double sgm_vel_prc = 0.3;  // 速度过程噪声标准差
    const double sgm_acc_prc = 0.01;  // 加速度过程噪声标准差
    const double sgm_att_prc = 0.2 * DEG_2_RAD;  // 姿态过程噪声标准差（0.2度转弧度）
    const double sgm_gyro_bias_prc = 0.01 * DEG_2_RAD;  // 陀螺偏置过程噪声标准差（0.01度转弧度）
    for (int i = 0; i != 3; ++i) {  // 循环3次
        prc_mat(i, i) = dt * pow(sgm_vel_prc, 2.0);  // 速度过程噪声（时间*标准差平方）
        prc_mat(3 + i, 3 + i) = dt * pow(sgm_acc_prc, 2.0);  // 加速度过程噪声
        prc_mat(6 + i, 6 + i) = dt * pow(sgm_att_prc, 2.0);  // 姿态过程噪声
        prc_mat(9 + i, 9 + i) = dt * pow(sgm_gyro_bias_prc, 2.0);  // 陀螺偏置过程噪声
    }
    Eigen::MatrixXd rot_mat = q_.toRotationMatrix();  // 姿态旋转矩阵
    Eigen::MatrixXd sys_mat = Eigen::MatrixXd::Zero(x_.rows(), x_.rows());  // 系统矩阵
    sys_mat.block<3, 1>(0, 5) = -rot_mat.col(2);  // 系统矩阵块赋值（3行1列，起始(0,5)）
    sys_mat.block<3, 3>(0, 6) = -2.0 * get_cross_mat(rot_mat * (imu_data_.linear_acceleration - acc_bias_ - x_.segment(3, 3))).block<3, 3>(0, 0);  // 系统矩阵块赋值（3x3，起始(0,6)）
    sys_mat.block<3, 3>(6, 9) = -0.5 * rot_mat;  // 系统矩阵块赋值（3x3，起始(6,9)）
    Eigen::MatrixXd disc_sys_mat = Eigen::MatrixXd::Identity(sys_mat.rows(), sys_mat.cols());  // 离散系统矩阵（初始单位矩阵）
    disc_sys_mat += dt * sys_mat;  // 离散化（欧拉近似）
    P_ = disc_sys_mat * P_ * disc_sys_mat.transpose() + prc_mat;  // 协方差预测（离散系统矩阵*P*转置 + 过程噪声）
}
void DeadReckoningCore::LocalStateUpdate() {  // 局部状态更新
    const double max_gyro_bias = 7.0 * DEG_2_RAD;  // 最大陀螺偏置（7度转弧度）
    VectorXd ru = x_.segment(6, 3);  // 状态向量中6-8维（姿态误差）
    Eigen::Quaterniond quat_error(1.0, ru(0), ru(1), ru(2));  // 姿态误差四元数
    quat_error.normalize();  // 归一化
    q_ = quat_error * q_;  // 更新姿态四元数（误差四元数*当前姿态）
    AINFO << "quat_error: " << FIXED << SETPRECISION(6) << quat_error.x() << "; " << quat_error.y() << "; " << quat_error.z() << "; " << quat_error.w();  // 输出姿态误差四元数
    gyro_bias_ += x_.segment(9, 3);  // 更新陀螺偏置（加上状态向量9-11维）
    for (unsigned int i = 0; i != 3; ++i) {  // 限制陀螺偏置在[-max, max]
        gyro_bias_(i) = std::min(std::max(gyro_bias_(i), -max_gyro_bias), max_gyro_bias);
    }
    x_.block<3, 1>(6, 0) = VectorXd::Zero(3, 1);  // 状态向量6-8维归零
    x_.block<3, 1>(9, 0) = VectorXd::Zero(3, 1);  // 状态向量9-11维归零
}
void DeadReckoningCore::LocalYawCovUpdate() {  // 偏航协方差更新
    const double sgm_yaw = 3.0 * DEG_2_RAD;  // 偏航观测噪声标准差（3度转弧度）
    Eigen::MatrixXd r_mat = Eigen::MatrixXd::Zero(1, 1);  // 观测噪声协方差矩阵
    r_mat(0, 0) = pow(sgm_yaw, 2.0);  // 观测噪声方差（标准差平方）
    Eigen::MatrixXd trans_mat = q_.toRotationMatrix();  // 姿态旋转矩阵
    Eigen::VectorXd res_error = Eigen::VectorXd::Zero(1);  // 残差（初始化0）
    Eigen::MatrixXd h_mat = Eigen::MatrixXd::Zero(1, x_.rows());  // 观测矩阵
    h_mat(0, 8) = 1.0;  // 观测矩阵第0行第8列设为1（观测偏航相关状态）
    yaw_cov_corrector_.AdaptiveCorrect(&x_, &P_, res_error, h_mat, r_mat);  // 自适应校正（更新状态和协方差）
    LocalStateUpdate();  // 更新局部状态
}
void DeadReckoningCore::LocalCarSpeedCorrect() {  // 车速校正
    Eigen::Vector3d v_car = Eigen::Vector3d::Zero(3);  // 车辆速度向量
    v_car(0) = (wheel_speed_.rl_wheel_speed + wheel_speed_.rr_wheel_speed) * 0.5;  // 车辆x方向速度（后轮平均速度）
    const double sgm_vel = 0.03 + 0.01 * v_car.norm();  // 速度观测噪声标准差（与速度相关）
    Eigen::MatrixXd r_mat = Eigen::MatrixXd::Zero(3, 3);  // 观测噪声协方差矩阵
    r_mat(0, 0) = pow(sgm_vel, 2.0);  // x方向方差
    r_mat(1, 1) = r_mat(2, 2) = r_mat(0, 0) * 0.05;  // y、z方向方差（x方向的5%）
    Eigen::MatrixXd trans_mat = q_.toRotationMatrix();  // 姿态旋转矩阵
    Eigen::VectorXd res_error = v_car - trans_mat.transpose() * x_.segment(0, 3);  // 残差（观测速度 - 预测速度）
    ADEBUG << "current_v_car," << v_car(0) << " vel_err_0: " << res_error(0) << " vel_err_1: " << res_error(1);  // 调试日志：当前速度及残差
    Eigen::MatrixXd h_mat = Eigen::MatrixXd::Zero(3, x_.rows());  // 观测矩阵
    h_mat.block<3, 3>(0, 0) = trans_mat.transpose();  // 观测矩阵块（3x3，起始(0,0)）
    h_mat.block<3, 3>(0, 6) = 2.0 * get_cross_mat(trans_mat.transpose() * x_.segment(0, 3));  // 观测矩阵块（3x3，起始(0,6)）
    car_speed_corrector_.AdaptiveCorrect(&x_, &P_, res_error, h_mat, r_mat);  // 自适应校正（更新状态和协方差）
    AINFO << "AdaptiveCorrect speed, x(0) :" << x_(0) << ", x(1): " << x_(1);  // 输出校正后的速度状态
    LocalStateUpdate();  // 更新局部状态
}
Eigen::MatrixX3d get_cross_mat(const Eigen::Vector3d& w) {  // 生成向量的叉乘矩阵
    double wx = w(0, 0);  // x分量
    double wy = w(1, 0);  // y分量
    double wz = w(2, 0);  // z分量
    Eigen::MatrixXd a_mat(3, 3);  // 3x3矩阵
    a_mat << 0.0, -wz, wy, wz, 0.0, -wx, -wy, wx, 0.0;  // 叉乘矩阵（[w]×）
    return a_mat;  // 返回矩阵
}
void DeadReckoningCore::PublishLocalization(const std::shared_ptr<Magna::localization::Localization>& localization) {  // 发布定位信息
    FillHeader("DeadReckoning", localization.get());  // 填充消息头
    localization->set_measurement_time(chassis_received_->header().publish_stamp());  // 设置测量时间为底盘时间
    auto* pose = localization->mutable_pose();  // 获取姿态指针
    Eigen::Vector3d euler = Quaternion2EulerZyx(local_state_.q_i_w);  // 四元数转欧拉角（ZYX顺序）
    if (euler(2) < -M_PI) { euler(2) = euler(2) + 2 * M_PI; }
    else if (euler(2) > M_PI) { euler(2) = euler(2) - 2 * M_PI; }  // 归一化偏航角到[-π,π]
    double heading_angle_pub = euler(2) + 0.5 * M_PI;  // 计算航向角（偏航角+90度）
    if (heading_angle_pub > M_PI) { heading_angle_pub = heading_angle_pub - 2 * M_PI; }
    else if (heading_angle_pub < -M_PI) { heading_angle_pub = heading_angle_pub + 2 * M_PI; }  // 归一化航向角到[-π,π]
    pose->mutable_position()->set_x(-1.0 * local_state_.pos_w(1));  // 位置x设为局部状态y的负值
    pose->mutable_position()->set_y(local_state_.pos_w(0));  // 位置y设为局部状态x
    pose->mutable_position()->set_z(local_state_.pos_w(2));  // 位置z设为局部状态z
    pose->set_heading(heading_angle_pub);  // 设置航向角
    Eigen::Quaterniond q_rot = (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())) * q_;  // 姿态旋转（此处无旋转）
    pose->mutable_quaternion()->set_w(q_rot.w());  // 四元数w
    pose->mutable_quaternion()->set_x(q_rot.x());  // 四元数x
    pose->mutable_quaternion()->set_y(q_rot.y());  // 四元数y
    pose->mutable_quaternion()->set_z(q_rot.z());  // 四元数z
    pose->mutable_linear_velocity_vrf()->set_x(-1.0 * local_state_.vel_w(1));  // 速度VRF x设为局部速度y的负值
    pose->mutable_linear_velocity_vrf()->set_y(local_state_.vel_w(0));  // 速度VRF y设为局部速度x
    pose->mutable_linear_velocity_vrf()->set_z(local_state_.vel_w(2));  // 速度VRF z设为局部速度z
    pose->mutable_linear_acceleration_vrf()->set_x(-1.0 * local_state_.acc_w(1));  // 加速度VRF x设为局部加速度y的负值
    pose->mutable_linear_acceleration_vrf()->set_y(local_state_.acc_w(0));  // 加速度VRF y设为局部加速度x
    pose->mutable_linear_acceleration_vrf()->set_z(local_state_.acc_w(2) - CON_g0);  // 加速度VRF z设为局部加速度z减重力
    pose->mutable_angular_velocity_vrf()->set_x(-1.0 * local_state_.omg_i(1));  // 角速度VRF x设为局部角速度y的负值
    pose->mutable_angular_velocity_vrf()->set_y(local_state_.omg_i(0));  // 角速度VRF y设为局部角速度x
    pose->mutable_angular_velocity_vrf()->set_z(local_state_.omg_i(2));  // 角速度VRF z设为局部角速度z
    Eigen::MatrixXd trans_mat = q_.toRotationMatrix();  // 姿态旋转矩阵
    TransformToMRF(pose->linear_velocity_vrf(), pose->quaternion(), pose->mutable_linear_velocity());  // 转换速度到MRF坐标系
    TransformToMRF(pose->linear_acceleration_vrf(), pose->quaternion(), pose->mutable_linear_acceleration());  // 转换加速度到MRF坐标系
    TransformToMRF(pose->angular_velocity_vrf(), pose->quaternion(), pose->mutable_angular_velocity());  // 转换角速度到MRF坐标系
    pose->mutable_euler_angles()->set_x(euler(0));  // 欧拉角x（横滚）
    pose->mutable_euler_angles()->set_y(euler(1));  // 欧拉角y（俯仰）
    pose->mutable_euler_angles()->set_z(euler(2));  // 欧拉角z（偏航）
    localization->mutable_pose()->mutable_pos_utm_01()->CopyFrom(pose->position());  // 复制位置到utm01
    localization->mutable_pose()->mutable_pos_utm_02()->CopyFrom(pose->position());  // 复制位置到utm02
    localization->mutable_pose()->set_utm_zone_01(FLAGS_local_utm_zone_id);  // 设置utm01区域
    localization->mutable_pose()->set_utm_zone_02(FLAGS_local_utm_zone_id);  // 设置utm02区域
    localization->mutable_pose()->set_using_utm_zone(FLAGS_local_utm_zone_id);  // 设置使用的utm区域
    ADEBUG << "euler_angles: " << FIXED << SETPRECISION(6) << euler(0) << ";, " << euler(1) << ";, " << euler(2);  // 调试日志：欧拉角
    ADEBUG << "point x," << FIXED << SETPRECISION(3) << pose->position().x() << ",y," << pose->position().y() << ",heading," << pose->heading() << " ,v_x," << pose->linear_velocity().x() << ", v_y: " << pose->linear_velocity().y() << ", a_x: " << pose->linear_acceleration().x() << ", a_y: " << pose->linear_acceleration().y();  // 调试日志：位置、航向、速度、加速度
    std::vector<double> temp(std::vector<double>(21, 0.0));  // 日志数据向量
    temp[0] = chassis_received_->header().publish_stamp();  // 时间戳
    temp[1] = pose->position().x();  // x位置
    temp[2] = pose->position().y();  // y位置
    temp[3] = pose->position().z();  // z位置
    temp[4] = pose->linear_velocity().x();  // x速度
    temp[5] = pose->linear_velocity().y();  // y速度
    temp[6] = pose->linear_velocity().z();  // z速度
    temp[7] = pose->linear_acceleration().x();  // x加速度
    temp[8] = pose->linear_acceleration().y();  // y加速度
    temp[9] = pose->linear_acceleration().z();  // z加速度
    temp[12] = pose->heading();  // 航向角
    temp[13] = pose->angular_velocity().x();  // x角速度
    temp[14] = pose->angular_velocity().y();  // y角速度
    temp[15] = pose->angular_velocity().z();  // z角速度
    temp[16] = pose->quaternion().x();  // 四元数x
    temp[17] = pose->quaternion().y();  // 四元数y
    temp[18] = pose->quaternion().z();  // 四元数z
    temp[18] = pose->quaternion().w();  // 四元数w（覆盖上一个，可能笔误）
    temp[19] = wheel_speed_.rr_wheel_speed;  // 右后轮速
    temp[20] = wheel_counter_speed_.rr_wheel_speed;  // 右后轮计数器速度
    std::ofstream ofs("/home/liubei/dr_output_log.txt", std::ios::app);  // 打开日志文件（追加模式）
    for (int ii = 0; ii < 21; ii++) { ofs << std::setprecision(16) << temp[ii] << '\t'; }  // 写入日志数据
    ofs << std::endl;  // 换行
    ofs.close();  // 关闭文件
}
}  // namespace dead_reckoning
}  // namespace Magna
