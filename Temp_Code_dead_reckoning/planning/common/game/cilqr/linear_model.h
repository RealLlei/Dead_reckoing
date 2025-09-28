/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description: linear_model.h
 */
#pragma once
#include <Eigen/Dense>
#include "common/math/math_utils.h"

namespace TL::planning::game_common {
struct ModelParam {
  // EP40
  double wheelbase = 2.96;
  double steer_min = -0.25;  // 方向盘转角
  double steer_max = 0.25;
  double accel_min = -4.95;
  double accel_max = 2.5;
  double steer_ratio = 15.17;  // 传动比
  double max_speed = 30.0;
  double Ts = 0.2;
};
class LinearModel {
 private:
  ModelParam model_param_ = {};

 public:
  LinearModel() = default;

  explicit LinearModel(const ModelParam& model_param)
      : model_param_(model_param) {}

  Eigen::Vector4d ForwardSimulate(const Eigen::Vector4d& state,
                                  const Eigen::Vector2d& control) const {
    const double clipped_accel =
        std::clamp(control(0), model_param_.accel_min, model_param_.accel_max);
    const double clipped_yaw_rate = std::clamp(
        control(1),
        state(2) * std::tan(model_param_.steer_min / model_param_.steer_ratio) /
            model_param_.wheelbase,
        state(2) * std::tan(model_param_.steer_max / model_param_.steer_ratio) /
            model_param_.wheelbase);

    Eigen::Vector4d next_state;
    next_state(0) =
        state(0) + std::cos(state(3)) * (state(2) * model_param_.Ts +
                                         0.5 * clipped_accel * model_param_.Ts *
                                             model_param_.Ts);
    next_state(1) =
        state(1) + std::sin(state(3)) * (state(2) * model_param_.Ts +
                                         0.5 * clipped_accel * model_param_.Ts *
                                             model_param_.Ts);
    next_state(2) = std::clamp(state(2) + clipped_accel * model_param_.Ts, 0.0,
                               model_param_.max_speed);
    next_state(3) = common::math::NormalizeAngle(
        (state(3) + clipped_yaw_rate * model_param_.Ts));

    return next_state;
  }

  Eigen::Matrix4d GetAMatrix(double velocity, double theta,
                             double acceleration) const {
    Eigen::Matrix4d A;
    A(0, 0) = 1.0;
    A(0, 1) = 0.0;
    A(0, 2) = std::cos(theta) * model_param_.Ts;
    A(0, 3) = -(velocity * model_param_.Ts +
                (0.5 * acceleration * model_param_.Ts * model_param_.Ts) *
                    std::sin(theta));
    A(1, 0) = 0.0;
    A(1, 1) = 1.0;
    A(1, 2) = std::sin(theta) * model_param_.Ts;
    A(1, 3) = velocity * model_param_.Ts +
              (0.5 * acceleration * model_param_.Ts * model_param_.Ts) *
                  std::cos(theta);
    A(2, 0) = 0.0;
    A(2, 1) = 0.0;
    A(2, 2) = 1.0;
    A(2, 3) = 0.0;
    A(3, 0) = 0.0;
    A(3, 1) = 0.0;
    A(3, 2) = 0.0;
    A(3, 3) = 1.0;
    return A;
  }

  Eigen::MatrixXd GetBMatrix(double theta) const {
    Eigen::MatrixXd B(4, 2);
    B(0, 0) = 0.5 * model_param_.Ts * model_param_.Ts * std::cos(theta);
    B(0, 1) = 0.0;
    B(1, 0) = 0.5 * model_param_.Ts * model_param_.Ts * std::sin(theta);
    B(1, 1) = 0.0;
    B(2, 0) = model_param_.Ts;
    B(2, 1) = 0.0;
    B(3, 0) = 0.0;
    B(3, 1) = model_param_.Ts;
    return B;
  }
};
}  // namespace TL::planning::game_common
