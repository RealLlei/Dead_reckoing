/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  kinematic_model.cc
 */

#include "planning/open_space/ilqr_smoother/kinematic_model.h"
#include "common/math/double_type.h"

void KinematicModel::Simulate(
    const Eigen::Ref<const Eigen::VectorXd>& cur_state,
    Eigen::Ref<Eigen::VectorXd> cur_control,
    Eigen::Ref<Eigen::VectorXd> next_state) const {
  if (cur_state.size() != param_.state_dim ||
      cur_control.size() != param_.control_dim ||
      next_state.size() != cur_state.size()) {
    return;
  }
  // use cur_state and cur_control to compute next_state
  if (cur_control[0] > param_.max_acc) {
    cur_control[0] = param_.max_acc;
  } else if (cur_control[0] < param_.min_acc) {
    cur_control[0] = param_.min_acc;
  }
  auto steer = [&](double spd, double sterr_limit) {
    return TL::common::math::double_type::IsZero(param_.wheelbase)
               ? sterr_limit
               : spd * std::tan(sterr_limit) / param_.wheelbase;
  };
  double steer_limit_upper = steer(cur_state[2], param_.max_w);
  double steer_limit_lower = steer(cur_state[2], param_.min_w);
  if (steer_limit_lower > steer_limit_upper) {
    std::swap(steer_limit_upper, steer_limit_lower);
  }

  if (cur_control[1] > steer_limit_upper) {
    cur_control[1] = steer_limit_upper;
  } else if (cur_control[1] < steer_limit_lower) {
    cur_control[1] = steer_limit_lower;
  }
  double acc = cur_control[0];
  double w = cur_control[1];
  const double dis =
      cur_state[2] * param_.dt + acc * param_.dt * param_.dt * 0.5;
  next_state[0] = cur_state[0] + std::cos(cur_state[3]) * dis;
  next_state[1] = cur_state[1] + std::sin(cur_state[3]) * dis;
  next_state[2] = cur_state[2] + acc * param_.dt;
  next_state[3] = cur_state[3] + w * param_.dt;
}

void KinematicModel::StateJacobian(
    const double theta, const double spd, const double acc,
    Eigen::Ref<Eigen::MatrixXd> state_first_derivative) const {
  if (state_first_derivative.rows() != state_first_derivative.cols() ||
      state_first_derivative.rows() != param_.state_dim) {
    return;
  }
  const double dis = spd * param_.dt + 0.5 * acc * param_.dt * param_.dt;
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);
  state_first_derivative(0, 0) = 1.0;
  state_first_derivative(0, 2) = cos_theta * param_.dt;
  state_first_derivative(0, 3) = -dis * sin_theta;

  state_first_derivative(1, 1) = 1.0;
  state_first_derivative(1, 2) = sin_theta * param_.dt;
  state_first_derivative(1, 3) = dis * cos_theta;

  state_first_derivative(2, 2) = 1.0;

  state_first_derivative(3, 3) = 1.0;
}

void KinematicModel::ControlJacobian(
    const double theta,
    Eigen::Ref<Eigen::MatrixXd> control_first_derivative) const {
  if (control_first_derivative.rows() != param_.state_dim ||
      control_first_derivative.cols() != param_.control_dim) {
    return;
  }
  control_first_derivative(0, 0) = 0.5 * cos(theta) * param_.dt * param_.dt;
  control_first_derivative(1, 0) = 0.5 * sin(theta) * param_.dt * param_.dt;
  control_first_derivative(2, 0) = param_.dt;
  control_first_derivative(3, 1) = param_.dt;
}
