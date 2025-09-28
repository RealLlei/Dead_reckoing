/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description: ideal_steer_model.cc
 */
#include "planning/common/game/vehicle_model/ideal_steer_model.h"
#include <cmath>
#include "common/math/math_utils.h"

namespace TL::planning::game_common {
using common::math::NormalizeAngle;

IdealSteerModel::IdealSteerModel(
    const double wheelbase_len, const double max_lon_acc,
    const double max_lon_dec, const double max_lon_acc_jerk,
    const double max_lon_dec_jerk, const double max_lat_acc,
    const double max_lat_jerk, const double max_steering_angle,
    const double max_steer_rate, const double max_curvature)
    : wheelbase_len_(wheelbase_len),
      max_lon_acc_(max_lon_acc),
      max_lon_dec_(max_lon_dec),
      max_lon_acc_jerk_(max_lon_acc_jerk),
      max_lon_dec_jerk_(max_lon_dec_jerk),
      max_lat_acc_(max_lat_acc),
      max_lat_jerk_(max_lat_jerk),
      max_steering_angle_(max_steering_angle),
      max_steer_rate_(max_steer_rate),
      max_curvature_(max_curvature) {
  UpdateInternalState();
}

void IdealSteerModel::Step(const double dt) {
  state_.steer = atan(state_.kappa * wheelbase_len_);
  UpdateInternalState();
  control_.velocity = fmax(0.0, control_.velocity);
  control_.steer =
      fmin(fmax(control_.steer, -max_steering_angle_), max_steering_angle_);
  TruncateControl(dt);
  desired_lon_acc_ = (control_.velocity - state_.velocity) / dt;
  desired_steer_rate_ = NormalizeAngle(control_.steer - state_.steer) / dt;
  Linear(internal_state_, dt, &internal_state_);
  state_.position.set_x(internal_state_[0]);
  state_.position.set_y(internal_state_[1]);
  state_.angle = NormalizeAngle(internal_state_[2]);
  state_.velocity = internal_state_[3];
  state_.steer = NormalizeAngle(internal_state_[4]);
  state_.kappa = tan(state_.steer) * 1.0 / wheelbase_len_;
  state_.acceleration = desired_lon_acc_;
  UpdateInternalState();
}

void IdealSteerModel::Linear(const InternalState& x, const double dt,
                             InternalState* x_out) const {
  State cur_state;
  cur_state.position.set_x(x[0]);
  cur_state.position.set_y(x[1]);
  cur_state.angle = x[2];
  cur_state.velocity = x[3];
  cur_state.steer = x[4];

  x_out->at(0) = x.at(0) + cos(cur_state.angle) * cur_state.velocity * dt;
  x_out->at(1) = x.at(1) + sin(cur_state.angle) * cur_state.velocity * dt;
  x_out->at(2) =
      x.at(2) + tan(cur_state.steer) * cur_state.velocity / wheelbase_len_ * dt;
  x_out->at(3) = x.at(3) + desired_lon_acc_ * dt;
  x_out->at(4) = x.at(4) + desired_steer_rate_ * dt;
}

void IdealSteerModel::set_control(const Control& control) {
  control_ = control;
}

const State& IdealSteerModel::state() const {
  return state_;
}

void IdealSteerModel::set_state(const State& state) {
  state_ = state;
  UpdateInternalState();
}

void IdealSteerModel::TruncateControl(const double dt) {
  desired_lon_acc_ = (control_.velocity - state_.velocity) / dt;
  double desired_lon_jerk = (desired_lon_acc_ - state_.acceleration) / dt;
  desired_lon_jerk =
      fmin(fmax(desired_lon_jerk, -max_lon_dec_jerk_), max_lon_acc_jerk_);
  desired_lon_acc_ = desired_lon_jerk * dt + state_.acceleration;
  desired_lon_acc_ = fmin(fmax(desired_lon_acc_, -max_lon_dec_), max_lon_acc_);
  control_.velocity = fmax(state_.velocity + desired_lon_acc_ * dt, 0.0);

  desired_lat_acc_ =
      pow(control_.velocity, 2) * (tan(control_.steer) / wheelbase_len_);
  double lat_acc_ori = pow(state_.velocity, 2) * state_.kappa;
  double lat_jerk_desired = (desired_lat_acc_ - lat_acc_ori) / dt;
  lat_jerk_desired =
      fmin(fmax(lat_jerk_desired, -max_lat_jerk_), max_lat_jerk_);
  desired_lat_acc_ = lat_jerk_desired * dt + lat_acc_ori;
  desired_lat_acc_ = fmin(fmax(desired_lat_acc_, -max_lat_acc_), max_lat_acc_);
  constexpr double kBigEPS = 0.01;
  control_.steer = atan(desired_lat_acc_ * wheelbase_len_ /
                        fmax(pow(control_.velocity, 2), kBigEPS));
  desired_steer_rate_ = NormalizeAngle(control_.steer - state_.steer) / dt;
  desired_steer_rate_ =
      fmin(fmax(desired_steer_rate_, -max_steer_rate_), max_steer_rate_);
  control_.steer = NormalizeAngle(state_.steer + desired_steer_rate_ * dt);
}

void IdealSteerModel::UpdateInternalState() {
  internal_state_[0] = state_.position.x();
  internal_state_[1] = state_.position.y();
  internal_state_[2] = state_.angle;
  internal_state_[3] = state_.velocity;
  internal_state_[4] = state_.steer;
}
}  // namespace TL::planning::game_common
