/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description: ideal_steer_model.h
 */
#pragma once

#include "planning/common/game/game_common/semantics/semantics.h"

namespace TL::planning::game_common {
using game_common::State;

class IdealSteerModel {
  using InternalState = std::array<double, 5>;

 public:
  struct Control {
    double steer = 0.0;
    double velocity = 0.0;

    Control() = default;

    Control(const double steer, const double velocity)
        : steer(steer), velocity(velocity) {}
  };

  IdealSteerModel(double wheelbase_len, double max_lon_acc, double max_lon_dec,
                  double max_lon_acc_jerk, double max_lon_dec_jerk,
                  double max_lat_acc, double max_lat_jerk,
                  double max_steering_angle, double max_steer_rate,
                  double max_curvature);
  IdealSteerModel() = default;

  /**
   * @brief set_state.
   * @param state
   */
  void set_state(const State& state);

  /**
   * @brief set_control.
   * @param control
   */
  void set_control(const Control& control);

  /**
   * @brief state.
   */
  const State& state() const;

  /**
   * @brief TruncateControl.
   * @param dt
   */
  void TruncateControl(double dt);

  /**
   * @brief Step.
   * @param dt
   */
  void Step(double dt);

  void Linear(const InternalState& x, double dt, InternalState* x_out) const;

 private:
  /**
   * @brief UpdateInternalState.
   */
  void UpdateInternalState();

  double wheelbase_len_ = 0.0;
  double max_lon_acc_ = 0.0;
  double max_lon_dec_ = 0.0;
  double max_lon_acc_jerk_ = 0.0;
  double max_lon_dec_jerk_ = 0.0;
  double max_lat_acc_ = 0.0;
  double max_lat_jerk_ = 0.0;
  double max_steering_angle_ = 0.0;
  double max_steer_rate_ = 0.0;
  double max_curvature_ = 0.0;
  State state_;
  InternalState internal_state_ = {};

  double desired_lon_acc_ = 0.0;
  double desired_steer_rate_ = 0.0;
  double desired_lat_acc_ = 0.0;

  Control control_;
};
};  // namespace TL::planning::game_common
