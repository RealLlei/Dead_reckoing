/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  idm.h
 */

#pragma once

#include "common/math/cartesian_frenet_conversion.h"

namespace TL::planning::game_common {
using TL::common::math::CartesianFrenetConverter;

// using apollo::common::math;

class IDM {
 public:
  struct State {
    double s = 0.0;
    double v = 0.0;
    double s_front = 0.0;
    double v_front = 0.0;
    State() = default;

    State(double s_, double v_, double s_front_, double v_front_)
        : s(s_), v(v_), s_front(s_front_), v_front(v_front_) {}
  };

  struct Param {
    double desired_velocity = 0.0;
    double vehicle_length = 5.0;
    double minimum_spacing = 2.0;
    double desired_ahead_time = 1.0;
    double acceleration = 2.0;
    double comfortable_braking_decelerate = 3.0;
    double hard_breaking_decelerate = 5.0;
    int exponent = 4;
  };

  /**
   * @brief GetIdmDesiredAcceleration.
   * @param param
   * @param state
   * @param acc
   * @return true: success.
   */
  static bool GetIdmDesiredAcceleration(const Param& param, const State& state,
                                        double* acc);

  /**
   * @brief GetIIdmDesiredAcceleration.
   * @param param
   * @param state
   * @param acc
   * @return true: success.
   */
  static bool GetIIdmDesiredAcceleration(const Param& param, const State& state,
                                         double* acc);

  /**
   * @brief GetAccDesiredAcceleration.
   * @param param
   * @param state
   * @param acc
   * @return true: success.
   */
  static bool GetAccDesiredAcceleration(const Param& param, const State& state,
                                        double* acc);
  State state_;
  Param param_;
};
}  // namespace TL::planning::game_common
