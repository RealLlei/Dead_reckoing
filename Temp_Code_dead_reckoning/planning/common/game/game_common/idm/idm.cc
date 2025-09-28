/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  idm.cc
 */

#include "planning/common/game/game_common/idm/idm.h"
#include <cmath>
#include "common/util/util.h"

namespace TL::planning::game_common {
using common::util::IsFloatEqual;

bool IDM::GetIdmDesiredAcceleration(const Param& param, const State& state,
                                    double* const acc) {
  if (acc == nullptr) {
    return false;
  }
  if (IsFloatEqual(param.acceleration, 0.0) ||
      IsFloatEqual(param.comfortable_braking_decelerate, 0.0) ||
      IsFloatEqual(param.desired_velocity, 0.0) || param.acceleration < 0.0 ||
      param.comfortable_braking_decelerate < 0.0 ||
      param.desired_velocity < 0.0) {
    return false;
  }

  const double s_star =
      param.minimum_spacing +
      fmax(0.0, state.v * param.desired_ahead_time +
                    state.v * (state.v - state.v_front) /
                        (2.0 * sqrt(param.acceleration *
                                    param.comfortable_braking_decelerate)));
  const double s_alpha =
      fmax(0.0, state.s_front - state.s - param.vehicle_length);
  if (IsFloatEqual(s_alpha, 0.0)) {
    return false;
  }
  *acc = param.acceleration *
         (1.0 - pow(state.v / param.desired_velocity, param.exponent) -
          pow(s_star / s_alpha, 2));
  return true;
}

bool IDM::GetIIdmDesiredAcceleration(const Param& param, const State& state,
                                     double* const acc) {
  // The Improved IntelligentDriverModel (IIDM) tries to address two
  // deficiencies of the original IDM model:
  // 1) If the actual speed exceeds the desired speed (e.g., after entering a
  // zone with a reduced speed limit), the deceleration is unrealistically
  // large, particularly for large values of the acceleration exponent δ.
  // 2) Near the desired speed v0, the steady-state gap becomes much
  // greater than s∗(v, 0) = s0 + vT so that the model parameter T loses its
  // meaning as the desired time gap. This means that a platoon of identical
  // drivers and vehicles disperses much more than observed. Moreover, not all
  // cars will reach the desired speed
  if (acc == nullptr) {
    return false;
  }

  if (IsFloatEqual(param.acceleration, 0.0) ||
      IsFloatEqual(param.comfortable_braking_decelerate, 0.0) ||
      IsFloatEqual(param.desired_velocity, 0.0) || param.acceleration < 0.0 ||
      param.comfortable_braking_decelerate < 0.0 ||
      param.desired_velocity < 0.0) {
    return false;
  }

  const double a_free =
      state.v <= param.desired_velocity
          ? param.acceleration *
                (1.0 - pow(state.v / param.desired_velocity, param.exponent))
          : -param.comfortable_braking_decelerate *
                (1.0 - pow(param.desired_velocity / state.v,
                           param.acceleration * param.exponent /
                               param.comfortable_braking_decelerate));
  const double s_alpha =
      fmax(0.0, state.s_front - state.s - param.vehicle_length);
  if (IsFloatEqual(s_alpha, 0.0)) {
    return false;
  }
  // s_intention >= 1.0 应该拉开距离到期望距离
  // s_intention < 1.0 应该缩小距离到期望距离
  const double s_intention =
      (param.minimum_spacing +
       fmax(0.0, state.v * param.desired_ahead_time +
                     state.v * (state.v - state.v_front) /
                         (2.0 * sqrt(param.acceleration *
                                     param.comfortable_braking_decelerate)))) /
      s_alpha;
  double a_out =
      state.v < param.desired_velocity
          ? (s_intention >= 1.0
                 ? param.acceleration * (1 - pow(s_intention, 2))
                 : a_free * (1.0 - pow(s_intention,
                                       2.0 * param.acceleration / a_free)))
          : (s_intention >= 1.0
                 ? a_free + param.acceleration * (1 - pow(s_intention, 2))
                 : a_free);
  a_out =
      fmax(fmin(param.acceleration, a_out), -param.hard_breaking_decelerate);
  *acc = a_out;

  return true;
}

bool IDM::GetAccDesiredAcceleration(const Param& param, const State& state,
                                    double* acc) {
  double acc_iidm = 0.0;
  if (!GetIIdmDesiredAcceleration(param, state, &acc_iidm)) {
    return false;
  }
  const double ds = fmax(0.0, state.s_front - state.s);
  const double acc_cah =
      (state.v * state.v * -param.comfortable_braking_decelerate) /
      (state.v_front * state.v_front -
       2 * ds * -param.comfortable_braking_decelerate);
  const double coolness = 0.99;
  if (acc_iidm >= acc_cah) {
    *acc = acc_iidm;
  } else {
    *acc = (1 - coolness) * acc_iidm +
           coolness *
               (acc_cah - param.comfortable_braking_decelerate *
                              std::tanh((acc_iidm - acc_cah) /
                                        -param.comfortable_braking_decelerate));
  }

  return true;
}
}  // namespace TL::planning::game_common
