/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  intelligent_velocity_controller.cc
 */

#include "planning/common/game/controllers/intelligent_velocity_controller.h"
#include <cmath>
#include "planning/common/game/vehicle_model/idm_model.h"

namespace TL::planning::game_common {

using game_common::IntelligentDriverModel;

void IntelligentVelocityControl::CalculateDesiredVelocity(
    const IDM::Param& param, const double s, const double s_front,
    const double v, const double v_front, const double dt,
    double* velocity_at_dt) {
  IntelligentDriverModel model(param);
  IDM::State state;
  state.s = s;
  state.v = std::fmax(0.0, v);
  state.s_front = s_front;
  state.v_front = v_front;
  model.set_state(state);
  model.Step(dt);

  const auto& desired_state = model.state();
  *velocity_at_dt = std::fmax(0.0, desired_state.v);
}
}  // namespace TL::planning::game_common
