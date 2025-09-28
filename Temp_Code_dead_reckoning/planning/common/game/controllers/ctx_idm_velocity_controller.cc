/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  ctx_idm_velocity_controller.cc
 */

#include "planning/common/game/controllers/ctx_idm_velocity_controller.h"
#include <algorithm>

namespace TL::planning::game_common {

bool ContextIntelligentVelocityControl::CalculateDesiredVelocity(
    const IDM::Param& idm_param,
    const ContextIntelligentDriverModel::CtxParam& ctx_param, const double s,
    const double s_front, const double s_target, const double v,
    const double v_front, const double v_target, const double dt,
    double* velocity_at_dt) {
  ContextIntelligentDriverModel model(idm_param, ctx_param);
  ContextIntelligentDriverModel::CtxIdmState state;
  state.s = s;
  state.s_front = s_front;
  state.s_target = s_target;
  state.v = std::max(0.0, v);
  state.v_front = v_front;
  state.v_target = v_target;

  model.set_state(state);
  model.Step(dt);

  auto desired_state = model.state();
  *velocity_at_dt = std::max(0.0, desired_state.v);

  return true;
}
}  // namespace TL::planning::game_common
