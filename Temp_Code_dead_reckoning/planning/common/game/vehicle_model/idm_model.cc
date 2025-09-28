/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  idm_model.cc
 */

#include "planning/common/game/vehicle_model/idm_model.h"
#include <algorithm>
#include "planning/common/game/game_common/idm/idm.h"

namespace TL::planning::game_common {

IntelligentDriverModel::IntelligentDriverModel() {
  UpdateInternalState();
}

IntelligentDriverModel::IntelligentDriverModel(const IDM::Param& param)
    : param_(param) {
  UpdateInternalState();
}

void IntelligentDriverModel::Step(const double dt) {
  Linear(internal_state_, dt, &internal_state_);
  state_.s = internal_state_[0];
  state_.v = internal_state_[1];
  state_.s_front = internal_state_[2];
  state_.v_front = internal_state_[3];
  UpdateInternalState();
}

void IntelligentDriverModel::Linear(const InternalState& x, const double dt,
                                    InternalState* x_out) {
  IDM::State cur_state = {x[0], x[1], x[2], x[3]};

  double acc = 0.0;
  IDM::GetIIdmDesiredAcceleration(param_, cur_state, &acc);

  acc = std::max(acc,
                 -std::min(param_.hard_breaking_decelerate, cur_state.v / dt));

  (*x_out)[0] = x[0] + cur_state.v * dt + 0.5 * acc * dt * dt;
  (*x_out)[1] = cur_state.v + acc * dt;
  (*x_out)[2] = x[2] + x[3] * dt;
  (*x_out)[3] = x[3];
}

void IntelligentDriverModel::UpdateInternalState() {
  internal_state_[0] = state_.s;
  internal_state_[1] = state_.v;
  internal_state_[2] = state_.s_front;
  internal_state_[3] = state_.v_front;
}
}  // namespace TL::planning::game_common
