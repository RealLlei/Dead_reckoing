/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description: ctx_idm_model.cc
 */

#include "planning/common/game/vehicle_model/ctx_idm_model.h"
#include "planning/common/game/game_common/idm/idm.h"

namespace TL::planning::game_common {

ContextIntelligentDriverModel::ContextIntelligentDriverModel() {
  UpdateInternalState();
}

ContextIntelligentDriverModel::ContextIntelligentDriverModel(
    const IDM::Param& idm_parm, const CtxParam& ctx_param)
    : ctx_param_(ctx_param), idm_param_(idm_parm) {
  UpdateInternalState();
}

void ContextIntelligentDriverModel::Step(const double dt) {
  Linear(internal_state_, dt, &internal_state_);
  state_.s = internal_state_[0];
  state_.v = internal_state_[1];
  state_.s_front = internal_state_[2];
  state_.v_front = internal_state_[3];
  state_.s_target = internal_state_[4];
  state_.v_target = internal_state_[5];
  UpdateInternalState();
}

void ContextIntelligentDriverModel::UpdateInternalState() {
  internal_state_[0] = state_.s;
  internal_state_[1] = state_.v;
  internal_state_[2] = state_.s_front;
  internal_state_[3] = state_.v_front;
  internal_state_[4] = state_.s_target;
  internal_state_[5] = state_.v_target;
}

const ContextIntelligentDriverModel::CtxIdmState&
ContextIntelligentDriverModel::state() const {
  return state_;
}

void ContextIntelligentDriverModel::set_state(
    const ContextIntelligentDriverModel::CtxIdmState& state) {
  state_ = state;
  UpdateInternalState();
}

void ContextIntelligentDriverModel::Linear(const InternalState& x,
                                           const double dt,
                                           InternalState* x_out) {
  CtxIdmState cur_state;
  cur_state.s = x[0];
  cur_state.v = x[1];
  cur_state.s_front = x[2];
  cur_state.v_front = x[3];
  cur_state.s_target = x[4];
  cur_state.v_target = x[5];

  IDM::State idm_state;
  idm_state.s = cur_state.s;
  idm_state.s_front = cur_state.s_front;
  idm_state.v = cur_state.v;
  idm_state.v_front = cur_state.v_front;

  double acc_idm = 0.0;

  IDM::GetAccDesiredAcceleration(idm_param_, idm_state, &acc_idm);

  acc_idm = fmax(acc_idm,
                 -fmin(idm_param_.hard_breaking_decelerate, cur_state.v / dt));

  const double v_ref =
      cur_state.v_target + ctx_param_.k_s * (cur_state.s_target - cur_state.s);

  double acc_track = ctx_param_.k_v * (v_ref - cur_state.v);
  acc_track = fmin(fmax(acc_track, -1.0), 1.0);
  const double acc = fmin(acc_track, acc_idm);

  x_out->at(0) = x.at(0) + cur_state.v * dt + 0.5 * dt * dt * acc;
  x_out->at(1) = cur_state.v + acc * dt;
  x_out->at(2) = x.at(3) + cur_state.v_front * dt;
  x_out->at(3) = x.at(3);
  x_out->at(4) = x.at(4) + cur_state.v_target * dt;
  x_out->at(5) = x.at(5);
}
}  // namespace TL::planning::game_common
