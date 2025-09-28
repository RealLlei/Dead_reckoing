/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description: ctx_idm_model.h
 */
#pragma once
#include "planning/common/game/game_common/idm/idm.h"

namespace TL::planning::game_common {
using InternalState = std::array<double, 6>;
using game_common::IDM;

class ContextIntelligentDriverModel {
 public:
  struct CtxParam {
    double k_s = 0.5;
    double k_v = 2.0 * k_s;

    CtxParam() = default;

    CtxParam(const double k_s, const double k_v) : k_s(k_s), k_v(k_v) {}
  };

  struct CtxIdmState {
    double s = 0.0;
    double v = 0.0;
    double s_front = 0.0;
    double v_front = 0.0;
    double s_target = 0.0;
    double v_target = 0.0;
  };

  /**
   * @brief Step.
   * @param dt
   */
  void Step(double dt);

  /**
   * @brief state.
   * @return CtxIdmState
   */
  const CtxIdmState& state() const;

  /**
   * @brief set_state.
   * @param state
   */
  void set_state(const CtxIdmState& state);

  ContextIntelligentDriverModel();

  ContextIntelligentDriverModel(const IDM::Param& idm_parm,
                                const CtxParam& ctx_param);
  void Linear(const InternalState& x, double dt, InternalState* x_out);

 private:
  void UpdateInternalState();
  InternalState internal_state_ = {};
  CtxIdmState state_;

  CtxParam ctx_param_;
  IDM::Param idm_param_;
};
}  // namespace TL::planning::game_common
