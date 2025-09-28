/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  idm_model.h
 */

#pragma once

#include "planning/common/game/game_common/idm/idm.h"

namespace TL::planning::game_common {
using game_common::IDM;
using InternalState = std::array<double, 4>;

class IntelligentDriverModel {
 public:
  IntelligentDriverModel();
  explicit IntelligentDriverModel(const IDM::Param& param);

  /**
   * @brief state.
   * @return state
   */
  inline const IDM::State& state() const { return state_; }

  /**
   * @brief set_state.
   * @param state
   */
  inline void set_state(const IDM::State& state) {
    state_ = state;
    UpdateInternalState();
  }

  /**
   * @brief Step.
   * @param dt
   */
  void Step(double dt);

  void Linear(const InternalState& x, double dt, InternalState* x_out);

 private:
  /**
   * @brief UpdateInternalState.
   */
  void UpdateInternalState();

  IDM::Param param_;
  IDM::State state_;

  InternalState internal_state_ = {};
};
}  // namespace TL::planning::game_common
