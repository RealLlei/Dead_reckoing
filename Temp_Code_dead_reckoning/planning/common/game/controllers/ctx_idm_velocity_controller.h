/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  ctx_idm_velocity_controller.h
 */

#include "planning/common/game/game_common/idm/idm.h"
#include "planning/common/game/vehicle_model/ctx_idm_model.h"

namespace TL::planning::game_common {

using game_common::ContextIntelligentDriverModel;
using game_common::IDM;

class ContextIntelligentVelocityControl {
 public:
  /**
   * @brief CalculateDesiredVelocity.
   * @param idm_param
   * @param ctx_param
   * @param s
   * @param s_front
   * @param s_target
   * @param v
   * @param v_front
   * @param v_target
   * @param dt
   * @param velocity_at_dt
   * @return true: success.
   */
  static bool CalculateDesiredVelocity(
      const IDM::Param& idm_param,
      const ContextIntelligentDriverModel::CtxParam& ctx_param, double s,
      double s_front, double s_target, double v, double v_front,
      double v_target, double dt, double* velocity_at_dt);
};

}  // namespace TL::planning::game_common
