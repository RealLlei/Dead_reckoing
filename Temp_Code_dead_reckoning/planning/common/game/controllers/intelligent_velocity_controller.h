/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  intelligent_velocity_controller.h
 */
#include "planning/common/game/game_common/idm/idm.h"

namespace TL::planning::game_common {

using game_common::IDM;

class IntelligentVelocityControl {
 public:
  /**
   * @brief CalculateDesiredVelocity.
   * @param param
   * @param s
   * @param s_front
   * @param v
   * @param v_front
   * @param dt
   * @param velocity_at_dt
   * @return true: success.
   */
  static void CalculateDesiredVelocity(const IDM::Param& param, double s,
                                       double s_front, double v, double v_front,
                                       double dt, double* velocity_at_dt);
};
}  // namespace TL::planning::game_common
