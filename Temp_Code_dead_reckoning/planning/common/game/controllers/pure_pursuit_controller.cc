/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  pure_pursuit_controller.cc
 */

#include "planning/common/game/controllers/pure_pursuit_controller.h"
#include <cmath>

namespace TL::planning::game_common {
bool PurePursuitControl::CalculateDesiredSteer(const double wheelbase_len,
                                               const double angle_diff,
                                               const double look_ahead_dist,
                                               double* steer) {
  if (steer == nullptr) {
    return false;
  }
  *steer =
      std::atan2(2.0 * wheelbase_len * std::sin(angle_diff), look_ahead_dist);
  return true;
}
}  // namespace TL::planning::game_common
