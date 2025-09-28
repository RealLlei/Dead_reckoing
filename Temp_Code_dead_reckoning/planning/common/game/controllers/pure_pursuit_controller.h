/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  pure_pursuit_controller.h
 */

namespace TL::planning::game_common {

class PurePursuitControl {
 public:
  /**
   * @brief CalculateDesiredSteer.
   * @param wheelbase_len
   * @param angle_diff
   * @param look_ahead_dist
   * @param steer
   * @return true: success.
   */
  static bool CalculateDesiredSteer(double wheelbase_len, double angle_diff,
                                    double look_ahead_dist, double* steer);
};
}  // namespace TL::planning::game_common
