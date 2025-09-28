/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  responsibility_sensitive_safety.h
 */

#include "planning/common/game/game_common/semantics/semantics.h"

namespace TL::planning::game_common {
using game_common::FrenetState;

class RssChecker {
 public:
  enum LongitudinalDirection { Front = 0, Rear };

  struct RssConfig {
    double response_time = 0.1;
    double longitudinal_acc_max = 2.0;
    double longitudinal_break_min = 4.0;
    double longitudinal_break_max = 5.0;

    RssConfig() = default;

    RssConfig(const double response_time, const double longitudinal_acc_max,
              const double longitudinal_break_min,
              const double longitudinal_break_max)
        : response_time(response_time),
          longitudinal_acc_max(longitudinal_acc_max),
          longitudinal_break_min(longitudinal_break_min),
          longitudinal_break_max(longitudinal_break_max) {}
  };

  RssChecker() = default;

  /**
   * @brief CalculateSafeLongitudinalDistance.
   * @param ego_vel
   * @param other_vel
   * @param direction
   * @param config
   * @param distance
   * @return true: success.
   */
  static bool CalculateSafeLongitudinalDistance(
      double ego_vel, double other_vel, const LongitudinalDirection& direction,
      const RssConfig& config, double* distance);
};
}  // namespace TL::planning::game_common
