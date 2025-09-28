/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  responsibility_sensitive_safety.cc
 */

#include "planning/common/game/game_common/rss/responsibility_sensitive_safety.h"
#include <cmath>

namespace TL::planning::game_common {

bool RssChecker::CalculateSafeLongitudinalDistance(
    const double ego_vel, const double other_vel,
    const LongitudinalDirection& direction, const RssConfig& config,
    double* const distance) {
  const double ego_vel_abs = fabs(ego_vel);
  const double other_vel_abs = fabs(other_vel);
  const double ego_vel_at_response_time =
      ego_vel_abs + config.longitudinal_acc_max * config.response_time;
  const double other_vel_at_response_time =
      other_vel_abs + config.longitudinal_acc_max * config.response_time;

  double ego_distance_driven = 0.0;
  double other_distance_driven = 0.0;

  if (direction == LongitudinalDirection::Front) {
    ego_distance_driven =
        (ego_vel_abs + ego_vel_at_response_time) / 2.0 * config.response_time +
        ego_vel_at_response_time * ego_vel_at_response_time /
            (2.0 * config.longitudinal_break_min);
    if (ego_vel >= 0.0 && other_vel >= 0.0) {
      // ego vehicle ==> other vehicle -->
      other_distance_driven = (other_vel_abs * other_vel_abs) /
                              (2.0 * config.longitudinal_break_max);
      *distance = other_distance_driven - ego_distance_driven;
    } else if (ego_vel >= 0.0 && other_vel <= 0.0) {
      // ego vehicle ==> <-- other vehicle
      other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                  2.0 * config.response_time +
                              other_vel_at_response_time *
                                  other_vel_at_response_time /
                                  (2.0 * config.longitudinal_break_min);
      *distance = ego_distance_driven + other_distance_driven;
    } else {
      // 自车不支持倒车
      *distance = 0.0;
    }
  } else if (direction == LongitudinalDirection::Rear) {
    ego_distance_driven =
        ego_vel_abs * ego_vel_abs / (2.0 * config.longitudinal_break_max);
    if (ego_vel >= 0.0 && other_vel >= 0.0) {
      // other vehicle --> ego vehicle ==>
      other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                  2.0 * config.response_time +
                              other_vel_at_response_time *
                                  other_vel_at_response_time /
                                  (2.0 * config.longitudinal_break_min);
      *distance = other_distance_driven - ego_distance_driven;
    } else {
      // 自车不支持倒车
      *distance = 0.0;
    }
  }

  *distance = *distance > 0.0 ? *distance : 0.0;
  return true;
}

}  // namespace TL::planning::game_common
