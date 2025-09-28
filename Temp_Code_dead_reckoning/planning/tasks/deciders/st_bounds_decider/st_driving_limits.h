/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 *   @file
 **/

#pragma once

#include <tuple>
#include <utility>
#include <vector>

#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {

class STDrivingLimits {
 public:
  STDrivingLimits() = default;
  virtual ~STDrivingLimits() = default;

  void Init(double max_acc, double max_dec, double max_v, double curr_v);

  /** @brief Given time t, calculate the driving limits in s due to
   * vehicle's dynamics.
   * @param Timestamp t.
   * @return The lower and upper bounds.
   */
  [[nodiscard]] std::pair<double, double> GetVehicleDynamicsLimits(
      double t) const;

  /** @brief Update the anchoring of the vehicle dynamics limits.
   * For example, when ADC is blocked by some obstacle, its max.
   * drivable area, max. speed, etc. are also limited subsequently.
   * @param Time t
   * @param lower bound in s
   * @param lower bound's corresponding speed.
   * @param upper bound in s
   * @param upper bound's corresponding speed.
   */
  void UpdateBlockingInfo(double t, double lower_s, double lower_v,
                          double upper_s, double upper_v);

  void ChangeMaxDece(double acce);

  [[nodiscard]] double GetMaxDece() const { return max_dec_; }

  void SetMaxAcce(double acce) {
    if (max_acc_ < acce) {
      max_acc_ = acce;
    }
  }

 private:
  // Private variables for calculating vehicle dynamic limits:
  double max_acc_ = 0.0;
  double max_dec_ = 0.0;
  double max_v_ = 0.0;

  double upper_t0_ = 0.0;
  double upper_v0_ = 0.0;
  double upper_s0_ = 0.0;

  double lower_t0_ = 0.0;
  double lower_v0_ = 0.0;
  double lower_s0_ = 0.0;

  // The limits expressed as v vs. s, which contains the following parts:
  //  1. speed limits at path segments with big curvatures.
  std::vector<std::tuple<double, double, double>> curvature_speed_limits_s_v_;
  //  2. speed limits from traffic limits (speed bumps, etc.).
  std::vector<std::tuple<double, double, double>> traffic_speed_limits_s_v_;
  //  3. speed limits for safety considerations when other obstacles are nearby
  std::vector<std::tuple<double, double, double>> obstacles_speed_limits_s_v_;
};

}  // namespace planning
}  // namespace TL
