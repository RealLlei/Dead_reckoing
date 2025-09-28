#ifndef PLANNING_COMMON_SPEED_PROFILE_GENERATOR_H
#define PLANNING_COMMON_SPEED_PROFILE_GENERATOR_H

/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file speed_profile_generator.h
 **/

#pragma once

#include <utility>
#include <vector>

#include "planning/common/ego_info.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/speed/speed_data.h"
#include "planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

class SpeedProfileGenerator {
 public:
  SpeedProfileGenerator() = delete;

  static SpeedData GenerateFallbackSpeed(const EgoInfo* ego_info,
                                         double max_dece = 0.0,
                                         double stop_distance = 0.0,
                                         bool is_forward_plan = true);

  static void FillEnoughSpeedPoints(SpeedData* speed_data);

  static SpeedData GenerateFixedDistanceCreepProfile(double distance,
                                                     double max_speed);

 private:
  static SpeedData GenerateStopProfile(double init_speed, double init_acc,
                                       bool is_forward_plan);
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_SPEED_PROFILE_GENERATOR_H
