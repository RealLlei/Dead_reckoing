#ifndef PLANNING_COMMON_TRAJECTORY1D_CONSTANT_JERK_TRAJECTORY1D_H
#define PLANNING_COMMON_TRAJECTORY1D_CONSTANT_JERK_TRAJECTORY1D_H

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
 * @file
 **/

#pragma once

#include <string>

#include "planning/math/curve1d/curve1d.h"

namespace TL {
namespace planning {

class ConstantJerkTrajectory1d : public Curve1d {
 public:
  ConstantJerkTrajectory1d(double p0, double v0, double a0, double jerk,
                           double param);

  ~ConstantJerkTrajectory1d() override = default;

  double Evaluate(std::uint32_t order, double param) const override;
  /**
   * @brief 
   * 
   */
  void Init();

  double ParamLength() const override;

  std::string ToString() const override;

  double start_position() const;

  double start_velocity() const;

  double start_acceleration() const;

  double end_position() const;

  double end_velocity() const;

  double end_acceleration() const;

  double jerk() const;

 private:
  double p0_ = 0.0;
  double v0_ = 0.0;
  double a0_ = 0.0;

  double p1_ = 0.0;
  double v1_ = 0.0;
  double a1_ = 0.0;

  double param_ = 0.0;

  double jerk_ = 0.0;
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_TRAJECTORY1D_CONSTANT_JERK_TRAJECTORY1D_H
