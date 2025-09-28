#ifndef PLANNING_COMMON_TRAJECTORY1D_CONSTANT_DECELERATION_TRAJECTORY1D_H
#define PLANNING_COMMON_TRAJECTORY1D_CONSTANT_DECELERATION_TRAJECTORY1D_H

/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

class ConstantDecelerationTrajectory1d : public Curve1d {
 public:
  ConstantDecelerationTrajectory1d(double init_s, double init_v, double a);

  ~ConstantDecelerationTrajectory1d() override = default;

  double ParamLength() const override;

  std::string ToString() const override;

  // handles extrapolation internally
  double Evaluate(std::uint32_t order, double param) const override;

 private:
  double Evaluate_s(double t) const;

  double Evaluate_v(double t) const;

  double Evaluate_a(double t) const;

  static double Evaluate_j(double t);

  double init_s_;

  double init_v_;

  double deceleration_;

  double end_t_;

  double end_s_;
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_TRAJECTORY1D_CONSTANT_DECELERATION_TRAJECTORY1D_H
