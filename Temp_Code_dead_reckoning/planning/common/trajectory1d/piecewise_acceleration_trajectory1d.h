#ifndef PLANNING_COMMON_TRAJECTORY1D_PIECEWISE_ACCELERATION_TRAJECTORY1D_H
#define PLANNING_COMMON_TRAJECTORY1D_PIECEWISE_ACCELERATION_TRAJECTORY1D_H

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

#include <array>
#include <string>
#include <vector>

#include "planning/math/curve1d/curve1d.h"

namespace TL {
namespace planning {

class PiecewiseAccelerationTrajectory1d : public Curve1d {
 public:
  PiecewiseAccelerationTrajectory1d(double start_s, double start_v);

  ~PiecewiseAccelerationTrajectory1d() override = default;

  void AppendSegment(double a, double t_duration);

  void PopSegment();

  double ParamLength() const override;

  std::string ToString() const override;

  double Evaluate(std::uint32_t order, double param) const override;

  std::array<double, 4> Evaluate(double t) const;

  size_t Size() const { return t_.size(); }

 private:
  double Evaluate_s(double t) const;

  double Evaluate_v(double t) const;

  double Evaluate_a(double t) const;

  static double Evaluate_j(double t);

 private:
  // accumulated s
  std::vector<double> s_;

  std::vector<double> v_;

  // accumulated t
  std::vector<double> t_;

  std::vector<double> a_;
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_TRAJECTORY1D_PIECEWISE_ACCELERATION_TRAJECTORY1D_H
