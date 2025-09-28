#ifndef PLANNING_COMMON_TRAJECTORY1D_PIECEWISE_JERK_TRAJECTORY1D_H
#define PLANNING_COMMON_TRAJECTORY1D_PIECEWISE_JERK_TRAJECTORY1D_H

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
#include <vector>

#include "planning/common/trajectory1d/constant_jerk_trajectory1d.h"
#include "planning/math/curve1d/curve1d.h"

namespace TL {
namespace planning {

class PiecewiseJerkTrajectory1d : public Curve1d {
 public:
  PiecewiseJerkTrajectory1d(double p, double v, double a);

  ~PiecewiseJerkTrajectory1d() override = default;

  double Evaluate(std::uint32_t order, double param) const override;

  double ParamLength() const override;

  std::string ToString() const override;

  void AppendSegment(double jerk, double param);

 private:
  void Init(double p, double v, double a);
  std::vector<ConstantJerkTrajectory1d> segments_;

  double last_p_ = 0.0;

  double last_v_ = 0.0;

  double last_a_ = 0.0;

  std::vector<double> param_;
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_TRAJECTORY1D_PIECEWISE_JERK_TRAJECTORY1D_H
