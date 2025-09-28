#ifndef PLANNING_COMMON_TRAJECTORY1D_PIECEWISE_TRAJECTORY1D_H
#define PLANNING_COMMON_TRAJECTORY1D_PIECEWISE_TRAJECTORY1D_H

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

#include <memory>
#include <string>
#include <vector>

#include "planning/math/curve1d/curve1d.h"

namespace TL {
namespace planning {

class PiecewiseTrajectory1d : public Curve1d {
 public:
  PiecewiseTrajectory1d() = default;

  ~PiecewiseTrajectory1d() override = default;

  double Evaluate(std::uint32_t order, double param) const override;

  double ParamLength() const override;

  std::string ToString() const override;

  void AppendSegment(const std::shared_ptr<Curve1d>& trajectory);

  void PopSegment();

  size_t NumOfSegments() const;

 private:
  std::vector<std::shared_ptr<Curve1d>> trajectory_segments_;

  std::vector<double> accumulated_param_lengths_;
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_TRAJECTORY1D_PIECEWISE_TRAJECTORY1D_H
