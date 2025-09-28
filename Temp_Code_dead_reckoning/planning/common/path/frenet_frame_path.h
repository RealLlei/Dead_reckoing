#ifndef PLANNING_COMMON_PATH_FRENET_FRAME_PATH_H
#define PLANNING_COMMON_PATH_FRENET_FRAME_PATH_H

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
 * @file frenet_frame_path.h
 **/

#pragma once

#include <utility>
#include <vector>

#include "planning/common/planning_gflags.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/planning/sl_boundary.pb.h"

namespace TL {
namespace planning {

class FrenetFramePath : public std::vector<common::FrenetFramePoint> {
 public:
  FrenetFramePath() = default;
  explicit FrenetFramePath(std::vector<common::FrenetFramePoint> points);

  double Length() const;
  common::FrenetFramePoint EvaluateByS(double s) const;

  /**
   * @brief Get the FrenetFramePoint that is within SLBoundary, or the one with
   * smallest l() in SLBoundary's s range [start_s(), end_s()]
   */
  common::FrenetFramePoint GetNearestPoint(const SLBoundary& sl) const;

  bool is_forward_path() const;

  /**
   * @brief Get space resolution
   * 
   * @return double 
   */
  double GetSpaceResolution() const { return space_resolution_; }

 private:
  static bool LowerBoundComparatorForForwardPath(
      const common::FrenetFramePoint& p, const double s) {
    return p.s() < s;
  }

  static bool LowerBoundComparatorForBackwardPath(
      const common::FrenetFramePoint& p, const double s) {
    return s < p.s();
  }

  static bool UpperBoundComparatorForForwardPath(
      const double s, const common::FrenetFramePoint& p) {
    return s < p.s();
  }

  static bool UpperBoundComparatorForBackwardPath(
      const double s, const common::FrenetFramePoint& p) {
    return p.s() < s;
  }

 private:
  bool is_forward_path_ = true;
  double space_resolution_ = FLAGS_trajectory_space_resolution;
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_PATH_FRENET_FRAME_PATH_H
