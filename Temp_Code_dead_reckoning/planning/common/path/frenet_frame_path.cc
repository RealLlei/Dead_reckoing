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
 * @file frenet_frame_path.cc
 **/
#include "planning/common/path/frenet_frame_path.h"

#include <algorithm>
#include <limits>

#include "common/math/linear_interpolation.h"

namespace TL {
namespace planning {

using TL::common::FrenetFramePoint;

FrenetFramePath::FrenetFramePath(std::vector<FrenetFramePoint> points)
    : std::vector<FrenetFramePoint>(std::move(points)) {
  if (size() > 1) {
    const double kEpsilon = 1e-4;
    is_forward_path_ = back().s() - front().s() > kEpsilon;
    space_resolution_ = at(1).s() - at(0).s();
  } else {
    is_forward_path_ = true;
    space_resolution_ = FLAGS_trajectory_space_resolution;
  }
}

double FrenetFramePath::Length() const {
  if (empty()) {
    return 0.0;
  }
  return is_forward_path_ ? back().s() - front().s() : front().s() - back().s();
}

FrenetFramePoint FrenetFramePath::GetNearestPoint(const SLBoundary& sl) const {
  auto it_lower = is_forward_path_
                      ? std::lower_bound(begin(), end(), sl.start_s(),
                                         LowerBoundComparatorForForwardPath)
                      : std::lower_bound(begin(), end(), sl.end_s(),
                                         LowerBoundComparatorForBackwardPath);
  if (it_lower == end()) {
    return back();
  }
  auto it_upper = is_forward_path_
                      ? std::upper_bound(it_lower, end(), sl.end_s(),
                                         UpperBoundComparatorForForwardPath)
                      : std::upper_bound(it_lower, end(), sl.start_s(),
                                         UpperBoundComparatorForBackwardPath);
  double min_dist = std::numeric_limits<double>::max();
  auto min_it = it_upper;
  for (auto it = it_lower; it != it_upper; ++it) {
    if (it->l() >= sl.start_l() && it->l() <= sl.end_l()) {
      return *it;
    }
    if (it->l() > sl.end_l()) {
      double diff = it->l() - sl.end_l();
      if (diff < min_dist) {
        min_dist = diff;
        min_it = it;
      }
    } else {
      double diff = sl.start_l() - it->l();
      if (diff < min_dist) {
        min_dist = diff;
        min_it = it;
      }
    }
  }
  return *min_it;
}

FrenetFramePoint FrenetFramePath::EvaluateByS(const double s) const {
  if (size() < 2) {
    FrenetFramePoint p;
    p.set_s(s);
    p.set_l(0);
    p.set_dl(0);
    p.set_ddl(0);
    return p;
  }
  auto it_lower = is_forward_path_
                      ? std::lower_bound(begin(), end(), s,
                                         LowerBoundComparatorForForwardPath)
                      : std::lower_bound(begin(), end(), s,
                                         LowerBoundComparatorForBackwardPath);
  if (it_lower == begin()) {
    return front();
  }
  if (it_lower == end()) {
    return back();
  }
  const auto& p0 = *(it_lower - 1);
  const auto s0 = p0.s();
  const auto& p1 = *it_lower;
  const auto s1 = p1.s();

  FrenetFramePoint p;
  p.set_s(s);
  p.set_l(common::math::lerp(p0.l(), s0, p1.l(), s1, s));
  p.set_dl(common::math::lerp(p0.dl(), s0, p1.dl(), s1, s));
  p.set_ddl(common::math::lerp(p0.ddl(), s0, p1.ddl(), s1, s));
  return p;
}

bool FrenetFramePath::is_forward_path() const {
  return is_forward_path_;
}

}  // namespace planning
}  // namespace TL
