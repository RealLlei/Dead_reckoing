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

#include "planning/common/trajectory1d/piecewise_acceleration_trajectory1d.h"
#include <sys/types.h>

#include <algorithm>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "common/file/log.h"
#include "common/math/linear_interpolation.h"
#include "planning/common/planning_gflags.h"

namespace TL {
namespace planning {

PiecewiseAccelerationTrajectory1d::PiecewiseAccelerationTrajectory1d(
    const double start_s, const double start_v) {
  s_.push_back(start_s);
  v_.push_back(start_v);
  a_.push_back(0.0);
  t_.push_back(0.0);
}

void PiecewiseAccelerationTrajectory1d::AppendSegment(const double a,
                                                      const double t_duration) {
  if (t_duration < FLAGS_numerical_epsilon) {
    return;
  }
  double s0 = s_.back();
  double v0 = v_.back();
  double t0 = t_.back();

  double v1 = std::max(0.0, v0 + a * t_duration);
  ACHECK(v1 >= -FLAGS_numerical_epsilon);

  double delta_s = (v0 + v1) * t_duration * 0.5;
  double s1 = std::max(s0 + delta_s, s0);
  double t1 = t0 + t_duration;

  ACHECK(s1 >= s0 - FLAGS_numerical_epsilon);
  s1 = std::max(s1, s0);
  s_.push_back(s1);
  v_.push_back(v1);
  a_.push_back(a);
  t_.push_back(t1);
}

void PiecewiseAccelerationTrajectory1d::PopSegment() {
  if (!a_.empty()) {
    s_.pop_back();
    v_.pop_back();
    a_.pop_back();
    t_.pop_back();
  }
}

double PiecewiseAccelerationTrajectory1d::ParamLength() const {
  CHECK_GT(t_.size(), 1U);
  return t_.back() - t_.front();
}

std::string PiecewiseAccelerationTrajectory1d::ToString() const {
  return absl::StrCat(absl::StrJoin(s_, "\t"), absl::StrJoin(t_, "\t"),
                      absl::StrJoin(v_, "\t"), absl::StrJoin(a_, "\t"), "\n");
}

double PiecewiseAccelerationTrajectory1d::Evaluate(const std::uint32_t order,
                                                   const double param) const {
  CHECK_GT(t_.size(), 1U);
  ACHECK(t_.front() <= param && param <= t_.back());  //NOLINT

  switch (order) {
    case 0:
      return Evaluate_s(param);
    case 1:
      return Evaluate_v(param);
    case 2:
      return Evaluate_a(param);
    case 3:
      return Evaluate_j(param);
    default:
      return 0.0;
  }
  return 0.0;
}

double PiecewiseAccelerationTrajectory1d::Evaluate_s(const double t) const {
  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  auto index = std::distance(t_.begin(), it_lower);
  if (index < 1) {
    return s_[0];
  }
  double s0 = s_[index - 1];
  double v0 = v_[index - 1];
  double t0 = t_[index - 1];

  double v1 = v_[index];
  double t1 = t_[index];

  double v = common::math::lerp(v0, t0, v1, t1, t);
  double s = (v0 + v) * (t - t0) * 0.5 + s0;
  return s;
}

double PiecewiseAccelerationTrajectory1d::Evaluate_v(const double t) const {
  constexpr static double index_tmp = 0.1;
  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);

  auto index = static_cast<uint>(std::distance(t_.begin(), it_lower));
  if (index < 1) {
    return v_[0];
  }
  double v0 = v_[index - 1];
  double t0 = t_[index - 1];

  double v1 = v_[index];
  double t1 = t_[index];

  double v = TL::common::math::lerp(v0, t0, v1, t1, t);
  if (index < index_tmp) {
    v = v_[index];
    return v;
  }
  return v;
}

double PiecewiseAccelerationTrajectory1d::Evaluate_a(const double t) const {
  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  auto index = std::distance(t_.begin(), it_lower);
  if (index < 1) {
    return a_[0];
  }
  return a_[index];
}

double PiecewiseAccelerationTrajectory1d::Evaluate_j(const double t) {
  UNUSED(t);
  return 0.0;
}

std::array<double, 4> PiecewiseAccelerationTrajectory1d::Evaluate(
    const double t) const {
  CHECK_GT(t_.size(), 1U);
  ACHECK(t_.front() <= t && t <= t_.back());  //NOLINT

  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  auto index = std::distance(t_.begin(), it_lower);

  if (index < 1) {
    return {s_[0], v_[0], a_[0], 0.0};
  }

  double s0 = s_[index - 1];
  double v0 = v_[index - 1];
  double t0 = t_[index - 1];

  double v1 = v_[index];
  double t1 = t_[index];

  double v = common::math::lerp(v0, t0, v1, t1, t);
  double s = (v0 + v) * (t - t0) * 0.5 + s0;

  double a = a_[index];
  double j = 0.0;

  return {{s, v, a, j}};
}

}  // namespace planning
}  // namespace TL
