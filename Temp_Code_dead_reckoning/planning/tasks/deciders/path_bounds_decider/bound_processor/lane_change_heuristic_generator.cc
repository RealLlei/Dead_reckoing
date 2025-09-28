/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Description:  planning path lane change heuristic path generator
 * Author: ROC
 */

#include "planning/tasks/deciders/path_bounds_decider/bound_processor/lane_change_heuristic_generator.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include "common/math/double_type.h"
#include "planning/tasks/deciders/path_bounds_decider/util/path_info.h"

namespace TL {
namespace planning {
using TL::common::math::double_type::IsZero;

// lane change length here needs to be a little larger than length used for planning
// to avoid too tight constraints
double LaneChangeHeuristicGenerator::ComputeLaneChangeLength(const double v) {
  return std::max(15.0, 5.5 * v + 0.2 * std::pow(v, 2));
}

bool LaneChangeHeuristicGenerator::InitSuccess() const {
  return init_success_;
}

// cos function to generate a heuristic lane change reference curve
// Init vehicle state（s0、l0、dl0）
// f(s) = A * (1+cos(B*s+C)), parameters(A，B，C)
// [1]f(s0) = l0
// [2]f’(s0) = dl0
// [3]B = 2 * PI/T
// [4]T = 2 * g(v)
// [5]g(v) = 5.5 * v + 0.2 * v * v, g(v): heuristic lane change finish distance
void LaneChangeHeuristicGenerator::Init(const double s0, const double l0,
                                        const double dl0,
                                        const double lane_change_length) {
  if (lane_change_length < KAlmostZero1ENegtive3 || IsZero(l0)) {
    AERROR << "Input params is too small";
    init_success_ = false;
    return;
  }
  s0_ = s0;
  s0_1_ = s0;
  l0_ = l0;
  s1_ = s0;
  l1_ = l0;
  dl0_ = dl0;
  const double t_cycle = 2 * lane_change_length;
  b_ = 2 * M_PI / t_cycle;
  a_ = (std::pow(b_, 2) * std::pow(l1_, 2) + std::pow(dl0_, 2)) /
       (2 * l1_ * std::pow(b_, 2));
  const double sin_s1 = -dl0_ / a_ / b_;
  const double cos_s1 = l1_ / a_ - 1;
  double theta = asin(sin_s1);

  if (sin_s1 > 0 && cos_s1 < 0) {
    theta = M_PI - asin(sin_s1);
  } else if (sin_s1 < 0 && cos_s1 < 0) {
    theta = -M_PI - asin(sin_s1);
  }
  c_ = theta - b_ * s1_;
  // heuristic overshoot f(s_zero_) = 0.0 to get s_zero_
  int k = ceil(((s1_ * b_ + c_) / M_PI + 1) / 2);
  s_zero_ = ((2 * k - 1) * M_PI - c_) / b_;
  init_success_ = true;
}

// cos function to generate a heuristic lane change reference curve
// Init vehicle state（s0、l0、dl0）
// f(s) = A * (1+cos(B*s+C)), parameters(A，B，C)
// [1]f(s0) = l0
// [2]f’(s0) = dl0
// [3]B = 2 * PI/T
// [4]T = 2 * g(v)
// [5]g(v) = 5.5 * v + 0.2 * v * v, g(v): heuristic lane change finish distance
void LaneChangeHeuristicGenerator::Init(
    const double s0, const double l0, const double dl0,
    const double lane_change_length, const double s_prepare,
    const double l_limit, const double dl_limit, const bool enable_dl_limit) {
  if (lane_change_length < KAlmostZero1ENegtive3 || IsZero(l0) ||
      IsZero(dl_limit)) {
    AERROR << "Input params is too small, lane_change_length: "
           << lane_change_length << ", l0: " << l0
           << ", dl_limit: " << dl_limit;
    init_success_ = false;
    return;
  }
  s0_ = s0;
  l0_ = l0;
  dl0_ = IsZero(dl0) ? KAlmostZero1ENegtive8 : dl0;

  s1_ = s0_ + s_prepare;
  l1_ = l0_ + (s1_ - s0_) * dl0_;

  s_prepare_ = s_prepare;

  ADEBUG << "s_prepare: " << s_prepare_;
  if ((l0_ >= 0 && l1_ <= l_limit) || (l0_ <= 0 && l1_ >= -l_limit)) {
    l1_ = l0_ > 0 ? l_limit : -l_limit;
    s1_ = (l1_ - l0_) / dl0_ + s0_;
    s_prepare_ = std::min(std::max(s1_ - s0_, 0.0), s_prepare_);
  }
  ADEBUG << "l s_prepare: " << s_prepare_ << ", l0_: " << l0_
         << ", s0_: " << s0_ << ", l1_: " << l1_ << ", s1_: " << s1_;

  if (enable_dl_limit) {
    const double dl1_max = l0_ > 0 ? -dl_limit : dl_limit;
    s_prepare_ = std::max(((dl1_max - dl0_) * s_prepare_) / dl1_max, 0.0);
    ADEBUG << "dl s_prepare: " << s_prepare_ << ", dl0_: " << dl0_
           << ", dl1_max: " << dl1_max;
  }

  l0_1_ = (l1_ + l0_) / 2;
  l0_1_ = l0_;

  if ((l0_1_ >= l0_ && l0_1_ <= l1_) || (l0_1_ >= l1_ && l0_1_ <= l0_)) {
    s0_1_ = (l0_1_ - l0_) / dl0_ + s0_;
  }

  const double t_cycle = 2 * lane_change_length;
  b_ = 2 * M_PI / t_cycle;
  a_ = (std::pow(b_, 2) * std::pow(l1_, 2) + std::pow(dl0_, 2)) /
       (2 * l1_ * std::pow(b_, 2));
  const double sin_s1 = -dl0_ / a_ / b_;
  const double cos_s1 = l1_ / a_ - 1;
  double theta = asin(sin_s1);

  if (sin_s1 > 0 && cos_s1 < 0) {
    theta = M_PI - asin(sin_s1);
  } else if (sin_s1 < 0 && cos_s1 < 0) {
    theta = -M_PI - asin(sin_s1);
  }
  c_ = theta - b_ * s1_;
  // heuristic overshoot f(s_zero_) = 0.0 to get s_zero_
  int k = ceil(((s1_ * b_ + c_) / M_PI + 1) / 2);
  s_zero_ = ((2 * k - 1) * M_PI - c_) / b_;
  init_success_ = true;
}

double LaneChangeHeuristicGenerator::GetLWithS(const double s) const {
  if (s <= s0_1_) {
    return l0_;
  }
  if (s <= s1_) {
    return l0_ + (s - s0_) * dl0_;
  }
  if (s <= s_zero_) {
    return a_ * (1.0 + cos(b_ * s + c_));
  }
  return 0.0;
}

double LaneChangeHeuristicGenerator::GetDlWithS(const double s) const {
  if (s <= s0_) {
    return 0.0;
  }
  if (s <= s1_) {
    return dl0_;
  }
  if (s <= s_zero_) {
    return -a_ * b_ * sin(b_ * s + c_);
  }
  return 0.0;
}
}  // namespace planning
}  // namespace TL
