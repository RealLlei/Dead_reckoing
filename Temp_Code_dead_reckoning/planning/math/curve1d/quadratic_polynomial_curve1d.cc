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
 * @file quartic_polynomial_curve1d.cc
 **/

#include "planning/math/curve1d/quadratic_polynomial_curve1d.h"

#include <cmath>
#include <cstddef>
#include <limits>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"

#include "common/file/log.h"
#include "common/math/double_type.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreater;
using common::math::double_type::DefinitelyLess;

QuadraticPolynomialCurve1d::QuadraticPolynomialCurve1d(
    const std::array<double, 3>& start, const double param)
    : start_condition_(start) {
  param_ = param;
  ComputeCoefficients(start_condition_[0], start_condition_[1],
                      start_condition_[2], param);
  end_condition_[0] = Evaluate(0, param);
  end_condition_[1] = Evaluate(1, param);
}

QuadraticPolynomialCurve1d::QuadraticPolynomialCurve1d(
    const std::array<double, 3>& coef)
    : coef_(coef) {}

void QuadraticPolynomialCurve1d::DerivedFromCubicCurve(
    const PolynomialCurve1d& other) {
  CHECK_EQ(other.Order(), 3U);
  param_ = other.ParamLength();
  for (size_t i = 1; i < 4; ++i) {
    coef_.at(i - 1) = other.Coef(i) * static_cast<double>(i);
  }
}

double QuadraticPolynomialCurve1d::Evaluate(const std::uint32_t order,
                                            const double p) const {
  switch (order) {
    case 0: {
      return (coef_[2] * p + coef_[1]) * p + coef_[0];
    }
    case 1: {
      return 2.0 * coef_[2] * p + coef_[1];
    }
    case 2: {
      return 2.0 * coef_[2];
    }
    default:
      return 0.0;
  }
}

void QuadraticPolynomialCurve1d::ComputeCoefficients(const double x0,
                                                     const double dx0,
                                                     const double ddx0,
                                                     const double p) {
  CHECK_GT(p, 0.0);
  coef_[0] = x0;
  coef_[1] = dx0;
  coef_[2] = 0.5 * ddx0;
}

std::string QuadraticPolynomialCurve1d::ToString() const {
  return absl::StrCat(absl::StrJoin(coef_, "\t"), param_, "\n");
}

double QuadraticPolynomialCurve1d::Coef(const size_t order) const {
  CHECK_GT(5U, order);
  return coef_.at(order);
}

std::vector<double> QuadraticPolynomialCurve1d::Solve(const double p) const {
  if (common::math::double_type::IsZero(coef_[2])) {
    return {(p - coef_[0]) / coef_[1]};
  }
  return {
      0.5 *
          (-coef_[1] - sqrt(pow(coef_[1], 2) - 4 * coef_[2] * (coef_[0] - p))) /
          coef_[2],
      0.5 *
          (-coef_[1] + sqrt(pow(coef_[1], 2) - 4 * coef_[2] * (coef_[0] - p))) /
          coef_[2]};
}

bool QuadraticPolynomialCurve1d::Solve(std::vector<double>* const root) const {
  bool ret = false;
  if (root == nullptr) {
    AERROR << "Solve inout check fails";
    return ret;
  }
  root->clear();
  if (common::math::double_type::IsZero(coef_.at(2))) {
    if (!common::math::double_type::IsZero(coef_.at(1))) {
      root->emplace_back(-coef_[0] / coef_[1]);
      ret = true;
    } else {
      AERROR << "coeffs are not valid";
    }
  } else {
    const double delta = pow(coef_[1], 2) - 4 * coef_[2] * coef_[0];
    if (delta > 0) {
      ret = true;
      root->emplace_back((-coef_[1] - sqrt(delta)) / coef_[2] / 2);
      root->emplace_back((-coef_[1] + sqrt(delta)) / coef_[2] / 2);
    } else if (common::math::double_type::IsZero(delta)) {
      ret = true;
      root->emplace_back(-coef_[1] / coef_[2] / 2);

    } else {
      ADEBUG << "there is no real solution";
    }
  }
  return ret;
}

double QuadraticPolynomialCurve1d::GetPeakValue(const double min_p,
                                                const double max_p) const {
  const auto peak_p = -0.5 * coef_[1] / coef_[2];
  if (std::isfinite(peak_p) && peak_p > min_p && peak_p < max_p) {
    return Evaluate(0, peak_p);
  }
  return std::numeric_limits<double>::infinity();
}

std::vector<std::pair<bool, double>>
QuadraticPolynomialCurve1d::GetMinAndMaxValue(double min_p, double max_p,
                                              double* min_x,
                                              double* max_x) const {
  std::vector<std::pair<bool, double>> peek_values;
  if (max_x == nullptr || min_x == nullptr) {
    return peek_values;
  }

  peek_values.reserve(1);
  const auto start_x = Evaluate(0, min_p);
  const auto end_x = Evaluate(0, max_p);
  *min_x = fmin(start_x, end_x);
  *max_x = fmax(start_x, end_x);

  const auto p1 = -0.5 * coef_[1] / coef_[2];
  if (std::isfinite(p1) && DefinitelyGreater(p1, min_p) &&
      DefinitelyLess(p1, max_p)) {
    const auto x = Evaluate(0, p1);
    *min_x = fmin(*min_x, x);
    *max_x = fmax(*max_x, x);
    if (Evaluate(0, p1 + 1e-2) > x) {
      peek_values.emplace_back(true, x);
    } else {
      peek_values.emplace_back(false, x);
    }
  }

  return peek_values;
}

}  // namespace planning
}  // namespace TL
