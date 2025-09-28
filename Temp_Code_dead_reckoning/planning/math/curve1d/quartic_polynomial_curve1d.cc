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

#include "planning/math/curve1d/quartic_polynomial_curve1d.h"

#include <limits>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"

#include "common/file/log.h"
#include "common/math/double_type.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreater;
using common::math::double_type::DefinitelyLess;
using common::math::double_type::SeemsEqual;

QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
    const std::array<double, 3>& start, const std::array<double, 2>& end,
    const double param, FitType fit_type)
    : start_condition_(start), end_condition_(end) {
  param_ = param;
  switch (fit_type) {
    case FitType::ComputeCoefficients:
      ComputeCoefficients(start_condition_[0], start_condition_[1],
                          start_condition_[2], end[0], end[1], param);
      break;
    case FitType::FitWithEndPointFirstOrder:
      FitWithEndPointFirstOrder(start_condition_[0], start_condition_[1],
                                start_condition_[2], end[0], end[1], param);
      break;
    case FitType::FitWithEndPointSecondOrder:
      FitWithEndPointSecondOrder(start_condition_[0], start_condition_[1],
                                 start_condition_[2], end[0], end[1], param);
      break;
  }
}

QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
    const double x0, const double dx0, const double ddx0, const double dx1,
    const double ddx1, const double param) {
  param_ = param;
  start_condition_[0] = x0;
  start_condition_[1] = dx0;
  start_condition_[2] = ddx0;
  end_condition_[0] = dx1;
  end_condition_[1] = ddx1;
  ComputeCoefficients(x0, dx0, ddx0, dx1, ddx1, param);
}

// QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
//     const QuarticPolynomialCurve1d& other) {
//   param_ = other.param_;
//   coef_ = other.coef_;
// }

double QuarticPolynomialCurve1d::Evaluate(const std::uint32_t order,
                                          const double p) const {
  switch (order) {
    case 0: {
      return (((coef_[4] * p + coef_[3]) * p + coef_[2]) * p + coef_[1]) * p +
             coef_[0];
    }
    case 1: {
      return ((4.0 * coef_[4] * p + 3.0 * coef_[3]) * p + 2.0 * coef_[2]) * p +
             coef_[1];
    }
    case 2: {
      return (12.0 * coef_[4] * p + 6.0 * coef_[3]) * p + 2.0 * coef_[2];
    }
    case 3: {
      return 24.0 * coef_[4] * p + 6.0 * coef_[3];
    }
    case 4: {
      return 24.0 * coef_[4];
    }
    default:
      return 0.0;
  }
}

QuarticPolynomialCurve1d& QuarticPolynomialCurve1d::FitWithEndPointFirstOrder(
    const double x0, const double dx0, const double ddx0, const double x1,
    const double dx1, const double p) {
  CHECK_GT(p, 0.0);

  param_ = p;

  coef_[0] = x0;

  coef_[1] = dx0;

  coef_[2] = 0.5 * ddx0;

  double p2 = p * p;
  double p3 = p2 * p;
  double p4 = p3 * p;

  double b0 = x1 - coef_[0] - coef_[1] * p - coef_[2] * p2;
  double b1 = dx1 - dx0 - ddx0 * p;

  coef_[4] = (b1 * p - 3 * b0) / p4;
  coef_[3] = (4 * b0 - b1 * p) / p3;
  return *this;
}

QuarticPolynomialCurve1d& QuarticPolynomialCurve1d::FitWithEndPointSecondOrder(
    const double x0, const double dx0, const double x1, const double dx1,
    const double ddx1, const double p) {
  CHECK_GT(p, 0.0);

  param_ = p;

  coef_[0] = x0;

  coef_[1] = dx0;

  double p2 = p * p;
  double p3 = p2 * p;
  double p4 = p3 * p;

  double b0 = x1 - coef_[0] - coef_[1] * p;
  double b1 = dx1 - coef_[1];
  double c1 = b1 * p;
  double c2 = ddx1 * p2;

  coef_[2] = (0.5 * c2 - 3 * c1 + 6 * b0) / p2;
  coef_[3] = (-c2 + 5 * c1 - 8 * b0) / p3;
  coef_[4] = (0.5 * c2 - 2 * c1 + 3 * b0) / p4;

  return *this;
}

QuarticPolynomialCurve1d& QuarticPolynomialCurve1d::IntegratedFromCubicCurve(
    const PolynomialCurve1d& other, const double init_value) {
  CHECK_EQ(other.Order(), 3U);
  param_ = other.ParamLength();
  coef_[0] = init_value;
  for (size_t i = 0; i < 4; ++i) {
    coef_.at(i + 1) = other.Coef(i) / (static_cast<double>(i) + 1);
  }
  return *this;
}

QuarticPolynomialCurve1d& QuarticPolynomialCurve1d::DerivedFromQuinticCurve(
    const PolynomialCurve1d& other) {
  CHECK_EQ(other.Order(), 5U);
  param_ = other.ParamLength();
  for (size_t i = 1; i < 6; ++i) {
    coef_.at(i - 1) = other.Coef(i) * static_cast<double>(i);
  }
  return *this;
}

void QuarticPolynomialCurve1d::ComputeCoefficients(
    const double x0, const double dx0, const double ddx0, const double dx1,
    const double ddx1, const double p) {
  CHECK_GT(p, 0.0);

  coef_[0] = x0;
  coef_[1] = dx0;
  coef_[2] = 0.5 * ddx0;

  double b0 = dx1 - ddx0 * p - dx0;
  double b1 = ddx1 - ddx0;

  double p2 = p * p;
  double p3 = p2 * p;

  coef_[3] = (3 * b0 - b1 * p) / (3 * p2);
  coef_[4] = (-2 * b0 + b1 * p) / (4 * p3);
}

std::string QuarticPolynomialCurve1d::ToString() const {
  return absl::StrCat(absl::StrJoin(coef_, "\t"), param_, "\n");
}

double QuarticPolynomialCurve1d::Coef(const size_t order) const {
  CHECK_GT(5U, order);
  return coef_.at(order);
}

std::vector<std::pair<bool, double>>
QuarticPolynomialCurve1d::GetMinAndMaxValue(double min_p, double max_p,
                                            double* min_x,
                                            double* max_x) const {
  std::vector<std::pair<bool, double>> peek_values;
  if (max_x == nullptr || min_x == nullptr) {
    return peek_values;
  }

  peek_values.reserve(3);
  const auto start_x = Evaluate(0, min_p);
  const auto end_x = Evaluate(0, max_p);
  *min_x = fmin(start_x, end_x);
  *max_x = fmax(start_x, end_x);

  const auto a = 4.0 * coef_[4];
  const auto b = 3.0 * coef_[3];
  const auto c = 2.0 * coef_[2];
  const auto d = 1.0 * coef_[1];

  const auto A = b * b - 3 * a * c;
  const auto B = b * c - 9 * a * d;
  const auto C = c * c - 3 * b * d;
  const auto delta = B * B - 4 * A * C;
  double p1 = std::numeric_limits<double>::infinity();
  double p2 = std::numeric_limits<double>::infinity();
  double p3 = std::numeric_limits<double>::infinity();
  if (SeemsEqual(A, 0.0) && SeemsEqual(B, 0.0)) {
    // 当A=B=0时，方程有一个三重实根
    p1 = -b / (3.0 * a);
  } else if (DefinitelyGreater(delta, 0)) {
    // 当delta>0时，方程有一个实根和一对共轭复根
    const auto delta_sqrt = sqrt(delta);
    const auto y1 = A * b + 3 * a * (-B + delta_sqrt) / 2;
    const auto y2 = A * b + 3 * a * (-B - delta_sqrt) / 2;
    p1 = (-b - cbrt(y1) - cbrt(y2)) / (3 * a);
  } else if (SeemsEqual(delta, 0.0)) {
    // 当delta=0时，方程有三个实根，其中有一个二重根
    const auto K = B / A;
    p1 = -b / a + K;
    p2 = -K / 2;
  } else {
    // 当delta<0时，方程有三个不相等的实根
    const auto T = (2 * A * b - 3 * a * B) / (2 * sqrt(A * A * A));
    const auto theta = acos(T);
    const auto sin_theta = sin(theta / 3.0);
    const auto cos_theta = cos(theta / 3.0);
    const auto A_sqrt = sqrt(A);
    p1 = (-b - 2 * A_sqrt * cos_theta) / (3 * a);
    p2 = (-b + A_sqrt * (cos_theta + sqrt(3) * sin_theta)) / (3 * a);
    p3 = (-b + A_sqrt * (cos_theta - sqrt(3) * sin_theta)) / (3 * a);
  }

  if (std::isfinite(p1) && DefinitelyGreater(p1, min_p) &&
      DefinitelyLess(p1, max_p)) {
    const auto dx = Evaluate(0, p1);
    *min_x = fmin(*min_x, dx);
    *max_x = fmax(*max_x, dx);
    if (Evaluate(0, p1 + 1e-2) > dx) {
      peek_values.emplace_back(true, dx);
    } else {
      peek_values.emplace_back(false, dx);
    }
  }

  if (std::isfinite(p2) && DefinitelyGreater(p2, min_p) &&
      DefinitelyLess(p2, max_p)) {
    const auto dx = Evaluate(0, p2);
    *min_x = fmin(*min_x, dx);
    *max_x = fmax(*max_x, dx);
    if (Evaluate(0, p2 + 1e-2) > dx) {
      peek_values.emplace_back(true, dx);
    } else {
      peek_values.emplace_back(false, dx);
    }
  }

  if (std::isfinite(p3) && DefinitelyGreater(p3, min_p) &&
      DefinitelyLess(p3, max_p)) {
    const auto dx = Evaluate(0, p3);
    *min_x = fmin(*min_x, dx);
    *max_x = fmax(*max_x, dx);
    if (Evaluate(0, p3 + 1e-2) > dx) {
      peek_values.emplace_back(true, dx);
    } else {
      peek_values.emplace_back(false, dx);
    }
  }

  return peek_values;
}

}  // namespace planning
}  // namespace TL
