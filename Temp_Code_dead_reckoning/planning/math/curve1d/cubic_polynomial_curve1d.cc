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

#include "planning/math/curve1d/cubic_polynomial_curve1d.h"

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"

#include "common/file/log.h"
#include "common/math/double_type.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreater;
using common::math::double_type::DefinitelyLess;

CubicPolynomialCurve1d::CubicPolynomialCurve1d(
    const std::array<double, 3>& start, const double end, const double param)
    : CubicPolynomialCurve1d(start[0], start[1], start[2], end, param) {}

CubicPolynomialCurve1d::CubicPolynomialCurve1d(const double x0,
                                               const double dx0,
                                               const double ddx0,
                                               const double x1,
                                               const double param)
    : end_condition_(x1) {
  ComputeCoefficients(x0, dx0, ddx0, x1, param);
  param_ = param;
  start_condition_[0] = x0;
  start_condition_[1] = dx0;
  start_condition_[2] = ddx0;
}

void CubicPolynomialCurve1d::DerivedFromQuarticCurve(
    const PolynomialCurve1d& other) {
  CHECK_EQ(other.Order(), 4U);
  param_ = other.ParamLength();
  for (size_t i = 1; i < 5; ++i) {
    coef_.at(i - 1) = other.Coef(i) * static_cast<double>(i);
  }
}

double CubicPolynomialCurve1d::Evaluate(const std::uint32_t order,
                                        const double p) const {
  switch (order) {
    case 0: {
      return ((coef_[3] * p + coef_[2]) * p + coef_[1]) * p + coef_[0];
    }
    case 1: {
      return (3.0 * coef_[3] * p + 2.0 * coef_[2]) * p + coef_[1];
    }
    case 2: {
      return 6.0 * coef_[3] * p + 2.0 * coef_[2];
    }
    case 3: {
      return 6.0 * coef_[3];
    }
    default:
      return 0.0;
  }
}

std::string CubicPolynomialCurve1d::ToString() const {
  return absl::StrCat(absl::StrJoin(coef_, "\t"), param_, "\n");
}

void CubicPolynomialCurve1d::ComputeCoefficients(const double x0,
                                                 const double dx0,
                                                 const double ddx0,
                                                 const double x1,
                                                 const double param) {
  DCHECK(param > 0.0);
  const double p2 = param * param;
  const double p3 = param * p2;
  coef_[0] = x0;
  coef_[1] = dx0;
  coef_[2] = 0.5 * ddx0;
  coef_[3] = (x1 - x0 - dx0 * param - coef_[2] * p2) / p3;
}

double CubicPolynomialCurve1d::Coef(const size_t order) const {
  CHECK_GT(4U, order);
  return coef_.at(order);
}

std::vector<std::pair<bool, double>> CubicPolynomialCurve1d::GetMinAndMaxValue(
    double min_p, double max_p, double* min_x, double* max_x) const {
  std::vector<std::pair<bool, double>> peek_values;
  if (max_x == nullptr || min_x == nullptr) {
    return peek_values;
  }

  peek_values.reserve(2);
  const auto start_x = Evaluate(0, min_p);
  const auto end_x = Evaluate(0, max_p);
  *min_x = fmin(start_x, end_x);
  *max_x = fmax(start_x, end_x);

  const auto a = 3.0 * coef_[3];
  const auto b = 2.0 * coef_[2];
  const auto c = 1.0 * coef_[1];
  const auto temp = sqrt(b * b - 4.0 * a * c);
  const auto p1 = (-b + temp) / (2 * a);
  const auto p2 = (-b - temp) / (2 * a);

  if (std::isfinite(p1) && DefinitelyGreater(p1, min_p) &&
      DefinitelyLess(p1, max_p)) {
    const auto ddx = Evaluate(0, p1);
    *min_x = fmin(*min_x, ddx);
    *max_x = fmax(*max_x, ddx);
    if (Evaluate(0, p1 + 1e-2) > ddx) {
      peek_values.emplace_back(true, ddx);
    } else {
      peek_values.emplace_back(false, ddx);
    }
  }

  if (std::isfinite(p2) && DefinitelyGreater(p2, min_p) &&
      DefinitelyLess(p2, max_p)) {
    const auto ddx = Evaluate(0, p2);
    *min_x = fmin(*min_x, ddx);
    *max_x = fmax(*max_x, ddx);
    if (Evaluate(0, p2 + 1e-2) > ddx) {
      peek_values.emplace_back(true, ddx);
    } else {
      peek_values.emplace_back(false, ddx);
    }
  }

  return peek_values;
}

}  // namespace planning
}  // namespace TL
