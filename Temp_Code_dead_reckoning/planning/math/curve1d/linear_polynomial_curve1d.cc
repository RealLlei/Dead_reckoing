/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file linear_polynomial_curve1d.cc
 **/

#include "planning/math/curve1d/linear_polynomial_curve1d.h"

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"

#include "common/file/log.h"
#include "common/math/double_type.h"

namespace TL {
namespace planning {

LinearPolynomialCurve1d::LinearPolynomialCurve1d(const double x0,
                                                 const double x1,
                                                 const double param) {
  ComputeCoefficients(x0, x1, param);
  param_ = param;
}

void LinearPolynomialCurve1d::DerivedFromQuadraticCurve(
    const PolynomialCurve1d& other) {
  CHECK_EQ(other.Order(), 2U);
  param_ = other.ParamLength();
  for (size_t i = 1; i < 3; ++i) {
    coef_.at(i - 1) = other.Coef(i) * static_cast<double>(i);
  }
}

double LinearPolynomialCurve1d::Evaluate(const std::uint32_t order,
                                         const double p) const {
  switch (order) {
    case 0: {
      return coef_[1] * p + coef_[0];
    }
    case 1: {
      return coef_[1];
    }
    default:
      return 0.0;
  }
}

std::string LinearPolynomialCurve1d::ToString() const {
  return absl::StrCat(absl::StrJoin(coef_, "\t"), param_, "\n");
}

void LinearPolynomialCurve1d::ComputeCoefficients(const double x0,
                                                  const double x1,
                                                  const double param) {
  DCHECK(param > 0.0);
  coef_[0] = x0;
  coef_[1] = (x1 - x0) / param;
}

double LinearPolynomialCurve1d::Coef(const size_t order) const {
  CHECK_GT(4U, order);
  return coef_.at(order);
}

void LinearPolynomialCurve1d::GetMinAndMaxValue(double min_p, double max_p,
                                                double* min_x,
                                                double* max_x) const {
  if (max_x == nullptr || min_x == nullptr) {
    return;
  }

  const auto start_x = Evaluate(0, min_p);
  const auto end_x = Evaluate(0, max_p);
  *min_x = fmin(start_x, end_x);
  *max_x = fmax(start_x, end_x);
}

}  // namespace planning
}  // namespace TL
