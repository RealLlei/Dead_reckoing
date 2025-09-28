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
 * @file quartic_polynomial_curve1d.h
 **/

#pragma once

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "planning/math/curve1d/polynomial_curve1d.h"

namespace TL {
namespace planning {

// 1D quadratic polynomial curve: (x0, dx0, ddx0) -- [0, param] --> (dx1, ddx1)
class QuadraticPolynomialCurve1d : public PolynomialCurve1d {
 public:
  QuadraticPolynomialCurve1d() = default;

  QuadraticPolynomialCurve1d(const std::array<double, 3>& start, double param);

  explicit QuadraticPolynomialCurve1d(const std::array<double, 3>& coef);

  virtual ~QuadraticPolynomialCurve1d() = default;

  void DerivedFromCubicCurve(const PolynomialCurve1d& other);

  double Evaluate(std::uint32_t order, double p) const override;

  double ParamLength() const override { return param_; }

  std::string ToString() const override;

  double Coef(size_t order) const override;

  void SetCoef(const size_t order, const double coef) override {
    coef_.at(order) = coef;
  }

  void Init(const double a0, const double a1, const double a2) {
    coef_[0] = a0;
    coef_[1] = a1;
    coef_[2] = a2;
  }

  /**
   * @brief Get the Peak Value
   *
   * @param min_p
   * @param max_p
   * @return double when p is in (min_p, max_p), if there there is a peak value,
   * return it, else return inf
   */
  double GetPeakValue(double min_p, double max_p) const;

  size_t Order() const override { return 2; }

  const std::array<double, 3>& end_condition() { return end_condition_; }

  double Integral(const double p) const {
    return ((coef_[2] / 3 * p + coef_[1] / 2) * p + coef_[0]) * p;
  }

  std::vector<double> Solve(double p) const;

  /**
   * @brief 
   * 
   * @return std::vector<double> 
   */
  bool Solve(std::vector<double>* root) const;

  std::vector<std::pair<bool, double>> GetMinAndMaxValue(double min_p,
                                                         double max_p,
                                                         double* min_x,
                                                         double* max_x) const;

 private:
  void ComputeCoefficients(double x0, double dx0, double ddx0, double param);

  std::array<double, 3> coef_ = {{0.0, 0.0, 0.0}};
  std::array<double, 3> start_condition_ = {{0.0, 0.0, 0.0}};
  std::array<double, 3> end_condition_ = {{0.0, 0.0, 0.0}};
};

}  // namespace planning
}  // namespace TL
