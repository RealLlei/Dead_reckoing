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

// 1D quartic polynomial curve: (x0, dx0, ddx0) -- [0, param] --> (dx1, ddx1)
class QuarticPolynomialCurve1d : public PolynomialCurve1d {
 public:
  enum class FitType {
    ComputeCoefficients,
    FitWithEndPointFirstOrder,
    FitWithEndPointSecondOrder
  };

  QuarticPolynomialCurve1d() = default;

  QuarticPolynomialCurve1d(const std::array<double, 3>& start,
                           const std::array<double, 2>& end, double param,
                           FitType fit_type = FitType::ComputeCoefficients);

  QuarticPolynomialCurve1d(double x0, double dx0, double ddx0, double dx1,
                           double ddx1, double param);

  // QuarticPolynomialCurve1d(const QuarticPolynomialCurve1d& other);

  ~QuarticPolynomialCurve1d() override = default;

  double Evaluate(std::uint32_t order, double p) const override;

  double Integral(const double p) const {
    return ((((coef_[4] * 0.2 * p + coef_[3] * 0.25) * p + coef_[2] / 3) * p +
             coef_[1] * 0.5) *
                p +
            coef_[0]) *
           p;
  }

  /**
   * Interface with refine quartic polynomial by meets end first order
   * and start second order boundary condition:
   * @param  x0    init point x location
   * @param  dx0   init point derivative
   * @param  ddx0  init point second order derivative
   * @param  x1    end point x location
   * @param  dx1   end point derivative
   * @param  param parameter length
   * @return       self
   */
  QuarticPolynomialCurve1d& FitWithEndPointFirstOrder(double x0, double dx0,
                                                      double ddx0, double x1,
                                                      double dx1, double param);

  /**
   * Interface with refine quartic polynomial by meets end point second order
   * and start point first order boundary condition
   */
  QuarticPolynomialCurve1d& FitWithEndPointSecondOrder(double x0, double dx0,
                                                       double x1, double dx1,
                                                       double ddx1,
                                                       double param);

  /*
   * Integrated from cubic curve with init value
   */
  QuarticPolynomialCurve1d& IntegratedFromCubicCurve(
      const PolynomialCurve1d& other, double init_value);

  /*
   * Derived from quintic curve
   */
  QuarticPolynomialCurve1d& DerivedFromQuinticCurve(
      const PolynomialCurve1d& other);

  double ParamLength() const override { return param_; }

  std::string ToString() const override;

  double Coef(size_t order) const override;

  void SetCoef(const size_t order, const double coef) override {
    coef_.at(order) = coef;
  }

  size_t Order() const override { return 4; }

  std::vector<std::pair<bool, double>> GetMinAndMaxValue(double min_p,
                                                         double max_p,
                                                         double* min_x,
                                                         double* max_x) const;

  void ComputeCoefficients(double x0, double dx0, double ddx0, double dx1,
                           double ddx1, double param);

 private:
  std::array<double, 5> coef_ = {{0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> start_condition_ = {{0.0, 0.0, 0.0}};
  std::array<double, 2> end_condition_ = {{0.0, 0.0}};
};

}  // namespace planning
}  // namespace TL
