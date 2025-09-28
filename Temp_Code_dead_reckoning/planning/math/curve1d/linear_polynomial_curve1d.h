/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file linear_polynomial_curve1d.h
 **/

#pragma once

#include <array>
#include <string>

#include "planning/math/curve1d/polynomial_curve1d.h"

namespace TL {
namespace planning {

/**
 * @brief LinearPolynomialCurve1d
 * 
 */
class LinearPolynomialCurve1d : public PolynomialCurve1d {
 public:
  LinearPolynomialCurve1d() = default;
  ~LinearPolynomialCurve1d() override = default;

  /**
   * x0 is the value when f(x = 0);
   * x1 is the value when f(x = param);
   */
  LinearPolynomialCurve1d(double x0, double x1, double param);

  /**
   * @brief Get coef derived from quadratic curve
   * 
   * @param other 
   */
  void DerivedFromQuadraticCurve(const PolynomialCurve1d& other);

  /**
   * @brief If order = 0, Get f(p), if order = 1, Get f'(p)
   * 
   * @param order 
   * @param p 
   * @return double 
   */
  double Evaluate(std::uint32_t order, double p) const override;

  /**
   * @brief Get param length
   * 
   * @return double 
   */
  double ParamLength() const override { return param_; }

  /**
   * @brief Get debug string
   * 
   * @return std::string 
   */
  std::string ToString() const override;

  /**
   * @brief Get curve coef
   * 
   * @param order 
   * @return double 
   */
  double Coef(size_t order) const override;

  /**
   * @brief Get curve order
   * 
   * @return size_t 
   */
  size_t Order() const override { return 1; }

  /**
   * @brief Get the min and max f(p) when p is in [min_p, max_p]
   * p
   * @param min_p 
   * @param max_p 
   * @param min_x 
   * @param max_x 
   */
  void GetMinAndMaxValue(double min_p, double max_p, double* min_x,
                         double* max_x) const;

 private:
  /**
   * @brief Compute coef_
   * 
   * @param x0 
   * @param x1 
   * @param param 
   */
  void ComputeCoefficients(double x0, double x1, double param);

  std::array<double, 2> coef_ = {{0.0, 0.0}};
};

}  // namespace planning
}  // namespace TL
