/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  sl_curve.h
 */

#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace TL {
namespace planning {
using State = std::array<double, 3>;
using Condition = std::pair<State, double>;

class alignas(CACHELINE_SIZE) SlCurve {
 public:
  SlCurve() = default;
  ~SlCurve() = default;

  /**
   * @brief InitPiecewiseSlCurve.
   * @param init_lat_state 
   * @param l_conditions 
   */
  bool InitPiecewiseSlCurve(const std::array<double, 3>& init_lat_state,
                            const std::vector<Condition>& l_conditions);

  /**
   * @brief Evaluate.
   * @param order 
   * @param relative_s 
   */
  double Evaluate(uint32_t order, double relative_s) const;

  [[nodiscard]] std::string DebugString() const;

 private:
  std::vector<Condition> l_conditions_ = {};
  std::vector<QuinticPolynomialCurve1d> l_pieces_;
  std::vector<double> accumulated_s_ = {};

  double end_l_ = 0.0;
};

}  // namespace planning
}  // namespace TL
