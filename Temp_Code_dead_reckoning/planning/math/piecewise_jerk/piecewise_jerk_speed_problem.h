/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <utility>
#include <vector>

#include "planning/math/piecewise_jerk/piecewise_jerk_problem.h"

namespace TL {
namespace planning {

/*
 * @brief:
 * This class solve the path time optimization problem:
 * s
 * |
 * |                       P(t1, s1)  P(t2, s2)
 * |            P(t0, s0)                       ... P(t(k-1), s(k-1))
 * |P(start)
 * |
 * |________________________________________________________ t
 *
 * we suppose t(k+1) - t(k) == t(k) - t(k-1)
 *
 * Given the s, s', s'' at P(start),  The goal is to find t0, t1, ... t(k-1)
 * which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */

class PiecewiseJerkSpeedProblem : public PiecewiseJerkProblem {
 public:
  PiecewiseJerkSpeedProblem(size_t num_of_knots, double delta_s,
                            const std::array<double, 3>& x_init);

  virtual ~PiecewiseJerkSpeedProblem() = default;

  void SetDxRef(double weight_dx_ref, double dx_ref);

  void SetPenaltyDx(std::vector<double> penalty_dx);

 protected:
  // naming convention follows osqp solver.
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr) override;

  void CalculateOffset(std::vector<c_float>* q) override;

  /**
   * @brief Calculate Affine Constraint
   * 
   * @param A_data 
   * @param A_indices 
   * @param A_indptr 
   * @param lower_bounds 
   * @param upper_bounds 
   */
  void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                 std::vector<c_int>* A_indices,
                                 std::vector<c_int>* A_indptr,
                                 std::vector<c_float>* lower_bounds,
                                 std::vector<c_float>* upper_bounds) override;

  bool has_dx_ref_ = false;
  double weight_dx_ref_ = 0.0;
  double dx_ref_ = 0.0;

  std::vector<double> penalty_dx_;
};

}  // namespace planning
}  // namespace TL
