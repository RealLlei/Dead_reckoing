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

#include <mutex>
#include <utility>
#include <vector>

#include "osqp/osqp.h"

namespace TL {
namespace planning {

class FemPosDeviationSqpOsqpInterface {
 public:
  FemPosDeviationSqpOsqpInterface() = default;

  virtual ~FemPosDeviationSqpOsqpInterface() = default;

  void SetRefPoints(const std::vector<std::pair<double, double>>& ref_points) {
    ref_points_ = ref_points;
  }

  void SetBoundsAroundRefs(const std::vector<double>& bounds_around_refs) {
    bounds_around_refs_ = bounds_around_refs;
  }

  void SetWeightFemPosDeviation(const double weight_fem_pos_deviation) {
    weight_fem_pos_deviation_ = weight_fem_pos_deviation;
  }

  void SetWeightPathLength(const double weight_path_length) {
    weight_path_length_ = weight_path_length;
  }

  void SetWeightRefDeviation(const double weight_ref_deviation) {
    weight_ref_deviation_ = weight_ref_deviation;
  }

  void SetWeightCurvatureConstraintSlackVar(
      const double weight_curvature_constraint_slack_var) {
    weight_curvature_constraint_slack_var_ =
        weight_curvature_constraint_slack_var;
  }

  void SetCurvatureConstraint(const double curvature_constraint) {
    curvature_constraint_ = curvature_constraint;
  }

  void SetMaxIter(const int max_iter) { max_iter_ = max_iter; }

  void SetTimeLimit(const double time_limit) { time_limit_ = time_limit; }

  void SetVerbose(const bool verbose) { verbose_ = verbose; }

  void SetScaledTermination(const bool scaled_termination) {
    scaled_termination_ = scaled_termination;
  }

  void SetWarmStart(const bool warm_start) { warm_start_ = warm_start; }

  void SetSqpPenMaxIter(const int sqp_pen_max_iter) {
    sqp_pen_max_iter_ = sqp_pen_max_iter;
  }

  void SetSqpFtol(const double sqp_ftol) { sqp_ftol_ = sqp_ftol; }

  void SetSqpSubMaxIter(const int sqp_sub_max_iter) {
    sqp_sub_max_iter_ = sqp_sub_max_iter;
  }

  void SetSqpCtol(const double sqp_ctol) { sqp_ctol_ = sqp_ctol; }

  bool Solve();

  const std::vector<std::pair<double, double>>& OptXY() const {
    return opt_xy_;
  }

 private:
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr);

  void CalculateOffset(std::vector<c_float>* q);

  std::vector<double> CalculateLinearizedFemPosParams(
      const std::vector<std::pair<double, double>>& points, const size_t index);

  void CalculateAffineConstraint(
      const std::vector<std::pair<double, double>>& points,
      std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
      std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
      std::vector<c_float>* upper_bounds);

  void SetPrimalWarmStart(const std::vector<std::pair<double, double>>& points,
                          std::vector<c_float>* primal_warm_start);

  bool OptimizeWithOsqp(const std::vector<c_float>& primal_warm_start,
                        OSQPWorkspace** work);

  double CalculateConstraintViolation(
      const std::vector<std::pair<double, double>>& points);

 private:
  // Init states and constraints
  std::vector<std::pair<double, double>> ref_points_;
  std::vector<double> bounds_around_refs_;
  double curvature_constraint_ = 0.2;

  // Weights in optimization cost function
  double weight_fem_pos_deviation_ = 1.0e5;
  double weight_path_length_ = 1.0;
  double weight_ref_deviation_ = 1.0;
  double weight_curvature_constraint_slack_var_ = 1.0e5;

  // Settings of osqp
  int max_iter_ = 4000;
  double time_limit_ = 0.0;
  bool verbose_ = false;
  bool scaled_termination_ = true;
  bool warm_start_ = true;

  // Settings of sqp
  int sqp_pen_max_iter_ = 100;
  double sqp_ftol_ = 1e-2;
  int sqp_sub_max_iter_ = 100;
  double sqp_ctol_ = 1e-2;

  // Optimization problem definitions
  int num_of_points_ = 0;
  int num_of_pos_variables_ = 0;
  int num_of_slack_variables_ = 0;
  int num_of_variables_ = 0;
  int num_of_variable_constraints_ = 0;
  int num_of_curvature_constraints_ = 0;
  int num_of_constraints_ = 0;

  // Optimized_result
  std::vector<std::pair<double, double>> opt_xy_;
  std::vector<double> slack_;
  double average_interval_length_ = 0.0;
};
}  // namespace planning
}  // namespace TL
