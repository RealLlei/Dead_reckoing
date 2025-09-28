/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  planning piecewise jerk path problem
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>
#include "common/configs/vehicle_config_helper.h"
#include "planning/math/piecewise_jerk/piecewise_jerk_problem.h"

namespace TL {
namespace planning {

/*
 * @brief:
 * FEM stands for finite element method.
 * This class solve an optimization problem:
 * x
 * |
 * |                       P(s1, x1)  P(s2, x2)
 * |            P(s0, x0)                       ... P(s(k-1), x(k-1))
 * |P(start)
 * |
 * |________________________________________________________ s
 *
 * we suppose s(k+1) - s(k) == s(k) - s(k-1)
 *
 * Given the x, x', x'' at P(start),  The goal is to find x0, x1, ... x(k-1)
 * which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */

class PiecewiseJerkPathProblem : public PiecewiseJerkProblem {
 public:
  PiecewiseJerkPathProblem(size_t num_of_knots, double delta_s,
                           const std::array<double, 3>& x_init);

  PiecewiseJerkPathProblem(size_t num_of_knots, double delta_s,
                           const std::array<double, 3>& x_init,
                           bool forward_plan, bool enable_curvature,
                           bool enable_curvature_derivative,
                           bool enable_collision_constraint = false);

  ~PiecewiseJerkPathProblem() override = default;

  /**
   * @brief P Matrix l kernel
   * 
   * @param p 
   */
  void PMatrixLKernel(
      std::vector<std::vector<std::pair<c_int, c_float>>>* p) const;

  /**
   * @brief P Matrix dl kernel
   * 
   * @param p 
   */
  void PMatrixDlKernel(
      std::vector<std::vector<std::pair<c_int, c_float>>>* p) const;

  /**
   * @brief P Matrix ddl kernel
   * 
   * @param p 
   */
  void PMatrixDdlKernel(
      std::vector<std::vector<std::pair<c_int, c_float>>>* p) const;

  /**
   * @brief P Matrix dddl kernel
   * 
   * @param p 
   */
  void PMatrixDddlKernel(
      std::vector<std::vector<std::pair<c_int, c_float>>>* p) const;

  /**
   * @brief P Matrix soft l constraint kernel
   * 
   * @param p 
   */
  void PMatrixSoftLConstraintKernel(
      std::vector<std::vector<std::pair<c_int, c_float>>>* p) const;

  /**
   * @brief Set the kappa ref object
   * 
   * @param kappa_ref 
   */
  void SetKappaRef(std::vector<double>&& kappa_ref) {
    kappa_ref_ = std::move(kappa_ref);
  }

  /**
   * @brief Set the dkappa ref object
   * 
   * @param dkappa_ref 
   */
  void SetDkappaRef(std::vector<double>&& dkappa_ref) {
    dkappa_ref_ = std::move(dkappa_ref);
  }

  /**
   * @brief Set the max kappa object
   * 
   * @param max_kappa 
   */
  void SetMaxKappa(const double max_kappa) { max_kappa_ = max_kappa; }

  /**
   * @brief Set the max dkappa object
   * 
   * @param max_dkappa 
   */
  void SetMaxDkappa(double max_dkappa) { max_dkappa_ = max_dkappa; }

  /**
   * @brief Set the weight soft l constraint object
   * 
   * @param weight_soft_l_constraint 
   */
  void SetWeightSoftLConstraint(const double weight_soft_l_constraint) {
    weight_soft_l_constraint_ = weight_soft_l_constraint;
  }

  /**
   * @brief Set the Enable Q Matrix Start End State Offset_ object
   * 
   * @param enable_q_matrix_start_end_state_offset 
   */
  void SetEnableQMatrixStartEndStateOffset(
      const bool enable_q_matrix_start_end_state_offset) {
    enable_q_matrix_start_end_state_offset_ =
        enable_q_matrix_start_end_state_offset;
  }

  /**
   * @brief Set the Enable Q Matrix Ddl Offset object
   * 
   * @param enable_q_matrix_ddl_offset 
   */
  void SetEnableQMatrixDdlOffset(const bool enable_q_matrix_ddl_offset) {
    enable_q_matrix_ddl_offset_ = enable_q_matrix_ddl_offset;
  }

 protected:
  /**
   * @brief Calculate Kernel
   * 
   * @param P_data 
   * @param P_indices 
   * @param P_indptr 
   */
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr) override;

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

  /**
   * @brief A matrix basic constraint
   * 
   * @param variables 
   * @param lower_bounds 
   * @param upper_bounds 
   * @param constraint_index 
   * @return true 
   * @return false 
   */
  bool AMatrixBasicConstraint(
      std::vector<std::vector<std::pair<c_int, c_float>>>* variables,
      std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds,
      int* constraint_index);

  /**
   * @brief A matrix start end state constraint
   * 
   * @param variables 
   * @param lower_bounds 
   * @param upper_bounds 
   * @param constraint_index 
   * @return true 
   * @return false 
   */
  bool AMatrixStartEndStateConstraint(
      std::vector<std::vector<std::pair<c_int, c_float>>>* variables,
      std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds,
      int* constraint_index);

  /**
   * @brief add collision constraints at the corner points of the vehicle
   * 
   * @param n number of path points
   * @param point_dx_flu x coordinate of corner point at ego vehicle flu
   * @param variables A matrix, containing linearized constraints coefficients
   * @param lower_bounds b vector, lower bound
   * @param upper_bounds u vector, upper bound
   * @param constraint_index number of constraints
   */
  bool AMatrixCollisionConstraint(
      size_t n, double point_dx_flu,
      std::vector<std::vector<std::pair<c_int, c_float>>>* variables,
      std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds,
      int* constraint_index);

  /**
   * @brief A matrix curvature constranint process
   * 
   * @param variables 
   * @param lower_bounds 
   * @param upper_bounds 
   * @param constraint_index 
   * @return true 
   * @return false 
   */
  bool AMatrixCurvatureConstraint(
      std::vector<std::vector<std::pair<c_int, c_float>>>* variables,
      std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds,
      int* constraint_index);

  /**
   * @brief A matrix curvature derivative constranint process
   * 
   * @param variables 
   * @param lower_bounds 
   * @param upper_bounds 
   * @param constraint_index 
   * @return true 
   * @return false 
   */
  bool AMatrixCurvatureDerivativeConstraint(
      std::vector<std::vector<std::pair<c_int, c_float>>>* variables,
      std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds,
      int* constraint_index);

  /**
   * @brief Calculate Offset
   * 
   * @param q 
   */
  void CalculateOffset(std::vector<c_float>* q) override;

  /**
   * @brief Q Matrix L offset process
   * 
   * @param q 
   * @return true 
   * @return false 
   */
  bool QMatrixLOffset(std::vector<c_float>* q);

  /**
   * @brief Q Matrix Ddl offset
   * 
   * @param q
   * @return true 
   * @return false 
   */
  bool QMatrixDdlOffset(std::vector<c_float>* q);

  /**
   * @brief Q Matrix Start end state offset process
   * 
   * @param q 
   * @return true 
   * @return false 
   */
  bool QMatrixStartEndStateOffset(std::vector<c_float>* q);

  /**
   * @brief Matrix debug
   * 
   * @param name 
   * @param matrix 
   */
  static void MatrixDebug(const std::string& name,
                          const std::vector<std::vector<c_float>>& matrix);

 private:
  bool enable_collision_constraint_ = false;
  bool forward_plan_ = true;
  bool enable_curvature_ = false;
  bool enable_curvature_derivative_ = false;
  bool enable_q_matrix_start_end_state_offset_ = false;
  bool enable_q_matrix_ddl_offset_ = false;
  std::vector<double> kappa_ref_;
  std::vector<double> dkappa_ref_;
  double max_kappa_ = 0.2;
  double max_dkappa_ = 0.1;
  double weight_soft_l_constraint_ = 1e7;
};

}  // namespace planning
}  // namespace TL
