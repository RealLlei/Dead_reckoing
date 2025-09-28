/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  planning piecewise jerk problem
 */

#pragma once

#include <cstring>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "common/util/string_util.h"
#include "osqp/osqp.h"

namespace TL {
namespace planning {

/*
 * @brief:
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

class PiecewiseJerkProblem {
 public:
  PiecewiseJerkProblem(size_t num_of_knots, double delta_s,
                       const std::array<double, 3>& x_init);

  virtual ~PiecewiseJerkProblem() = default;

  /**
   * @brief Set x bounds
   * 
   * @param x_bounds 
   */
  void SetXBounds(std::vector<std::pair<double, double>> x_bounds);

  /**
   * @brief Set x bounds
   * 
   * @param x_lower_bound 
   * @param x_upper_bound 
   */
  void SetXBounds(double x_lower_bound, double x_upper_bound);

  /**
   * @brief Set the Dx Bounds object
   * 
   * @param dx_bounds 
   */
  void SetDxBounds(std::vector<std::pair<double, double>> dx_bounds);

  /**
   * @brief Set the Dx Bounds object
   * 
   * @param dx_lower_bound 
   * @param dx_upper_bound 
   */
  void SetDxBounds(double dx_lower_bound, double dx_upper_bound);

  /**
   * @brief Set the Ddx Bounds object
   * 
   * @param ddx_bounds 
   */
  void SetDdxBounds(std::vector<std::pair<double, double>> ddx_bounds);

  /**
   * @brief Set the Ddx Bounds object
   * 
   * @param ddx_lower_bound 
   * @param ddx_upper_bound 
   */
  void SetDdxBounds(double ddx_lower_bound, double ddx_upper_bound);

  /**
   * @brief Set the Dddx Bounds object
   * 
   * @param dddx_bound 
   */
  void SetDddxBounds(double dddx_bound);

  /**
   * @brief Set the Dddx Bounds object
   * 
   * @param dddx_lower_bound 
   * @param dddx_upper_bound 
   */
  void SetDddxBounds(double dddx_lower_bound, double dddx_upper_bound);

  /**
   * @brief Set Dddx Bounds
   * 
   * @param dddx_bounds 
   */
  void SetDddxBounds(std::vector<std::pair<double, double>> dddx_bounds);

  /**
   * @brief Set the Weight X object
   * 
   * @param weight_x 
   */
  void SetWeightX(const double weight_x) { weight_x_ = weight_x; }

  /**
   * @brief Set the Weight Dx object
   * 
   * @param weight_dx 
   */
  void SetWeightDx(const double weight_dx) { weight_dx_ = weight_dx; }

  /**
   * @brief Set the Weight Ddx object
   * 
   * @param weight_ddx 
   */
  void SetWeightDdx(const double weight_ddx) { weight_ddx_ = weight_ddx; }

  /**
   * @brief Set the Weight Dddx object
   * 
   * @param weight_dddx 
   */
  void SetWeightDddx(const double weight_dddx) { weight_dddx_ = weight_dddx; }

  /**
   * @brief Set the Weight Ddx object
   * 
   * @param weight_ddx_offset 
   */
  void SetWeightDdxOffset(const double weight_ddx_offset) {
    weight_ddx_offset_ = weight_ddx_offset;
  }

  /**
   * @brief Set the Scale Factor object
   * 
   * @param scale_factor 
   */
  void SetScaleFactor(const std::array<double, 3>& scale_factor) {
    scale_factor_ = scale_factor;
  }

  /**
   * @brief Set the x ref object and the uniform x_ref weighting
   *
   * @param weight_x_ref: uniform weighting for x_ref
   * @param x_ref: objective value of x
   */
  void SetXRef(double weight_x_ref, std::vector<double> x_ref);

  /**
   * @brief Set the x ref object and piecewised x_ref weightings
   *
   * @param weight_x_ref_vec: piecewised x_ref weightings
   * @param x_ref: objective value of x
   */
  void SetXRef(std::vector<double> weight_x_ref_vec, std::vector<double> x_ref);

  /**
   * @brief Set the Start State Ref object
   * 
   * @param weight_start_state 
   * @param start_state_ref 
   */
  void SetStartStateRef(const std::array<double, 3>& weight_start_state,
                        const std::array<double, 3>& start_state_ref);

  /**
   * @brief Set the End State Ref object
   * 
   * @param weight_end_state 
   * @param end_state_ref 
   */
  void SetEndStateRef(const std::array<double, 3>& weight_end_state,
                      const std::array<double, 3>& end_state_ref);

  /**
   * @brief Optimize
   * 
   * @param name 
   * @return true 
   * @return false 
   */
  virtual bool Optimize(const std::string& name);

  /**
   * @brief Opt X
   * 
   * @return const std::vector<double>& 
   */
  [[nodiscard]] const std::vector<double>& OptX() const { return x_; }

  /**
   * @brief Opt Dx
   * 
   * @return const std::vector<double>& 
   */
  [[nodiscard]] const std::vector<double>& OptDx() const { return dx_; }

  /**
   * @brief Opt Ddx
   * 
   * @return const std::vector<double>& 
   */
  [[nodiscard]] const std::vector<double>& OptDdx() const { return ddx_; }

  // OSQP settings
  /**
   * @brief Set the Osqp Verbose object
   * 
   * @param verbose 
   */
  void SetOsqpVerbose(const bool verbose) { verbose_ = verbose; }

  /**
   * @brief Set the Osqp Eps Abs object
   * 
   * @param eps_abs 
   */
  void SetOsqpEpsAbs(const double eps_abs) { eps_abs_ = eps_abs; }

  /**
   * @brief Set the Osqp Eps Rel object
   * 
   * @param eps_rel 
   */
  void SetOsqpEpsRel(const double eps_rel) { eps_rel_ = eps_rel; }

  /**
   * @brief Set the Osqp Eps Prim Inf object
   * 
   * @param eps_prim_inf 
   */
  void SetOsqpEpsPrimInf(const double eps_prim_inf) {
    eps_prim_inf_ = eps_prim_inf;
  }

  /**
   * @brief Set the Osqp Eps Dual Inf object
   * 
   * @param eps_dual_inf 
   */
  void SetOsqpEpsDualInf(const double eps_dual_inf) {
    eps_dual_inf_ = eps_dual_inf;
  }

  /**
   * @brief Set the Osqp Scaled Termination object
   * 
   * @param scaled_termination 
   */
  void SetOsqpScaledTermination(const bool scaled_termination) {
    scaled_termination_ = scaled_termination;
  }

  /**
   * @brief Set the Osqp Enable Adaptive Rho object
   * 
   * @param adaptive_rho 
   */
  void SetOsqpEnableAdaptiveRho(const bool adaptive_rho) {
    adaptive_rho_ = adaptive_rho;
  }

  /**
   * @brief Set the Osqp Polish object
   * 
   * @param polish 
   */
  void SetOsqpPolish(const bool polish) { polish_ = polish; }

  /**
   * @brief Set the Osqp Polish Refine Iter object
   * 
   * @param polish_refine_iter 
   */
  void SetOsqpPolishRefineIter(const int polish_refine_iter) {
    polish_refine_iter_ = polish_refine_iter;
  }

  /**
   * @brief Set the Osqp Max Iter object
   * 
   * @param max_iter 
   */
  void SetOsqpMaxIter(const int max_iter) { max_iter_ = max_iter; }

  /**
   * @brief Set the Osqp Alpha object
   * 
   * @param alpha 
   */
  void SetOsqpAlpha(const double alpha) { alpha_ = alpha; }

  /**
   * @brief Set the Osqp Adaptive Rho Interval object
   * 
   * @param adaptive_rho_interval 
   */
  void SetOsqpAdaptiveRhoInterval(const int adaptive_rho_interval) {
    adaptive_rho_interval_ = adaptive_rho_interval;
  }

  /**
   * @brief Set the Osqp Time Limit object
   * 
   * @param time_limit 
   */
  void SetOsqpTimeLimit(const double time_limit) { time_limit_ = time_limit; }

 protected:
  // naming convention follows osqp solver.
  /**
   * @brief Calculate Kernel
   * 
   * @param P_data 
   * @param P_indices 
   * @param P_indptr 
   */
  virtual void CalculateKernel(std::vector<c_float>* P_data,
                               std::vector<c_int>* P_indices,
                               std::vector<c_int>* P_indptr) = 0;

  /**
   * @brief Calculate Offset
   * 
   * @param q 
   */
  virtual void CalculateOffset(std::vector<c_float>* q) = 0;

  /**
   * @brief Calculate Affine Constraint
   * 
   * @param A_data 
   * @param A_indices 
   * @param A_indptr 
   * @param lower_bounds 
   * @param upper_bounds 
   */
  virtual void CalculateAffineConstraint(
      std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
      std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
      std::vector<c_float>* upper_bounds) = 0;

  /**
   * @brief Solver Default Settings
   * 
   * @return OSQPSettings* 
   */
  virtual OSQPSettings* SolverDefaultSettings();

  /**
   * @brief Formulate Problem
   * 
   * @return OSQPData* 
   */
  OSQPData* FormulateProblem();

  /**
   * @brief Free Osqp Data
   * 
   * @param data 
   * @param settings 
   * @param osqp_work 
   */
  static void FreeOsqpData(OSQPData* data, OSQPSettings* settings,
                           OSQPWorkspace* osqp_work);

  template <typename T>
  T* CopyData(const std::vector<T>& vec) {
    T* data = new T[vec.size()];  //NOLINT
    memcpy(data, vec.data(), sizeof(T) * vec.size());
    return data;
  }

  size_t num_of_knots_ = 0;

  // output
  std::vector<double> x_;
  std::vector<double> dx_;
  std::vector<double> ddx_;

  std::array<double, 3> x_init_ = {{0.0, 0.0, 0.0}};
  std::array<double, 3> scale_factor_ = {{1.0, 1.0, 1.0}};

  std::vector<std::pair<double, double>> x_bounds_;
  std::vector<std::pair<double, double>> dx_bounds_;
  std::vector<std::pair<double, double>> ddx_bounds_;
  std::vector<std::pair<double, double>> dddx_bounds_;

  double weight_x_ = 0.0;
  double weight_dx_ = 0.0;
  double weight_ddx_ = 0.0;
  double weight_dddx_ = 0.0;
  double weight_ddx_offset_ = 0.0;

  double delta_s_ = 1.0;

  bool has_x_ref_ = false;
  double weight_x_ref_ = 0.0;
  std::vector<double> x_ref_;
  // un-uniformed weighting
  std::vector<double> weight_x_ref_vec_;

  // start end state process
  bool has_start_state_ref_ = false;
  bool has_end_state_ref_ = false;
  std::array<double, 3> weight_start_state_ = {{0.0, 0.0, 0.0}};
  std::array<double, 3> start_state_ref_ = {{0.0, 0.0, 0.0}};
  std::array<double, 3> weight_end_state_ = {{0.0, 0.0, 0.0}};
  std::array<double, 3> end_state_ref_ = {{0.0, 0.0, 0.0}};

  // Settings of osqp
  bool verbose_ = false;
  double eps_abs_ = 1e-4;
  double eps_rel_ = 1e-2;
  double eps_prim_inf_ = 1e-5;
  double eps_dual_inf_ = 1e-5;
  bool scaled_termination_ = true;
  bool polish_ = true;
  int polish_refine_iter_ = 3;
  int max_iter_ = 3000;
  double alpha_ = 1.6;
  bool adaptive_rho_ = false;
  int adaptive_rho_interval_ = 15;
  double time_limit_ = 0;
};

}  // namespace planning
}  // namespace TL
