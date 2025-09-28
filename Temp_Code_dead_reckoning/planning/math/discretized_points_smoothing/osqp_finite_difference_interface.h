/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning osqp finite differfence interface smoother
 * Author: ROC
 */

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "osqp/osqp.h"

namespace TL {
namespace planning {

class OsqpFiniteDifferenceInterface {
 public:
  OsqpFiniteDifferenceInterface();
  int Smooth(const std::vector<c_float>& eval_x,
             const std::vector<c_float>& eval_y,
             const std::vector<c_float>& eval_heading,
             const std::vector<c_float>& lon,
             // first left, second right
             const std::vector<std::pair<c_float, c_float>>& lat,
             std::vector<c_float>* solution);

  void SetWeightFirstOrder1st(const double first_order_1st) {
    first_order_1st_ = first_order_1st;
  }

  void SetWeightFirstOrder2nd(const double first_order_2nd) {
    first_order_2nd_ = first_order_2nd;
  }

  void SetWeightFirstOrder4th(const double first_order_4th) {
    first_order_4th_ = first_order_4th;
  }

  void SetWeightSecondOrder2nd(const double second_order_2nd) {
    second_order_2nd_ = second_order_2nd;
  }

  void SetWeightSecondOrder4th(const double second_order_4th) {
    second_order_4th_ = second_order_4th;
  }

  void SetWeightThirdOrder1st(const double third_order_1st) {
    third_order_1st_ = third_order_1st;
  }

  void SetWeightThirdOrder2nd(const double third_order_2nd) {
    third_order_2nd_ = third_order_2nd;
  }

  void SetWeightFourthOrder2nd(const double fourth_order_2nd) {
    fourth_order_2nd_ = fourth_order_2nd;
  }

  void SetWeightLatDeviation(const double lat_deviation) {
    lat_deviation_ = lat_deviation;
  }

  void SetWeightLonDeviation(const double lon_deviation) {
    lon_deviation_ = lon_deviation;
  }

  void SetWeightEulerDeviation(const double euler_deviation) {
    euler_deviation_ = euler_deviation;
  }

  void SetWeightCtWeight(const double ct_weight) { ct_weight_ = ct_weight; }

  void SetWeightCurvature(const double curvature) { curvature_ = curvature; }

  void SetWeightsQpPenMaxIter(const int sqp_pen_max_iter) {
    sqp_pen_max_iter_ = sqp_pen_max_iter;
  }

  void SetWeightsQpSubMaxIter(const int sqp_sub_max_iter) {
    sqp_sub_max_iter_ = sqp_sub_max_iter;
  }

  void SetWeightsSqpFtol(const double sqp_ftol) { sqp_ftol_ = sqp_ftol; }

  // OSQP settings
  void SetOsqpVerbose(const bool verbose) { verbose_ = verbose; }

  void SetOsqpEpsAbs(const double eps_abs) { eps_abs_ = eps_abs; }

  void SetOsqpEpsRel(const double eps_rel) { eps_rel_ = eps_rel; }

  void SetOsqpEpsPrimInf(const double eps_prim_inf) {
    eps_prim_inf_ = eps_prim_inf;
  }

  void SetOsqpEpsDualInf(const double eps_dual_inf) {
    eps_dual_inf_ = eps_dual_inf;
  }

  void SetOsqpScaledTermination(const bool scaled_termination) {
    scaled_termination_ = scaled_termination;
  }

  void SetOsqpEnableAdaptiveRho(const bool adaptive_rho) {
    adaptive_rho_ = adaptive_rho;
  }

  void SetOsqpPolish(const bool polish) { polish_ = polish; }

  void SetOsqpPolishRefineIter(const int polish_refine_iter) {
    polish_refine_iter_ = polish_refine_iter;
  }

  void SetOsqpMaxIter(const int max_iter) { max_iter_ = max_iter; }

  void SetOsqpAlpha(const double alpha) { alpha_ = alpha; }

  void SetOsqpAdaptiveRhoInterval(const int adaptive_rho_interval) {
    adaptive_rho_interval_ = adaptive_rho_interval;
  }

  void SetOsqpTimeLimit(const double time_limit) { time_limit_ = time_limit; }

  /**
   * @brief display matrix and align numbers
   *
   * @param name matrix name
   * @param matrix matrix
   */
  static void MatrixDebug(
      const std::string& name,
      const std::map<c_int, std::map<c_int, c_float>>& matrix);
  static void MatrixDebug(const std::string& name,
                          const std::vector<std::vector<c_float>>& matrix);

 private:
  static void Cscm(const std::map<c_int, std::map<c_int, c_float>>& mat,
                   std::vector<c_float>* data, std::vector<c_int>* indices,
                   std::vector<c_int>* indptr, bool tri = false);

  static void FirstForwardDerivativeKernel1stOrderAccuracy(
      std::vector<std::vector<c_float>>* p);

  static void FirstCentralDerivativeKernel2ndOrderAccuracy(
      std::vector<std::vector<c_float>>* p);

  static void FirstCentralDerivativeKernel4thOrderAccuracy(
      std::vector<std::vector<c_float>>* p);

  static void SecondCentralDerivativeKernel2ndOrderAccuracy(
      std::vector<std::vector<c_float>>* p);

  static void SecondCentralDerivativeKernel4thOrderAccuracy(
      std::vector<std::vector<c_float>>* p);

  static void ThirdForwardDerivativeKernel1stOrderAccuracy(
      std::vector<std::vector<c_float>>* p);

  static void ThirdForwardDerivativeKernel2ndOrderAccuracy(
      std::vector<std::vector<c_float>>* p);

  static void FourthCentralDerivativeKernel2ndOrderAccuracy(
      std::vector<std::vector<c_float>>* p);

  static void LatDeviationKernel(const std::vector<c_float>& eval_x,
                                 const std::vector<c_float>& eval_y,
                                 const std::vector<c_float>& eval_heading,
                                 std::vector<std::vector<c_float>>* p,
                                 std::vector<c_float>* g);

  static void LonDeviationKernel(const std::vector<c_float>& eval_x,
                                 const std::vector<c_float>& eval_y,
                                 const std::vector<c_float>& eval_heading,
                                 std::vector<std::vector<c_float>>* p,
                                 std::vector<c_float>* g);

  static void EulerDeviationKernel(const std::vector<c_float>& eval_x,
                                   const std::vector<c_float>& eval_y,
                                   const std::vector<c_float>& eval_heading,
                                   std::vector<std::vector<c_float>>* p,
                                   std::vector<c_float>* g);

  /**
   * @brief Get the Partitial Derivatives object
   * @param eval_x smooth point eval x
   * @param eval_y smooth point eval y
   * @param ith
   * @param delta_s acculumated_s / (eval_x.size() - 1)
   * @param c smooth point eval curvature
   * @return std::vector<c_float>
   */
  static std::vector<c_float> GetPartitialDerivatives(
      const std::vector<c_float>& eval_x, const std::vector<c_float>& eval_y,
      c_int ith, c_float delta_s, c_float c);

  /**
   * @brief Update Deviation kernel
   * @param p P matrix
   * @param cur_weight weight of P kernel
   * @param n number of point size * 2
   */
  void UpdateDeviationKernel(const std::vector<std::vector<c_float>>& p,
                             c_float cur_weight, c_int n);

  /**
   * @brief Upfate different P kernel in pm_ matrix
   * @param p P matrix
   * @param weight weight of P kernel
   * @param n number of point size * 2
   */
  void UpdateDifferenceKernel(const std::vector<std::vector<c_float>>& p,
                              c_float weight, c_int n);

  /**
   * @brief Update Kernel Map
   * @param k num of rows
   * @param j num of cols
   * @param val current P point value
   */
  void UpdateKernelMap(c_int k, c_int j, c_float val);

  /**
   * @brief assemble P matrix kernel
   * @param num_vars number of point size
   * @param eval_x smooth point eval x
   * @param eval_y smooth point eval y
   * @param eval_heading smooth point eval heading
   */
  void AssembleKernel(c_int num_vars, const std::vector<c_float>& eval_x,
                      const std::vector<c_float>& eval_y,
                      const std::vector<c_float>& eval_heading);

  /**
   * @brief Generate constraint matrix: A matrix, lb, ub
   * @param num_vars number of point size
   * @param eval_heading smooth point eval heading
   * @param eval_x smooth point eval x
   * @param eval_y smooth point eval y
   * @param lon lon bound
   * @param lat lat bound
   */
  void GenerateConstraintMatrix(
      c_int num_vars, const std::vector<c_float>& eval_heading,
      const std::vector<c_float>& eval_x, const std::vector<c_float>& eval_y,
      const std::vector<c_float>& lon,
      const std::vector<std::pair<c_float, c_float>>& lat);

  void SetConstraintWithSlackVariable(c_int num_vars, c_int num_slacks,
                                      const std::vector<c_float>& eval_x,
                                      const std::vector<c_float>& eval_y);

  void PrepareOsqpData(const std::vector<c_float>& eval_x,
                       const std::vector<c_float>& eval_y,
                       const std::vector<c_float>& eval_heading,
                       const std::vector<c_float>& lon,
                       // first left, second right
                       const std::vector<std::pair<c_float, c_float>>& lat,
                       c_int num_vars, c_int num_slacks);

  static bool Optimize(const std::vector<c_float>& primal_warm_start,
                       OSQPWorkspace** work, c_int n,
                       std::vector<c_float>* opt_xy, c_int num_slacks,
                       std::vector<c_float>* slacks);

  static void SetPrimalWarmStart(const std::vector<c_float>& eval_x,
                                 const std::vector<c_float>& eval_y,
                                 const std::vector<c_float>& slacks,
                                 std::vector<c_float>* primal_warm_start);

  static void GetOptXY(const std::vector<c_float>& opt_xy,
                       std::vector<c_float>* opt_x,
                       std::vector<c_float>* opt_y);

  static void GetSolution(const std::vector<c_float>& opt_xy,
                          std::vector<c_float>* solution);

  void GetNumVariables(c_int num_points, c_int* n, c_int* num_slack,
                       c_int* total) const;

  static void FreeOsqp(OSQPData* data, OSQPWorkspace* work,
                       OSQPSettings* settings);

  // Settings of osqp
  bool verbose_ = false;
  double eps_abs_ = 1e-4;
  double eps_rel_ = 1e-4;
  double eps_prim_inf_ = 1e-4;
  double eps_dual_inf_ = 1e-4;
  bool scaled_termination_ = false;
  bool adaptive_rho_ = true;
  bool polish_ = true;
  int polish_refine_iter_ = 100;
  int max_iter_ = 1000;
  double alpha_ = 1.0;
  int adaptive_rho_interval_ = 0;

  // Weights in optimization cost function
  double first_order_1st_ = 0.0;
  double first_order_2nd_ = 0.0;
  double first_order_4th_ = 0.0;
  double second_order_2nd_ = 1e6;
  double second_order_4th_ = 0.0;
  double third_order_1st_ = 1e5;
  double third_order_2nd_ = 0.0;
  double fourth_order_2nd_ = 1e4;
  double lat_deviation_ = 1e0;
  double lon_deviation_ = 0.0;
  double euler_deviation_ = 0.0;
  double ct_weight_ = 0.0;
  double curvature_ = 0.1;
  int sqp_pen_max_iter_ = 1;
  int sqp_sub_max_iter_ = 5;
  double sqp_ftol_ = 1e-4;
  double time_limit_ = 0.05;

  // Optimized_result
  std::map<c_int, std::map<c_int, c_float>> pm_;
  std::map<c_int, std::map<c_int, c_float>> am_;
  std::vector<c_float> g_;
  std::vector<c_float> lb_;
  std::vector<c_float> ub_;
};
}  // namespace planning
}  // namespace TL
