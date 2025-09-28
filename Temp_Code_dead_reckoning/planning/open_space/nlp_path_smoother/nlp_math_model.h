/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  nlp_math_model.h
 */

#pragma once

#include <utility>
#include <vector>
#include "coin/IpTNLP.hpp"
#include "planning/open_space/nlp_path_smoother/nlp_input_param.h"

namespace TL {
namespace planning {

class NlpMathModel : public Ipopt::TNLP {
 public:
  NlpMathModel(NlpMathModel&&) = delete;
  NlpMathModel& operator=(NlpMathModel&&) = delete;
  explicit NlpMathModel(NlpInputParam nlp_input_param);

  ~NlpMathModel() override = default;

  /**
   * @brief Get the nlp info object
   * 
   * @param n 
   * @param m 
   * @param nnz_jac_g 
   * @param nnz_h_lag 
   * @param index_style 
   * @return true 
   * @return false 
   */
  bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                    Ipopt::Index& nnz_h_lag,
                    IndexStyleEnum& index_style) override;

  /**
   * @brief Get the bounds info object
   * 
   * @param n 
   * @param x_l 
   * @param x_u 
   * @param m 
   * @param g_l 
   * @param g_u 
   * @return true 
   * @return false 
   */
  bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                       Ipopt::Index m, Ipopt::Number* g_l,
                       Ipopt::Number* g_u) override;

  /**
   * @brief Get the starting point object
   * 
   * @param n 
   * @param init_x 
   * @param x 
   * @param init_z 
   * @param z_L 
   * @param z_U 
   * @param m 
   * @param init_lambda 
   * @param lambda 
   * @return true 
   * @return false 
   */
  bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                          bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                          Ipopt::Index m, bool init_lambda,
                          Ipopt::Number* lambda) override;

  /**
   * @brief eval_f
   * 
   * @param n 
   * @param x 
   * @param new_x 
   * @param obj_value 
   * @return true 
   * @return false 
   */
  bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
              Ipopt::Number& obj_value) override;

  /**
   * @brief eval_grad_f
   * 
   * @param n 
   * @param x 
   * @param new_x 
   * @param grad_f 
   * @return true 
   * @return false 
   */
  bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                   Ipopt::Number* grad_f) override;

  /**
   * @brief eval_g
   * 
   * @param n 
   * @param x 
   * @param new_x 
   * @param m 
   * @param g 
   * @return true 
   * @return false 
   */
  bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
              Ipopt::Index m, Ipopt::Number* g) override;

  /**
   * @brief eval_jac_g
   * 
   * @param n 
   * @param x 
   * @param new_x 
   * @param m 
   * @param nele_jac 
   * @param iRow 
   * @param jCol 
   * @param values 
   * @return true 
   * @return false 
   */
  bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                  Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                  Ipopt::Index* jCol, Ipopt::Number* values) override;

  /**
   * @brief eval_h
   * 
   * @param n 
   * @param x 
   * @param new_x 
   * @param obj_factor 
   * @param m 
   * @param lambda 
   * @param new_lambda 
   * @param nele_hess 
   * @param iRow 
   * @param jCol 
   * @param values 
   * @return true 
   * @return false 
   */
  bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
              Ipopt::Number obj_factor, Ipopt::Index m,
              const Ipopt::Number* lambda, bool new_lambda,
              Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol,
              Ipopt::Number* values) override;

  /**
   * @brief finalize_solution
   * 
   * @param status 
   * @param n 
   * @param x 
   * @param z_L 
   * @param z_U 
   * @param m 
   * @param g 
   * @param lambda 
   * @param obj_value 
   * @param ip_data 
   * @param ip_cq 
   */
  void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                         const Ipopt::Number* x, const Ipopt::Number* z_L,
                         const Ipopt::Number* z_U, Ipopt::Index m,
                         const Ipopt::Number* g, const Ipopt::Number* lambda,
                         Ipopt::Number obj_value,
                         const Ipopt::IpoptData* ip_data,
                         Ipopt::IpoptCalculatedQuantities* ip_cq) override;

  /**
   * @brief FLUToENU
   * 
   * @param x 
   * @param y 
   * @param ENU_x 
   * @param ENU_y 
   * @param ENU_heading 
   * @return std::pair<double, double> 
   */
  static std::pair<double, double> FLUToENU(double x, double y, double ENU_x,
                                            double ENU_y, double ENU_heading);

  /**
   * @brief optimized_result
   * 
   * @param result 
   */
  void optimized_result(std::vector<double>* result) const;

  /**
   * @brief SparseJacobianMatrix
   * 
   * @param m 
   * @param n 
   * @param x 
   * @param nonzero_row 
   * @param nonzero_col 
   * @param nonzero_values 
   */
  void SparseJacobianMatrix(Ipopt::Index m, Ipopt::Index n,
                            const Ipopt::Number* x,
                            std::vector<Ipopt::Index>* nonzero_row,
                            std::vector<Ipopt::Index>* nonzero_col,
                            std::vector<Ipopt::Number>* nonzero_values);

  /**
   * @brief SparseHessianMatrix
   * 
   * @param m 
   * @param n 
   * @param x 
   * @param obj_factor 
   * @param lambda 
   * @param nonzero_row 
   * @param nonzero_col 
   * @param nonzero_values 
   */
  void SparseHessianMatrix(Ipopt::Index m, Ipopt::Index n,
                           const Ipopt::Number* x, Ipopt::Number obj_factor,
                           const Ipopt::Number* lambda,
                           std::vector<Ipopt::Index>* nonzero_row,
                           std::vector<Ipopt::Index>* nonzero_col,
                           std::vector<Ipopt::Number>* nonzero_values);

 private:
  NlpMathModel(const NlpMathModel&);
  NlpMathModel& operator=(const NlpMathModel&);

  const NlpInputParam nlp_input_param_;
  const int finite_element_num_;
  const int constraint_num_;

  std::vector<double> optimized_result_;
};

}  // namespace planning
}  // namespace TL
