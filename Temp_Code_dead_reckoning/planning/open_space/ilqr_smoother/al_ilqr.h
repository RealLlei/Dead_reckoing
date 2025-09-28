/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  al_ilqr.h
 */

#pragma once
#include <algorithm>
#include <memory>
#include <utility>
#include <vector>
#include "common/math/vec2d.h"
#include "planning/open_space/ilqr_smoother/al_cost.h"
#include "planning/open_space/ilqr_smoother/kinematic_model.h"

using MatrixType = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

struct Regularization {
  double rho = 1e-6;
  const double max_reg = 1e8;
  const double min_reg = 1e-8;
  const double reg_scale = 1.5;
  bool is_const_rho = false;

  bool CheckRegularization() const { return std::fabs(rho - max_reg) > 1e-2; }

  void IncreaseRegularization() {
    rho = std::min(max_reg, std::max(rho * reg_scale, min_reg));
    rho = std::max(rho, reg_scale);
  }

  void DecreaseRegularization() {
    rho = std::min(max_reg, std::max(rho / reg_scale, min_reg));
    if (rho < reg_scale) {
      rho = 0.0;
    }
  }

  void InitRegularization() { rho = 1e-6; }
};

class ALILQR {
 public:
  explicit ALILQR(const Param& param);
  ~ALILQR() = default;

  /**
   * @brief Get the Optimal object
   * 
   * @param direction 
   * @param initial_line 
   * @param ref_line 
   * @param obs 
   * @param pose_relax 
   * @param x_op 
   * @param u_op 
   * @return true 
   * @return false 
   */
  bool GetOptimal(
      int direction, const Eigen::Ref<const Eigen::MatrixXd>& initial_line,
      const Eigen::Ref<const Eigen::MatrixXd>& ref_line,
      const std::vector<std::pair<TL::common::math::Vec2d, double>>& obs,
      const std::pair<double, double>& pose_relax,
      Eigen::Ref<Eigen::MatrixXd> x_op, Eigen::Ref<Eigen::MatrixXd> u_op);

  /**
   * @brief Get Initial Path
   * 
   * @param is_lat_spot 
   * @param x1 
   * @param y1 
   * @param theta1 
   * @param x2 
   * @param y2 
   * @param theta2 
   * @param xu 
   */
  void GetInitial(bool is_lat_spot, double x1, double y1, double theta1,
                  double x2, double y2, double theta2,
                  Eigen::Ref<Eigen::MatrixXd> xu) const;

 private:
  /**
   * @brief Compute Cubic Curve Coefficients
   * 
   * @param x1 
   * @param y1 
   * @param dy1 
   * @param x2 
   * @param y2 
   * @param dy2 
   * @param coef 
   */
  static void ComputeCubicCurveCoefficients(double x1, double y1, double dy1,
                                            double x2, double y2, double dy2,
                                            Eigen::VectorXd* coef);

  /**
    * @brief Set the End Pose Constraint object
    * 
    * @param end_pose 
    */
  void SetEndPoseConstraint(const Eigen::Ref<const Eigen::VectorXd>& end_pose,
                            const std::pair<double, double>& pose_relax);

  /**
   * @brief Set the Control Bound object
   * 
   * @param direction 
   */
  void SetControlBound(int direction);

  /**
   * @brief Set the Obs Constraint object
   * 
   * @param initial_lines 
   * @param obs 
   */
  void SetObsConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& initial_line,
      const std::vector<std::pair<TL::common::math::Vec2d, double>>& obs);

  /**
   * @brief Simulate the trajectory
   * 
   * @param init_state 
   * @param u 
   * @param x 
   */
  void Simulate(const Eigen::Ref<const Eigen::VectorXd>& init_state,
                Eigen::Ref<Eigen::MatrixXd> u, Eigen::Ref<Eigen::MatrixXd> x);

  /**
   * @brief Forward the trajectory
   * 
   * @param init_state 
   * @param K 
   * @param d 
   * @param u 
   * @param x 
   */
  void Forward(const Eigen::Ref<const Eigen::VectorXd>& init_state,
               const std::vector<MatrixType>& K,
               const Eigen::Ref<Eigen::MatrixXd>& d,
               Eigen::Ref<Eigen::MatrixXd> u, Eigen::Ref<Eigen::MatrixXd> x);

  /**
   * @brief Backward the trajectory
   * 
   * @param x 
   * @param u 
   * @param K 
   * @param d 
   * @return true 
   * @return false 
   */
  bool Backward(const Eigen::Ref<const Eigen::MatrixXd>& x,
                const Eigen::Ref<const Eigen::MatrixXd>& u,
                std::vector<MatrixType>* K, Eigen::Ref<Eigen::MatrixXd> d);

  /**
   * @brief Caculate the cost
   * 
   * @param ref_line 
   * @param xu 
   * @return double 
   */
  double CaculateCost(const Eigen::Ref<const Eigen::MatrixXd>& ref_line,
                      const Eigen::Ref<const Eigen::MatrixXd>& xu);

  /**
   * @brief Evaluate the derivative
   * 
   * @param ref_line 
   * @param xu 
   */
  void EvaluateDerivative(const Eigen::Ref<const Eigen::MatrixXd>& ref_line,
                          const Eigen::Ref<const Eigen::MatrixXd>& xu);

  /**
   * @brief Get the Max Constraint Violation object
   * 
   * @param xu 
   * @return double 
   */
  double MaxConstraintViolation(const Eigen::Ref<const Eigen::MatrixXd>& xu);

  /**
   * @brief Update the dual variable
   * 
   * @param xu 
   */
  void UpdateDualVar(const Eigen::Ref<const Eigen::MatrixXd>& xu);

  std::shared_ptr<KinematicModel> kinematic_model_ptr_;
  std::vector<std::shared_ptr<ALCost>> al_costs_vec_;
  Param param_;
  Eigen::MatrixXd weight_;  // 6*6
  int horizon_ = 0;
  Regularization regularization_;
};
