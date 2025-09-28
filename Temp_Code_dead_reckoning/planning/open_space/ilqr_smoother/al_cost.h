/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  al_cost.h
 */

#pragma once
#include <algorithm>
#include <memory>
#include <vector>
#include "Eigen/Dense"

#include "planning/open_space/ilqr_smoother/al_constrain.h"
#include "planning/open_space/ilqr_smoother/kinematic_model.h"

class ALCost {
 public:
  explicit ALCost(const Param& param) : param_(param) {
    dx_ = Eigen::VectorXd::Zero(param_.state_dim);
    du_ = Eigen::VectorXd::Zero(param_.control_dim);
    ddx_ = Eigen::MatrixXd::Zero(4, 4);
    ddu_ = Eigen::MatrixXd::Zero(2, 2);
    dudx_ = Eigen::MatrixXd::Zero(2, 4);
    weight_ = Eigen::MatrixXd::Zero(6, 6);
    weight_(0, 0) = param_.w_x;
    weight_(1, 1) = param_.w_y;
    weight_(2, 2) = param_.w_spd;
    weight_(3, 3) = param_.w_theta;
    weight_(4, 4) = param_.w_acc;
    weight_(5, 5) = param_.w_w;
  }

  /**
   * @brief Set the Constraint object
   * 
   * @param constrain 
   */
  void SetConstraint(const std::shared_ptr<Constrain>& constrain);

  /**
   * @brief Evaluate the cost function
   * 
   * @param xu 
   * @param xu_ref 
   */
  void EvaluateDerivative(const Eigen::Ref<const Eigen::VectorXd>& xu,
                          const Eigen::Ref<const Eigen::VectorXd>& xu_ref);

  /**
   * @brief Evaluate the constraints
   * 
   * @param xu 
   * @return double 
   */
  double CalculateConstraintsCost(const Eigen::Ref<const Eigen::VectorXd>& xu);

  /**
   * @brief Update the dual variable
   * 
   * @param xu 
   */
  void UpdateDualVar(const Eigen::Ref<const Eigen::VectorXd>& xu);

  /**
   * @brief Calculate the max violation
   * 
   * @param xu 
   * @return double 
   */
  double MaxVio(const Eigen::Ref<const Eigen::VectorXd>& xu) {
    double max_vio = 0.0;
    for (const auto& constrain : constrains_) {
      constrain->Evaluate(xu);
      if (constrain->GetType() == EQUALITY) {
        max_vio = std::max(max_vio, constrain->GetEval().cwiseAbs().maxCoeff());
      } else if (constrain->GetType() == INEQUALITY) {
        max_vio = std::max(max_vio, constrain->GetEval().maxCoeff());
      }
    }
    return max_vio;
  }

  const Eigen::VectorXd& Getdx() { return dx_; }

  const Eigen::VectorXd& Getdu() { return du_; }

  const Eigen::MatrixXd& Getddx() { return ddx_; }

  const Eigen::MatrixXd& Getddu() { return ddu_; }

  const Eigen::MatrixXd& Getdudx() { return dudx_; }

 private:
  Eigen::VectorXd dx_;    // 1 * 4
  Eigen::VectorXd du_;    // 1*2
  Eigen::MatrixXd ddx_;   // 4*4
  Eigen::MatrixXd ddu_;   // 2*2
  Eigen::MatrixXd dudx_;  // 2*4

  Eigen::MatrixXd weight_;  // 6*6

  std::vector<std::shared_ptr<Constrain>> constrains_;
  Param param_;
};
