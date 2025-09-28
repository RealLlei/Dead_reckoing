/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  al_constrain.h
 */

#pragma once
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include "Eigen/Dense"

enum ConstrainType {
  UNKNOWN = 0,
  EQUALITY = 1,
  INEQUALITY = 2,
};

class Constrain {
 public:
  Constrain() = default;
  virtual ~Constrain() = default;

  virtual void Evaluate(const Eigen::Ref<const Eigen::VectorXd>& xu) = 0;
  virtual void Jacobian(const Eigen::Ref<const Eigen::VectorXd>& xu) = 0;

  void UpdateDual(const Eigen::Ref<const Eigen::VectorXd>& xu) {
    Evaluate(xu);
    for (int i = 0; i < N_; ++i) {
      if (constrain_type_ == EQUALITY) {
        lambda_(i) += mu_(i) * c_(i);
      } else if (constrain_type_ == INEQUALITY) {
        lambda_(i) = std::max(0.0, lambda_(i) + mu_(i) * c_(i));
      }
      mu_(i) = std::min(1.0e3, mu_(i) * 5.0);
      I_mu_(i, i) = mu_(i);
      if (constrain_type_ == INEQUALITY && c_(i) < 0.0 &&
          std::fabs(lambda_(i)) < 1e-3) {
        I_mu_(i, i) = 0.0;
      }
    }
  }

  ConstrainType GetType() const { return constrain_type_; }

  std::string GetName() const { return constrain_name_; }

  const Eigen::VectorXd& GetEval() { return c_; }

  const Eigen::MatrixXd& GetJac() { return jac_; }

  const Eigen::VectorXd& GetLambda() { return lambda_; }

  const Eigen::VectorXd& GetMu() { return mu_; }

  const Eigen::MatrixXd& GetI() { return I_mu_; }

 protected:
  int N_ = 0;
  Eigen::VectorXd lambda_;  // N * 1
  Eigen::VectorXd mu_;      // N * 1
  Eigen::MatrixXd I_mu_;    // N * N
  Eigen::VectorXd c_;       // N * 1
  Eigen::MatrixXd jac_;     // N * 6
  ConstrainType constrain_type_ = UNKNOWN;
  std::string constrain_name_;
};
