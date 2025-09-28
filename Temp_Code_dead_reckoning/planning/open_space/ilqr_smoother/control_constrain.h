/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  control_constrain.h
 */

#pragma once
#include <algorithm>
#include <memory>
#include "Eigen/Dense"
#include "planning/open_space/ilqr_smoother/al_constrain.h"

class ControlConstrain : public Constrain {
 public:
  ~ControlConstrain() override = default;

  void Init(const double max_acc, const double min_acc, const double max_w,
            const double min_w, const int direction) {
    N_ = 5;
    max_acc_ = max_acc;
    min_acc_ = min_acc;
    max_w_ = max_w;
    min_w_ = min_w;
    direction_ = direction;
    constrain_type_ = INEQUALITY;
    constrain_name_ = "control";
    c_ = Eigen::VectorXd::Zero(N_, 1);
    jac_ = Eigen::MatrixXd::Zero(N_, 6);
    lambda_ = Eigen::VectorXd::Zero(N_, 1);
    mu_ = Eigen::VectorXd::Ones(N_, 1);
    I_mu_ = Eigen::MatrixXd::Identity(N_, N_);
  }

  void Evaluate(const Eigen::Ref<const Eigen::VectorXd>& xu) override {
    c_[0] = xu[4] - max_acc_;   // a <= max_a
    c_[1] = -xu[4] + min_acc_;  // a >= min_a
    c_[2] = xu[5] - max_w_;     // w <= max_w
    c_[3] = -xu[5] + min_w_;    // w >= max_w
    c_[4] = 0.0;
    if (direction_ == 1) {
      c_[4] = -xu[2];  // v >= 0
    } else if (direction_ == -1) {
      c_[4] = xu[2];  // v <= 0
    }
  }

  void Jacobian(const Eigen::Ref<const Eigen::VectorXd>& xu) override {
    UNUSED(xu);
    jac_(0, 4) = 1.0;
    jac_(1, 4) = -1.0;
    jac_(2, 5) = 1.0;
    jac_(3, 5) = -1.0;
    jac_(4, 3) = 0.0;
    if (direction_ == 1) {
      jac_(4, 3) = -1.0;
    } else if (direction_ == -1) {
      jac_(4, 3) = 1.0;
    }
  }

 private:
  double max_acc_ = 0.0;
  double min_acc_ = 0.0;
  double max_w_ = 0.0;
  double min_w_ = 0.0;
  int direction_ = 0;
};
