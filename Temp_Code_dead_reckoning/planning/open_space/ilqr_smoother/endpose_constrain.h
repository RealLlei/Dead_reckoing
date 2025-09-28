/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  endpose_constrain.h
 */

#pragma once
#include <algorithm>
#include <memory>
#include <utility>
#include "Eigen/Dense"
#include "planning/open_space/ilqr_smoother/al_constrain.h"

class EndPoseConstrain : public Constrain {
 public:
  ~EndPoseConstrain() override = default;

  void Init(const Eigen::VectorXd& end_pose) {
    N_ = 3;
    end_pose_ = end_pose;
    constrain_type_ = EQUALITY;
    constrain_name_ = "end_pose";
    c_ = Eigen::VectorXd::Zero(N_, 1);
    jac_ = Eigen::MatrixXd::Zero(N_, 6);
    lambda_ = Eigen::VectorXd::Zero(N_, 1);
    mu_ = Eigen::VectorXd::Ones(N_, 1);
    I_mu_ = Eigen::MatrixXd::Identity(N_, N_);
  }

  void Evaluate(const Eigen::Ref<const Eigen::VectorXd>& xu) override {
    c_[0] = xu[0] - end_pose_[0];  // x - x_e
    c_[1] = xu[1] - end_pose_[1];  // y - y_e
    c_[2] = xu[3] - end_pose_[3];  // theta - theta_e
  }

  void Jacobian(const Eigen::Ref<const Eigen::VectorXd>& xu) override {
    UNUSED(xu);
    jac_(0, 0) = 1.0;
    jac_(1, 1) = 1.0;
    jac_(2, 3) = 1.0;
  }

 private:
  Eigen::VectorXd end_pose_;
};

class EndPoseXConstrain : public Constrain {
 public:
  ~EndPoseXConstrain() override = default;

  void Init(const std::pair<double, double>& pose_relax) {
    N_ = 2;
    x_end_relax_ = pose_relax;
    constrain_type_ = INEQUALITY;
    constrain_name_ = "end_pose_x";
    c_ = Eigen::VectorXd::Zero(N_, 1);
    jac_ = Eigen::MatrixXd::Zero(N_, 6);
    lambda_ = Eigen::VectorXd::Zero(N_, 1);
    mu_ = Eigen::VectorXd::Ones(N_, 1);
    I_mu_ = Eigen::MatrixXd::Identity(N_, N_);
  }

  void Evaluate(const Eigen::Ref<const Eigen::VectorXd>& xu) override {
    c_[0] = x_end_relax_.first - xu[0];   // x >= x_e_min
    c_[1] = xu[0] - x_end_relax_.second;  // x <= x_e_max
  }

  void Jacobian(const Eigen::Ref<const Eigen::VectorXd>& xu) override {
    UNUSED(xu);
    jac_(0, 0) = -1.0;
    jac_(1, 0) = 1.0;
  }

 private:
  std::pair<double, double> x_end_relax_;
};

class EndPoseYConstrain : public Constrain {
 public:
  ~EndPoseYConstrain() override = default;

  void Init(const Eigen::VectorXd& end_pose) {
    N_ = 3;
    y_end_ = end_pose[1];
    constrain_type_ = EQUALITY;
    constrain_name_ = "end_pose_y";
    c_ = Eigen::VectorXd::Zero(N_, 1);
    jac_ = Eigen::MatrixXd::Zero(N_, 6);
    lambda_ = Eigen::VectorXd::Zero(N_, 1);
    mu_ = Eigen::VectorXd::Ones(N_, 1);
    I_mu_ = Eigen::MatrixXd::Identity(N_, N_);
  }

  void Evaluate(const Eigen::Ref<const Eigen::VectorXd>& xu) override {
    c_[0] = xu[1] - y_end_;  // y - y_e
  }

  void Jacobian(const Eigen::Ref<const Eigen::VectorXd>& xu) override {
    UNUSED(xu);
    jac_(0, 1) = 1.0;
  }

 private:
  double y_end_ = 0.0;
};

class EndPoseThetaConstrain : public Constrain {
 public:
  ~EndPoseThetaConstrain() override = default;

  void Init(const Eigen::VectorXd& end_pose) {
    N_ = 1;
    theta_end_ = end_pose[3];
    constrain_type_ = EQUALITY;
    constrain_name_ = "end_pose_theta";
    c_ = Eigen::VectorXd::Zero(N_, 1);
    jac_ = Eigen::MatrixXd::Zero(N_, 6);
    lambda_ = Eigen::VectorXd::Zero(N_, 1);
    mu_ = Eigen::VectorXd::Ones(N_, 1);
    I_mu_ = Eigen::MatrixXd::Identity(N_, N_);
  }

  void Evaluate(const Eigen::Ref<const Eigen::VectorXd>& xu) override {
    c_[0] = xu[3] - theta_end_;  // theta - theta_e
  }

  void Jacobian(const Eigen::Ref<const Eigen::VectorXd>& xu) override {
    UNUSED(xu);
    jac_(0, 3) = 1.0;
  }

 private:
  double theta_end_ = 0.0;
};
