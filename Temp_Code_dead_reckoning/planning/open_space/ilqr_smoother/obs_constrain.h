/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  obs_constrain.h
 */

#pragma once
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
#include "Eigen/Dense"
#include "common/math/vec2d.h"
#include "planning/open_space/ilqr_smoother/al_constrain.h"

class ObsConstrain : public Constrain {
 public:
  ~ObsConstrain() override = default;

  void Init(const TL::common::math::Vec2d& obs, const double buffer) {
    N_ = 3;
    obs_ = obs;
    buffer_ = buffer;
    constrain_type_ = INEQUALITY;
    constrain_name_ = "obs";
    c_ = Eigen::VectorXd::Zero(N_, 1);
    jac_ = Eigen::MatrixXd::Zero(N_, 6);
    lambda_ = Eigen::VectorXd::Zero(N_, 1);
    mu_ = Eigen::VectorXd::Ones(N_, 1);
    I_mu_ = Eigen::MatrixXd::Identity(N_, N_);
  }

  void Evaluate(const Eigen::Ref<const Eigen::VectorXd>& xu) override {
    const double R = radius_ + buffer_;
    for (int i = 0; i < N_; ++i) {
      const double dx = xu[0] + d_[i] * std::cos(xu[3]) - obs_.x();
      const double dy = xu[1] + d_[i] * std::sin(xu[3]) - obs_.y();
      // (x - x_obs) ^ 2 + (y - y_obs) ^ 2 >= R ^ 2
      c_[i] = R * R - dx * dx - dy * dy;
    }
  }

  void Jacobian(const Eigen::Ref<const Eigen::VectorXd>& xu) override {
    for (int i = 0; i < N_; ++i) {
      const double dx = xu[0] + d_[i] * std::cos(xu[3]) - obs_.x();
      const double dy = xu[1] + d_[i] * std::sin(xu[3]) - obs_.y();
      jac_(i, 0) = -2.0 * dx;
      jac_(i, 1) = -2.0 * dy;
      jac_(i, 3) =
          2.0 * (dx * d_[i] * std::sin(xu[3]) - dy * d_[i] * std::cos(xu[3]));
    }
  }

  bool IsTooFarToConsider(const Eigen::VectorXd& xu,
                          const TL::common::math::Vec2d& obs,
                          const double buffer) {
    const double R = radius_ + buffer;
    constexpr double kConsiderDisThreshold = 1.0;
    return !std::any_of(d_.begin(), d_.end(), [&](const double d) {
      const double dx = xu[0] + d * std::cos(xu[3]) - obs.x();
      const double dy = xu[1] + d * std::sin(xu[3]) - obs.y();
      return std::hypot(dx, dy) - R < kConsiderDisThreshold;
    });
  }

 private:
  TL::common::math::Vec2d obs_;
  double buffer_ = 0.0;
  double radius_ = 1.0;
  // circle center x in flu coordinate
  std::vector<double> d_ = {0.0, 1.48, 2.96};
};
