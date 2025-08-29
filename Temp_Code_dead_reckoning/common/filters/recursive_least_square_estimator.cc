/******************************************************************************
 * Copyright 2022 XuXin@TL. All Rights Reserved.
 * *****************************************************************************/
#include "common/filters/recursive_least_square_estimator.h"

#include <algorithm>

RecursiveLeastSquare::RecursiveLeastSquare(const uint na, const uint nb,
                                           const uint d, const double ts,
                                           const double lambda)
    : na_(na), nb_(nb), d_(d), ts_(ts), lambda_(lambda) {
  Init();
}

void RecursiveLeastSquare::Init(const uint na, const uint nb, const uint d,
                                const double ts, const double lambda) {
  na_ = na;
  nb_ = nb;
  d_ = d;
  ts_ = ts;
  lambda_ = lambda;

  Init();
}

void RecursiveLeastSquare::Init() {
  u_.clear();
  y_.clear();

  y_sampled_size_ = 0;
  u_sampled_size_ = 0;
  sample_size_ = std::max(na_ + 1, nb_ + d_ + 1);

  nu_ = na_ + nb_ + 1;

  mt_p_.resize(nu_, nu_);
  mt_p_ = Eigen::MatrixXd::Identity(nu_, nu_);
  // set intial p(0) to a large value to represent low confidence of the
  // first few inputs
  mt_p_ *= 1e6;

  mt_pha_ = Eigen::VectorXd::Zero(nu_, 1);
  mt_k_ = mt_p_ * mt_pha_ / (lambda_ + mt_pha_.transpose() * mt_p_ * mt_pha_);

  mt_theta_ = Eigen::VectorXd::Zero(nu_, 1);

  u_.resize(nb_ + d_ + 1, 0);
  y_.resize(na_ + 1, 0);
}

bool RecursiveLeastSquare::RLSEstimate() {
  if (y_sampled_size_ < sample_size_ || u_sampled_size_ < sample_size_) {
    return false;
  }

  mt_k_ = mt_p_ * mt_pha_ / (lambda_ + mt_pha_.transpose() * mt_p_ * mt_pha_);
  const double eps = y_[0] - mt_pha_.transpose() * mt_theta_;
  mt_theta_ = mt_theta_ + mt_k_ * eps;
  mt_p_ = (Eigen::MatrixXd::Identity(nu_, nu_) - mt_k_ * mt_pha_.transpose()) *
          mt_p_ / lambda_;
  mt_p_ = mt_p_ * 0.5F + mt_p_.transpose().eval() * 0.5F;

  return true;
}

void RecursiveLeastSquare::UpdateY(const double y) {
  for (uint i = y_.size() - 1; i > 0; --i) {
    y_[i] = y_[i - 1];
  }
  y_[0] = y;

  for (uint i = 0; i < na_; ++i) {
    mt_pha_[i] = -y_[i + 1];
  }

  ++y_sampled_size_;
}

void RecursiveLeastSquare::UpdateMu(const double u) {
  for (uint i = u_.size() - 1; i > 0; --i) {
    u_[i] = u_[i - 1];
  }
  u_[0] = u;

  uint j = 0;
  for (uint i = na_; i < mt_pha_.size(); ++i) {
    mt_pha_[i] = u_[j + d_];
    ++j;
  }

  ++u_sampled_size_;
}

const Eigen::VectorXd& RecursiveLeastSquare::ThetaHat() const {
  return mt_theta_;
}
