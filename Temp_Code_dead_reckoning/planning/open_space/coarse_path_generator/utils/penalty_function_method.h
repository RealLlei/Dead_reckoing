/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  penalty_function_method.h
 */

#pragma once
#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "common/math/polygon2d.h"
#include "osqp/osqp.h"

namespace TL {
namespace planning {

static constexpr double kAccuracy = 1e-5;

template <typename A, typename B>
class AffineFunction {
 public:
  AffineFunction(const A& a, const B& b) : a_(a), b_(b) {}

  virtual ~AffineFunction() = default;

  B Value(const B& x) { return a_ * x - b_; }

  B Gradient(const B& x);

  const A& Slope() const { return a_; }

  const B& Bias() const { return b_; }

 private:
  A a_;
  B b_;
};

void TransPolygon2HyperPlan(const common::math::Polygon2d& polygon,
                            Eigen::Ref<Eigen::MatrixXd> A,
                            Eigen::Ref<Eigen::VectorXd> b);

class PenaltyFunctionMethod {
 public:
  explicit PenaltyFunctionMethod(const Eigen::Ref<const Eigen::MatrixXd>& x_ref)
      : x_ref_(x_ref) {
    w_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  }

  ~PenaltyFunctionMethod() = default;

  double Value(const Eigen::Vector3d& x);

  void Gradient(const Eigen::Vector3d& x, Eigen::Ref<Eigen::Vector3d> grad);

  void AddAffineConstrains(Eigen::Ref<const Eigen::MatrixXd> A,
                           Eigen::Ref<const Eigen::VectorXd> b);

  void CalculaterKernal(bool is_park_out);

  bool CheckConstrain(const Eigen::Vector3d& x);

  bool Optimize(Eigen::Ref<Eigen::Vector3d> x);

  bool OptimizeWithOSQP(Eigen::Ref<Eigen::Vector3d> x);

 private:
  Eigen::Vector3d x_ref_;
  Eigen::Matrix3d w_;
  uint32_t max_iter_ = 100;
  double mu_ = 1.0;
  double mu_factor_ = 5.0;
  double alpha_ = 0.1;
  double armijo_c1_ = 1e-4;
  double min_step_size_ = 1e-8;
  std::unique_ptr<AffineFunction<Eigen::MatrixXd, Eigen::VectorXd>>
      affine_constraints_;
  bool use_osqp_ = true;
  std::vector<c_float> P_data_;
  std::vector<c_int> P_indices_;
  std::vector<c_int> P_indptr_;
};

}  // namespace planning
}  // namespace TL
