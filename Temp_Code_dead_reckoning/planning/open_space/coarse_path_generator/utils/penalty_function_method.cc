/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  penalty_function_method.cc
 */

#include "planning/open_space/coarse_path_generator/utils/penalty_function_method.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include "Eigen/src/Core/Matrix.h"
#include "osqp/osqp.h"

namespace TL {
namespace planning {

void TransPolygon2HyperPlan(const common::math::Polygon2d& polygon,
                            Eigen::Ref<Eigen::MatrixXd> A,
                            Eigen::Ref<Eigen::VectorXd> b) {
  if (polygon.num_points() < 3) {
    return;
  }
  auto points = polygon.GetAllVertices();
  int pt_size = static_cast<int>(points.size());
  for (int i = 0; i < pt_size; ++i) {
    int ni = (i + 1) % pt_size;
    if (fabs(points[i].x() - points[ni].x()) < kAccuracy) {
      // vertical
      if (points[i].y() > points[ni].y()) {
        A(i, 0) = -1;
        A(i, 1) = 0;
        b.row(i) << -1 * points[i].x();
      } else {
        A(i, 0) = 1;
        A(i, 1) = 0;
        b.row(i) << points[i].x();
      }
    } else if (fabs(points[i].y() - points[ni].y()) < kAccuracy) {
      // horizontal
      if (points[i].x() > points[ni].x()) {
        A(i, 0) = 0;
        A(i, 1) = 1;
        b.row(i) << points[i].y();
      } else {
        A(i, 0) = 0;
        A(i, 1) = -1;
        b.row(i) << -1 * points[i].y();
      }
    } else {
      double k =
          (points[ni].y() - points[i].y()) / (points[ni].x() - points[i].x());
      double bias = points[i].y() - k * points[i].x();
      if (points[i].x() > points[ni].x()) {
        A(i, 0) = -k;
        A(i, 1) = 1;
        b.row(i) << bias;
      } else {
        A(i, 0) = k;
        A(i, 1) = -1;
        b.row(i) << -1 * bias;
      }
    }
  }
}

double PenaltyFunctionMethod::Value(const Eigen::Vector3d& x) {
  Eigen::Vector3d x_diff = x - x_ref_;
  double fx = x_diff.transpose() * w_ * x_diff;
  if (affine_constraints_ != nullptr) {
    auto affine_constraints = affine_constraints_->Value(x);
    affine_constraints = affine_constraints.unaryExpr(
        [](double a) { return a < 0.0 ? 0.0 : a; });
    fx += mu_ * pow(affine_constraints.sum(), 2);
  }
  return fx;
}

void PenaltyFunctionMethod::Gradient(const Eigen::Vector3d& x,
                                     Eigen::Ref<Eigen::Vector3d> grad) {
  Eigen::Vector3d x_diff = x - x_ref_;
  grad = 2 * w_ * x_diff;
  if (affine_constraints_ != nullptr) {
    auto affine_constraints = affine_constraints_->Value(x);
    auto A(affine_constraints_->Slope());
    for (int i = 0; i < affine_constraints.size(); i++) {
      if (affine_constraints[i] < 0.0) {
        affine_constraints[i] = 0.0;
        A.row(i).setZero();
      }
    }
    grad += 2.0 * mu_ * A.transpose() * affine_constraints;
  }
}

void PenaltyFunctionMethod::AddAffineConstrains(
    Eigen::Ref<const Eigen::MatrixXd> A,    // NOLINT
    Eigen::Ref<const Eigen::VectorXd> b) {  //NOLINT
  affine_constraints_ =
      std::make_unique<AffineFunction<Eigen::MatrixXd, Eigen::VectorXd>>(A, b);
}

void PenaltyFunctionMethod::CalculaterKernal(const bool is_park_out) {
  P_data_ = {1.0, 1.0, 1.0};
  if (is_park_out) {
    P_data_ = {0.0, 1.0, 0.0};
  }
  P_indices_ = {0, 1, 2};
  P_indptr_ = {0, 1, 2, 3};
}

bool PenaltyFunctionMethod::CheckConstrain(const Eigen::Vector3d& x) {
  if (affine_constraints_ != nullptr) {
    static double kEps = kAccuracy;
    auto affine_constraints = affine_constraints_->Value(x);
    return (affine_constraints.array() < kEps).all();
  }
  return true;
}

bool PenaltyFunctionMethod::OptimizeWithOSQP(Eigen::Ref<Eigen::Vector3d> x) {
  // NOLINTBEGIN
  const auto& A = affine_constraints_->Slope();
  std::vector<c_float> A_data(A.rows() * A.cols());
  std::vector<c_int> A_indices(A.rows() * A.cols());
  std::vector<c_int> A_indptr(A.cols() + 1);
  for (int j = 0; j < A.cols(); ++j) {
    for (int i = 0; i < A.rows(); ++i) {
      A_indices[j * A.rows() + i] = i;
      A_data[j * A.rows() + i] = A(i, j);
    }
    A_indptr[j] = j * A.rows();
  }
  A_indptr[A.cols()] = A.rows() * A.cols();

  const auto& B = affine_constraints_->Bias();
  std::vector<c_float> upper_bounds(B.data(), B.data() + B.size());
  std::vector<c_float> lower_bounds(B.size(), -INFINITY);
  std::vector<c_float> q(x.size(), 0.0);

  std::vector<c_float> primal_warm_start(x.data(), x.data() + x.size());
  auto* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  auto* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  OSQPWorkspace* work = nullptr;
  osqp_set_default_settings(settings);
  settings->max_iter = max_iter_;
  settings->warm_start = true;

  data->n = x.size();
  data->m = A.rows();
  data->P = csc_matrix(data->n, data->n, P_data_.size(), P_data_.data(),
                       P_indices_.data(), P_indptr_.data());
  data->q = q.data();
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = lower_bounds.data();
  data->u = upper_bounds.data();
  c_int exitflag = osqp_setup(&work, data, settings);
  if (work == nullptr) {
    AERROR << "planning failed: osqp_work is nullptr! exitflag is " << exitflag;
    c_free(data->A);
    c_free(data->P);
    c_free(data);
    c_free(settings);
    return false;
  }
  osqp_warm_start_x(work, primal_warm_start.data());

  osqp_solve(work);

  auto status = (work)->info->status_val;

  if (status != 1 || work->solution == nullptr) {
    AERROR << "Failed to find solution. optimization status "
           << work->info->status;
    // Cleanup
    osqp_cleanup(work);
    c_free(data->A);
    c_free(data->P);
    c_free(data);
    c_free(settings);
    return false;
  }
  AINFO << "suc in osqp";
  x << work->solution->x[0], work->solution->x[1], work->solution->x[2];
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);
  return true;
  // NOLINTEND
}

bool PenaltyFunctionMethod::Optimize(Eigen::Ref<Eigen::Vector3d> x) {
  if (affine_constraints_ == nullptr) {
    x = x_ref_;
    ADEBUG << "has no constraints, return ref directly";
    return true;
  }

  if (CheckConstrain(x)) {
    ADEBUG << "init point satisify constraints";
    return true;
  }
  if (use_osqp_) {
    // currently we do not optimize theta
    // it is a simple qp programming problem
    // use osqp directly instead
    return OptimizeWithOSQP(x) && CheckConstrain(x);
  }
  Eigen::Vector3d grad;
  Eigen::Vector3d search_direction;
  for (int i = 0; i < max_iter_; ++i) {
    double alpha = alpha_;
    // use gradient descent to get the minimum point
    for (int j = 0; j < 2000; ++j) {
      Gradient(x, grad);
      search_direction = -1 * grad;
      const double val = Value(x);
      // armijo rule
      while (Value(x + alpha * search_direction) >
             val + armijo_c1_ * alpha * grad.transpose() * search_direction) {
        alpha *= 0.5;
        if (alpha < min_step_size_) {
          break;
        }
      }
      if (alpha < min_step_size_) {
        ADEBUG << "step size is too short " << alpha << " j = " << j;
        break;
      }
      x += alpha * search_direction;
    }
    mu_ *= mu_factor_;
  }
  // check grad value to return
  return CheckConstrain(x);
}
}  // namespace planning
}  // namespace TL
