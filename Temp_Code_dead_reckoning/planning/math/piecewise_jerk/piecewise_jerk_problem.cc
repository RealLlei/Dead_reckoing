/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  planning piecewise jerk problem
 */

#include "planning/math/piecewise_jerk/piecewise_jerk_problem.h"
#include "osqp/glob_opts.h"
#include "planning/common/planning_gflags.h"

namespace TL {
namespace planning {

namespace {
constexpr double kMaxVariableRange = 1.0e10;
}  // namespace

PiecewiseJerkProblem::PiecewiseJerkProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init) {
  CHECK_GE(num_of_knots, 2U);
  num_of_knots_ = num_of_knots;

  x_init_ = x_init;

  delta_s_ = delta_s;

  x_bounds_.resize(num_of_knots_,
                   std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  dx_bounds_.resize(num_of_knots_,
                    std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  ddx_bounds_.resize(num_of_knots_,
                     std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  dddx_bounds_.resize(num_of_knots_,
                      std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  weight_x_ref_vec_ = std::vector<double>(num_of_knots_, 0.0);
}

OSQPData* PiecewiseJerkProblem::FormulateProblem() {
  // calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  // calculate offset
  std::vector<c_float> q;
  CalculateOffset(&q);
  // NOLINTBEGIN
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  // NOLINTEND
  CHECK_EQ(lower_bounds.size(), upper_bounds.size());

  auto kernel_dim = 3 * static_cast<c_int>(num_of_knots_);
  auto num_affine_constraint = static_cast<c_int>(lower_bounds.size());

  data->n = kernel_dim;
  data->m = num_affine_constraint;
  data->P =
      csc_matrix(kernel_dim, kernel_dim, static_cast<c_int>(P_data.size()),
                 CopyData(P_data), CopyData(P_indices), CopyData(P_indptr));
  data->q = CopyData(q);
  data->A = csc_matrix(num_affine_constraint, kernel_dim,
                       static_cast<c_int>(A_data.size()), CopyData(A_data),
                       CopyData(A_indices), CopyData(A_indptr));
  data->l = CopyData(lower_bounds);
  data->u = CopyData(upper_bounds);
  return data;
}

bool PiecewiseJerkProblem::Optimize(const std::string& name) {
  OSQPData* data = FormulateProblem();
  OSQPSettings* settings = SolverDefaultSettings();
  OSQPWorkspace* osqp_work = nullptr;
  osqp_setup(&osqp_work, data, settings);

  if (osqp_work == nullptr) {
    AERROR << "planning failed: osqp_work is nullptr!";
    FreeOsqpData(data, settings, osqp_work);
    return false;
  }

  osqp_solve(osqp_work);
  auto status = osqp_work->info->status_val;
  // TL::common::util::OsqpDebug(data, osqp_work, name,
  //                                 FLAGS_enable_path_bound_debug);

  if (status < 0 || (status != 1 && status != 2)) {
    AERROR << "failed optimization status:\t"
           << osqp_work->info->status;  //NOLINT
    FreeOsqpData(data, settings, osqp_work);
    return false;
  }
  if (osqp_work->solution == nullptr) {
    AERROR << "The solution from OSQP is nullptr";
    FreeOsqpData(data, settings, osqp_work);
    return false;
  }

  // extract primal results
  x_.resize(num_of_knots_);
  dx_.resize(num_of_knots_);
  ddx_.resize(num_of_knots_);

  // NOLINTBEGIN
  for (size_t i = 0; i < num_of_knots_; ++i) {
    x_.at(i) = osqp_work->solution->x[i] / scale_factor_[0];
    dx_.at(i) = osqp_work->solution->x[i + num_of_knots_] / scale_factor_[1];
    ddx_.at(i) =
        osqp_work->solution->x[i + 2 * num_of_knots_] / scale_factor_[2];
  }
  // NOLINTEND

  // Cleanup
  FreeOsqpData(data, settings, osqp_work);
  return true;
}

OSQPSettings* PiecewiseJerkProblem::SolverDefaultSettings() {
  // Define Solver default settings
  // NOLINTBEGIN
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  // NOLINTEND
  osqp_set_default_settings(settings);
  settings->verbose = static_cast<c_int>(verbose_);
  settings->eps_abs = eps_abs_;
  settings->eps_rel = eps_rel_;
  settings->eps_prim_inf = eps_prim_inf_;
  settings->eps_dual_inf = eps_dual_inf_;
  settings->scaled_termination = static_cast<c_int>(scaled_termination_);
  settings->adaptive_rho = static_cast<c_int>(adaptive_rho_);
  settings->polish = static_cast<c_int>(polish_);
  settings->polish_refine_iter = polish_refine_iter_;
  settings->max_iter = max_iter_;
  settings->alpha = alpha_;
  settings->adaptive_rho_interval = adaptive_rho_interval_;
  settings->time_limit = time_limit_;
  return settings;
}

void PiecewiseJerkProblem::SetXBounds(
    std::vector<std::pair<double, double>> x_bounds) {
  CHECK_EQ(x_bounds.size(), num_of_knots_);
  x_bounds_ = std::move(x_bounds);
}

void PiecewiseJerkProblem::SetDxBounds(
    std::vector<std::pair<double, double>> dx_bounds) {
  CHECK_EQ(dx_bounds.size(), num_of_knots_);
  dx_bounds_ = std::move(dx_bounds);
}

void PiecewiseJerkProblem::SetDdxBounds(
    std::vector<std::pair<double, double>> ddx_bounds) {
  CHECK_EQ(ddx_bounds.size(), num_of_knots_);
  ddx_bounds_ = std::move(ddx_bounds);
}

void PiecewiseJerkProblem::SetDddxBounds(
    std::vector<std::pair<double, double>> dddx_bounds) {
  CHECK_EQ(dddx_bounds.size(), num_of_knots_);
  dddx_bounds_ = std::move(dddx_bounds);
}

void PiecewiseJerkProblem::SetDddxBounds(const double dddx_bound) {
  SetDddxBounds(-dddx_bound, dddx_bound);
}

void PiecewiseJerkProblem::SetDddxBounds(const double dddx_lower_bound,
                                         const double dddx_upper_bound) {
  for (auto& dddx : dddx_bounds_) {
    dddx.first = dddx_lower_bound;
    dddx.second = dddx_upper_bound;
  }
}

void PiecewiseJerkProblem::SetXBounds(const double x_lower_bound,
                                      const double x_upper_bound) {
  for (auto& x : x_bounds_) {
    x.first = x_lower_bound;
    x.second = x_upper_bound;
  }
}

void PiecewiseJerkProblem::SetDxBounds(const double dx_lower_bound,
                                       const double dx_upper_bound) {
  for (auto& x : dx_bounds_) {
    x.first = dx_lower_bound;
    x.second = dx_upper_bound;
  }
}

void PiecewiseJerkProblem::SetDdxBounds(const double ddx_lower_bound,
                                        const double ddx_upper_bound) {
  for (auto& x : ddx_bounds_) {
    x.first = ddx_lower_bound;
    x.second = ddx_upper_bound;
  }
}

void PiecewiseJerkProblem::SetXRef(const double weight_x_ref,
                                   std::vector<double> x_ref) {
  if (x_ref.size() < num_of_knots_) {
    AERROR << "SetXRef x_ref_size < num_of_knots! x_ref_size: " << x_ref.size()
           << ", num_of_knots: " << num_of_knots_;
    return;
  }
  weight_x_ref_ = weight_x_ref;
  // set uniform weighting
  weight_x_ref_vec_ = std::vector<double>(num_of_knots_, weight_x_ref);
  x_ref_ = std::move(x_ref);
  has_x_ref_ = true;
}

void PiecewiseJerkProblem::SetXRef(std::vector<double> weight_x_ref_vec,
                                   std::vector<double> x_ref) {
  if (x_ref.size() < num_of_knots_ || weight_x_ref_vec.size() < num_of_knots_) {
    AERROR << "x_ref_size or weight_x_ref_vec_size < num_of_knots! x_ref_size: "
           << x_ref.size()
           << "weight_x_ref_vec_size:" << weight_x_ref_vec.size()
           << ", num_of_knots: " << num_of_knots_;
    return;
  }
  // set piecewise weighting
  weight_x_ref_vec_ = std::move(weight_x_ref_vec);
  x_ref_ = std::move(x_ref);
  has_x_ref_ = true;
}

void PiecewiseJerkProblem::SetStartStateRef(
    const std::array<double, 3>& weight_start_state,
    const std::array<double, 3>& start_state_ref) {
  weight_start_state_ = weight_start_state;
  start_state_ref_ = start_state_ref;
  has_start_state_ref_ = true;
}

void PiecewiseJerkProblem::SetEndStateRef(
    const std::array<double, 3>& weight_end_state,
    const std::array<double, 3>& end_state_ref) {
  weight_end_state_ = weight_end_state;
  end_state_ref_ = end_state_ref;
  has_end_state_ref_ = true;
}

void PiecewiseJerkProblem::FreeOsqpData(OSQPData* data, OSQPSettings* settings,
                                        OSQPWorkspace* osqp_work) {
  // NOLINTBEGIN
  if (data != nullptr) {
    if (data->q != nullptr) {
      delete[] data->q;
      data->q = nullptr;
    }

    if (data->l != nullptr) {
      delete[] data->l;
      data->l = nullptr;
    }

    if (data->u != nullptr) {
      delete[] data->u;
      data->u = nullptr;
    }

    if (data->P != nullptr) {
      delete[] data->P->i;
      delete[] data->P->p;
      delete[] data->P->x;
      c_free(data->P);
      data->P = nullptr;
    }

    if (data->A != nullptr) {
      delete[] data->A->i;
      delete[] data->A->p;
      delete[] data->A->x;
      c_free(data->A);
      data->A = nullptr;
    }

    c_free(data);
    data = nullptr;
  }

  if (settings != nullptr) {
    c_free(settings);
    settings = nullptr;
  }
  // NOLINTEND

  if (osqp_work != nullptr) {
    osqp_cleanup(osqp_work);
    osqp_work = nullptr;
  }
}
}  // namespace planning
}  // namespace TL
