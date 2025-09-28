/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  planning piecewise jerk path problem
 */

#include "planning/math/piecewise_jerk/piecewise_jerk_path_problem.h"
#include <cstddef>
#include <vector>

#include "osqp/glob_opts.h"

namespace TL {
namespace planning {

PiecewiseJerkPathProblem::PiecewiseJerkPathProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init)
    : PiecewiseJerkProblem(num_of_knots, delta_s, x_init) {}

PiecewiseJerkPathProblem::PiecewiseJerkPathProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init, const bool forward_plan,
    const bool enable_curvature, const bool enable_curvature_derivative,
    const bool enable_collision_constraint)
    : PiecewiseJerkProblem(num_of_knots, delta_s, x_init),
      enable_collision_constraint_(enable_collision_constraint),
      forward_plan_(forward_plan),
      enable_curvature_(enable_curvature),
      enable_curvature_derivative_(enable_curvature_derivative) {}

void PiecewiseJerkPathProblem::CalculateKernel(
    std::vector<c_float>* const P_data, std::vector<c_int>* const P_indices,
    std::vector<c_int>* const P_indptr) {
  if (P_data == nullptr || P_indices == nullptr || P_indptr == nullptr) {
    AERROR << "Calculate kernel input has nullptr pointer.";
    return;
  }

  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;
  // const int num_of_constraints =
  //     num_of_variables + 3 * (n - 1) + 3 +
  //     n * static_cast<int>(enable_collision_constraint_);
  std::vector<std::vector<std::pair<c_int, c_float>>> p;
  p.resize(num_of_variables);
  // p.resize(num_of_variables + num_of_constraints);

  // 3 rows in first n cols
  for (int col = 0; col < n; ++col) {
    for (int row = std::max(0, col - 2); row < col + 1; ++row) {
      p[col].emplace_back(row, 0.0);
    }
  }
  // 1 row in n~2n cols
  for (int col = n; col < 2 * n; ++col) {
    p[col].emplace_back(col, 0.0);
  }
  // 2 rows in 2n~3n cols
  for (int col = 2 * n; col < 3 * n; ++col) {
    for (int row = std::max(2 * n, col - 1); row < col + 1; ++row) {
      p[col].emplace_back(row, 0.0);
    }
  }
  // 1 row in 3n~3n+num_of_constraints cols
  // for (int col = 3 * n; col < 3 * n + num_of_constraints; ++col) {
  //   p[col].emplace_back(col, 0.0);
  // }

  PMatrixLKernel(&p);
  PMatrixDlKernel(&p);
  PMatrixDdlKernel(&p);
  PMatrixDddlKernel(&p);
  // PMatrixSoftLConstraintKernel(&p);

  // std::vector<c_float> nn;
  // nn.reserve(p.size());
  // for (int j = 0; j < p.size(); ++j){
  //   nn.push_back(0.0);
  // }
  // std::vector<std::vector<c_float>> m;
  //   m.reserve(p.size());
  // for (int j = 0; j < p.size(); ++j){
  //   m.push_back(nn);
  // }
  // for (int j = 0; j < p.size(); ++j){
  //   for(int i=0; i < p[j].size(); ++i){
  //     m[p[j][i].first][j]=p[j][i].second;
  //   }
  // }
  // MatrixDebug("p matrix: ", m);
  // AERROR << "Matrix End =======================";

  const auto max_variables_size = num_of_variables * 5;
  P_indptr->reserve(max_variables_size);
  P_data->reserve(max_variables_size);
  P_indices->reserve(max_variables_size);

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    P_indptr->emplace_back(ind_p);
    for (const auto& row_data_pair : p[i]) {
      P_data->emplace_back(row_data_pair.second * 2.0);
      P_indices->emplace_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->emplace_back(ind_p);

  P_indptr->shrink_to_fit();
  P_data->shrink_to_fit();
  P_indices->shrink_to_fit();
}

void PiecewiseJerkPathProblem::PMatrixLKernel(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const p) const {
  if (p == nullptr) {
    AERROR << "P pointer is nullptr.";
    return;
  }

  // x(i)^2 * (w_x + w_x_ref[i]), w_x_ref might be a uniform value for all x(i)
  // or piecewise values for different x(i)
  const size_t n = num_of_knots_;
  auto& columns = *p;
  const double scaled_weight_l =
      weight_x_ / (scale_factor_[0] * scale_factor_[0]);

  for (size_t i = 0; i < n; ++i) {
    if (weight_x_ref_vec_.empty() || isnan(weight_x_ref_vec_[i])) {
      columns[i].back().second += scaled_weight_l;
    } else {
      columns[i].back().second +=
          scaled_weight_l +
          weight_x_ref_vec_[i] / (scale_factor_[0] * scale_factor_[0]);
    }
  }
}

void PiecewiseJerkPathProblem::PMatrixSoftLConstraintKernel(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const p) const {
  if (p == nullptr) {
    AERROR << "P pointer is nullptr.";
    return;
  }

  const size_t n = num_of_knots_;
  const size_t num_of_variables = 3 * n;
  const size_t num_of_constraints = num_of_variables + 3 * (n - 1) + 3;
  auto& columns = *p;
  const double scaled_weight_soft_constraint =
      weight_soft_l_constraint_ / (scale_factor_[0] * scale_factor_[0]);

  for (size_t i = 3 * n; i < 3 * n + num_of_constraints; ++i) {
    columns[i].back().second += scaled_weight_soft_constraint;
  }
}

void PiecewiseJerkPathProblem::PMatrixDlKernel(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const p) const {
  if (p == nullptr) {
    AERROR << "P pointer is nullptr.";
    return;
  }

  // x(i)'^2 * w_dx
  // add_dl_kernel
  const size_t n = num_of_knots_;
  auto& columns = *p;
  const double scaled_weight_dl =
      (weight_dx_) / (scale_factor_[1] * scale_factor_[1]);

  for (size_t i = n; i < 2 * n; ++i) {
    columns[i].back().second += scaled_weight_dl;
  }
}

void PiecewiseJerkPathProblem::PMatrixDdlKernel(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const p) const {
  if (p == nullptr) {
    AERROR << "P pointer is nullptr.";
    return;
  }

  // add ddl kernel
  const size_t n = num_of_knots_;
  auto& columns = *p;
  const double scaled_weight_ddl =
      weight_ddx_ / (scale_factor_[2] * scale_factor_[2]);

  for (size_t i = 2 * n; i < 3 * n; ++i) {
    columns[i].back().second += scaled_weight_ddl;
  }
}

void PiecewiseJerkPathProblem::PMatrixDddlKernel(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const p) const {
  if (p == nullptr) {
    AERROR << "P pointer is nullptr.";
    return;
  }

  // add dddl kernel
  const size_t n = num_of_knots_;
  auto& columns = *p;
  const auto delta_s_square = delta_s_ * delta_s_;
  const double scaled_weight_dddl =
      weight_dddx_ / (delta_s_square * scale_factor_[2] * scale_factor_[2]);

  for (size_t i = 2 * n + 1; i < 3 * n; ++i) {
    // col i
    int idx = static_cast<int>(columns[i].size()) - 1;
    columns[i][idx].second += scaled_weight_dddl;
    columns[i][idx - 1].second += -scaled_weight_dddl;
    // col i-1
    columns[i - 1].back().second += scaled_weight_dddl;
  }
}

void PiecewiseJerkPathProblem::CalculateAffineConstraint(
    std::vector<c_float>* const A_data, std::vector<c_int>* const A_indices,
    std::vector<c_int>* const A_indptr,
    std::vector<c_float>* const lower_bounds,
    std::vector<c_float>* const upper_bounds) {
  if (A_data == nullptr || A_indices == nullptr || A_indptr == nullptr ||
      lower_bounds == nullptr || upper_bounds == nullptr) {
    AERROR << "Calculate affine constraint input pointer is nullptr.";
    return;
  }

  // 3N params bounds on x, x', x''
  // 3(N-1) constraints on x, x', x''
  // 3 constraints on x_init_
  const size_t n = num_of_knots_;
  const size_t num_of_variables = 3 * n;
  const size_t num_of_constraints = num_of_variables + 3 * (n - 1) + 3 +
                                    (enable_collision_constraint_ ? n : 0) +
                                    (enable_curvature_ ? n : 0) +
                                    (enable_curvature_derivative_ ? n : 0);
  const auto max_variables_size = num_of_variables * 10;
  A_indptr->reserve(max_variables_size);
  A_data->reserve(max_variables_size);
  A_indices->reserve(max_variables_size);
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);

  std::vector<std::vector<std::pair<c_int, c_float>>> variables(
      num_of_variables);
  int constraint_index = 0;

  // basic constraint
  if (!AMatrixBasicConstraint(&variables, lower_bounds, upper_bounds,
                              &constraint_index)) {
    AERROR << "A Matrix basic constraint process failed.";
    return;
  }

  // start end state constraint
  AMatrixStartEndStateConstraint(&variables, lower_bounds, upper_bounds,
                                 &constraint_index);

  // collision constraint
  if (enable_collision_constraint_) {
    const auto& vehicle_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    AMatrixCollisionConstraint(n, vehicle_param.wheel_base(), &variables,
                               lower_bounds, upper_bounds, &constraint_index);
  }

  // curvature constraint
  AMatrixCurvatureConstraint(&variables, lower_bounds, upper_bounds,
                             &constraint_index);

  // curvature derivative constraint
  AMatrixCurvatureDerivativeConstraint(&variables, lower_bounds, upper_bounds,
                                       &constraint_index);

  if (constraint_index != num_of_constraints) {
    AERROR << "A Matrix failed! constraint_index != num_of_constraints.";
    return;
  }

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    A_indptr->emplace_back(ind_p);
    for (const auto& variable_nz : variables[i]) {
      // coefficient
      A_data->emplace_back(variable_nz.second);

      // constraint index
      A_indices->emplace_back(variable_nz.first);
      ++ind_p;
    }
  }
  // We indeed need this line because of
  // https://github.com/oxfordcontrol/osqp/blob/master/src/cs.c#L255
  A_indptr->emplace_back(ind_p);

  A_indptr->shrink_to_fit();
  A_data->shrink_to_fit();
  A_indices->shrink_to_fit();
}

bool PiecewiseJerkPathProblem::AMatrixBasicConstraint(
    std::vector<std::vector<std::pair<c_int, c_float>>>* variables,
    std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds,
    int* constraint_index) {
  if (variables == nullptr || lower_bounds == nullptr ||
      upper_bounds == nullptr || constraint_index == nullptr) {
    AERROR << "A matrix basic constraint process poniter is "
              "nullptr.";
    return false;
  }

  // set x, x', x'' bounds
  const size_t n = num_of_knots_;
  const size_t num_of_variables = 3 * n;

  for (int i = 0; i < num_of_variables; ++i) {
    if (i < n) {
      variables->at(i).emplace_back(*constraint_index, 1.0);
      lower_bounds->at(*constraint_index) =
          x_bounds_[i].first * scale_factor_[0];
      upper_bounds->at(*constraint_index) =
          x_bounds_[i].second * scale_factor_[0];
    } else if (i < 2 * n) {
      variables->at(i).emplace_back(*constraint_index, 1.0);

      lower_bounds->at(*constraint_index) =
          dx_bounds_[i - n].first * scale_factor_[1];
      upper_bounds->at(*constraint_index) =
          dx_bounds_[i - n].second * scale_factor_[1];
    } else {
      variables->at(i).emplace_back(*constraint_index, 1.0);
      lower_bounds->at(*constraint_index) =
          ddx_bounds_[i - 2 * n].first * scale_factor_[2];
      upper_bounds->at(*constraint_index) =
          ddx_bounds_[i - 2 * n].second * scale_factor_[2];
    }
    ++*constraint_index;
  }

  if (*constraint_index != num_of_variables) {
    AERROR << "constraint_index != num_of_variables, failed!";
    return false;
  }

  // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
  for (int i = 0; i + 1 < n; ++i) {
    variables->at(2 * n + i).emplace_back(*constraint_index, -1.0);
    variables->at(2 * n + i + 1).emplace_back(*constraint_index, 1.0);
    lower_bounds->at(*constraint_index) =
        dddx_bounds_.at(i).first * delta_s_ * scale_factor_[2];
    upper_bounds->at(*constraint_index) =
        dddx_bounds_.at(i).second * delta_s_ * scale_factor_[2];
    ++*constraint_index;
  }

  // x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
  for (int i = 0; i + 1 < n; ++i) {
    variables->at(n + i).emplace_back(*constraint_index,
                                      -1.0 * scale_factor_[2]);
    variables->at(n + i + 1).emplace_back(*constraint_index,
                                          1.0 * scale_factor_[2]);
    variables->at(2 * n + i).emplace_back(*constraint_index,
                                          -0.5 * delta_s_ * scale_factor_[1]);
    variables->at(2 * n + i + 1)
        .emplace_back(*constraint_index, -0.5 * delta_s_ * scale_factor_[1]);
    lower_bounds->at(*constraint_index) = 0.0;
    upper_bounds->at(*constraint_index) = 0.0;
    ++*constraint_index;
  }

  // x(i+1) - x(i) - delta_s * x(i)'
  // - 1/3 * delta_s^2 * x(i)'' - 1/6 * delta_s^2 * x(i+1)''
  auto delta_s_sq_ = delta_s_ * delta_s_;
  for (int i = 0; i + 1 < n; ++i) {
    variables->at(i).emplace_back(*constraint_index,
                                  -1.0 * scale_factor_[1] * scale_factor_[2]);
    variables->at(i + 1).emplace_back(
        *constraint_index, 1.0 * scale_factor_[1] * scale_factor_[2]);
    variables->at(n + i).emplace_back(
        *constraint_index, -delta_s_ * scale_factor_[0] * scale_factor_[2]);
    variables->at(2 * n + i).emplace_back(
        *constraint_index,
        -delta_s_sq_ / 3.0 * scale_factor_[0] * scale_factor_[1]);
    variables->at(2 * n + i + 1)
        .emplace_back(*constraint_index,
                      -delta_s_sq_ / 6.0 * scale_factor_[0] * scale_factor_[1]);

    lower_bounds->at(*constraint_index) = 0.0;
    upper_bounds->at(*constraint_index) = 0.0;
    ++*constraint_index;
  }
  return true;
}

bool PiecewiseJerkPathProblem::AMatrixStartEndStateConstraint(
    std::vector<std::vector<std::pair<c_int, c_float>>>* variables,
    std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds,
    int* constraint_index) {
  if (variables == nullptr || lower_bounds == nullptr ||
      upper_bounds == nullptr || constraint_index == nullptr) {
    AERROR << "A matrix start end state process poniter is "
              "nullptr.";
    return false;
  }

  const size_t n = num_of_knots_;

  // constrain on x_init
  if (forward_plan_) {
    // constrain on start state
    variables->at(0).emplace_back(*constraint_index, 1.0);
    lower_bounds->at(*constraint_index) = x_init_[0] * scale_factor_[0];
    upper_bounds->at(*constraint_index) = x_init_[0] * scale_factor_[0];
    ++*constraint_index;

    variables->at(n).emplace_back(*constraint_index, 1.0);
    lower_bounds->at(*constraint_index) = x_init_[1] * scale_factor_[1];
    upper_bounds->at(*constraint_index) = x_init_[1] * scale_factor_[1];
    ++*constraint_index;

    variables->at(2 * n).emplace_back(*constraint_index, 1.0);
    lower_bounds->at(*constraint_index) = x_init_[2] * scale_factor_[2];
    upper_bounds->at(*constraint_index) = x_init_[2] * scale_factor_[2];
    ++*constraint_index;
  } else {
    // constrain on end state
    variables->at(n - 1).emplace_back(*constraint_index, 1.0);
    lower_bounds->at(*constraint_index) = x_init_[0] * scale_factor_[0];
    upper_bounds->at(*constraint_index) = x_init_[0] * scale_factor_[0];
    ++*constraint_index;

    variables->at(2 * n - 1).emplace_back(*constraint_index, 1.0);
    lower_bounds->at(*constraint_index) = x_init_[1] * scale_factor_[1];
    upper_bounds->at(*constraint_index) = x_init_[1] * scale_factor_[1];
    ++*constraint_index;

    variables->at(3 * n - 1).emplace_back(*constraint_index, 1.0);
    lower_bounds->at(*constraint_index) = x_init_[2] * scale_factor_[2];
    upper_bounds->at(*constraint_index) = x_init_[2] * scale_factor_[2];
    ++*constraint_index;
  }
  return true;
}

bool PiecewiseJerkPathProblem::AMatrixCollisionConstraint(
    const size_t n, const double point_dx_flu,
    std::vector<std::vector<std::pair<c_int, c_float>>>* const variables,
    std::vector<c_float>* const lower_bounds,
    std::vector<c_float>* const upper_bounds, int* const constraint_index) {
  if (variables == nullptr || lower_bounds == nullptr ||
      upper_bounds == nullptr || constraint_index == nullptr) {
    AERROR << "A Matrix collision constraint input pointer is nullptr.";
    return false;
  }

  const size_t delta_i = floor(point_dx_flu / delta_s_ + 0.5);
  for (size_t i = 0; i < n; ++i) {
    // Add detailed constraints to describe corners of the vehicle
    // Calculate the upper and lower bounds at the exact corner point
    double b = 0.0;
    double u = 0.0;
    if (i + delta_i < 0) {
      b = x_bounds_[0].first;
      u = x_bounds_[0].second;
    } else if (i + delta_i > static_cast<size_t>(n - 1)) {
      b = x_bounds_[n - 1].first;
      u = x_bounds_[n - 1].second;
    } else {
      b = x_bounds_[i + delta_i].first;
      u = x_bounds_[i + delta_i].second;
    }
    // insert the constraint into the A matrix
    variables->at(i).emplace_back(*constraint_index, 1.0 / scale_factor_[0]);
    variables->at(i + n).emplace_back(*constraint_index,
                                      point_dx_flu / scale_factor_[1]);
    lower_bounds->at(*constraint_index) = b;
    upper_bounds->at(*constraint_index) = u;
    ++(*constraint_index);
  }
  return true;
}

bool PiecewiseJerkPathProblem::AMatrixCurvatureConstraint(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const variables,
    std::vector<c_float>* const lower_bounds,
    std::vector<c_float>* const upper_bounds, int* const constraint_index) {
  if (variables == nullptr || lower_bounds == nullptr ||
      upper_bounds == nullptr || constraint_index == nullptr) {
    AERROR << "A matrix curvature constraint process poniter is "
              "nullptr.";
    return false;
  }

  // ke_linear = kr^2*l + 0*l' + l" + kr
  if (enable_curvature_) {
    for (size_t i = 0; i < num_of_knots_; ++i) {
      variables->at(i).emplace_back(
          *constraint_index, kappa_ref_[i] * kappa_ref_[i] / scale_factor_[0]);
      variables->at(2 * num_of_knots_ + i)
          .emplace_back(*constraint_index, 1.0 / scale_factor_[2]);
      lower_bounds->at(*constraint_index) = -max_kappa_ - kappa_ref_[i];
      upper_bounds->at(*constraint_index) = max_kappa_ - kappa_ref_[i];
      ++*constraint_index;
    }
  }
  return true;
}

bool PiecewiseJerkPathProblem::AMatrixCurvatureDerivativeConstraint(
    std::vector<std::vector<std::pair<c_int, c_float>>>* variables,
    std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds,
    int* constraint_index) {
  if (variables == nullptr || lower_bounds == nullptr ||
      upper_bounds == nullptr || constraint_index == nullptr) {
    AERROR << "A matrix curvature derivative constraint process poniter is "
              "nullptr.";
    return false;
  }

  // k'e_linear = 2kr*kr'*l + kr'
  if (enable_curvature_derivative_) {
    for (size_t i = 0; i < num_of_knots_; ++i) {
      variables->at(i).emplace_back(
          *constraint_index,
          2 * kappa_ref_[i] * dkappa_ref_[i] / scale_factor_[0]);
      lower_bounds->at(*constraint_index) = -max_dkappa_ - dkappa_ref_[i];
      upper_bounds->at(*constraint_index) = max_dkappa_ - dkappa_ref_[i];
      ++*constraint_index;
    }
  }
  return true;
}

void PiecewiseJerkPathProblem::CalculateOffset(std::vector<c_float>* const q) {
  if (q == nullptr) {
    AERROR << "q matrix is nullptr!";
    return;
  }

  q->resize(3 * num_of_knots_, 0.0);

  QMatrixLOffset(q);

  if (enable_q_matrix_start_end_state_offset_) {
    QMatrixStartEndStateOffset(q);
  }

  if (enable_q_matrix_ddl_offset_) {
    QMatrixDdlOffset(q);
  }
}

bool PiecewiseJerkPathProblem::QMatrixLOffset(std::vector<c_float>* const q) {
  if (q == nullptr) {
    AERROR << "q matrix is nullptr!";
    return false;
  }

  if (has_x_ref_) {
    for (size_t i = 0; i < num_of_knots_; ++i) {
      q->at(i) += -2.0 * weight_x_ref_vec_.at(i) * x_ref_[i] / scale_factor_[0];
    }
  }
  return true;
}

bool PiecewiseJerkPathProblem::QMatrixDdlOffset(std::vector<c_float>* const q) {
  if (q == nullptr) {
    AERROR << "q matrix is nullptr!";
    return false;
  }

  if (!kappa_ref_.empty() && !dkappa_ref_.empty()) {
    for (size_t i = 0; i < num_of_knots_; ++i) {
      q->at(i + 2 * num_of_knots_) += -2 * weight_ddx_offset_;
    }
  }
  return true;
}

bool PiecewiseJerkPathProblem::QMatrixStartEndStateOffset(
    std::vector<c_float>* q) {
  if (q == nullptr) {
    AERROR << "q matrix is nullptr!";
    return false;
  }

  const size_t n = num_of_knots_;

  if (forward_plan_ && has_end_state_ref_) {
    q->at(n - 1) +=
        -2.0 * weight_end_state_[0] * end_state_ref_[0] / scale_factor_[0];
    q->at(2 * n - 1) +=
        -2.0 * weight_end_state_[1] * end_state_ref_[1] / scale_factor_[1];
    q->at(3 * n - 1) +=
        -2.0 * weight_end_state_[2] * end_state_ref_[2] / scale_factor_[2];
  } else if (!forward_plan_ && has_start_state_ref_) {
    q->at(0) +=
        -2.0 * weight_start_state_[0] * start_state_ref_[0] / scale_factor_[0];
    q->at(n) +=
        -2.0 * weight_start_state_[1] * start_state_ref_[1] / scale_factor_[1];
    q->at(2 * n) +=
        -2.0 * weight_start_state_[2] * start_state_ref_[2] / scale_factor_[2];
  }
  return true;
}

void PiecewiseJerkPathProblem::MatrixDebug(
    const std::string& name, const std::vector<std::vector<c_float>>& matrix) {
  AERROR << name;
  for (const auto& row : matrix) {
    std::string str;
    for (double val : row) {
      std::string str1;
      str.append(" ");
      str1 = std::to_string(val);
      str.append(str1.substr(0, 8));
    }
    AERROR << str;
  }
}

}  // namespace planning
}  // namespace TL
