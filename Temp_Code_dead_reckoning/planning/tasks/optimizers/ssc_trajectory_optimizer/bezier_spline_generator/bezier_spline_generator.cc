/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file bezier_spline_generator.cc
 **/

#include "planning/tasks/optimizers/ssc_trajectory_optimizer/bezier_spline_generator/bezier_spline_generator.h"

#include <algorithm>
#include <cstddef>
#include <map>
#include <utility>
#include <vector>

#include "common/math/double_type.h"
#include "common/math/math_utils.h"
#include "osqp/cs.h"

#include "osqp/glob_opts.h"
#include "osqp/types.h"
#include "planning/tasks/optimizers/ssc_trajectory_optimizer/bezier_spline_generator/bezier_utils.h"

namespace TL::planning {

template <int N_DEG, int N_DIM>
OSQPSettings* BezierSplineGenerator<N_DEG, N_DIM>::SolverDefaultSettings() {
  // Settings of osqp
  bool verbose_ = false;
  double eps_abs_ = 1e-4;
  double eps_rel_ = 1e-2;
  double eps_prim_inf_ = 1e-5;
  double eps_dual_inf_ = 1e-5;
  bool scaled_termination_ = true;
  bool polish_ = true;
  int polish_refine_iter_ = 3;
  int max_iter_ = 3000;
  double alpha_ = 1.6;
  bool adaptive_rho_ = false;
  int adaptive_rho_interval_ = 15;
  double time_limit_ = 0;

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

template <int N_DEG, int N_DIM>
bool BezierSplineGenerator<N_DEG, N_DIM>::CalculateKernel(
    const std::vector<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
    const std::vector<double>& ref_stamps,
    const std::vector<std::array<double, N_DIM>>& ref_points,
    std::vector<c_float>* p_data, std::vector<c_int>* p_indices,
    std::vector<c_int>* p_row_indices, std::vector<c_float>* q) {
  if (p_data == nullptr || p_indices == nullptr || p_row_indices == nullptr ||
      q == nullptr) {
    return false;
  }

  const int num_segments = static_cast<int>(cubes.size());
  const int num_order = N_DEG + 1;
  const int total_num_vals = N_DIM * num_segments * num_order;

  std::vector<std::map<c_int, c_float>> columns(total_num_vals);
  q->resize(total_num_vals, 0.0);

  // for (int i = 0; i < total_num_vals; i++) {
  //   for (int j = 0; j < total_num_vals; j++) {
  //     if (i == j) {
  //       columns[std::max(i, j)][std::min(i, j)] += 20000.0;
  //     } else if (i > j) {
  //        columns[std::max(i, j)][std::min(i, j)] += 10000.0;
  //     } else {
  //       columns[std::max(i, j)][std::min(i, j)] += 10000.0;
  //     }
  //     // columns[std::max(i, j)][std::min(i, j)] += 20000.0;
  //   }
  // }

  // 三阶导数平方
  int derivative_degree = 3;
  const auto hessian =
      BezierUtils<N_DEG>::GetBezierHessianMat(derivative_degree);
  int row_id = 0;
  int col_id = 0;
  for (int n = 0; n < num_segments; n++) {
    const auto duration = cubes[n].t_ub - cubes[n].t_lb;
    for (int d = 0; d < N_DIM; d++) {
      for (int j = 0; j < num_order; j++) {
        for (int k = 0; k < num_order; k++) {
          row_id = d * num_segments * num_order + n * num_order + j;
          col_id = d * num_segments * num_order + n * num_order + k;
          if (common::math::double_type::SeemsEqual(hessian.at(j).at(k), 0.0) ||
              row_id >= total_num_vals || col_id >= total_num_vals ||
              row_id > col_id) {
            continue;
          }

          columns[col_id][row_id] +=
              hessian.at(j).at(k) / pow(duration, 2 * derivative_degree - 3);
        }
      }
    }
  }

  // * Only position difference is considered
  if (!ref_stamps.empty()) {
    for (std::size_t i = 0; i < ref_stamps.size(); ++i) {
      if (ref_stamps[i] < cubes[0].t_lb ||
          ref_stamps[i] > cubes[num_segments - 1].t_ub) {
        continue;
      }

      int n = 0;
      for (n = 0; n < num_segments; n++) {
        if (cubes[n].t_ub > ref_stamps[i]) {
          break;
        }
      }
      n = std::min(num_segments - 1, n);
      const auto s = cubes[n].t_ub - cubes[n].t_lb;
      const auto t = ref_stamps[i] - cubes[n].t_lb;
      for (int d = 0; d < N_DIM; d++) {
        for (int j = 0; j < num_order; j++) {
          row_id = d * num_segments * num_order + n * num_order + j;
          q->at(row_id) += -2 * ref_points[i][d] * s *
                           common::math::Nnchoosek(N_DEG, j) * pow(t / s, j) *
                           pow(1 - t / s, N_DEG - j);
          for (int k = 0; k < num_order; k++) {
            col_id = d * num_segments * num_order + n * num_order + k;
            if (col_id >= row_id) {
              columns[col_id][row_id] +=
                  s * s * common::math::Nnchoosek(N_DEG, j) *
                  common::math::Nnchoosek(N_DEG, k) * pow(t / s, j + k) *
                  pow(1 - t / s, 2 * N_DEG - j - k);
            }
          }
        }
      }
    }
  }

  int index = 0;
  for (const auto& column : columns) {
    p_indices->push_back(index);
    for (const auto& row_data_pair : column) {
      p_data->push_back(row_data_pair.second * 2.0);
      p_row_indices->push_back(row_data_pair.first);
      ++index;
    }
  }
  p_indices->push_back(index);

  return true;
}

template <int N_DEG, int N_DIM>
bool BezierSplineGenerator<N_DEG, N_DIM>::CalculateConstraint(
    const std::vector<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
    const std::vector<std::array<double, N_DIM>>& start_constraints,
    const std::vector<std::array<double, N_DIM>>& end_constraints,
    std::vector<c_float>* a_data, std::vector<c_int>* a_indices,
    std::vector<c_int>* a_row_indices, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  if (a_data == nullptr || a_indices == nullptr || a_row_indices == nullptr ||
      lower_bounds == nullptr || upper_bounds == nullptr) {
    return false;
  }

  const int num_segments = static_cast<int>(cubes.size());
  const int num_order = N_DEG + 1;
  const int total_num_vals = N_DIM * num_segments * num_order;

  // 1.分段连续约束(0/1/2阶导连续)
  const int num_continuity = 3;
  const int num_connections = num_segments - 1;
  const int num_continuity_constraints =
      num_connections * num_continuity * N_DIM;

  // 2.第一段起点状态约束(0/1/2阶导连续)
  const int num_start_eq_constraints =
      static_cast<int>(start_constraints.size()) * N_DIM;

  // 3.最后一段终点状态约束(0/1/2阶导连续)
  const int num_end_eq_constraints =
      static_cast<int>(end_constraints.size()) * N_DIM;

  // 4.stack inequality constraints
  int num_not_equal_constraints = 0;
  for (int i = 0; i < num_segments; i++) {
    num_not_equal_constraints +=
        (static_cast<int>(cubes[i].p_ub.size())) * num_order;
    num_not_equal_constraints +=
        (static_cast<int>(cubes[i].v_ub.size())) * (num_order - 1);
    num_not_equal_constraints +=
        (static_cast<int>(cubes[i].a_ub.size())) * (num_order - 2);
  }

  int total_num_constraints =
      num_continuity_constraints + num_start_eq_constraints +
      num_end_eq_constraints + num_not_equal_constraints;
  //  int total_num_constraints = num_start_eq_constraints;

  std::vector<std::map<c_int, double>> columns(total_num_vals);
  lower_bounds->resize(total_num_constraints, 0.0);
  upper_bounds->resize(total_num_constraints, 0.0);

  int row_id = 0;
  int col_id = 0;
  constexpr auto kEpison = 1e-20;

  // 1.分段连续约束(0/1/2阶导连续)
  for (int d = 0; d < N_DIM; d++) {
    for (int n = 0; n < num_connections; n++) {
      auto prev_duration = cubes[n].t_ub - cubes[n].t_lb;
      auto next_duration = cubes[n + 1].t_ub - cubes[n + 1].t_lb;
      for (int c = 0; c < num_continuity; c++) {
        auto prev_scale = pow(prev_duration, 1 - c);
        auto next_scale = pow(next_duration, 1 - c);
        if (c == 0) {
          // ~ position end
          col_id = d * num_segments * num_order + n * num_order + N_DEG;
          columns[col_id][row_id] = 1.0 * prev_scale;
          // ~ position begin
          col_id = d * num_segments * num_order + (n + 1) * num_order + 0;
          columns[col_id][row_id] = -1.0 * next_scale;
        } else if (c == 1) {
          // ~ velocity end
          col_id = d * num_segments * num_order + n * num_order + N_DEG - 1;
          columns[col_id][row_id] = -1.0 * prev_scale;
          col_id = d * num_segments * num_order + n * num_order + N_DEG;
          columns[col_id][row_id] = 1.0 * prev_scale;
          // ~ velocity begin
          col_id = d * num_segments * num_order + (n + 1) * num_order;
          columns[col_id][row_id] = 1.0 * next_scale;
          col_id = d * num_segments * num_order + (n + 1) * num_order + 1;
          columns[col_id][row_id] = -1.0 * next_scale;
        } else if (c == 2) {
          // ~ acceleration end
          col_id = d * num_segments * num_order + n * num_order + N_DEG - 2;
          columns[col_id][row_id] = 1.0 * prev_scale;
          col_id = d * num_segments * num_order + n * num_order + N_DEG - 1;
          columns[col_id][row_id] = -2.0 * prev_scale;
          col_id = d * num_segments * num_order + n * num_order + N_DEG;
          columns[col_id][row_id] = 1.0 * prev_scale;
          // ~ acceleration begin
          col_id = d * num_segments * num_order + (n + 1) * num_order;
          columns[col_id][row_id] = -1.0 * next_scale;
          col_id = d * num_segments * num_order + (n + 1) * num_order + 1;
          columns[col_id][row_id] = 2.0 * next_scale;
          col_id = d * num_segments * num_order + (n + 1) * num_order + 2;
          columns[col_id][row_id] = -1.0 * next_scale;
        } else if (c == 3) {
          // ~ jerk end
          col_id = d * num_segments * num_order + n * num_order + N_DEG - 3;
          columns[col_id][row_id] = -1.0 * prev_scale;
          col_id = d * num_segments * num_order + n * num_order + N_DEG - 2;
          columns[col_id][row_id] = 3.0 * prev_scale;
          col_id = d * num_segments * num_order + n * num_order + N_DEG - 1;
          columns[col_id][row_id] = -3.0 * prev_scale;
          col_id = d * num_segments * num_order + n * num_order + N_DEG;
          columns[col_id][row_id] = 1.0 * prev_scale;
          // ~ jerk begin
          col_id = d * num_segments * num_order + (n + 1) * num_order;
          columns[col_id][row_id] = 1.0 * next_scale;
          col_id = d * num_segments * num_order + (n + 1) * num_order + 1;
          columns[col_id][row_id] = -3.0 * next_scale;
          col_id = d * num_segments * num_order + (n + 1) * num_order + 2;
          columns[col_id][row_id] = 3.0 * next_scale;
          col_id = d * num_segments * num_order + (n + 1) * num_order + 3;
          columns[col_id][row_id] = -1.0 * next_scale;
        }
        lower_bounds->at(row_id) = -kEpison;
        upper_bounds->at(row_id) = kEpison;
        ++row_id;
      }
    }
  }

  // 2.第一段起点状态约束(0/1/2阶导连续)
  int num_order_constraint_start = static_cast<int>(start_constraints.size());
  auto duration = cubes[0].t_ub - cubes[0].t_lb;
  auto scale = 0.0;
  int n = 0;
  for (int j = 0; j < num_order_constraint_start; j++) {
    scale = pow(duration, 1 - j);
    for (int d = 0; d < N_DIM; d++) {
      if (j == 0) {
        col_id = d * num_segments * num_order + n * num_order + 0;
        columns[col_id][row_id] = 1.0 * scale;
      } else if (j == 1) {
        col_id = d * num_segments * num_order + n * num_order + 0;
        columns[col_id][row_id] = -1.0 * N_DEG * scale;
        col_id = d * num_segments * num_order + n * num_order + 1;
        columns[col_id][row_id] = 1.0 * N_DEG * scale;
      } else if (j == 2) {
        col_id = d * num_segments * num_order + n * num_order + 0;
        columns[col_id][row_id] = 1.0 * N_DEG * (N_DEG - 1) * scale;
        col_id = d * num_segments * num_order + n * num_order + 1;
        columns[col_id][row_id] = -2.0 * N_DEG * (N_DEG - 1) * scale;
        col_id = d * num_segments * num_order + n * num_order + 2;
        columns[col_id][row_id] = 1.0 * N_DEG * (N_DEG - 1) * scale;
      }
      lower_bounds->at(row_id) = start_constraints[j][d] - kEpison;
      upper_bounds->at(row_id) = start_constraints[j][d] + kEpison;
      ++row_id;
    }
  }

  // 3.最后一段终点状态约束(0/1/2阶导连续)
  int num_order_constraint_end = static_cast<int>(end_constraints.size());
  duration = cubes[num_segments - 1].t_ub - cubes[num_segments - 1].t_lb;
  scale = 0.0;
  n = num_segments - 1;
  for (int j = 0; j < num_order_constraint_end; j++) {
    scale = pow(duration, 1 - j);
    for (int d = 0; d < N_DIM; d++) {
      // if (j == 0 && d == 0) continue;
      if (j == 0) {
        col_id = d * num_segments * num_order + n * num_order + N_DEG;
        columns[col_id][row_id] = 1.0 * scale;
      } else if (j == 1) {
        col_id = d * num_segments * num_order + n * num_order + N_DEG - 1;
        columns[col_id][row_id] = -1.0 * N_DEG * scale;
        col_id = d * num_segments * num_order + n * num_order + N_DEG;
        columns[col_id][row_id] = 1.0 * N_DEG * scale;
      } else if (j == 2) {
        col_id = d * num_segments * num_order + n * num_order + N_DEG - 2;
        columns[col_id][row_id] = 1.0 * N_DEG * (N_DEG - 1) * scale;
        col_id = d * num_segments * num_order + n * num_order + N_DEG - 1;
        columns[col_id][row_id] = -2.0 * N_DEG * (N_DEG - 1) * scale;
        col_id = d * num_segments * num_order + n * num_order + N_DEG;
        columns[col_id][row_id] = 1.0 * N_DEG * (N_DEG - 1) * scale;
      }
      lower_bounds->at(row_id) = end_constraints[j][d] - kEpison;
      upper_bounds->at(row_id) = end_constraints[j][d] + kEpison;
      ++row_id;
    }
  }

  // 4.stack inequality constraints
  for (int n = 0; n < num_segments; n++) {
    auto duration = cubes[n].t_ub - cubes[n].t_lb;
    auto scale = 0.0;
    for (int d = 0; d < N_DIM; d++) {
      // 0阶导数约束（位置）
      scale = pow(duration, 1 - 0);
      for (int j = 0; j < num_order; j++) {
        col_id = d * num_segments * num_order + n * num_order + j;
        columns[col_id][row_id] += scale;
        lower_bounds->at(row_id) = cubes[n].p_lb.at(d) - kEpison;
        upper_bounds->at(row_id) = cubes[n].p_ub.at(d) + kEpison;
        ++row_id;
      }

      // 1阶导数约束（速度）
      scale = pow(duration, 1 - 1);
      for (int j = 0; j < num_order - 1; j++) {
        col_id = d * num_segments * num_order + n * num_order + j;
        columns[col_id][row_id] += -N_DEG * scale;
        col_id = d * num_segments * num_order + n * num_order + (j + 1);
        columns[col_id][row_id] = N_DEG * scale;
        lower_bounds->at(row_id) = cubes[n].v_lb.at(d) - kEpison;
        upper_bounds->at(row_id) = cubes[n].v_ub.at(d) + kEpison;
        ++row_id;
      }

      // 2阶导数约束（加速度）
      scale = pow(duration, 1 - 2);
      for (int j = 0; j < num_order - 2; j++) {
        col_id = d * num_segments * num_order + n * num_order + j;
        columns[col_id][row_id] = N_DEG * (N_DEG - 1) * scale;
        col_id = d * num_segments * num_order + n * num_order + (j + 1);
        columns[col_id][row_id] = -2.0 * N_DEG * (N_DEG - 1) * scale;
        col_id = d * num_segments * num_order + n * num_order + (j + 2);
        columns[col_id][row_id] = N_DEG * (N_DEG - 1) * scale;
        lower_bounds->at(row_id) = cubes[n].a_lb.at(d) - kEpison;
        upper_bounds->at(row_id) = cubes[n].a_ub.at(d) + kEpison;
        ++row_id;
      }

      // 2阶导数约束（加加速度）
    }
  }

  int index = 0;
  for (const auto& column : columns) {
    a_indices->emplace_back(index);
    for (const auto& row_pair : column) {
      // coefficient
      a_data->emplace_back(row_pair.second);
      // constraint index
      a_row_indices->emplace_back(row_pair.first);
      ++index;
    }
  }
  a_indices->push_back(index);

  return true;
}

template <int N_DEG, int N_DIM>
bool BezierSplineGenerator<N_DEG, N_DIM>::GetBezierSplineUsingCorridor(
    const std::vector<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
    const std::vector<std::array<double, N_DIM>>& start_constraints,
    const std::vector<std::array<double, N_DIM>>& end_constraints,
    const std::vector<double>& ref_stamps,
    const std::vector<std::array<double, N_DIM>>& ref_points,
    BezierSplineType* bezier_spline) {
  if (bezier_spline == nullptr) {
    return false;
  }

  const int num_segments = static_cast<int>(cubes.size());
  const int num_order = N_DEG + 1;
  const int total_num_vals = N_DIM * num_segments * num_order;

  std::vector<c_float> p_data;
  std::vector<c_int> p_indices;
  std::vector<c_int> p_row_indices;
  std::vector<c_float> q;
  if (!CalculateKernel(cubes, ref_stamps, ref_points, &p_data, &p_indices,
                       &p_row_indices, &q)) {
    AERROR << "CalculateKernel failed";
    return false;
  }

  // calculate affine constraints
  std::vector<c_float> a_data;
  std::vector<c_int> a_indices;
  std::vector<c_int> a_row_indices;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  if (!CalculateConstraint(cubes, start_constraints, end_constraints, &a_data,
                           &a_indices, &a_row_indices, &lower_bounds,
                           &upper_bounds)) {
    AERROR << "CalculateConstraint failed";
    return false;
  }

  auto* data =                                                  // NOLINT
      reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));  // NOLINT

  auto num_affine_constraint = static_cast<c_int>(lower_bounds.size());

  data->n = total_num_vals;
  data->m = num_affine_constraint;
  data->P = csc_matrix(total_num_vals, total_num_vals,
                       static_cast<c_int>(p_data.size()), CopyData(p_data),
                       CopyData(p_row_indices), CopyData(p_indices));
  data->q = CopyData(q);
  data->A = csc_matrix(num_affine_constraint, total_num_vals,
                       static_cast<c_int>(a_data.size()), CopyData(a_data),
                       CopyData(a_row_indices), CopyData(a_indices));
  data->l = CopyData(lower_bounds);
  data->u = CopyData(upper_bounds);

  OSQPSettings* settings = SolverDefaultSettings();
  OSQPWorkspace* osqp_work = nullptr;
  auto flag = osqp_setup(&osqp_work, data, settings);
  if (flag != 0) {
    FreeOsqpData(data, settings, osqp_work);
    AERROR << "osqp_setup failed";
    return false;
  }

  if (osqp_work == nullptr) {
    FreeOsqpData(data, settings, osqp_work);
    AERROR << "planning failed: osqp_work is nullptr!";
    return false;
  }

  osqp_solve(osqp_work);
  auto status = osqp_work->info->status_val;
  if (status != 0 && status != 1) {
    FreeOsqpData(data, settings, osqp_work);
    AERROR << "osqp_solve failed, status:" << status;
    return false;
  }

  if (osqp_work->solution == nullptr) {
    AERROR << "The solution from OSQP is nullptr";
    FreeOsqpData(data, settings, osqp_work);
    return false;
  }

  std::vector<double> accumulate_time(cubes.size() + 1, 0.0);
  for (size_t i = 1; i < cubes.size(); ++i) {
    accumulate_time.at(i + 1) = cubes.at(i).t_ub;
  }
  bezier_spline->SetAccumulateTime(std::move(accumulate_time));

  std::vector<typename BezierSpline<N_DEG, N_DIM>::ControlPointSegment>
      control_point_segments(num_segments);
  int index = 0;
  for (int i = 0; i < N_DIM; ++i) {
    for (int j = 0; j < num_segments; ++j) {
      for (size_t m = 0; m <= N_DEG; ++m) {
        control_point_segments.at(j).at(m).at(i) =
            osqp_work->solution->x[index++];  // NOLINT
      }
    }
  }
  FreeOsqpData(data, settings, osqp_work);

  bezier_spline->SetControlPointSegments(std::move(control_point_segments));

  return true;
}

void FreeOsqpData(OSQPData* data, OSQPSettings* settings,
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

template class BezierSplineGenerator<5, 2>;
template class BezierSplineGenerator<5, 1>;

}  // namespace TL::planning
