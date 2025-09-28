/*
 * Copyright (c) TL Technologies Co., Ltd. 2023. All rights reserved.
 * Description:  ipopt_pos_optimize_math_model.cpp
 */

#include "planning/math/discretized_points_smoothing/ipopt_pos_optimize_math_model.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "coin/ThirdParty/macros.h"
#include "common/file/log.h"
#include "common/util/span.hpp"
#include "google/protobuf/stubs/port.h"

namespace TL::planning {

using Ipopt::Index;
using Ipopt::Number;

namespace {
constexpr double kMaxValue = 1e20;
}  // namespace

IpoptPosOptimizeMathModel::IpoptPosOptimizeMathModel(
    IpoptPosOptimizeMathModelParam nlp_input_param)
    : nlp_input_param_(std::move(nlp_input_param)),
      finite_element_num_(4 * nlp_input_param_.path_point_size),
      constraint_num_(3 * nlp_input_param_.path_point_size +
                      (nlp_input_param_.is_collision_free
                           ? 0
                           : 2 * nlp_input_param_.path_point_size)) {}

bool IpoptPosOptimizeMathModel::get_nlp_info(Index& n, Index& m,
                                             Index& nnz_jac_g, Index& nnz_h_lag,
                                             IndexStyleEnum& index_style) {
  n = finite_element_num_;
  m = constraint_num_;

  std::vector<Index> nonzero_row;
  std::vector<Index> nonzero_col;
  SparseJacobianMatrix(m, n, nullptr, &nonzero_row, &nonzero_col, nullptr);
  nnz_jac_g = static_cast<int32_t>(nonzero_row.size());

  SparseHessianMatrix(m, n, nullptr, 0.0, nullptr, &nonzero_row, &nonzero_col,
                      nullptr);
  nnz_h_lag = static_cast<int32_t>(nonzero_row.size());
  ADEBUG << "nnz_jac_g: " << nnz_jac_g << ", n: " << n << ", m: " << m
         << ", nnz_h_lag: " << nnz_h_lag;

  index_style = TNLP::C_STYLE;

  return true;
}

bool IpoptPosOptimizeMathModel::get_bounds_info(Index n, Number* x_l,
                                                Number* x_u, Index m,
                                                Number* g_l, Number* g_u) {
  assert(n == finite_element_num_);
  assert(m == constraint_num_);
  boost::span<Number> sx_l(x_l, n);
  boost::span<Number> sx_u(x_u, n);
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    sx_l[i] = nlp_input_param_.constraint_line[i].min_x;
    sx_u[i] = nlp_input_param_.constraint_line[i].max_x;

    sx_l[i + nlp_input_param_.path_point_size] =
        nlp_input_param_.constraint_line[i].min_y;
    sx_u[i + nlp_input_param_.path_point_size] =
        nlp_input_param_.constraint_line[i].max_y;

    sx_l[i + 2 * nlp_input_param_.path_point_size] = -kMaxValue;
    sx_u[i + 2 * nlp_input_param_.path_point_size] = kMaxValue;

    sx_l[i + 3 * nlp_input_param_.path_point_size] =
        -nlp_input_param_.max_kappa;
    sx_u[i + 3 * nlp_input_param_.path_point_size] = nlp_input_param_.max_kappa;
  }

  sx_l[0] = std::get<0>(nlp_input_param_.refine_path_point.front());
  sx_u[0] = std::get<0>(nlp_input_param_.refine_path_point.front());
  sx_l[nlp_input_param_.path_point_size] =
      std::get<1>(nlp_input_param_.refine_path_point.front());
  sx_u[nlp_input_param_.path_point_size] =
      std::get<1>(nlp_input_param_.refine_path_point.front());
  sx_l[2 * static_cast<size_t>(nlp_input_param_.path_point_size)] =
      std::get<2>(nlp_input_param_.refine_path_point.front());
  sx_u[2 * static_cast<size_t>(nlp_input_param_.path_point_size)] =
      std::get<2>(nlp_input_param_.refine_path_point.front());
  if (nlp_input_param_.enable_fix_start_kappa) {
    sx_l[3 * static_cast<size_t>(nlp_input_param_.path_point_size)] =
        std::get<3>(nlp_input_param_.refine_path_point.front());
    sx_u[3 * static_cast<size_t>(nlp_input_param_.path_point_size)] =
        std::get<3>(nlp_input_param_.refine_path_point.front());
  }
  if (nlp_input_param_.enable_fix_end_state) {
    sx_l[nlp_input_param_.path_point_size - 1] =
        std::get<0>(nlp_input_param_.refine_path_point.back());
    sx_u[nlp_input_param_.path_point_size - 1] =
        std::get<0>(nlp_input_param_.refine_path_point.back());
    sx_l[2 * nlp_input_param_.path_point_size - 1] =
        std::get<1>(nlp_input_param_.refine_path_point.back());
    sx_u[2 * nlp_input_param_.path_point_size - 1] =
        std::get<1>(nlp_input_param_.refine_path_point.back());
    sx_l[3 * nlp_input_param_.path_point_size - 1] =
        std::get<2>(nlp_input_param_.refine_path_point.back());
    sx_u[3 * nlp_input_param_.path_point_size - 1] =
        std::get<2>(nlp_input_param_.refine_path_point.back());
  }
  boost::span<Number> sg_l(g_l, m);
  boost::span<Number> sg_u(g_u, m);
  for (int32_t i = 0; i < 3 * nlp_input_param_.path_point_size; ++i) {
    sg_l[i] = 0.0;
    sg_u[i] = 0.0;
  }
  if (!nlp_input_param_.is_collision_free) {
    for (int32_t i = 3 * nlp_input_param_.path_point_size; i < m; ++i) {
      sg_l[i] = nlp_input_param_.min_distance_threshold;
      sg_u[i] = kMaxValue;
    }
  }

  return true;
}

bool IpoptPosOptimizeMathModel::get_starting_point(Index n, bool init_x,
                                                   Number* x, bool init_z,
                                                   Number* z_L, Number* z_U,
                                                   Index m, bool init_lambda,
                                                   Number* lambda) {
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);
  (void)init_x;
  (void)init_z;
  (void)z_L;
  (void)z_U;
  (void)m;
  (void)init_lambda;
  (void)lambda;
  boost::span<Number> sx(x, n);
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    sx[i] = std::get<0>(nlp_input_param_.refine_path_point[i]);
    sx[nlp_input_param_.path_point_size + i] =
        std::get<1>(nlp_input_param_.refine_path_point[i]);
    sx[2 * nlp_input_param_.path_point_size + i] =
        std::get<2>(nlp_input_param_.refine_path_point[i]);
    sx[3 * nlp_input_param_.path_point_size + i] =
        std::get<3>(nlp_input_param_.refine_path_point[i]);
  }

  return true;
}

bool IpoptPosOptimizeMathModel::eval_f(Index n, const Number* x, bool new_x,
                                       Number& obj_value) {
  assert(n == finite_element_num_);
  (void)new_x;
  boost::span<const Number> sx(x, n);
  double bias_square = 0.0;
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    if (i == 0) {
      continue;
    }
    const double dx =
        sx[i] - std::get<0>(nlp_input_param_.refine_path_point[i]);
    const double dy = sx[nlp_input_param_.path_point_size + i] -
                      std::get<1>(nlp_input_param_.refine_path_point[i]);
    bias_square += dx * dx + dy * dy;
  }

  double kappa_square = 0.0;
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    kappa_square += sx[3 * nlp_input_param_.path_point_size + i] *
                    sx[3 * nlp_input_param_.path_point_size + i];
  }

  double dkappa_square = 0.0;
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    if (i == 0) {
      continue;
    }
    const double dkappa = sx[3 * nlp_input_param_.path_point_size + i] -
                          sx[3 * nlp_input_param_.path_point_size + i - 1];
    dkappa_square += dkappa * dkappa;
  }
  obj_value = nlp_input_param_.bias_weight * bias_square +
              nlp_input_param_.kappa_weight * kappa_square +
              nlp_input_param_.dkappa_weight * dkappa_square;

  return true;
}

bool IpoptPosOptimizeMathModel::eval_grad_f(Index n, const Number* x,
                                            bool new_x, Number* grad_f) {
  assert(n == finite_element_num_);
  (void)new_x;
  boost::span<const Number> sx(x, n);
  boost::span<Number> sgrad_f(grad_f, n);
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    sgrad_f[i] = 0.0;
    sgrad_f[i + nlp_input_param_.path_point_size] = 0.0;
    sgrad_f[i + 2 * nlp_input_param_.path_point_size] = 0.0;
    sgrad_f[i + 3 * nlp_input_param_.path_point_size] = 0.0;
  }

  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    if (i == 0) {
      continue;
    }
    sgrad_f[i] += 2.0 *
                  (sx[i] - std::get<0>(nlp_input_param_.refine_path_point[i])) *
                  nlp_input_param_.bias_weight;
    sgrad_f[i + nlp_input_param_.path_point_size] +=
        2.0 *
        (sx[i + nlp_input_param_.path_point_size] -
         std::get<1>(nlp_input_param_.refine_path_point[i])) *
        nlp_input_param_.bias_weight;
  }
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    sgrad_f[i + 3 * nlp_input_param_.path_point_size] +=
        2.0 * sx[i + 3 * nlp_input_param_.path_point_size] *
        nlp_input_param_.kappa_weight;
  }
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    if (i == 0) {
      continue;
    }
    sgrad_f[i + 3 * nlp_input_param_.path_point_size - 1] +=
        -2.0 *
        (sx[i + 3 * nlp_input_param_.path_point_size] -
         sx[i + 3 * nlp_input_param_.path_point_size - 1]) *
        nlp_input_param_.dkappa_weight;
    sgrad_f[i + 3 * nlp_input_param_.path_point_size] +=
        2.0 *
        (sx[i + 3 * nlp_input_param_.path_point_size] -
         sx[i + 3 * nlp_input_param_.path_point_size - 1]) *
        nlp_input_param_.dkappa_weight;
  }

  return true;
}

bool IpoptPosOptimizeMathModel::eval_g(Index n, const Number* x, bool new_x,
                                       Index m, Number* g) {
  assert(n == finite_element_num_);
  assert(m == constraint_num_);
  (void)new_x;
  boost::span<const Number> sx(x, n);
  boost::span<Number> sg(g, m);
  size_t counter = 0;
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    if (i == 0) {
      sg[counter++] = 0.0;
      continue;
    }
    sg[counter++] = (sx[i] - sx[i - 1]) *
                        sin(sx[2 * nlp_input_param_.path_point_size + i - 1]) -
                    (sx[nlp_input_param_.path_point_size + i] -
                     sx[nlp_input_param_.path_point_size + i - 1]) *
                        cos(sx[2 * nlp_input_param_.path_point_size + i - 1]);
  }
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    if (i == 0) {
      sg[counter++] = 0.0;
      continue;
    }
    const double dx = sx[i] - sx[i - 1];
    const double dy = sx[nlp_input_param_.path_point_size + i] -
                      sx[nlp_input_param_.path_point_size + i - 1];
    sg[counter++] = sqrt(dx * dx + dy * dy) *
                        sx[3 * nlp_input_param_.path_point_size + i - 1] -
                    (sx[2 * nlp_input_param_.path_point_size + i] -
                     sx[2 * nlp_input_param_.path_point_size + i - 1]);
  }
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    if (i == 0) {
      sg[counter++] = 0.0;
      continue;
    }
    const double a = nlp_input_param_.constraint_line[i].equation_a;
    const double b = nlp_input_param_.constraint_line[i].equation_b;
    const double c = nlp_input_param_.constraint_line[i].equation_c;
    sg[counter++] =
        a * sx[i] + b * sx[nlp_input_param_.path_point_size + i] + c;
  }
  if (!nlp_input_param_.is_collision_free) {
    for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
      int32_t front_index = 0;
      if (nlp_input_param_.is_forward_path) {
        front_index = amin(i + nlp_input_param_.front_delta_index,
                           nlp_input_param_.path_point_size - 1);
      } else {
        front_index = amax(i - nlp_input_param_.front_delta_index, 0);
      }
      const double cos_theta =
          cos(sx[2 * nlp_input_param_.path_point_size + i]);
      const double sin_theta =
          sin(sx[2 * nlp_input_param_.path_point_size + i]);
      sg[counter++] =
          cos_theta *
              (nlp_input_param_.xy_lower_upper_bounds.second[front_index].y() -
               sx[nlp_input_param_.path_point_size + i]) -
          sin_theta *
              (nlp_input_param_.xy_lower_upper_bounds.second[front_index].x() -
               sx[i]);
      sg[counter++] =
          sin_theta *
              (nlp_input_param_.xy_lower_upper_bounds.first[front_index].x() -
               sx[i]) -
          cos_theta *
              (nlp_input_param_.xy_lower_upper_bounds.first[front_index].y() -
               sx[nlp_input_param_.path_point_size + i]);
    }
  }

  assert(counter == m);
  return true;
}

bool IpoptPosOptimizeMathModel::eval_jac_g(Index n, const Number* x, bool new_x,
                                           Index m, Index nele_jac, Index* iRow,
                                           Index* jCol, Number* values) {
  assert(n == finite_element_num_);
  assert(m == constraint_num_);
  (void)new_x;
  (void)nele_jac;
  std::vector<Index> nonzero_row;
  std::vector<Index> nonzero_col;
  Index idx = 0;
  if (values == nullptr) {
    SparseJacobianMatrix(m, n, x, &nonzero_row, &nonzero_col, nullptr);
    assert(nonzero_row.size() == nonzero_col.size());
    boost::span<Index> siRow(iRow, nonzero_row.size());
    boost::span<Index> sjCol(jCol, nonzero_col.size());
    for (size_t i = 0; i < nonzero_row.size(); ++i) {
      siRow[idx] = nonzero_row[i];
      sjCol[idx] = nonzero_col[i];
      idx++;
    }
  } else {
    std::vector<Number> nonzero_values;
    SparseJacobianMatrix(m, n, x, &nonzero_row, &nonzero_col, &nonzero_values);
    boost::span<Number> svalues(values, nonzero_values.size());
    for (const auto v : nonzero_values) {
      svalues[idx++] = v;
    }
  }
  assert(idx == nele_jac);

  return true;
}

bool IpoptPosOptimizeMathModel::eval_h(Index n, const Number* x, bool new_x,
                                       Number obj_factor, Index m,
                                       const Number* lambda, bool new_lambda,
                                       Index nele_hess, Index* iRow,
                                       Index* jCol, Number* values) {
  assert(n == finite_element_num_);
  assert(m == constraint_num_);
  (void)new_x;
  (void)new_lambda;
  (void)nele_hess;
  std::vector<Index> nonzero_row;
  std::vector<Index> nonzero_col;
  Index idx = 0;
  if (values == nullptr) {
    SparseHessianMatrix(m, n, x, obj_factor, lambda, &nonzero_row, &nonzero_col,
                        nullptr);
    assert(nonzero_row.size() == nonzero_col.size());
    boost::span<Index> siRow(iRow, nonzero_row.size());
    boost::span<Index> sjCol(jCol, nonzero_col.size());
    for (size_t i = 0; i < nonzero_row.size(); ++i) {
      siRow[idx] = nonzero_row[i];
      sjCol[idx] = nonzero_col[i];
      idx++;
    }
  } else {
    std::vector<Number> nonzero_values;
    SparseHessianMatrix(m, n, x, obj_factor, lambda, &nonzero_row, &nonzero_col,
                        &nonzero_values);
    boost::span<Number> svalues(values, nonzero_values.size());
    for (const auto v : nonzero_values) {
      svalues[idx++] = v;
    }
  }
  assert(idx == nele_hess);

  return true;
}

void IpoptPosOptimizeMathModel::finalize_solution(
    Ipopt::SolverReturn status, Index n, const Number* x, const Number* z_L,
    const Number* z_U, Index m, const Number* g, const Number* lambda,
    Number obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
  (void)status;
  (void)z_L;
  (void)z_U;
  (void)m;
  (void)g;
  (void)lambda;
  (void)obj_value;
  (void)ip_data;
  (void)ip_cq;
  boost::span<const Number> sx(x, n);
  optimized_result_.clear();
  optimized_result_.reserve(n);
  for (int i = 0; i < n; i++) {
    optimized_result_.push_back(sx[i]);
  }
}

void IpoptPosOptimizeMathModel::optimized_result(
    std::vector<double>* const result) const {
  assert(result != nullptr);
  *result = optimized_result_;
}

void IpoptPosOptimizeMathModel::SparseJacobianMatrix(
    const Ipopt::Index& m, const Ipopt::Index& n, const Ipopt::Number* x,
    std::vector<Ipopt::Index>* const nonzero_row,
    std::vector<Ipopt::Index>* const nonzero_col,
    std::vector<Ipopt::Number>* const nonzero_values) const {
  assert(n == finite_element_num_);
  assert(m == constraint_num_);
  assert(nonzero_row != nullptr);
  assert(nonzero_col != nullptr);
  (void)m;
  nonzero_row->clear();
  nonzero_col->clear();
  if (nonzero_values != nullptr) {
    assert(x != nullptr);
    nonzero_values->clear();
  }
  size_t counter = 0;
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    if (i == 0) {
      counter += n;
      continue;
    }
    if (nonzero_values == nullptr) {
      int32_t col = i - 1;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      col = i;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      col = i + nlp_input_param_.path_point_size - 1;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      col = i + nlp_input_param_.path_point_size;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      col = i + 2 * nlp_input_param_.path_point_size - 1;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
    } else {
      boost::span<const Number> sx(x, n);
      nonzero_values->push_back(
          -sin(sx[i + 2 * nlp_input_param_.path_point_size - 1]));
      nonzero_values->push_back(
          sin(sx[i + 2 * nlp_input_param_.path_point_size - 1]));
      nonzero_values->push_back(
          cos(sx[i + 2 * nlp_input_param_.path_point_size - 1]));
      nonzero_values->push_back(
          -cos(sx[i + 2 * nlp_input_param_.path_point_size - 1]));
      nonzero_values->push_back(
          (sx[i] - sx[i - 1]) *
              cos(sx[2 * nlp_input_param_.path_point_size + i - 1]) +
          (sx[nlp_input_param_.path_point_size + i] -
           sx[nlp_input_param_.path_point_size + i - 1]) *
              sin(sx[2 * nlp_input_param_.path_point_size + i - 1]));
    }
    counter += n;
  }
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    if (i == 0) {
      counter += n;
      continue;
    }
    if (nonzero_values == nullptr) {
      int32_t col = i - 1;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      col = i;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      col = i + nlp_input_param_.path_point_size - 1;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      col = i + nlp_input_param_.path_point_size;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      col = i + 2 * nlp_input_param_.path_point_size - 1;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      col = i + 2 * nlp_input_param_.path_point_size;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      col = i + 3 * nlp_input_param_.path_point_size - 1;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
    } else {
      boost::span<const Number> sx(x, n);
      const double dx = sx[i] - sx[i - 1];
      const double dy = sx[nlp_input_param_.path_point_size + i] -
                        sx[nlp_input_param_.path_point_size + i - 1];
      nonzero_values->push_back(
          sx[i + 3 * nlp_input_param_.path_point_size - 1] *
          (sx[i - 1] - sx[i]) / sqrt(dx * dx + dy * dy));
      nonzero_values->push_back(
          sx[i + 3 * nlp_input_param_.path_point_size - 1] *
          (sx[i] - sx[i - 1]) / sqrt(dx * dx + dy * dy));
      nonzero_values->push_back(
          sx[i + 3 * nlp_input_param_.path_point_size - 1] *
          (sx[i + nlp_input_param_.path_point_size - 1] -
           sx[i + nlp_input_param_.path_point_size]) /
          sqrt(dx * dx + dy * dy));
      nonzero_values->push_back(
          sx[i + 3 * nlp_input_param_.path_point_size - 1] *
          (sx[i + nlp_input_param_.path_point_size] -
           sx[i + nlp_input_param_.path_point_size - 1]) /
          sqrt(dx * dx + dy * dy));
      nonzero_values->push_back(1);
      nonzero_values->push_back(-1);
      nonzero_values->push_back(sqrt(dx * dx + dy * dy));
    }
    counter += n;
  }
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    if (i == 0) {
      counter += n;
      continue;
    }
    if (nonzero_values == nullptr) {
      int32_t col = i;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      col = i + nlp_input_param_.path_point_size;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
    } else {
      const double a = nlp_input_param_.constraint_line[i].equation_a;
      const double b = nlp_input_param_.constraint_line[i].equation_b;
      nonzero_values->push_back(a);
      nonzero_values->push_back(b);
    }
    counter += n;
  }

  if (!nlp_input_param_.is_collision_free) {
    for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
      int32_t front_index = 0;
      if (nlp_input_param_.is_forward_path) {
        front_index = amin(i + nlp_input_param_.front_delta_index,
                           nlp_input_param_.path_point_size - 1);
      } else {
        front_index = amax(i - nlp_input_param_.front_delta_index, 0);
      }

      if (nonzero_values == nullptr) {
        int32_t col = i;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = i + nlp_input_param_.path_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = i + 2 * nlp_input_param_.path_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      } else {
        boost::span<const Number> sx(x, n);
        const double cos_theta =
            cos(sx[2 * nlp_input_param_.path_point_size + i]);
        const double sin_theta =
            sin(sx[2 * nlp_input_param_.path_point_size + i]);
        const double dx =
            nlp_input_param_.xy_lower_upper_bounds.second[front_index].x() -
            sx[i];
        const double dy =
            nlp_input_param_.xy_lower_upper_bounds.second[front_index].y() -
            sx[nlp_input_param_.path_point_size + i];
        nonzero_values->push_back(sin_theta);
        nonzero_values->push_back(-cos_theta);
        nonzero_values->push_back(-dx * cos_theta - dy * sin_theta);
      }
      counter += n;

      if (nonzero_values == nullptr) {
        int32_t col = i;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = i + nlp_input_param_.path_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = i + 2 * nlp_input_param_.path_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      } else {
        boost::span<const Number> sx(x, n);
        const double cos_theta =
            cos(sx[2 * nlp_input_param_.path_point_size + i]);
        const double sin_theta =
            sin(sx[2 * nlp_input_param_.path_point_size + i]);
        const double dx =
            nlp_input_param_.xy_lower_upper_bounds.first[front_index].x() -
            sx[i];
        const double dy =
            nlp_input_param_.xy_lower_upper_bounds.first[front_index].y() -
            sx[nlp_input_param_.path_point_size + i];
        nonzero_values->push_back(-sin_theta);
        nonzero_values->push_back(cos_theta);
        nonzero_values->push_back(dx * cos_theta + dy * sin_theta);
      }
      counter += n;
    }
  }

  assert(nonzero_row->size() == nonzero_col->size());
}

void IpoptPosOptimizeMathModel::SparseHessianMatrix(
    const Ipopt::Index& m, const Ipopt::Index& n, const Ipopt::Number* x,
    Ipopt::Number obj_factor, const Ipopt::Number* lambda,
    std::vector<Ipopt::Index>* const nonzero_row,
    std::vector<Ipopt::Index>* const nonzero_col,
    std::vector<Ipopt::Number>* const nonzero_values) const {
  assert(n == finite_element_num_);
  assert(m == constraint_num_);
  assert(nonzero_row != nullptr);
  assert(nonzero_col != nullptr);
  nonzero_row->clear();
  nonzero_col->clear();
  if (nonzero_values != nullptr) {
    assert(x != nullptr);
    assert(lambda != nullptr);
    nonzero_values->clear();
  }

  std::vector<std::vector<std::pair<bool, double>>> matrix_n(
      n, std::vector<std::pair<bool, double>>(n, {false, 0.0}));
  // objective
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    if (i == 0) {
      continue;
    }
    matrix_n[i][i].first = true;
    matrix_n[i + nlp_input_param_.path_point_size]
            [i + nlp_input_param_.path_point_size]
                .first = true;
    if (nonzero_values != nullptr) {
      matrix_n[i][i].second += 2.0 * nlp_input_param_.bias_weight * obj_factor;
      matrix_n[i + nlp_input_param_.path_point_size]
              [i + nlp_input_param_.path_point_size]
                  .second += 2.0 * nlp_input_param_.bias_weight * obj_factor;
    }
  }

  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    matrix_n[i + 3 * nlp_input_param_.path_point_size]
            [i + 3 * nlp_input_param_.path_point_size]
                .first = true;
    if (nonzero_values != nullptr) {
      matrix_n[i + 3 * nlp_input_param_.path_point_size]
              [i + 3 * nlp_input_param_.path_point_size]
                  .second += 2.0 * nlp_input_param_.kappa_weight * obj_factor;
    }
  }

  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    if (i == 0) {
      continue;
    }
    matrix_n[i + 3 * nlp_input_param_.path_point_size - 1]
            [i + 3 * nlp_input_param_.path_point_size - 1]
                .first = true;
    matrix_n[i + 3 * nlp_input_param_.path_point_size]
            [i + 3 * nlp_input_param_.path_point_size - 1]
                .first = true;
    matrix_n[i + 3 * nlp_input_param_.path_point_size]
            [i + 3 * nlp_input_param_.path_point_size]
                .first = true;
    if (nonzero_values != nullptr) {
      matrix_n[i + 3 * nlp_input_param_.path_point_size - 1]
              [i + 3 * nlp_input_param_.path_point_size - 1]
                  .second += 2.0 * nlp_input_param_.dkappa_weight * obj_factor;
      matrix_n[i + 3 * nlp_input_param_.path_point_size]
              [i + 3 * nlp_input_param_.path_point_size - 1]
                  .second += -2.0 * nlp_input_param_.dkappa_weight * obj_factor;
      matrix_n[i + 3 * nlp_input_param_.path_point_size]
              [i + 3 * nlp_input_param_.path_point_size]
                  .second += 2.0 * nlp_input_param_.dkappa_weight * obj_factor;
    }
  }

  // constraint
  size_t counter = 0;
  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    if (i == 0) {
      counter++;
      continue;
    }
    matrix_n[i + 2 * nlp_input_param_.path_point_size - 1][i - 1].first = true;
    matrix_n[i + 2 * nlp_input_param_.path_point_size - 1][i].first = true;
    matrix_n[i + 2 * nlp_input_param_.path_point_size - 1]
            [i + nlp_input_param_.path_point_size - 1]
                .first = true;
    matrix_n[i + 2 * nlp_input_param_.path_point_size - 1]
            [i + nlp_input_param_.path_point_size]
                .first = true;
    matrix_n[i + 2 * nlp_input_param_.path_point_size - 1]
            [i + 2 * nlp_input_param_.path_point_size - 1]
                .first = true;
    if (nonzero_values != nullptr) {
      boost::span<const Number> sx(x, n);
      boost::span<const Number> slambda(lambda, m);
      matrix_n[i + 2 * nlp_input_param_.path_point_size - 1][i - 1].second +=
          -cos(sx[i + 2 * nlp_input_param_.path_point_size - 1]) *
          slambda[counter];
      matrix_n[i + 2 * nlp_input_param_.path_point_size - 1][i].second +=
          cos(sx[i + 2 * nlp_input_param_.path_point_size - 1]) *
          slambda[counter];
      matrix_n[i + 2 * nlp_input_param_.path_point_size -
               1][i + nlp_input_param_.path_point_size - 1]
          .second += -sin(sx[i + 2 * nlp_input_param_.path_point_size - 1]) *
                     slambda[counter];
      matrix_n[i + 2 * nlp_input_param_.path_point_size -
               1][i + nlp_input_param_.path_point_size]
          .second += sin(sx[i + 2 * nlp_input_param_.path_point_size - 1]) *
                     slambda[counter];
      matrix_n[i + 2 * nlp_input_param_.path_point_size -
               1][i + 2 * nlp_input_param_.path_point_size - 1]
          .second += (sx[i + nlp_input_param_.path_point_size] -
                      sx[i + nlp_input_param_.path_point_size - 1]) *
                         cos(sx[i + 2 * nlp_input_param_.path_point_size - 1]) *
                         slambda[counter] -
                     (sx[i] - sx[i - 1]) *
                         sin(sx[i + 2 * nlp_input_param_.path_point_size - 1]) *
                         slambda[counter];
    }
    counter++;
  }

  for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
    if (i == 0) {
      counter++;
      continue;
    }
    matrix_n[i - 1][i - 1].first = true;
    matrix_n[i][i - 1].first = true;
    matrix_n[i][i].first = true;
    matrix_n[i + nlp_input_param_.path_point_size - 1][i - 1].first = true;
    matrix_n[i + nlp_input_param_.path_point_size - 1][i].first = true;
    matrix_n[i + nlp_input_param_.path_point_size - 1]
            [i + nlp_input_param_.path_point_size - 1]
                .first = true;
    matrix_n[i + nlp_input_param_.path_point_size][i - 1].first = true;
    matrix_n[i + nlp_input_param_.path_point_size][i].first = true;
    matrix_n[i + nlp_input_param_.path_point_size]
            [i + nlp_input_param_.path_point_size - 1]
                .first = true;
    matrix_n[i + nlp_input_param_.path_point_size]
            [i + nlp_input_param_.path_point_size]
                .first = true;
    matrix_n[i + 3 * nlp_input_param_.path_point_size - 1][i - 1].first = true;
    matrix_n[i + 3 * nlp_input_param_.path_point_size - 1][i].first = true;
    matrix_n[i + 3 * nlp_input_param_.path_point_size - 1]
            [i + nlp_input_param_.path_point_size - 1]
                .first = true;
    matrix_n[i + 3 * nlp_input_param_.path_point_size - 1]
            [i + nlp_input_param_.path_point_size]
                .first = true;
    if (nonzero_values != nullptr) {
      boost::span<const Number> sx(x, n);
      boost::span<const Number> slambda(lambda, m);
      const double dx = sx[i] - sx[i - 1];
      const double dy = sx[i + nlp_input_param_.path_point_size] -
                        sx[i + nlp_input_param_.path_point_size - 1];
      matrix_n[i - 1][i - 1].second +=
          slambda[counter] * sx[i + 3 * nlp_input_param_.path_point_size - 1] *
          (1.0 / sqrt(dx * dx + dy * dy) -
           dx * dx / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy));
      matrix_n[i][i - 1].second +=
          slambda[counter] * sx[i + 3 * nlp_input_param_.path_point_size - 1] *
          (dx * dx / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy) -
           1.0 / sqrt(dx * dx + dy * dy));
      matrix_n[i][i].second +=
          slambda[counter] * sx[i + 3 * nlp_input_param_.path_point_size - 1] *
          (1.0 / sqrt(dx * dx + dy * dy) -
           dx * dx / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy));
      matrix_n[i + nlp_input_param_.path_point_size - 1][i - 1].second +=
          -slambda[counter] * sx[i + 3 * nlp_input_param_.path_point_size - 1] *
          dx * dy / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy);
      matrix_n[i + nlp_input_param_.path_point_size - 1][i].second +=
          slambda[counter] * sx[i + 3 * nlp_input_param_.path_point_size - 1] *
          dx * dy / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy);
      matrix_n[i + nlp_input_param_.path_point_size - 1]
              [i + nlp_input_param_.path_point_size - 1]
                  .second +=
          slambda[counter] * sx[i + 3 * nlp_input_param_.path_point_size - 1] *
          (1.0 / sqrt(dx * dx + dy * dy) -
           dy * dy / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy));
      matrix_n[i + nlp_input_param_.path_point_size][i - 1].second +=
          slambda[counter] * sx[i + 3 * nlp_input_param_.path_point_size - 1] *
          dx * dy / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy);
      matrix_n[i + nlp_input_param_.path_point_size][i].second +=
          -slambda[counter] * sx[i + 3 * nlp_input_param_.path_point_size - 1] *
          dx * dy / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy);
      matrix_n[i + nlp_input_param_.path_point_size]
              [i + nlp_input_param_.path_point_size - 1]
                  .second +=
          slambda[counter] * sx[i + 3 * nlp_input_param_.path_point_size - 1] *
          (dy * dy / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy) -
           1.0 / sqrt(dx * dx + dy * dy));
      matrix_n[i + nlp_input_param_.path_point_size]
              [i + nlp_input_param_.path_point_size]
                  .second +=
          slambda[counter] * sx[i + 3 * nlp_input_param_.path_point_size - 1] *
          (1.0 / sqrt(dx * dx + dy * dy) -
           dy * dy / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy));
      matrix_n[i + 3 * nlp_input_param_.path_point_size - 1][i - 1].second +=
          -slambda[counter] * dx / sqrt(dx * dx + dy * dy);
      matrix_n[i + 3 * nlp_input_param_.path_point_size - 1][i].second +=
          slambda[counter] * dx / sqrt(dx * dx + dy * dy);
      matrix_n[i + 3 * nlp_input_param_.path_point_size - 1]
              [i + nlp_input_param_.path_point_size - 1]
                  .second += -slambda[counter] * dy / sqrt(dx * dx + dy * dy);
      matrix_n[i + 3 * nlp_input_param_.path_point_size - 1]
              [i + nlp_input_param_.path_point_size]
                  .second += slambda[counter] * dy / sqrt(dx * dx + dy * dy);
    }
    counter++;
  }

  counter += nlp_input_param_.path_point_size;

  if (!nlp_input_param_.is_collision_free) {
    for (int32_t i = 0; i < nlp_input_param_.path_point_size; ++i) {
      int32_t front_index = 0;
      if (nlp_input_param_.is_forward_path) {
        front_index = amin(i + nlp_input_param_.front_delta_index,
                           nlp_input_param_.path_point_size - 1);
      } else {
        front_index = amax(i - nlp_input_param_.front_delta_index, 0);
      }

      matrix_n[2 * nlp_input_param_.path_point_size + i][i].first = true;
      matrix_n[2 * nlp_input_param_.path_point_size + i]
              [nlp_input_param_.path_point_size + i]
                  .first = true;
      matrix_n[2 * nlp_input_param_.path_point_size + i]
              [2 * nlp_input_param_.path_point_size + i]
                  .first = true;
      if (nonzero_values != nullptr) {
        boost::span<const Number> sx(x, n);
        boost::span<const Number> slambda(lambda, m);
        const double cos_theta =
            cos(sx[2 * nlp_input_param_.path_point_size + i]);
        const double sin_theta =
            sin(sx[2 * nlp_input_param_.path_point_size + i]);
        const double dx =
            nlp_input_param_.xy_lower_upper_bounds.second[front_index].x() -
            sx[i];
        const double dy =
            nlp_input_param_.xy_lower_upper_bounds.second[front_index].y() -
            sx[nlp_input_param_.path_point_size + i];
        matrix_n[2 * nlp_input_param_.path_point_size + i][i].second +=
            slambda[counter] * cos_theta;
        matrix_n[2 * nlp_input_param_.path_point_size + i]
                [nlp_input_param_.path_point_size + i]
                    .second += slambda[counter] * sin_theta;
        matrix_n[2 * nlp_input_param_.path_point_size +
                 i][2 * nlp_input_param_.path_point_size + i]
            .second += slambda[counter] * (-dy * cos_theta + dx * sin_theta);
      }
      counter++;

      if (nonzero_values != nullptr) {
        boost::span<const Number> sx(x, n);
        boost::span<const Number> slambda(lambda, m);
        const double cos_theta =
            cos(sx[2 * nlp_input_param_.path_point_size + i]);
        const double sin_theta =
            sin(sx[2 * nlp_input_param_.path_point_size + i]);
        const double dx =
            nlp_input_param_.xy_lower_upper_bounds.first[front_index].x() -
            sx[i];
        const double dy =
            nlp_input_param_.xy_lower_upper_bounds.first[front_index].y() -
            sx[nlp_input_param_.path_point_size + i];
        matrix_n[2 * nlp_input_param_.path_point_size + i][i].second +=
            -slambda[counter] * cos_theta;
        matrix_n[2 * nlp_input_param_.path_point_size + i]
                [nlp_input_param_.path_point_size + i]
                    .second += -slambda[counter] * sin_theta;
        matrix_n[2 * nlp_input_param_.path_point_size +
                 i][2 * nlp_input_param_.path_point_size + i]
            .second += slambda[counter] * (dy * cos_theta - dx * sin_theta);
      }
      counter++;
    }
  }

  ADEBUG << "counter: " << counter << ", m: " << m;

  for (int row = 0; row < n; row++) {
    for (int col = 0; col <= row; col++) {
      if (matrix_n[row][col].first) {
        if (nonzero_values == nullptr) {
          nonzero_row->push_back(row);
          nonzero_col->push_back(col);
        } else {
          nonzero_values->push_back(matrix_n[row][col].second);
        }
      }
    }
  }

  assert(nonzero_row->size() == nonzero_col->size());
}

}  // namespace TL::planning
