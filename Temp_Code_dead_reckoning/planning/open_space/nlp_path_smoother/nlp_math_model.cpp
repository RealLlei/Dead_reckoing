/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  nlp_math_model.cpp
 */

#include "planning/open_space/nlp_path_smoother/nlp_math_model.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "coin/IpTypes.hpp"
#include "common/util/span.hpp"

namespace TL {
namespace planning {

using Ipopt::Index;
using Ipopt::Number;

namespace {
constexpr double kMaxValue = 1e20;
}  // namespace

NlpMathModel::NlpMathModel(NlpInputParam nlp_input_param)
    : nlp_input_param_(std::move(nlp_input_param)),
      finite_element_num_(
          4 * static_cast<int32_t>(nlp_input_param_.total_point_size)),
      constraint_num_(
          3 * static_cast<int32_t>(nlp_input_param_.total_point_size) +
          3 * static_cast<int32_t>(nlp_input_param_.path_point_size.size() -
                                   1) +
          4 * static_cast<int32_t>(nlp_input_param_.total_point_size)) {}

bool NlpMathModel::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                                Index& nnz_h_lag, IndexStyleEnum& index_style) {
  // x,y,theta,kappa
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

bool NlpMathModel::get_bounds_info(Index n, Number* x_l, Number* x_u, Index m,
                                   Number* g_l, Number* g_u) {
  assert(n == finite_element_num_);
  assert(m == constraint_num_);

  boost::span<Number> sx_l(x_l, n);
  boost::span<Number> sx_u(x_u, n);
  uint32_t index = 0;
  for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
    for (uint32_t j = 0; j < nlp_input_param_.path_point_size[i]; ++j) {
      sx_l[index] = nlp_input_param_.constraint_lines[i][j].min_x;
      sx_u[index] = nlp_input_param_.constraint_lines[i][j].max_x;

      sx_l[index + nlp_input_param_.total_point_size] =
          nlp_input_param_.constraint_lines[i][j].min_y;
      sx_u[index + nlp_input_param_.total_point_size] =
          nlp_input_param_.constraint_lines[i][j].max_y;

      sx_l[index + 2 * nlp_input_param_.total_point_size] = -kMaxValue;
      sx_u[index + 2 * nlp_input_param_.total_point_size] = kMaxValue;

      sx_l[index + 3 * nlp_input_param_.total_point_size] =
          -nlp_input_param_.max_kappa;
      sx_u[index + 3 * nlp_input_param_.total_point_size] =
          nlp_input_param_.max_kappa;
      index++;
    }
  }

  sx_l[0] = std::get<0>(nlp_input_param_.refine_paths.front().front());
  sx_u[0] = std::get<0>(nlp_input_param_.refine_paths.front().front());
  sx_l[nlp_input_param_.total_point_size] =
      std::get<1>(nlp_input_param_.refine_paths.front().front());
  sx_u[nlp_input_param_.total_point_size] =
      std::get<1>(nlp_input_param_.refine_paths.front().front());
  sx_l[static_cast<int32_t>(2 * nlp_input_param_.total_point_size)] =
      std::get<2>(nlp_input_param_.refine_paths.front().front());
  sx_u[static_cast<int32_t>(2 * nlp_input_param_.total_point_size)] =
      std::get<2>(nlp_input_param_.refine_paths.front().front());
  if (nlp_input_param_.enable_fix_start_kappa) {
    sx_l[static_cast<int32_t>(3 * nlp_input_param_.total_point_size)] =
        std::get<3>(nlp_input_param_.refine_paths.front().front());
    sx_u[static_cast<int32_t>(3 * nlp_input_param_.total_point_size)] =
        std::get<3>(nlp_input_param_.refine_paths.front().front());
  }

  if (!nlp_input_param_.enable_dest_lat_region_constrain) {
    sx_l[nlp_input_param_.total_point_size - 1] =
        std::get<0>(nlp_input_param_.refine_paths.back().back());
    sx_u[nlp_input_param_.total_point_size - 1] =
        std::get<0>(nlp_input_param_.refine_paths.back().back());
    sx_l[2 * nlp_input_param_.total_point_size - 1] =
        std::get<1>(nlp_input_param_.refine_paths.back().back());
    sx_u[2 * nlp_input_param_.total_point_size - 1] =
        std::get<1>(nlp_input_param_.refine_paths.back().back());
  }
  sx_l[3 * nlp_input_param_.total_point_size - 1] =
      std::get<2>(nlp_input_param_.refine_paths.back().back());
  sx_u[3 * nlp_input_param_.total_point_size - 1] =
      std::get<2>(nlp_input_param_.refine_paths.back().back());
  // sx_l[4 * nlp_input_param_.total_point_size - 1] =
  //      std::get<3>(nlp_input_param_.refine_paths.back().back());
  // sx_u[4 * nlp_input_param_.total_point_size - 1] =
  //      std::get<3>(nlp_input_param_.refine_paths.back().back());

  boost::span<Number> sg_l(g_l, m);
  boost::span<Number> sg_u(g_u, m);
  for (uint32_t i = 0; i < 3 * (nlp_input_param_.total_point_size +
                                nlp_input_param_.path_point_size.size() - 1);
       i++) {
    sg_l[i] = 0.0;
    sg_u[i] = 0.0;
  }
  for (uint32_t i = 3 * (nlp_input_param_.total_point_size +
                         nlp_input_param_.path_point_size.size() - 1);
       i < m; i++) {
    sg_l[i] = nlp_input_param_.safety_buffer;
    sg_u[i] = kMaxValue;
  }

  return true;
}

bool NlpMathModel::get_starting_point(Index n, bool init_x, Number* x,
                                      bool init_z, Number* z_L, Number* z_U,
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
  uint32_t index = 0;
  for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
    for (uint32_t j = 0; j < nlp_input_param_.path_point_size[i]; ++j) {
      sx[index] = std::get<0>(nlp_input_param_.refine_paths[i][j]);
      sx[nlp_input_param_.total_point_size + index] =
          std::get<1>(nlp_input_param_.refine_paths[i][j]);
      sx[2 * nlp_input_param_.total_point_size + index] =
          std::get<2>(nlp_input_param_.refine_paths[i][j]);
      sx[3 * nlp_input_param_.total_point_size + index] =
          std::get<3>(nlp_input_param_.refine_paths[i][j]);
      index++;
    }
  }

  return true;
}

bool NlpMathModel::eval_f(Index n, const Number* x, bool new_x,
                          Number& obj_value) {
  assert(n == finite_element_num_);
  (void)new_x;
  boost::span<const Number> sx(x, n);
  // bias_square = (xi - xir)^2 + (yi - yir)^2
  size_t index = 0;
  double bias_square = 0.0;
  for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
    for (uint32_t j = 0; j < nlp_input_param_.path_point_size[i]; ++j) {
      if (j == 0) {
        index++;
        continue;
      }
      const double dx =
          sx[index] - std::get<0>(nlp_input_param_.refine_paths[i][j]);
      const double dy = sx[nlp_input_param_.total_point_size + index] -
                        std::get<1>(nlp_input_param_.refine_paths[i][j]);
      bias_square += dx * dx + dy * dy;
      index++;
    }
  }
  // kappa_square = ki * ki
  double kappa_square = 0.0;
  for (uint32_t i = 0; i < nlp_input_param_.total_point_size; ++i) {
    kappa_square += sx[3 * nlp_input_param_.total_point_size + i] *
                    sx[3 * nlp_input_param_.total_point_size + i];
  }
  // dkappa_square = (ki - ki-1)^2
  index = 0;
  double dkappa_square = 0.0;
  for (uint32_t i : nlp_input_param_.path_point_size) {
    for (uint32_t j = 0; j < i; ++j) {
      if (j == 0) {
        index++;
        continue;
      }
      const double dkappa =
          sx[3 * static_cast<size_t>(nlp_input_param_.total_point_size) +
             index] -
          sx[3 * static_cast<size_t>(nlp_input_param_.total_point_size) +
             index - 1];
      dkappa_square += dkappa * dkappa;
      index++;
    }
  }
  obj_value = nlp_input_param_.bias_weight * bias_square +
              nlp_input_param_.kappa_weight * kappa_square +
              nlp_input_param_.dkappa_weight * dkappa_square;

  return true;
}

bool NlpMathModel::eval_grad_f(Index n, const Number* x, bool new_x,
                               Number* grad_f) {
  assert(n == finite_element_num_);
  (void)new_x;
  boost::span<const Number> sx(x, n);
  boost::span<Number> sgrad_f(grad_f, n);
  for (uint32_t i = 0; i < nlp_input_param_.total_point_size; ++i) {
    sgrad_f[i] = 0.0;
    sgrad_f[i + nlp_input_param_.total_point_size] = 0.0;
    sgrad_f[i + 2 * nlp_input_param_.total_point_size] = 0.0;
    sgrad_f[i + 3 * nlp_input_param_.total_point_size] = 0.0;
  }

  uint32_t index = 0;
  for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
    for (uint32_t j = 0; j < nlp_input_param_.path_point_size[i]; ++j) {
      if (j == 0) {
        index++;
        continue;
      }
      sgrad_f[index] +=
          2.0 * (sx[index] - std::get<0>(nlp_input_param_.refine_paths[i][j])) *
          nlp_input_param_.bias_weight;
      sgrad_f[index + nlp_input_param_.total_point_size] +=
          2.0 *
          (sx[index + nlp_input_param_.total_point_size] -
           std::get<1>(nlp_input_param_.refine_paths[i][j])) *
          nlp_input_param_.bias_weight;
      index++;
    }
  }
  for (uint32_t i = 0; i < nlp_input_param_.total_point_size; ++i) {
    sgrad_f[i + 3 * nlp_input_param_.total_point_size] +=
        2.0 * sx[i + 3 * nlp_input_param_.total_point_size] *
        nlp_input_param_.kappa_weight;
  }
  index = 0;
  for (uint32_t i : nlp_input_param_.path_point_size) {
    for (uint32_t j = 0; j < i; ++j) {
      if (j == 0) {
        index++;
        continue;
      }
      sgrad_f[index + 3 * nlp_input_param_.total_point_size - 1] +=
          -2.0 *
          (sx[index + 3 * nlp_input_param_.total_point_size] -
           sx[index + 3 * nlp_input_param_.total_point_size - 1]) *
          nlp_input_param_.dkappa_weight;
      sgrad_f[index + 3 * nlp_input_param_.total_point_size] +=
          2.0 *
          (sx[index + 3 * nlp_input_param_.total_point_size] -
           sx[index + 3 * nlp_input_param_.total_point_size - 1]) *
          nlp_input_param_.dkappa_weight;
      index++;
    }
  }

  return true;
}

bool NlpMathModel::eval_g(Index n, const Number* x, bool new_x, Index m,
                          Number* g) {
  assert(n == finite_element_num_);
  assert(m == constraint_num_);
  (void)new_x;
  boost::span<const Number> sx(x, n);
  boost::span<Number> sg(g, m);
  uint32_t index = 0;
  size_t counter = 0;
  for (uint32_t i : nlp_input_param_.path_point_size) {
    for (uint32_t j = 0; j < i; ++j) {
      if (j == 0) {
        sg[counter++] = 0.0;
        index++;
        continue;
      }
      // (xi - xi-1) * sin(thetai-1) - (yi - yi-1) * cos(thetai-1) = 0
      sg[counter++] =
          (sx[index] - sx[index - 1]) *
              sin(sx[2 * nlp_input_param_.total_point_size + index - 1]) -
          (sx[nlp_input_param_.total_point_size + index] -
           sx[nlp_input_param_.total_point_size + index - 1]) *
              cos(sx[2 * nlp_input_param_.total_point_size + index - 1]);
      index++;
    }
  }
  index = 0;
  for (uint32_t i : nlp_input_param_.path_point_size) {
    for (uint32_t j = 0; j < i; ++j) {
      if (j == 0) {
        sg[counter++] = 0.0;
        index++;
        continue;
      }
      const double dx = sx[index] - sx[index - 1];
      const double dy = sx[nlp_input_param_.total_point_size + index] -
                        sx[nlp_input_param_.total_point_size + index - 1];
      // sqrt(dx * dx + dy * dy) * ki-1 - (thetai - thetai-1) = 0
      sg[counter++] =
          sqrt(dx * dx + dy * dy) *
              sx[3 * nlp_input_param_.total_point_size + index - 1] -
          (sx[2 * nlp_input_param_.total_point_size + index] -
           sx[2 * nlp_input_param_.total_point_size + index - 1]);
      index++;
    }
  }
  index = 0;
  for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
    for (uint32_t j = 0; j < nlp_input_param_.path_point_size[i]; ++j) {
      if (j == 0) {
        sg[counter++] = 0.0;
        index++;
        continue;
      }
      // a * xi + b * yi + c = 0
      const double a = nlp_input_param_.constraint_lines[i][j].equation_a;
      const double b = nlp_input_param_.constraint_lines[i][j].equation_b;
      const double c = nlp_input_param_.constraint_lines[i][j].equation_c;
      sg[counter++] =
          a * sx[index] + b * sx[nlp_input_param_.total_point_size + index] + c;
      index++;
    }
  }
  index = 0;
  for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
    if (i == 0) {
      index = nlp_input_param_.path_point_size[i] - 1;
      continue;
    }
    // xi - xi+1 = 0
    sg[counter++] = sx[index] - sx[index + 1];
    // yi - yi+1 = 0
    sg[counter++] = sx[index + nlp_input_param_.total_point_size] -
                    sx[index + nlp_input_param_.total_point_size + 1];
    // thetai - thetai+1 = 0
    sg[counter++] = sx[index + 2 * nlp_input_param_.total_point_size] -
                    sx[index + 2 * nlp_input_param_.total_point_size + 1];
    index += nlp_input_param_.path_point_size[i];
  }

  // four coner constrain
  index = 0;
  for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
    for (uint32_t j = 0; j < nlp_input_param_.path_point_size[i]; ++j) {
      const auto& left_bound_start =
          nlp_input_param_.refine_road_bounds[i][j].second.start();
      const auto& right_bound_start =
          nlp_input_param_.refine_road_bounds[i][j].first.start();
      const double sin_left_angle =
          sin(nlp_input_param_.refine_road_bounds[i][j].second.heading());
      const double cos_left_angle =
          cos(nlp_input_param_.refine_road_bounds[i][j].second.heading());
      const double sin_right_angle =
          sin(nlp_input_param_.refine_road_bounds[i][j].first.heading());
      const double cos_right_angle =
          cos(nlp_input_param_.refine_road_bounds[i][j].first.heading());
      const auto lf =
          FLUToENU(nlp_input_param_.front_collision_length,
                   nlp_input_param_.half_collision_width, sx[index],
                   sx[nlp_input_param_.total_point_size + index],
                   sx[2 * nlp_input_param_.total_point_size + index]);
      sg[counter++] = sin_left_angle * (lf.first - left_bound_start.x()) -
                      cos_left_angle * (lf.second - left_bound_start.y());
      const auto lb =
          FLUToENU(-nlp_input_param_.back_collision_length,
                   nlp_input_param_.half_collision_width, sx[index],
                   sx[nlp_input_param_.total_point_size + index],
                   sx[2 * nlp_input_param_.total_point_size + index]);
      sg[counter++] = sin_left_angle * (lb.first - left_bound_start.x()) -
                      cos_left_angle * (lb.second - left_bound_start.y());
      const auto rf =
          FLUToENU(nlp_input_param_.front_collision_length,
                   -nlp_input_param_.half_collision_width, sx[index],
                   sx[nlp_input_param_.total_point_size + index],
                   sx[2 * nlp_input_param_.total_point_size + index]);
      sg[counter++] = -sin_right_angle * (rf.first - right_bound_start.x()) +
                      cos_right_angle * (rf.second - right_bound_start.y());
      const auto rb =
          FLUToENU(-nlp_input_param_.back_collision_length,
                   -nlp_input_param_.half_collision_width, sx[index],
                   sx[nlp_input_param_.total_point_size + index],
                   sx[2 * nlp_input_param_.total_point_size + index]);
      sg[counter++] = -sin_right_angle * (rb.first - right_bound_start.x()) +
                      cos_right_angle * (rb.second - right_bound_start.y());
      index++;
    }
  }

  assert(counter == m);
  return true;
}

std::pair<double, double> NlpMathModel::FLUToENU(const double x, const double y,
                                                 const double ENU_x,
                                                 const double ENU_y,
                                                 const double ENU_heading) {
  auto final_x = ENU_x + cos(ENU_heading) * x - sin(ENU_heading) * y;
  auto final_y = ENU_y + sin(ENU_heading) * x + cos(ENU_heading) * y;
  return std::make_pair(final_x, final_y);
}

bool NlpMathModel::eval_jac_g(Index n, const Number* x, bool new_x, Index m,
                              Index nele_jac, Index* iRow, Index* jCol,
                              Number* values) {
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

bool NlpMathModel::eval_h(Index n, const Number* x, bool new_x,
                          Number obj_factor, Index m, const Number* lambda,
                          bool new_lambda, Index nele_hess, Index* iRow,
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

void NlpMathModel::finalize_solution(Ipopt::SolverReturn status, Index n,
                                     const Number* x, const Number* z_L,
                                     const Number* z_U, Index m,
                                     const Number* g, const Number* lambda,
                                     Number obj_value,
                                     const Ipopt::IpoptData* ip_data,
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

void NlpMathModel::optimized_result(std::vector<double>* const result) const {
  assert(result != nullptr);
  *result = optimized_result_;
}

void NlpMathModel::SparseJacobianMatrix(
    const Index m, const Index n, const Number* x,
    std::vector<Index>* const nonzero_row,
    std::vector<Index>* const nonzero_col,
    std::vector<Number>* const nonzero_values) {
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
  uint32_t index = 0;
  for (uint32_t i : nlp_input_param_.path_point_size) {
    for (uint32_t j = 0; j < i; ++j) {
      if (j == 0) {
        counter += n;
        index++;
        continue;
      }
      if (nonzero_values == nullptr) {
        uint32_t col = index - 1;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + nlp_input_param_.total_point_size - 1;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + nlp_input_param_.total_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + 2 * nlp_input_param_.total_point_size - 1;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      } else {
        boost::span<const Number> sx(x, n);
        nonzero_values->push_back(
            -sin(sx[index + 2 * nlp_input_param_.total_point_size - 1]));
        nonzero_values->push_back(
            sin(sx[index + 2 * nlp_input_param_.total_point_size - 1]));
        nonzero_values->push_back(
            cos(sx[index + 2 * nlp_input_param_.total_point_size - 1]));
        nonzero_values->push_back(
            -cos(sx[index + 2 * nlp_input_param_.total_point_size - 1]));
        nonzero_values->push_back(
            (sx[index] - sx[index - 1]) *
                cos(sx[2 * nlp_input_param_.total_point_size + index - 1]) +
            (sx[nlp_input_param_.total_point_size + index] -
             sx[nlp_input_param_.total_point_size + index - 1]) *
                sin(sx[2 * nlp_input_param_.total_point_size + index - 1]));
      }
      counter += n;
      index++;
    }
  }
  index = 0;
  for (uint32_t i : nlp_input_param_.path_point_size) {
    for (uint32_t j = 0; j < i; ++j) {
      if (j == 0) {
        counter += n;
        index++;
        continue;
      }
      if (nonzero_values == nullptr) {
        uint32_t col = index - 1;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + nlp_input_param_.total_point_size - 1;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + nlp_input_param_.total_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + 2 * nlp_input_param_.total_point_size - 1;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + 2 * nlp_input_param_.total_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + 3 * nlp_input_param_.total_point_size - 1;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      } else {
        boost::span<const Number> sx(x, n);
        const double dx = sx[index] - sx[index - 1];
        const double dy = sx[nlp_input_param_.total_point_size + index] -
                          sx[nlp_input_param_.total_point_size + index - 1];
        nonzero_values->push_back(
            sx[index + 3 * nlp_input_param_.total_point_size - 1] *
            (sx[index - 1] - sx[index]) / sqrt(dx * dx + dy * dy));
        nonzero_values->push_back(
            sx[index + 3 * nlp_input_param_.total_point_size - 1] *
            (sx[index] - sx[index - 1]) / sqrt(dx * dx + dy * dy));
        nonzero_values->push_back(
            sx[index + 3 * nlp_input_param_.total_point_size - 1] *
            (sx[index + nlp_input_param_.total_point_size - 1] -
             sx[index + nlp_input_param_.total_point_size]) /
            sqrt(dx * dx + dy * dy));
        nonzero_values->push_back(
            sx[index + 3 * nlp_input_param_.total_point_size - 1] *
            (sx[index + nlp_input_param_.total_point_size] -
             sx[index + nlp_input_param_.total_point_size - 1]) /
            sqrt(dx * dx + dy * dy));
        nonzero_values->push_back(1);
        nonzero_values->push_back(-1);
        nonzero_values->push_back(sqrt(dx * dx + dy * dy));
      }
      counter += n;
      index++;
    }
  }
  index = 0;
  for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
    for (uint32_t j = 0; j < nlp_input_param_.path_point_size[i]; ++j) {
      if (j == 0) {
        counter += n;
        index++;
        continue;
      }
      if (nonzero_values == nullptr) {
        uint32_t col = index;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + nlp_input_param_.total_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      } else {
        const double a = nlp_input_param_.constraint_lines[i][j].equation_a;
        const double b = nlp_input_param_.constraint_lines[i][j].equation_b;
        nonzero_values->push_back(a);
        nonzero_values->push_back(b);
      }
      counter += n;
      index++;
    }
  }
  index = 0;
  for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
    if (i == 0) {
      index = nlp_input_param_.path_point_size[i] - 1;
      continue;
    }
    if (nonzero_values == nullptr) {
      uint32_t col = index;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      col = index + 1;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
    } else {
      nonzero_values->push_back(1);
      nonzero_values->push_back(-1);
    }
    counter += n;

    if (nonzero_values == nullptr) {
      uint32_t col = index + nlp_input_param_.total_point_size;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      col = index + nlp_input_param_.total_point_size + 1;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
    } else {
      nonzero_values->push_back(1);
      nonzero_values->push_back(-1);
    }
    counter += n;

    if (nonzero_values == nullptr) {
      uint32_t col = index + 2 * nlp_input_param_.total_point_size;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      col = index + 2 * nlp_input_param_.total_point_size + 1;
      nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
      nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
    } else {
      nonzero_values->push_back(1);
      nonzero_values->push_back(-1);
    }
    counter += n;
    index += nlp_input_param_.path_point_size[i];
  }

  index = 0;
  for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
    for (uint32_t j = 0; j < nlp_input_param_.path_point_size[i]; ++j) {
      const double sin_left_angle =
          sin(nlp_input_param_.refine_road_bounds[i][j].second.heading());
      const double cos_left_angle =
          cos(nlp_input_param_.refine_road_bounds[i][j].second.heading());
      const double sin_right_angle =
          sin(nlp_input_param_.refine_road_bounds[i][j].first.heading());
      const double cos_right_angle =
          cos(nlp_input_param_.refine_road_bounds[i][j].first.heading());
      uint32_t col = 0;
      if (nonzero_values == nullptr) {
        col = index;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + nlp_input_param_.total_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + 2 * nlp_input_param_.total_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      } else {
        boost::span<const Number> sx(x, n);
        nonzero_values->push_back(sin_left_angle);
        nonzero_values->push_back(-cos_left_angle);
        nonzero_values->push_back(
            sin_left_angle *
                (-nlp_input_param_.half_collision_width *
                     cos(sx[2 * nlp_input_param_.total_point_size + index]) -
                 nlp_input_param_.front_collision_length *
                     sin(sx[2 * nlp_input_param_.total_point_size + index])) -
            cos_left_angle *
                (nlp_input_param_.front_collision_length *
                     cos(sx[2 * nlp_input_param_.total_point_size + index]) -
                 nlp_input_param_.half_collision_width *
                     sin(sx[2 * nlp_input_param_.total_point_size + index])));
      }
      counter += n;

      if (nonzero_values == nullptr) {
        col = index;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + nlp_input_param_.total_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + 2 * nlp_input_param_.total_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      } else {
        boost::span<const Number> sx(x, n);
        nonzero_values->push_back(sin_left_angle);
        nonzero_values->push_back(-cos_left_angle);
        nonzero_values->push_back(
            sin_left_angle *
                (-nlp_input_param_.half_collision_width *
                     cos(sx[2 * nlp_input_param_.total_point_size + index]) +
                 nlp_input_param_.back_collision_length *
                     sin(sx[2 * nlp_input_param_.total_point_size + index])) -
            cos_left_angle *
                (-nlp_input_param_.back_collision_length *
                     cos(sx[2 * nlp_input_param_.total_point_size + index]) -
                 nlp_input_param_.half_collision_width *
                     sin(sx[2 * nlp_input_param_.total_point_size + index])));
      }
      counter += n;

      if (nonzero_values == nullptr) {
        col = index;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + nlp_input_param_.total_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + 2 * nlp_input_param_.total_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      } else {
        boost::span<const Number> sx(x, n);
        nonzero_values->push_back(-sin_right_angle);
        nonzero_values->push_back(cos_right_angle);
        nonzero_values->push_back(
            -sin_right_angle *
                (nlp_input_param_.half_collision_width *
                     cos(sx[2 * nlp_input_param_.total_point_size + index]) -
                 nlp_input_param_.front_collision_length *
                     sin(sx[2 * nlp_input_param_.total_point_size + index])) +
            cos_right_angle *
                (nlp_input_param_.front_collision_length *
                     cos(sx[2 * nlp_input_param_.total_point_size + index]) +
                 nlp_input_param_.half_collision_width *
                     sin(sx[2 * nlp_input_param_.total_point_size + index])));
      }
      counter += n;

      if (nonzero_values == nullptr) {
        col = index;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + nlp_input_param_.total_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
        col = index + 2 * nlp_input_param_.total_point_size;
        nonzero_row->push_back(static_cast<int32_t>(counter + col) / n);
        nonzero_col->push_back(static_cast<int32_t>(counter + col) % n);
      } else {
        boost::span<const Number> sx(x, n);
        nonzero_values->push_back(-sin_right_angle);
        nonzero_values->push_back(cos_right_angle);
        nonzero_values->push_back(
            -sin_right_angle *
                (nlp_input_param_.half_collision_width *
                     cos(sx[2 * nlp_input_param_.total_point_size + index]) +
                 nlp_input_param_.back_collision_length *
                     sin(sx[2 * nlp_input_param_.total_point_size + index])) +
            cos_right_angle *
                (-nlp_input_param_.back_collision_length *
                     cos(sx[2 * nlp_input_param_.total_point_size + index]) +
                 nlp_input_param_.half_collision_width *
                     sin(sx[2 * nlp_input_param_.total_point_size + index])));
      }
      counter += n;
      index++;
    }
  }
  assert(nonzero_row->size() == nonzero_col->size());
}

void NlpMathModel::SparseHessianMatrix(
    const Index m, const Index n, const Number* x, Number obj_factor,
    const Number* lambda, std::vector<Index>* const nonzero_row,
    std::vector<Index>* const nonzero_col,
    std::vector<Number>* const nonzero_values) {
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
  uint32_t index = 0;
  for (uint32_t i : nlp_input_param_.path_point_size) {
    for (uint32_t j = 0; j < i; ++j) {
      if (j == 0) {
        index++;
        continue;
      }
      matrix_n[index][index].first = true;
      matrix_n[index + nlp_input_param_.total_point_size]
              [index + nlp_input_param_.total_point_size]
                  .first = true;
      if (nonzero_values != nullptr) {
        matrix_n[index][index].second +=
            2.0 * nlp_input_param_.bias_weight * obj_factor;
        matrix_n[index + nlp_input_param_.total_point_size]
                [index + nlp_input_param_.total_point_size]
                    .second += 2.0 * nlp_input_param_.bias_weight * obj_factor;
      }
      index++;
    }
  }

  for (uint32_t i = 0; i < nlp_input_param_.total_point_size; ++i) {
    matrix_n[i + 3 * nlp_input_param_.total_point_size]
            [i + 3 * nlp_input_param_.total_point_size]
                .first = true;
    if (nonzero_values != nullptr) {
      matrix_n[i + 3 * nlp_input_param_.total_point_size]
              [i + 3 * nlp_input_param_.total_point_size]
                  .second += 2.0 * nlp_input_param_.kappa_weight * obj_factor;
    }
  }

  index = 0;
  for (uint32_t i : nlp_input_param_.path_point_size) {
    for (uint32_t j = 0; j < i; ++j) {
      if (j == 0) {
        index++;
        continue;
      }
      matrix_n[index + 3 * nlp_input_param_.total_point_size - 1]
              [index + 3 * nlp_input_param_.total_point_size - 1]
                  .first = true;
      matrix_n[index + 3 * nlp_input_param_.total_point_size]
              [index + 3 * nlp_input_param_.total_point_size - 1]
                  .first = true;
      matrix_n[index + 3 * nlp_input_param_.total_point_size]
              [index + 3 * nlp_input_param_.total_point_size]
                  .first = true;
      if (nonzero_values != nullptr) {
        matrix_n[index + 3 * nlp_input_param_.total_point_size -
                 1][index + 3 * nlp_input_param_.total_point_size - 1]
            .second += 2.0 * nlp_input_param_.dkappa_weight * obj_factor;
        matrix_n[index + 3 * nlp_input_param_.total_point_size]
                [index + 3 * nlp_input_param_.total_point_size - 1]
                    .second +=
            -2.0 * nlp_input_param_.dkappa_weight * obj_factor;
        matrix_n[index + 3 * nlp_input_param_.total_point_size]
                [index + 3 * nlp_input_param_.total_point_size]
                    .second +=
            2.0 * nlp_input_param_.dkappa_weight * obj_factor;
      }
      index++;
    }
  }
  // constraint
  size_t counter = 0;
  index = 0;
  for (uint32_t i : nlp_input_param_.path_point_size) {
    for (uint32_t j = 0; j < i; ++j) {
      if (j == 0) {
        counter++;
        index++;
        continue;
      }
      matrix_n[index + 2 * nlp_input_param_.total_point_size - 1][index - 1]
          .first = true;
      matrix_n[index + 2 * nlp_input_param_.total_point_size - 1][index].first =
          true;
      matrix_n[index + 2 * nlp_input_param_.total_point_size - 1]
              [index + nlp_input_param_.total_point_size - 1]
                  .first = true;
      matrix_n[index + 2 * nlp_input_param_.total_point_size - 1]
              [index + nlp_input_param_.total_point_size]
                  .first = true;
      matrix_n[index + 2 * nlp_input_param_.total_point_size - 1]
              [index + 2 * nlp_input_param_.total_point_size - 1]
                  .first = true;
      if (nonzero_values != nullptr) {
        boost::span<const Number> sx(x, n);
        boost::span<const Number> slambda(lambda, m);
        matrix_n[index + 2 * nlp_input_param_.total_point_size - 1][index - 1]
            .second +=
            -cos(sx[index + 2 * nlp_input_param_.total_point_size - 1]) *
            slambda[counter];
        matrix_n[index + 2 * nlp_input_param_.total_point_size - 1][index]
            .second +=
            cos(sx[index + 2 * nlp_input_param_.total_point_size - 1]) *
            slambda[counter];
        matrix_n[index + 2 * nlp_input_param_.total_point_size - 1]
                [index + nlp_input_param_.total_point_size - 1]
                    .second +=
            -sin(sx[index + 2 * nlp_input_param_.total_point_size - 1]) *
            slambda[counter];
        matrix_n[index + 2 * nlp_input_param_.total_point_size - 1]
                [index + nlp_input_param_.total_point_size]
                    .second +=
            sin(sx[index + 2 * nlp_input_param_.total_point_size - 1]) *
            slambda[counter];
        matrix_n[index + 2 * nlp_input_param_.total_point_size - 1]
                [index + 2 * nlp_input_param_.total_point_size - 1]
                    .second +=
            (sx[index + nlp_input_param_.total_point_size] -
             sx[index + nlp_input_param_.total_point_size - 1]) *
                cos(sx[index + 2 * nlp_input_param_.total_point_size - 1]) *
                slambda[counter] -
            (sx[index] - sx[index - 1]) *
                sin(sx[index + 2 * nlp_input_param_.total_point_size - 1]) *
                slambda[counter];
        counter++;
      }
      index++;
    }
  }

  index = 0;
  for (uint32_t i : nlp_input_param_.path_point_size) {
    for (uint32_t j = 0; j < i; ++j) {
      if (j == 0) {
        counter++;
        index++;
        continue;
      }
      matrix_n[index - 1][index - 1].first = true;
      matrix_n[index][index - 1].first = true;
      matrix_n[index][index].first = true;
      matrix_n[index + nlp_input_param_.total_point_size - 1][index - 1].first =
          true;
      matrix_n[index + nlp_input_param_.total_point_size - 1][index].first =
          true;
      matrix_n[index + nlp_input_param_.total_point_size - 1]
              [index + nlp_input_param_.total_point_size - 1]
                  .first = true;
      matrix_n[index + nlp_input_param_.total_point_size][index - 1].first =
          true;
      matrix_n[index + nlp_input_param_.total_point_size][index].first = true;
      matrix_n[index + nlp_input_param_.total_point_size]
              [index + nlp_input_param_.total_point_size - 1]
                  .first = true;
      matrix_n[index + nlp_input_param_.total_point_size]
              [index + nlp_input_param_.total_point_size]
                  .first = true;
      matrix_n[index + 3 * nlp_input_param_.total_point_size - 1][index - 1]
          .first = true;
      matrix_n[index + 3 * nlp_input_param_.total_point_size - 1][index].first =
          true;
      matrix_n[index + 3 * nlp_input_param_.total_point_size - 1]
              [index + nlp_input_param_.total_point_size - 1]
                  .first = true;
      matrix_n[index + 3 * nlp_input_param_.total_point_size - 1]
              [index + nlp_input_param_.total_point_size]
                  .first = true;
      if (nonzero_values != nullptr) {
        boost::span<const Number> sx(x, n);
        boost::span<const Number> slambda(lambda, m);
        const double dx = sx[index] - sx[index - 1];
        const double dy = sx[index + nlp_input_param_.total_point_size] -
                          sx[index + nlp_input_param_.total_point_size - 1];
        matrix_n[index - 1][index - 1].second +=
            slambda[counter] *
            sx[index + 3 * nlp_input_param_.total_point_size - 1] *
            (1.0 / sqrt(dx * dx + dy * dy) -
             dx * dx / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy));
        matrix_n[index][index - 1].second +=
            slambda[counter] *
            sx[index + 3 * nlp_input_param_.total_point_size - 1] *
            (dx * dx / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy) -
             1.0 / sqrt(dx * dx + dy * dy));
        matrix_n[index][index].second +=
            slambda[counter] *
            sx[index + 3 * nlp_input_param_.total_point_size - 1] *
            (1.0 / sqrt(dx * dx + dy * dy) -
             dx * dx / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy));
        matrix_n[index + nlp_input_param_.total_point_size - 1][index - 1]
            .second += -slambda[counter] *
                       sx[index + 3 * nlp_input_param_.total_point_size - 1] *
                       dx * dy / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy);
        matrix_n[index + nlp_input_param_.total_point_size - 1][index].second +=
            slambda[counter] *
            sx[index + 3 * nlp_input_param_.total_point_size - 1] * dx * dy /
            (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy);
        matrix_n[index + nlp_input_param_.total_point_size - 1]
                [index + nlp_input_param_.total_point_size - 1]
                    .second +=
            slambda[counter] *
            sx[index + 3 * nlp_input_param_.total_point_size - 1] *
            (1.0 / sqrt(dx * dx + dy * dy) -
             dy * dy / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy));
        matrix_n[index + nlp_input_param_.total_point_size][index - 1].second +=
            slambda[counter] *
            sx[index + 3 * nlp_input_param_.total_point_size - 1] * dx * dy /
            (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy);
        matrix_n[index + nlp_input_param_.total_point_size][index].second +=
            -slambda[counter] *
            sx[index + 3 * nlp_input_param_.total_point_size - 1] * dx * dy /
            (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy);
        matrix_n[index + nlp_input_param_.total_point_size]
                [index + nlp_input_param_.total_point_size - 1]
                    .second +=
            slambda[counter] *
            sx[index + 3 * nlp_input_param_.total_point_size - 1] *
            (dy * dy / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy) -
             1.0 / sqrt(dx * dx + dy * dy));
        matrix_n[index + nlp_input_param_.total_point_size]
                [index + nlp_input_param_.total_point_size]
                    .second +=
            slambda[counter] *
            sx[index + 3 * nlp_input_param_.total_point_size - 1] *
            (1.0 / sqrt(dx * dx + dy * dy) -
             dy * dy / (dx * dx + dy * dy) / sqrt(dx * dx + dy * dy));
        matrix_n[index + 3 * nlp_input_param_.total_point_size - 1][index - 1]
            .second += -slambda[counter] * dx / sqrt(dx * dx + dy * dy);
        matrix_n[index + 3 * nlp_input_param_.total_point_size - 1][index]
            .second += slambda[counter] * dx / sqrt(dx * dx + dy * dy);
        matrix_n[index + 3 * nlp_input_param_.total_point_size - 1]
                [index + nlp_input_param_.total_point_size - 1]
                    .second += -slambda[counter] * dy / sqrt(dx * dx + dy * dy);
        matrix_n[index + 3 * nlp_input_param_.total_point_size - 1]
                [index + nlp_input_param_.total_point_size]
                    .second += slambda[counter] * dy / sqrt(dx * dx + dy * dy);
        counter++;
      }
      index++;
    }
  }

  if (nonzero_values != nullptr) {
    counter += nlp_input_param_.total_point_size +
               3 * (nlp_input_param_.path_point_size.size() - 1);
  }

  index = 0;
  for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
    for (uint32_t j = 0; j < nlp_input_param_.path_point_size[i]; ++j) {
      matrix_n[index + 2 * nlp_input_param_.total_point_size]
              [index + 2 * nlp_input_param_.total_point_size]
                  .first = true;
      if (nonzero_values != nullptr) {
        boost::span<const Number> sx(x, n);
        boost::span<const Number> slambda(lambda, m);
        const double sin_left_angle =
            sin(nlp_input_param_.refine_road_bounds[i][j].second.heading());
        const double cos_left_angle =
            cos(nlp_input_param_.refine_road_bounds[i][j].second.heading());
        const double sin_right_angle =
            sin(nlp_input_param_.refine_road_bounds[i][j].first.heading());
        const double cos_right_angle =
            cos(nlp_input_param_.refine_road_bounds[i][j].first.heading());
        matrix_n[index + 2 * nlp_input_param_.total_point_size]
                [index + 2 * nlp_input_param_.total_point_size]
                    .second +=
            (-cos_left_angle *
                 (-nlp_input_param_.half_collision_width *
                      cos(sx[index + 2 * nlp_input_param_.total_point_size]) -
                  nlp_input_param_.front_collision_length *
                      sin(sx[index + 2 * nlp_input_param_.total_point_size])) +
             sin_left_angle *
                 (-nlp_input_param_.front_collision_length *
                      cos(sx[index + 2 * nlp_input_param_.total_point_size]) +
                  nlp_input_param_.half_collision_width *
                      sin(sx[index + 2 * nlp_input_param_.total_point_size]))) *
            slambda[counter];
        counter++;

        matrix_n[index + 2 * nlp_input_param_.total_point_size]
                [index + 2 * nlp_input_param_.total_point_size]
                    .second +=
            (-cos_left_angle *
                 (-nlp_input_param_.half_collision_width *
                      cos(sx[index + 2 * nlp_input_param_.total_point_size]) +
                  nlp_input_param_.back_collision_length *
                      sin(sx[index + 2 * nlp_input_param_.total_point_size])) +
             sin_left_angle *
                 (nlp_input_param_.back_collision_length *
                      cos(sx[index + 2 * nlp_input_param_.total_point_size]) +
                  nlp_input_param_.half_collision_width *
                      sin(sx[index + 2 * nlp_input_param_.total_point_size]))) *
            slambda[counter];
        counter++;

        matrix_n[index + 2 * nlp_input_param_.total_point_size]
                [index + 2 * nlp_input_param_.total_point_size]
                    .second +=
            (cos_right_angle *
                 (nlp_input_param_.half_collision_width *
                      cos(sx[index + 2 * nlp_input_param_.total_point_size]) -
                  nlp_input_param_.front_collision_length *
                      sin(sx[index + 2 * nlp_input_param_.total_point_size])) -
             sin_right_angle *
                 (-nlp_input_param_.front_collision_length *
                      cos(sx[index + 2 * nlp_input_param_.total_point_size]) -
                  nlp_input_param_.half_collision_width *
                      sin(sx[index + 2 * nlp_input_param_.total_point_size]))) *
            slambda[counter];
        counter++;

        matrix_n[index + 2 * nlp_input_param_.total_point_size]
                [index + 2 * nlp_input_param_.total_point_size]
                    .second +=
            (cos_right_angle *
                 (nlp_input_param_.half_collision_width *
                      cos(sx[index + 2 * nlp_input_param_.total_point_size]) +
                  nlp_input_param_.back_collision_length *
                      sin(sx[index + 2 * nlp_input_param_.total_point_size])) -
             sin_right_angle *
                 (nlp_input_param_.back_collision_length *
                      cos(sx[index + 2 * nlp_input_param_.total_point_size]) -
                  nlp_input_param_.half_collision_width *
                      sin(sx[index + 2 * nlp_input_param_.total_point_size]))) *
            slambda[counter];
        counter++;
      }
      index++;
    }
  }
  if (nonzero_values != nullptr) {
    ADEBUG << "counter: " << counter << ", m: " << m;
  }

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

}  // namespace planning
}  // namespace TL
