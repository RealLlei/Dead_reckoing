/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning osqp finite differfence interface smoother
 * Author: ROC
 */

#include "planning/math/discretized_points_smoothing/osqp_finite_difference_interface.h"
// #include <glob_opts.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "osqp/glob_opts.h"

// #include "common/time/clock.h"

// using TL::common::Clock;
namespace TL {
namespace planning {
using common::math::double_type::AlmostEqual;

OsqpFiniteDifferenceInterface::OsqpFiniteDifferenceInterface() = default;

void OsqpFiniteDifferenceInterface::GetNumVariables(const c_int num_points,
                                                    c_int* const n,
                                                    c_int* const num_slacks,
                                                    c_int* const total) const {
  *n = 2 * num_points;
  if (ct_weight_ <= 0 || AlmostEqual(ct_weight_, 0.0)) {
    *num_slacks = 0;
  } else {
    *num_slacks = num_points - 2;
  }

  *total = (*n) + (*num_slacks);
}

void OsqpFiniteDifferenceInterface::Cscm(
    const std::map<c_int, std::map<c_int, c_float>>& mat,
    std::vector<c_float>* const data, std::vector<c_int>* const indices,
    std::vector<c_int>* const indptr, const bool tri /* = false */) {
  // assume mat is column-wise stored
  c_int ind_p = 0;
  for (auto it = mat.cbegin(); it != mat.cend(); ++it) {
    (*indptr).emplace_back(ind_p);
    const c_int& col = it->first;
    const auto& row = it->second;
    for (auto r_it = row.cbegin(); r_it != row.cend(); ++r_it) {
      const c_int& rn = r_it->first;
      const c_float& val = r_it->second;
      if (tri && col < rn) {
        continue;
      }
      (*data).emplace_back(val);
      (*indices).emplace_back(rn);
      ++ind_p;
    }
  }

  (*indptr).emplace_back(ind_p);
}

/*
for finite difference see
https://en.wikipedia.org/wiki/Finite_difference_coefficient
*/

// first order forward finite difference with 1st order accuracy
void OsqpFiniteDifferenceInterface::
    FirstForwardDerivativeKernel1stOrderAccuracy(
        std::vector<std::vector<c_float>>* p) {
  (*p) = {{1, 0, -1, 0}, {0, 1, 0, -1}, {-1, 0, 1, 0}, {0, -1, 0, 1}};
}

// first order central finite difference with 2nd order accuracy
void OsqpFiniteDifferenceInterface::
    FirstCentralDerivativeKernel2ndOrderAccuracy(
        std::vector<std::vector<c_float>>* p) {
  (*p) = {
      {1.0 / 4.0, 0, 0, 0, -1.0 / 4.0, 0},
      {0, 1.0 / 4.0, 0, 0, 0, -1.0 / 4.0},
      {0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0},
      {-1.0 / 4.0, 0, 0, 0, 1.0 / 4.0, 0},
      {0, -1.0 / 4.0, 0, 0, 0, 1.0 / 4.0},
  };
}

// first order central finite difference with 4th order accuracy
void OsqpFiniteDifferenceInterface::
    FirstCentralDerivativeKernel4thOrderAccuracy(
        std::vector<std::vector<c_float>>* p) {
  (*p) = {
      {1.0 / 144.0, 0, -1.0 / 18.0, 0, 0, 0, 1.0 / 18.0, 0, -1.0 / 144.0, 0},
      {0, 1.0 / 144.0, 0, -1.0 / 18.0, 0, 0, 0, 1.0 / 18.0, 0, -1.0 / 144.0},
      {-1.0 / 18.0, 0, 4.0 / 9.0, 0, 0, 0, -4.0 / 9.0, 0, 1.0 / 18.0, 0},
      {0, -1.0 / 18.0, 0, 4.0 / 9.0, 0, 0, 0, -4.0 / 9.0, 0, 1.0 / 18.0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {1.0 / 18.0, 0, -4.0 / 9.0, 0, 0, 0, 4.0 / 9.0, 0, -1.0 / 18.0, 0},
      {0, 1.0 / 18.0, 0, -4.0 / 9.0, 0, 0, 0, 4.0 / 9.0, 0, -1.0 / 18.0},
      {-1.0 / 144.0, 0, 1.0 / 18.0, 0, 0, 0, -1.0 / 18.0, 0, 1.0 / 144.0, 0},
      {0, -1.0 / 144.0, 0, 1.0 / 18.0, 0, 0, 0, -1.0 / 18.0, 0, 1.0 / 144.0},
  };
}

// second order central finite difference with 2nd order accuracy
void OsqpFiniteDifferenceInterface::
    SecondCentralDerivativeKernel2ndOrderAccuracy(
        std::vector<std::vector<c_float>>* p) {
  (*p) = {
      {1, 0, -2, 0, 1, 0},  {0, 1, 0, -2, 0, 1}, {-2, 0, 4, 0, -2, 0},
      {0, -2, 0, 4, 0, -2}, {1, 0, -2, 0, 1, 0}, {0, 1, 0, -2, 0, 1},
  };
}

// second order central finite difference with 4th order accuracy
void OsqpFiniteDifferenceInterface::
    SecondCentralDerivativeKernel4thOrderAccuracy(
        std::vector<std::vector<c_float>>* p) {
  (*p) = {
      {1.0 / 144.0, 0, -1.0 / 9.0, 0, 5.0 / 24.0, 0, -1.0 / 9.0, 0, 1.0 / 144.0,
       0},
      {0, 1.0 / 144.0, 0, -1.0 / 9.0, 0, 5.0 / 24.0, 0, -1.0 / 9.0, 0,
       1.0 / 144.0},
      {-1.0 / 9.0, 0, 16.0 / 9.0, 0, -10.0 / 3.0, 0, 16.0 / 9.0, 0, -1.0 / 9.0,
       0},
      {0, -1.0 / 9.0, 0, 16.0 / 9.0, 0, -10.0 / 3.0, 0, 16.0 / 9.0, 0,
       -1.0 / 9.0},
      {5.0 / 24.0, 0, -10.0 / 3.0, 0, 25.0 / 4.0, 0, -10.0 / 3.0, 0, 5.0 / 24.0,
       0},
      {0, 5.0 / 24.0, 0, -10.0 / 3.0, 0, 25.0 / 4.0, 0, -10.0 / 3.0, 0,
       5.0 / 24.0},
      {-1.0 / 9.0, 0, 16.0 / 9.0, 0, -10.0 / 3.0, 0, 16.0 / 9.0, 0, -1.0 / 9.0,
       0},
      {0, -1.0 / 9.0, 0, 16.0 / 9.0, 0, -10.0 / 3.0, 0, 16.0 / 9.0, 0,
       -1.0 / 9.0},
      {1.0 / 144.0, 0, -1.0 / 9.0, 0, 5.0 / 24.0, 0, -1.0 / 9.0, 0, 1.0 / 144.0,
       0},
      {0, 1.0 / 144.0, 0, -1.0 / 9.0, 0, 5.0 / 24.0, 0, -1.0 / 9.0, 0,
       1.0 / 144.0},
  };
}

// third order forward finite difference with 1st order accuracy
void OsqpFiniteDifferenceInterface::
    ThirdForwardDerivativeKernel1stOrderAccuracy(
        std::vector<std::vector<c_float>>* p) {
  (*p) = {{1, 0, -3, 0, 3, 0, -1, 0}, {0, 1, 0, -3, 0, 3, 0, -1},
          {-3, 0, 9, 0, -9, 0, 3, 0}, {0, -3, 0, 9, 0, -9, 0, 3},
          {3, 0, -9, 0, 9, 0, -3, 0}, {0, 3, 0, -9, 0, 9, 0, -3},
          {-1, 0, 3, 0, -3, 0, 1, 0}, {0, -1, 0, 3, 0, -3, 0, 1}};
}

// third order forward finite difference with 2nd order accuracy
void OsqpFiniteDifferenceInterface::
    ThirdForwardDerivativeKernel2ndOrderAccuracy(
        std::vector<std::vector<c_float>>* p) {
  (*p) = {
      {1.0 / 4.0, 0, -1.0 / 2.0, 0, 0, 0, 1.0 / 2.0, 0, -1.0 / 4.0, 0},
      {0, 1.0 / 4.0, 0, -1.0 / 2.0, 0, 0, 0, 1.0 / 2.0, 0, -1.0 / 4.0},
      {-1.0 / 2.0, 0, 1, 0, 0, 0, -1, 0, 1.0 / 2.0, 0},
      {0, -1.0 / 2.0, 0, 1, 0, 0, 0, -1, 0, 1.0 / 2.0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {1.0 / 2.0, 0, -1, 0, 0, 0, 1, 0, -1.0 / 2.0, 0},
      {0, 1.0 / 2.0, 0, -1, 0, 0, 0, 1, 0, -1.0 / 2.0},
      {-1.0 / 4.0, 0, 1.0 / 2.0, 0, 0, 0, -1.0 / 2.0, 0, 1.0 / 4.0, 0},
      {0, -1.0 / 4.0, 0, 1.0 / 2.0, 0, 0, 0, -1.0 / 2.0, 0, 1.0 / 4.0},
  };
}

// fourth order central finite difference with 2nd order accuracy
void OsqpFiniteDifferenceInterface::
    FourthCentralDerivativeKernel2ndOrderAccuracy(
        std::vector<std::vector<c_float>>* p) {
  (*p) = {
      {1, 0, -4, 0, 6, 0, -4, 0, 1, 0},
      {0, 1, 0, -4, 0, 6, 0, -4, 0, 1},
      {-4, 0, 16, 0, -24, 0, 16, 0, -4, 0},
      {0, -4, 0, 16, 0, -24, 0, 16, 0, -4},
      {6, 0, -24, 0, 36, 0, -24, 0, 6, 0},
      {0, 6, 0, -24, 0, 36, 0, -24, 0, 6},
      {-4, 0, 16, 0, -24, 0, 16, 0, -4, 0},
      {0, -4, 0, 16, 0, -24, 0, 16, 0, -4},
      {1, 0, -4, 0, 6, 0, -4, 0, 1, 0},
      {0, 1, 0, -4, 0, 6, 0, -4, 0, 1},
  };
}

// lateral deviation kernel
void OsqpFiniteDifferenceInterface::LatDeviationKernel(
    const std::vector<c_float>& eval_x, const std::vector<c_float>& eval_y,
    const std::vector<c_float>& eval_heading,
    std::vector<std::vector<c_float>>* const p, std::vector<c_float>* const g) {
  p->assign(eval_heading.size() * 2, {0, 0});
  g->assign(eval_heading.size() * 2, 0);
  // for lateral deviation, x1, y1 are used
  c_int i = 0;
  for (i = 0; i < static_cast<int>(eval_heading.size()); ++i) {
    const double& theta_i = eval_heading[i];
    const double sin_i = sin(theta_i);
    const double cos_i = cos(theta_i);
    (*p)[2 * i] = {2 * sin_i * sin_i, 2 * -cos_i * sin_i};
    (*p)[2 * i + 1] = {2 * -cos_i * sin_i, 2 * cos_i * cos_i};
    std::vector<c_float> cur_theta_q = {-sin_i, cos_i};
    std::vector<c_float> pts_j = {eval_x[i], eval_y[i]};
    const c_float tmp = pts_j[0] * cur_theta_q[0] + pts_j[1] * cur_theta_q[1];
    (*g)[2 * i] = 2 * sin_i * tmp;
    (*g)[2 * i + 1] = (-2) * cos_i * tmp;
  }
}

// longitudinal deviation kernel
void OsqpFiniteDifferenceInterface::LonDeviationKernel(
    const std::vector<c_float>& eval_x, const std::vector<c_float>& eval_y,
    const std::vector<c_float>& eval_heading,
    std::vector<std::vector<c_float>>* const p, std::vector<c_float>* const g) {
  p->assign(eval_heading.size() * 2, {0, 0});
  g->assign(eval_heading.size() * 2, 0);
  // for lateral deviation, x1, y1 are used
  c_int i = 0;
  for (i = 0; i < static_cast<c_int>(eval_heading.size()); ++i) {
    const double& theta_i = eval_heading[i];
    const double sin_i = sin(theta_i);
    const double cos_i = cos(theta_i);
    (*p)[2 * i] = {2 * cos_i * cos_i, 2 * cos_i * sin_i};
    (*p)[2 * i + 1] = {2 * cos_i * sin_i, 2 * sin_i * sin_i};
    std::vector<c_float> cur_theta_q = {cos_i, sin_i};
    std::vector<c_float> pts_j = {eval_x[i], eval_y[i]};
    const c_float tmp = pts_j[0] * cur_theta_q[0] + pts_j[1] * cur_theta_q[1];
    (*g)[2 * i] = (-2) * cos_i * tmp;
    (*g)[2 * i + 1] = (-2) * sin_i * tmp;
  }
}

// euler deviation kernel
void OsqpFiniteDifferenceInterface::EulerDeviationKernel(
    const std::vector<c_float>& eval_x, const std::vector<c_float>& eval_y,
    const std::vector<c_float>& eval_heading,
    std::vector<std::vector<c_float>>* p, std::vector<c_float>* g) {
  (*p) = {{2 * 1}};
  g->assign(eval_heading.size() * 2, 0);
  c_int i = 0;
  c_int j = 0;
  for (i = 0; i < static_cast<c_int>(eval_heading.size()); ++i) {
    (*g)[2 * i] = -2 * eval_x[j];
    (*g)[2 * i + 1] = -2 * eval_y[j];
    ++j;
  }
}

// get partitial derivatives
std::vector<c_float> OsqpFiniteDifferenceInterface::GetPartitialDerivatives(
    const std::vector<c_float>& eval_x, const std::vector<c_float>& eval_y,
    const c_int ith, const c_float delta_s, const c_float c) {
  const c_float xf = eval_x[ith - 1];
  const c_float yf = eval_y[ith - 1];
  const c_float xm = eval_x[ith];
  const c_float ym = eval_y[ith];
  const c_float xl = eval_x[ith + 1];
  const c_float yl = eval_y[ith + 1];
  const c_float exp_x = xf - 2 * xm + xl;
  const c_float exp_y = yf - 2 * ym + yl;
  const c_float dxf = 2 * exp_x;
  const c_float dyf = 2 * exp_y;
  const c_float dxm = -4 * exp_x;
  const c_float dym = -4 * exp_y;
  const c_float dxl = 2 * exp_x;
  const c_float dyl = 2 * exp_y;

  const c_float l = (delta_s * delta_s * c) * (delta_s * delta_s * c);
  const c_float f =
      (dxf * xf + dyf * yf + dxm * xm + dym * ym + dxl * xl + dyl * yl);
  const c_float m = exp_x * exp_x + exp_y * exp_y;

  std::vector<c_float> pd = {dxf, dyf, dxm, dym, dxl, dyl, l + f - m};

  return pd;
}

void OsqpFiniteDifferenceInterface::SetConstraintWithSlackVariable(
    const c_int num_vars, const c_int num_slacks,
    const std::vector<c_float>& eval_x, const std::vector<c_float>& eval_y) {
  if (num_slacks <= 0) {
    return;
  }

  const c_float& curvature_weight = ct_weight_;
  const c_float& curvature = curvature_;
  const c_int n = 2 * num_vars;

  // first get delta s and curvature constraint
  c_int i = 0;
  c_float acculumated_s = 0.0;
  for (i = 1; i < static_cast<c_int>(eval_x.size()); ++i) {
    const auto& x = eval_x[i] - eval_x[i - 1];
    const auto& y = eval_y[i] - eval_y[i - 1];
    acculumated_s += std::sqrt(x * x + y * y);
  }
  const c_float delta_s = acculumated_s / (static_cast<int>(eval_x.size()) - 1);

  // set-up kernel
  for (i = n; i < n + num_slacks; ++i) {
    (pm_)[i][i] = curvature_weight;
  }

  // set-up linear part
  (g_).resize(n + num_slacks, 0);

  // set-up affine matrix and boundary
  (lb_).resize(n + num_slacks + num_slacks, 0);
  (ub_).resize(n + num_slacks + num_slacks, 0);
  for (i = n; i < n + num_slacks; ++i) {
    (am_)[i][i] = 1.0;
    (lb_)[i] = 0.0;
    (ub_)[i] = OSQP_INFTY;
  }

  c_int j = 0;
  for (i = n + num_slacks; i < n + num_slacks + num_slacks; ++i) {
    const c_int idx = j * 2;
    const auto& pd =
        GetPartitialDerivatives(eval_x, eval_y, j + 1, delta_s, curvature);

    (am_)[idx][i] = pd[0];
    (am_)[idx + 1][i] = pd[1];
    (am_)[idx + 2][i] = pd[2];
    (am_)[idx + 3][i] = pd[3];
    (am_)[idx + 4][i] = pd[4];
    (am_)[idx + 5][i] = pd[5];
    (am_)[n + j][i] = -1.0;
    (ub_)[i] = pd[6];
    (lb_)[i] = -OSQP_INFTY;

    ++j;
  }
}

void OsqpFiniteDifferenceInterface::UpdateKernelMap(const c_int k,
                                                    const c_int j,
                                                    const c_float val) {
  // for kernel we only need up triangle part
  if (j > k) {
    return;
  }
  if (!AlmostEqual(val, 0.0)) {
    (pm_)[k][j] += val;
    return;
  }
}

void OsqpFiniteDifferenceInterface::UpdateDifferenceKernel(
    const std::vector<std::vector<c_float>>& p, const c_float weight,
    const c_int n) {
  if (AlmostEqual(weight, 0.0)) {
    return;
  }

  const c_int step = static_cast<c_int>(p.size()) - 2;
  c_int i = 0;
  c_int j = 0;

  for (i = 2; i + step <= n; i += 2) {
    c_int idx = 0;
    for (j = i - 2; j < i + step; ++j) {
      const auto& row = p[idx++];
      c_int k = 0;
      c_int ith = 0;
      for (k = i - 2; k < i + step; ++k) {
        const c_float tmp = weight * row[ith++];
        UpdateKernelMap(k, j, tmp);
      }
    }
  }
  // OsqpFiniteDifferenceInterface::MatrixDebug("pm_ matrix", pm_);
}

void OsqpFiniteDifferenceInterface::UpdateDeviationKernel(
    const std::vector<std::vector<c_float>>& p, const c_float cur_weight,
    c_int n) {
  c_int i = 0;
  for (i = 0; i < n; i += 2) {
    c_float tmp = cur_weight * p[i][0];
    UpdateKernelMap(i, i, tmp);
    tmp = cur_weight * p[i][1];
    UpdateKernelMap(i + 1, i, tmp);
    tmp = cur_weight * p[i + 1][0];
    UpdateKernelMap(i, i + 1, tmp);
    tmp = cur_weight * p[i + 1][1];
    UpdateKernelMap(i + 1, i + 1, tmp);
  }
}

void OsqpFiniteDifferenceInterface::AssembleKernel(
    const c_int num_vars, const std::vector<c_float>& eval_x,
    const std::vector<c_float>& eval_y,
    const std::vector<c_float>& eval_heading) {
  const c_int n = 2 * num_vars;

  pm_.clear();

  if (!AlmostEqual(first_order_1st_, 0.0)) {
    // at least two points
    std::vector<std::vector<c_float>> p_first_order_1st;
    FirstForwardDerivativeKernel1stOrderAccuracy(&p_first_order_1st);
    UpdateDifferenceKernel(p_first_order_1st, first_order_1st_, n);
  }

  if (!AlmostEqual(first_order_2nd_, 0.0)) {
    // at least three points
    std::vector<std::vector<c_float>> p_first_order_2nd;
    FirstCentralDerivativeKernel2ndOrderAccuracy(&p_first_order_2nd);
    UpdateDifferenceKernel(p_first_order_2nd, first_order_2nd_, n);
  }

  if (!AlmostEqual(first_order_4th_, 0.0)) {
    // at least five points
    std::vector<std::vector<c_float>> p_first_order_4th;
    FirstCentralDerivativeKernel4thOrderAccuracy(&p_first_order_4th);
    UpdateDifferenceKernel(p_first_order_4th, first_order_4th_, n);
  }

  if (!AlmostEqual(second_order_2nd_, 0.0)) {
    // at least three points
    std::vector<std::vector<c_float>> p_second_order_2nd;
    SecondCentralDerivativeKernel2ndOrderAccuracy(&p_second_order_2nd);
    UpdateDifferenceKernel(p_second_order_2nd, second_order_2nd_, n);
  }

  if (!AlmostEqual(second_order_4th_, 0.0)) {
    // at least five points
    std::vector<std::vector<c_float>> p_second_order_4th;
    SecondCentralDerivativeKernel4thOrderAccuracy(&p_second_order_4th);
    UpdateDifferenceKernel(p_second_order_4th, second_order_4th_, n);
  }

  if (!AlmostEqual(third_order_1st_, 0.0)) {
    // at least four points
    std::vector<std::vector<c_float>> p_third_order_1st;
    ThirdForwardDerivativeKernel1stOrderAccuracy(&p_third_order_1st);
    UpdateDifferenceKernel(p_third_order_1st, third_order_1st_, n);
  }

  if (!AlmostEqual(third_order_2nd_, 0.0)) {
    // at least five points
    std::vector<std::vector<c_float>> p_third_order_2nd;
    ThirdForwardDerivativeKernel2ndOrderAccuracy(&p_third_order_2nd);
    UpdateDifferenceKernel(p_third_order_2nd, third_order_2nd_, n);
  }

  if (!AlmostEqual(fourth_order_2nd_, 0.0)) {
    // at least five points
    std::vector<std::vector<c_float>> p_fourth_order_2nd;
    FourthCentralDerivativeKernel2ndOrderAccuracy(&p_fourth_order_2nd);
    UpdateDifferenceKernel(p_fourth_order_2nd, fourth_order_2nd_, n);
  }

  std::vector<c_float> f;
  std::vector<std::vector<c_float>> p;
  c_int i = 0;
  (g_).assign(n, 0);
  LatDeviationKernel(eval_x, eval_y, eval_heading, &p, &f);
  UpdateDeviationKernel(p, lat_deviation_, n);
  for (i = 0; i < n; i += 2) {
    (g_)[i] += lat_deviation_ * f[i];
    (g_)[i + 1] += lat_deviation_ * f[i + 1];
  }

  LonDeviationKernel(eval_x, eval_y, eval_heading, &p, &f);
  UpdateDeviationKernel(p, lon_deviation_, n);
  for (i = 0; i < n; i += 2) {
    (g_)[i] += lon_deviation_ * f[i];
    (g_)[i + 1] += lon_deviation_ * f[i + 1];
  }

  EulerDeviationKernel(eval_x, eval_y, eval_heading, &p, &f);
  for (i = 0; i < n; i += 2) {
    c_float tmp = euler_deviation_ * p[0][0];
    UpdateKernelMap(i, i, tmp);
    UpdateKernelMap(i + 1, i + 1, tmp);
    (g_)[i] += euler_deviation_ * f[i];
    (g_)[i + 1] += euler_deviation_ * f[i + 1];
  }
}

// A matrix, lb, ub
void OsqpFiniteDifferenceInterface::GenerateConstraintMatrix(
    const c_int num_vars, const std::vector<c_float>& eval_heading,
    const std::vector<c_float>& eval_x, const std::vector<c_float>& eval_y,
    const std::vector<c_float>& lon,
    const std::vector<std::pair<c_float, c_float>>& lat) {
  std::vector<std::vector<c_float>> theta_p(num_vars,
                                            std::vector<c_float>(2, 0));
  std::vector<std::vector<c_float>> theta_q(num_vars,
                                            std::vector<c_float>(2, 0));

  c_int i = 0;
  for (i = 0; i < num_vars; ++i) {
    const auto& cur_heading = eval_heading[i];
    theta_p[i][0] = std::cos(cur_heading);
    theta_p[i][1] = std::sin(cur_heading);
    theta_q[i][0] = -theta_p[i][1];
    theta_q[i][1] = theta_p[i][0];
  }

  const c_int n = 2 * num_vars;
  (lb_).assign(n, 0);
  (ub_).assign(n, 0);
  c_int j = 0;
  (am_).clear();
  for (i = 0; i < n; i += 2) {
    const auto& cur_theta_p = theta_p[j];
    const auto& cur_theta_q = theta_q[j];
    (am_)[i][i] = cur_theta_p.front();
    (am_)[i][i + 1] = cur_theta_q.front();
    (am_)[i + 1][i] = cur_theta_p.back();
    (am_)[i + 1][i + 1] = cur_theta_q.back();
    (lb_)[i] = -lon[j] + eval_x[j] * cur_theta_p.front() +
               eval_y[j] * cur_theta_p.back();
    (ub_)[i] = lon[j] + eval_x[j] * cur_theta_p.front() +
               eval_y[j] * cur_theta_p.back();
    (lb_)[i + 1] = -lat[j].second + eval_x[j] * cur_theta_q.front() +
                   eval_y[j] * cur_theta_q.back();
    (ub_)[i + 1] = lat[j].first + eval_x[j] * cur_theta_q.front() +
                   eval_y[j] * cur_theta_q.back();

    ++j;
  }
}

void OsqpFiniteDifferenceInterface::PrepareOsqpData(
    const std::vector<c_float>& eval_x, const std::vector<c_float>& eval_y,
    const std::vector<c_float>& eval_heading, const std::vector<c_float>& lon,
    const std::vector<std::pair<c_float, c_float>>& lat, const c_int num_vars,
    const c_int num_slacks) {
  // std::lock_guard<std::mutex> lock(matrix_mutex_);

  AssembleKernel(num_vars, eval_x, eval_y, eval_heading);

  GenerateConstraintMatrix(num_vars, eval_heading, eval_x, eval_y, lon, lat);
  // if exists curvature constraints
  SetConstraintWithSlackVariable(num_vars, num_slacks, eval_x, eval_y);
}

bool OsqpFiniteDifferenceInterface::Optimize(
    const std::vector<c_float>& primal_warm_start, OSQPWorkspace** work,
    const c_int n, std::vector<c_float>* opt_xy, const c_int num_slacks,
    std::vector<c_float>* slacks) {
  osqp_warm_start_x(*work, primal_warm_start.data());
  // Solve Problem
  osqp_solve(*work);

  auto status = (*work)->info->status_val;
  if (status < 0) {
    AERROR << "failed optimization status: \t" << (*work)->info->status;
    return false;
  }

  if (status != 1 && status != 2) {
    AERROR << "failed optimization status: \t" << (*work)->info->status;
    return false;
  }

  // Extract primal results
  (*opt_xy).assign(n, 0);
  c_int i = 0;
  for (i = 0; i < n; ++i) {
    (*opt_xy)[i] = (*work)->solution->x[i];  // NOLINT
  }

  (*slacks).assign(num_slacks, 0);
  for (i = n; i < n + num_slacks; ++i) {
    (*slacks)[i - n] = (*work)->solution->x[i];  // NOLINT
  }

  return true;
}

void OsqpFiniteDifferenceInterface::SetPrimalWarmStart(
    const std::vector<c_float>& eval_x, const std::vector<c_float>& eval_y,
    const std::vector<c_float>& slacks,
    std::vector<c_float>* primal_warm_start) {
  // Set states
  primal_warm_start->assign(eval_x.size() + eval_y.size() + slacks.size(), 0);

  c_int i = 0;
  for (i = 0; i < static_cast<c_int>(eval_x.size()); ++i) {
    (*primal_warm_start)[2 * i] = eval_x[i];
    (*primal_warm_start)[2 * i + 1] = eval_y[i];
  }

  const c_int n = static_cast<int>(eval_x.size() + eval_y.size());
  for (i = 0; i < static_cast<c_int>(slacks.size()); ++i) {
    (*primal_warm_start)[i + n] = slacks[i];
  }
}

void OsqpFiniteDifferenceInterface::GetOptXY(const std::vector<c_float>& opt_xy,
                                             std::vector<c_float>* opt_x,
                                             std::vector<c_float>* opt_y) {
  c_int i = 0;
  c_int j = 0;
  (*opt_x).assign(opt_xy.size() / 2, 0);
  (*opt_y).assign(opt_xy.size() / 2, 0);
  for (i = 0; i < static_cast<c_int>(opt_xy.size()); i += 2) {
    (*opt_x)[j] = opt_xy[i];
    (*opt_y)[j] = opt_xy[i + 1];
    ++j;
  }
}

void OsqpFiniteDifferenceInterface::GetSolution(
    const std::vector<c_float>& opt_xy, std::vector<c_float>* solution) {
  (*solution).assign(opt_xy.size(), 0);
  c_int i = 0;
  for (i = 0; i < static_cast<c_int>(opt_xy.size()); ++i) {
    (*solution)[i] = opt_xy[i];
  }
}

void OsqpFiniteDifferenceInterface::FreeOsqp(OSQPData* data,
                                             OSQPWorkspace* work,
                                             OSQPSettings* settings) {
  // std::lock_guard<std::mutex> lock(osqp_mutex_);
  // NOLINTBEGIN
  if (work != nullptr) {
    osqp_cleanup(work);
    work = nullptr;
  }
  if (data->A != nullptr) {
    c_free(data->A);
    data->A = nullptr;
  }
  if (data->P != nullptr) {
    c_free(data->P);
    data->P = nullptr;
  }
  if (data != nullptr) {
    c_free(data);
    data = nullptr;
  }
  if (settings != nullptr) {
    c_free(settings);
    settings = nullptr;
  }
  // NOLINTEND
}

int OsqpFiniteDifferenceInterface::Smooth(
    const std::vector<c_float>& eval_x, const std::vector<c_float>& eval_y,
    const std::vector<c_float>& eval_heading, const std::vector<c_float>& lon,
    const std::vector<std::pair<c_float, c_float>>& lat,
    std::vector<c_float>* const solution) {
  // PERF_FUNCTION();
  // set-up data
  const c_int num_vars = static_cast<int>(eval_x.size());
  c_int n = 0;
  c_int num_slacks = 0;
  c_int total = 0;
  GetNumVariables(num_vars, &n, &num_slacks, &total);

  std::vector<c_float> slacks;
  slacks.assign(num_slacks, 0);
  std::vector<c_float> primal_warm_start;
  SetPrimalWarmStart(eval_x, eval_y, slacks, &primal_warm_start);
  PrepareOsqpData(eval_x, eval_y, eval_heading, lon, lat, num_vars, num_slacks);

  // NOLINTBEGIN
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  ACHECK(data);

  std::vector<c_float> p_data;
  std::vector<c_int> p_indices;
  std::vector<c_int> p_indptr;
  std::vector<c_float> a_data;
  std::vector<c_int> a_indices;
  std::vector<c_int> a_indptr;
  Cscm(pm_, &p_data, &p_indices, &p_indptr);
  Cscm(am_, &a_data, &a_indices, &a_indptr);

  (data)->n = 2 * num_vars + num_slacks;
  (data)->m = 2 * num_vars + 2 * num_slacks;
  (data)->P =
      csc_matrix((data)->n, (data)->n, static_cast<c_int>(p_data.size()),
                 p_data.data(), p_indices.data(), p_indptr.data());
  (data)->A =
      csc_matrix((data)->m, (data)->n, static_cast<c_int>(a_data.size()),
                 a_data.data(), a_indices.data(), a_indptr.data());
  (data)->q = (g_).data();
  (data)->l = (lb_).data();
  (data)->u = (ub_).data();

  // set-up osqp settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  // NOLINTEND

  ACHECK(settings);
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

  OSQPWorkspace* work(nullptr);
  c_int exitflag = osqp_setup(&work, data, settings);
  ADEBUG << "osqp setup status: " << exitflag;

  // get intial solution
  std::vector<c_float> opt_xy;
  if (work == nullptr) {
    AERROR << "planning failed: osqp_work is nullptr!";
    return -1;
  }
  bool initial_res =
      Optimize(primal_warm_start, &work, n, &opt_xy, num_slacks, &slacks);

  if (!initial_res) {
    AERROR << "Intital Optimization Failed: " << work->info->status;
    FreeOsqp(data, work, settings);
    return -1;
  }
  GetSolution(opt_xy, solution);
  // if curvature constraint exists, we need to use sequential qp
  int sequential_qp = 0;
  if (num_slacks > 0) {
    AERROR << "Enter into sqp";
    int pen_itr = 0;
    const int& sqp_pen_max_iter = sqp_pen_max_iter_;
    const int& sqp_sub_max_iter = sqp_sub_max_iter_;
    const double& sqp_ftol = sqp_ftol_;
    const double& original_slack_penalty = ct_weight_;
    double last_fvalue = work->info->obj_val;

    while (pen_itr < sqp_pen_max_iter) {
      int sub_itr = 0;
      bool fconverged = false;
      while (sub_itr < sqp_sub_max_iter) {
        std::vector<c_float> opt_x;
        std::vector<c_float> opt_y;
        GetOptXY(opt_xy, &opt_x, &opt_y);
        SetPrimalWarmStart(opt_x, opt_y, slacks, &primal_warm_start);
        SetConstraintWithSlackVariable(num_vars, num_slacks, opt_x, opt_y);

        std::vector<c_float> p_data;
        std::vector<c_int> p_indices;
        std::vector<c_int> p_indptr;
        Cscm(pm_, &p_data, &p_indices, &p_indptr);
        osqp_update_P(work, p_data.data(), OSQP_NULL,
                      static_cast<c_int>(p_data.size()));
        std::vector<c_float> a_data;
        std::vector<c_int> a_indices;
        std::vector<c_int> a_indptr;
        Cscm(am_, &a_data, &a_indices, &a_indptr);
        osqp_update_A(work, a_data.data(), OSQP_NULL,
                      static_cast<c_int>(a_data.size()));

        osqp_update_bounds(work, lb_.data(), ub_.data());

        bool iterative_solve_res =
            Optimize(primal_warm_start, &work, n, &opt_xy, num_slacks, &slacks);

        if (!iterative_solve_res) {
          ADEBUG << "iteration at " << pen_itr << ", solving fails at "
                 << sub_itr << "with info: " << work->info->status;
          ct_weight_ = original_slack_penalty;
          FreeOsqp(data, work, settings);
          return sequential_qp;
        }

        const double& cur_fvalue = work->info->obj_val;
        double ftol = std::abs((last_fvalue - cur_fvalue) / last_fvalue);
        if (ftol < sqp_ftol) {
          fconverged = true;
          break;
        }

        last_fvalue = cur_fvalue;
        ++sub_itr;
      }

      if (!fconverged) {
        ADEBUG << "max iteration reached at " << pen_itr;
        ct_weight_ = original_slack_penalty;
        FreeOsqp(data, work, settings);
        return sequential_qp;
      }

      GetSolution(opt_xy, solution);
      ++sequential_qp;
      ct_weight_ *= 10;
      ++pen_itr;
    }
  }

  FreeOsqp(data, work, settings);
  return 0;
}

void OsqpFiniteDifferenceInterface::MatrixDebug(
    const std::string& name,
    const std::map<c_int, std::map<c_int, c_float>>& matrix) {
  ADEBUG << name;
  for (const auto& row : matrix) {
    std::string str;
    for (const auto& val : row.second) {
      std::string str1;
      str.append(" ");
      str1 = std::to_string(val.second);
      str.append(str1.substr(0, 8));
    }
    ADEBUG << str;
  }
}

void OsqpFiniteDifferenceInterface::MatrixDebug(
    const std::string& name, const std::vector<std::vector<c_float>>& matrix) {
  ADEBUG << name;
  for (const auto& row : matrix) {
    std::string str;
    for (double val : row) {
      std::string str1;
      str.append(" ");
      str1 = std::to_string(val);
      str.append(str1.substr(0, 8));
    }
    ADEBUG << str;
  }
}
}  // namespace planning
}  // namespace TL
