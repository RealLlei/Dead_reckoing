/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file bezier_utils.h
 **/

#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace TL::planning {

template <int N_DEG>
class BezierUtils {
 public:
  /**
   * @brief compute the hessian matrix for non-scaled Bezier basis
   */
  static std::array<std::array<double, N_DEG + 1>, N_DEG + 1>
  GetBezierHessianMat(int derivative_degree) {
    if (N_DEG == 5 && derivative_degree == 3) {
      // jerk hessian
      return {{{720.0, -1800.0, 1200.0, 0.0, 0.0, -120.0},
               {-1800.0, 4800.0, -3600.0, 0.0, 600.0, 0.0},
               {1200.0, -3600.0, 3600.0, -1200.0, 0.0, 0.0},
               {0.0, 0.0, -1200.0, 3600.0, -3600.0, 1200.0},
               {0.0, 600.0, 0.0, -3600.0, 4800.0, -1800.0},
               {-120.0, 0.0, 0.0, 1200.0, -1800.0, 720.0}}};
    }
    return {{{0.0}}};
  }

  /**
   * @brief compute the bezier basis for non-scaled control points [c_0, c_1,
   * ....]
   */
  static std::array<double, N_DEG + 1> GetBezierBasis(int derivative_degree,
                                                      double t) {
    t = std::max(std::min(1.0, t), 0.0);
    std::array<double, N_DEG + 1> basis;
    if (N_DEG == 5) {
      if (derivative_degree == 0) {
        basis.at(0) = -pow(t - 1, 5);
        basis.at(1) = 5 * t * pow(t - 1, 4);
        basis.at(2) = -10 * pow(t, 2) * pow(t - 1, 3);
        basis.at(3) = 10 * pow(t, 3) * pow(t - 1, 2);
        basis.at(4) = -5 * pow(t, 4) * (t - 1);
        basis.at(5) = pow(t, 5);
      } else if (derivative_degree == 1) {
        basis.at(0) = -5 * pow(t - 1, 4),
        basis.at(1) = 20 * t * pow(t - 1, 3) + 5 * pow(t - 1, 4),
        basis.at(2) = -20 * t * pow(t - 1, 3) - 30 * pow(t, 2) * pow(t - 1, 2),
        basis.at(3) =
            10 * pow(t, 3) * (2 * t - 2) + 30 * pow(t, 2) * pow(t - 1, 2),
        basis.at(4) = -20 * pow(t, 3) * (t - 1) - 5 * pow(t, 4),
        basis.at(5) = 5 * pow(t, 4);
      } else if (derivative_degree == 2) {
        basis.at(0) = -20 * pow(t - 1, 3);
        basis.at(1) = 60 * t * pow(t - 1, 2) + 40 * pow(t - 1, 3);
        basis.at(2) = -120 * t * pow(t - 1, 2) - 20 * pow(t - 1, 3) -
                      30 * t * t * (2 * t - 2);
        basis.at(3) =
            60 * t * pow(t - 1, 2) + 60 * t * t * (2 * t - 2) + 20 * t * t * t;
        basis.at(4) = -60 * t * t * (t - 1) - 40 * t * t * t;
        basis.at(5) = 20 * t * t * t;
      } else if (derivative_degree == 3) {
        basis.at(0) = -60 * pow(t - 1, 2);
        basis.at(1) = 60 * t * (2 * t - 2) + 180 * pow(t - 1, 2);
        basis.at(2) = -180 * t * (2 * t - 2) - 180 * pow(t - 1, 2) - 60 * t * t;
        basis.at(3) = 180 * t * (2 * t - 2) + 60 * pow(t - 1, 2) + 180 * t * t;
        basis.at(4) = -120 * t * (t - 1) - 180 * t * t;
        basis.at(5) = 60 * t * t;
      }
    }
    return basis;
  }
};

}  // namespace TL::planning
