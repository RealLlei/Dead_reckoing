/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  nlp_math_model_cppad.h
 */

#pragma once

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "cppad/ipopt/solve.hpp"
#include "planning/open_space/nlp_path_smoother/nlp_input_param.h"

namespace TL {
namespace planning {

using CppAD::AD;
using Dvector = CppAD::vector<double>;

class FG_eval {
 public:
  using ADvector = CppAD::vector<AD<double>>;

  explicit FG_eval(NlpInputParam nlp_input_param)
      : nlp_input_param_(std::move(nlp_input_param)) {}

  void operator()(ADvector& fg, const ADvector& x) {
    assert(fg.size() == 3 * nlp_input_param_.total_point_size +
                            3 * (nlp_input_param_.path_point_size.size() - 1) +
                            4 * nlp_input_param_.total_point_size + 1);
    assert(x.size() == 4 * nlp_input_param_.total_point_size);

    size_t index = 0;
    AD<double> bias_square = 0.0;
    for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
      for (uint32_t j = 0; j < nlp_input_param_.path_point_size[i]; ++j) {
        if (j == 0) {
          index++;
          continue;
        }
        AD<double> dx =
            std::get<0>(nlp_input_param_.refine_paths[i][j]) - x[index];
        AD<double> dy = std::get<1>(nlp_input_param_.refine_paths[i][j]) -
                        x[nlp_input_param_.total_point_size + index];
        bias_square += dx * dx + dy * dy;
        index++;
      }
    }
    AD<double> kappa_square = 0.0;
    for (uint32_t i = 0; i < nlp_input_param_.total_point_size; ++i) {
      kappa_square += x[3 * nlp_input_param_.total_point_size + i] *
                      x[3 * nlp_input_param_.total_point_size + i];
    }
    index = 3 * static_cast<size_t>(nlp_input_param_.total_point_size);
    AD<double> dkappa_square = 0.0;
    for (uint32_t i : nlp_input_param_.path_point_size) {
      for (uint32_t j = 0; j < i; ++j) {
        if (j == 0) {
          index++;
          continue;
        }
        AD<double> dkappa = x[index] - x[index - 1];
        dkappa_square += dkappa * dkappa;
        index++;
      }
    }
    fg[0] = nlp_input_param_.bias_weight * bias_square +
            nlp_input_param_.kappa_weight * kappa_square +
            nlp_input_param_.dkappa_weight * dkappa_square;

    index = 0;
    uint32_t counter = 0;
    for (uint32_t i : nlp_input_param_.path_point_size) {
      for (uint32_t j = 0; j < i; ++j) {
        if (j == 0) {
          fg[++counter] = 0.0;
          index++;
          continue;
        }
        fg[++counter] = (x[index] - x[index - 1]) *
                            sin(x[2 * static_cast<size_t>(
                                          nlp_input_param_.total_point_size) +
                                  index - 1]) -
                        (x[nlp_input_param_.total_point_size + index] -
                         x[nlp_input_param_.total_point_size + index - 1]) *
                            cos(x[2 * static_cast<size_t>(
                                          nlp_input_param_.total_point_size) +
                                  index - 1]);
        index++;
      }
    }

    index = 0;
    for (uint32_t i : nlp_input_param_.path_point_size) {
      for (uint32_t j = 0; j < i; ++j) {
        if (j == 0) {
          fg[++counter] = 0.0;
          index++;
          continue;
        }
        AD<double> dx = x[index] - x[index - 1];
        AD<double> dy = x[nlp_input_param_.total_point_size + index] -
                        x[nlp_input_param_.total_point_size + index - 1];
        fg[++counter] =
            sqrt(dx * dx + dy * dy) *
                x[3 * static_cast<size_t>(nlp_input_param_.total_point_size) +
                  index - 1] -
            (x[2 * static_cast<size_t>(nlp_input_param_.total_point_size) +
               index] -
             x[2 * static_cast<size_t>(nlp_input_param_.total_point_size) +
               index - 1]);
        index++;
      }
    }

    index = 0;
    for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
      for (uint32_t j = 0; j < nlp_input_param_.path_point_size[i]; ++j) {
        if (j == 0) {
          fg[++counter] = 0.0;
          index++;
          continue;
        }
        AD<double> a = nlp_input_param_.constraint_lines[i][j].equation_a;
        AD<double> b = nlp_input_param_.constraint_lines[i][j].equation_b;
        AD<double> c = nlp_input_param_.constraint_lines[i][j].equation_c;
        fg[++counter] =
            a * x[index] + b * x[nlp_input_param_.total_point_size + index] + c;
        index++;
      }
    }

    index = 0;
    for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
      if (i == 0) {
        index = nlp_input_param_.path_point_size[i] - 1;
        continue;
      }
      fg[++counter] = x[index] - x[index + 1];
      fg[++counter] = x[index + nlp_input_param_.total_point_size] -
                      x[index + nlp_input_param_.total_point_size + 1];
      fg[++counter] =
          x[index +
            2 * static_cast<size_t>(nlp_input_param_.total_point_size)] -
          x[index + 2 * static_cast<size_t>(nlp_input_param_.total_point_size) +
            1];
      index += nlp_input_param_.path_point_size[i];
    }

    index = 0;
    for (size_t i = 0; i < nlp_input_param_.path_point_size.size(); ++i) {
      for (uint32_t j = 0; j < nlp_input_param_.path_point_size[i]; ++j) {
        const auto& left_bound_start =
            nlp_input_param_.refine_road_bounds[i][j].second.start();
        const auto& right_bound_start =
            nlp_input_param_.refine_road_bounds[i][j].first.start();
        const double left_bound_angle =
            nlp_input_param_.refine_road_bounds[i][j].second.heading();
        const double right_bound_angle =
            nlp_input_param_.refine_road_bounds[i][j].first.heading();
        const auto lf = FLUToENU(
            nlp_input_param_.front_collision_length,
            nlp_input_param_.half_collision_width, x[index],
            x[nlp_input_param_.total_point_size + index],
            x[2 * static_cast<size_t>(nlp_input_param_.total_point_size) +
              index]);
        fg[++counter] =
            sin(left_bound_angle) * (lf.first - left_bound_start.x()) -
            cos(left_bound_angle) * (lf.second - left_bound_start.y());
        const auto lb = FLUToENU(
            -nlp_input_param_.back_collision_length,
            nlp_input_param_.half_collision_width, x[index],
            x[nlp_input_param_.total_point_size + index],
            x[2 *
              static_cast<size_t>(nlp_input_param_.total_point_size + index)]);
        fg[++counter] =
            sin(left_bound_angle) * (lb.first - left_bound_start.x()) -
            cos(left_bound_angle) * (lb.second - left_bound_start.y());
        const auto rf = FLUToENU(
            nlp_input_param_.front_collision_length,
            -nlp_input_param_.half_collision_width, x[index],
            x[nlp_input_param_.total_point_size + index],
            x[2 *
              static_cast<size_t>(nlp_input_param_.total_point_size + index)]);
        fg[++counter] =
            -sin(right_bound_angle) * (rf.first - right_bound_start.x()) +
            cos(right_bound_angle) * (rf.second - right_bound_start.y());
        const auto rb = FLUToENU(
            -nlp_input_param_.back_collision_length,
            -nlp_input_param_.half_collision_width, x[index],
            x[nlp_input_param_.total_point_size + index],
            x[2 *
              static_cast<size_t>(nlp_input_param_.total_point_size + index)]);
        fg[++counter] =
            -sin(right_bound_angle) * (rb.first - right_bound_start.x()) +
            cos(right_bound_angle) * (rb.second - right_bound_start.y());
        index++;
      }
    }
    ADEBUG << "counter: " << counter
           << ", total_point_size: " << nlp_input_param_.total_point_size
           << ", path_size: " << nlp_input_param_.path_point_size.size();
  }

  static std::pair<AD<double>, AD<double>> FLUToENU(
      const double x, const double y, const AD<double>& ENU_x,
      const AD<double>& ENU_y, const AD<double>& ENU_heading) {
    auto final_x = ENU_x + cos(ENU_heading) * x - sin(ENU_heading) * y;
    auto final_y = ENU_y + sin(ENU_heading) * x + cos(ENU_heading) * y;
    return std::make_pair(final_x, final_y);
  }

 private:
  const NlpInputParam nlp_input_param_;
};

class NlpMathModelCppAD {
 public:
  NlpMathModelCppAD() = default;

  static bool IpoptSolveWithCppAD(const NlpInputParam& nlp_input_param,
                                  std::vector<double>* optimum);
};

};  // namespace planning
}  // namespace TL
