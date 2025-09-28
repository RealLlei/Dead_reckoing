/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  nlp_math_model_cppad.cc
 */

#include "planning/open_space/nlp_path_smoother/nlp_math_model_cppad.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace TL {
namespace planning {

namespace {
constexpr double kMaxValue = 1e20;
}  // namespace

bool NlpMathModelCppAD::IpoptSolveWithCppAD(
    const NlpInputParam& nlp_input_param, std::vector<double>* const optimum) {
  DCHECK(nullptr != optimum);
  optimum->clear();

  const size_t nx = 4 * static_cast<size_t>(nlp_input_param.total_point_size);
  const size_t ng =
      3 * static_cast<size_t>(nlp_input_param.total_point_size) +
      3 * static_cast<size_t>(nlp_input_param.path_point_size.size() - 1) +
      4 * static_cast<size_t>(nlp_input_param.total_point_size);

  Dvector xi(nx);
  uint32_t index = 0;
  for (size_t i = 0; i < nlp_input_param.path_point_size.size(); ++i) {
    for (uint32_t j = 0; j < nlp_input_param.path_point_size[i]; ++j) {
      xi[index] = std::get<0>(nlp_input_param.refine_paths[i][j]);
      xi[nlp_input_param.total_point_size + index] =
          std::get<1>(nlp_input_param.refine_paths[i][j]);
      xi[2 * nlp_input_param.total_point_size + index] =
          std::get<2>(nlp_input_param.refine_paths[i][j]);
      xi[3 * nlp_input_param.total_point_size + index] =
          std::get<3>(nlp_input_param.refine_paths[i][j]);
      index++;
    }
  }

  Dvector xl(nx);
  Dvector xu(nx);
  index = 0;
  for (size_t i = 0; i < nlp_input_param.path_point_size.size(); ++i) {
    for (uint32_t j = 0; j < nlp_input_param.path_point_size[i]; ++j) {
      xl[index] = nlp_input_param.constraint_lines[i][j].min_x;
      xu[index] = nlp_input_param.constraint_lines[i][j].max_x;

      xl[index + nlp_input_param.total_point_size] =
          nlp_input_param.constraint_lines[i][j].min_y;
      xu[index + nlp_input_param.total_point_size] =
          nlp_input_param.constraint_lines[i][j].max_y;

      xl[index + 2 * nlp_input_param.total_point_size] = -kMaxValue;
      xu[index + 2 * nlp_input_param.total_point_size] = kMaxValue;

      xl[index + 3 * nlp_input_param.total_point_size] =
          -nlp_input_param.max_kappa;
      xu[index + 3 * nlp_input_param.total_point_size] =
          nlp_input_param.max_kappa;
      index++;
    }
  }
  xl[0] = std::get<0>(nlp_input_param.refine_paths.front().front());
  xu[0] = std::get<0>(nlp_input_param.refine_paths.front().front());
  xl[nlp_input_param.total_point_size] =
      std::get<1>(nlp_input_param.refine_paths.front().front());
  xu[nlp_input_param.total_point_size] =
      std::get<1>(nlp_input_param.refine_paths.front().front());
  xl[2 * nlp_input_param.total_point_size] =
      std::get<2>(nlp_input_param.refine_paths.front().front());
  xu[2 * nlp_input_param.total_point_size] =
      std::get<2>(nlp_input_param.refine_paths.front().front());
  if (nlp_input_param.enable_fix_start_kappa) {
    xl[3 * nlp_input_param.total_point_size] =
        std::get<3>(nlp_input_param.refine_paths.front().front());
    xu[3 * nlp_input_param.total_point_size] =
        std::get<3>(nlp_input_param.refine_paths.front().front());
  }

  if (!nlp_input_param.enable_dest_lat_region_constrain) {
    xl[nlp_input_param.total_point_size - 1] =
        std::get<0>(nlp_input_param.refine_paths.back().back());
    xu[nlp_input_param.total_point_size - 1] =
        std::get<0>(nlp_input_param.refine_paths.back().back());
    xl[2 * nlp_input_param.total_point_size - 1] =
        std::get<1>(nlp_input_param.refine_paths.back().back());
    xu[2 * nlp_input_param.total_point_size - 1] =
        std::get<1>(nlp_input_param.refine_paths.back().back());
  }
  xl[3 * nlp_input_param.total_point_size - 1] =
      std::get<2>(nlp_input_param.refine_paths.back().back());
  xu[3 * nlp_input_param.total_point_size - 1] =
      std::get<2>(nlp_input_param.refine_paths.back().back());
  //   xl[4 * nlp_input_param.total_point_size - 1] =
  //       std::get<3>(nlp_input_param.refine_paths.back().back());
  //   xu[4 * nlp_input_param.total_point_size - 1] =
  //       std::get<3>(nlp_input_param.refine_paths.back().back());

  Dvector gl(ng);
  Dvector gu(ng);
  for (uint32_t i = 0; i < 3 * (nlp_input_param.total_point_size +
                                nlp_input_param.path_point_size.size() - 1);
       i++) {
    gl[i] = 0.0;
    gu[i] = 0.0;
  }
  for (uint32_t i = 3 * (nlp_input_param.total_point_size +
                         nlp_input_param.path_point_size.size() - 1);
       i < ng; i++) {
    gl[i] = nlp_input_param.safety_buffer;
    gu[i] = kMaxValue;
  }

  std::string options;
  options += "Integer print_level " +
             std::to_string(nlp_input_param.config.print_level()) + "\n";
  options += "Numeric max_cpu_time " +
             std::to_string(nlp_input_param.config.max_cpu_time()) + "\n";
  options += "Integer max_iter " +
             std::to_string(nlp_input_param.config.max_iter_num()) + "\n";
  options +=
      "Numeric tol " + std::to_string(nlp_input_param.config.tol()) + "\n";
  options += "Numeric acceptable_tol " +
             std::to_string(nlp_input_param.config.acceptable_tol()) + "\n";
  options += "String  sb  yes\n";
  options += "Sparse  true  forward\n";

  CppAD::ipopt::solve_result<Dvector> solution;
  FG_eval fg_eval(nlp_input_param);
  CppAD::ipopt::solve<Dvector, FG_eval>(options, xi, xl, xu, gl, gu, fg_eval,
                                        solution);
  const bool success =
      solution.status == CppAD::ipopt::solve_result<Dvector>::success ||
      solution.status ==
          CppAD::ipopt::solve_result<Dvector>::stop_at_acceptable_point;
  if (success) {
    optimum->reserve(nx);
    for (size_t i = 0; i < nx; ++i) {
      optimum->push_back(solution.x[i]);
    }
  }
  ADEBUG << "success: " << success << ", solution.status: " << solution.status;

  return success;
}

}  // namespace planning
}  // namespace TL
