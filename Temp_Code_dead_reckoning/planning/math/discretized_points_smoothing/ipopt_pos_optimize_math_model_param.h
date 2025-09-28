/*
 * Copyright (c) TL Technologies Co., Ltd. 2023. All rights reserved.
 * Description:  ipopt_pos_optimize_math_model_param.h
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

#include "common/math/vec2d.h"
#include "proto/common/pnc_point.pb.h"

namespace TL::planning {

struct IpoptPosOptimizeMathModelParam {
  int32_t path_point_size = 0;
  bool enable_fix_start_kappa = false;
  bool enable_fix_end_state = false;
  bool is_forward_path = true;

  double max_kappa = 0.0;
  double bias_weight = 0.0;
  double kappa_weight = 0.0;
  double dkappa_weight = 0.0;

  common::math::Vec2d origin_point;
  std::vector<std::tuple<double, double, double, double>> refine_path_point;

  bool is_collision_free = true;
  double min_distance_threshold = 0.0;
  int32_t front_delta_index = 0;
  std::pair<std::vector<common::math::Vec2d>, std::vector<common::math::Vec2d>>
      xy_lower_upper_bounds;

  struct ConstraintLine {
    double equation_a = 0.0;
    double equation_b = 0.0;
    double equation_c = 0.0;
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::min();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::min();
  };

  std::vector<ConstraintLine> constraint_line;
};

}  // namespace TL::planning
