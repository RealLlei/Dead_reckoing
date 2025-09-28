/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  nlp_input_param.h
 */

#pragma once

#include <cstdint>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"
#include "planning/proto/planner_open_space_config.pb.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

struct NlpInputParam {
  planning::NlpPathSmootherConfig config;
  uint32_t total_point_size = 0;
  std::vector<uint32_t> path_point_size;
  bool enable_fix_start_kappa = false;
  bool enable_dest_lat_region_constrain = false;

  double max_kappa = 0.0;
  double front_collision_length = 0.0;
  double back_collision_length = 0.0;
  double half_collision_width = 0.0;
  double safety_buffer = 0.0;

  double bias_weight = 0.0;
  double kappa_weight = 0.0;
  double dkappa_weight = 0.0;

  common::math::Vec2d origin_point;
  std::vector<std::vector<std::tuple<double, double, double, double>>>
      refine_paths;

  struct ConstraintLine {
    double equation_a = 0.0;
    double equation_b = 0.0;
    double equation_c = 0.0;
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::min();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::min();
  };

  std::vector<std::vector<ConstraintLine>> constraint_lines;
  std::vector<std::vector<
      std::pair<common::math::LineSegment2d, common::math::LineSegment2d>>>
      refine_road_bounds;
};

};  // namespace planning
}  // namespace TL
