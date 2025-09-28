/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file yield_obstacle_cache.cc
 **/
#include "planning/tasks/optimizers/speed_data_optimizer/caches/yield_obstacle_cache.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>

#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/math_utils.h"
#include "common/util/macros.h"
#include "planning/common/obstacle.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/util/common.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {
namespace speed_evaluator {

using common::math::double_type::AlmostEqual;
using common::math::double_type::DefinitelyGreater;
using common::math::double_type::DefinitelyGreaterEqual;
using common::math::double_type::DefinitelyLess;

YieldObstacleCache::YieldObstacleCache(
    const SpeedCacheConfig& config,
    const ReferenceLineInfo& reference_line_info, const Obstacle* obstacle,
    const double time_unit, const int st_graph_time_count)
    : obstacle_(obstacle),
      time_unit_(time_unit),
      st_graph_time_count_(st_graph_time_count) {
  UNUSED(config);
  obstacle_infos_.assign(st_graph_time_count, ObstacleInfo());
  if (obstacle_ == nullptr) {
    return;
  }
  ADEBUG << "st_graph_time_count:" << st_graph_time_count_
         << ", id:" << obstacle_->Id();

  CacheObstacleInfos(reference_line_info);

  is_front_ = obstacle->PerceptionSLBoundary().start_s() >
              reference_line_info.AdcSlBoundary().end_s();
}

void YieldObstacleCache::CacheObstacleInfos(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_data = reference_line_info.path_data();
  const auto& discretized_path = path_data.discretized_path();
  const auto& slt_boundary = obstacle_->GetPathSLTBoundary();
  const auto& st_boundary = slt_boundary.GetSTBoundary();
  const auto& lt_boundary = slt_boundary.GetLTBoundary();

  // calculate obstacle speed in st_boundary_time_count_
  auto s_upper = std::numeric_limits<double>::lowest();
  auto s_lower = std::numeric_limits<double>::max();
  auto s_range = std::numeric_limits<double>::infinity();
  auto l_upper = std::numeric_limits<double>::lowest();
  auto l_lower = std::numeric_limits<double>::max();
  auto ds = std::numeric_limits<double>::infinity();
  auto dl = std::numeric_limits<double>::infinity();

  const auto path_length = discretized_path.Length();

  // add extra overtake buffer for front obstacle
  const auto need_overtake_buffer =
      obstacle_->PerceptionSLBoundary().end_s() >
      reference_line_info.AdcSlBoundary().start_s();

  auto time_index = std::numeric_limits<int>::lowest();
  auto time_count = static_cast<int>(round(st_boundary.max_t() / time_unit_));
  time_count = std::min(time_count + 1, st_graph_time_count_);

  const auto& st_lower_points = st_boundary.lower_points();
  const auto& st_upper_points = st_boundary.upper_points();
  const auto& lt_lower_points = lt_boundary.lower_points();
  const auto& lt_upper_points = lt_boundary.upper_points();
  const auto& ds_points = slt_boundary.GetDsPoints();
  const auto& dl_points = slt_boundary.GetDlPoints();

  int next_point_index = 0;

  for (int i = 0; i < time_count; ++i) {
    const auto t = i * time_unit_;
    if (t < st_boundary.min_t() || t > st_boundary.max_t()) {
      continue;
    }

    auto& obstacle_info = obstacle_infos_.at(i);

    // calculate s_lower / s_upper / l_lower / l_upper
    while (next_point_index + 1 < st_lower_points.size() &&
           st_lower_points.at(next_point_index).t() < t) {
      ++next_point_index;
    }
    const auto& prev_point_index =
        next_point_index == 0 ? 0 : next_point_index - 1;

    const auto& prev_st_lower_point = st_lower_points.at(prev_point_index);
    const auto& next_st_lower_point = st_lower_points.at(next_point_index);

    const auto dt = (next_st_lower_point.t() - prev_st_lower_point.t());
    const auto ratio = common::math::double_type::IsZero(dt)
                           ? 0.0
                           : (t - prev_st_lower_point.t()) / dt;
    s_lower =
        (1 - ratio) * prev_st_lower_point.s() + ratio * next_st_lower_point.s();
    s_upper = (1 - ratio) * st_upper_points.at(prev_point_index).s() +
              ratio * st_upper_points.at(next_point_index).s();
    l_lower = (1 - ratio) * lt_lower_points.at(prev_point_index).s() +
              ratio * lt_lower_points.at(next_point_index).s();
    l_upper = (1 - ratio) * lt_upper_points.at(prev_point_index).s() +
              ratio * lt_upper_points.at(next_point_index).s();
    ds = (1 - ratio) * ds_points.at(prev_point_index).s() +
         ratio * ds_points.at(next_point_index).s();
    dl = (1 - ratio) * dl_points.at(prev_point_index).s() +
         ratio * dl_points.at(next_point_index).s();

    // update min_t_, max_t_
    if (time_index < 0) {
      min_t_ = t;
    }
    max_t_ = t;
    time_index = i;

    ADEBUG << "i:" << i << ", t:" << t << ", ds:" << ds;

    // calculate s_lower， s_upper and s_range
    if (std::isinf(s_range) ||
        (DefinitelyGreaterEqual(ds, 0.0) &&
         DefinitelyLess(s_upper, path_length)) ||
        (DefinitelyLess(ds, 0.0) && DefinitelyGreater(s_lower, 0.0))) {
      s_range = s_upper - s_lower;
    }
    if (DefinitelyGreater(ds, 0.0)) {
      s_upper = fmax(s_lower + s_range, s_upper);
    } else {
      s_lower = fmin(s_upper - s_range, s_lower);
    }
    obstacle_info.s_upper = s_upper;
    obstacle_info.s_lower = s_lower;
    obstacle_info.l_upper = l_upper;
    obstacle_info.l_lower = l_lower;
    obstacle_info.ds = ds;
    obstacle_info.dl = dl;
    if (need_overtake_buffer) {
#ifdef FOR_BAIDU_SIMULATION
      obstacle_info.s_upper += fmax(2.0, 0.4 * pow(t, 2));
#else
      obstacle_info.s_upper += fmax(2.0, 0.1 * pow(t, 2));
#endif
    }
  }

  // determine whether obstacle is still on path_data extended line after
  // st_boundary_time_count_
  is_cut_out_ = true;
  const auto& trajectory_points = obstacle_->Trajectory().trajectory_point();
  const auto trajectory_time =
      trajectory_points.empty() ? 0.0
                                : trajectory_points.rbegin()->relative_time();
  if (std::isinf(ds)) {
    is_cut_out_ = true;
  } else if (AlmostEqual(st_boundary.max_t(), trajectory_time)) {
    is_cut_out_ = false;
  } else if (DefinitelyGreaterEqual(obstacle_infos_.at(time_index).ds, 0.0)) {
    is_cut_out_ = DefinitelyLess(st_boundary.upper_points().back().s(),
                                 path_data.discretized_path().Length());
  } else {
    is_cut_out_ = DefinitelyGreater(st_boundary.lower_points().back().s(), 0.0);
  }

  // calculate obstacle speed in st_graph_time_count_
  if (!is_cut_out_) {
    for (int i = time_index + 1; i < st_graph_time_count_; ++i) {
      auto& obstacle_info = obstacle_infos_.at(i);
      obstacle_info.ds = ds;
      const auto s = ds * (i - time_index) * time_unit_;
      obstacle_info.s_upper = s_upper + s;
      obstacle_info.s_lower = s_lower + s;
      const auto l = dl * (i - time_index) * time_unit_;
      obstacle_info.l_upper = l_upper + l;
      obstacle_info.l_lower = l_lower + l;
    }
    max_t_ = (st_graph_time_count_ - 1) * time_unit_;
  }
  min_t_ -= 1e-3;
  max_t_ += 1e-3;

  const auto s_buffer = 1.0;
  const auto l_buffer = 0.5;
  for (auto& obstacle_info : obstacle_infos_) {
    obstacle_info.s_lower_with_buffer = obstacle_info.s_lower - s_buffer;
    obstacle_info.s_upper_with_buffer = obstacle_info.s_upper + s_buffer;
    obstacle_info.l_lower_with_buffer = obstacle_info.l_lower - l_buffer;
    obstacle_info.l_upper_with_buffer = obstacle_info.l_upper + l_buffer;
  }
}

}  // namespace speed_evaluator
}  // namespace planning
}  // namespace TL
