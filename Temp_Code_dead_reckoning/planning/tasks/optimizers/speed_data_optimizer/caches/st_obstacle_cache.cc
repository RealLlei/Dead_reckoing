/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_cost.h
 **/
#include "planning/tasks/optimizers/speed_data_optimizer/caches/st_obstacle_cache.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>

#include "common/file/log.h"
#include "common/interpolation/interpolation_2d.h"
#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "planning/common/obstacle.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/util/common.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreater;
using common::math::double_type::DefinitelyLess;

STObstacleCache::STObstacleCache(const SpeedCacheConfig& config,
                                 const ReferenceLineInfo& reference_line_info,
                                 const Obstacle* obstacle,
                                 const double time_unit,
                                 const int st_graph_time_count)
    : obstacle_(obstacle),
      time_unit_(time_unit),
      st_graph_time_count_(st_graph_time_count) {
  obstacle_infos_.assign(st_graph_time_count, ObstacleInfo());
  if (obstacle_ == nullptr || obstacle->path_st_boundary().IsEmpty()) {
    return;
  }
  ADEBUG << "st_graph_time_count:" << st_graph_time_count_
         << ", id:" << obstacle->Id();

  CacheObstacleInfos(reference_line_info, reference_line_info.path_data());

  min_stop_distacne_ = obstacle_->NormalStopDistance();

  collision_check_distance_buffer_ =
      obstacle->IsCrossReferenceLine()
          ? config.cross_collision_check_distance_buffer()
          : config.non_cross_collision_check_distance_buffer();

  is_front_ = obstacle->PerceptionSLBoundary().end_s() >
              reference_line_info.AdcSlBoundary().end_s();

  id_ = obstacle->Id();
  perception_id_ = obstacle_->PerceptionId();
}

void STObstacleCache::CacheObstacleInfos(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  const auto& discretized_path = path_data.discretized_path();
  const auto& boundary = obstacle_->path_st_boundary();
  // calculate obstacle speed in st_boundary_time_count_
  // auto s_upper = std::numeric_limits<double>::lowest();
  // auto s_lower = std::numeric_limits<double>::max();
  auto trajectory_point_x = 0.0;
  auto trajectory_point_y = 0.0;
  auto trajectory_point_theta = 0.0;
  auto trajectory_point_v = 0.0;
  auto trajectory_point_a = 0.0;
  auto v = std::numeric_limits<double>::infinity();
  const auto space_resolution =
      fmax(path_data.frenet_frame_path().GetSpaceResolution(), 0.00001);

  const auto& st_lower_points = boundary.lower_points();
  const auto& st_upper_points = boundary.upper_points();
  std::size_t next_boundary_point_index = 0;

  const auto& trajectory_points = obstacle_->Trajectory().trajectory_point();
  int next_trajectory_point_index = 0;

  auto time_index = std::numeric_limits<int>::lowest();
  auto time_count = static_cast<int>(
      round(obstacle_->path_st_boundary().max_t() / time_unit_));
  time_count = std::min(time_count + 1, st_graph_time_count_);
  for (int i = 0; i < time_count; ++i) {
    const auto t = i * time_unit_;
    if (t < boundary.min_t() || t > boundary.max_t()) {
      continue;
    }

    auto& obstacle_info = obstacle_infos_.at(i);

    // calculate s_lower / s_upper / l_lower / l_upper
    while (next_boundary_point_index + 1 < st_lower_points.size() &&
           st_lower_points.at(next_boundary_point_index).t() < t) {
      ++next_boundary_point_index;
    }
    const auto prev_boundary_point_index =
        next_boundary_point_index == 0 ? 0 : next_boundary_point_index - 1;

    const auto& prev_st_lower_point =
        st_lower_points.at(prev_boundary_point_index);
    const auto& next_st_lower_point =
        st_lower_points.at(next_boundary_point_index);

    auto dt = next_st_lower_point.t() - prev_st_lower_point.t();
    const auto boundary_point_ratio = common::math::double_type::IsZero(dt)
                                          ? 0.0
                                          : (t - prev_st_lower_point.t()) / dt;
    obstacle_info.s_lower =
        (1 - boundary_point_ratio) * prev_st_lower_point.s() +
        boundary_point_ratio * next_st_lower_point.s();
    obstacle_info.s_upper =
        (1 - boundary_point_ratio) *
            st_upper_points.at(prev_boundary_point_index).s() +
        boundary_point_ratio *
            st_upper_points.at(next_boundary_point_index).s();

    // update min_t_, max_t_
    if (time_index < 0) {
      min_t_ = t;
    }
    max_t_ = t;
    time_index = i;

    // find trajectory_point projection on path data
    while (next_trajectory_point_index + 1 < trajectory_points.size() &&
           trajectory_points.at(next_trajectory_point_index).relative_time() <
               t) {
      ++next_trajectory_point_index;
    }
    const auto prev_trajectory_point_index =
        next_trajectory_point_index == 0 ? 0 : next_trajectory_point_index - 1;

    const auto& prev_trajectory_point =
        trajectory_points.at(prev_trajectory_point_index);
    const auto& next_trajectory_point =
        trajectory_points.at(next_trajectory_point_index);

    dt = next_trajectory_point.relative_time() -
         prev_trajectory_point.relative_time();
    const auto ratio =
        (prev_trajectory_point_index == next_trajectory_point_index)
            ? 0.0
            : (t - prev_trajectory_point.relative_time()) / dt;
    trajectory_point_x = (1 - ratio) * prev_trajectory_point.path_point().x() +
                         ratio * next_trajectory_point.path_point().x();
    trajectory_point_y = (1 - ratio) * prev_trajectory_point.path_point().y() +
                         ratio * next_trajectory_point.path_point().y();
    trajectory_point_theta =
        prev_trajectory_point.path_point().theta() +
        ratio * TL::common::math ::NormalizeAngle(
                    next_trajectory_point.path_point().theta() -
                    prev_trajectory_point.path_point().theta());
    trajectory_point_v = (1 - ratio) * prev_trajectory_point.v() +
                         ratio * next_trajectory_point.v();
    trajectory_point_a = (1 - ratio) * prev_trajectory_point.a() +
                         ratio * next_trajectory_point.a();

    const auto before_index = common::math::Clamp(
        static_cast<int>(std::floor(obstacle_info.s_lower / space_resolution)),
        0, static_cast<int>(discretized_path.size() - 1));
    const auto after_index = common::math::Clamp(
        static_cast<int>(std::ceil(obstacle_info.s_upper / space_resolution)),
        0, static_cast<int>(discretized_path.size() - 1));
    auto min_dis_index = before_index;
    auto min_dis = std::numeric_limits<double>::max();
    for (auto j = before_index; j <= after_index; ++j) {
      auto dis = pow(discretized_path.at(j).x() - trajectory_point_x, 2) +
                 pow(discretized_path.at(j).y() - trajectory_point_y, 2);
      if (dis < min_dis) {
        min_dis = dis;
        min_dis_index = j;
      }
    }

    // calcualte v and a
    const auto cos_delta_theta = cos(
        discretized_path.at(min_dis_index).theta() - trajectory_point_theta);
    v = fmax(trajectory_point_v, 0.0) * cos_delta_theta;
    obstacle_info.v = v;
    obstacle_info.a = trajectory_point_a * cos_delta_theta;

    ADEBUG << "i:" << i << ", t:" << t << ", v:" << v
           << ", s_lower:" << obstacle_info.s_lower
           << ", s_upper:" << obstacle_info.s_upper;
  }

  min_t_ -= 1e-3;
  max_t_ += 1e-3;

  // add extra overtake buffer for front obstacle
  if (obstacle_->PerceptionSLBoundary().end_s() >
      reference_line_info.AdcSlBoundary().start_s()) {
    for (int i = 0; i < st_graph_time_count_; ++i) {
      const auto t = i * time_unit_;
#ifdef FOR_BAIDU_SIMULATION
      obstacle_infos_.at(i).s_upper += fmax(2.0, 0.4 * pow(t, 2));
#else
      obstacle_infos_.at(i).s_upper += fmax(2.0, 0.1 * pow(t, 2));
#endif
    }
  }
}

void STObstacleCache::EstimateObstacleInfos(
    const Frame& frame, const std::unique_ptr<TL::common::Interpolation2D>&
                            jerk_interpolation_ptr) {
  if (obstacle_ == nullptr ||
      obstacle_->Trajectory().trajectory_point().empty() ||
      obstacle_infos_.empty() || DefinitelyGreater(min_t_, 0.0) ||
      jerk_interpolation_ptr == nullptr) {
    return;
  }

  const auto& trajectory_points = obstacle_->Trajectory().trajectory_point();
  const auto& start_point = trajectory_points.at(0);
  ADEBUG << "front obs id: " << obstacle_->Perception().id()
         << ", start_v: " << start_point.v() << ", start_a:" << start_point.a()
         << ", perception_a_x:" << obstacle_->Perception().acceleration().x()
         << ", perception_a_y:" << obstacle_->Perception().acceleration().y();

  if (start_point.a() > 0.0) {
    return;
  }

  constexpr auto kMaxJerk = 1.0;
  constexpr auto kMinJerk = -1.0;
  auto jerk = jerk_interpolation_ptr->Interpolate(
      std::pair<double, double>(start_point.v(), start_point.a()));
  jerk = common::math::Clamp(jerk, kMinJerk, kMaxJerk);
  ADEBUG << " modify jerk: " << jerk;
  if (obstacle_->Trajectory().trajectory_point(0).v() > 16.67) {
    jerk = 0.0;
  }
  ModifyObstacleInfos(frame, 0.0, jerk, true);
}

void STObstacleCache::ModifyObstacleInfos(const Frame& frame, double accel,
                                          double jerk, bool extended) {
  const auto time_diff =
      frame.local_view().GetPredictionObstacles()->header().data_stamp() -
      (frame.vehicle_state().timestamp() +
       frame.PlanningStartPoint().relative_time());

  const auto& st_boundary = obstacle_->path_st_boundary();

  const auto& lower_points = st_boundary.lower_points();
  const auto& upper_points = st_boundary.upper_points();
  if (lower_points.empty() || upper_points.empty()) {
    return;
  }
  const auto& trajectory_points = obstacle_->Trajectory().trajectory_point();
  std::vector<double> accumulated_s;
  if (frame.local_view().HasFunctionManagerOut() &&
      frame.local_view().GetFunctionManagerOut() != nullptr &&
      frame.local_view().GetFunctionManagerOut()->fsm_state() ==
          functionmanager::MachineStateType::PERCEPTION_TYPE &&
      frame.local_view().GetFunctionManagerOut()->perception_sub_state() ==
          functionmanager::PerceptionSubState::CRUISE_TYPE &&
      (!frame.local_view().GetCruiseTargetId().empty() &&
       std::find(frame.local_view().GetCruiseTargetId().begin(),
                 frame.local_view().GetCruiseTargetId().end(),
                 obstacle_->PerceptionId()) !=
           frame.local_view().GetCruiseTargetId().end())) {
    accumulated_s.reserve(lower_points.size());
    accumulated_s.emplace_back(0.0);
    for (int i = 1; i < lower_points.size(); ++i) {
      accumulated_s.emplace_back(lower_points.at(i).s() -
                                 lower_points.front().s());
    }
  } else {
    accumulated_s.reserve(trajectory_points.size());
    auto iter = lower_points.begin();
    double s = 0.0;
    for (int i = 0; i < trajectory_points.size() && iter != lower_points.end();
         ++i) {
      const auto t = trajectory_points.at(i).relative_time() + time_diff;
      if (i > 0) {
        const auto& prev_point = trajectory_points.at(i - 1).path_point();
        const auto& cur_point = trajectory_points.at(i).path_point();
        s += std::hypot(cur_point.x() - prev_point.x(),
                        cur_point.y() - prev_point.y());
      }

      if (fabs(t - iter->t()) < 1e-4) {
        accumulated_s.emplace_back(s);
        ++iter;
      }
    }
  }

  if (accumulated_s.size() != lower_points.size()) {
    return;
  }
  std::vector<std::vector<double>> vec_vec_state;
  const auto& start_point = trajectory_points.at(0);
  if (DefinitelyGreater(start_point.a(), accel)) {
    TL::planning::util::GetStateAtMinJerk(
        start_point.v(), start_point.a(), 0.0,
        std::numeric_limits<double>::max(), accel, -fabs(jerk),
        st_graph_time_count_ * time_unit_, time_unit_, 0, &vec_vec_state);
  } else if (DefinitelyLess(start_point.a(), accel)) {
    TL::planning::util::GetStateAtMaxJerk(
        start_point.v(), start_point.a(), 0.0,
        std::numeric_limits<double>::max(), accel, fabs(jerk),
        st_graph_time_count_ * time_unit_, time_unit_, 0, &vec_vec_state);
  } else {
    TL::planning::util::GetStateAtMaxJerk(
        start_point.v(), start_point.a(), 0.0,
        std::numeric_limits<double>::max(), accel, 0.0,
        st_graph_time_count_ * time_unit_, time_unit_, 0, &vec_vec_state);
  }

  if (vec_vec_state.size() < 4) {
    return;
  }

  const auto& vec_s = vec_vec_state.at(1);
  const auto& vec_v = vec_vec_state.at(2);
  const auto count = common::math::Clamp(static_cast<int>(vec_s.size()), 0,
                                         st_graph_time_count_);
  auto time_index = std::numeric_limits<int>::lowest();
  int next_point_index = 0;
  for (int i = 0; i < count; ++i) {
    const auto t = i * time_unit_;
    const auto s = vec_s.at(i);

    auto& obstacle_info = obstacle_infos_.at(i);
    if (s < accumulated_s.front()) {
      continue;
    }

    if (s > accumulated_s.back()) {
      continue;
    }
    while (next_point_index + 1 < accumulated_s.size() &&
           accumulated_s.at(next_point_index) < s) {
      ++next_point_index;
    }

    const auto& prev_point_index =
        next_point_index == 0 ? 0 : next_point_index - 1;

    const auto ds =
        accumulated_s.at(next_point_index) - accumulated_s.at(prev_point_index);
    const auto ratio = common::math::double_type::IsZero(ds)
                           ? 0.0
                           : (s - accumulated_s.at(prev_point_index)) / ds;

    obstacle_info.s_lower =
        (1 - ratio) * lower_points.at(prev_point_index).s() +
        ratio * lower_points.at(next_point_index).s();
    obstacle_info.s_upper =
        (1 - ratio) * upper_points.at(prev_point_index).s() +
        ratio * upper_points.at(next_point_index).s();
    obstacle_info.v = vec_v.at(i);

    // update min_t_, max_t_
    if (time_index < 0) {
      min_t_ = t;
    }
    max_t_ = t;
    time_index = i;
  }

  if (extended && time_index >= 0) {
    auto& last_obstacle_info = obstacle_infos_.at(time_index);
    for (int i = time_index + 1; i < count; ++i) {
      const auto t = i * time_unit_;
      const auto delta_s = vec_s.at(i) - vec_s.at(time_index);
      auto& obstacle_info = obstacle_infos_.at(i);
      obstacle_info.s_lower = last_obstacle_info.s_lower + delta_s;
      obstacle_info.s_upper = last_obstacle_info.s_upper + delta_s;
      obstacle_info.v = vec_v.at(i);
      max_t_ = t;
    }
  }

  min_t_ -= 1e-3;
  max_t_ += 1e-3;
}

}  // namespace planning
}  // namespace TL
