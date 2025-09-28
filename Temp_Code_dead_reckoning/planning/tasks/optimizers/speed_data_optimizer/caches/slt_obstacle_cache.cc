/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file yield_obstacle_cache.cc
 **/
#include "planning/tasks/optimizers/speed_data_optimizer/caches/slt_obstacle_cache.h"
#include <math.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>

#include "common/configs/config_gflags.h"
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/util/macros.h"
#include "planning/common/obstacle.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/util/common.h"

#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {

SLTObstacleCache::SLTObstacleCache(const SpeedCacheConfig& config,
                                   const ReferenceLineInfo& reference_line_info,
                                   const Obstacle* obstacle,
                                   const double time_unit, const int time_count,
                                   int index)
    : index_(index),
      obstacle_(obstacle),
      time_unit_(time_unit),
      time_count_(time_count) {
  UNUSED(config);
  obstacle_infos_.assign(time_count, ObstacleInfo());
  if (obstacle_ == nullptr) {
    return;
  }
  ADEBUG << "time_count:" << time_count_ << ", id:" << obstacle_->Id();

  CacheObstacleInfos(reference_line_info);
  CalculateStTimeLengthCoef();
  is_front_ = obstacle->PerceptionSLBoundary().end_s() >
              reference_line_info.AdcSlBoundary().end_s();

  min_stop_distance_ = obstacle_->NormalStopDistance();
  min_follow_distance_ = min_stop_distance_;

  id_ = obstacle->Id();
  perception_id_ = obstacle_->PerceptionId();
}

void SLTObstacleCache::CacheObstacleInfos(
    const ReferenceLineInfo& reference_line_info) {
  const auto& slt_boundary = obstacle_->GetPathSLTBoundary();
  const auto& st_boundary = slt_boundary.GetSTBoundary();
  const auto& lt_boundary = slt_boundary.GetLTBoundary();
  const auto& st_lower_points = st_boundary.lower_points();
  const auto& st_upper_points = st_boundary.upper_points();
  const auto& lt_lower_points = lt_boundary.lower_points();
  const auto& lt_upper_points = lt_boundary.upper_points();
  const auto& ds_points = slt_boundary.GetDsPoints();
  const auto& dl_points = slt_boundary.GetDlPoints();

  int next_point_index = 0;
  auto time_count = static_cast<int>(round(st_boundary.max_t() / time_unit_));
  time_count = std::min(time_count + 1, time_count_);
  auto time_index = std::numeric_limits<int>::lowest();
  for (int i = 0; i < time_count; ++i) {
    const auto t = i * time_unit_;
    if (t < st_boundary.min_t()) {
      continue;
    }

    if (t > st_boundary.max_t()) {
      break;
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
    obstacle_info.s_lower =
        (1 - ratio) * prev_st_lower_point.s() + ratio * next_st_lower_point.s();
    obstacle_info.s_upper =
        (1 - ratio) * st_upper_points.at(prev_point_index).s() +
        ratio * st_upper_points.at(next_point_index).s();
    obstacle_info.l_lower =
        (1 - ratio) * lt_lower_points.at(prev_point_index).s() +
        ratio * lt_lower_points.at(next_point_index).s();
    obstacle_info.l_upper =
        (1 - ratio) * lt_upper_points.at(prev_point_index).s() +
        ratio * lt_upper_points.at(next_point_index).s();
    obstacle_info.ds = (1 - ratio) * ds_points.at(prev_point_index).s() +
                       ratio * ds_points.at(next_point_index).s();
    obstacle_info.dl = (1 - ratio) * dl_points.at(prev_point_index).s() +
                       ratio * dl_points.at(next_point_index).s();
    obstacle_info.t = t;

    // update min_t_, max_t_
    if (time_index < 0) {
      min_t_ = t;
    }
    max_t_ = t;
    time_index = i;
  }

  if (time_index >= 0 && time_index < time_count_) {
    // calculate obstacle speed in st_boundary_time_count_
    auto& last_obstacle_info = obstacle_infos_.at(time_index);
    for (int j = time_index + 1; j < time_count_; ++j) {
      auto& obstacle_info = obstacle_infos_.at(j);
      obstacle_info.ds = last_obstacle_info.ds;
      obstacle_info.dds = last_obstacle_info.dds;
      const auto dt = (j - time_index) * time_unit_;
      const auto s =
          last_obstacle_info.ds * dt + 0.5 * last_obstacle_info.dds * dt * dt;
      obstacle_info.s_upper = last_obstacle_info.s_upper + s;
      obstacle_info.s_lower = last_obstacle_info.s_lower + s;
      obstacle_info.dl = last_obstacle_info.dl;
      const auto l = last_obstacle_info.dl * (j - time_index) * time_unit_;
      obstacle_info.l_upper = last_obstacle_info.l_upper + l;
      obstacle_info.l_lower = last_obstacle_info.l_lower + l;
      obstacle_info.t = j * time_unit_;
    }
    max_t_ = (time_count_ - 1) * time_unit_;
  }

  min_t_ -= 1e-3;
  max_t_ += 1e-3;

  // add extra overtake buffer for front obstacle
  if (obstacle_->PerceptionSLBoundary().end_s() >
      reference_line_info.AdcSlBoundary().start_s()) {
    for (int i = 0; i < time_count; ++i) {
      const auto t = i * time_unit_;
#ifdef FOR_BAIDU_SIMULATION
      obstacle_infos_.at(i).s_upper += fmax(2.0, 0.4 * pow(t, 2));
#else
      obstacle_infos_.at(i).s_upper += fmax(2.0, 0.1 * pow(t, 2));
#endif
    }
  }

  CalculateLRange();
}

void SLTObstacleCache::CheckIfCutInBegin(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  is_cut_in_ = !IsPerceptionSLOnReferenceLine(reference_line_info) &&
               IsTrajectoryOnReferenceLine(frame, reference_line_info);
}

bool SLTObstacleCache::IsPerceptionSLOnReferenceLine(
    const ReferenceLineInfo& reference_line_info) {
  if (obstacle_ == nullptr) {
    return false;
  }
  const auto& obs_boundary = obstacle_->PerceptionSLBoundary();
  const auto middle_s = (obs_boundary.start_s() + obs_boundary.end_s()) / 2.0;
  const auto middle_l = (obs_boundary.start_l() + obs_boundary.end_l()) / 2.0;
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line_info.reference_line().map_path().GetLaneWidth(
      middle_s, &lane_left_width, &lane_right_width);
  return (middle_l < lane_left_width) && (middle_l > -lane_right_width);
}

bool SLTObstacleCache::IsTrajectoryOnReferenceLine(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  if (obstacle_ == nullptr || frame.GetReferenceLineProvider() == nullptr ||
      frame.GetReferenceLineProvider()->GetPncMap() == nullptr) {
    return false;
  }

  const auto& adc_lane =
      reference_line_info.reference_line().GetADCWaypoint().lane;
  if (adc_lane == nullptr) {
    return false;
  }

  const auto& trajectory = obstacle_->Trajectory();
  if (trajectory.trajectory_point().empty()) {
    return false;
  }

  const auto target_lane_id =
      trajectory.trajectory_point(trajectory.trajectory_point().size() - 1)
          .path_point()
          .lane_id();
  const auto& adc_passage =
      frame.GetReferenceLineProvider()->GetPncMap()->GetAdcPassageRoutingInfo();
  if (adc_passage) {
    return adc_passage->IsLaneExtExist(target_lane_id);
  }
  return false;
}

void SLTObstacleCache::EstimateObstacleInfos(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const std::unique_ptr<TL::common::Interpolation2D>&
        jerk_interpolation_ptr) {
  if (obstacle_ == nullptr ||
      obstacle_->Trajectory().trajectory_point().empty() ||
      obstacle_infos_.empty() || jerk_interpolation_ptr == nullptr) {
    return;
  }

  const auto& slt_boundary = obstacle_->GetPathSLTBoundary();
  const auto& st_boundary = slt_boundary.GetSTBoundary();
  const auto& lt_boundary = slt_boundary.GetLTBoundary();
  const auto& st_lower_points = st_boundary.lower_points();
  const auto& st_upper_points = st_boundary.upper_points();
  const auto& lt_lower_points = lt_boundary.lower_points();
  const auto& lt_upper_points = lt_boundary.upper_points();
  const auto& theta_points = slt_boundary.GetThetaPoints();

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
    accumulated_s.reserve(st_lower_points.size());
    accumulated_s.emplace_back(0);
    for (int i = 1; i < st_lower_points.size(); ++i) {
      accumulated_s.emplace_back(st_lower_points.at(i).s() -
                                 st_lower_points.front().s());
    }
  } else {
    accumulated_s.reserve(trajectory_points.size());
    accumulated_s.emplace_back(0.0);
    for (int i = 1; i < trajectory_points.size(); ++i) {
      const auto& prev_point = trajectory_points.at(i - 1).path_point();
      const auto& cur_point = trajectory_points.at(i).path_point();
      accumulated_s.emplace_back(accumulated_s.back() +
                                 std::hypot(cur_point.x() - prev_point.x(),
                                            cur_point.y() - prev_point.y()));
    }
  }

  const auto& start_point = trajectory_points.at(0);
  ADEBUG << "start_v: " << start_point.v() << ", start_a:" << start_point.a()
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
  std::vector<std::vector<double>> vec_vec_state;

  auto start_s = 0.0;
  auto start_v = start_point.v();
  auto start_a = start_point.a();
  const auto time_diff =
      (frame.vehicle_state().timestamp() +
       frame.PlanningStartPoint().relative_time()) -
      frame.local_view().GetPredictionObstacles()->header().data_stamp();
  if (time_diff > 0 &&
      TL::planning::util::GetStateAtMaxJerk(
          start_v, start_a, 0.0, std::numeric_limits<double>::max(), 0.0, jerk,
          time_diff, time_diff, 0, &vec_vec_state) &&
      vec_vec_state.size() >= 4 && !vec_vec_state.at(1).empty() &&
      !vec_vec_state.at(2).empty() && !vec_vec_state.at(3).empty()) {
    start_s = vec_vec_state.at(1).back();
    start_v = vec_vec_state.at(2).back();
    start_a = vec_vec_state.at(3).back();
  }

  vec_vec_state.clear();
  TL::planning::util::GetStateAtMaxJerk(
      start_v, start_a, 0.0, std::numeric_limits<double>::max(), 0.0, jerk,
      time_count_ * time_unit_, time_unit_, 0, &vec_vec_state);

  if (vec_vec_state.size() < 4) {
    return;
  }
  const auto& vec_s = vec_vec_state.at(1);
  const auto& vec_v = vec_vec_state.at(2);
  const auto& vec_a = vec_vec_state.at(3);
  const auto count =
      common::math::Clamp(static_cast<int>(vec_s.size()), 0, time_count_);

  int i = 0;
  int next_point_index = 0;
  auto cos_theta = 0.0;
  auto sin_theta = 0.0;
  for (; i < count; ++i) {
    auto& obstacle_info = obstacle_infos_.at(i);
    const auto s = vec_s.at(i) + start_s;
    const auto v = vec_v.at(i);
    const auto a = vec_a.at(i);

    if (s >= accumulated_s.back()) {
      break;
    }

    while (next_point_index + 1 < st_lower_points.size() &&
           accumulated_s.at(next_point_index) < s) {
      ++next_point_index;
    }

    const auto& prev_point_index =
        next_point_index == 0 ? 0 : next_point_index - 1;

    const auto s_interval =
        accumulated_s.at(next_point_index) - accumulated_s.at(prev_point_index);
    const auto ratio =
        common::math::double_type::IsZero(s_interval)
            ? 0.0
            : (s - accumulated_s.at(prev_point_index)) / s_interval;

    obstacle_info.s_lower =
        (1 - ratio) * st_lower_points.at(prev_point_index).s() +
        ratio * st_lower_points.at(next_point_index).s();
    obstacle_info.s_upper =
        (1 - ratio) * st_upper_points.at(prev_point_index).s() +
        ratio * st_upper_points.at(next_point_index).s();
    obstacle_info.l_lower =
        (1 - ratio) * lt_lower_points.at(prev_point_index).s() +
        ratio * lt_lower_points.at(next_point_index).s();
    obstacle_info.l_upper =
        (1 - ratio) * lt_upper_points.at(prev_point_index).s() +
        ratio * lt_upper_points.at(next_point_index).s();
    auto theta = (1 - ratio) * theta_points.at(prev_point_index).s() +
                 ratio * theta_points.at(next_point_index).s();

    cos_theta = cos(theta);
    sin_theta = sin(theta);
    obstacle_info.ds = v * cos_theta;
    obstacle_info.dl = v * sin_theta;
    obstacle_info.dds = a * cos_theta;
    obstacle_info.ddl = a * cos_theta;
    obstacle_info.t = i * time_unit_;
  }

  if (i > 0) {
    // calculate obstacle speed in st_boundary_time_count_
    auto& last_obstacle_info = obstacle_infos_.at(i - 1);
    const auto last_s = vec_s.at(i - 1);

    for (int j = i; j < count; ++j) {
      auto& obstacle_info = obstacle_infos_.at(j);

      const auto delta_s = (vec_s.at(j) - last_s) * cos_theta;
      obstacle_info.s_upper = last_obstacle_info.s_upper + delta_s;
      obstacle_info.s_lower = last_obstacle_info.s_lower + delta_s;

      const auto delta_l = (vec_s.at(j) - last_s) * sin_theta;
      obstacle_info.l_upper = last_obstacle_info.l_upper + delta_l;
      obstacle_info.l_lower = last_obstacle_info.l_lower + delta_l;

      obstacle_info.ds = vec_v.at(j) * cos_theta;
      obstacle_info.dl = vec_v.at(j) * sin_theta;
      obstacle_info.dds = vec_a.at(j) * cos_theta;
      obstacle_info.ddl = vec_a.at(j) * sin_theta;
      obstacle_info.t = i * time_unit_;
    }

    for (int j = count; j < time_count_; ++j) {
      obstacle_infos_.at(j) = obstacle_infos_.at(count - 1);
    }
    max_t_ = (time_count_ - 1) * time_unit_;
  }

  min_t_ -= 1e-3;
  max_t_ += 1e-3;

  // add extra overtake buffer for front obstacle
  if (obstacle_->PerceptionSLBoundary().end_s() >
      reference_line_info.AdcSlBoundary().start_s()) {
    for (int i = 0; i < time_count_; ++i) {
      const auto t = i * time_unit_;
#ifdef FOR_BAIDU_SIMULATION
      obstacle_infos_.at(i).s_upper += fmax(2.0, 0.4 * pow(t, 2));
#else
      obstacle_infos_.at(i).s_upper += fmax(2.0, 0.1 * pow(t, 2));
#endif
    }
  }

  CalculateLRange();
}

bool SLTObstacleCache::Forecast(const Obstacle* obstacle, const double time) {
  if (obstacle == nullptr) {
    return false;
  }
  obstacle_ = obstacle;
  auto old_obstacle_infos = obstacle_infos_;
  const auto old_min_t_ = min_t_;
  const auto old_max_t_ = max_t_;
  min_t_ = std::numeric_limits<double>::max();
  max_t_ = std::numeric_limits<double>::lowest();
  for (int i = 0; i < time_count_; ++i) {
    const auto t = i * time_unit_;
    const auto old_t = t + time;
    const auto old_index = old_t / time_unit_;
    const auto prev_old_index = static_cast<int>(floor(old_index));
    const auto next_old_index = static_cast<int>(ceil(old_index));
    if (old_t < old_min_t_ || old_t > old_max_t_ || prev_old_index < 0 ||
        prev_old_index >= time_count_ || next_old_index < 0 ||
        next_old_index >= time_count_) {
      continue;
    }

    if (max_t_ < 0) {
      min_t_ = t;
    }
    max_t_ = t;

    const auto ratio =
        (t + time - old_obstacle_infos.at(prev_old_index).t) / time_unit_;
    const auto& prev_old_obstacle_info = old_obstacle_infos.at(prev_old_index);
    const auto& next_old_obstacle_info = old_obstacle_infos.at(next_old_index);
    auto& obstacle_info = obstacle_infos_.at(i);
    obstacle_info.s_lower = (1 - ratio) * prev_old_obstacle_info.s_lower +
                            ratio * next_old_obstacle_info.s_lower;
    obstacle_info.s_upper = (1 - ratio) * prev_old_obstacle_info.s_upper +
                            ratio * next_old_obstacle_info.s_upper;
    obstacle_info.l_lower = (1 - ratio) * prev_old_obstacle_info.l_lower +
                            ratio * next_old_obstacle_info.l_lower;
    obstacle_info.l_upper = (1 - ratio) * prev_old_obstacle_info.l_upper +
                            ratio * next_old_obstacle_info.l_upper;
    obstacle_info.ds = (1 - ratio) * prev_old_obstacle_info.ds +
                       ratio * next_old_obstacle_info.ds;
    obstacle_info.dl = (1 - ratio) * prev_old_obstacle_info.dl +
                       ratio * next_old_obstacle_info.dl;
    obstacle_info.dds = (1 - ratio) * prev_old_obstacle_info.dds +
                        ratio * next_old_obstacle_info.dds;
    obstacle_info.ddl = (1 - ratio) * prev_old_obstacle_info.ddl +
                        ratio * next_old_obstacle_info.ddl;
  }

  min_t_ -= 1e-3;
  max_t_ += 1e-3;

  CalculateLRange();

  return true;
}

void SLTObstacleCache::CalculateStTimeLengthCoef() {
  if (obstacle_ == nullptr || obstacle_->path_st_boundary().IsEmpty()) {
    st_time_length_coef_ = 1.0;
    return;
  }
  const auto start_t = obstacle_->path_st_boundary().min_t();
  const auto end_t = obstacle_->path_st_boundary().max_t();
  const auto start_t_coef = 1;
  // const auto time_length_coef =
  //     fmin(fmax(end_t - start_t, 0.01) / FLAGS_trajectory_time_length, 1.0);
  const auto end_t_coef =
      fmin(fmax(end_t, 0.01) / FLAGS_trajectory_time_length, 1.0);
  const auto start_coef =
      fmin((fmax(FLAGS_trajectory_time_length - start_t, 0.01)) /
               FLAGS_trajectory_time_length,
           1.0);
  st_time_length_coef_ = end_t_coef * start_coef;
}

void SLTObstacleCache::CalculateLRange() {
  min_l_ = -std::numeric_limits<double>::infinity();
  max_l_ = std::numeric_limits<double>::infinity();

  for (int i = 0; i < time_count_; ++i) {
    const auto t = i * time_unit_;
    if (t < min_t_) {
      continue;
    }

    if (t > max_t_) {
      break;
    }

    const auto& obstacle_info = obstacle_infos_.at(i);
    if (std::isinf(min_l_) || min_l_ > obstacle_info.l_lower) {
      min_l_ = obstacle_info.l_lower;
    }
    if (std::isinf(max_l_) || max_l_ < obstacle_info.l_upper) {
      max_l_ = obstacle_info.l_upper;
    }
  }
}

}  // namespace planning
}  // namespace TL
