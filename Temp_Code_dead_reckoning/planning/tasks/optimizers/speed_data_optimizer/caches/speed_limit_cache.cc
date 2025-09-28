/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_cost.h
 **/
#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_limit_cache.h"
#include <math.h>  // NOLINT

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>
#include <set>
#include <tuple>
#include <vector>

#include "common/configs/vehicle_config_helper.h"
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"
#include "map/hdmap/path.h"
#include "planning/common/frame.h"
#include "planning/common/obstacle.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/speed_limit.h"
#include "planning/common/util/common.h"
#include "planning/hmi/lon_hmi/speed_convertor/speed_convertor.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/slt_obstacle_cache.h"
#include "planning/proto/planner_open_space_config.pb.h"
#include "planning/proto/speed_evaluator_config.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreaterEqual;
using common::math::double_type::DefinitelyLessEqual;
using common::math::double_type::SeemsEqual;

SpeedLimitCache::SpeedLimitCache()
    : vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
}

void SpeedLimitCache::Init(
    const SpeedCacheConfig& config, const Frame& frame,
    const ReferenceLineInfo& reference_line_info,
    const common::TrajectoryPoint& init_point, const Frame* last_frame,
    const double distance_from_last_frame,
    const std::vector<const SLTObstacleCache*>& speed_limit_obstacle_caches_) {

  if (cruise_target_speed_filter_ == nullptr) {
    cruise_target_speed_filter_ = std::make_shared<common::MeanFilter>(
        config.cruise_target_speed_filter_window_size());
  }

  distance_from_last_frame_ = distance_from_last_frame;
  const auto sequence_num = frame.SequenceNum();

  // if this is a new frame, record last_speed_limit_infos_
  if (sequence_num_ != sequence_num &&
      sequence_num_ != std::numeric_limits<uint32_t>::max()) {
    last_position_speed_limits_ = position_speed_limits_;
    last_start_s_ = current_start_s_;
    last_end_s_ = current_end_s_;
    last_unit_s_ = current_unit_s_;
  }
  sequence_num_ = sequence_num;
  if (speeds_.empty() && nudge_speed_limit_coefs_.empty()) {
    if (config.has_nudge_speed_limit_coef_calibration_2d_table() &&
        config.nudge_speed_limit_coef_calibration_2d_table()
                .calibration_info()
                .size() > 1) {
      for (const auto& calibration_info :
           config.nudge_speed_limit_coef_calibration_2d_table()
               .calibration_info()) {
        speeds_.emplace_back(calibration_info.speed());
        nudge_speed_limit_coefs_.emplace_back(
            calibration_info.nudge_speed_limit_coef());
      }
    } else {
      speeds_ = {1, 6};
      nudge_speed_limit_coefs_ = {6, 15};
    }
  }

  time_speed_limits_.clear();
  position_speed_limits_.clear();
  keep_clear_range_.clear();
  low_speed_thresholds_.clear();

  UpdatePreviewDistance(init_point);
  CachePositionSpeedLimits(config, frame, reference_line_info, last_frame);
  CacheTimeSpeedLimits(config, frame, reference_line_info, init_point,
                       last_frame);
  AddToKeepClearRange(reference_line_info.path_decision().obstacles().Items());
  CalculateCruiseTargetSpeed(reference_line_info);
  ModifySpeedLimits(cruise_target_speed_);
  CacheLowSpeedThreshold(init_point, reference_line_info);
  CachePedestrianSpeedLimit(config, init_point);
  CacheNudgeSpeedLimit(config, speed_limit_obstacle_caches_, frame, init_point,
                       reference_line_info);
}

void SpeedLimitCache::CachePositionSpeedLimits(
    const SpeedCacheConfig& config, const Frame& frame,
    const ReferenceLineInfo& reference_line_info, const Frame* last_frame) {
  const auto& speed_limit_info_points = reference_line_info.st_graph_data()
                                            .speed_limit()
                                            .speed_limit_info_points();
  // if speed limit point size < 2
  if (speed_limit_info_points.size() < 2) {
    current_start_s_ = 0.0;
    current_end_s_ =
        reference_line_info.path_data().discretized_path().Length();
    current_unit_s_ = (current_end_s_ - current_start_s_) / (s_count_ - 1);

    if (static_cast<int>(position_speed_limits_.size()) < s_count_) {
      position_speed_limits_.resize(s_count_);
    }
    auto curvature_speed_limit = FLAGS_planning_upper_speed_limit;
    auto map_speed_limit = FLAGS_planning_upper_speed_limit;
    auto decision_speed_limit = FLAGS_planning_upper_speed_limit;
    auto pedestrian_speed_limit = FLAGS_planning_upper_speed_limit;
    auto origin_map_speed_limit = FLAGS_planning_upper_speed_limit;
    auto allow_over_speed = false;
    if (!speed_limit_info_points.empty()) {
      const auto& first_point = speed_limit_info_points.front();
      curvature_speed_limit = first_point.second.curvature_speed_limit;
      map_speed_limit = first_point.second.map_speed_limit;
      decision_speed_limit = first_point.second.decision_speed_limit;
      pedestrian_speed_limit = first_point.second.pedestrian_speed_limit;
      origin_map_speed_limit = first_point.second.origin_map_speed_limit;
      allow_over_speed = first_point.second.allow_over_speed;
    }
    for (int i = 0; i < s_count_; ++i) {
      position_speed_limits_.at(i).curvature_speed_limit =
          curvature_speed_limit;
      position_speed_limits_.at(i).map_speed_limit = map_speed_limit;
      position_speed_limits_.at(i).decision_speed_limit = decision_speed_limit;
      position_speed_limits_.at(i).pedestrian_speed_limit =
          pedestrian_speed_limit;
      position_speed_limits_.at(i).origin_map_speed_limit =
          origin_map_speed_limit;
      position_speed_limits_.at(i).allow_over_speed = allow_over_speed;
    }
    return;
  }

  // if speed limit point size >= 2
  current_start_s_ = speed_limit_info_points.front().first;
  current_end_s_ = speed_limit_info_points.back().first;
  current_unit_s_ = (current_end_s_ - current_start_s_) / (s_count_ - 1);

  if (static_cast<int>(position_speed_limits_.size()) < s_count_) {
    position_speed_limits_.resize(s_count_);
  }

  int point_index = 1;
  for (int i = 0; i < s_count_; ++i) {
    const auto s = current_start_s_ + i * current_unit_s_;
    bool cross = false;
    while (point_index + 1 < static_cast<int>(speed_limit_info_points.size()) &&
           s > speed_limit_info_points.at(point_index).first) {
      cross = true;
      ++point_index;
    }
    const auto& next_point = speed_limit_info_points.at(point_index);
    const auto& prev_point = speed_limit_info_points.at(point_index - 1);
    auto ratio = (s - prev_point.first) / (next_point.first - prev_point.first);

    auto& position_speed_limit = position_speed_limits_.at(i);
    position_speed_limit.curvature_speed_limit =
        (1 - ratio) * prev_point.second.curvature_speed_limit +
        ratio * next_point.second.curvature_speed_limit;
    position_speed_limit.decision_speed_limit =
        (1 - ratio) * prev_point.second.decision_speed_limit +
        ratio * next_point.second.decision_speed_limit;
    position_speed_limit.pedestrian_speed_limit =
        (1 - ratio) * prev_point.second.pedestrian_speed_limit +
        ratio * next_point.second.pedestrian_speed_limit;
    position_speed_limit.map_speed_limit = next_point.second.map_speed_limit;
    position_speed_limit.origin_map_speed_limit =
        next_point.second.origin_map_speed_limit;
    position_speed_limit.allow_over_speed = next_point.second.allow_over_speed;
    if (cross) {
      position_speed_limit.curvature_speed_limit =
          fmin(position_speed_limit.curvature_speed_limit,
               prev_point.second.curvature_speed_limit);
      position_speed_limit.decision_speed_limit =
          fmin(position_speed_limit.decision_speed_limit,
               prev_point.second.decision_speed_limit);
    }
  }

  // smooth curvature speed limit between different frames
  if (last_frame != nullptr &&
      (frame.vehicle_state().driving_mode() ==
           soc::Chassis::COMPLETE_AUTO_DRIVE ||
       frame.vehicle_state().driving_mode() == soc::Chassis::AUTO_SPEED_ONLY) &&
      (last_frame->vehicle_state().driving_mode() ==
           soc::Chassis::COMPLETE_AUTO_DRIVE ||
       last_frame->vehicle_state().driving_mode() ==
           soc::Chassis::AUTO_SPEED_ONLY)) {
    SmoothCurvatureSpeedLimit(config, reference_line_info.path_data());
  }
}

void SpeedLimitCache::SmoothCurvatureSpeedLimit(const SpeedCacheConfig& config,
                                                const PathData& path_data) {
  if (last_position_speed_limits_.empty()) {
    return;
  }

  const auto& discretized_path = path_data.discretized_path();
  const auto need_smooth = std::all_of(
      discretized_path.begin(), discretized_path.end(), [&](const auto& point) {
        return fabs(point.l()) <
               config.curvature_speed_limit_smooth_l_threshold();
      });

  for (int i = 0; i < s_count_; ++i) {
    const auto s = current_start_s_ + i * current_unit_s_;
    auto& position_speed_limit = position_speed_limits_.at(i);
    const auto& last_position_speed_limit =
        GetLastPositionSpeedLimit(s + distance_from_last_frame_);

    // for curvature speed limit, quick decelerate and slow accelerate
    if (position_speed_limit.curvature_speed_limit >
            last_position_speed_limit.curvature_speed_limit &&
        position_speed_limit.curvature_speed_limit <
            last_position_speed_limit.curvature_speed_limit +
                config.delta_curvature_speed_limit_threshold()) {
      position_speed_limit.curvature_speed_limit =
          last_position_speed_limit.curvature_speed_limit;
      continue;
    }

    // smooth curvature speed limit between different frames
    if (need_smooth) {
      const auto ratio =
          fmin(1.0, s / discretized_path.Length() *
                        config.curvature_speed_limit_smooth_s_ratio());
      position_speed_limit.curvature_speed_limit =
          position_speed_limit.curvature_speed_limit * ratio +
          (1 - ratio) * last_position_speed_limit.curvature_speed_limit;
    }
  }
}

void SpeedLimitCache::CacheTimeSpeedLimits(
    const SpeedCacheConfig& config, const Frame& frame,
    const ReferenceLineInfo& reference_line_info,
    const common::TrajectoryPoint& init_point, const Frame* last_frame) {
  time_count_ = static_cast<int>(round(
                    reference_line_info.st_graph_data().total_time_by_conf() /
                    time_unit_)) +
                1;
  time_speed_limits_.assign(
      time_count_,
      {reference_line_info.GetCruiseSpeed(), FLAGS_planning_upper_speed_limit,
       FLAGS_planning_upper_speed_limit, FLAGS_planning_upper_speed_limit});

  ProcessMapSpeedLimitsDecreased(config, init_point, reference_line_info,
                                 last_frame);
  CacheCriticalSpeedLimit(config, init_point, frame, reference_line_info);

  CacheComfortableSpeedLimit(config, init_point, frame, reference_line_info);

  if (nudge_speed_limits_.size() < time_count_) {
    nudge_speed_limits_.assign(
        time_count_,
        std::vector<double>(s_count_, FLAGS_planning_upper_speed_limit));
  }
}

bool SpeedLimitCache::CalculateComfortDecelSpeedLimits(
    const common::TrajectoryPoint& init_point, const double target_speed,
    const double target_speed_start_s, const double comfort_decel,
    const double comfort_jerk, const double time_length,
    std::vector<double>* speed_limits) const {
  if (speed_limits == nullptr) {
    return false;
  }
  speed_limits->reserve(time_count_);

  std::vector<std::vector<double>> vec_vec_state;
  if (DefinitelyGreaterEqual(target_speed, init_point.v()) ||
      !TL::planning::util::GetStateAtMinJerk(
          init_point.v(), init_point.a(), target_speed,
          std::numeric_limits<double>::max(), comfort_decel, comfort_jerk,
          time_length, time_unit_, 0.0, &vec_vec_state) ||
      vec_vec_state.size() < 4) {
    return false;
  }

  const auto& s_state = vec_vec_state.at(1);
  const auto& v_state = vec_vec_state.at(2);
  const auto& a_state = vec_vec_state.at(3);
  const auto state_count =
      std::min({s_state.size(), v_state.size(), a_state.size()});
  if (state_count <= 0) {
    return false;
  }

  double end_s = s_state.back();
  for (int i = 0; i < static_cast<int>(state_count); ++i) {
    const auto v = v_state.at(i);
    const auto a = a_state.at(i);
    speed_limits->emplace_back(v);

    const auto remain_t = a / comfort_jerk;
    if (DefinitelyLessEqual(remain_t, 0.0) ||
        DefinitelyGreaterEqual(
            (v + a * remain_t - comfort_jerk * pow(remain_t, 2) / 2.0),
            target_speed)) {
      continue;
    }

    const auto remain_t_count =
        static_cast<int>(round(remain_t / time_unit_)) + 1;
    for (int j = 1; j < remain_t_count; ++j) {
      const auto t = j * time_unit_;
      speed_limits->emplace_back(
          fmax(target_speed, v + a * t - comfort_jerk * pow(t, 2) / 2.0));
    }
    for (std::size_t j = speed_limits->size(); j < time_count_; ++j) {
      speed_limits->emplace_back(target_speed);
    }
    end_s = s_state.at(i) + v * remain_t + a * pow(remain_t, 2) / 2.0 -
            comfort_jerk * pow(remain_t, 3) / 6.0;
    break;
  }
  return end_s > target_speed_start_s;
}

void SpeedLimitCache::CacheCruiseSpeedLimits(
    const SpeedCacheConfig& config,
    const ReferenceLineInfo& reference_line_info,
    const common::TrajectoryPoint& init_point) {

  std::vector<double> speed_limits;
  const auto cruise_speed = reference_line_info.GetCruiseSpeed();
  if (DefinitelyLessEqual(init_point.v(), cruise_speed) ||
      !CalculateComfortDecelSpeedLimits(
          init_point, cruise_speed, 0.0, config.dece_when_exceed_cruise_speed(),
          config.jerk_when_exceed_cruise_speed(), 100.0, &speed_limits) ||
      static_cast<int>(speed_limits.size()) < time_count_ ||
      static_cast<int>(time_speed_limits_.size()) < time_count_) {
    return;
  }

  for (int i = 0; i < time_count_; ++i) {
    time_speed_limits_.at(i).cruise_speed_limit = speed_limits.at(i);
  }
}

void SpeedLimitCache::ProcessMapSpeedLimitsDecreased(
    const SpeedCacheConfig& config, const common::TrajectoryPoint& init_point,
    const ReferenceLineInfo& reference_line_info, const Frame* last_frame) {
  auto current_map_speed_limit = std::numeric_limits<double>::max();
  std::vector<std::vector<double>> map_speed_limit_sequences;

  // calculate speed limit info
  const auto& discretized_path =
      reference_line_info.path_data().discretized_path();
  const auto& frenet_frame_path =
      reference_line_info.path_data().frenet_frame_path();
  if (discretized_path.size() != frenet_frame_path.size()) {
    return;
  }

  const auto last_frame_valid =
      last_frame != nullptr && last_frame->DriveReferenceLineInfo() != nullptr;

  for (int i = 0; i < static_cast<int>(discretized_path.size()); ++i) {
    const auto& position_speed_limit =
        GetPositionSpeedLimit(discretized_path.at(i).s());

    if (SeemsEqual(current_map_speed_limit,
                   position_speed_limit.map_speed_limit)) {
      continue;
    }
    current_map_speed_limit = position_speed_limit.map_speed_limit;

    // for map speed limit, keep comfort decelerate
    if (init_point.v() < current_map_speed_limit) {
      continue;
    }

    // adc encounter a map speed limit, there two cases
    // 1. this is a new map speed limit that has not been processed in the previous frame
    //    if map speed limit start s is so far away, ads dose not need to slow down immediately
    // 2. this is a old speed limit that has been processed in the previous frame
    //    adc should slow down immediately
    auto target_speed_start_s = discretized_path.at(i).s();
    const auto reference_point =
        reference_line_info.reference_line().GetReferencePoint(
            frenet_frame_path.at(i).s());
    if (reference_point.lane_waypoints().empty()) {
      continue;
    }

    if (last_frame_valid) {
      const auto& lane_id =
          reference_point.lane_waypoints().front().lane->id().id();
      const auto& route_segments =
          last_frame->DriveReferenceLineInfo()->Lanes();
      if (std::any_of(route_segments.begin(), route_segments.end(),
                      [&](const auto& route_segment) {
                        return route_segment.lane != nullptr &&
                               route_segment.lane->id().id() == lane_id;
                      })) {
        target_speed_start_s = std::numeric_limits<double>::lowest();
      }
    }

    std::vector<double> comfort_map_speed_limits;
    if (CalculateComfortDecelSpeedLimits(
            init_point, current_map_speed_limit, target_speed_start_s,
            config.dece_when_exceed_map_speed_limit(),
            config.jerk_when_exceed_map_speed_limit(), 100.0,
            &comfort_map_speed_limits)) {
      map_speed_limit_sequences.emplace_back(
          std::move(comfort_map_speed_limits));
    }
  }

  for (int i = 0; i < time_count_; ++i) {
    auto min_map_speed_limit = std::numeric_limits<double>::infinity();
    for (const auto& map_speed_limit_sequence : map_speed_limit_sequences) {
      if (map_speed_limit_sequence.empty()) {
        continue;
      }
      if (i < static_cast<int>(map_speed_limit_sequence.size())) {
        min_map_speed_limit =
            fmin(min_map_speed_limit, map_speed_limit_sequence.at(i));
      } else {
        min_map_speed_limit =
            fmin(min_map_speed_limit, map_speed_limit_sequence.back());
      }
    }

    time_speed_limits_.at(i).map_speed_limit = min_map_speed_limit;
  }

  for (auto& position_speed_limit : position_speed_limits_) {
    position_speed_limit.real_map_speed_limit =
        position_speed_limit.map_speed_limit;
#ifndef FOR_BAIDU_SIMULATION
    position_speed_limit.map_speed_limit =
        fmax(position_speed_limit.map_speed_limit, init_point.v());
#endif
  }
}

void SpeedLimitCache::UpdatePreviewDistance(
    const common::TrajectoryPoint& init_point) {
  constexpr auto a_comfort = -1.0;
  constexpr auto j_comfort = -1.0;
  std::vector<std::vector<double>> vec_vec_state;
  auto s = fabs(init_point.v()) * 5.0;
  if (TL::planning::util::GetStateAtMinJerk(
          init_point.v(), init_point.a(), 0.0, FLAGS_planning_upper_speed_limit,
          a_comfort, j_comfort, 40.0, 1.0, 0, &vec_vec_state) &&
      !vec_vec_state[1].empty()) {
    s = fmax(
        *std::max_element(vec_vec_state[1].begin(), vec_vec_state[1].end()), s);
  }

  if (s > preview_distance_ || s < preview_distance_ - 20) {
    preview_distance_ = s;
  }
  ADEBUG << "preview_distance:" << preview_distance_;
}

void SpeedLimitCache::CalculateCruiseTargetSpeed(
    const ReferenceLineInfo& reference_line_info) {
  const double max_adc_stop_speed = common::VehicleConfigHelper::GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  const auto cruise_speed = reference_line_info.GetCruiseSpeed();
  const auto s =
      fmin(preview_distance_,
           reference_line_info.path_data().discretized_path().Length());
  const auto cruise_target_speed = fmax(
      max_adc_stop_speed,
      fmin(
          cruise_speed,
          fmin(GetMinTimeSpeedLimit(
                   0, reference_line_info.st_graph_data().total_time_by_conf()),
               GetMinPositionSpeedLimit(0, s))));

  if (cruise_target_speed_filter_ == nullptr) {
    return;
  }

  if (fabs(cruise_target_speed_ - cruise_target_speed) > 0.5) {
    cruise_target_speed_filter_->Reset();
  }

  const auto current_cruise_target_speed =
      cruise_target_speed_filter_->Update(cruise_target_speed);
  if (DefinitelyGreaterEqual(
          fabs(current_cruise_target_speed - cruise_target_speed_), 0.025) ||
      DefinitelyGreaterEqual(current_cruise_target_speed, cruise_speed)) {
    cruise_target_speed_ = current_cruise_target_speed;
  }
}

void SpeedLimitCache::ModifySpeedLimits(const double cruise_target_speed) {
  for (auto& position_speed_limit : position_speed_limits_) {
    position_speed_limit.curvature_speed_limit =
        fmax(position_speed_limit.curvature_speed_limit, cruise_target_speed);
    position_speed_limit.map_speed_limit =
        fmax(position_speed_limit.map_speed_limit, cruise_target_speed);
    position_speed_limit.decision_speed_limit =
        fmax(position_speed_limit.decision_speed_limit, cruise_target_speed);
    position_speed_limit.speed_limit =
        fmin(fmin(position_speed_limit.curvature_speed_limit,
                  position_speed_limit.map_speed_limit),
             position_speed_limit.decision_speed_limit);
  }

  for (auto& time_speed_limit : time_speed_limits_) {
    time_speed_limit.cruise_speed_limit =
        fmax(time_speed_limit.cruise_speed_limit, cruise_target_speed);
    time_speed_limit.comfortable_speed_limit =
        fmax(time_speed_limit.comfortable_speed_limit, cruise_target_speed);
    time_speed_limit.critical_speed_limit =
        fmax(time_speed_limit.critical_speed_limit, cruise_target_speed);
  }
}

void SpeedLimitCache::AddToKeepClearRange(
    const std::vector<const Obstacle*>& obstacles) {
  for (const auto& obstacle : obstacles) {
    if (obstacle->path_st_boundary().IsEmpty()) {
      continue;
    }
    if (obstacle->path_st_boundary().boundary_type() !=
        STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    double start_s = obstacle->path_st_boundary().min_s();
    double end_s = obstacle->path_st_boundary().max_s();
    keep_clear_range_.emplace_back(start_s, end_s);
    ADEBUG << FIXED << SETPRECISION(5) << "keep_clear_range start_s:" << start_s
           << ", end_s:" << end_s;
  }
  SortAndMergeRange(&keep_clear_range_);
}

bool SpeedLimitCache::InKeepClearRange(double s) const {
  return std::any_of(
      keep_clear_range_.begin(), keep_clear_range_.end(),
      [&](const auto& p) { return (p.first <= s && p.second >= s); });
}

void SpeedLimitCache::SortAndMergeRange(
    std::vector<std::pair<double, double>>* keep_clear_range) {
  if ((keep_clear_range == nullptr) || keep_clear_range->empty()) {
    return;
  }
  std::sort(keep_clear_range->begin(), keep_clear_range->end());
  size_t i = 0;
  size_t j = i + 1;
  while (j < keep_clear_range->size()) {
    if (keep_clear_range->at(i).second < keep_clear_range->at(j).first) {
      ++i;
      ++j;
    } else {
      keep_clear_range->at(i).second = std::max(keep_clear_range->at(i).second,
                                                keep_clear_range->at(j).second);
      ++j;
    }
  }
  keep_clear_range->resize(i + 1);
}

void SpeedLimitCache::CachePedestrianSpeedLimit(
    const SpeedCacheConfig& config, const common::TrajectoryPoint& init_point) {

  auto target_speed = FLAGS_planning_upper_speed_limit;
  for (int i = 0; i < s_count_; ++i) {
    target_speed =
        fmin(target_speed, position_speed_limits_.at(i).pedestrian_speed_limit);
  }

  std::vector<std::vector<double>> vec_vec_state;
  if (DefinitelyGreaterEqual(target_speed, init_point.v()) ||
      !TL::planning::util::GetStateAtMinJerk(
          init_point.v(), init_point.a(), target_speed,
          FLAGS_planning_upper_speed_limit,
          config.dece_when_exceed_pedestrian_speed_limit(),
          config.jerk_when_exceed_pedestrian_speed_limit(), 100.0, 0.1, 0.0,
          &vec_vec_state) ||
      vec_vec_state.size() < 4) {
    return;
  }

  const auto& s_state = vec_vec_state.at(1);
  const auto& v_state = vec_vec_state.at(2);
  const auto& a_state = vec_vec_state.at(3);
  const auto state_count =
      std::min({s_state.size(), v_state.size(), a_state.size()});
  if (state_count <= 0) {
    return;
  }

  std::size_t next_point_index = 1;
  for (int i = 0; i < s_count_; ++i) {
    const auto s = current_start_s_ + i * current_unit_s_;
    // calculate s_lower / s_upper / l_lower / l_upper
    while (next_point_index + 1 < s_state.size() &&
           s_state.at(next_point_index) < s) {
      ++next_point_index;
    }
    const auto prev_point_index =
        next_point_index == 0 ? 0 : next_point_index - 1;
    const auto& prev_s = s_state.at(prev_point_index);
    const auto& next_s = s_state.at(next_point_index);

    auto ds = next_s - prev_s;
    const auto ratio =
        common::math::double_type::IsZero(ds) ? 0.0 : (s - prev_s) / ds;

    const auto v = (1 - ratio) * v_state.at(prev_point_index) +
                   ratio * v_state.at(next_point_index);
    ADEBUG << "v:" << v << ", pedestrian_speed_limit:"
           << position_speed_limits_.at(i).pedestrian_speed_limit;
    position_speed_limits_.at(i).pedestrian_speed_limit =
        fmax(position_speed_limits_.at(i).pedestrian_speed_limit, v);
  }
}

void SpeedLimitCache::CacheLowSpeedThreshold(
    const common::TrajectoryPoint& init_point,
    const ReferenceLineInfo& reference_line_info) {
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto max_acceleration = vehicle_param.max_acceleration();
  const auto max_deceleration = vehicle_param.max_deceleration();
  const auto cruise_speed = reference_line_info.GetCruiseSpeed();

  auto v = init_point.v();
  auto s = 0.0;
  low_speed_thresholds_.emplace_back(v);
  for (int i = 1; i < time_count_; ++i) {
    const auto time_speed_limit = GetTimeSpeedLimit(i * time_unit_);
    const auto max_v = fmin(v + max_acceleration * time_unit_,
                            time_speed_limit.critical_speed_limit);
    const auto min_v = fmax(v + max_deceleration * time_unit_, 0.0);
    auto next_v = max_v;
    auto next_s = s;
    const double v_unit = 0.1;
    int v_count = static_cast<int>((next_v - min_v) / v_unit);
    for (int j = 0; j <= v_count; ++j) {
      next_s = s + 0.5 * (v + next_v) * time_unit_;
      const auto position_speed_limit = GetPositionSpeedLimit(next_s);
      if (next_v < position_speed_limit.speed_limit && next_v < cruise_speed) {
        break;
      }
      next_v -= v_unit;
    }
    v = next_v;
    s = next_s;
    low_speed_thresholds_.emplace_back(v);
  }
}

std::pair<bool, std::vector<double>> SpeedLimitCache::ModifyFrontLaneSpeedLimit(
    const common::TrajectoryPoint& init_point,
    const hdmap::LaneRangeInfo& front_lane, const double comfort_decel,
    const double comfort_jerk,
    const double speed_limit_point_distance_to_frone_lane,
    hdmap::LaneRangeInfo* last_lane_info, bool* deceleration_for_lane) {
  std::vector<double> front_lane_speeds;
  auto valid = false;
  if (last_lane_info == nullptr || deceleration_for_lane == nullptr ||
      front_lane.start_lane == nullptr) {
    return {valid, front_lane_speeds};
  }
  auto front_lane_speed = front_lane.speed_limit;
  if (DefinitelyLessEqual(front_lane.start_s, 0.0) &&
      !position_speed_limits_.empty() &&
      position_speed_limits_.front().allow_over_speed) {
    front_lane_speed =
        fmax(front_lane_speed, position_speed_limits_.front().map_speed_limit);
  }

  if (DefinitelyLessEqual(init_point.v(), front_lane_speed)) {
    *last_lane_info = front_lane;
    *deceleration_for_lane = false;
    return {valid, front_lane_speeds};
  }

  auto front_lane_speed_limit_start_s = front_lane.start_s;
  front_lane_speed_limit_start_s -= speed_limit_point_distance_to_frone_lane;
  front_lane_speed_limit_start_s = fmax(front_lane_speed_limit_start_s, 0.0);

  if (front_lane.start_lane == last_lane_info->start_lane &&
      *deceleration_for_lane) {
    front_lane_speed_limit_start_s = 0.0;
  }

  if (!CalculateComfortDecelSpeedLimits(
          init_point, front_lane_speed, front_lane_speed_limit_start_s,
          comfort_decel, comfort_jerk, 100.0, &front_lane_speeds) ||
      static_cast<int>(front_lane_speeds.size()) < time_count_ ||
      static_cast<int>(time_speed_limits_.size()) < time_count_) {
    return {valid, front_lane_speeds};
  }
  valid = true;
  *deceleration_for_lane = true;
  *last_lane_info = front_lane;
  return {valid, front_lane_speeds};
}

void SpeedLimitCache::CacheNudgeSpeedLimit(
    const SpeedCacheConfig& config,
    const std::vector<const SLTObstacleCache*>& slt_obstacle_caches,
    const Frame& frame, const common::TrajectoryPoint& init_point,
    const ReferenceLineInfo& reference_line_info) {
  UNUSED(config);
  UNUSED(init_point);
  std::set<int> used_perception_ids;
  for (const auto* slt_obstacle_cache : slt_obstacle_caches) {
    if (slt_obstacle_cache == nullptr ||
        slt_obstacle_cache->GetObstacle() == nullptr) {
      continue;
    }
    const auto& obstacle_info = slt_obstacle_cache->GetObstacleInfoAtTime(0);
    const auto perception_id =
        slt_obstacle_cache->GetObstacle()->PerceptionId();
    if (used_perception_ids.find(perception_id) != used_perception_ids.end()) {
      continue;
    }
    used_perception_ids.insert(perception_id);

    auto& obstacle_history = obstacle_history_info_map_[perception_id];
    obstacle_history.slt_obstacle_cache = slt_obstacle_cache;
    if (obstacle_history.historical_trajectory.size() >= kObstacleHistorySize) {
      obstacle_history.historical_trajectory.pop_front();
    }
    obstacle_history.historical_trajectory.push_back(obstacle_info);
  }
  std::set<int> not_can_use_ids;

  for (const auto& obstacle_history : obstacle_history_info_map_) {
    if (used_perception_ids.find(obstacle_history.first) ==
        used_perception_ids.end()) {
      not_can_use_ids.insert(obstacle_history.first);
    }
  }
  for (const auto& not_can_use_id : not_can_use_ids) {
    obstacle_history_info_map_.erase(not_can_use_id);
  }

  for (auto& one_obstacle_history : obstacle_history_info_map_) {
    double obstacle_sum_l_lower = 0;
    double obstacle_sum_l_upper = 0;
    double obstacle_sum_ds = 0;

    for (const auto& obstacle_history :
         one_obstacle_history.second.historical_trajectory) {
      obstacle_sum_l_lower += obstacle_history.l_lower;
      obstacle_sum_l_upper += obstacle_history.l_upper;
      obstacle_sum_ds += obstacle_history.ds;
    }

    if (one_obstacle_history.second.historical_trajectory.size() ==
        kObstacleHistorySize) {
      one_obstacle_history.second.obstacle_average_l_upper =
          obstacle_sum_l_upper / static_cast<double>(kObstacleHistorySize);
      one_obstacle_history.second.obstacle_average_l_lower =
          obstacle_sum_l_lower / static_cast<double>(kObstacleHistorySize);
      one_obstacle_history.second.obstacle_average_ds =
          obstacle_sum_ds / static_cast<double>(kObstacleHistorySize);

      // AERROR << "id" << perception_id
      //        << ", setof_obstacle_history_[perception_id].size();  "
      //        << setof_obstacle_history_[perception_id]
      //               .continuous_historical_trajectory.size();
    }
  }

  // for (std::size_t j = 0; j < slt_obstacle_caches.size(); ++j) {
  //   AERROR << " ,[slt_obstacle_caches]:  "
  //          << slt_obstacle_caches[j]->GetObstacle()->Id();
  // }
  // const double adc_v = init_point.v();
  const auto adc_l_upper = 0.5 * vehicle_param_.width();
  const auto adc_l_lower = -adc_l_upper;
  for (int i = 0; i < time_count_; ++i) {
    const auto& t = i * time_unit_;

    for (int j = 0; j < s_count_; ++j) {
      auto& nudge_speed_limit = nudge_speed_limits_.at(i).at(j);
      nudge_speed_limit = FLAGS_planning_upper_speed_limit;

      const auto s = j * current_unit_s_;
      const auto adc_s_lower = s - vehicle_param_.back_edge_to_center();
      const auto adc_s_upper = s + vehicle_param_.front_edge_to_center();

      for (const auto& obstacle_history : obstacle_history_info_map_) {
        if (TL::common::math::double_type::DefinitelyLess(
                obstacle_history.second.obstacle_average_ds, 0)) {
          break;
        }
        if (obstacle_history.second.historical_trajectory.size() <
                kObstacleHistorySize ||
            obstacle_history.second.slt_obstacle_cache == nullptr ||
            t < obstacle_history.second.slt_obstacle_cache->GetMinT() ||
            t > obstacle_history.second.slt_obstacle_cache->GetMaxT()) {
          continue;
        }

        const auto& obstacle_info =
            obstacle_history.second.slt_obstacle_cache->GetObstacleInfoAtTime(
                t);
        if (adc_s_lower > obstacle_info.s_upper ||
            adc_s_upper < obstacle_info.s_lower) {
          continue;
        }
        const double bigvehicle_limit_buffer_coef =
            obstacle_history.second.slt_obstacle_cache->GetObstacle()
                    ->IsOversizedVehicle()
                ? 0.7
                : 1;
        double entering_the_lane_coef = 1;
        const auto is_avp_mode =
            frame.local_view().HasFunctionManagerIn() &&
            frame.local_view().GetFunctionManagerIn()->ta_pilot_mode() ==
                TL::functionmanager::TaPilotMode::AVP;
        double start_s_lane_left_width = 0.0;
        double start_s_lane_right_width = 0.0;
        reference_line_info.reference_line().GetLaneWidth(
            obstacle_info.s_upper, &start_s_lane_left_width,
            &start_s_lane_right_width);
        bool is_entering_the_lane =
            obstacle_history.second.obstacle_average_l_upper >
                -start_s_lane_right_width &&
            obstacle_history.second.obstacle_average_l_lower <
                start_s_lane_left_width;
        if (!is_avp_mode && is_entering_the_lane) {
          const auto entering_width =
              fmin(fabs(obstacle_history.second.obstacle_average_l_upper +
                        start_s_lane_right_width),
                   fabs(start_s_lane_left_width -
                        obstacle_history.second.obstacle_average_l_lower));
          const auto per_entering_width = fmin(entering_width / 0.5, 1);
          entering_the_lane_coef = 1 - per_entering_width;
        }

        auto l_distance = std::numeric_limits<double>::max();
        if (adc_l_lower > obstacle_history.second.obstacle_average_l_upper) {
          l_distance =
              adc_l_lower - obstacle_history.second.obstacle_average_l_upper;
        } else if (obstacle_history.second.obstacle_average_l_lower >
                   adc_l_upper) {
          l_distance =
              obstacle_history.second.obstacle_average_l_lower - adc_l_upper;
        }

        const double nudge_speed_limit_coef = common::math::InterpolationOne(
            obstacle_history.second.obstacle_average_ds, speeds_,
            nudge_speed_limit_coefs_);
        nudge_speed_limit =
            fmin(nudge_speed_limit,
                 (obstacle_history.second.obstacle_average_ds +
                  fabs(l_distance) * nudge_speed_limit_coef *
                      bigvehicle_limit_buffer_coef * entering_the_lane_coef));
        //   AERROR << "nudge_speed_limit" << nudge_speed_limit;
        //   AERROR << "obstacle_history.second.obstacle_average_ds"
        //          << obstacle_history.second.obstacle_average_ds;
        //   AERROR << "l_distance" << l_distance;
        //   AERROR << "nudge_speed_limit_coef" << nudge_speed_limit_coef;
        //   AERROR << "bigvehicle_limit_buffer_coef"
        //          << bigvehicle_limit_buffer_coef;
        //   AERROR << "entering_the_lane_coef" << entering_the_lane_coef;
      }
    }
  }
}

void SpeedLimitCache::CacheCriticalSpeedLimit(
    const SpeedCacheConfig& config, const common::TrajectoryPoint& init_point,
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  const auto& local_view = frame.local_view();
  CacheRampSpeedLimit(config, frame, init_point, reference_line_info);
  CacheTunnelSpeedLimit(config, frame, init_point, reference_line_info);
  CacheConeSpeedLimit(config, reference_line_info, init_point);
  CacheTollhouseSpeedLimit(config, frame, init_point);
  if (local_view.HasFunctionManagerIn()) {
    const auto& fct_in = local_view.GetFunctionManagerIn();
    const auto cdcs_speed_limit =
        fct_in->fct_nnp_in().cdcs_info().cdcs_speed_limit();
    const auto has_camera_speed_limit =
        cdcs_speed_limit.camera_speed_limit_km() >=
            (adc_in_tunnel_ ? 60 : 80) &&
        cdcs_speed_limit.camera_distance() > 0 &&
        !cdcs_speed_limit.camera_distance_stuck();
    std::vector<double> front_lane_speeds;
    const auto camera_speed_ms = SpeedConventor::ConvertDisplaySpdToReal(
        static_cast<int>(cdcs_speed_limit.camera_speed_limit_km()));
    if (has_camera_speed_limit &&
        CalculateComfortDecelSpeedLimits(
            init_point, camera_speed_ms,
            fmax(cdcs_speed_limit.camera_distance() -
                     config.camera_preview_distance(),
                 0.0),
            config.dece_when_exceed_camera_speed_limit(),
            config.jerk_when_exceed_camera_speed_limit(), 20.0,
            &front_lane_speeds) &&
        static_cast<int>(front_lane_speeds.size()) >= time_count_ &&
        static_cast<int>(time_speed_limits_.size()) >= time_count_) {
      for (int i = 0; i < time_count_; ++i) {
        time_speed_limits_.at(i).critical_speed_limit =
            fmin(time_speed_limits_.at(i).critical_speed_limit,
                 front_lane_speeds.at(i));
      }
    }
  }
}

void SpeedLimitCache::CacheComfortableSpeedLimit(
    const SpeedCacheConfig& config, const common::TrajectoryPoint& init_point,
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  UNUSED(frame);
  CacheCruiseSpeedLimits(config, reference_line_info, init_point);
  for (auto& time_speed_limit : time_speed_limits_) {
    time_speed_limit.comfortable_speed_limit = fmin(
        time_speed_limit.cruise_speed_limit, time_speed_limit.map_speed_limit);
  }
}

void SpeedLimitCache::CacheRampSpeedLimit(
    const SpeedCacheConfig& config, const Frame& frame,
    const common::TrajectoryPoint& init_point,
    const ReferenceLineInfo& reference_line_info) {
  const auto* reference_line_provider = frame.GetReferenceLineProvider();
  if (reference_line_provider == nullptr) {
    return;
  }

  const auto& pnc_map = reference_line_provider->GetPncMap();
  if (pnc_map == nullptr) {
    return;
  }
  const auto adc_in_ramp = pnc_map->GetADCWaypoint().lane != nullptr &&
                           pnc_map->GetADCWaypoint().lane->IsRampRoad();
  if (adc_in_ramp) {
    return;
  }
  const auto& ramp_info = reference_line_info.GetFrontRamp();
  if (ramp_info.start_lane == nullptr) {
    return;
  }
  const auto ramp_speed_limit_points = ModifyFrontLaneSpeedLimit(
      init_point, ramp_info, config.dece_when_exceed_ramp_speed_limit(),
      config.jerk_when_exceed_ramp_speed_limit(), 0.0, &last_ramp_info_,
      &deceleration_for_ramp_);

  if (ramp_speed_limit_points.first) {
    for (int i = 0; i < time_count_; ++i) {
      time_speed_limits_.at(i).critical_speed_limit =
          fmin(time_speed_limits_.at(i).critical_speed_limit,
               ramp_speed_limit_points.second.at(i));
    }
  }
}

void SpeedLimitCache::CacheTunnelSpeedLimit(
    const SpeedCacheConfig& config, const Frame& frame,
    const common::TrajectoryPoint& init_point,
    const ReferenceLineInfo& reference_line_info) {
  const auto* reference_line_provider = frame.GetReferenceLineProvider();
  if (reference_line_provider == nullptr) {
    return;
  }

  const auto& pnc_map = reference_line_provider->GetPncMap();
  if (pnc_map == nullptr) {
    return;
  }
  adc_in_tunnel_ =
      pnc_map->GetADCWaypoint().lane != nullptr &&
      pnc_map->GetADCWaypoint().lane->lane().map_lane_type().tunnel_lane();
  if (adc_in_tunnel_) {
    return;
  }
  const auto& tunnel_info = reference_line_info.GetFrontTunnel();
  if (tunnel_info.start_lane == nullptr) {
    return;
  }
  const auto tunnel_speed_limit_points = ModifyFrontLaneSpeedLimit(
      init_point, tunnel_info, config.dece_when_exceed_tunnel_speed_limit(),
      config.jerk_when_exceed_tunnel_speed_limit(),
      config.speed_limit_point_distance_to_tunnel(), &last_tunnel_info_,
      &deceleration_for_tunnel_);

  if (tunnel_speed_limit_points.first) {
    for (int i = 0; i < time_count_; ++i) {
      time_speed_limits_.at(i).critical_speed_limit =
          fmin(time_speed_limits_.at(i).critical_speed_limit,
               tunnel_speed_limit_points.second.at(i));
    }
  }
}

void SpeedLimitCache::CacheConeSpeedLimit(
    const SpeedCacheConfig& config,
    const ReferenceLineInfo& reference_line_info,
    const common::TrajectoryPoint& init_point) {

  auto current_cone_speed_limit = std::numeric_limits<double>::max();
  std::vector<std::vector<double>> cone_speed_limit_sequences;

  for (const auto& speed_limit_info_point : reference_line_info.st_graph_data()
                                                .speed_limit()
                                                .speed_limit_info_points()) {
    if (SeemsEqual(current_cone_speed_limit,
                   speed_limit_info_point.second.cone_speed_limit)) {
      continue;
    }
    current_cone_speed_limit = speed_limit_info_point.second.cone_speed_limit;

    if (init_point.v() < current_cone_speed_limit) {
      continue;
    }

    std::vector<double> cone_speed_limits;
    if (CalculateComfortDecelSpeedLimits(
            init_point, current_cone_speed_limit, speed_limit_info_point.first,
            config.dece_when_exceed_cone_speed_limit(),
            config.jerk_when_exceed_cone_speed_limit(), 100.0,
            &cone_speed_limits)) {
      cone_speed_limit_sequences.emplace_back(std::move(cone_speed_limits));
    }
  }

  for (int i = 0; i < time_count_; ++i) {
    auto min_map_speed_limit = std::numeric_limits<double>::infinity();
    for (const auto& cone_speed_limit_sequence : cone_speed_limit_sequences) {
      if (cone_speed_limit_sequence.empty()) {
        continue;
      }
      if (i < static_cast<int>(cone_speed_limit_sequence.size())) {
        min_map_speed_limit =
            fmin(min_map_speed_limit, cone_speed_limit_sequence.at(i));
      } else {
        min_map_speed_limit =
            fmin(min_map_speed_limit, cone_speed_limit_sequence.back());
      }
    }

    time_speed_limits_.at(i).critical_speed_limit = fmin(
        time_speed_limits_.at(i).critical_speed_limit, min_map_speed_limit);
  }
}

void SpeedLimitCache::CacheTollhouseSpeedLimit(
    const SpeedCacheConfig& config, const Frame& frame,
    const common::TrajectoryPoint& init_point) {
  const auto& local_view = frame.local_view();
  if (!local_view.HasFunctionManagerOut() ||
      local_view.GetFunctionManagerOut() == nullptr) {
    return;
  }
  const auto& odd_info = local_view.GetFunctionManagerOut()->odd_info();
  // 收费站前50m要降速到60
  if (odd_info.type() != routing::LaneWaypointType::ODD_START ||
      odd_info.odd_type() != routing::LaneWaypoint::SPECIAL_AREA) {
    return;
  }

  std::vector<double> tollhouse_speeds;
  const auto speed_limit_start_s =
      fmax(0.0, odd_info.to_end_len() -
                    config.speed_limit_point_distance_to_tollhouse());
  auto tollhouse_speed_ms = SpeedConventor::ConvertDisplaySpdToReal(
      config.tollhouse_speed_limit_km());
  if (DefinitelyLessEqual(init_point.v(), tollhouse_speed_ms)) {
    return;
  }
  if (!CalculateComfortDecelSpeedLimits(
          init_point, tollhouse_speed_ms, speed_limit_start_s,
          config.dece_when_exceed_tollhouse_speed_limit(),
          config.jerk_when_exceed_tollhouse_speed_limit(), 100.0,
          &tollhouse_speeds) ||
      static_cast<int>(tollhouse_speeds.size()) < time_count_ ||
      static_cast<int>(time_speed_limits_.size()) < time_count_) {
    return;
  }
  for (int i = 0; i < time_count_; ++i) {
    time_speed_limits_.at(i).critical_speed_limit = fmin(
        time_speed_limits_.at(i).critical_speed_limit, tollhouse_speeds.at(i));
  }
}

}  // namespace planning
}  // namespace TL
