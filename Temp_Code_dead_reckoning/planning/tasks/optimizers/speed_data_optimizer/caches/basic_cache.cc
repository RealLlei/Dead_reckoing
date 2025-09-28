/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_optimizer.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/caches/basic_cache.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "common/file/log.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "planning/common/obstacle.h"
#include "planning/common/path/frenet_frame_path.h"
#include "planning/common/reference_line_info.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {
namespace speed_evaluator {

void BasicCache::Init(const SpeedCacheConfig& config,
                      const std::shared_ptr<DependencyInjector>& injector,
                      const Frame& frame,
                      const ReferenceLineInfo& reference_line_info,
                      const common::TrajectoryPoint& init_point) {
  if (stop_average_accel_thresholds_.first.empty() &&
      stop_average_accel_thresholds_.second.empty()) {
    auto& speeds = stop_average_accel_thresholds_.first;
    auto& accels = stop_average_accel_thresholds_.second;
    const auto& calibration_infos =
        config.stop_average_accel_threshold_calibration_info();
    if (calibration_infos.empty()) {
      speeds = {0.0, 40.0};
      accels = {-0.25, -0.25};
    } else {
      speeds.reserve(calibration_infos.size());
      accels.reserve(calibration_infos.size());
      for (const auto& calibration_info : calibration_infos) {
        speeds.emplace_back(calibration_info.speed());
        accels.emplace_back(calibration_info.accel());
      }
    }
  }

  CalculateInfoFromLastFrame(injector, frame);

  CalculateExpectedStopS(config, frame, reference_line_info, init_point);

  CalculateRoadRightRange(reference_line_info);

  is_forward_path_ =
      reference_line_info.path_data().frenet_frame_path().is_forward_path();

  ADEBUG << "follow_time:" << reference_line_info.GetLonCtrlTime();

  const auto& obstacles =
      reference_line_info.path_decision().obstacles().Items();
  has_pedestrian_ =
      std::any_of(obstacles.begin(), obstacles.end(), [](const auto* obstacle) {
        return obstacle != nullptr &&
               obstacle->Perception().type() ==
                   perception::PerceptionObstacle::PEDESTRIAN;
      });
}

void BasicCache::CalculateInfoFromLastFrame(
    const std::shared_ptr<DependencyInjector>& injector, const Frame& frame) {
  last_frame_ = nullptr;
  time_from_last_frame_ = 0.0;
  distance_from_last_frame_ = 0.0;
  if (injector == nullptr || injector->frame_history() == nullptr) {
    return;
  }

  last_frame_ = injector->frame_history()->Latest();
  if (last_frame_ == nullptr) {
    return;
  }

  const auto& planning_start_point = frame.PlanningStartPoint();
  const auto& last_planning_start_point = last_frame_->PlanningStartPoint();

  time_from_last_frame_ = (frame.vehicle_state().timestamp() +
                           planning_start_point.relative_time()) -
                          (last_frame_->vehicle_state().timestamp() +
                           last_planning_start_point.relative_time());
  distance_from_last_frame_ = fmax(
      0.0, 0.5 * (planning_start_point.v() + last_planning_start_point.v()) *
               time_from_last_frame_);
}

void BasicCache::CalculateExpectedStopS(
    const SpeedCacheConfig& config, const Frame& frame,
    const ReferenceLineInfo& reference_line_info,
    const common::TrajectoryPoint& init_point) {
  // go through every obstacle, calculate block_obstacle_id and stop_s
  last_frame_block_obstacle_id_ = block_obstacle_id_;
  block_obstacle_id_.clear();
  auto stop_s = std::numeric_limits<double>::infinity();
  nearest_stop_s_ = 0.0;
  farthest_stop_s_ = std::numeric_limits<double>::infinity();
  const Obstacle* block_obstacle_ptr = nullptr;
  bool is_avp_mode = false;
  if (frame.local_view().HasFunctionManagerIn()) {
    is_avp_mode = functionmanager::AVP ==
                  frame.local_view().GetFunctionManagerIn()->ta_pilot_mode();
  }

  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  for (const auto& obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (obstacle == nullptr || obstacle->path_st_boundary().IsEmpty()) {
      continue;
    }

    if (!obstacle->LongitudinalDecision().has_stop() &&
        (obstacle->path_st_boundary().boundary_type() !=
             STBoundary::BoundaryType::KEEP_CLEAR ||
         obstacle->LongitudinalDecision().has_ignore())) {
      continue;
    }

    auto path_st_boundary_min_s = obstacle->path_st_boundary().min_s();
    const auto xy_distance =
        obstacle->PerceptionBoundingBox().DistanceTo(common::math::Vec2d{
            init_point.path_point().x(), init_point.path_point().y()}) +
        obstacle->LongitudinalDecision().stop().distance_s() -
        vehicle_param.front_edge_to_center();
    path_st_boundary_min_s = fmax(path_st_boundary_min_s, xy_distance);

    if (path_st_boundary_min_s > config.stop_prefinish_distance_threshold()) {
      path_st_boundary_min_s = config.stop_prefinish_distance_threshold() +
                               (path_st_boundary_min_s -
                                config.stop_prefinish_distance_threshold()) *
                                   config.stop_distance_backward_coef();
    }

    if (path_st_boundary_min_s < stop_s) {
      block_obstacle_id_ = obstacle->Id();
      stop_s = path_st_boundary_min_s;
      block_obstacle_ptr = obstacle;
    }
  }

  if (block_obstacle_ptr != nullptr) {
    nearest_stop_s_ =
        fmax(nearest_stop_s_, stop_s - config.max_stop_distance());
    double farthest_stop_s_tmp =
        stop_s +
        fmax(-block_obstacle_ptr->LongitudinalDecision().stop().distance_s() -
                 config.min_stop_distance(),
             1e-6);
    farthest_stop_s_ =
        is_avp_mode ? stop_s
                    : fmax(0.0, fmin(farthest_stop_s_, farthest_stop_s_tmp));
    stop_s = fmax(0.0, stop_s);
    nearest_stop_s_ = fmax(0.0, nearest_stop_s_);
    farthest_stop_s_ = fmax(0.0, farthest_stop_s_);
  }

  ADEBUG << FIXED << SETPRECISION(3) << "stop_s:" << stop_s
         << ", last_frame_block_obstacle_id:" << last_frame_block_obstacle_id_
         << ", block_obstacle_id:" << block_obstacle_id_;

  // check if first enter stop
  const auto is_first_stop =
      (!std::isinf(stop_s) && std::isinf(expected_stop_s_)) ||
      (!block_obstacle_id_.empty() &&
       last_frame_block_obstacle_id_ != block_obstacle_id_);

  // if first enter stop, calculate stop buffer
  if (is_first_stop) {
    CalculateStopBuffer(init_point, stop_s, is_avp_mode, config);
  }

  // calculate real_stop_s_
  real_stop_s_ = common::math::Clamp(stop_s + stop_s_buffer_, nearest_stop_s_,
                                     farthest_stop_s_);

  // check if stop position is far away from vehicle
  if ((is_first_stop || (frame.vehicle_state().driving_mode() !=
                             soc::Chassis::COMPLETE_AUTO_DRIVE &&
                         frame.vehicle_state().driving_mode() !=
                             soc::Chassis::AUTO_SPEED_ONLY)) &&
      CheckIfStopPostionFarAwayFromVehicle(frame, init_point)) {
    expected_stop_s_ = std::numeric_limits<double>::infinity();
  } else {
    expected_stop_s_ = real_stop_s_;
  }

  if (is_first_stop && !std::isinf(expected_stop_s_)) {
    stop_slide_time_threshold_ = fmax(0.0, 5.0 - init_point.v() * 0.25);
  }

  if (block_obstacle_id_.empty() ||
      last_frame_block_obstacle_id_ != block_obstacle_id_ ||
      expected_stop_s_ > config.stop_prefinish_distance_threshold()) {
    is_stop_prefinish_ = false;
  } else if (!block_obstacle_id_.empty() &&
             expected_stop_s_ < config.stop_prefinish_distance_threshold()) {
    is_stop_prefinish_ = true;
  }

  ADEBUG << "nearest_stop_s:" << nearest_stop_s_
         << ", faarthest_stop_s:" << farthest_stop_s_
         << ", real_stop_s:" << real_stop_s_
         << ", expected_stop_s:" << expected_stop_s_
         << ", is_stop_prefinish:" << is_stop_prefinish_;
}

void BasicCache::CalculateStopBuffer(const common::TrajectoryPoint& init_point,
                                     const double stop_s,
                                     const bool is_avp_mode,
                                     const SpeedCacheConfig& config) {
  stop_s_buffer_ = 0.0;
  if (is_avp_mode) {
    return;
  }
  const auto endv_dec_distance = pow(config.stop_curve_end_speed(), 2) /
                                 fabs(2.0 * config.stop_curve_end_accel());
  const auto temp = pow(init_point.v(), 2) / 2.0;
  const auto stop_a = temp / fmax(stop_s - endv_dec_distance, 0.001);
  const auto farthest_stop_a =
      temp / fmax(farthest_stop_s_ - endv_dec_distance, 0.001);
  if (stop_a - farthest_stop_a > 0.25 && stop_a > 1.0) {
    const auto expected_stop_a =
        common::math::Clamp(farthest_stop_a, 0.3, stop_a);
    stop_s_buffer_ =
        common::math::Clamp(pow(init_point.v(), 2) / (2.0 * expected_stop_a),
                            stop_s, farthest_stop_s_ - 1e-5) -
        stop_s;
    ADEBUG << "stop_a:" << stop_a << ", farthest_stop_a:" << farthest_stop_a
           << ", expected_stop_a:" << expected_stop_a
           << ", stop_s_buffer:" << stop_s_buffer_;
  }
}

bool BasicCache::CheckIfStopPostionFarAwayFromVehicle(
    const Frame& frame, const common::TrajectoryPoint& init_point) const {
  UNUSED(frame);
  auto max_stop_s = 0.0;
  if (init_point.a() > 0.0) {
    constexpr auto j = -0.5;
    auto t = fabs(init_point.a() / j);
    auto v = init_point.v() + init_point.a() * t + j * t * t / 2.0;
    max_stop_s =
        init_point.v() * t + init_point.a() * t * t / 2.0 + j * t * t * t / 6;
    max_stop_s += v * v /
                  (2.0 * fabs(common::math::InterpolationOne(
                             v, stop_average_accel_thresholds_.first,
                             stop_average_accel_thresholds_.second)));
  } else {
    max_stop_s =
        pow(init_point.v(), 2) /
        (2.0 * fabs(common::math::InterpolationOne(
                   init_point.v(), stop_average_accel_thresholds_.first,
                   stop_average_accel_thresholds_.second)));
  }

  ADEBUG << "v:" << init_point.v() << ", a:" << init_point.a()
         << ", max_stop_s:" << max_stop_s << ", real_stop_s_:" << real_stop_s_;

  return max_stop_s < real_stop_s_;
}

void BasicCache::CalculateRoadRightRange(
    const ReferenceLineInfo& reference_line_info) {

  high_road_right_end_s_ = std::numeric_limits<double>::max();
  low_road_right_end_s_ = std::numeric_limits<double>::lowest();
  has_low_road_right_ = false;

  const auto& path_road_right =
      reference_line_info.path_data().GetPathRoadRight();
  const auto& discretized_path =
      reference_line_info.path_data().discretized_path();
  if (path_road_right.empty() || discretized_path.empty() ||
      path_road_right.size() != discretized_path.size()) {
    return;
  }

  int point_count = static_cast<int>(discretized_path.size());
  int high_road_right_end_index = -1;
  int low_road_right_end_index = -1;
  for (int i = 0; i < point_count; ++i) {
    if (path_road_right.at(i) != PathData::RoadRightType::HIGH_ROAD_RIGHT) {
      has_low_road_right_ = true;
      low_road_right_end_index = i;
    }

    if (!has_low_road_right_) {
      high_road_right_end_index = i;
    }
  }

  if (high_road_right_end_index < 0) {
    high_road_right_end_s_ = std::numeric_limits<double>::lowest();
  } else if (high_road_right_end_index < point_count - 1) {
    high_road_right_end_s_ = discretized_path.at(high_road_right_end_index).s();
  } else {
    high_road_right_end_s_ = std::numeric_limits<double>::max();
  }

  if (low_road_right_end_index < 0) {
    low_road_right_end_s_ = std::numeric_limits<double>::lowest();
  } else if (low_road_right_end_index < point_count - 1) {
    low_road_right_end_s_ = discretized_path.at(low_road_right_end_index).s();
  } else {
    low_road_right_end_s_ = std::numeric_limits<double>::max();
  }
}

void BasicCache::SetAccelLimitTable(const std::vector<double>& accel_speeds,
                                    const std::vector<double>& accels,
                                    const std::vector<double>& decel_speeds,
                                    const std::vector<double>& decels) {
  for (int i = 0; i < static_cast<int>(accel_limit_table_.size()); ++i) {
    const double v = i * speed_epsilon_;
    accel_limit_table_.at(i).first =
        common::math::InterpolationOne(v, decel_speeds, decels);
    accel_limit_table_.at(i).second =
        common::math::InterpolationOne(v, accel_speeds, accels);
  }

  accel_limit_speeds_ = accel_speeds;
  accel_limits_ = accels;
  decel_limit_speeds_ = decel_speeds;
  decel_limits_ = decels;
}

void BasicCache::SetJerkLimitTable(const std::vector<double>& speeds,
                                   const std::vector<double>& jerk) {
  for (int i = 0; i < static_cast<int>(jerk_limit_table_.size()); ++i) {
    const double v = i * speed_epsilon_;
    jerk_limit_table_.at(i).first =
        common::math::InterpolationOne(v, speeds, jerk) * (-1.0);
    jerk_limit_table_.at(i).second =
        common::math::InterpolationOne(v, speeds, jerk);
  }
  jerk_limit_speeds_ = speeds;
  jerk_limits_ = jerk;
}

std::pair<double, double> BasicCache::GetSmoothedAccelLimit(
    const double speed) const {
  return {
      common::math::InterpolationOne(speed, decel_limit_speeds_, decel_limits_),
      common::math::InterpolationOne(speed, accel_limit_speeds_,
                                     accel_limits_)};
}

std::pair<double, double> BasicCache::GetSmoothedJerkLimit(
    const double speed) const {
  const auto jerk =
      common::math::InterpolationOne(speed, jerk_limit_speeds_, jerk_limits_);
  return {-jerk, jerk};
}

}  // namespace speed_evaluator
}  // namespace planning
}  // namespace TL
