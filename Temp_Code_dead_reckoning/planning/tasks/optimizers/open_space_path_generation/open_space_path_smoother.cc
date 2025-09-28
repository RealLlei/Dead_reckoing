/*
 * Copyright (c) TL Technologies Co., Ltd. 2023. All rights reserved.
 * Description:  open_space_path_smoother.CC
 */

#include "planning/tasks/optimizers/open_space_path_generation/open_space_path_smoother.h"

#include <cstddef>
#include <string>
#include <tuple>

#include "common/math/line_segment2d.h"
#include "common/status/status.h"
#include "proto/common/error_code.pb.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

OpenSpacePathSmoother::OpenSpacePathSmoother(
    const NlpPathSmootherConfig& config)
    : config_(config) {
  nlp_path_smoother_ = std::make_unique<NlpPathSmoother>(config_);
}

void OpenSpacePathSmoother::Smooth(const OpenSpacePathInput& input,
                                   OpenSpacePathOutput* const output) {
  if (output == nullptr || !output->error_msg.empty()) {
    return;
  }
  output->has_smoothed = false;
  std::vector<PathGearPair> partitioned_smooth_paths;
  const auto& partition_paths = output->partitioned_path;
  const size_t size = partition_paths.size();
  const bool is_narrow_passage =
      input.path_strategy.path_search_strategy.is_narrow_passage_scenario;
  static constexpr size_t kMaxPathSizeThreshold = 17;
  const size_t max_path_partition_size =
      is_narrow_passage ? kMaxPathSizeThreshold
                        : config_.max_path_partition_size();
  if (size > max_path_partition_size || size == 0) {
    output->error_msg = "Coarse path partition size too large or zero";
    return;
  }
  static constexpr size_t kMinPathPointSize = 2;
  for (const auto& path_pair : partition_paths) {
    if (path_pair.first.size() < kMinPathPointSize) {
      output->error_msg = "The size of one path among partition_paths is " +
                          std::to_string(path_pair.first.size());
      return;
    }
  }

  const double smoother_start_time = common::Clock::NowInSeconds();
  ADEBUG << "paths size in smoother is " << size;
  auto path_gear = soc::Chassis::GEAR_PARKING;
  common::PathPoint init_point;
  common::PathPoint second_point;
  init_point.set_x(partition_paths.front().first[0].x());
  init_point.set_y(partition_paths.front().first[0].y());
  init_point.set_theta(partition_paths.front().first[0].theta());
  second_point.set_x(partition_paths.front().first[1].x());
  second_point.set_y(partition_paths.front().first[1].y());
  second_point.set_theta(partition_paths.front().first[1].theta());
  if (!common::math::GetGearFromPath(init_point, second_point, &path_gear)) {
    output->error_msg = "get init gear fails";
    return;
  }
  partitioned_smooth_paths.resize(size);
  // 0: left/right buffer; 1: has region; 2: use region
  std::tuple<std::pair<double, double>, bool, bool> dest_lat_region_constrain{
      {0.0, 0.0}, false, false};
  const auto& dest_region = std::get<0>(input.dest_region_with_angle);
  if (dest_region.num_points() >= 3) {
    const auto end_vec =
        common::math::Vec2d(partition_paths.back().first.back().x(),
                            partition_paths.back().first.back().y());
    const auto unit_vec = common::math::Vec2d::CreateUnitVec2d(
        partition_paths.back().first.back().theta() + M_PI_2);
    constexpr double kHalfLength = 10.0;
    const common::math::LineSegment2d cross_line(
        end_vec - kHalfLength * unit_vec, end_vec + kHalfLength * unit_vec);
    common::math::Vec2d first;
    common::math::Vec2d second;
    if (dest_region.IsPointIn(end_vec) &&
        dest_region.GetOverlap(cross_line, &first, &second)) {
      const double first_l = unit_vec.InnerProd({first - end_vec});
      const double second_l = unit_vec.InnerProd({second - end_vec});
      std::pair<double, double> left_right_moving_buffer;
      if (first_l >= 0.0) {
        left_right_moving_buffer.first = first_l;
        left_right_moving_buffer.second = -second_l;
      } else {
        left_right_moving_buffer.first = second_l;
        left_right_moving_buffer.second = -first_l;
      }
      constexpr double kMinLatWidth = 0.01;
      std::get<0>(dest_lat_region_constrain) = left_right_moving_buffer;
      std::get<1>(dest_lat_region_constrain) =
          left_right_moving_buffer.first + left_right_moving_buffer.second >
          kMinLatWidth;
      std::get<2>(dest_lat_region_constrain) =
          input.path_strategy.path_search_strategy.is_plan_from_start
              ? std::get<1>(dest_lat_region_constrain)
              : false;
    }
  }
  std::pair<double, bool> init_kappa_constrain{input.start_point.kappa(),
                                               false};
  const int first_moving_direction =
      path_gear == soc::Chassis::GEAR_DRIVE ? 1 : -1;
  init_kappa_constrain.second =
      (input.path_strategy.init_moving_direction * first_moving_direction > 0);
  std::vector<PathGearPair> path_gear_pairs;
  std::vector<std::vector<
      std::pair<common::math::LineSegment2d, common::math::LineSegment2d>>>
      xy_road_bounds;
  common::Status status = common::Status::OK();
  if (output->need_collision_free_smooth) {
    status = nlp_path_smoother_->XYRoadPreprocessor(
        partition_paths, input.obstacles_segments_vec, init_kappa_constrain,
        &path_gear_pairs, &xy_road_bounds);
  } else {
    // do not consider obstacles
    std::vector<std::pair<common::math::LineSegment2d, double>>
        empty_obstacles_segments_vec;
    status = nlp_path_smoother_->XYRoadPreprocessor(
        partition_paths, empty_obstacles_segments_vec, init_kappa_constrain,
        &path_gear_pairs, &xy_road_bounds);
  }
  if (!status.ok()) {
    output->error_msg = status.ToString();
    return;
  }
  bool force_enable_dest_lat_constrain = false;
  if (!nlp_path_smoother_->NlpSolver(
          path_gear_pairs, xy_road_bounds, init_kappa_constrain.second,
          dest_lat_region_constrain, &partitioned_smooth_paths,
          &force_enable_dest_lat_constrain)) {
    output->error_msg = "NlpSolver failed to solve.";
    return;
  }

  const double smoother_total_time =
      common::Clock::NowInSeconds() - smoother_start_time;
  debug_.set_smooth_total_time(smoother_total_time);
  debug_.set_smoother_enable_init_kappa_constrain(init_kappa_constrain.second);
  debug_.set_smoother_init_kappa(init_kappa_constrain.first);
  debug_.set_smoother_enable_dest_lat_constrain(
      std::get<2>(dest_lat_region_constrain));
  debug_.set_smoother_force_enable_dest_lat_constrain(
      force_enable_dest_lat_constrain);
  if (!partitioned_smooth_paths.empty() &&
      !partitioned_smooth_paths.back().first.empty()) {
    const double dx =
        std::fabs(partitioned_smooth_paths.back().first.back().x() -
                  partition_paths.back().first.back().x());
    const double dy =
        std::fabs(partitioned_smooth_paths.back().first.back().y() -
                  partition_paths.back().first.back().y());
    debug_.set_smoother_dest_lat_deviation(std::hypot(dx, dy));
  }
  AINFO << "Path smoothing total time: " << smoother_total_time << "s"
        << ", partitioned_smooth_paths.size: "
        << partitioned_smooth_paths.size();
  debug_.clear_partition_smoothed_path();
  for (const auto& path : partitioned_smooth_paths) {
    auto* path_ptr = debug_.add_partition_smoothed_path();
    for (const auto& p : path.first) {
      auto* point = path_ptr->add_path_point();
      point->CopyFrom(p);
    }
  }

  // local visualization
  debug_.clear_smoothed_path();
  auto* smoothed_path_ptr = debug_.mutable_smoothed_path();
  smoothed_path_ptr->Clear();
  common::PathPoint tmp;
  for (const auto& path : partitioned_smooth_paths) {
    for (size_t i = 0; i + 1 < path.first.size(); i++) {
      tmp = path.first.at(i);
      PathPointNormalizing(input.rotate_angle, input.translate_origin, &tmp);
      auto* smoothed_point_ptr = smoothed_path_ptr->Add();
      *smoothed_point_ptr = tmp;
    }
  }
  tmp = partitioned_smooth_paths.back().first.back();
  PathPointNormalizing(input.rotate_angle, input.translate_origin, &tmp);
  auto* smoothed_point_ptr = smoothed_path_ptr->Add();
  *smoothed_point_ptr = tmp;
  output->partitioned_path = std::move(partitioned_smooth_paths);
  output->has_smoothed = true;
}

void OpenSpacePathSmoother::PathPointNormalizing(
    const double rotate_angle, const Vec2d& translate_origin,
    common::PathPoint* const path_point_ptr) {
  if (path_point_ptr != nullptr) {
    double x = path_point_ptr->x() - translate_origin.x();
    double y = path_point_ptr->y() - translate_origin.y();
    path_point_ptr->set_x(x * std::cos(-rotate_angle) -
                          y * std::sin(-rotate_angle));
    path_point_ptr->set_y(x * std::sin(-rotate_angle) +
                          y * std::cos(-rotate_angle));
    path_point_ptr->set_theta(
        common::math::NormalizeAngle(path_point_ptr->theta() - rotate_angle));
  }
}

void OpenSpacePathSmoother::UpdateDebugInfo(
    planning_internal::OpenSpaceDebug* const debug) {
  if (debug == nullptr) {
    return;
  }
  debug->clear_partition_smoothed_path();
  debug->clear_smoothed_path();
  debug->MergeFrom(debug_);
}

}  // namespace planning
}  // namespace TL
