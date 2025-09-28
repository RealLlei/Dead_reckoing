/*
 * Copyright (c) TL Technologies Co., Ltd. 2023. All rights reserved.
 * Description:  open_space_path_generator.CC
 */

#include "planning/tasks/optimizers/open_space_path_generation/open_space_path_generator.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "planning/common/open_space_info.h"
#include "planning/common/path/discretized_path.h"
#include "planning/open_space/coarse_path_generator/geometry_path_generator.h"
#include "planning/open_space/coarse_path_generator/hybrid_a_star.h"
#include "planning/open_space/coarse_path_generator/ilqr_path.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/common/vehicle_model_config.pb.h"

namespace TL {
namespace planning {

using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::math::Vec2d;

OpenSpacePathGenerator::OpenSpacePathGenerator(const WarmStartConfig& config)
    : config_(config),
      hybrid_a_star_planner_(std::make_unique<HybridAStar>(config_)),
      geometry_planner_(std::make_unique<GeometryPathGenerator>(config_)),
      geometric_path_planner_(std::make_unique<GeometricPath>(config_)),
      ilqr_planner_(std::make_unique<ILQR>(config_)) {}

void OpenSpacePathGenerator::Plan(
    const std::atomic<bool>& atomic_early_stop_flag,
    const OpenSpacePathInput& input, OpenSpacePathOutput* const output) {
  // step 1. check validity of input data
  if (output == nullptr) {
    return;
  }
  output->Reset();
  if (input.xy_bounds.empty()) {
    output->error_msg = "empty xy_bounds";
    output->partitioned_path.clear();
    return;
  }
  output->replan_status = input.replan_status;
  const auto start_timestamp = common::Clock::NowInSeconds();
  auto start_point_local = input.start_point;
  auto end_pose_local = input.end_pose;
  auto dest_region_with_angle_local = input.dest_region_with_angle;
  auto obstacles_segments_vec_local = input.obstacles_segments_vec;
  TransInputToLocalFrame(input.rotate_angle, input.translate_origin,
                         &start_point_local, &end_pose_local,
                         &dest_region_with_angle_local,
                         &obstacles_segments_vec_local);
  default_warm_start_path_result_.reset();
  if (!input.warm_start_path.empty()) {
    auto warm_start_path_local = input.warm_start_path;
    for (auto& point : warm_start_path_local) {
      PathPointNormalizing(input.rotate_angle, input.translate_origin, &point);
      default_warm_start_path_result_.x.emplace_back(point.x());
      default_warm_start_path_result_.y.emplace_back(point.y());
      default_warm_start_path_result_.phi.emplace_back(point.theta());
    }
    default_warm_start_path_result_.path_type =
        planning_internal::PathUpdateStatus::TRACE_PATH;
  }
  output->partitioned_path.clear();
  PathStrategy path_strategy_loc = input.path_strategy;
  if (path_strategy_loc.path_search_strategy.collision_free_search_strategy
          .replan_due_to_collision) {
    common::PathPoint collsion_point_loc =
        path_strategy_loc.path_search_strategy.collision_free_search_strategy
            .collision_path_point;
    PathPointNormalizing(input.rotate_angle, input.translate_origin,
                         &collsion_point_loc);
    path_strategy_loc.path_search_strategy.collision_free_search_strategy
        .collision_path_point = collsion_point_loc;
  }
  if (path_strategy_loc.path_search_strategy.trace_adjust_search_strategy
          .is_trace_adjust) {
    auto& trace_path_local = path_strategy_loc.path_search_strategy
                                 .trace_adjust_search_strategy.trace_path;
    for (auto& point : trace_path_local) {
      PathPointNormalizing(input.rotate_angle, input.translate_origin, &point);
    }
  }
  // trans referenceline to local frame
  auto& reference_line = path_strategy_loc.path_search_strategy.reference_line;
  if (path_strategy_loc.path_search_strategy.is_nns_adjust_senario &&
      !reference_line.reference_points().empty()) {
    std::vector<ReferencePoint> temp_reference_line;
    ReferencePoint temp_path_point;
    for (const auto& reference_point : reference_line.reference_points()) {
      temp_path_point = reference_point;
      ReferencePointNormalizing(input.rotate_angle, input.translate_origin,
                                &temp_path_point);
      temp_reference_line.push_back(temp_path_point);
    }
    reference_line.SetReferencePoints(std::move(temp_reference_line),
                                      reference_line.GetInterval());
  }
  RemoveCollisionVirtualObs(start_point_local, &obstacles_segments_vec_local);
  // step 2.Generate coarse path by hybrid a star or geometry planner
  ADEBUG << "start generate coarse path";
  if (FLAGS_enable_record_openspace_debug) {
    RecordWarmStartInputDebugInfo(input.xy_bounds, input.start_point,
                                  input.end_pose, obstacles_segments_vec_local,
                                  input.translate_origin, input.rotate_angle,
                                  &debug_);
  }
  PathGeneratorResult result;
  output->need_collision_free_smooth = true;
  const double coarse_start_time = common::Clock::NowInSeconds();
  Status status = GenerateCoarsePath(
      atomic_early_stop_flag, start_point_local, end_pose_local,
      input.xy_bounds, obstacles_segments_vec_local,
      dest_region_with_angle_local, path_strategy_loc, &result,
      &(output->need_collision_free_smooth));
  if (status != Status::OK()) {
    output->error_msg = status.error_message();
    return;
  }
  ADEBUG << "result.path_type " << result.path_type;
  output->path_type = result.path_type;

  if (FLAGS_enable_record_openspace_debug) {
    RecordWarmStartResultDebugInfo(result, &debug_);
  }
  // step 3.Path partition
  if (!PathGenerator::PathPartition(result, &(output->partitioned_path))) {
    output->error_msg = "Hybrid Astar partition failed";
    return;
  }
  PathDeNormal(input.translate_origin, input.rotate_angle,
               &(output->partitioned_path));

  const double coarse_total_time =
      common::Clock::NowInSeconds() - coarse_start_time;
  debug_.set_coarse_total_time(coarse_total_time);
  ADEBUG << "open space coarse path total time(unit s): "
         << common::Clock::NowInSeconds() - start_timestamp;
}

void OpenSpacePathGenerator::TransInputToLocalFrame(
    const double origin_heading, const common::math::Vec2d& origin_point,
    common::PathPoint* const start_point_ptr,
    common::PathPoint* const end_pose_ptr,
    DestRegionWithAng* const dest_region_with_angle_ptr,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const
        obstacles_segments_vec_ptr) {
  PathPointNormalizing(origin_heading, origin_point, start_point_ptr);
  PathPointNormalizing(origin_heading, origin_point, end_pose_ptr);
  const auto& polygon_enu = std::get<0>(*dest_region_with_angle_ptr);
  const double from_angle = std::get<1>(*dest_region_with_angle_ptr);
  const double to_angle = std::get<2>(*dest_region_with_angle_ptr);
  if (polygon_enu.points().size() > 2) {
    std::vector<Vec2d> polygon_points(polygon_enu.points());
    for (auto& point : polygon_points) {
      point -= origin_point;
      point.SelfRotate(-origin_heading);
    }
    *dest_region_with_angle_ptr = {common::math::Polygon2d(polygon_points),
                                   from_angle - origin_heading,
                                   to_angle - origin_heading};
  }
  for (auto& seg_pair : *obstacles_segments_vec_ptr) {
    seg_pair.first.Transform(origin_point, origin_heading);
  }
}

void OpenSpacePathGenerator::RemoveCollisionVirtualObs(
    const common::PathPoint& start_point,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const
        obstacles_segments_vec_ptr) {
  if (nullptr == obstacles_segments_vec_ptr ||
      obstacles_segments_vec_ptr->empty()) {
    return;
  }
  static constexpr double kEps = 1e-5;
  const auto start_polygon = common::math::Polygon2d(
      common::VehicleConfigHelper::GetBoundingBox(start_point));
  int cnt = 0;
  auto iter = obstacles_segments_vec_ptr->begin();
  while (iter != obstacles_segments_vec_ptr->end()) {
    if (iter->second < kEps && start_polygon.HasOverlap(iter->first)) {
      // virtual obs has overlaps with start_point
      iter = obstacles_segments_vec_ptr->erase(iter);
      ++cnt;
    } else {
      ++iter;
    }
  }
  ADEBUG << "RemoveCollisionVirtualObs cnt " << cnt;
}

void OpenSpacePathGenerator::RecordWarmStartInputDebugInfo(
    const std::vector<double>& xy_bounds, const common::PathPoint& start_point,
    const common::PathPoint& end_point,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec,
    const common::math::Vec2d& origin_point, const double origin_heading,
    planning_internal::OpenSpaceDebug* const debug) {
  if (debug == nullptr) {
    return;
  }
  debug->clear_warm_start_path();
  // load start point in world coordinate
  *debug->mutable_trajectory_stitching_point()->mutable_path_point() =
      start_point;

  // load translation origin and heading angle
  auto* roi_shift_point = debug->mutable_roi_shift_point();
  roi_shift_point->Clear();
  roi_shift_point->mutable_path_point()->set_x(origin_point.x());
  roi_shift_point->mutable_path_point()->set_y(origin_point.y());
  roi_shift_point->mutable_path_point()->set_theta(origin_heading);

  // load end_pose into debug
  *debug->mutable_end_point()->mutable_path_point() = end_point;
  // load xy boundary (xmin, xmax, ymin, ymax)
  debug->mutable_xy_boundary()->Clear();
  if (xy_bounds.size() > 3) {
    debug->add_xy_boundary(xy_bounds[0]);
    debug->add_xy_boundary(xy_bounds[1]);
    debug->add_xy_boundary(xy_bounds[2]);
    debug->add_xy_boundary(xy_bounds[3]);
  }

  // load obstacles
  debug->mutable_obstacles()->Clear();
  for (const auto& obstacle_vertices : obstacles_segments_vec) {
    const auto& linesegment = obstacle_vertices.first;
    auto* obstacle_ptr = debug->add_obstacles();
    obstacle_ptr->add_vertices_x_coords(linesegment.start().x());
    obstacle_ptr->add_vertices_y_coords(linesegment.start().y());
    obstacle_ptr->add_vertices_x_coords(linesegment.end().x());
    obstacle_ptr->add_vertices_y_coords(linesegment.end().y());
    if (fabs(obstacle_vertices.second - 0.3) < kEpsilon &&
        obstacle_vertices.first.length() < kEpsilon) {
      obstacle_ptr->set_id("wheel_mask");
    } else {
      obstacle_ptr->set_id("default");
    }
  }
}

void OpenSpacePathGenerator::RecordWarmStartResultDebugInfo(
    const PathGeneratorResult& result,
    planning_internal::OpenSpaceDebug* const debug) {
  if (debug == nullptr) {
    return;
  }
  debug->clear_warm_start_path();
  size_t horizon = result.x.size();
  auto* warm_start_path =
      debug->mutable_warm_start_path()->mutable_warm_start_path_points();
  for (size_t i = 0; i < horizon; ++i) {
    auto* warm_start_point = warm_start_path->Add();
    warm_start_point->set_x(result.x.at(i));
    warm_start_point->set_y(result.y.at(i));
    warm_start_point->set_theta(result.phi.at(i));
  }
  *debug->mutable_warm_start_path()->mutable_rs_connect_point() =
      result.rs_connect_point;
  *debug->mutable_path_planner_type() =
      planning_internal::PathUpdateStatus::PathType_Name(result.path_type);
}

TL::common::Status OpenSpacePathGenerator::GenerateCoarsePath(
    const std::atomic<bool>& atomic_early_stop_flag,
    const common::PathPoint& start_point_local,
    const common::PathPoint& end_pose_local,
    const std::vector<double>& xy_bounds,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec_local,
    const DestRegionWithAng& dest_region_with_angle_local,
    const PathStrategy& path_strategy,
    PathGeneratorResult* const coarse_path_ptr,
    bool* const need_collision_free_smooth) {
  ADEBUG << "start generate coarse path";
  if (nullptr == coarse_path_ptr) {
    std::string msg = "coarse path ptr is null";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_PATHGENERATOR_ERROR, msg);
  }
  if (path_strategy.disable_search) {
    ADEBUG << "disable search path, use default path directly";
    *coarse_path_ptr = default_warm_start_path_result_;
    default_warm_start_path_result_.reset();
    return Status::OK();
  }
  if (path_strategy.path_search_strategy.park_direction == PARKIN &&
      ilqr_planner_->Plan(
          atomic_early_stop_flag, start_point_local, end_pose_local, xy_bounds,
          obstacles_segments_vec_local, dest_region_with_angle_local,
          path_strategy.path_search_strategy, coarse_path_ptr)) {
    ADEBUG << "ilqr suc, return directly";
    return Status::OK();
  }
  PathSearchStrategy path_search_strategy = path_strategy.path_search_strategy;
  PathSearchStrategy precise_pose_strategy = path_search_strategy;
  PathSearchStrategy precise_angle_strategy = path_search_strategy;
  if (!SeparateGeometryStrategy(
          path_search_strategy, &(precise_pose_strategy.use_geometry_strategy),
          &(precise_angle_strategy.use_geometry_strategy))) {
    std::string msg = "load geometry strategy fail";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_PATHGENERATOR_ERROR, msg);
  }
  const auto& use_geometry =
      path_search_strategy.use_geometry_strategy.use_geometry;
  if (nullptr != need_collision_free_smooth) {
    *need_collision_free_smooth = true;
  }
  std::string warm_start_msg = "init to generate coarse path";
  switch (use_geometry) {
    case ONLY_USE: {
      if (!geometric_path_planner_->Plan(
              atomic_early_stop_flag, start_point_local, end_pose_local,
              xy_bounds, obstacles_segments_vec_local,
              dest_region_with_angle_local, path_strategy.path_search_strategy,
              coarse_path_ptr)) {
        warm_start_msg = "use geometry only coarse path  fail";
        AERROR << warm_start_msg;
      }
      break;
    }
    case USE_FIRST: {
      if (!geometric_path_planner_->Plan(
              atomic_early_stop_flag, start_point_local, end_pose_local,
              xy_bounds, obstacles_segments_vec_local,
              dest_region_with_angle_local, path_strategy.path_search_strategy,
              coarse_path_ptr) &&
          !hybrid_a_star_planner_->Plan(
              atomic_early_stop_flag, start_point_local, end_pose_local,
              xy_bounds, obstacles_segments_vec_local,
              dest_region_with_angle_local, path_search_strategy,
              coarse_path_ptr)) {
        warm_start_msg = "use geometry first,  generate coarse path  fail";
        AERROR << warm_start_msg;
      }
      break;
    }
    case USE_FIRST_LAST:
    case USE_BOTH: {
      // TODO(jyw): remove geometry_planner_
      if (geometry_planner_->Plan(atomic_early_stop_flag, start_point_local,
                                  end_pose_local, xy_bounds,
                                  obstacles_segments_vec_local,
                                  dest_region_with_angle_local,
                                  precise_pose_strategy, coarse_path_ptr)) {
        ADEBUG
            << " first use precise pose geometry success, skip use following "
               "procedure";
        break;
      }
    }
    case USE_LAST: {
      PathGeneratorResult result_explore;
      PathGeneratorResult result_geometry;
      const bool explore_ret =
          (use_geometry != USE_BOTH) &&
          hybrid_a_star_planner_->Plan(atomic_early_stop_flag,
                                       start_point_local, end_pose_local,
                                       xy_bounds, obstacles_segments_vec_local,
                                       dest_region_with_angle_local,
                                       path_search_strategy, &result_explore);
      const bool geometry_ret = geometry_planner_->Plan(
          atomic_early_stop_flag, start_point_local, end_pose_local, xy_bounds,
          obstacles_segments_vec_local, dest_region_with_angle_local,
          precise_angle_strategy, &result_geometry);
      if (!explore_ret && !geometry_ret) {
        ADEBUG << "both explore and  geomery fail";
        if (path_search_strategy.space_structure != LAT_PARK_LOT ||
            !path_search_strategy.collision_free_search_strategy
                 .replan_due_to_collision) {
          warm_start_msg =
              "both precise angle geomery and search fail, use history path";
          AERROR << warm_start_msg;
          break;
        }
        if (result_geometry.x.empty() ||
            result_geometry.x.size() != result_geometry.y.size() ||
            result_geometry.x.size() != result_geometry.phi.size()) {
          warm_start_msg = "precise angle geomery fallback validity checkfails";
          AERROR << warm_start_msg;
          break;
        }
        ADEBUG << "precise angle purpose in lat slot park, use geomery result "
                  "fallback";
        *coarse_path_ptr = result_geometry;
        if (nullptr != need_collision_free_smooth) {
          *need_collision_free_smooth = false;
        }
      }
      if (!explore_ret) {
        *coarse_path_ptr = result_geometry;
      } else if (!geometry_ret) {
        *coarse_path_ptr = result_explore;
      } else {
        if (result_explore.y.empty() || result_geometry.y.empty()) {
          warm_start_msg = "geometry or explore result is invalid";
          AERROR << warm_start_msg;
          break;
        }
        *coarse_path_ptr = result_geometry;
        if (path_search_strategy.space_structure == LAT_PARK_LOT &&
            result_geometry.y.back() > result_explore.y.back()) {
          ADEBUG << " explore ret is better";
          *coarse_path_ptr = result_explore;
        }
      }
      break;
    }
    default: {
      if (!hybrid_a_star_planner_->Plan(
              atomic_early_stop_flag, start_point_local, end_pose_local,
              xy_bounds, obstacles_segments_vec_local,
              dest_region_with_angle_local, path_search_strategy,
              coarse_path_ptr)) {
        warm_start_msg = "use hybrid only, generate coarse path  fail";
        AERROR << warm_start_msg;
      }
      break;
    }
  }
  if (path_search_strategy.trace_adjust_search_strategy.is_trace_adjust) {
    const auto search_path_result = *coarse_path_ptr;
    CombineTraceAdjustPath(
        path_search_strategy.trace_adjust_search_strategy.trace_path,
        search_path_result, coarse_path_ptr);
  }
  if (coarse_path_ptr->x.empty()) {
    // failed generate coarse path, use defalut path
    if (default_warm_start_path_result_.x.empty()) {
      return Status(ErrorCode::PLANNER_PARKING_PATHGENERATOR_ERROR,
                    warm_start_msg);
    }
    *coarse_path_ptr = default_warm_start_path_result_;
    default_warm_start_path_result_.reset();
  }
  return Status::OK();
}

void OpenSpacePathGenerator::PathPointNormalizing(
    double rotate_angle, const Vec2d& translate_origin,
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

void OpenSpacePathGenerator::ReferencePointNormalizing(
    const double rotate_angle, const Vec2d& translate_origin,
    ReferencePoint* const reference_point_ptr) {
  if (reference_point_ptr != nullptr) {
    double x = reference_point_ptr->x() - translate_origin.x();
    double y = reference_point_ptr->y() - translate_origin.y();
    reference_point_ptr->set_x(x * std::cos(-rotate_angle) -
                               y * std::sin(-rotate_angle));
    reference_point_ptr->set_y(x * std::sin(-rotate_angle) +
                               y * std::cos(-rotate_angle));
    reference_point_ptr->set_heading(common::math::NormalizeAngle(
        reference_point_ptr->heading() - rotate_angle));
  }
}

bool OpenSpacePathGenerator::SeparateGeometryStrategy(
    const PathSearchStrategy& path_strategy,
    GeometryStrategy* const precise_pose_geometry_strategy,
    GeometryStrategy* const precise_angle_geometry_strategy) {
  if (nullptr == precise_pose_geometry_strategy ||
      nullptr == precise_angle_geometry_strategy ||
      (path_strategy.use_geometry_strategy.use_geometry != NOT_USE &&
       (path_strategy.use_geometry_strategy.use_purpose.empty() ||
        path_strategy.use_geometry_strategy.use_purpose.size() !=
            path_strategy.use_geometry_strategy.geometry_path_type.size() ||
        path_strategy.use_geometry_strategy.use_purpose.size() !=
            path_strategy.use_geometry_strategy.longitudal_bound.size()))) {
    AERROR << "SeparateGeometryStrategy input check fails";
    return false;
  }
  if (path_strategy.use_geometry_strategy.use_geometry == NOT_USE) {
    ADEBUG << " geometry planner is not used";
    return true;
  }
  precise_pose_geometry_strategy->geometry_path_type.clear();
  precise_pose_geometry_strategy->longitudal_bound.clear();
  precise_pose_geometry_strategy->use_purpose.clear();
  precise_angle_geometry_strategy->geometry_path_type.clear();
  precise_angle_geometry_strategy->longitudal_bound.clear();
  precise_angle_geometry_strategy->use_purpose.clear();
  for (size_t i = 0; i < path_strategy.use_geometry_strategy.use_purpose.size();
       i++) {
    if (path_strategy.use_geometry_strategy.use_purpose.at(i) == PRECISEPOSE) {
      precise_pose_geometry_strategy->use_purpose.emplace_back(PRECISEPOSE);
      precise_pose_geometry_strategy->geometry_path_type.emplace_back(
          path_strategy.use_geometry_strategy.geometry_path_type.at(i));
      precise_pose_geometry_strategy->longitudal_bound.emplace_back(
          path_strategy.use_geometry_strategy.longitudal_bound.at(i));
    } else {
      precise_angle_geometry_strategy->use_purpose.emplace_back(PRECISEANGLE);
      precise_angle_geometry_strategy->geometry_path_type.emplace_back(
          path_strategy.use_geometry_strategy.geometry_path_type.at(i));
      precise_angle_geometry_strategy->longitudal_bound.emplace_back(
          path_strategy.use_geometry_strategy.longitudal_bound.at(i));
    }
  }
  return true;
}

void OpenSpacePathGenerator::PathDeNormal(
    const common::math::Vec2d& origin_point, const double origin_heading,
    std::vector<PathGearPair>* const partition_paths) {
  if (partition_paths == nullptr) {
    AERROR << "PathDeNormal input check fails";
    return;
  }
  for (auto& path_pair : *partition_paths) {
    for (auto& point : path_pair.first) {
      double tmp_x = point.x();
      double tmp_y = point.y();
      point.set_x(tmp_x * std::cos(origin_heading) -
                  tmp_y * std::sin(origin_heading) + origin_point.x());
      point.set_y(tmp_x * std::sin(origin_heading) +
                  tmp_y * std::cos(origin_heading) + origin_point.y());
      point.set_theta(
          common::math::NormalizeAngle(point.theta() + origin_heading));
    }
  }
}

void OpenSpacePathGenerator::CombineTraceAdjustPath(
    const DiscretizedPath& trace_path,
    const PathGeneratorResult& search_path_result,
    PathGeneratorResult* const path_result) {
  if (nullptr == path_result || search_path_result.x.empty()) {
    return;
  }
  *path_result = default_warm_start_path_result_;
  common::SLPoint search_end_sl;
  if (!trace_path.XYToSL(search_path_result.x.back(),
                         search_path_result.y.back(), &search_end_sl)) {
    return;
  }
  *path_result = search_path_result;
  const double step_s = 0.1;
  double acc_s = search_end_sl.s() + step_s;
  common::PathPoint path_point;
  while (acc_s < trace_path.Length()) {
    path_point = trace_path.Evaluate(acc_s);
    path_result->x.push_back(path_point.x());
    path_result->y.push_back(path_point.y());
    path_result->phi.push_back(path_point.theta());
    acc_s += step_s;
  }
  path_result->path_type = default_warm_start_path_result_.path_type;
}

void OpenSpacePathGenerator::UpdateDebugInfo(
    planning_internal::OpenSpaceDebug* const debug) {
  if (debug == nullptr) {
    return;
  }

  debug->clear_warm_start_path();
  debug->clear_xy_boundary();
  debug->clear_obstacles();
  debug->MergeFrom(debug_);
}

}  // namespace planning
}  // namespace TL
