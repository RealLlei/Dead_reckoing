/*
 * Copyright (c) TL Technologies Co., Ltd. 2023. All rights reserved.
 * Description:  open_space_path_generator.h
 */

#pragma once

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "planning/common/path/discretized_path.h"
#include "planning/open_space/coarse_path_generator/geometric_path.h"
#include "planning/open_space/coarse_path_generator/ilqr_path.h"
#include "planning/open_space/coarse_path_generator/path_generator.h"
#include "planning/proto/open_space_task_config.pb.h"
#include "proto/planning/planning_internal.pb.h"

namespace TL {
namespace planning {

class OpenSpacePathGenerator {
 public:
  explicit OpenSpacePathGenerator(const WarmStartConfig& config);

  ~OpenSpacePathGenerator() = default;

  /**
   * @brief Plan
   * 
   * @param atomic_early_stop_flag 
   * @param input 
   * @param output 
   */
  void Plan(const std::atomic<bool>& atomic_early_stop_flag,
            const OpenSpacePathInput& input, OpenSpacePathOutput* output);

  /**
   * @brief UpdateDebugInfo
   * 
   * @param debug 
   */
  void UpdateDebugInfo(planning_internal::OpenSpaceDebug* debug);

 private:
  /**
   * @brief RecordWarmStartInputDebugInfo
   * 
   * @param xy_bounds 
   * @param start_point 
   * @param end_point 
   * @param obstacles_segments_vec 
   * @param origin_point 
   * @param origin_heading 
   * @param debug 
   */
  static void RecordWarmStartInputDebugInfo(
      const std::vector<double>& xy_bounds,
      const common::PathPoint& start_point, const common::PathPoint& end_point,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec,
      const common::math::Vec2d& origin_point, double origin_heading,
      planning_internal::OpenSpaceDebug* debug);

  /**
   * @brief RecordWarmStartResultDebugInfo
   * 
   * @param result 
   */
  static void RecordWarmStartResultDebugInfo(
      const PathGeneratorResult& result,
      planning_internal::OpenSpaceDebug* debug);

  /**
   * @brief TransInputToLocalFrame
   * 
   * @param origin_heading 
   * @param origin_point 
   * @param start_point 
   * @param end_pose 
   * @param dest_region_with_angle 
   * @param obstacles_segments_vec_ptr 
   */
  static void TransInputToLocalFrame(
      double origin_heading, const common::math::Vec2d& origin_point,
      common::PathPoint* start_point, common::PathPoint* end_pose,
      DestRegionWithAng* dest_region_with_angle,
      std::vector<std::pair<common::math::LineSegment2d, double>>*
          obstacles_segments_vec_ptr);

  /**
   * @brief RemoveCollisionVirtualObs
   * 
   * @param start_point 
   * @param obstacles_segments_vec_ptr 
   */
  static void RemoveCollisionVirtualObs(
      const common::PathPoint& start_point,
      std::vector<std::pair<common::math::LineSegment2d, double>>*
          obstacles_segments_vec_ptr);

  /**
   * @brief GenerateCoarsePath
   * 
   * @param atomic_early_stop_flag 
   * @param start_point_local 
   * @param end_pose_local 
   * @param xy_bounds 
   * @param obstacles_segments_vec_local 
   * @param dest_region_with_angle_local 
   * @param path_strategy 
   * @param coarse_path_ptr 
   * @param need_collision_free_smooth 
   * @return TL::common::Status 
   */
  TL::common::Status GenerateCoarsePath(
      const std::atomic<bool>& atomic_early_stop_flag,
      const common::PathPoint& start_point_local,
      const common::PathPoint& end_pose_local,
      const std::vector<double>& xy_bounds,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec_local,
      const DestRegionWithAng& dest_region_with_angle_local,
      const PathStrategy& path_strategy, PathGeneratorResult* coarse_path_ptr,
      bool* need_collision_free_smooth);

  /**
   * @brief PathPointNormalizing
   * 
   * @param rotate_angle 
   * @param translate_origin 
   * @param path_point_ptr 
   */
  static void PathPointNormalizing(double rotate_angle,
                                   const Vec2d& translate_origin,
                                   common::PathPoint* path_point_ptr);
  /**
   * @brief ReferencePointNormalizing
   * 
   * @param rotate_angle 
   * @param translate_origin 
   * @param reference_point_ptr 
   */
  static void ReferencePointNormalizing(double rotate_angle,
                                        const Vec2d& translate_origin,
                                        ReferencePoint* reference_point_ptr);

  /**
   * @brief SeparateGeometryStrategy
   * 
   * @param path_strategy 
   * @param precise_pose_geometry_strategy 
   * @param precise_angle_geometry_strategy 
   * @return true 
   * @return false 
   */
  static bool SeparateGeometryStrategy(
      const PathSearchStrategy& path_strategy,
      GeometryStrategy* precise_pose_geometry_strategy,
      GeometryStrategy* precise_angle_geometry_strategy);

  /**
   * @brief PathDeNormal
   * 
   * @param origin_point 
   * @param origin_heading 
   * @param partition_paths 
   */
  static void PathDeNormal(const common::math::Vec2d& origin_point,
                           double origin_heading,
                           std::vector<PathGearPair>* partition_paths);

  /**
   * @brief CombineTraceAdjustPath
   * 
   * @param trace_path
   * @param search_path
   * @param path_result 
   */
  void CombineTraceAdjustPath(const DiscretizedPath& trace_path,
                              const PathGeneratorResult& search_path,
                              PathGeneratorResult* path_result);

  const WarmStartConfig config_;
  std::unique_ptr<PathGenerator> hybrid_a_star_planner_;
  std::unique_ptr<PathGenerator> geometry_planner_;
  PathGeneratorResult default_warm_start_path_result_;
  std::unique_ptr<GeometricPath> geometric_path_planner_;
  std::unique_ptr<ILQR> ilqr_planner_;
  planning_internal::OpenSpaceDebug debug_;
};
}  // namespace planning
}  // namespace TL
