/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  geometry_path_generator.h
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "planning/common/open_space_info.h"
#include "planning/open_space/coarse_path_generator/path_generator.h"
#include "planning/open_space/coarse_path_generator/shape/geometry_path.h"
#include "planning/proto/planner_open_space_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {

class GeometryPathGenerator : public PathGenerator {
 public:
  explicit GeometryPathGenerator(const WarmStartConfig& warm_start_config);
  ~GeometryPathGenerator() override = default;

  /**
 * @brief Use hybird a star algorithm to plan a feasible path with kinemic
 * constrain
 * 
 * @param atomic_early_stop_flag 
 * @param start_point 
 * @param end_point 
 * @param xy_bounds 
 * @param obstacles_segments_vec 
 * @param dest_region_with_angle 
 * @param path_search_strategy 
 * @param result 
 * @return true 
 * @return false 
 */
  bool Plan(const std::atomic<bool>& atomic_early_stop_flag,
            const common::PathPoint& start_point,
            const common::PathPoint& end_point,
            const std::vector<double>& xy_bounds,
            const std::vector<std::pair<common::math::LineSegment2d, double>>&
                obstacles_segments_vec,
            const DestRegionWithAng& dest_region_with_angle,
            const PathSearchStrategy& path_search_strategy,
            PathGeneratorResult* result) override;

 private:
  /**
 * @brief 
 * 
 * @param is_cycle_straight_connect 
 * @param start_point 
 * @param end_point 
 * @param intermediate_point_longitudal_bound 
 * @param geometry_path 
 * @return true 
 * @return false 
 */
  bool IsGeometryPathAcceptable(
      bool is_cycle_straight_connect, const common::PathPoint& start_point,
      const common::PathPoint& end_point,
      const std::pair<double, double>& intermediate_point_longitudal_bound,
      const GeometryPath& geometry_path) const;
  /**
   * @brief Get the Result object
   * 
   * @param pursuit_precise_pose_connect 
   * @param geometry_path 
   * @param result 
   * @return true 
   * @return false 
   */
  static bool GetResult(bool pursuit_precise_pose_connect,
                        const GeometryPath& geometry_path,
                        PathGeneratorResult* result);
  /**
 * @brief 
 * 
 * @param geometry_path 
 * @param obstacles_without_virtual 
 * @param obstacles_segments_vec_geometry 
 * @return true 
 * @return false 
 */
  bool CollisonFreeCheck(
      const GeometryPath& geometry_path,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_without_virtual,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec_geometry);
  /**
 * @brief 
 * 
 * @param pursuit_precise_pose_connect 
 * @param start_point 
 * @param end_point 
 * @param park_direction 
 * @param cycle_direction 
 */
  void CycleDirectionStrategy(bool pursuit_precise_pose_connect,
                              const common::PathPoint& start_point,
                              const common::PathPoint& end_point,
                              const ParkDirection& park_direction,
                              CycleDirection* cycle_direction) const;

  /**
   * @brief Caculate  min radius for geometry path
   * 
   */
  inline double GetMinRadius() {
    static constexpr double kEpsion = 1e-6;
    const double max_steer_angle =
        vehicle_param_.max_steer_angle() - max_steer_angle_margin_;

    const double max_beta = common::VehicleStateProvider::EstimateBetaAngle(
        max_steer_angle, vehicle_param_);
    return vehicle_param_.wheel_base() / (fabs(tan(max_beta)) + kEpsion);
  }

  PathSearchStrategy path_search_strategy_;
  std::unique_ptr<Geometry> geometry_planner_;
  double extra_distance_for_geometry_ = 0.05;
  const double max_steer_angle_margin_;
};

}  // namespace planning
}  // namespace TL
