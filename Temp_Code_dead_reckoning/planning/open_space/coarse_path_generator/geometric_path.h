/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  geometric_path.h
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
#include "planning/open_space/coarse_path_generator/shape/scs_shape_path.h"
#include "planning/proto/planner_open_space_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {

class GeometricPath : public PathGenerator {
 public:
  explicit GeometricPath(const WarmStartConfig& warm_start_config);
  ~GeometricPath() override = default;

  /**
 * @brief Use geometry algorithm to plan a feasible path with kinemic
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
 * @param geometry_path 
 * @param obstacles_without_virtual 
 * @param obstacles_segments_vec_geometry 
 * @return true 
 * @return false 
 */
  bool CollisonCheck(
      const PathGeneratorResult& geometry_path,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_without_virtual,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec_geometry);

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

  std::unique_ptr<SCSShapePath> scs_shape_path_planner_;
  const double extra_distance_for_geometry_ = 0.05;
  const double max_steer_angle_margin_;
};

}  // namespace planning
}  // namespace TL
