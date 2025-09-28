/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description: path_generator.h
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

#include "common/configs/vehicle_config_helper.h"

#include "common/vehicle_state/vehicle_state_provider.h"
#include "planning/open_space/coarse_path_generator/node3d.h"

#include "planning/common/open_space_info.h"
#include "planning/proto/planner_open_space_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {

struct PathGeneratorResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  common::PathPoint rs_connect_point;
  planning_internal::PathUpdateStatus::PathType path_type;

  void reset() {
    x.clear();
    y.clear();
    phi.clear();
    rs_connect_point.Clear();
    path_type = planning_internal::PathUpdateStatus::DEFAULT;
  }
};

class PathGenerator {
 public:
  explicit PathGenerator(const WarmStartConfig& warm_start_config);

  virtual ~PathGenerator() = default;

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
  virtual bool Plan(
      const std::atomic<bool>& atomic_early_stop_flag,
      const common::PathPoint& start_point, const common::PathPoint& end_point,
      const std::vector<double>& xy_bounds,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec,
      const DestRegionWithAng& dest_region_with_angle,
      const PathSearchStrategy& path_search_strategy,
      PathGeneratorResult* result) = 0;

  /**
 * @brief 
 * 
 * @param result 
 * @param partition_paths 
 * @return true 
 * @return false 
 */
  static bool PathPartition(const PathGeneratorResult& result,
                            std::vector<PathGearPair>* partition_paths);

 protected:
  /**
   * @brief check node is valid or not and return distance value
   *
   * @param node
   * @param obstacles_segments_vec obstacles which described as line segment
   * @param distance_to_obstalce minimal distance to obstacle
   * @return true has collision
   * @return false collision free
   */
  bool ValidityCheck(
      const std::shared_ptr<Node3d>& node,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec,
      double* distance_to_obstalce = nullptr);
  /**
   * @brief insert value into vec with binary order
   * 
   * @param par number to be sort
   * @param vec save the new value
   */
  void InsertWithBinaryOrder(const std::pair<size_t, size_t>& par,
                             std::vector<size_t>* vec) const;

  /**
   * @brief 
   * 
   * @param kappa 
   * @return double 
   */
  inline double TransKappaToSteering(double kappa) {
    return atan(kappa * vehicle_param_.wheel_base()) *
           vehicle_param_.steer_ratio();
  }

  const WarmStartConfig warm_start_config_;
  common::VehicleParam vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  double obstacle_filter_distance_ = 0.5;
  std::vector<double> xy_bounds_;
  double xy_grid_resolution_ = 0.1;
  double phi_grid_resolution_ = 0.1;
};

}  // namespace planning
}  // namespace TL
