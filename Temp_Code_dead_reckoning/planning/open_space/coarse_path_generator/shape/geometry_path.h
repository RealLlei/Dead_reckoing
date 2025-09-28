/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  open_space_obstacle.h
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "common/math/vec2d.h"
#include "planning/proto/planner_open_space_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {

struct GeometryOnePathPoint {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
};

enum CycleDirection {
  ANY = 0,
  FORCELEFT = 1,
  FORCERIGHT = 2,
};

enum GeometryPathType { STRAIGHT = 0, CYCLE = 1 };

using GeometryPath =
    std::vector<std::pair<GeometryOnePathPoint, GeometryPathType>>;

class Geometry {
 public:
  Geometry(const common::VehicleParam& vehicle_param,
           const WarmStartConfig& warm_start_config);
  virtual ~Geometry() = default;
  /**
 * @brief 
 * 
 * @param start_node 
 * @param end_node 
 * @param is_cycle_straight_connect 
 * @param min_radius 
 * @param consider_kappa_diff 
 * @param cycle_direction 
 * @param geometry_path 
 * @param precise_pose_connect 
 * @return true 
 * @return false 
 */
  bool GenerateGeometryPath(const common::PathPoint& start_node,
                            const common::PathPoint& end_node,
                            bool is_cycle_straight_connect, double min_radius,
                            bool consider_kappa_diff,
                            const CycleDirection& cycle_direction,
                            GeometryPath* geometry_path,
                            bool precise_pose_connect = true);
  /**
 * @brief 
 * 
 * @param start_node 
 * @param end_node 
 * @param min_radius 
 * @param consider_kappa_diff 
 * @param geometry_path 
 * @return true 
 * @return false 
 */
  bool GeneratePrecisePosePath(const common::PathPoint& start_node,
                               const common::PathPoint& end_node,
                               double min_radius, bool consider_kappa_diff,
                               GeometryPath* geometry_path);

  /**
 * @brief 
 * 
 * @param start_node 
 * @param end_node 
 * @param cycle_direction 
 * @param min_radius 
 * @param path_result 
 * @return true 
 * @return false 
 */
  bool GeneratePreciseAnglePath(const common::PathPoint& start_node,
                                const common::PathPoint& end_node,
                                const CycleDirection& cycle_direction,
                                double min_radius, GeometryPath* path_result);
  /**
 * @brief 
 * 
 * @param start_node 
 * @param end_node 
 * @param path_result 
 */
  bool LoadStraightPath(const common::PathPoint& start_node,
                        const common::PathPoint& end_node,
                        GeometryPath* path_result);
  /**
 * @brief 
 * 
 * @param start_node 
 * @param end_node 
 * @param path_result 
 */
  static bool LoadCyclePath(const common::PathPoint& start_node,
                            const common::math::Vec2d& end_point,
                            const common::math::Vec2d& radius_center,
                            double radius, double cycle_length,
                            GeometryPath* path_result);

  /**
   * @brief tranform result back relative by  start pose
   * 
   * @param start_node 
   * @param geometry_path 
   * @return true 
   * @return false 
   */
  static bool LocalCoordinateTransform(const common::PathPoint& start_node,
                                       GeometryPath* geometry_path);

  /**
   * @brief 
   * 
   * @param geometry_path 
   */
  static void ReversePath(GeometryPath* geometry_path);

 private:
  common::VehicleParam vehicle_param_;
  const WarmStartConfig warm_start_config_;
};
}  // namespace planning
}  // namespace TL
