/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  scs_shape_path.h
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
#include "planning/common/open_space_info.h"
#include "planning/open_space/coarse_path_generator/path_generator.h"
#include "planning/proto/planner_open_space_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {

class SCSShapePath {
 public:
  SCSShapePath() = default;
  virtual ~SCSShapePath() = default;

  /**
   * @brief Generate SCS shape path, end pose x,y,phi, and radius can be optimized
   * 
   * @param start_node 
   * @param end_node 
   * @param dest_region_with_angle dest region, if region emptym,than should reach recise pose
   * @param min_radius 
   * @param is_park_out
   * @param geometry_path 
   * @return true 
   * @return false 
   */
  bool GenerateSCSShapePath(const common::PathPoint& start_node,
                            const common::PathPoint& end_node,
                            const DestRegionWithAng& dest_region_with_angle,
                            double min_radius, bool is_park_out,
                            PathGeneratorResult* path_result);

 private:
  template <typename T>
  void TransPointBasedOnQuadrantint(int quadrant, T* const p) {
    if (nullptr == p) {
      return;
    }
    switch (quadrant) {
      case 2: {
        p->set_x(-p->x());
        break;
      }
      case 3: {
        p->set_x(-p->x());
        p->set_y(-p->y());
        break;
      }
      case 4: {
        p->set_y(-p->y());
        break;
      }
      default:
        break;
    }
  }

  static double TransAngleBasedOnQuadrantint(const int quadrant,
                                             const double angle) {
    return (quadrant == 1 || quadrant == 3) ? angle : -angle;
  }

  /**
   * @brief 
   * 
   * @param origin 
   * @param end_node_ptr 
   * @param dest_region_with_angle_ptr 
   * @param quadrant 
   */
  void Trans2LocalCoor(const common::PathPoint& origin,
                       common::PathPoint* end_node_ptr,
                       DestRegionWithAng* dest_region_with_angle_ptr,
                       int* quadrant);

  /**
   * @brief key function of generate SCS shape path, 
   * The curve can be determined by obtaining the end position and radius.
   * 
   * @param dest_region_with_angle dest region
   * @param min_radius 
   * @param is_park_out
   * @param target_pose_ptr 
   * @param radius_ptr init with ref radius
   */
  bool GetSLSInfo(const DestRegionWithAng& dest_region_with_angle,
                  double min_radius, bool is_park_out,
                  common::PathPoint* target_pose_ptr, double* radius_ptr);

  /**
   * @brief 
   * 
   * @param is_collinear 
   * @param end_node 
   * @param radius 
   * @param geometry_path 
   * @return true 
   * @return false 
   */
  bool GenerateSLSPath(bool is_collinear, const common::PathPoint& end_node,
                       double radius, PathGeneratorResult* sls_path_ptr);
};
}  // namespace planning
}  // namespace TL
