#ifndef PLANNING_COMMON_PATH_DISCRETIZED_PATH_H
#define PLANNING_COMMON_PATH_DISCRETIZED_PATH_H

/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file discretized_path.h
 **/

#pragma once

#include <utility>
#include <vector>

#include "common/math/box2d.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/planning/sl_boundary.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

static constexpr double kDisThreshold = 1e-2;
static constexpr double kAngThreshold = 0.02;

class DiscretizedPath : public std::vector<common::PathPoint> {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<common::PathPoint> path_points);

  double Length() const;

  double TotalLength() const;

  common::PathPoint Evaluate(double path_s) const;

  common::PathPoint EvaluateForGreaterThanMaxS(double path_s) const;

  common::PathPoint EvaluateReverse(double path_s) const;

  /**
   * @brief get box sl boundary fourth corner
   *
   * @param box
   * @param sl_boundary
   * @return true
   * @return false
   */
  bool GetSLBoundary(const common::math::Box2d& box,
                     SLBoundary* sl_boundary) const;

  /**
   * @brief x y to s l
   *
   * @param x
   * @param y
   * @param sl_point
   * @return true
   * @return false
   */
  bool XYToSL(double x, double y, common::SLPoint* sl_point) const;
  /**
   * @brief Generate the Stop Path object
   * 
   * @param x 
   * @param y 
   * @param theta 
   * @param kappa
   */
  void GenerateStopPath(double x, double y, double theta, double kappa);

  /**
   * @brief check is point in path
   * 
   * @param point 
   * @return int 
   */
  bool IsPointIn(const common::PathPoint& point,
                 double dis_threshold = kDisThreshold,
                 double ang_threshold = kAngThreshold) const;

  /**
   * @brief check is same point or not
   * 
   * @param p_a 
   * @param p_b 
   * @return true 
   * @return false 
   */
  static bool IsSamePoint(const common::PathPoint& p_a,
                          const common::PathPoint& p_b,
                          double dis_threshold = kDisThreshold,
                          double ang_threshold = kAngThreshold);

  bool GetProjection(const common::math::Vec2d& point, double* accumulate_s,
                     double* lateral, double* min_distance, int* index_min,
                     double radius1d, int index_center) const;

 protected:
  std::vector<common::PathPoint>::const_iterator QueryLowerBound(
      double path_s) const;
  std::vector<common::PathPoint>::const_iterator QueryUpperBound(
      double path_s) const;
  std::vector<double> accumulated_s_{};
  std::vector<common::math::LineSegment2d> segments_{};
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_PATH_DISCRETIZED_PATH_H
