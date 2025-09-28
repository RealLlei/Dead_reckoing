#ifndef PLANNING_COMMON_SPEED_ST_BOUNDARY_H
#define PLANNING_COMMON_SPEED_ST_BOUNDARY_H

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
 *   @file
 **/

#pragma once

#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

// #include "gtest/gtest_prod.h"
#include "common/math/box2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "planning/common/speed/st_point.h"
#include "proto/planning/planning.pb.h"

namespace TL {
namespace planning {

class STBoundary : public common::math::Polygon2d {
 public:
  /** Constructors:
   *   STBoundary must be initialized with a vector of ST-point pairs.
   *   Each pair refers to a time t, with (lower_s, upper_s).
   */
  STBoundary() = default;
  explicit STBoundary(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs,
      bool is_accurate_boundary = false);
  explicit STBoundary(const common::math::Box2d& box) = delete;
  explicit STBoundary(std::vector<common::math::Vec2d> points) = delete;

  /** @brief Wrapper of the constructor (old).
   */
  static STBoundary CreateInstance(const std::vector<STPoint>& lower_points,
                                   const std::vector<STPoint>& upper_points);

  /** @brief Wrapper of the constructor. It doesn't use RemoveRedundantPoints
   * and generates an accurate ST-boundary.
   */
  static STBoundary CreateInstanceAccurate(
      const std::vector<STPoint>& lower_points,
      const std::vector<STPoint>& upper_points);

  [[nodiscard]] bool IsEmpty() const { return lower_points_.empty(); }

  bool GetUnblockSRange(double curr_time, double* s_upper,
                        double* s_lower) const;

  bool GetBoundarySRange(double curr_time, double* s_upper,
                         double* s_lower) const;

  bool GetBoundarySlopes(double curr_time, double* ds_upper,
                         double* ds_lower) const;

  // if you need to add boundary type, make sure you modify
  // GetUnblockSRange accordingly.
  enum class BoundaryType {
    UNKNOWN,
    STOP,
    FOLLOW,
    YIELD,
    OVERTAKE,
    KEEP_CLEAR,
    NUDGE
  };

  static std::string TypeName(BoundaryType type);

  [[nodiscard]] BoundaryType boundary_type() const;
  [[nodiscard]] const std::string& id() const;
  [[nodiscard]] double characteristic_length() const;

  void set_id(const std::string& id);
  void SetBoundaryType(const BoundaryType& boundary_type);
  void SetCharacteristicLength(double characteristic_length);

  [[nodiscard]] double min_s() const { return min_s_; }

  [[nodiscard]] double min_t() const { return min_t_; }

  [[nodiscard]] double max_s() const { return max_s_; }

  [[nodiscard]] double max_t() const { return max_t_; }

  [[nodiscard]] const std::vector<STPoint>& upper_points() const {
    return upper_points_;
  }

  [[nodiscard]] const std::vector<STPoint>& lower_points() const {
    return lower_points_;
  }

  void set_speed_points(std::vector<STPoint>&& speed_points) {
    speed_points_ = std::move(speed_points);
  }

  [[nodiscard]] const std::vector<STPoint>& speed_points() const {
    return speed_points_;
  }

  // Used by st-optimizer.
  [[nodiscard]] bool IsPointInBoundary(const STPoint& st_point) const;
  [[nodiscard]] STBoundary ExpandByS(double s) const;
  [[nodiscard]] STBoundary ExpandByT(double t) const;

  // Unused function so far.
  [[nodiscard]] STBoundary CutOffByT(double t) const;

  // Used by Lattice planners.
  [[nodiscard]] STPoint upper_left_point() const;
  [[nodiscard]] STPoint upper_right_point() const;
  [[nodiscard]] STPoint bottom_left_point() const;
  [[nodiscard]] STPoint bottom_right_point() const;

  void set_upper_left_point(STPoint st_point);
  void set_upper_right_point(STPoint st_point);
  void set_bottom_left_point(STPoint st_point);
  void set_bottom_right_point(STPoint st_point);

  void set_obstacle_road_right_ending_t(double road_right_ending_t) {
    obstacle_road_right_ending_t_ = road_right_ending_t;
  }

  [[nodiscard]] double obstacle_road_right_ending_t() const {
    return obstacle_road_right_ending_t_;
  }

  /** @brief Given time t, find a segment denoted by left and right idx, that
   * contains the time t.
   * - If t is less than all or larger than all, return false.
   */
  static bool GetIndexRange(const std::vector<STPoint>& points, double t,
                            size_t* left, size_t* right);
  // FRIEND_TEST(StBoundaryTest, get_index_range);

 private:
  /** @brief The sanity check function for a vector of ST-point pairs.
   */
  [[nodiscard]] static bool IsValid(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs);

  /** @brief Returns true if point is within max_dist distance to seg.
   */
  static bool IsPointNear(const common::math::LineSegment2d& seg,
                          const common::math::Vec2d& point, double max_dist);

  /** @brief Sometimes a sequence of upper and lower points lie almost on
   * two straightlines. In this case, the intermediate points are removed,
   * with only the end-points retained.
   */
  // TODO(all): When slope is high, this may introduce significant errors.
  // Also, when accumulated for multiple t, the error can get significant.
  // This function should be reconsidered, because it may be dangerous.
  static void RemoveRedundantPoints(
      std::vector<std::pair<STPoint, STPoint>>* point_pairs);
  // FRIEND_TEST(StBoundaryTest, remove_redundant_points);

  BoundaryType boundary_type_ = BoundaryType::UNKNOWN;

  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;
  std::vector<STPoint> speed_points_;

  std::string id_;
  double characteristic_length_ = 1.0;
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();

  STPoint bottom_left_point_;
  STPoint bottom_right_point_;
  STPoint upper_left_point_;
  STPoint upper_right_point_;

  double obstacle_road_right_ending_t_ = 0.0;
};

class SLTBoundary {
 public:
  enum class BoundaryType {
    SAFE_CAUTION,
    SPEED_LIMIT_CAUTION,
    NUDGE_CAUTION,
  };

  bool IsEmpty() const {
    return st_boundary_.IsEmpty() || lt_boundary_.IsEmpty();
  }

  const std::set<BoundaryType>& GetBoundaryTypes() const {
    return boundary_types_;
  }

  const STBoundary& GetSTBoundary() const { return st_boundary_; }

  const STBoundary& GetLTBoundary() const { return lt_boundary_; }

  const std::vector<STPoint>& GetDsPoints() const { return ds_points_; }

  const std::vector<STPoint>& GetDlPoints() const { return dl_points_; }

  const std::vector<STPoint>& GetThetaPoints() const { return theta_points_; }

  void SetBoundaryTypes(std::set<BoundaryType>&& boundary_types) {
    boundary_types_ = std::move(boundary_types);
  }

  void SetSTBoundary(STBoundary&& st_boundary) {
    st_boundary_ = std::move(st_boundary);  // NOLINT
  }

  void SetLTBoundary(STBoundary&& lt_boundary) {
    lt_boundary_ = std::move(lt_boundary);  // NOLINT
  }

  void SetThetaPoints(std::vector<STPoint>&& theta_points) {
    theta_points_ = std::move(theta_points);
  }

  void SetDsPoints(std::vector<STPoint>&& ds_points) {
    ds_points_ = std::move(ds_points);
  }

  void SetDlPoints(std::vector<STPoint>&& dl_points) {
    dl_points_ = std::move(dl_points);
  }

 private:
  std::set<BoundaryType> boundary_types_;
  STBoundary st_boundary_;
  STBoundary lt_boundary_;
  std::vector<STPoint> ds_points_;
  std::vector<STPoint> dl_points_;
  std::vector<STPoint> theta_points_;
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_SPEED_ST_BOUNDARY_H
