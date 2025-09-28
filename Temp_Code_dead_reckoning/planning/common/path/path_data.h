#ifndef PLANNING_COMMON_PATH_PATH_DATA_H
#define PLANNING_COMMON_PATH_PATH_DATA_H

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
 * @file path_data.h
 **/

#pragma once

#include <cstddef>
#include <limits>
#include <list>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "common/math/line_segment2d.h"
#include "planning/common/open_space_info.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/path/frenet_frame_path.h"
#include "planning/reference_line/reference_line.h"

namespace TL {
namespace planning {

class PathData {
 public:
  enum class PathPointType {
    IN_LANE,
    OUT_ON_FORWARD_LANE,
    OUT_ON_REVERSE_LANE,
    OFF_ROAD,
    UNKNOWN,
  };

  enum class RoadRightType : size_t {
    HIGH_ROAD_RIGHT = 0,
    LANE_TURN_LEFT_LOW_ROAD_RIGHT = 1,
    LANE_TURN_RIGHT_LOW_ROAD_RIGHT = 2,
    LANE_CROSS_LEFT_LOW_ROAD_RIGHT = 4,
    LANE_CROSS_RIGHT_LOW_ROAD_RIGHT = 8,
  };

  struct PathEnvelope {
    double min_ref_s = std::numeric_limits<double>::max();
    double max_ref_s = std::numeric_limits<double>::lowest();
    double min_ref_l = std::numeric_limits<double>::max();
    double max_ref_l = std::numeric_limits<double>::lowest();
  };

  enum PathCautionDirection {
    LEFT,
    RIGHT,
  };

  struct PathCautionEnvelope {
    PathCautionDirection direction = PathCautionDirection::LEFT;
    PathEnvelope envelope;
  };

  struct PathFrenetInfo {
    // obstacle longitudinal speed
    double cos_theta = 1.0;
    // obstacle longitudinal speed
    double sin_theta = 1.0;
    // obstacle boundary s_lower
    double s_lower = std::numeric_limits<double>::max();
    // obstacle boundary s_upper
    double s_upper = std::numeric_limits<double>::lowest();
    // obstacle boundary l_lower
    double l_lower = std::numeric_limits<double>::max();
    // obstacle boundary l_upper
    double l_upper = std::numeric_limits<double>::lowest();
  };

  PathData() = default;

  bool SetDiscretizedPath(DiscretizedPath path);

  bool SetFrenetPath(FrenetFramePath frenet_path);

  void SetReferenceLine(const ReferenceLine* reference_line);

  bool SetPathPointDecisionGuide(
      std::vector<std::tuple<double, PathPointType, double>>
          path_point_decision_guide);

  const DiscretizedPath& discretized_path() const;

  const FrenetFramePath& frenet_frame_path() const;

  const std::vector<std::tuple<double, PathPointType, double>>&
  path_point_decision_guide() const;

  common::PathPoint GetPathPointWithPathS(double s) const;

  /*
   * brief: this function will find the path_point in discretized_path whose
   * projection to reference line has s value closest to ref_s.
   */
  bool GetPathPointWithRefS(double ref_s, common::PathPoint* path_point) const;

  bool LeftTrimWithRefS(const common::FrenetFramePoint& frenet_point);

  bool UpdateFrenetFramePath(const ReferenceLine* reference_line);

  void Clear();

  bool Empty() const;

  std::string DebugString() const;

  void set_path_label(const std::string& label);

  const std::string& path_label() const;

  void set_blocking_obstacle_id(const std::string& obs_id) {
    blocking_obstacle_id_ = obs_id;
  }

  const std::string& blocking_obstacle_id() const {
    return blocking_obstacle_id_;
  }

  bool is_valid_path_reference() const { return is_valid_path_reference_; }

  void set_is_valid_path_reference(bool is_valid_path_reference) {
    is_valid_path_reference_ = is_valid_path_reference;
  }

  bool is_optimized_towards_trajectory_reference() const {
    return is_optimized_towards_trajectory_reference_;
  }

  void set_is_optimized_towards_trajectory_reference(
      bool is_optimized_towards_trajectory_reference) {
    is_optimized_towards_trajectory_reference_ =
        is_optimized_towards_trajectory_reference;
  }

  bool is_beyond_bound_optimize_result() const {
    return is_beyond_bound_optimize_result_;
  }

  void set_is_beyond_bound_optimize_result(
      bool is_beyond_bound_optimize_result) {
    is_beyond_bound_optimize_result_ = is_beyond_bound_optimize_result;
  }

  const std::vector<common::PathPoint>& path_reference() const;
  void set_path_reference(const std::vector<common::PathPoint>& path_reference);

  bool GetRefSWithPathS(double path_s, double* ref_s) const;

  /**
   * @brief Get the Path Road Right object
   * 
   * @return const std::vector<RoadRightType>& 
   */
  [[nodiscard]] const std::vector<RoadRightType>& GetPathRoadRight() const {
    return path_road_right_;
  }

  /**
   * @brief Set the Path Road Right object
   * 
   * @param path_road_right 
   */
  void SetPathRoadRight(const std::vector<RoadRightType>& path_road_right) {
    path_road_right_ = path_road_right;
  }

  [[nodiscard]] const std::vector<PathFrenetInfo>& GetPathFrenetInfos() const {
    return path_frenet_infos_;
  }

  void SetPathFrenetInfos(std::vector<PathFrenetInfo>&& path_frenet_infos) {
    path_frenet_infos_ = std::move(path_frenet_infos);
  }

  [[nodiscard]] const PathEnvelope& GetPathEnvelope() const {
    return path_envelope_;
  }

  void SetPathEnvelope(const PathEnvelope& path_envelope) {
    path_envelope_ = path_envelope;
  }

  [[nodiscard]] const std::vector<PathCautionEnvelope>&
  GetPathCautionEnvelopes() const {
    return path_caution_envelopes_;
  }

  void SetPathCautionEnvelopes(
      std::vector<PathCautionEnvelope>&& path_caution_envelopes) {
    path_caution_envelopes_ = std::move(path_caution_envelopes);
  }

  std::vector<FreeSpaceSegment>* MutableFreeSpaceSegments() {
    return &freespace_segments_;
  }

  const std::vector<FreeSpaceSegment>& GetFreeSpaceSegments() const {
    return freespace_segments_;
  }

  void SetOffsetCompensation(const common::PathPoint& shift_point);

 private:
  /*
   * convert frenet path to cartesian path by reference line
   */
  bool SLToXY(const FrenetFramePath& frenet_path,
              DiscretizedPath* discretized_path);
  bool XYToSL(const DiscretizedPath& discretized_path,
              FrenetFramePath* frenet_path);
  const ReferenceLine* reference_line_ = nullptr;
  DiscretizedPath discretized_path_;
  FrenetFramePath frenet_path_;
  /**
   * @brief speed decision generated by path analyzer for guiding speed limit
   * generation in speed bounds decider
   * @param tuple consists of s axis position on reference line; PathPointType
   * Enum; distance to closest obstacle
   */
  std::vector<std::tuple<double, PathPointType, double>>
      path_point_decision_guide_;

  std::string path_label_;
  std::string blocking_obstacle_id_;

  /**
   * @brief parameters for using the learning model output as a path reference
   *
   */
  // wheter this PathData is a path reference serving as an optimization target
  // for later modules
  bool is_valid_path_reference_ = false;

  /**
   * @brief Given a trajectory reference, whether this PathData is optimized
   * according to the "path" part of the trajectory so that "speed" part of the
   * trajectory could be used in later modules accordingly
   *
   */
  bool is_optimized_towards_trajectory_reference_ = false;

  /**
   * @brief whether optimized path is beyond bound, 
   * if beyond all the obstacles should do collision check 
   * or just block obstacle need do coolision check
   *
   */
  bool is_beyond_bound_optimize_result_ = false;

  // path reference
  std::vector<common::PathPoint> path_reference_;

  // path road right
  std::vector<RoadRightType> path_road_right_;
  std::vector<PathFrenetInfo> path_frenet_infos_;
  std::vector<PathCautionEnvelope> path_caution_envelopes_;
  PathEnvelope path_envelope_;
  std::vector<FreeSpaceSegment> freespace_segments_;
};

inline constexpr PathData::RoadRightType operator|(
    PathData::RoadRightType lhs, PathData::RoadRightType rhs) {
  return static_cast<PathData::RoadRightType>(static_cast<size_t>(lhs) |
                                              static_cast<size_t>(rhs));
}

inline constexpr PathData::RoadRightType operator&(
    PathData::RoadRightType lhs, PathData::RoadRightType rhs) {
  return static_cast<PathData::RoadRightType>(static_cast<size_t>(lhs) &
                                              static_cast<size_t>(rhs));
}

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_PATH_PATH_DATA_H
