/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "common/math/vec2d.h"

#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/map/navigation.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
// NOLINTBEGIN
class NaviLaneCentralConstructorOld {
 public:
  NaviLaneCentralConstructorOld() = default;

  bool BuildReferenceLine(
      const double central_length, const double lane_width,
      const perception::LaneMarker& right_lane_marker,
      const perception::LaneMarker& left_lane_marker,
      std::vector<::TL::common::math::Vec2d>* const central_line_pionts,
      std::vector<::TL::common::math::Vec2d>* left_boundary,
      std::vector<::TL::common::math::Vec2d>* right_boundary,
      ::TL::common::math::Vec2d* delta_point);

 private:
  bool DoProjection(
      const std::vector<TL::common::math::Vec2d>& ref_v, const double& width,
      std::vector<TL::common::math::Vec2d>* const results) const;

  bool BuildCentralLine(
      const std::vector<::TL::common::math::Vec2d>& left_line,
      const std::vector<::TL::common::math::Vec2d>& right_line,
      const double& weight_left_boundary, const double& width,
      std::vector<::TL::common::math::Vec2d>* const central_pts) const;

  bool EvaulateBoundaries(
      const perception::LaneMarker& right_lanemarker,
      const perception::LaneMarker& left_lanemarker, const double& length,
      std::vector<::TL::common::math::Vec2d>* const left_vecs,
      std::vector<::TL::common::math::Vec2d>* const right_vecs,
      ::TL::common::math::Vec2d* point) const;

  double EvaulateBoundary(
      const perception::LaneMarker& lanmarker, const double& stepwise_factor,
      const double& rest_of_length,
      std::vector<TL::common::math::Vec2d>* line_pts) const;

  bool FindPerpendicular(const ::TL::common::math::Vec2d& p0,
                         const ::TL::common::math::Vec2d& p1,
                         const double& distance,
                         TL::common::math::Vec2d* res) const;

  double default_left_width_ = 1.875;
  double default_right_width_ = 1.875;
  double min_lane_half_width_ = 1.5;
  double max_lane_half_width_ = 2.0;

 private:
  mutable TL::common::math::Vec2d history_point_;
};

// NOLINTEND
}  // namespace planning
}  // namespace TL
