/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#pragma once

#include <memory>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "common/math/linear_interpolation.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"

#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/map/navigation.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
using common::Status;
using TL::common::math::Vec2d;
using TL::perception::LaneMarker;
// 0:point 1.heading 2.kappa 3.dkappa
using EndPonitTuple = std::tuple<Vec2d, double, double, double>;

class NaviLaneCentralConstructor {
 public:
  NaviLaneCentralConstructor() = default;

  TL::common::Status Init();

  Status BuildReferenceLine(
      double central_length, double lane_width,
      const LaneMarker& right_lane_marker, const LaneMarker& left_lane_marker,
      std::vector<Vec2d>* left_points, std::vector<Vec2d>* right_points,
      std::vector<Vec2d>* central_line_pionts,
      std::vector<Vec2d>* left_boundary, std::vector<Vec2d>* right_boundary,
      EndPonitTuple* end_point_tuple, Vec2d* delta_front_point,
      bool is_central_lane = false);
  bool ExtendNextReferenceLine(const std::vector<Vec2d>& ref_boundary_points,
                               double lane_width,
                               std::vector<Vec2d>* central_pionts,
                               std::vector<Vec2d>* extend_boundary_points);

  void ClearHistoryPoint() {
    history_end_point_tuple_ = std::make_tuple(Vec2d(), 0.0, 0.0, 0.0);
    history_front_point_ = Vec2d();
  }

 private:
  static Status DoProjection(const std::vector<Vec2d>& ref_v,
                             const double& width, std::vector<Vec2d>* results);

  Status BuildCentralLine(const std::vector<Vec2d>& left_line,
                          const std::vector<Vec2d>& right_line,
                          const double& weight_left_boundary,
                          const double& width,
                          std::vector<Vec2d>* central_pts) const;

  Status EvaulateBoundaries(const double& length,
                            std::vector<Vec2d>* left_input_points,
                            std::vector<Vec2d>* right_input_points,
                            std::vector<Vec2d>* left_vecs,
                            std::vector<Vec2d>* right_vecs,
                            EndPonitTuple* end_point_tuple,
                            Vec2d* delta_front_point,
                            const perception::LaneMarker& average_lane_marker,
                            bool is_central_lane, double lane_width);

  static double EvaulateBoundary(const perception::LaneMarker& lanmarker,
                                 const double& stepwise_factor,
                                 const double& rest_of_length,
                                 std::vector<Vec2d>* line_pts);

  static bool FindPerpendicular(const Vec2d& p0, const Vec2d& p1,
                                const double& distance, Vec2d* res);

  double EvaulateLanemarkerLength(std::vector<Vec2d>* boundary_points,
                                  double start_position,
                                  double end_position) const;
  static bool AddStartAndEndPoints(std::vector<Vec2d>* boundary_points,
                                   double start_x, double end_x,
                                   double step_length);
  static double CalculateVec2dLength(const std::vector<Vec2d>& input_points);

  Status LaneWidthCheck(std::vector<Vec2d>* left_points,
                        std::vector<Vec2d>* right_points, double min_lane_width,
                        std::vector<Vec2d>* ori_left_points,
                        std::vector<Vec2d>* ori_right_points,
                        double min_length);
  static void RemoveRedundantPoint(std::vector<Vec2d>* ori_left_points,
                                   std::vector<Vec2d>* ori_right_points);
  static EndPonitTuple GetEndPointTupleByCoefficient(
      const EndPonitTuple& tuple_first, const EndPonitTuple& tuple_second,
      double k = 0.5);
  static bool DoubleHasSameSign(double first, double second);

 private:
  mutable EndPonitTuple history_end_point_tuple_{};
  mutable TL::common::math::Vec2d history_front_point_;
  TL::planning::lanelineprocess::DebounceModule lane_width_debounce_;
  bool lane_width_state_last_{};
};
}  // namespace planning
}  // namespace TL
