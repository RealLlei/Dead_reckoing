/*
 * Copyright (c) TL auto Co., Ltd. 2023-2024. All rights reserved.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "common/interpolation/interpolation_1d.h"
#include "common/math/vec2d.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/vehicle_state_decider.h"
#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
namespace missilelane {
using TL::common::Interpolation1D;
using TL::common::math::Vec2d;
using TL::perception::LaneMarker;

class PointsFilter {
 public:
  PointsFilter() = delete;
  explicit PointsFilter(
      const planning::PerceptionMapConfig& config,
      const std::shared_ptr<MissileVehicleState>& vehicle_state);
  ~PointsFilter() = default;

  Status Init();
  void LoadKalmanGainScheduler(
      const planning::LanemarkerFilterConfig& filter_conf);
  std::vector<Vec2d> Filter(const LaneMarker& lanemarker);

 private:
  bool DoPredictUpdate(double view_range, std::vector<Vec2d>* history_points,
                       Eigen::MatrixXd* pre_points_x,
                       Eigen::MatrixXd* pre_points_y);
  bool DoKalmanGainUpdate(const Eigen::MatrixXd& pre_points_x,
                          const Eigen::MatrixXd& pre_points_y,
                          const Eigen::MatrixXd& measure_points_y,
                          double view_range, bool reset, double speed,
                          bool splitlane_flag, std::vector<Vec2d>* esti_points,
                          Eigen::MatrixXd* kalman_gain);
  void CreatInitLaneMarkerPoints(const LaneMarker& input_lanemarker,
                                 std::vector<Vec2d>* history_esti);
  static bool DoMeasureUpdate(const LaneMarker& copy_lanemarker,
                              const Eigen::MatrixXd& pre_points_x,
                              Eigen::MatrixXd* measure_points_y);
  const planning::PerceptionMapConfig& config_;
  std::shared_ptr<MissileVehicleState> vehicle_state_{nullptr};
  std::unique_ptr<Interpolation1D> length_interpolation_{nullptr};
  std::unique_ptr<Interpolation1D> diff_length_interpolation_{nullptr};
  std::vector<Vec2d> history_points_{};
  bool has_no_history_points_{true};
};
}  // namespace missilelane
}  // namespace planning
}  // namespace TL
