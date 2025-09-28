/*
 * Copyright (c) TL auto Co., Ltd. 2023-2024. All rights reserved.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "common/status/status.h"
#include "planning/common/obstacle.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/vehicle_state_decider.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/obs_history_points_generator.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/obstacles_state.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "planning/localview/local_view.h"
#include "common/filters/digital_filter.h"
#include "common/filters/digital_filter_coefficients.h"

namespace TL {
namespace planning {
namespace missilelane {
using TL::common::Status;

class ObstacleDecider {
 public:
  ObstacleDecider() = default;
  ~ObstacleDecider() = default;
  Status Init(const std::shared_ptr<ObstaclesState>& obstacles_state_ptr);
  bool UpdateFollowingObs(const std::shared_ptr<LocalView>& local_view,
                          functionmanager::FunctionManagerOut* to_fct);

  const std::vector<Vec2d>& GetObsPoints() {
    return history_points_generator_->GetObsPoints();
  }

  const TL::perception::LaneMarker& GetObsLanemarker() {
    return history_points_generator_->GetObsLanemarker();
  }

  perception::PerceptionObstacle& FollowingObs() { return following_obs_; }

  void GeneratePoints(common::Path* init_points);

  double half_lane_width() const {
    return (history_points_generator_ == nullptr)
               ? 1.875
               : history_points_generator_->half_lane_width();
  }

 private:
  bool DeciderObsBeforeVehicle();
  static double CalculateObsY(const TL::perception::LaneMarker& lane_marker,
                              double obs_x, double obs_y);

  std::shared_ptr<const perception::PerceptionObstacles> perception_obstacles_{
      nullptr};
  std::shared_ptr<ObsHisPointsGenerator> history_points_generator_{nullptr};
  TL::common::DigitalFilter heading_filter_;
  std::shared_ptr<MissileVehicleState> vehicle_state_{nullptr};
  perception::PerceptionObstacle following_obs_{};
  std::vector<Vec2d> following_obs_points_{};
  // bool obs_theta_flu_state_{true};
};

}  // namespace missilelane
}  // namespace planning
}  // namespace TL
