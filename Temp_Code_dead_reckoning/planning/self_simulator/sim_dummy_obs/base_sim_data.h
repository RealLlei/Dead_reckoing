/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#pragma once

#include <cmath>
#include <functional>
#include <memory>

#include "planning/localview/local_view.h"
#include "planning/self_simulator/sim_dummy_gflags.h"
#include "proto/localization/localization.pb.h"
#include "proto/map/navigation.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"

namespace TL {
namespace simdummy {
struct DummyObsInputDataBase {
  DummyObsInputDataBase();

  void Update(
      double t_delta_time,
      std::shared_ptr<const TL::planning::ADCTrajectory> t_traj,
      std::shared_ptr<common::PathPoint> adc_position,
      std::shared_ptr<TL::perception::PerceptionObstacles> t_perception,
      std::shared_ptr<TL::planning::LocalView> local_view,
      std::shared_ptr<hdmap::HDMap> map_ptr);

  double x1 = 0.0;
  double y2 = 0.0;
  double x3 = 0.0;

  double delta_time = 0.0;

  double left_lane_near_obstacle_speed =
      FLAGS_left_lane_near_obstacle_speed_km_h / 3.6;
  double left_lane_near_obstacle_s = FLAGS_left_lane_near_obstacle_display_dis;

  double front_obstacle_speed = FLAGS_front_obstacle_speed_km_h / 3.6;
  double front_obstacle_s = FLAGS_front_obstacle_display_dis;

  double left_lane_far_obstacle_speed =
      FLAGS_left_lane_far_obstacle_speed_km_h / 3.6;
  double left_lane_far_obstacle_s = FLAGS_left_lane_far_obstacle_display_dis;

  double right_lane_far_obstacle_speed =
      FLAGS_right_lane_far_obstacle_speed_km_h / 2.6;
  double right_lane_far_obstacle_s = FLAGS_right_lane_far_obstacle_display_dis;

  std::shared_ptr<common::PathPoint> adc_position_;
  std::shared_ptr<const TL::planning::ADCTrajectory> current_trajectory_;
  std::shared_ptr<TL::perception::PerceptionObstacles> perception;
  std::shared_ptr<TL::planning::LocalView> local_view_;
  std::shared_ptr<hdmap::HDMap> map_ptr_;
};

}  // namespace simdummy
}  // namespace TL
