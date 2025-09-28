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

#include "planning/prediction/predictor/free_move/free_move_predictor.h"

#include "planning/prediction/common/prediction_gflags.h"
#include "planning/prediction/proto/prediction_conf.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/prediction/feature.pb.h"

namespace TL {
namespace prediction {

using TL::common::TrajectoryPoint;
using TL::perception::PerceptionObstacle;  // NOLINT

FreeMovePredictor::FreeMovePredictor() {
  SetPredictorType(ObstacleConf::FREE_MOVE_PREDICTOR);
}

bool FreeMovePredictor::Predict(
    const ADCTrajectoryContainer* adc_trajectory_container, Obstacle* obstacle,
    ObstaclesContainer* obstacles_container) {
  UNUSED(adc_trajectory_container);
  UNUSED(obstacles_container);
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  obstacle->AddPredictorType(GetPredictorType());

  const Feature& feature = obstacle->latest_feature();

  if (!feature.has_position() || !feature.has_velocity() ||
      !feature.position().has_x() || !feature.position().has_y()) {
    AERROR << "Obstacle [" << obstacle->id()
           << " is missing position or velocity";
    return false;
  }

  double prediction_total_time =
      FLAGS_prediction_trajectory_time_length_freemove;

  double still_speed_threshold = FLAGS_still_obstacle_speed_threshold;
  if (feature.type() == perception::PerceptionObstacle::PEDESTRIAN) {
    still_speed_threshold = FLAGS_still_pedestrian_speed_threshold;
  }

  if (std::hypot(feature.velocity().x(), feature.velocity().y()) <
      still_speed_threshold) {
    obstacle->mutable_latest_feature()->set_is_still(true);
    return true;
  }

  if (feature.predicted_trajectory().empty()) {
    Trajectory trajectory;
    Eigen::Vector2d position(feature.position().x(), feature.position().y());
    Eigen::Vector2d velocity(feature.velocity().x(), feature.velocity().y());
    Eigen::Vector2d acc(feature.acceleration().x(), feature.acceleration().y());
    double theta = feature.velocity_heading();

    DrawFreeMoveTrajectoryPoints(position, velocity, acc, theta, 0.0,
                                 prediction_total_time,
                                 FLAGS_prediction_trajectory_time_resolution,
                                 still_speed_threshold, &trajectory);

    obstacle->mutable_latest_feature()->add_predicted_trajectory()->Swap(
        &trajectory);
    SetEqualProbability(1.0, 0, obstacle);
  } else {
    for (int i = 0; i < feature.predicted_trajectory_size(); ++i) {
      Trajectory* trajectory =
          obstacle->mutable_latest_feature()->mutable_predicted_trajectory(i);
      if (trajectory->trajectory_point().empty()) {
        AERROR
            << "Empty predicted trajectory found, add current point as start";
        auto* trajectory_point = trajectory->add_trajectory_point();
        auto* path_point = trajectory_point->mutable_path_point();
        path_point->set_x(feature.position().x());
        path_point->set_y(feature.position().y());
        path_point->set_theta(feature.velocity_heading());
        trajectory_point->set_v(feature.speed());
        trajectory_point->set_a(feature.acc());
        trajectory_point->set_relative_time(0.0);
      }
      int traj_size = trajectory->trajectory_point_size();
      Trajectory trajectory_copy;
      const TrajectoryPoint& last_point =
          trajectory->trajectory_point(traj_size - 1);
      double theta = last_point.path_point().theta();
      Eigen::Vector2d position(last_point.path_point().x(),
                               last_point.path_point().y());
      Eigen::Vector2d velocity(last_point.v() * std::cos(theta),
                               last_point.v() * std::sin(theta));
      Eigen::Vector2d acc(last_point.a() * std::cos(theta),
                          last_point.a() * std::sin(theta));
      double last_relative_time = last_point.relative_time();
      DrawFreeMoveTrajectoryPoints(position, velocity, acc, theta,
                                   last_relative_time,
                                   prediction_total_time - last_relative_time,
                                   FLAGS_prediction_trajectory_time_resolution,
                                   still_speed_threshold, &trajectory_copy);
      // The following for-loop starts from index 1 because the vector points
      // includes the last point in the existing predicted trajectory
      trajectory->MergeFrom(trajectory_copy);
    }
  }
  return true;
}

}  // namespace prediction
}  // namespace TL
