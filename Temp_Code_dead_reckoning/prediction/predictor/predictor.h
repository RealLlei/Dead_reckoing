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
 * @file
 * @brief Define the predictor base class
 */

#pragma once

#include <list>
#include <string>
#include <vector>

#include "planning/prediction/common/prediction_util.h"
#include "planning/prediction/container/adc_trajectory/adc_trajectory_container.h"
#include "planning/prediction/container/obstacles/obstacle.h"
#include "planning/prediction/container/obstacles/obstacles_container.h"
#include "proto/prediction/feature.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"

/**
 * @namespace TL::prediction
 * @brief TL::prediction
 */
namespace TL {
namespace prediction {

class Predictor {
 public:
  /**
   * @brief Constructor
   */
  Predictor() = default;

  /**
   * @brief Destructor
   */
  virtual ~Predictor() = default;

  /**
   * @brief Make prediction
   * @param Obstacle pointer
   * @param Obstacles container
   * @return If predicted successfully
   */
  virtual bool Predict(const ADCTrajectoryContainer* adc_trajectory_container,
                       Obstacle* obstacle,
                       ObstaclesContainer* obstacles_container) = 0;

  /**
   * @brief Make prediction
   * @param Obstacles container
   * @return If predicted successfully
   */
  virtual bool PredictObstacles(
      const ADCTrajectoryContainer* adc_trajectory_container,
      const std::list<Obstacle*>& obstacles,
      ObstaclesContainer* obstacles_container) {
    bool result = true;
    for (const auto& obs : obstacles) {
      obs->mutable_latest_feature()->clear_predicted_trajectory();
      if (!Predict(adc_trajectory_container, obs, obstacles_container)) {
        ADEBUG << "Prediction failure with obstacle [" << obs->id() << "]";
        result = false;
      }
    }
    return result;
  }

  /**
   * @brief Get trajectory size
   * @return Size of trajectories
   */
  static int NumOfTrajectories(const Obstacle& obstacle);

  /**
   * @brief Clear all trajectories
   */
  virtual void Clear();

  /**
   * @brief Trim prediction trajectories by adc trajectory
   * @param ADC trajectory container
   * @param obstacle,
   */
  static void TrimTrajectories(
      const ADCTrajectoryContainer& adc_trajectory_container,
      Obstacle* obstacle);

  /**
   * @brief get the predictor type
   * @return the predictor type
   */
  const ObstacleConf::PredictorType& predictor_type();

 protected:
  /**
   * @brief Set equal probability to prediction trajectories
   * @param probability total probability
   * @param start_index The start index to set equal probability
   * @param obstacle
   */
  static void SetEqualProbability(double probability, int start_index,
                                  Obstacle* obstacle_ptr);

  /**
   * @brief Trim a single prediction trajectory,
   *        keep the portion that is not in junction.
   * @param adc_segments trajectory segments of ADC trajectory
   * @param obstacle
   * @param trajectory The trimed prediction trajectory
   * @return If the prediction trajectory is trimed
   */
  static bool TrimTrajectory(
      const ADCTrajectoryContainer& adc_trajectory_container,
      Obstacle* obstacle, Trajectory* trajectory);

  /**
   * @brief Determine if an obstacle is supposed to stop within a distance
   * @param The latest feature of obstacle
   * @param The distance to stop
   * @param The output param of acceleration
   * @return If the obstacle is supposed to stop within a distance
   */
  static bool SupposedToStop(const Feature& feature, double stop_distance,
                             double* acceleration);

  static void DrawFreeMoveTrajectoryPoints(const Eigen::Vector2d& position,
                                           const Eigen::Vector2d& velocity,
                                           const Eigen::Vector2d& acc,
                                           double theta, double start_time,
                                           double total_time, double period,
                                           double still_speed_th,
                                           prediction::Trajectory* trajectory);

  static bool DrawStitchFreemoveTrajectory(
      double period, prediction::Trajectory* trajectory,
      double total_time = FLAGS_prediction_trajectory_time_length_freemove);

  static bool DrawTrajFollowEgoLane(const Obstacle& ego_obs, double total_time,
                                    double period, bool is_extend,
                                    Trajectory* trajectory, Obstacle* obstacle);

  static void ExtrapolateByLane(double obs_speed,
                                const std::string& start_lane_id,
                                Trajectory* trajectory_ptr);

  static bool DrawCubicBezierTrajectory(const Feature& feature,
                                        const Eigen::Vector2d& end_p,
                                        double end_heading, double total_time,
                                        double period, Trajectory* trajectory);

  ObstacleConf::PredictorType GetPredictorType() const {
    return predictor_type_;
  }

  void SetPredictorType(ObstacleConf::PredictorType predictor_type) {
    predictor_type_ = predictor_type;
  }

 private:
  ObstacleConf::PredictorType predictor_type_ = ObstacleConf::EMPTY_PREDICTOR;
};

}  // namespace prediction
}  // namespace TL
