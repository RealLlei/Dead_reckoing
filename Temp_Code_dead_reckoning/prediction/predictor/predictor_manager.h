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
 * @brief Use predictor manager to manage all predictors
 */

#pragma once

#include <list>
#include <map>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/status/status.h"
#include "planning/prediction/container/obstacles/obstacles_container.h"
#include "planning/prediction/inference_manager.h"
#include "planning/prediction/predictor/predictor.h"
#include "planning/prediction/scenario/scenario_manager.h"
#include "planning/prediction/proto/prediction_conf.pb.h"
#include "proto/planning/planning_internal.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"
#include "proto/prediction/scenario.pb.h"

#ifdef ISMDC
#elif defined(ISORIN)
#else
#include <onnxruntime_cxx_api.h>
#endif

/**
 * @namespace TL::prediction
 * @brief TL::prediction
 */
namespace TL {
namespace prediction {
using TL::perception::PerceptionObstacle;

class PredictorManager {
 public:
  /**
   * @brief Constructor
   */
  PredictorManager();

  /**
   * @brief Destructor
   */
  virtual ~PredictorManager() = default;

  /**
   * @brief Initializer
   * @param Prediction config
   */
  void Init(InferenceManager* inference_manager, const PredictionConf& config);

  /**
   * @brief Get predictor
   * @return Pointer to the predictor
   */
  Predictor* GetPredictor(const ObstacleConf::PredictorType& type);

  /**
   * @brief Execute the predictor generation
   * @param Adc trajectory container
   * @param Obstacles container
   */
  common::Status Run(
      const TL::perception::PerceptionObstacles& perception_obstacles,
      const ADCTrajectoryContainer* adc_trajectory_container,
      PoseContainer* pose_container, ObstaclesContainer* obstacles_container,
      ScenarioManager* scenario_manager,
      PredictionObstacles* prediction_obstacles);

 private:
  /**
   * @brief Register a predictor by type
   * @param Predictor type
   */
  void RegisterPredictor(const ObstacleConf::PredictorType& type);

  /**
   * @brief Create a predictor by type
   * @param Predictor type
   * @return A unique pointer to the predictor
   */
  std::unique_ptr<Predictor> CreatePredictor(
      const ObstacleConf::PredictorType& type);

  /**
   * @brief Register all predictors
   */
  void RegisterPredictors();

  std::list<ObstacleConf::PredictorType> SelectPredictors(
      Obstacle* obstacle, ScenarioManager* scenario_manager);

  common::Status DispatchObstacleToPredictors(
      const std::list<ObstacleConf::PredictorType>& predictor_list,
      Obstacle* obstacle);

  /**
   * @brief Predict a single obstacle
   * @param A pointer to adc_trajectory_container
   * @param A pointer to the specific obstacle
   * @param A pointer to the obstacles container
   * @param A pointer to prediction_obstacle
   * @param A pointer to scenario_manager
   */
  common::Status PostProcessObstacle(
      const ADCTrajectoryContainer* adc_trajectory_container,
      Obstacle* obstacle, ObstaclesContainer* obstacles_container,
      PredictionObstacle* prediction_obstacle);

  common::Status PredictObstacles(
      const TL::perception::PerceptionObstacles& perception_obstacles,
      const ADCTrajectoryContainer* adc_trajectory_container,
      PoseContainer* pose_container, ObstaclesContainer* obstacles_container,
      ScenarioManager* scenario_manager,
      PredictionObstacles* prediction_obstacles);

  common::Status PredictObstaclesInParallel(
      const TL::perception::PerceptionObstacles& perception_obstacles,
      const ADCTrajectoryContainer* adc_trajectory_container,
      ObstaclesContainer* obstacles_container);

  void RunFreeMovePredictor(
      const ADCTrajectoryContainer* adc_trajectory_container,
      Obstacle* obstacle, ObstaclesContainer* obstacles_container);

  static void FilterPredictionTrajectory(
      Obstacle* obstacle, PredictionObstacle* prediction_obstacle);

 private:
  void RecordDebugData(Obstacle* obstacle,
                       PredictionObstacle* prediction_obstacle,
                       PredictionObstacles* prediction_obstacles);

  InferenceManager* inference_manager_;

  std::map<ObstacleConf::PredictorType, std::unique_ptr<Predictor>> predictors_;

  std::map<std::tuple<TL::prediction::Scenario::Type,
                      perception::PerceptionObstacle::Type,
                      ObstacleConf::ObstacleStatus>,
           std::list<ObstacleConf::PredictorType>>
      predictor_configs_;

  std::unordered_map<perception::PerceptionObstacle::Type,
                     perception::PerceptionObstacle::Type>
      obstacle_subtype_map_;

  std::vector<
      std::unordered_map<ObstacleConf::PredictorType, std::list<Obstacle*>>>
      prediction_pipeline_;
  std::vector<std::pair<Obstacle*, const PerceptionObstacle*>> obstacle_list_;
};

}  // namespace prediction
}  // namespace TL
