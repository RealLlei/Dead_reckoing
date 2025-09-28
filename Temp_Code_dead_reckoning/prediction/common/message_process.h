/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 */

#pragma once

#include <memory>
#include <string>

#include "common/status/status.h"
#include "planning/localview/local_view.h"
#include "planning/prediction/container/container_manager.h"
#include "planning/prediction/container/pose/pose_container.h"
#include "planning/prediction/predictor/predictor_manager.h"
#include "planning/prediction/scenario/scenario_manager.h"
#include "planning/prediction/proto/prediction_conf.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/map/navigation.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"

namespace TL {
namespace prediction {

class MessageProcess {
 public:
  MessageProcess() = delete;

  static common::Status Init(ContainerManager* container_manager,
                             PredictorManager* predictor_manager,
                             InferenceManager* inference_manager,
                             const PredictionConf& prediction_conf);

  static common::Status InitContainers(ContainerManager* container_manager);

  static common::Status InitPredictors(PredictorManager* predictor_manager,
                                       InferenceManager* inference_manager,
                                       const PredictionConf& prediction_conf);

  static common::Status ContainerProcess(
      const std::shared_ptr<ContainerManager>& container_manager,
      const perception::PerceptionObstacles& perception_obstacles,
      ScenarioManager* scenario_manger);
  /**
   * @brief perception message process
   *
   */
  static common::Status OnPerception(
      const std::shared_ptr<TL::planning::LocalView>& local_view,
      const std::shared_ptr<ContainerManager>& container_manager,
      PredictorManager* predictor_manager, ScenarioManager* scenario_manager,
      PredictionObstacles* prediction_obstacles);

  static common::Status OnLocalization(
      ContainerManager* container_manager,
      const localization::Localization& localization,
      ScenarioManager* scenario_manager);

  static void OnPlanning(ContainerManager* container_manager,
                         const planning::ADCTrajectory& adc_trajectory);

  //   static void OnStoryTelling(ContainerManager *container_manager,
  //                              const storytelling::Stories &story);

  static ObstaclesContainer* ptr_obstacles_container_;
  static PoseContainer* ptr_ego_pose_container_;
  static ADCTrajectoryContainer* ptr_ego_trajectory_container_;
};

}  // namespace prediction
}  // namespace TL
