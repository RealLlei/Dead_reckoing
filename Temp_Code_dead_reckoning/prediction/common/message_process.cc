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

#include "planning/prediction/common/message_process.h"

#include <memory>

#include "common/adapters/adapter_gflags.h"
#include "common/file/file.h"
#include "common/time/clock.h"
#include "common/util/perf_util.h"
#include "planning/prediction/common/feature_output.h"
#include "planning/prediction/common/junction_analyzer.h"
#include "planning/prediction/common/prediction_constants.h"
#include "planning/prediction/common/prediction_gflags.h"
#include "planning/prediction/common/prediction_system_gflags.h"
#include "planning/prediction/common/validation_checker.h"
#include "planning/prediction/container/pose/pose_container.h"
#include "planning/prediction/inference_manager.h"
#include "planning/prediction/predictor/predictor_manager.h"
#include "planning/prediction/util/data_extraction.h"
#include "proto/common/error_code.pb.h"

namespace TL {
namespace prediction {

using TL::common::Status;
using TL::common::adapter::AdapterConfig;
using TL::perception::PerceptionObstacle;

ObstaclesContainer* MessageProcess::ptr_obstacles_container_ = nullptr;
PoseContainer* MessageProcess::ptr_ego_pose_container_ = nullptr;
ADCTrajectoryContainer* MessageProcess::ptr_ego_trajectory_container_ = nullptr;

Status MessageProcess::Init(ContainerManager* container_manager,
                            PredictorManager* predictor_manager,
                            InferenceManager* inference_manager,
                            const PredictionConf& prediction_conf) {
  auto containers_status = InitContainers(container_manager);
  auto predictors_status =
      InitPredictors(predictor_manager, inference_manager, prediction_conf);

  // if (!FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
  //   AERROR << "Map cannot be loaded.";
  //   return false;
  // }
  // if (!PredictionMap::Ready()) {
  //   AERROR << "Map cannot be loaded.";
  //   return false;
  // }
  ptr_obstacles_container_ =
      container_manager->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  ptr_ego_pose_container_ = container_manager->GetContainer<PoseContainer>(
      AdapterConfig::LOCALIZATION);
  ptr_ego_trajectory_container_ =
      container_manager->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);

  if (!containers_status.ok() || !predictors_status.ok() ||
      ptr_obstacles_container_ == nullptr ||
      ptr_ego_pose_container_ == nullptr ||
      ptr_ego_trajectory_container_ == nullptr) {
    return Status(common::ErrorCode::PREDICTION_CONTAINER_ERROR);
  }

  return Status(common::ErrorCode::OK);
}

Status MessageProcess::InitContainers(ContainerManager* container_manager) {
  common::adapter::AdapterManagerConfig adapter_conf;
  if (!common::GetProtoFromFile(FLAGS_prediction_adapter_config_filename,
                                &adapter_conf)) {
    return Status(common::ErrorCode::PREDICTION_CONTAINER_ERROR,
                  "Unable to load adapter conf file");
  }
  ADEBUG << "Adapter config file is loaded into: "
         << adapter_conf.ShortDebugString();

  container_manager->Init(adapter_conf);
  return Status(common::ErrorCode::OK);
}

Status MessageProcess::InitPredictors(PredictorManager* predictor_manager,
                                      InferenceManager* inference_manager,
                                      const PredictionConf& prediction_conf) {
  predictor_manager->Init(inference_manager, prediction_conf);
  return Status(common::ErrorCode::OK);
}

Status MessageProcess::ContainerProcess(
    const std::shared_ptr<ContainerManager>& container_manager,
    const perception::PerceptionObstacles& perception_obstacles,
    ScenarioManager* scenario_manager) {
  UNUSED(container_manager);
  ADEBUG << "Received a perception message ["
         << perception_obstacles.ShortDebugString() << "].";

  ptr_obstacles_container_->CleanUp();
  ptr_obstacles_container_->ClearCluster();

  // ptr_obstacles_container->Insert(perception_obstacles);
  // Insert ADC into the obstacle_container.
  const PerceptionObstacle* ptr_ego_vehicle =
      ptr_ego_pose_container_->ToPerceptionObstacle();
  if (ptr_ego_vehicle != nullptr) {
    double perception_obs_timestamp = ptr_ego_vehicle->timestamp();
    if (perception_obstacles.has_header() &&
        perception_obstacles.header().has_data_stamp()) {
      ADEBUG << "Correcting " << FIXED << SETPRECISION(6)
             << ptr_ego_vehicle->timestamp() << " to "
             << perception_obstacles.header().data_stamp();
      perception_obs_timestamp = perception_obstacles.header().data_stamp();
    }
    ptr_obstacles_container_->InsertPerceptionObstacle(
        *ptr_ego_vehicle, perception_obs_timestamp, scenario_manager);
    double x = ptr_ego_vehicle->position().x();
    double y = ptr_ego_vehicle->position().y();
    ADEBUG << "Get ADC position [" << FIXED << SETPRECISION(6) << x << ", " << y
           << "].";
    ptr_ego_trajectory_container_->SetPosition({x, y});
  }
  // Insert perception_obstacles
  ptr_obstacles_container_->Insert(perception_obstacles, scenario_manager);

  ptr_obstacles_container_->BuildJunctionMap();
  ptr_obstacles_container_->BuildJunctionFeature();

  return Status(common::ErrorCode::OK);
}

Status MessageProcess::OnPerception(
    const std::shared_ptr<TL::planning::LocalView>& local_view,
    const std::shared_ptr<ContainerManager>& container_manager,
    PredictorManager* predictor_manager, ScenarioManager* scenario_manager,
    PredictionObstacles* prediction_obstacles) {
  const auto& perception_obstacles = local_view->GetPerceptionObstacles();
  auto* ptr_obstacles_container =
      container_manager->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  auto status = Status(common::ErrorCode::OK);

  if (nullptr == ptr_obstacles_container) {
    return Status(common::ErrorCode::PREDICTION_CONTAINER_ERROR);
  }

  double pre_start_timestamp = common::Clock::NowInSeconds();
  status = ContainerProcess(container_manager, *perception_obstacles,
                            scenario_manager);

  // Scenario analysis
  scenario_manager->ProcessContainer(container_manager);

  // // Build junction feature for the obstacles in junction
  // const Scenario scenario = scenario_manager->scenario();
  // if (scenario.type() == Scenario::JUNCTION && scenario.has_junction_id()) {
  //   ptr_obstacles_container->GetJunctionAnalyzer()->Init(
  //       scenario.junction_id());
  //   ptr_obstacles_container->BuildJunctionFeature();
  // }

  double pre_end_timestamp = common::Clock::NowInSeconds();
  if (!status.ok()) {
    return status;
  }

  ADEBUG << "prediction container period = "
         << (pre_end_timestamp - pre_start_timestamp) * 1000;

  auto* ptr_ego_trajectory_container =
      container_manager->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  auto* ptr_ego_pose_container = container_manager->GetContainer<PoseContainer>(
      AdapterConfig::LOCALIZATION);
  if (nullptr == ptr_ego_trajectory_container ||
      nullptr == ptr_ego_pose_container) {
    return Status(common::ErrorCode::PREDICTION_CONTAINER_ERROR);
  }

  pre_start_timestamp = common::Clock::NowInSeconds();

  // Make predictions
  status = predictor_manager->Run(
      *perception_obstacles, ptr_ego_trajectory_container,
      ptr_ego_pose_container, ptr_obstacles_container, scenario_manager,
      prediction_obstacles);
  pre_end_timestamp = common::Clock::NowInSeconds();

  ADEBUG << "predictor period = "
         << (pre_end_timestamp - pre_start_timestamp) * 1000;

  // Insert features to FeatureOutput for offline_mode
  if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpFeatureProto ||
      FLAGS_enable_online_record4Prediction) {
    for (const int id :
         ptr_obstacles_container->curr_frame_movable_obstacle_ids()) {
      Obstacle* obstacle_ptr = ptr_obstacles_container->GetObstacle(id);
      if (obstacle_ptr == nullptr) {
        AERROR << "Null obstacle found.";
        continue;
      }
      if (!obstacle_ptr->latest_feature().IsInitialized()) {
        AERROR << "Obstacle [" << id << "] has no latest feature.";
        continue;
      }

      FeatureOutput::InsertFeatureProto(obstacle_ptr->latest_feature());
      ADEBUG << "Insert feature into feature output";
    }

    // insert ego feature
    Obstacle* ego_ptr =
        ptr_obstacles_container->GetObstacle(FLAGS_ego_vehicle_id);
    if (ego_ptr == nullptr) {
      AERROR << "Null obstacle found.";
    } else {
      FeatureOutput::InsertFeatureProto(ego_ptr->latest_feature());
    }

    // Not doing evaluation on offline mode
    if (!FLAGS_enable_online_record4Prediction) {
      return status;
    }
  }

  Obstacle* ego_ptr =
      ptr_obstacles_container->GetObstacle(FLAGS_ego_vehicle_id);

  if (ego_ptr == nullptr) {
    AERROR << "Null obstacle found.";
  } else {
    const auto& ego_feat = ego_ptr->latest_feature();
    auto* out_ego_feat = prediction_obstacles->mutable_feature();
    out_ego_feat->mutable_position()->CopyFrom(ego_feat.position());
    out_ego_feat->mutable_velocity()->CopyFrom(ego_feat.velocity());
    out_ego_feat->set_velocity_heading(ego_feat.velocity_heading());
    out_ego_feat->set_theta(ego_feat.theta());
    out_ego_feat->set_speed(ego_feat.speed());
    if (ego_ptr->latest_feature().has_vector_net_feature()) {
      out_ego_feat->mutable_vector_net_feature()->CopyFrom(
          ego_ptr->latest_feature().vector_net_feature());
    }
  }
  return status;
}

Status MessageProcess::OnLocalization(
    ContainerManager* container_manager,
    const localization::Localization& localization,
    ScenarioManager* scenario_manager) {
  auto* ptr_ego_pose_container = container_manager->GetContainer<PoseContainer>(
      AdapterConfig::LOCALIZATION);
  if (ptr_ego_pose_container == nullptr) {
    return Status(common::ErrorCode::PREDICTION_CONTAINER_ERROR);
  }

  if (scenario_manager->VehicleReferenceFrame()) {
    const auto& perception_localization =
        PoseContainer::PurePerceptionTransform(localization);
    ptr_ego_pose_container->Insert(perception_localization);
  } else {
    ptr_ego_pose_container->Insert(localization);
  }
  return Status(common::ErrorCode::OK);
}

void MessageProcess::OnPlanning(ContainerManager* container_manager,
                                const planning::ADCTrajectory& adc_trajectory) {
  auto* ptr_ego_trajectory_container =
      container_manager->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  ptr_ego_trajectory_container->Insert(adc_trajectory);

  ADEBUG << "Received a planning message [" << adc_trajectory.ShortDebugString()
         << "].";

  // auto ptr_storytelling_container =
  //     container_manager->GetContainer<StoryTellingContainer>(
  //         AdapterConfig::STORYTELLING);
  // CHECK_NOTNULL(ptr_storytelling_container);
  // ptr_ego_trajectory_container->SetJunction(
  //     ptr_storytelling_container->ADCJunctionId(),
  //     ptr_storytelling_container->ADCDistanceToJunction());
}

// void MessageProcess::OnStoryTelling(ContainerManager* container_manager,
//                                     const Stories& story) {
//   auto ptr_storytelling_container =
//       container_manager->GetContainer<StoryTellingContainer>(
//           AdapterConfig::STORYTELLING);
//   CHECK_NOTNULL(ptr_storytelling_container);
//   ptr_storytelling_container->Insert(story);

//   ADEBUG << "Received a storytelling message [" << story.ShortDebugString()
//          << "].";
// }

}  // namespace prediction
}  // namespace TL
