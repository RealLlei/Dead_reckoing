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

#include "planning/prediction/predictor/predictor_manager.h"

#include <cstddef>
#include <list>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>

#include "common/file/log.h"
#include "common/math/angle.h"
#include "common/math/linear_interpolation.h"
#include "common/status/status.h"
#include "planning/prediction/common/feature_output.h"
#include "planning/prediction/common/prediction_constants.h"
#include "planning/prediction/common/prediction_gflags.h"
#include "planning/prediction/common/prediction_system_gflags.h"
#include "planning/prediction/common/prediction_thread_pool.h"
#include "planning/prediction/container/container_manager.h"
#include "planning/prediction/container/obstacles/obstacle.h"
#include "planning/prediction/container/pose/pose_container.h"
#include "planning/prediction/predictor/cost/cost_predictor.h"
#include "planning/prediction/predictor/dbn/dbn_junction_predictor.h"
#include "planning/prediction/predictor/dbn/dbn_predictor.h"
#include "planning/prediction/predictor/empty/empty_predictor.h"
#include "planning/prediction/predictor/extrapolation/extrapolation_predictor.h"
#include "planning/prediction/predictor/follow_ego_lane/follow_ego_lane_predictor.h"
#include "planning/prediction/predictor/follow_ego_lane/follow_urban_ego_lane_predictor.h"
#include "planning/prediction/predictor/free_move/free_move_predictor.h"
#include "planning/prediction/predictor/vectornet/tnt_predictor.h"
// #include "planning/prediction/predictor/interaction/interaction_predictor.h"
// #include "planning/prediction/predictor/junction/junction_predictor.h"
// #include "planning/prediction/predictor/lane_sequence/lane_sequence_predictor.h"
#include "planning/prediction/predictor/ego/self_predictor.h"
#include "planning/prediction/predictor/move_sequence/move_sequence_predictor.h"
#include "planning/prediction/predictor/pedestrian/pedestrian_trajnet_predictor.h"
#include "planning/prediction/predictor/static/static_predictor.h"
// #include "planning/prediction/predictor/single_lane/single_lane_predictor.h"
#include "planning/prediction/proto/prediction_conf.pb.h"
#include "proto/common/error_code.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"
#include "proto/prediction/scenario.pb.h"

#ifndef ISMDC
#include "planning/prediction/predictor/qcnet/qcnet_predictor.h"
#endif

namespace TL {
namespace prediction {
namespace {

using TL::common::Status;
using TL::perception::PerceptionObstacle;
using TL::perception::PerceptionObstacles;
using IdObstacleListMap = std::unordered_map<int, std::list<Obstacle*>>;

void GroupObstaclesByObstacleId(const int obstacle_id,
                                ObstaclesContainer* const obstacles_container,
                                IdObstacleListMap* const id_obstacle_map) {
  Obstacle* obstacle_ptr = obstacles_container->GetObstacle(obstacle_id);
  if (obstacle_ptr == nullptr) {
    AERROR << "Null obstacle [" << obstacle_id << "] found";
    return;
  }
  int id_mod = obstacle_id % FLAGS_max_thread_num;
  (*id_obstacle_map)[id_mod].push_back(obstacle_ptr);
}

}  // namespace

PredictorManager::PredictorManager() : inference_manager_(nullptr) {}

void PredictorManager::RegisterPredictors() {
  // RegisterPredictor(ObstacleConf::LANE_SEQUENCE_PREDICTOR);
  RegisterPredictor(ObstacleConf::MOVE_SEQUENCE_PREDICTOR);
  // RegisterPredictor(ObstacleConf::SINGLE_LANE_PREDICTOR);
  RegisterPredictor(ObstacleConf::FREE_MOVE_PREDICTOR);
  RegisterPredictor(ObstacleConf::EMPTY_PREDICTOR);
  // RegisterPredictor(ObstacleConf::JUNCTION_PREDICTOR);
  RegisterPredictor(ObstacleConf::EXTRAPOLATION_PREDICTOR);
  // RegisterPredictor(ObstacleConf::INTERACTION_PREDICTOR);
  RegisterPredictor(ObstacleConf::FOLLOW_EGO_LANE_PREDICTOR);
  RegisterPredictor(ObstacleConf::FOLLOW_URBAN_EGO_LANE_PREDICTOR);
  RegisterPredictor(ObstacleConf::COST_PREDICTOR);
  RegisterPredictor(ObstacleConf::PEDESTRIAN_TRAJNET_PREDICTOR);
  RegisterPredictor(ObstacleConf::STATIC_PREDICTOR);
  RegisterPredictor(ObstacleConf::TNT_PREDICTOR);
  RegisterPredictor(ObstacleConf::DBN_PREDICTOR);
  RegisterPredictor(ObstacleConf::SELF_PREDICTOR);
#ifndef ISMDC
  RegisterPredictor(ObstacleConf::QCNET_PREDICTOR);
#endif
  RegisterPredictor(ObstacleConf::DBN_JUNCTION_PREDICTOR);
}

void PredictorManager::Init(InferenceManager* inference_manager,
                            const PredictionConf& config) {
  inference_manager_ = inference_manager;

  RegisterPredictors();

  for (const auto& conf : config.obstacle_conf()) {
    if (!conf.has_obstacle_type()) {
      AERROR << "Obstacle config [" << conf.ShortDebugString()
             << "] has not defined obstacle type.";
      continue;
    }

    if (conf.predictor_type_size() == 0) {
      AERROR << "Obstacle config [" << conf.ShortDebugString()
             << "] has not defined predictor type.";
      continue;
    }

    std::list<ObstacleConf::PredictorType> predictor_list;
    for (const auto& predictor_type : conf.predictor_type()) {
      predictor_list.push_back(
          static_cast<ObstacleConf::PredictorType>(predictor_type));
    }

    predictor_configs_[std::make_tuple(
        conf.scenario_type(), conf.obstacle_type(), conf.obstacle_status())] =
        predictor_list;
  }
  obstacle_subtype_map_[PerceptionObstacle::CYCLIST] =
      PerceptionObstacle::BICYCLE;
}

Predictor* PredictorManager::GetPredictor(
    const ObstacleConf::PredictorType& type) {
  auto it = predictors_.find(type);
  return it != predictors_.end() ? it->second.get() : nullptr;
}

Status PredictorManager::Run(
    const PerceptionObstacles& perception_obstacles,
    const ADCTrajectoryContainer* adc_trajectory_container,
    PoseContainer* pose_container, ObstaclesContainer* obstacles_container,
    ScenarioManager* scenario_manager,
    PredictionObstacles* prediction_obstacles) {
  if (nullptr == prediction_obstacles) {
    return Status(common::PREDICTION_PREDICTOR_ERROR,
                  "prediction_obstacles is nullptr");
  }
  auto status = Status::OK();
  status = PredictObstacles(perception_obstacles, adc_trajectory_container,
                            pose_container, obstacles_container,
                            scenario_manager, prediction_obstacles);

  return status;
}

Status PredictorManager::DispatchObstacleToPredictors(
    const std::list<ObstacleConf::PredictorType>& predictor_list,
    Obstacle* obstacle) {
  if (predictor_list.empty()) {
    return Status(common::ErrorCode::PREDICTION_PREDICTOR_ERROR);
  }
  size_t current_step = 0;
  for (const auto& predictor_type : predictor_list) {
    if (prediction_pipeline_.size() > current_step) {
      auto& tasks = prediction_pipeline_.at(current_step);
      tasks[predictor_type].emplace_back(obstacle);
    } else if (prediction_pipeline_.size() == current_step) {
      std::unordered_map<ObstacleConf::PredictorType, std::list<Obstacle*>>
          new_task;
      new_task[predictor_type].emplace_back(obstacle);
      prediction_pipeline_.emplace_back(new_task);
    } else {
      AERROR << "configure prediction pipeline with incorrect size["
             << prediction_pipeline_.size() << "]";
      return Status(common::ErrorCode::PREDICTION_PREDICTOR_ERROR);
    }
    current_step++;
  }
  return Status::OK();
}

Status PredictorManager::PredictObstacles(
    const PerceptionObstacles& perception_obstacles,
    const ADCTrajectoryContainer* adc_trajectory_container,
    PoseContainer* pose_container, ObstaclesContainer* obstacles_container,
    ScenarioManager* scenario_manager,
    PredictionObstacles* prediction_obstacles) {
  auto status = Status::OK();
  obstacle_list_.clear();
  prediction_pipeline_.clear();

  if ("fusion" == perception_obstacles.header().frame_id()) {
    DBNPredictor::SetObstacleFrom(true);
  } else {
    DBNPredictor::SetObstacleFrom(false);
  }
  DBNPredictor::SetPilotType(scenario_manager->PilotType());

  Obstacle* ego_ptr = obstacles_container->GetObstacle(FLAGS_ego_vehicle_id);
  if (ego_ptr != nullptr) {
    const PerceptionObstacle* ego_perception_obstacle =
        pose_container->ToPerceptionObstacle();
    PredictionObstacle* prediction_obstacle =
        prediction_obstacles->add_prediction_obstacle();
    prediction_obstacle->mutable_perception_obstacle()->CopyFrom(
        *ego_perception_obstacle);
    prediction_obstacle->set_timestamp(ego_ptr->timestamp());
    if (FLAGS_enable_prediction_debug && ego_ptr->history_size() > 0) {
      RecordDebugData(ego_ptr, prediction_obstacle, prediction_obstacles);
    }
  }

  auto cmp =
      [](const std::tuple<Obstacle*, const PerceptionObstacle*, double>& a,
         const std::tuple<Obstacle*, const PerceptionObstacle*, double>& b) {
        return std::get<2>(a) > std::get<2>(b);
      };
  std::priority_queue<
      std::tuple<Obstacle*, const PerceptionObstacle*, double>,
      std::vector<std::tuple<Obstacle*, const PerceptionObstacle*, double>>,
      decltype(cmp)>
      prioritized_obstacles(cmp);

  for (const PerceptionObstacle& perception_obstacle :
       perception_obstacles.perception_obstacle()) {
    int id = perception_obstacle.id();
    if (id < 0) {
      ADEBUG << "The obstacle has invalid id [" << id << "].";
      continue;
    }
    Obstacle* obstacle = obstacles_container->GetObstacle(id);

    // if obstacle == nullptr, that means obstacle is unmovable
    // Checkout the logic of unmovable in obstacle.cc
    if (obstacle != nullptr && obstacle->history_size() > 0) {
      const auto& feat = obstacle->latest_feature();
      double distance =
          feat.has_position_flu()
              ? std::hypot(feat.position_flu().x(), feat.position_flu().y())
              : 0.0;
      prioritized_obstacles.emplace(obstacle, &perception_obstacle, distance);
    } else {  // obstacle == nullptr
      PredictionObstacle* prediction_obstacle =
          prediction_obstacles->add_prediction_obstacle();
      prediction_obstacle->mutable_perception_obstacle()->CopyFrom(
          perception_obstacle);
      prediction_obstacle->set_timestamp(perception_obstacle.timestamp());
      prediction_obstacle->set_is_static(true);
      prediction_obstacle->mutable_track_status()->set_is_tracking(true);
      prediction_obstacle->mutable_track_status()->set_tracking_time(
          perception_obstacle.tracking_time());
      prediction_obstacle->mutable_track_status()->set_valid_tracking_time(
          perception_obstacle.tracking_time());
      int tracking_counter =
          std::isless(perception_obstacle.tracking_time(), 0.1)
              ? 1
              : static_cast<int>(perception_obstacle.tracking_time() * 10);
      prediction_obstacle->mutable_track_status()->set_tracking_counter(
          tracking_counter);
      prediction_obstacle->mutable_track_status()->set_valid_tracking_counter(
          tracking_counter);
      prediction_obstacle->mutable_track_status()->set_motion_tracking_time(
          perception_obstacle.tracking_time());
      prediction_obstacle->mutable_track_status()->set_motion_tracking_counter(
          prediction_obstacle->track_status().valid_tracking_counter());

      prediction_obstacle->set_predicted_period(
          FLAGS_prediction_trajectory_time_length);
      if (FLAGS_enable_prediction_debug && obstacle != nullptr &&
          obstacle->history_size() > 0) {
        RecordDebugData(obstacle, prediction_obstacle, prediction_obstacles);
      }
    }
  }

  auto QC_capacity = QCNETPredictor::DeviceInferenceCapacity();
  while (!prioritized_obstacles.empty()) {
    auto item = prioritized_obstacles.top();
    std::list<ObstacleConf::PredictorType> predictor_list =
        SelectPredictors(std::get<0>(item), scenario_manager);
    if (ObstacleConf::QCNET_PREDICTOR == predictor_list.front()) {
      if (QC_capacity > 0) {
        QC_capacity--;
      } else {
        predictor_list.clear();
        predictor_list.emplace_back(static_cast<ObstacleConf::PredictorType>(
            ObstacleConf::DBN_PREDICTOR));
        predictor_list.emplace_back(static_cast<ObstacleConf::PredictorType>(
            ObstacleConf::MOVE_SEQUENCE_PREDICTOR));
      }
    }
    status = DispatchObstacleToPredictors(predictor_list, std::get<0>(item));
    obstacle_list_.emplace_back(std::get<0>(item), std::get<1>(item));
    prioritized_obstacles.pop();
  }

  for (auto& tasks : prediction_pipeline_) {
    for (auto& task : tasks) {
      Predictor* predictor = GetPredictor(task.first);
      if (nullptr == predictor) {
        AERROR << "Nullptr found for predictor type [" << task.first << "]";
        predictor = GetPredictor(ObstacleConf::FREE_MOVE_PREDICTOR);
      }
      predictor->PredictObstacles(adc_trajectory_container, task.second,
                                  obstacles_container);
    }
  }

  for (auto& obs : obstacle_list_) {
    PredictionObstacle* prediction_obstacle =
        prediction_obstacles->add_prediction_obstacle();
    prediction_obstacle->mutable_perception_obstacle()->CopyFrom(*obs.second);
    prediction_obstacle->mutable_perception_obstacle()->set_type(
        obs.first->type());
    prediction_obstacle->mutable_track_status()->set_is_tracking(true);
    prediction_obstacle->mutable_track_status()->set_tracking_time(
        obs.second->tracking_time());
    status = PostProcessObstacle(adc_trajectory_container, obs.first,
                                 obstacles_container, prediction_obstacle);
    prediction_obstacle->set_predicted_period(
        FLAGS_prediction_trajectory_time_length);
    if (FLAGS_enable_prediction_debug && obs.first != nullptr &&
        obs.first->history_size() > 0) {
      RecordDebugData(obs.first, prediction_obstacle, prediction_obstacles);
    }
  }

  if (ego_ptr != nullptr) {
    PredictionObstacle* prediction_obstacle =
        prediction_obstacles->mutable_prediction_obstacle(0);
    if (ego_ptr->latest_feature().predicted_trajectory().empty()) {
      Predictor* predictor = GetPredictor(ObstacleConf::QCNET_PREDICTOR);
      if (predictor != nullptr) {
        std::list<Obstacle*> obstacle_list;
        obstacle_list.emplace_back(ego_ptr);
        predictor->PredictObstacles(adc_trajectory_container, obstacle_list,
                                    obstacles_container);
      }
    }
    prediction_obstacle->mutable_trajectory()->Swap(
        ego_ptr->mutable_latest_feature()->mutable_predicted_trajectory());
  }
  // ego_ptr->BuildVectorNetGraph();
  // Predictor* predictor = GetPredictor(ObstacleConf::SELF_PREDICTOR);
  // std::set<std::string> routing_lane_set = scenario_manager->RoutingLaneIds();
  // if (nullptr != predictor && !routing_lane_set.empty() &&
  //     ego_ptr->latest_feature().has_vector_net_feature()) {
  //   VectorNet::AppendRoutingLaneGraph(
  //       routing_lane_set,
  //       ego_ptr->mutable_latest_feature()->mutable_vector_net_feature());
  //   std::list<Obstacle*> ego_list;
  //   ego_list.emplace_back(ego_ptr);
  //   if (!predictor->PredictObstacles(adc_trajectory_container, ego_list,
  //                                    obstacles_container)) {
  //     AERROR << "ego vehicle self trajectory prediction failed";
  //   }
  // PredictionObstacle prediction_obstacle;
  // for (const auto& trajectory :
  //      obstacle->latest_feature().predicted_trajectory()) {
  //   prediction_obstacle->add_trajectory()->CopyFrom(trajectory);
  // }
  // prediction_obstacle.mutable_track_status()->set_is_tracking(true);
  // prediction_obstacle.set_predicted_period(
  //     FLAGS_prediction_trajectory_time_length);
  // prediction_obstacles_.add_prediction_obstacle()->CopyFrom(
  //     prediction_obstacle);
  //}
  //}

  DBNPredictor::RemoveNotExistsObs(perception_obstacles);

  return status;
}

Status PredictorManager::PredictObstaclesInParallel(
    const PerceptionObstacles& perception_obstacles,
    const ADCTrajectoryContainer* adc_trajectory_container,
    ObstaclesContainer* obstacles_container) {
  auto status = Status::OK();
  std::unordered_map<int, std::shared_ptr<PredictionObstacle>>
      id_prediction_obstacle_map;
  for (const PerceptionObstacle& perception_obstacle :
       perception_obstacles.perception_obstacle()) {
    int id = perception_obstacle.id();
    id_prediction_obstacle_map[id] = std::make_shared<PredictionObstacle>();
  }
  IdObstacleListMap id_obstacle_map;
  for (const auto& perception_obstacle :
       perception_obstacles.perception_obstacle()) {
    int id = perception_obstacle.id();
    Obstacle* obstacle = obstacles_container->GetObstacle(id);
    if (obstacle == nullptr) {
      std::shared_ptr<PredictionObstacle> prediction_obstacle_ptr =
          id_prediction_obstacle_map[id];
      prediction_obstacle_ptr->set_is_static(true);
      prediction_obstacle_ptr->set_timestamp(perception_obstacle.timestamp());
    } else {
      GroupObstaclesByObstacleId(id, obstacles_container, &id_obstacle_map);
    }
  }
  PredictionThreadPool::ForEach(
      id_obstacle_map.begin(), id_obstacle_map.end(),
      [&](IdObstacleListMap::iterator::value_type& obstacles_iter) {
        for (auto* obstacle_ptr : obstacles_iter.second) {
          int id = obstacle_ptr->id();
          status = PostProcessObstacle(adc_trajectory_container, obstacle_ptr,
                                       obstacles_container,
                                       id_prediction_obstacle_map[id].get());
        }
      });
  for (const PerceptionObstacle& perception_obstacle :
       perception_obstacles.perception_obstacle()) {
    int id = perception_obstacle.id();
    auto prediction_obstacle_ptr = id_prediction_obstacle_map[id];
    if (prediction_obstacle_ptr == nullptr) {
      AERROR << "Prediction obstacle [" << id << "] not found.";
      continue;
    }
    prediction_obstacle_ptr->set_predicted_period(
        FLAGS_prediction_trajectory_time_length);
    prediction_obstacle_ptr->mutable_perception_obstacle()->CopyFrom(
        perception_obstacle);
    // prediction_obstacles_.add_prediction_obstacle()->CopyFrom(
    //     *prediction_obstacle_ptr);
  }
  return status;
}

std::list<ObstacleConf::PredictorType> PredictorManager::SelectPredictors(
    Obstacle* obstacle, ScenarioManager* scenario_manager) {
  std::list<ObstacleConf::PredictorType> predictor_list;

  if (obstacle->ToIgnore() ||
      (obstacle->IsStill() && !obstacle->IsPedestrian())) {
    ADEBUG << "Ignore obstacle [" << obstacle->id() << "]";
    predictor_list.push_back(static_cast<ObstacleConf::PredictorType>(
        ObstacleConf::EMPTY_PREDICTOR));
  } else {
    auto obstacle_type = obstacle->type();
    if (obstacle_subtype_map_.find(obstacle_type) !=
        obstacle_subtype_map_.end()) {
      obstacle_type = obstacle_subtype_map_[obstacle_type];
    }
    auto itr = predictor_configs_.find(
        std::make_tuple(scenario_manager->scenario().type(), obstacle_type,
                        obstacle->GetObstacleStatus()));
    if (itr != predictor_configs_.end()) {
      predictor_list = itr->second;
    } else {
      AERROR << "Obstacle [" << obstacle->id()
             << "] in unconfigured scenario, run freemove predictor"
             << "scenario: " << scenario_manager->scenario().type()
             << " obstacle_type: " << obstacle_type
             << " obstacle_status: " << obstacle->GetObstacleStatus();
      predictor_list.push_back(static_cast<ObstacleConf::PredictorType>(
          ObstacleConf::FREE_MOVE_PREDICTOR));
    }
  }
  return predictor_list;
}

Status PredictorManager::PostProcessObstacle(
    const ADCTrajectoryContainer* adc_trajectory_container, Obstacle* obstacle,
    ObstaclesContainer* obstacles_container,
    PredictionObstacle* const prediction_obstacle) {
  if (obstacle == nullptr) {
    return Status(common::ErrorCode::PREDICTION_PREDICTOR_ERROR);
  }
  if (PredictionConstants::kDumpRecord == FLAGS_prediction_offline_mode) {
    Obstacle* ego_ptr = obstacles_container->GetObstacle(FLAGS_ego_vehicle_id);
    if (ego_ptr != nullptr &&
        !ego_ptr->latest_feature().has_vector_net_feature()) {
      ego_ptr->BuildVectorNetGraph();
    }
  }
  if (obstacle->ToIgnore()) {
    prediction_obstacle->mutable_priority()->set_priority(
        ObstaclePriority::IGNORE);
  }
  prediction_obstacle->set_timestamp(obstacle->timestamp());
  // fallback
  if (!obstacle->ToIgnore() && !obstacle->IsStill()) {
    bool need_fallback = true;
    for (const auto& traj : obstacle->latest_feature().predicted_trajectory()) {
      need_fallback = false;
      if (traj.trajectory_point().empty()) {
        need_fallback = true;
        break;
      }
    }
    if (need_fallback) {
      ADEBUG << "No trajectory predicted for obstacle [" << obstacle->id()
             << "]. Use free move predictor as fallback. "
             << obstacle->obstacle_conf().ShortDebugString();
      RunFreeMovePredictor(adc_trajectory_container, obstacle,
                           obstacles_container);
    }
  }
  prediction_obstacle->mutable_trajectory()->Swap(
      obstacle->mutable_latest_feature()->mutable_predicted_trajectory());

  bool is_static = obstacle->IsStill();
  int tracking_counter = obstacle->history_size();

  prediction_obstacle->set_timestamp(obstacle->timestamp());
  prediction_obstacle->mutable_priority()->CopyFrom(
      obstacle->latest_feature().priority());
  prediction_obstacle->mutable_intent()->set_type(
      obstacle->latest_feature().intent().type());
  prediction_obstacle->set_is_static(is_static);

  prediction_obstacle->mutable_track_status()->set_creation_time(
      obstacle->feature(0).timestamp());
  prediction_obstacle->mutable_track_status()->set_tracking_counter(
      tracking_counter);

  bool find_is_static_changed = false;
  for (int i = 1; i < tracking_counter; ++i) {
    const auto& feat = obstacle->feature(tracking_counter - 1 - i);
    if (feat.is_still() != is_static) {
      find_is_static_changed = true;
      prediction_obstacle->mutable_track_status()->set_motion_tracking_time(
          obstacle->latest_feature().timestamp() - feat.timestamp());
      prediction_obstacle->mutable_track_status()->set_motion_tracking_counter(
          tracking_counter);
      break;
    }
  }
  if (!find_is_static_changed) {
    prediction_obstacle->mutable_track_status()->set_motion_tracking_time(
        obstacle->latest_feature().timestamp() -
        obstacle->feature(0).timestamp());
    prediction_obstacle->mutable_track_status()->set_motion_tracking_counter(
        tracking_counter);
  }

  if (obstacle->tracking_frame() > 0) {
    if (obstacle->tracking_frame() <= obstacle->history_size()) {
      const auto& valid_tracking_feat = obstacle->feature(
          obstacle->history_size() - obstacle->tracking_frame());
      prediction_obstacle->mutable_track_status()->set_valid_tracking_time(
          obstacle->latest_feature().timestamp() -
          valid_tracking_feat.timestamp());
      prediction_obstacle->mutable_track_status()->set_valid_tracking_counter(
          obstacle->tracking_frame());
    } else {
      prediction_obstacle->mutable_track_status()->set_valid_tracking_time(
          obstacle->latest_feature().timestamp() -
          obstacle->feature(0).timestamp());
      prediction_obstacle->mutable_track_status()->set_valid_tracking_counter(
          obstacle->history_size());
    }
  }

  prediction_obstacle->mutable_currentlane_ids()->CopyFrom(
      obstacle->mutable_latest_feature()->currentlane_ids());
  prediction_obstacle->mutable_behavior_infos()->CopyFrom(
      obstacle->mutable_latest_feature()->behavior_infos());

  return Status(common::ErrorCode::OK);
}

std::unique_ptr<Predictor> PredictorManager::CreatePredictor(
    const ObstacleConf::PredictorType& type) {
  std::unique_ptr<Predictor> predictor_ptr(nullptr);
  switch (type) {
    // case ObstacleConf::LANE_SEQUENCE_PREDICTOR: {
    //   predictor_ptr = std::make_unique<LaneSequencePredictor>();
    //   break;
    // }
    case ObstacleConf::MOVE_SEQUENCE_PREDICTOR: {
      predictor_ptr = std::make_unique<MoveSequencePredictor>();
      break;
    }
    // case ObstacleConf::SINGLE_LANE_PREDICTOR: {
    //   predictor_ptr = std::make_unique<SingleLanePredictor>();
    //   break;
    // }
    case ObstacleConf::FREE_MOVE_PREDICTOR: {
      predictor_ptr = std::make_unique<FreeMovePredictor>();
      break;
    }
    // case ObstacleConf::JUNCTION_PREDICTOR: {
    //   predictor_ptr = std::make_unique<JunctionPredictor>();
    //   break;
    // }
    case ObstacleConf::EXTRAPOLATION_PREDICTOR: {
      predictor_ptr = std::make_unique<ExtrapolationPredictor>();
      break;
    }
    // case ObstacleConf::INTERACTION_PREDICTOR: {
    //   predictor_ptr = std::make_unique<InteractionPredictor>();
    //   break;
    // }
    case ObstacleConf::EMPTY_PREDICTOR: {
      predictor_ptr = std::make_unique<EmptyPredictor>();
      break;
    }
    case ObstacleConf::FOLLOW_EGO_LANE_PREDICTOR: {
      predictor_ptr = std::make_unique<LaneFollowPredictor>();
      break;
    }
    case ObstacleConf::FOLLOW_URBAN_EGO_LANE_PREDICTOR: {
      predictor_ptr = std::make_unique<UrbanLaneFollowPredictor>();
      break;
    }
    case ObstacleConf::COST_PREDICTOR: {
      predictor_ptr = std::make_unique<CostPredictor>();
      break;
    }
    case ObstacleConf::PEDESTRIAN_TRAJNET_PREDICTOR: {
      predictor_ptr =
          std::make_unique<PedestrianTrajNetPredictor>(inference_manager_);
      break;
    }
    case ObstacleConf::TNT_PREDICTOR: {
      predictor_ptr = std::make_unique<TNTPredictor>(inference_manager_);
      break;
    }
    case ObstacleConf::STATIC_PREDICTOR: {
      predictor_ptr = std::make_unique<StaticPredictor>();
      break;
    }
    case ObstacleConf::DBN_PREDICTOR: {
      predictor_ptr = std::make_unique<DBNPredictor>();
      break;
    }
    case ObstacleConf::SELF_PREDICTOR: {
      predictor_ptr = std::make_unique<SelfPredictor>(inference_manager_);
      break;
    }
#ifndef ISMDC
    case ObstacleConf::QCNET_PREDICTOR: {
      predictor_ptr = std::make_unique<QCNETPredictor>(inference_manager_);
      break;
    }
#endif
    case ObstacleConf::DBN_JUNCTION_PREDICTOR: {
      predictor_ptr = std::make_unique<DBNJunctionPredictor>();
      break;
    }

    default: {
      break;
    }
  }

  return predictor_ptr;
}

void PredictorManager::RegisterPredictor(
    const ObstacleConf::PredictorType& type) {
  predictors_[type] = CreatePredictor(type);
  AINFO << "Predictor [" << type << "] is registered.";
}

void PredictorManager::RunFreeMovePredictor(
    const ADCTrajectoryContainer* adc_trajectory_container, Obstacle* obstacle,
    ObstaclesContainer* obstacles_container) {
  Predictor* predictor = nullptr;
  predictor = GetPredictor(ObstacleConf::FREE_MOVE_PREDICTOR);
  if (predictor == nullptr) {
    AERROR << "Nullptr found for obstacle [" << obstacle->id() << "]";
    return;
  }
  predictor->Predict(adc_trajectory_container, obstacle, obstacles_container);
}

void PredictorManager::FilterPredictionTrajectory(
    Obstacle* obstacle, PredictionObstacle* prediction_obstacle) {
  for (const auto& trajectory :
       obstacle->latest_feature().predicted_trajectory()) {
    auto* filter_trajectory = prediction_obstacle->add_trajectory();
    if (trajectory.has_probability()) {
      filter_trajectory->set_probability(trajectory.probability());
    }
    for (int i = 0; i < trajectory.trajectory_point_size(); ++i) {
      const auto& point = trajectory.trajectory_point(i);
      if (i == 0 || i == trajectory.trajectory_point_size() - 1 ||
          point.v() < FLAGS_double_precision) {
        filter_trajectory->add_trajectory_point()->CopyFrom(point);
        continue;
      }
      const auto& prev_point = trajectory.trajectory_point(i - 1);
      const auto& next_point = trajectory.trajectory_point(i + 1);

      double filter_x =
          (prev_point.path_point().x() + next_point.path_point().x()) / 2;
      double filter_y =
          (prev_point.path_point().y() + next_point.path_point().y()) / 2;
      double dy = next_point.path_point().y() - prev_point.path_point().y();
      double dx = next_point.path_point().x() - prev_point.path_point().x();
      double filter_heading =
          common::math::atan2(static_cast<float>(dy), static_cast<float>(dx));

      auto* filter_point = filter_trajectory->add_trajectory_point();
      filter_point->mutable_path_point()->set_x(filter_x);
      filter_point->mutable_path_point()->set_y(filter_y);
      filter_point->mutable_path_point()->set_theta(filter_heading);
      filter_point->set_v(point.v());
      filter_point->set_a(point.a());
      filter_point->set_relative_time(point.relative_time());

      if (point.path_point().has_lane_id()) {
        filter_point->mutable_path_point()->set_lane_id(
            point.path_point().lane_id());
      }
      if (point.path_point().has_s()) {
        filter_point->mutable_path_point()->set_s(point.path_point().s());
      }
    }
  }
}

void PredictorManager::RecordDebugData(  // NOLINT
    Obstacle* obstacle, PredictionObstacle* prediction_obstacle,
    PredictionObstacles* prediction_obstacles) {
  if (obstacle == nullptr || obstacle->mutable_latest_feature() == nullptr ||
      prediction_obstacle == nullptr) {
    return;
  }

  // record history info
  for (int i = 0; i < obstacle->history_size(); ++i) {
    if (i >= FLAGS_prediction_debug_history_frame_num) {
      break;
    }
    const auto& feat = obstacle->feature(obstacle->history_size() - 1 - i);
    auto* output = prediction_obstacle->add_feature();
    output->mutable_position()->set_x(feat.position().x());
    output->mutable_position()->set_y(feat.position().y());
    output->mutable_velocity()->set_x(feat.velocity().x());
    output->mutable_velocity()->set_y(feat.velocity().y());
    output->mutable_acceleration()->set_x(feat.acceleration().x());
    output->mutable_acceleration()->set_y(feat.acceleration().y());

    if (feat.has_raw_position() && feat.has_raw_velocity()) {
      output->mutable_raw_position()->set_x(feat.raw_position().x());
      output->mutable_raw_position()->set_y(feat.raw_position().y());
      output->mutable_raw_velocity()->set_x(feat.raw_velocity().x());
      output->mutable_raw_velocity()->set_y(feat.raw_velocity().y());
    }
    if (feat.has_t_acceleration()) {
      output->mutable_t_acceleration()->set_x(feat.t_acceleration().x());
      output->mutable_t_acceleration()->set_y(feat.t_acceleration().y());
    }
    if (feat.has_position_flu() && feat.has_velocity_flu()) {
      output->mutable_position_flu()->set_x(feat.position_flu().x());
      output->mutable_position_flu()->set_y(feat.position_flu().y());
      output->mutable_velocity_flu()->set_x(feat.velocity_flu().x());
      output->mutable_velocity_flu()->set_y(feat.velocity_flu().y());
    }

    output->set_velocity_heading(feat.velocity_heading());
    output->set_theta(feat.theta());
    output->set_speed(feat.speed());
    output->set_acc(feat.acc());
    output->set_length(feat.length());
    output->set_width(feat.width());
    output->set_timestamp(feat.timestamp());
  }

  // record current_lane_feature
  if (!prediction_obstacle->feature().empty() &&
      obstacle->latest_feature().has_lane()) {
    auto* out_feat = prediction_obstacle->mutable_feature(0);
    const auto& feat = obstacle->latest_feature();
    for (const auto& lane : feat.lane().current_lane_feature()) {
      out_feat->mutable_lane()->add_current_lane_feature()->CopyFrom(lane);
    }
    for (const auto& lane : feat.lane().nearby_lane_feature()) {
      out_feat->mutable_lane()->add_nearby_lane_feature()->CopyFrom(lane);
    }
    // record lane_sequence debug data

    if (obstacle->latest_feature().lane().has_lane_graph()) {
      const auto& lane_graph = obstacle->latest_feature().lane().lane_graph();

      for (const auto& lane_sequence : lane_graph.lane_sequence()) {
        auto* out_lane_sequence =
            out_feat->mutable_lane()->mutable_lane_graph()->add_lane_sequence();
        out_lane_sequence->set_lane_sequence_id(
            lane_sequence.lane_sequence_id());

        out_lane_sequence->set_lane_l(lane_sequence.lane_l());

        for (const auto& seg : lane_sequence.lane_segment()) {
          auto* out_lane_segment = out_lane_sequence->add_lane_segment();
          out_lane_segment->set_lane_id(seg.lane_id());
        }

        if (lane_sequence.lane_segment_size() > 0 &&
            lane_sequence.lane_segment(0).lane_point_size() > 0) {
          auto* out_lane_point =
              out_lane_sequence->mutable_lane_segment(0)->add_lane_point();
          out_lane_point->set_width(
              lane_sequence.lane_segment(0).lane_point(0).width());
          out_lane_point->mutable_position()->set_x(
              lane_sequence.lane_segment(0).lane_point(0).position().x());
          out_lane_point->mutable_position()->set_y(
              lane_sequence.lane_segment(0).lane_point(0).position().y());
          out_lane_point->mutable_position()->set_z(
              lane_sequence.lane_segment(0).lane_point(0).position().z());
          out_lane_point->set_heading(
              lane_sequence.lane_segment(0).lane_point(0).heading());
        }

        out_lane_sequence->set_probability(lane_sequence.probability());
        for (const auto& near_by_obstacle : lane_sequence.nearby_obstacle()) {
          out_lane_sequence->add_nearby_obstacle()->CopyFrom(near_by_obstacle);
        }
        out_lane_sequence->mutable_evidence_info()->CopyFrom(
            lane_sequence.evidence_info());
        out_lane_sequence->mutable_dbn_probs()->CopyFrom(
            lane_sequence.dbn_probs());
      }
    }
  }

  // record current_lane_feature
  if (!prediction_obstacle->feature().empty() &&
      obstacle->latest_feature().has_junction_feature()) {
    prediction_obstacle->mutable_feature(0)
        ->mutable_junction_feature()
        ->CopyFrom(obstacle->latest_feature().junction_feature());
  }

  if (obstacle->latest_feature().goals_2d_size() > 0) {
    for (const auto& goal_2d : obstacle->latest_feature().goals_2d()) {
      prediction_obstacles->add_goals_2d()->CopyFrom(goal_2d);
    }
  }
}

}  // namespace prediction
}  // namespace TL
