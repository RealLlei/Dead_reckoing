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

#include "planning/prediction/container/obstacles/obstacles_container.h"

#include <cstddef>
#include <iomanip>
#include <memory>
#include <utility>

#include <set>
#include "common/time/clock.h"
#include "common/util/perf_util.h"
#include "planning/prediction/common/feature_output.h"
#include "planning/prediction/common/junction_analyzer.h"
#include "planning/prediction/common/prediction_constants.h"
#include "planning/prediction/common/prediction_gflags.h"
#include "planning/prediction/common/prediction_map.h"
#include "planning/prediction/common/prediction_system_gflags.h"
#include "planning/prediction/container/obstacles/obstacle_clusters.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace prediction {

using TL::perception::PerceptionObstacle;
using TL::perception::PerceptionObstacles;

ObstaclesContainer::ObstaclesContainer()
    : ptr_obstacles_(FLAGS_max_num_obstacles),
      clusters_(new ObstacleClusters()) {}

void ObstaclesContainer::CleanUp() {
  // Clean up the history and get the PerceptionObstacles
  curr_frame_movable_obstacle_ids_.clear();
  curr_frame_unmovable_obstacle_ids_.clear();
  curr_frame_considered_obstacle_ids_.clear();
}

void ObstaclesContainer::ClearCluster() {
  clusters_->GetLaneObstacles().clear();
}

// This is called by Perception module at every frame to insert all
// detected obstacles.
void ObstaclesContainer::Insert(const ::google::protobuf::Message& message) {
  PerceptionObstacles perception_obstacles;
  perception_obstacles.CopyFrom(
      dynamic_cast<const PerceptionObstacles&>(message));

  // Get the new timestamp and update it in the class
  // - If it's more than 10sec later than the most recent one, clear the
  //   obstacle history.
  // - If it's not a valid time (earlier than history), continue.
  // - Also consider the offline_mode case.
  double timestamp = 0.0;
  if (perception_obstacles.has_header() &&
      perception_obstacles.header().has_data_stamp()) {
    timestamp = perception_obstacles.header().data_stamp();
  }
  if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap) {
    ptr_obstacles_.Clear();
    ADEBUG << "Replay mode is enabled.";
  } else if (timestamp <= timestamp_) {
    AERROR << "Invalid timestamp curr [" << timestamp << "] v.s. prev ["
           << timestamp_ << "].";
    return;
  }

  if (FLAGS_enable_online_record4Prediction) {
    if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap ||
        FeatureOutput::Size() > FLAGS_max_num_dump_feature) {
      FeatureOutput::WriteFeatureProto();
      FeatureOutput::WriteDataForLearning();
    }
  }

  switch (FLAGS_prediction_offline_mode) {
    case 1: {
      if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap ||
          FeatureOutput::Size() > FLAGS_max_num_dump_feature) {
        FeatureOutput::WriteFeatureProto();
      }
      break;
    }
    case 2: {
      if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap ||
          FeatureOutput::SizeOfDataForLearning() >
              FLAGS_max_num_dump_dataforlearn) {
        FeatureOutput::WriteDataForLearning();
      }
      break;
    }
    case 3: {
      if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap ||
          FeatureOutput::SizeOfPredictionResult() >
              FLAGS_max_num_dump_feature) {
        FeatureOutput::WritePredictionResult();
      }
      break;
    }
    case 4: {
      if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap ||
          FeatureOutput::SizeOfFrameEnv() > FLAGS_max_num_dump_feature) {
        FeatureOutput::WriteFrameEnv();
      }
      break;
    }
    case 5: {
      if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap ||
          FeatureOutput::SizeOfFrameEnv() > FLAGS_max_num_dump_feature) {
        FeatureOutput::WriteDataForTuning();
      }
      break;
    }
    default: {
      // No data dump
      if (!FLAGS_enable_online_record4Prediction) {
        FeatureOutput::Clear();
      }
      break;
    }
  }

  timestamp_ = timestamp;
  ADEBUG << "Current timestamp is [" << FIXED << SETPRECISION(3) << timestamp_
         << "]";

  clusters_->GetLaneObstacles().clear();

  // Set up the ObstacleClusters:
  // Insert the Obstacles one by one
  for (const PerceptionObstacle& perception_obstacle :
       perception_obstacles.perception_obstacle()) {
    ADEBUG << "Perception obstacle [" << perception_obstacle.id() << "] "
           << "was detected";
    InsertPerceptionObstacle(perception_obstacle, timestamp_, nullptr);
    ADEBUG << "Perception obstacle [" << perception_obstacle.id() << "] "
           << "was inserted";
  }

  SetConsideredObstacleIds();
  clusters_->SortObstacles();
}

Obstacle* ObstaclesContainer::GetObstacle(const int id) {
  auto* ptr_obstacle = ptr_obstacles_.GetSilently(id);
  if (ptr_obstacle != nullptr) {
    return ptr_obstacle->get();
  }
  return nullptr;
}

Obstacle* ObstaclesContainer::GetObstacleWithLRUUpdate(const int obstacle_id) {
  auto* ptr_obstacle = ptr_obstacles_.Get(obstacle_id);
  if (ptr_obstacle != nullptr) {
    return ptr_obstacle->get();
  }
  return nullptr;
}

void ObstaclesContainer::Clear() {
  ptr_obstacles_.Clear();
  timestamp_ = -1.0;
}

const std::vector<int>& ObstaclesContainer::curr_frame_movable_obstacle_ids() {
  return curr_frame_movable_obstacle_ids_;
}

const std::vector<int>&
ObstaclesContainer::curr_frame_unmovable_obstacle_ids() {
  return curr_frame_unmovable_obstacle_ids_;
}

const std::vector<int>&
ObstaclesContainer::curr_frame_considered_obstacle_ids() {
  return curr_frame_considered_obstacle_ids_;
}

void ObstaclesContainer::SetConsideredObstacleIds() {
  curr_frame_considered_obstacle_ids_.clear();
  for (const int id : curr_frame_movable_obstacle_ids_) {
    Obstacle* obstacle_ptr = GetObstacle(id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Null obstacle found : " << id;
      continue;
    }
    if (obstacle_ptr->ToIgnore()) {
      ADEBUG << "Ignore obstacle [" << obstacle_ptr->id() << "]";
      continue;
    }
    curr_frame_considered_obstacle_ids_.push_back(id);
  }
}

std::vector<int> ObstaclesContainer::curr_frame_obstacle_ids() {
  std::vector<int> curr_frame_obs_ids = curr_frame_movable_obstacle_ids_;
  curr_frame_obs_ids.insert(curr_frame_obs_ids.end(),
                            curr_frame_unmovable_obstacle_ids_.begin(),
                            curr_frame_unmovable_obstacle_ids_.end());
  return curr_frame_obs_ids;
}

void ObstaclesContainer::Insert(
    const perception::PerceptionObstacles& perception_obstacles,
    ScenarioManager* scenario_manager) {
  if (scenario_manager == nullptr) {
    AERROR << "scenario_manager is nullptr";
    return;
  }
  // Get the new timestamp and update it in the class

  // - If it's more than 10sec later than the most recent one, clear the
  //   obstacle history.
  // - If it's not a valid time (earlier than history), continue.
  // - Also consider the offline_mode case.
  double timestamp = 0.0;
  if (perception_obstacles.has_header() &&
      perception_obstacles.header().has_data_stamp()) {
    timestamp = perception_obstacles.header().data_stamp();
  }
  if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap) {
    ptr_obstacles_.Clear();
    ADEBUG << "Replay mode is enabled.";
  } else if (timestamp <= timestamp_) {
    AERROR << "Invalid timestamp curr [" << timestamp << "] v.s. prev ["
           << timestamp_ << "].";
    // return;
  }

  timestamp_ = timestamp;
  ADEBUG << "Current timestamp is [" << FIXED << SETPRECISION(3) << timestamp_
         << "]";

  // clusters_->GetLaneObstacles().clear();

  // Set up the ObstacleClusters:
  // Insert the Obstacles one by one
  for (const PerceptionObstacle& perception_obstacle :
       perception_obstacles.perception_obstacle()) {
    ADEBUG << "Perception obstacle [" << perception_obstacle.id() << "] "
           << "was detected";
    InsertPerceptionObstacle(perception_obstacle, timestamp_, scenario_manager);
    ADEBUG << "Perception obstacle [" << perception_obstacle.id() << "] "
           << "was inserted";
  }

  SetConsideredObstacleIds();
  clusters_->SortObstacles();
}

void ObstaclesContainer::InsertPerceptionObstacle(
    const PerceptionObstacle& perception_obstacle, const double timestamp,
    ScenarioManager* scenario_manager) {
  // Sanity checks.
  int id = perception_obstacle.id();
  if (id < FLAGS_ego_vehicle_id) {
    AERROR << "Invalid ID [" << id << "]";
    return;
  }
  if (!IsMovable(perception_obstacle)) {
    ADEBUG << "Perception obstacle [" << perception_obstacle.id()
           << "] is unmovable.";
    if (GetObstacle(id) != nullptr) {
      ptr_obstacles_.Remove(id);
    }

    curr_frame_unmovable_obstacle_ids_.push_back(id);
    return;
  }

  // Insert the obstacle and also update the LRUCache.
  auto* obstacle_ptr = GetObstacleWithLRUUpdate(id);
  if (obstacle_ptr != nullptr) {
    if (scenario_manager != nullptr &&
        (scenario_manager->VehicleReferenceFrame() ||
         scenario_manager->PerceptionProviderChanging())) {
      obstacle_ptr->ClearHistory();
    }
    ADEBUG << "Current time = " << FIXED << SETPRECISION(3) << timestamp;
    obstacle_ptr->Insert(perception_obstacle, timestamp, id);
    ADEBUG << "Refresh obstacle [" << id << "]";
  } else {
    auto ptr_obstacle = Obstacle::Create(clusters_.get());
    if (ptr_obstacle == nullptr) {
      AERROR << "Failed to insert obstacle into container";
      return;
    }
    ptr_obstacle->Insert(perception_obstacle, timestamp, id);
    // ptr_obstacle->SetJunctionAnalyzer(&junction_analyzer_);
    ptr_obstacles_.Put(id, std::move(ptr_obstacle));
    ADEBUG << "Insert obstacle [" << id << "]";
  }

  if (FLAGS_prediction_offline_mode ==
          PredictionConstants::kDumpDataForLearning ||
      id != FLAGS_ego_vehicle_id) {
    curr_frame_movable_obstacle_ids_.push_back(id);
  }
}

void ObstaclesContainer::InsertFeatureProto(Feature* feature) {
  if (!feature->has_id()) {
    AERROR << "Invalid feature, no ID found.";
    return;
  }
  int id = feature->id();
  auto* obstacle_ptr = GetObstacleWithLRUUpdate(id);
  if (obstacle_ptr != nullptr) {
    obstacle_ptr->InsertFeature(feature);
  } else {
    auto ptr_obstacle = Obstacle::Create(feature, clusters_.get());
    if (ptr_obstacle == nullptr) {
      AERROR << "Failed to insert obstacle into container";
      return;
    }
    // ptr_obstacle->SetJunctionAnalyzer(&junction_analyzer_);
    ptr_obstacles_.Put(id, std::move(ptr_obstacle));
  }
}

void ObstaclesContainer::BuildEgoLaneGraph() {
  Obstacle* ego_vehicle_ptr = GetObstacle(FLAGS_ego_vehicle_id);
  if (ego_vehicle_ptr == nullptr) {
    AERROR << "Ego vehicle not inserted";
    return;
  }
  ego_vehicle_ptr->BuildLaneGraph();
  ego_vehicle_ptr->SetNearbyObstacles();
  // ego_vehicle_ptr->BuildVectorNetGraph();
  ego_vehicle_ptr->SetMergeInfo();

  if (ego_vehicle_ptr->latest_feature().has_intent() &&
      ObstacleIntent::MERGING ==
          ego_vehicle_ptr->latest_feature().intent().type()) {
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> merge_lane_ptr_list;
    std::vector<std::string> ignore_id_list;
    int num_lane_sequence = ego_vehicle_ptr->latest_feature()
                                .lane()
                                .lane_graph()
                                .lane_sequence_size();
    for (int i = 0; i < num_lane_sequence; ++i) {
      const auto& lane_seq_ptr =
          ego_vehicle_ptr->latest_feature().lane().lane_graph().lane_sequence(
              i);

      if (lane_seq_ptr.has_merge_lane_idx() &&
          lane_seq_ptr.merge_lane_idx() > 0) {
        int merge_idx = lane_seq_ptr.merge_lane_idx();
        auto merge_lane_ptr = PredictionMap::LaneById(
            lane_seq_ptr.lane_segment(merge_idx).lane_id());
        if (nullptr == merge_lane_ptr) {
          continue;
        }
        merge_lane_ptr_list.emplace_back(merge_lane_ptr);
        ignore_id_list.emplace_back(
            lane_seq_ptr.lane_segment(merge_idx - 1).lane_id());
      }
    }

    for (const int id : curr_frame_considered_obstacle_ids_) {
      Obstacle* obstacle_ptr = GetObstacle(id);
      if (obstacle_ptr == nullptr) {
        AERROR << "Null obstacle found.";
        continue;
      }

      for (int i = 0; i < merge_lane_ptr_list.size(); ++i) {
        static constexpr double dist_thresh = 200.0;
        static constexpr int search_level = 20;
        obstacle_ptr->SetObsMergingInfoAccording2Ego(merge_lane_ptr_list.at(i),
                                                     search_level, dist_thresh,
                                                     ignore_id_list.at(i));

        if (obstacle_ptr->latest_feature().intent().type() ==
            ObstacleIntent::MERGING) {
          ADEBUG << "Obstacle [" << id << "] is merging!";
          break;
        }
      }
    }
  }
}

void ObstaclesContainer::BuildLaneGraph() {
  // remove ignore obstacle in clusters_
  for (auto& lane_obstacle : clusters_->GetLaneObstacles()) {
    for (auto iter = lane_obstacle.second.begin();
         iter != lane_obstacle.second.end();) {
      const int id = iter->obstacle_id();
      Obstacle* obstacle_ptr = GetObstacle(id);
      if (obstacle_ptr == nullptr) {
        AERROR << "Null obstacle found.";
        iter++;
        continue;
      }
      if (obstacle_ptr->ToIgnore()) {
        iter = lane_obstacle.second.erase(iter);
      } else {
        iter++;
      }
    }
  }

  // Go through every obstacle in the current frame, after some
  // sanity checks, build lane graph for non-junction cases.BuildLaneGraph
  for (const int id : curr_frame_considered_obstacle_ids_) {
    Obstacle* obstacle_ptr = GetObstacle(id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Null obstacle found.";
      continue;
    }
    obstacle_ptr->BuildLaneGraph();

    // if (FLAGS_prediction_offline_mode !=
    //     PredictionConstants::kDumpDataForLearning) {
    //   ADEBUG << "Building Lane Graph.";
    //   obstacle_ptr->BuildLaneGraph();
    //   // obstacle_ptr->BuildLaneGraphFromLeftToRight();
    // } else {
    //   ADEBUG << "Building ordered Lane Graph.";
    //   ADEBUG << "Building lane graph for id = " << id;
    //   obstacle_ptr->BuildLaneGraphFromLeftToRight();
    // }
    obstacle_ptr->SetNearbyObstacles();
    obstacle_ptr->MakeDecision();
  }
}

void ObstaclesContainer::BuildJunctionFeature() {
  // Go through every obstacle in the current frame, after some
  // sanity checks, build junction features for those that are in junction.
  for (const int id : curr_frame_considered_obstacle_ids_) {
    Obstacle* obstacle_ptr = GetObstacle(id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Null obstacle found.";
      continue;
    }
    // 仅考虑车辆类型（小车，大车）
    if (PerceptionObstacle::VEHICLE != obstacle_ptr->type() &&
        !obstacle_ptr->IsOversizedVehicle()) {
      continue;
    }

    for (const auto& junction_analyzer : junction_analyzer_map_) {
      const auto& junction_id = junction_analyzer.second->GetJunctionId();
      const auto& junction_info_ptr = PredictionMap::JunctionById(junction_id);
      if (nullptr == junction_info_ptr) {
        continue;
      }

      bool near_entry_lane = false;
      for (const auto& lane :
           obstacle_ptr->latest_feature().lane().current_lane_feature()) {
        const auto& lane_id = lane.lane_id();
        if (!junction_analyzer.second->IsEntryLane(lane_id)) {
          continue;
        }
        const auto& lane_info_ptr = PredictionMap::LaneById(lane_id);
        if (nullptr == lane_info_ptr) {
          continue;
        }
        double length = lane_info_ptr->total_length();
        double dist2lane_end = length - lane.lane_s();
        if (dist2lane_end < 10.0) {
          near_entry_lane = true;
          break;
        }
      }

      if (!near_entry_lane && !PredictionMap::IsPointInJunction(
                                  obstacle_ptr->latest_feature().position().x(),
                                  obstacle_ptr->latest_feature().position().y(),
                                  junction_info_ptr)) {
        continue;
      }
      obstacle_ptr->SetJunctionAnalyzer(junction_analyzer.second.get());
      obstacle_ptr->BuildJunctionFeature();
      break;
    }
  }
}

void ObstaclesContainer::BuildJunctionMap() {
  Obstacle* ego_obstacle_ptr = GetObstacle(FLAGS_ego_vehicle_id);
  if (nullptr == ego_obstacle_ptr) {
    AERROR << "Ego obstacle ptr is nullptr, build junction map failed!";
    return;
  }
  Eigen::Vector2d ego_point(ego_obstacle_ptr->latest_feature().position().x(),
                            ego_obstacle_ptr->latest_feature().position().y());

  static constexpr double ego_junction_search_radius = 30.0;
  const auto& junction_ptr_vec =
      PredictionMap::GetJunctions(ego_point, ego_junction_search_radius);
  std::set<std::string> junction_id_set;
  for (const auto& junction_ptr : junction_ptr_vec) {
    if (nullptr == junction_ptr) {
      continue;
    }
    std::string junction_id = junction_ptr->id().id();
    junction_id_set.emplace(junction_id);
    if (junction_analyzer_map_.find(junction_id) !=
        junction_analyzer_map_.end()) {
      ADEBUG << "Junction " << junction_id << " map exists, reuse it!";
      continue;
    }
    std::shared_ptr<JunctionAnalyzer> junction_analyzer_ptr =
        std::make_shared<JunctionAnalyzer>();
    junction_analyzer_ptr->Init(junction_id);

    ADEBUG << "Build junction " << junction_id << " map!";
    junction_analyzer_map_.emplace(junction_id, junction_analyzer_ptr);
  }

  // remove redundant junction analyzer
  for (auto iter = junction_analyzer_map_.begin();
       iter != junction_analyzer_map_.end();) {
    const auto& junction_id = iter->first;
    if (junction_id_set.find(junction_id) == junction_id_set.end()) {
      iter = junction_analyzer_map_.erase(iter);
    } else {
      ++iter;
    }
  }
}

bool ObstaclesContainer::IsMovable(
    const PerceptionObstacle& perception_obstacle) {
  double velocity_value = std::hypot(perception_obstacle.velocity().x(),
                                     perception_obstacle.velocity().y());

  return (velocity_value > FLAGS_still_obstacle_speed_threshold) ||
         (perception_obstacle.has_type() &&
          perception_obstacle.type() != PerceptionObstacle::UNKNOWN_UNMOVABLE &&
          perception_obstacle.type() != PerceptionObstacle::TRANSPORT_ELEMENT &&
          perception_obstacle.type() != PerceptionObstacle::STATIC_OBSTACLE);
}

void ObstaclesContainer::SelectPreProcessMethod(
    const perception::PerceptionObstacle& perception_obstacle,
    ScenarioManager* scenario_manager, Obstacle* obstacle) {
  if (obstacle == nullptr) {
    return;
  }
  double raw_speed = std::hypot(perception_obstacle.velocity().x(),
                                perception_obstacle.velocity().y());
  double speed_threshold = FLAGS_still_obstacle_speed_threshold;
  if (perception_obstacle.type() == PerceptionObstacle::PEDESTRIAN ||
      obstacle->IsBicycle()) {
    speed_threshold = FLAGS_still_pedestrian_speed_threshold;
  } else if (perception_obstacle.type() == PerceptionObstacle::UNKNOWN ||
             perception_obstacle.type() ==
                 PerceptionObstacle::UNKNOWN_MOVABLE) {
    speed_threshold = FLAGS_still_unknown_speed_threshold;
  }

  if (scenario_manager == nullptr) {
    obstacle->SetUseKalmanFilter(false);
    return;
  }
  if (scenario_manager->VehicleReferenceFrame() ||
      !scenario_manager->scenario().has_type() ||
      scenario_manager->scenario().type() == Scenario::CRUISE_URBAN ||
      perception_obstacle.type() == PerceptionObstacle::PEDESTRIAN ||
      raw_speed < speed_threshold) {
    obstacle->SetUseKalmanFilter(false);
  } else {
    obstacle->SetUseKalmanFilter(true);
  }
}

double ObstaclesContainer::timestamp() const {
  return timestamp_;
}

// SubmoduleOutput ObstaclesContainer::GetSubmoduleOutput(
//     const size_t history_size, const double frame_start_time) {
//   SubmoduleOutput container_output;
//   for (int id : curr_frame_considered_obstacle_ids_) {
//     Obstacle* obstacle = GetObstacle(id);
//     if (obstacle == nullptr) {
//       AERROR << "Nullptr found for obstacle [" << id << "]";
//       continue;
//     }
//     obstacle->TrimHistory(history_size);
//     obstacle->ClearOldInformation();
//     container_output.InsertObstacle(std::move(*obstacle));
//   }

//   Obstacle* ego_obstacle = GetObstacle(FLAGS_ego_vehicle_id);
//   if (ego_obstacle != nullptr) {
//     container_output.InsertEgoVehicle(std::move(*ego_obstacle));
//   }

//   container_output.set_curr_frame_movable_obstacle_ids(
//       curr_frame_movable_obstacle_ids_);
//   container_output.set_curr_frame_unmovable_obstacle_ids(
//       curr_frame_unmovable_obstacle_ids_);
//   container_output.set_curr_frame_considered_obstacle_ids(
//       curr_frame_considered_obstacle_ids_);
//   container_output.set_frame_start_time(frame_start_time);

//   return container_output;
// }

const Scenario& ObstaclesContainer::curr_scenario() const {
  return curr_scenario_;
}

ObstacleClusters* ObstaclesContainer::GetClustersPtr() const {
  return clusters_.get();
}

// JunctionAnalyzer* ObstaclesContainer::GetJunctionAnalyzer() {
//   return &junction_analyzer_;
// }

// std::shared_ptr<JunctionAnalyzer> ObstaclesContainer::GetJunctionAnalyzer(
//     const std::string& junction_id) {
//   junction_analyzer_activation_map[junction_id] = true;

//   if (junction_analyzer_map.find(junction_id) != junction_analyzer_map.end()) {
//     return junction_analyzer_map[junction_id];
//   }

//   std::shared_ptr<JunctionAnalyzer> junction_analyzer_ptr =
//       std::make_shared<JunctionAnalyzer>();
//   junction_analyzer_ptr->Init(junction_id);
//   junction_analyzer_map[junction_id] = junction_analyzer_ptr;
//   return junction_analyzer_ptr;
// }
}  // namespace prediction
}  // namespace TL
