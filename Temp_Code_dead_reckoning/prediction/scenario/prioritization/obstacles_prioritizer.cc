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

#include "planning/prediction/scenario/prioritization/obstacles_prioritizer.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <memory>
#include <queue>
#include <set>
#include <unordered_map>
#include <utility>

#include "common/file/log.h"
#include "common/math/box2d.h"
#include "planning/prediction/common/prediction_gflags.h"
#include "planning/prediction/common/prediction_map.h"
#include "planning/prediction/common/prediction_util.h"
#include "planning/prediction/container/adc_trajectory/adc_trajectory_container.h"
#include "planning/prediction/container/obstacles/obstacle_clusters.h"
#include "planning/prediction/container/pose/pose_container.h"
// #include "planning/prediction/container/storytelling/storytelling_container.h"
#include "planning/prediction/scenario/perception_filter/perception_filter.h"
#include "proto/prediction/feature.pb.h"

namespace TL {
namespace prediction {

using TL::common::Point3D;
using TL::common::adapter::AdapterConfig;
using TL::common::math::Box2d;
using TL::common::math::Vec2d;
using TL::hdmap::LaneInfo;
using TL::hdmap::OverlapInfo;
using TL::perception::PerceptionObstacle;
using ConstLaneInfoPtr = std::shared_ptr<const LaneInfo>;

namespace {

bool IsLaneSequenceInReferenceLine(
    const LaneSequence& lane_sequence,
    const ADCTrajectoryContainer* ego_trajectory_container) {
  return std::any_of(
      lane_sequence.lane_segment().begin(), lane_sequence.lane_segment().end(),
      [&](const auto& lane_segment) {
        const auto& lane_id = lane_segment.lane_id();
        return ego_trajectory_container->IsLaneIdInTargetReferenceLine(lane_id);
      });
}

int NearestFrontObstacleIdOnLaneSequence(const LaneSequence& lane_sequence) {
  int nearest_front_obstacle_id = std::numeric_limits<int>::min();
  double smallest_relative_s = std::numeric_limits<double>::max();
  for (const auto& nearby_obs : lane_sequence.nearby_obstacle()) {
    if (nearby_obs.s() < 0.0 ||
        nearby_obs.s() > FLAGS_caution_search_distance_ahead) {
      continue;
    }
    if (nearby_obs.s() < smallest_relative_s) {
      smallest_relative_s = nearby_obs.s();
      nearest_front_obstacle_id = nearby_obs.id();
    }
  }
  return nearest_front_obstacle_id;
}

int NearestBackwardObstacleIdOnLaneSequence(const LaneSequence& lane_sequence) {
  int nearest_backward_obstacle_id = std::numeric_limits<int>::min();
  double smallest_relative_s = std::numeric_limits<double>::max();
  for (const auto& nearby_obs : lane_sequence.nearby_obstacle()) {
    if (nearby_obs.s() > 0.0 ||
        nearby_obs.s() < -FLAGS_caution_search_distance_backward) {
      continue;
    }
    if (-nearby_obs.s() < smallest_relative_s) {
      smallest_relative_s = -nearby_obs.s();
      nearest_backward_obstacle_id = nearby_obs.id();
    }
  }
  return nearest_backward_obstacle_id;
}

}  // namespace

static bool IsOutOfRoadEdge(const std::vector<mapping::RoadEdge>& road_edges,
                            double x, double y) {
  double left_boundary_y = std::numeric_limits<double>::infinity();
  double right_boundary_y = -std::numeric_limits<double>::infinity();
  for (const auto& road_edge : road_edges) {
    const auto& lane_param = road_edge.lane_param();
    if (lane_param.cubic_curve_set_size() > 0) {
      const auto& edge = lane_param.cubic_curve_set(0);
      ADEBUG << "cubic_curve_set_size " << lane_param.cubic_curve_set_size()
             << " " << edge.DebugString() << " " << x << " " << y;
      if (std::isgreater(x, edge.start_point_x()) &&
          std::isless(x, edge.end_point_x())) {
        std::array<double, 4> coefs = {edge.c0(), edge.c1(), edge.c2(),
                                       edge.c3()};
        double edge_y = prediction_util::EvaluateCubicPolynomial(coefs, x, 0);
        // 如果edge.start_point_x() > 0, 则取起始点的y值，防止多项式拟合的C0不准
        double nearest_y = edge.c0();
        if (edge.start_point_x() > 0) {
          nearest_y = prediction_util::EvaluateCubicPolynomial(
              coefs, edge.start_point_x(), 0);
        }
        if (nearest_y > 0 && std::isless(edge_y, left_boundary_y)) {
          left_boundary_y = edge_y;
        }
        if (nearest_y < 0 && std::isgreater(edge_y, right_boundary_y)) {
          right_boundary_y = edge_y;
        }
      }
    }
  }
  ADEBUG << "left_boundary_y " << left_boundary_y << " right_boundary_y "
         << right_boundary_y;
  return std::isless(y, right_boundary_y) || std::isgreater(y, left_boundary_y);
}

static bool IsOutOfRoadEdge(const std::vector<mapping::RoadEdge>& road_edges,
                            const Feature& feat) {
  double x = feat.position_flu().x();
  double y = feat.position_flu().y();
  // if center in Road , consider it all in road
  if (!IsOutOfRoadEdge(road_edges, x, y)) {
    return false;
  }

  // if any corner in Road , consider it in road
  common::math::Box2d box({x, y}, feat.theta_flu(), feat.length(),
                          feat.width());
  for (const auto& corner : box.GetAllCorners()) {
    if (!IsOutOfRoadEdge(road_edges, corner.x(), corner.y())) {
      return false;
    }
  }
  return true;
}

ObstaclesPrioritizer::ObstaclesPrioritizer(
    const std::shared_ptr<ContainerManager>& container_manager)
    : container_manager_(container_manager) {}

void ObstaclesPrioritizer::AssignIgnoreLevel(
    const std::vector<mapping::RoadEdge>& road_edges,
    ObstaclesContainer* obstacles_container, const Scenario* current_scenario) {
  if (obstacles_container == nullptr) {
    AERROR << "Obstacles container pointer is a null pointer.";
    return;
  }

  Obstacle* ego_obstacle_ptr =
      obstacles_container->GetObstacle(FLAGS_ego_vehicle_id);
  if (ego_obstacle_ptr == nullptr) {
    AERROR << "Ego obstacle nullptr found";
    return;
  }

  const Feature& ego_feature = ego_obstacle_ptr->latest_feature();
  const Point3D& ego_velocity = ego_feature.velocity();
  double ego_theta = ego_feature.theta();
  double ego_x = ego_feature.position().x();
  double ego_y = ego_feature.position().y();
  ADEBUG << "Get pose (" << ego_x << ", " << ego_y << ", " << ego_theta << ")";

  // Get the lane ids of the ego vehicle
  std::set<std::string> ego_lane_ids;
  if (ego_feature.has_lane() && ego_feature.lane().has_lane_graph()) {
    const auto& graph = ego_feature.lane().lane_graph();
    for (const auto& seq : graph.lane_sequence()) {
      for (const auto& seg : seq.lane_segment()) {
        if (seg.has_lane_id()) {
          ego_lane_ids.emplace(seg.lane_id());
        }
      }
    }
  }

  // Build rectangular scan_area
  Box2d scan_box({ego_x, ego_y}, ego_theta, FLAGS_scan_length,
                 FLAGS_scan_width);

  const auto& obstacle_ids =
      obstacles_container->curr_frame_movable_obstacle_ids();
  for (const int obstacle_id : obstacle_ids) {
    Obstacle* obstacle_ptr = obstacles_container->GetObstacle(obstacle_id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Null obstacle pointer found.";
      continue;
    }
    if (obstacle_ptr->history_size() == 0) {
      AERROR << "Obstacle [" << obstacle_ptr->id() << "] has no feature.";
      continue;
    }
    Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
    double obstacle_x = latest_feature_ptr->position().x();
    double obstacle_y = latest_feature_ptr->position().y();

    Vec2d ego_to_obstacle_vec(obstacle_x - ego_x, obstacle_y - ego_y);
    Vec2d ego_vec(ego_velocity.x(), ego_velocity.y());
    double s = ego_to_obstacle_vec.InnerProd(ego_vec);
    Vec2d obs_vec(latest_feature_ptr->velocity().x(),
                  latest_feature_ptr->velocity().y());
    bool ignore_obs = true;
    // Ignore the oncoming traffic behind the ego car
    if (obs_vec.InnerProd(ego_vec) > -FLAGS_double_precision - 1.0 ||
        s > -FLAGS_double_precision - 1.0) {
      double pedestrian_like_nearby_lane_radius =
          FLAGS_pedestrian_nearby_lane_search_radius;
      bool is_near_lane = PredictionMap::HasNearbyLane(
          obstacle_x, obstacle_y, pedestrian_like_nearby_lane_radius);

      // Decide if we need consider this obstacle
      bool is_in_scan_area = scan_box.IsPointIn({obstacle_x, obstacle_y});
      bool is_on_lane = obstacle_ptr->IsOnLane();

      bool is_pedestrian_like_in_front_near_lanes =
          s > FLAGS_back_dist_ignore_ped &&
          (latest_feature_ptr->type() == PerceptionObstacle::PEDESTRIAN ||
           latest_feature_ptr->type() == PerceptionObstacle::BICYCLE ||
           latest_feature_ptr->type() == PerceptionObstacle::UNKNOWN ||
           latest_feature_ptr->type() == PerceptionObstacle::UNKNOWN_MOVABLE) &&
          is_near_lane;
      bool is_near_junction = obstacle_ptr->IsNearJunction();
      bool is_near_ego_laneseq = false;
      if (latest_feature_ptr->has_lane()) {
        if (!latest_feature_ptr->lane().current_lane_feature().empty()) {
          for (const auto& lane_feature :
               latest_feature_ptr->lane().current_lane_feature()) {
            if (ego_lane_ids.find(lane_feature.lane_id()) !=
                ego_lane_ids.end()) {
              is_near_ego_laneseq = true;
              ADEBUG << "obstacle [" << obstacle_id
                     << "] is_near_ego_laneseq , lane_id :"
                     << lane_feature.lane_id();
              break;
            }
          }
        }
        if (!is_near_ego_laneseq &&
            !latest_feature_ptr->lane().nearby_lane_feature().empty()) {
          for (const auto& lane_feature :
               latest_feature_ptr->lane().nearby_lane_feature()) {
            if (ego_lane_ids.find(lane_feature.lane_id()) !=
                ego_lane_ids.end()) {
              is_near_ego_laneseq = true;
              ADEBUG << "obstacle [" << obstacle_id
                     << "] is_near_ego_laneseq , lane_id :"
                     << lane_feature.lane_id();
              break;
            }
          }
        }
      }

      ignore_obs =
          !(is_in_scan_area || is_on_lane || is_near_junction ||
            is_pedestrian_like_in_front_near_lanes || is_near_ego_laneseq);

      // ignore by perception filter
      double occluded_ignore_distance =
          20 + 2 * std::hypot(ego_velocity.x(), ego_velocity.y());
      if (!ignore_obs &&
          PerceptionFilter::Filter(obstacle_ptr, current_scenario,
                                   occluded_ignore_distance)) {
        ignore_obs = true;
      }
      // ignore by road edge
      if (!ignore_obs && latest_feature_ptr->has_position_flu() &&
          IsOutOfRoadEdge(road_edges, *latest_feature_ptr)) {
        ignore_obs = true;
      }
    }

    if (ignore_obs) {
      latest_feature_ptr->mutable_priority()->set_priority(
          ObstaclePriority::IGNORE);
    } else {
      latest_feature_ptr->mutable_priority()->set_priority(
          ObstaclePriority::NORMAL);
    }
  }

  obstacles_container->SetConsideredObstacleIds();
}

void ObstaclesPrioritizer::AssignCautionLevel(
    ObstaclesContainer* obstacles_container) {
  if (obstacles_container == nullptr) {
    AERROR << "Obstacles container pointer is a null pointer.";
    return;
  }
  Obstacle* ego_vehicle =
      obstacles_container->GetObstacle(FLAGS_ego_vehicle_id);
  if (ego_vehicle == nullptr) {
    AERROR << "Ego vehicle not found";
    return;
  }
  if (ego_vehicle->history_size() == 0) {
    AERROR << "Ego vehicle has no history";
    return;
  }
  // auto storytelling_container =
  //     container_manager_->GetContainer<StoryTellingContainer>(
  //         AdapterConfig::STORYTELLING);
  // if (storytelling_container != nullptr &&
  //     storytelling_container->ADCDistanceToJunction() <
  //         FLAGS_junction_distance_threshold) {
  //   AssignCautionLevelInJunction(*ego_vehicle, obstacles_container,
  //                                storytelling_container->ADCJunctionId());
  // }
  AssignCautionLevelCruiseKeepLane(*ego_vehicle, obstacles_container);
  AssignCautionLevelCruiseChangeLane(*ego_vehicle, obstacles_container);
  AssignCautionLevelByEgoReferenceLine(*ego_vehicle, obstacles_container);
  AssignCautionLevelPedestrianByEgoReferenceLine(*ego_vehicle,
                                                 obstacles_container);
  if (FLAGS_enable_all_pedestrian_caution_in_front) {
    AssignCautionLevelPedestrianInFront(*ego_vehicle, obstacles_container);
  }
  if (FLAGS_enable_rank_caution_obstacles) {
    RankingCautionLevelObstacles(*ego_vehicle, obstacles_container);
  }
}

void ObstaclesPrioritizer::AssignCautionLevelInJunction(
    const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container,
    const std::string& junction_id) {
  // TODO(Hongyi): get current junction_id from Storytelling
  const auto& obstacle_ids =
      obstacles_container->curr_frame_movable_obstacle_ids();
  for (const int obstacle_id : obstacle_ids) {
    Obstacle* obstacle_ptr = obstacles_container->GetObstacle(obstacle_id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Null obstacle pointer found.";
      continue;
    }
    if (obstacle_ptr->IsInJunction(junction_id)) {
      SetCautionIfCloseToEgo(ego_vehicle, FLAGS_caution_distance_threshold,
                             obstacle_ptr);
    }
  }
}

void ObstaclesPrioritizer::AssignCautionLevelCruiseKeepLane(
    const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container) {
  const Feature& ego_latest_feature = ego_vehicle.latest_feature();
  for (const LaneSequence& lane_sequence :
       ego_latest_feature.lane().lane_graph().lane_sequence()) {
    int nearest_front_obstacle_id =
        NearestFrontObstacleIdOnLaneSequence(lane_sequence);
    if (nearest_front_obstacle_id < 0) {
      continue;
    }
    Obstacle* obstacle_ptr =
        obstacles_container->GetObstacle(nearest_front_obstacle_id);
    if (obstacle_ptr == nullptr || obstacle_ptr->ToIgnore()) {
      AERROR << "Obstacle [" << nearest_front_obstacle_id << "] Not found";
      continue;
    }
    SetCautionIfCloseToEgo(ego_vehicle, FLAGS_caution_distance_threshold,
                           obstacle_ptr);
  }
}

void ObstaclesPrioritizer::AssignCautionLevelCruiseChangeLane(
    const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container) {
  auto* ego_trajectory_container =
      container_manager_->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  const Feature& ego_latest_feature = ego_vehicle.latest_feature();
  for (const LaneSequence& lane_sequence :
       ego_latest_feature.lane().lane_graph().lane_sequence()) {
    if (lane_sequence.vehicle_on_lane()) {
      int nearest_front_obstacle_id =
          NearestFrontObstacleIdOnLaneSequence(lane_sequence);
      if (nearest_front_obstacle_id < 0) {
        continue;
      }
      Obstacle* obstacle_ptr =
          obstacles_container->GetObstacle(nearest_front_obstacle_id);
      if (obstacle_ptr == nullptr || obstacle_ptr->ToIgnore()) {
        AERROR << "Obstacle [" << nearest_front_obstacle_id << "] Not found";
        continue;
      }
      SetCautionIfCloseToEgo(ego_vehicle, FLAGS_caution_distance_threshold,
                             obstacle_ptr);
    } else if (IsLaneSequenceInReferenceLine(lane_sequence,
                                             ego_trajectory_container)) {
      int nearest_front_obstacle_id =
          NearestFrontObstacleIdOnLaneSequence(lane_sequence);
      int nearest_backward_obstacle_id =
          NearestBackwardObstacleIdOnLaneSequence(lane_sequence);
      if (nearest_front_obstacle_id >= 0) {
        Obstacle* front_obstacle_ptr =
            obstacles_container->GetObstacle(nearest_front_obstacle_id);
        if (front_obstacle_ptr != nullptr && !front_obstacle_ptr->ToIgnore()) {
          SetCautionIfCloseToEgo(ego_vehicle, FLAGS_caution_distance_threshold,
                                 front_obstacle_ptr);
        }
      }
      if (nearest_backward_obstacle_id >= 0) {
        Obstacle* backward_obstacle_ptr =
            obstacles_container->GetObstacle(nearest_backward_obstacle_id);
        if (backward_obstacle_ptr != nullptr &&
            !backward_obstacle_ptr->ToIgnore()) {
          SetCautionIfCloseToEgo(ego_vehicle, FLAGS_caution_distance_threshold,
                                 backward_obstacle_ptr);
        }
      }
    }
  }
}

void ObstaclesPrioritizer::AssignCautionLevelByEgoReferenceLine(
    const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container) {
  auto* adc_trajectory_container =
      container_manager_->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  if (adc_trajectory_container == nullptr) {
    AERROR << "adc_trajectory_container is nullptr";
    return;
  }
  const std::vector<std::string>& lane_ids =
      adc_trajectory_container->GetADCTargetLaneIDSequence();
  if (lane_ids.empty()) {
    return;
  }

  const Feature& ego_feature = ego_vehicle.latest_feature();
  double ego_x = ego_feature.position().x();
  double ego_y = ego_feature.position().y();
  double ego_vehicle_s = std::numeric_limits<double>::max();
  double ego_vehicle_l = std::numeric_limits<double>::max();
  double accumulated_s = 0.0;
  // first loop for lane_ids to findout ego_vehicle_s
  for (const std::string& lane_id : lane_ids) {
    std::shared_ptr<const LaneInfo> lane_info_ptr =
        PredictionMap::LaneById(lane_id);
    if (lane_info_ptr == nullptr) {
      AERROR << "Null lane info pointer found.";
      continue;
    }
    double s = 0.0;
    double l = 0.0;
    if (PredictionMap::GetProjection({ego_x, ego_y}, lane_info_ptr, &s, &l)) {
      if (std::fabs(l) < std::fabs(ego_vehicle_l)) {
        ego_vehicle_s = accumulated_s + s;
        ego_vehicle_l = l;
        ego_lane_id_ = lane_id;
        ego_lane_s_ = s;
      }
    }
    accumulated_s += lane_info_ptr->total_length();
  }

  // insert ego_back_lane_id
  accumulated_s = 0.0;
  for (const std::string& lane_id : lane_ids) {
    if (lane_id == ego_lane_id_) {
      break;
    }
    ego_back_lane_id_set_.insert(lane_id);
  }

  std::unordered_set<std::string> visited_lanes(lane_ids.begin(),
                                                lane_ids.end());

  // then loop through lane_ids to AssignCaution for obstacle vehicles
  for (const std::string& lane_id : lane_ids) {
    if (ego_back_lane_id_set_.find(lane_id) != ego_back_lane_id_set_.end()) {
      continue;
    }
    std::shared_ptr<const LaneInfo> lane_info_ptr =
        PredictionMap::LaneById(lane_id);
    if (lane_info_ptr == nullptr) {
      AERROR << "Null lane info pointer found.";
      continue;
    }
    accumulated_s += lane_info_ptr->total_length();
    if (lane_id != ego_lane_id_) {
      AssignCautionByMerge(ego_vehicle, lane_info_ptr, &visited_lanes,
                           obstacles_container);
    }
    AssignCautionByOverlap(ego_vehicle, lane_info_ptr, &visited_lanes,
                           obstacles_container);
    if (accumulated_s > FLAGS_caution_search_distance_ahead + ego_vehicle_s) {
      break;
    }
  }
}

void ObstaclesPrioritizer::AssignCautionLevelPedestrianByEgoReferenceLine(
    const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container) {
  auto* adc_trajectory_container =
      container_manager_->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  if (adc_trajectory_container == nullptr) {
    AERROR << "adc_trajectory_container is nullptr";
    return;
  }
  for (const int obstacle_id :
       obstacles_container->curr_frame_movable_obstacle_ids()) {
    Obstacle* obstacle_ptr = obstacles_container->GetObstacle(obstacle_id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Null obstacle pointer found.";
      continue;
    }
    if (obstacle_ptr->ToIgnore()) {
      continue;
    }
    Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
    if (latest_feature_ptr->type() != PerceptionObstacle::PEDESTRIAN) {
      continue;
    }
    double start_x = latest_feature_ptr->position().x();
    double start_y = latest_feature_ptr->position().y();
    double end_x = start_x + FLAGS_caution_pedestrian_approach_time *
                                 latest_feature_ptr->raw_velocity().x();
    double end_y = start_y + FLAGS_caution_pedestrian_approach_time *
                                 latest_feature_ptr->raw_velocity().y();
    std::vector<std::string> nearby_lane_ids = PredictionMap::NearbyLaneIds(
        {start_x, start_y}, FLAGS_pedestrian_nearby_lane_search_radius);
    if (nearby_lane_ids.empty()) {
      continue;
    }
    for (const std::string& lane_id : nearby_lane_ids) {
      if (!adc_trajectory_container->IsLaneIdInTargetReferenceLine(lane_id)) {
        continue;
      }
      std::shared_ptr<const LaneInfo> lane_info_ptr =
          PredictionMap::LaneById(lane_id);
      if (lane_info_ptr == nullptr) {
        AERROR << "Null lane info pointer found.";
        continue;
      }
      double start_s = 0.0;
      double start_l = 0.0;
      double end_s = 0.0;
      double end_l = 0.0;
      if (PredictionMap::GetProjection({start_x, start_y}, lane_info_ptr,
                                       &start_s, &start_l) &&
          PredictionMap::GetProjection({end_x, end_y}, lane_info_ptr, &end_s,
                                       &end_l)) {
        if (std::fabs(start_l) < FLAGS_pedestrian_nearby_lane_search_radius ||
            std::fabs(end_l) < FLAGS_pedestrian_nearby_lane_search_radius ||
            start_l * end_l < 0.0) {
          SetCautionIfCloseToEgo(ego_vehicle, FLAGS_caution_distance_threshold,
                                 obstacle_ptr);
        }
      }
    }
  }
}

void ObstaclesPrioritizer::AssignCautionLevelPedestrianInFront(
    const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container) {
  const Point3D& ego_position = ego_vehicle.latest_feature().position();
  const Point3D& ego_velocity = ego_vehicle.latest_feature().velocity();
  const auto& obstacle_ids =
      obstacles_container->curr_frame_movable_obstacle_ids();
  for (const int obstacle_id : obstacle_ids) {
    Obstacle* obstacle_ptr = obstacles_container->GetObstacle(obstacle_id);
    if (obstacle_ptr == nullptr || !obstacle_ptr->IsPedestrian() ||
        obstacle_ptr->history_size() == 0) {
      continue;
    }
    const Feature& obs_feature = obstacle_ptr->latest_feature();
    double obs_x = obs_feature.position().x();
    double obs_y = obs_feature.position().y();
    double diff_x = obs_x - ego_position.x();
    double diff_y = obs_y - ego_position.y();
    double inner_prod = ego_velocity.x() * diff_x + ego_velocity.y() * diff_y;
    if (inner_prod < 0.0) {
      continue;
    }
    SetCautionIfCloseToEgo(ego_vehicle, FLAGS_caution_distance_threshold,
                           obstacle_ptr);
  }
}

void ObstaclesPrioritizer::RankingCautionLevelObstacles(
    const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container) {
  const Point3D& ego_position = ego_vehicle.latest_feature().position();
  const auto& obstacle_ids =
      obstacles_container->curr_frame_movable_obstacle_ids();
  std::priority_queue<std::pair<double, Obstacle*>> caution_obstacle_queue;
  for (const int obstacle_id : obstacle_ids) {
    Obstacle* obstacle_ptr = obstacles_container->GetObstacle(obstacle_id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Obstacle [" << obstacle_id << "] Not found";
      continue;
    }
    if (!obstacle_ptr->IsCaution()) {
      continue;
    }
    const Point3D& obstacle_position =
        obstacle_ptr->latest_feature().position();
    double distance = std::hypot(obstacle_position.x() - ego_position.x(),
                                 obstacle_position.y() - ego_position.y());
    caution_obstacle_queue.emplace(distance, obstacle_ptr);
  }
  while (static_cast<int>(caution_obstacle_queue.size()) >
         FLAGS_caution_obs_max_nums) {
    Obstacle* obstacle_ptr = caution_obstacle_queue.top().second;
    obstacle_ptr->mutable_latest_feature()->mutable_priority()->set_priority(
        ObstaclePriority::NORMAL);
    caution_obstacle_queue.pop();
  }
}

void ObstaclesPrioritizer::AssignCautionByMerge(
    const Obstacle& ego_vehicle,
    const std::shared_ptr<const LaneInfo>& lane_info_ptr,
    std::unordered_set<std::string>* const visited_lanes,
    ObstaclesContainer* obstacles_container) {
  SetCautionBackward(FLAGS_caution_search_distance_backward_for_merge,
                     ego_vehicle, lane_info_ptr, visited_lanes,
                     obstacles_container);
}

void ObstaclesPrioritizer::AssignCautionByOverlap(
    const Obstacle& ego_vehicle,
    const std::shared_ptr<const LaneInfo>& lane_info_ptr,
    std::unordered_set<std::string>* const visited_lanes,
    ObstaclesContainer* obstacles_container) {
  // const auto& lane_id = lane_info_ptr->id().id();
  const std::vector<std::shared_ptr<const OverlapInfo>> cross_lanes =
      lane_info_ptr->cross_lanes();
  for (const auto& overlap_ptr : cross_lanes) {
    bool consider_overlap = true;
    for (const auto& object : overlap_ptr->overlap().object()) {
      if (object.id().id() == lane_info_ptr->id().id() &&
          object.lane_overlap_info().end_s() < ego_lane_s_) {
        consider_overlap = false;
      }
    }

    if (!consider_overlap) {
      continue;
    }

    for (const auto& object : overlap_ptr->overlap().object()) {
      const auto& object_id = object.id().id();
      if (object_id == lane_info_ptr->id().id()) {
        continue;
      }
      std::shared_ptr<const LaneInfo> overlap_lane_ptr =
          PredictionMap::LaneById(object_id);
      if (overlap_lane_ptr == nullptr) {
        AERROR << "Null lane info pointer found.";
        continue;
      }
      // ahead_s is the length in front of the overlap
      double ahead_s = overlap_lane_ptr->total_length() -
                       object.lane_overlap_info().start_s();
      SetCautionBackward(
          ahead_s + FLAGS_caution_search_distance_backward_for_overlap,
          ego_vehicle, overlap_lane_ptr, visited_lanes, obstacles_container);
    }
  }
}

void ObstaclesPrioritizer::SetCautionBackward(
    const double max_distance, const Obstacle& ego_vehicle,
    const std::shared_ptr<const LaneInfo>& start_lane_info_ptr,
    std::unordered_set<std::string>* const visited_lanes,
    ObstaclesContainer* obstacles_container) {
  std::string start_lane_id = start_lane_info_ptr->id().id();
  if (ego_back_lane_id_set_.find(start_lane_id) !=
      ego_back_lane_id_set_.end()) {
    return;
  }
  std::unordered_map<std::string, std::vector<LaneObstacle>> lane_obstacles =
      obstacles_container->GetClustersPtr()->GetLaneObstacles();
  std::queue<std::pair<ConstLaneInfoPtr, double>> lane_info_queue;
  lane_info_queue.emplace(start_lane_info_ptr,
                          start_lane_info_ptr->total_length());
  while (!lane_info_queue.empty()) {
    ConstLaneInfoPtr curr_lane = lane_info_queue.front().first;
    double cumu_distance = lane_info_queue.front().second;
    lane_info_queue.pop();
    const std::string& lane_id = curr_lane->id().id();
    if (visited_lanes->find(lane_id) == visited_lanes->end() &&
        lane_obstacles.find(lane_id) != lane_obstacles.end() &&
        !lane_obstacles[lane_id].empty()) {
      visited_lanes->insert(lane_id);
      // find the obstacle with largest lane_s on the lane
      int obstacle_id = lane_obstacles[lane_id].front().obstacle_id();
      double obstacle_s = lane_obstacles[lane_id].front().lane_s();
      for (const LaneObstacle& lane_obstacle : lane_obstacles[lane_id]) {
        if (lane_obstacle.lane_s() > obstacle_s) {
          obstacle_id = lane_obstacle.obstacle_id();
          obstacle_s = lane_obstacle.lane_s();
        }
      }
      Obstacle* obstacle_ptr = obstacles_container->GetObstacle(obstacle_id);
      if (obstacle_ptr == nullptr || obstacle_ptr->ToIgnore()) {
        AERROR << "Obstacle [" << obstacle_id << "] Not found";
        continue;
      }
      SetCautionIfCloseToEgo(ego_vehicle, FLAGS_caution_distance_threshold,
                             obstacle_ptr);
      continue;
    }
    if (cumu_distance > max_distance) {
      continue;
    }
    for (const auto& pre_lane_id : curr_lane->lane().predecessor_id()) {
      if (ego_back_lane_id_set_.find(pre_lane_id.id()) !=
          ego_back_lane_id_set_.end()) {
        continue;
      }
      ConstLaneInfoPtr pre_lane_ptr = PredictionMap::LaneById(pre_lane_id.id());
      if (pre_lane_ptr == nullptr) {
        AERROR << "Null lane info pointer found.";
        continue;
      }
      lane_info_queue.emplace(pre_lane_ptr,
                              cumu_distance + pre_lane_ptr->total_length());
    }
  }
}

void ObstaclesPrioritizer::SetCautionIfCloseToEgo(
    const Obstacle& ego_vehicle, const double distance_threshold,
    Obstacle* obstacle_ptr) {
  const Point3D& obstacle_position = obstacle_ptr->latest_feature().position();
  const Point3D& ego_position = ego_vehicle.latest_feature().position();
  double diff_x = obstacle_position.x() - ego_position.x();
  double diff_y = obstacle_position.y() - ego_position.y();
  double distance = std::hypot(diff_x, diff_y);
  if (distance < distance_threshold) {
    obstacle_ptr->SetCaution();
  }
}

}  // namespace prediction
}  // namespace TL
