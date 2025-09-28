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
 * @brief Obstacle
 */

#pragma once

#include <deque>
#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "common/filters/digital_filter.h"
#include "common/filters/mean_filter.h"
#include "common/math/kalman_filter.h"
#include "google/protobuf/repeated_field.h"
#include "map/hdmap/hdmap_common.h"
#include "planning/prediction/common/junction_analyzer.h"
#include "planning/prediction/common/kalman_motion_fusion.h"
#include "planning/prediction/common/prediction_gflags.h"
#include "planning/prediction/container/obstacles/obstacle_clusters.h"
#include "planning/prediction/pipeline/vector_net.h"
#include "planning/prediction/proto/prediction_conf.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/prediction/feature.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"
#include "proto/prediction/scenario.pb.h"

/**
 * @namespace TL::prediction
 * @brief TL::prediction
 */
namespace TL {
namespace prediction {

using ::google::protobuf::RepeatedPtrField;

/**
 * @class Obstacle
 * @brief Prediction obstacle.
 */
class Obstacle {
 public:
  /**
   * @brief Constructor
   */
  static std::unique_ptr<Obstacle> Create(ObstacleClusters* clusters_ptr);

  static std::unique_ptr<Obstacle> Create(Feature* feature,
                                          ObstacleClusters* clusters_ptr);

  Obstacle() = default;

  /**
   * @brief Destructor
   */
  virtual ~Obstacle() = default;

  void SetJunctionAnalyzer(JunctionAnalyzer* junction_analyzer) {
    junction_analyzer_ = junction_analyzer;
  }

  /**
   * @brief Insert a perception obstacle with its timestamp.
   * @param perception_obstacle The obstacle from perception.
   * @param timestamp The timestamp when the perception obstacle was detected.
   */
  bool Insert(const perception::PerceptionObstacle& perception_obstacle,
              double timestamp, int prediction_obstacle_id);

  /**
   * @brief Insert a feature proto message.
   * @param feature proto message.
   */
  bool InsertFeature(Feature* feature);

  void ClearOldInformation();

  void ClearHistory();

  perception::PerceptionObstacle::Type type() const;
  /**
   * @brief Get the Subtype of perception obstacle's Subtype.
   * @return The Subtype pf perception obstacle.
   */
  perception::PerceptionObstacle::SubType Subtype() const;

  /**
   * @brief 判断是否为大车
   * @param sub_type 
   * @return true 
   * @return false 
   */
  bool IsOversizedVehicle() const {
    return latest_feature().sub_type() ==
               perception::PerceptionObstacle::ST_TRUCK ||
           latest_feature().sub_type() ==
               perception::PerceptionObstacle::ST_BIG_TRUCK ||
           latest_feature().sub_type() ==
               perception::PerceptionObstacle::ST_BUS ||
           latest_feature().sub_type() ==
               perception::PerceptionObstacle::ST_MIDDLE_BUS ||
           latest_feature().sub_type() ==
               perception::PerceptionObstacle::ST_PICKUP;
  }

  bool IsPedestrian() const;

  bool IsBicycle() const;

  /**
   * @brief Get the obstacle's ID.
   * @return The obstacle's ID.
   */
  int id() const;

  /**
   * @brief Get the obstacle's timestamp.
   * @return The obstacle's timestamp.
   */
  double timestamp() const;

  bool ReceivedOlderMessage(double timestamp) const;

  /**
   * @brief Get the ith feature from latest to earliest.
   * @param i The index of the feature.
   * @return The ith feature.
   */
  const Feature& feature(int i) const;

  /**
   * @brief Get a pointer to the ith feature from latest to earliest.
   * @param i The index of the feature.
   * @return A pointer to the ith feature.
   */
  Feature* mutable_feature(int i);

  /**
   * @brief Get the latest feature.
   * @return The latest feature.
   */
  const Feature& latest_feature() const;

  /**
   * @brief Get the earliest feature.
   * @return The earliest feature.
   */
  const Feature& earliest_feature() const;

  /**
   * @brief Get a pointer to the latest feature.
   * @return A pointer to the latest feature.
   */
  Feature* mutable_latest_feature();

  /**
   * @brief Set nearby obstacles.
   */
  void SetNearbyObstacles();

  void SetOccludedProb(double occluded_prob) { occluded_prob_ = occluded_prob; }

  /**
   * @brief Get the number of historical features.
   * @return The number of historical features.
   */
  int history_size() const;
  int tracking_frame() const;
  double OccludedProb() const;

  /**
   * @brief Check if the obstacle is still.
   * @return If the obstacle is still.
   */
  bool IsStill();

  /**
   * @brief Check if the obstacle is slow.
   * @return If the obstacle is slow.
   */
  bool IsSlow() const;

  /**
   * @brief Check if the obstacle is on any lane.
   * @return If the obstacle is on any lane.
   */
  bool IsOnLane() const;

  /**
   * @brief Check if the obstacle can be ignored.
   * @return If the obstacle can be ignored.
   */
  bool ToIgnore();

  /**
   * @brief Check if the obstacle is near a junction.
   * @return If the obstacle is near a junction.
   */
  bool IsNearJunction();

  /**
   * @brief Check if the obstacle is a junction.
   * @param junction ID
   * @return If the obstacle is in a junction.
   */
  bool IsInJunction(const std::string& junction_id) const;

  /**
   * @brief Check if the obstacle is close to a junction exit.
   * @return If the obstacle is closed to a junction exit.
   */
  bool IsCloseToJunctionExit() const;

  /**
   * @brief Check if the obstacle has junction feature.
   * @return If the obstacle has junction feature.
   */
  bool HasJunctionFeatureWithExits() const;

  /**
   * @brief Build junction feature.
   */
  void BuildJunctionFeature();

  /**
   * @brief Build obstacle's lane graph
   */
  void BuildLaneGraph();

  void SetObsMergingInfoAccording2Ego(
      const std::shared_ptr<const hdmap::LaneInfo>& lane_info_ptr, int level,
      double dist, const std::string& ignore_lane_id);

  /**
   * @brief is stop intent
   *
   */
  bool IsStop() const;

  /**
   * @brief make decision for obstacle
   *
   */
  void MakeDecision();

  void MakeHesitantLaneChangeDecision();

  /**
   * @brief Build obstacle's vectornet graph
   */
  void BuildVectorNetGraph();

  /**
   * @brief Set merge info in ego vehicle container
   */
  void SetMergeInfo();

  /**
   * @brief Build obstacle's lane graph with lanes being ordered.
   *        This would be useful for lane-scanning algorithms.
   */
  //   void BuildLaneGraphFromLeftToRight();

  /**
   * @brief Set the obstacle as caution level
   */
  void SetCaution();

  bool IsCaution() const;

  bool IsOppositeHighway() const;

  void AddPredictorType(const ObstacleConf::PredictorType& predictor_type);

  const ObstacleConf& obstacle_conf() { return obstacle_conf_; }

  bool HesitantLaneChangeState() const { return hesitant_lane_change_state_; }

  ObstacleConf::ObstacleStatus GetObstacleStatus();

  void SetUseKalmanFilter(bool use_kalman_filter);

  bool GetCurrentJunctionId(std::string* junction_id);

  bool IgnoreByOccludedProb(double occluded_ignore_distance);

 private:
  void CheckHistory(const perception::PerceptionObstacle& perception_obstacle,
                    double timestamp);
  void FilterJunction(Feature* feature) const;
  static void FilterUTurn(Feature* feature);
  void SetStatus(const perception::PerceptionObstacle& perception_obstacle,
                 double timestamp, Feature* feature);
  void SetStatusWithKalmanFilter(
      const perception::PerceptionObstacle& perception_obstacle,
      double timestamp, Feature* feature);

  bool SetId(const perception::PerceptionObstacle& perception_obstacle,
             Feature* feature, int prediction_obstacle_id = -1);

  void SetType(const perception::PerceptionObstacle& perception_obstacle,
               Feature* feature);

  static void SetIsNearJunction(
      const perception::PerceptionObstacle& perception_obstacle,
      Feature* feature);

  void SetTimestamp(const perception::PerceptionObstacle& perception_obstacle,
                    double timestamp, Feature* feature) const;

  void SetPolygonPoints(
      const perception::PerceptionObstacle& perception_obstacle,
      Feature* feature) const;

  void SetPosition(const perception::PerceptionObstacle& perception_obstacle,
                   Feature* feature) const;

  static void SetSensorType(
      const perception::PerceptionObstacle& perception_obstacle,
      Feature* feature);

  void SetOccluded(const perception::PerceptionObstacle& perception_obstacle,
                   Feature* feature);

  void SetVelocity(const perception::PerceptionObstacle& perception_obstacle,
                   Feature* feature);

  static void AdjustHeadingByLane(Feature* feature);

  static void UpdateVelocity(double theta, double* velocity_x,
                             double* velocity_y, double* velocity_heading,
                             double* speed);

  void SetAcceleration(
      const perception::PerceptionObstacle& perception_obstacle,
      Feature* feature) const;

  void SetTheta(const perception::PerceptionObstacle& perception_obstacle,
                Feature* feature) const;

  void SetLengthWidthHeight(
      const perception::PerceptionObstacle& perception_obstacle,
      Feature* feature) const;

  void UpdateLaneBelief(Feature* feature);

  void SetCurrentLanes(Feature* feature);

  void SetNearbyLanes(Feature* feature);

  static void SetSurroundingLaneIds(Feature* feature, double radius);

  void SetLaneSequenceStopSign(LaneSequence* lane_sequence_ptr);

  /** @brief This functions updates the lane-points into the lane-segments
   *        based on the given lane_point_spacing.
   */
  void SetLanePoints(Feature* feature);
  void SetLanePoints(const Feature* feature, double lane_point_spacing,
                     uint64_t max_num_lane_point, bool is_bidirection,
                     LaneGraph* lane_graph) const;

  void SetMotionStatus();

  void SetMotionStatusBySpeed();

  void InsertFeatureToHistory(Feature* feature);

  void SetJunctionFeatureWithEnterLane(const std::string& enter_lane_id,
                                       Feature* feature_ptr);

  void SetJunctionFeatureWithoutEnterLane(Feature* feature_ptr);

  void DiscardOutdatedHistory();

  //   void GetNeighborLaneSegments(
  //       const std::shared_ptr<const TL::hdmap::LaneInfo>& center_lane_info,
  //       bool is_left, int recursion_depth,
  //       std::list<std::string>* const lane_ids_ordered,
  //       std::unordered_set<std::string>* const existing_lane_ids);

  bool HasJunctionExitLane(
      const LaneSequence& lane_sequence,
      const std::unordered_set<std::string>& exit_lane_id_set) const;

  void SetClusters(ObstacleClusters* clusters_ptr);

  bool UseKalmanFilter() const { return use_kalman_filter_; }

 private:
  int id_ = FLAGS_ego_vehicle_id;

  perception::PerceptionObstacle::SubType sub_type_ =
      perception::PerceptionObstacle::ST_UNKNOWN_UNMOVABLE;
  perception::PerceptionObstacle::Type type_ =
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE;
  RepeatedPtrField<Feature> feature_history_;

  std::vector<std::string> current_lanes_id_;

  ObstacleConf obstacle_conf_;

  ObstacleClusters* clusters_ptr_ = nullptr;
  JunctionAnalyzer* junction_analyzer_ = nullptr;

  std::vector<std::string> lane_id_past_;

  KalmanMotionFusion kalman_motion_fusion_;

  bool hesitant_lane_change_state_ = false;
  bool use_kalman_filter_ = false;
  int tracking_frame_ = 0;
  double occluded_prob_ = -1.0;

  std::string current_junction_id_ = " ";
};

}  // namespace prediction
}  // namespace TL
