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
 *   @file
 **/

#pragma once

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/status/status.h"
#include "common/thread/thread_pool.h"
#include "planning/common/dependency_injector.h"
#include "planning/common/frame.h"
#include "planning/common/history.h"
#include "planning/common/obstacle.h"
#include "planning/common/path_decision.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/speed/st_boundary.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/planning/decision.pb.h"

namespace TL {
namespace planning {

using TL::common::thread::ThreadPool;
constexpr double kADCSafetyLBufferForDynamicObstacle = 0.30;
constexpr double kSIgnoreThreshold = 0.01;
constexpr double kTIgnoreThreshold = 0.1;
constexpr double kOvertakenObsCautionTime = 0.5;
constexpr double kFullyEnterNeighborLanethreshold = 1.5;
constexpr int kPathRightThreadCount = 4;

class STObstaclesProcessor {
 public:
  enum class ObstacleLongitudinalPosition {
    FRONT,
    MIDDLE,
    BACK,
  };

  enum class ObstacleLateralPosition {
    LEFT,
    CENTER,
    RIGHT,
  };

  STObstaclesProcessor() = default;

  bool Init(double planning_distance, double planning_time,
            ReferenceLineInfo* reference_line_info, const Frame* frame,
            History* history, const STBoundsDeciderConfig* st_bound_config,
            const Frame* last_frame);

  virtual ~STObstaclesProcessor() = default;

  common::Status MapObstaclesToSTBoundaries(PathDecision* path_decision);

  /**
   * @brief Map lon nudge obstacles to st boundary
   * 
   * @param path_decision 
   * @return common::Status 
   */
  common::Status MapNudgeObstaclesToSTBoundaries(PathDecision* path_decision);

  const std::unordered_map<std::string, STBoundary>& GetAllSTBoundaries() {
    return obs_id_to_st_boundary_;
  }

  /** @brief Given a time t, get the lower and upper s-boundaries.
   * If the boundary is well-defined based on decision made previously,
   * fill "available_s_bounds" with only one boundary.
   * Otherwise, fill "available_s_bounds with all candidates and
   * "available_obs_decisions" with corresponding possible obstacle decisions.
   * @param Time t
   * @param The available s-boundaries to be filled up.
   * @param The corresponding possible obstacle decisions.
   * @return Whether we can get valid s-bounds.
   */
  bool GetSBoundsFromDecisions(
      double t, std::vector<std::pair<double, double>>* available_s_bounds,
      std::vector<std::vector<std::pair<std::string, ObjectDecisionType>>>*
          available_obs_decisions);

  /** @brief Provided that decisions for all existing obstacles are made, get
   * the speed limiting info from limiting st-obstacles.
   * @param Time t.
   * @param The actual limiting speed-info: (lower, upper)
   * @return True if there is speed limiting info; otherwise, false.
   */
  bool GetLimitingSpeedInfo(double t,
                            std::pair<double, double>* limiting_speed_info);

  /** @brief Set the decision for a given obstacle.
   */
  void SetObstacleDecision(const std::string& obs_id,
                           const ObjectDecisionType& obs_decision);

  /** @brief Set the decision for a list of obstacles.
   */
  void SetObstacleDecision(
      const std::vector<std::pair<std::string, ObjectDecisionType>>&
          obstacle_decisions);

  const std::vector<std::tuple<int, double, double, double, std::string>>&
  GetObsTEdges() const;

  /** @brief Given a single obstacle, compute its ST-boundary.
   * @param An obstacle (if moving, should contain predicted trajectory).
   * @param A vector to be filled with lower edge of ST-polygon.
   * @param A vector to be filled with upper edge of ST-polygon.
   * @return If appears on ST-graph, return true; otherwise, false.
   */
  bool ComputeObstacleSTBoundary(const Obstacle& obstacle,
                                 std::vector<STPoint>* lower_points,
                                 std::vector<STPoint>* upper_points,
                                 bool* is_caution_obstacle,
                                 double* obs_caution_end_t);

  bool ComputeStaticObstacleSTBoundary(const Obstacle& obstacle,
                                       std::vector<STPoint>* lower_points,
                                       std::vector<STPoint>* upper_points,
                                       bool* is_caution_obstacle,
                                       double* obs_caution_end_t);

  bool ComputeDynamicObstacleSTBoundary(const Obstacle& obstacle,
                                        std::vector<STPoint>* lower_points,
                                        std::vector<STPoint>* upper_points,
                                        bool* is_caution_obstacle,
                                        double* obs_caution_end_t);

  bool ComputeCruiseTargetSTBoundary(const Obstacle& obstacle,
                                     std::vector<STPoint>* lower_points,
                                     std::vector<STPoint>* upper_points);

  /** @brief Given ADC's path and an obstacle instance at a certain timestep,
   * get the upper and lower s that ADC might overlap with the obs instance.
   * @param A vector of ADC planned path points.
   * @param A obstacle at a certain timestep.
   * @param ADC lateral buffer for safety consideration.
   * @param The overlapping upper and lower s to be updated.
   * @return Whether there is an overlap or not.
   */
  bool GetOverlappingS(const std::vector<common::PathPoint>& adc_path_points,
                       const common::math::Box2d& obstacle_instance,
                       double adc_l_buffer,
                       std::pair<double, double>* overlapping_s);

 private:
  /** @brief Over the s-dimension, find the last point that is before the
   * obstacle instance of the first point that is after the obstacle.
   * If there exists no such point within the given range, return -1.
   * @param ADC path points
   * @param The obstacle box
   * @param The s threshold, must be non-negative.
   * @param The direction
   * @param The start-idx
   * @param The end-idx
   * @return Whether there is overlapping or not.
   */
  int GetSBoundingPathPointIndex(
      const std::vector<common::PathPoint>& adc_path_points,
      const common::math::Box2d& obstacle_instance, double s_thresh,
      bool is_before, int start_idx, int end_idx);

  /** @brief Over the s-dimension, check if the path-point is away
   * from the projected obstacle in the given direction, direction is from
   * path point to next path point
   * @param path_point_index path_point_index
   * @param obs_box obstacle bounding box.
   * @param s_thresh threshold s to tell if path point is far away.
   * @param is_before Direction indicator. True if we want the path-point to be
   *        before the obstacle.
   * @return whether the path-point is away in the indicated direction.
   */
  bool IsPathPointAwayFromObstacle(int path_point_index,
                                   const common::math::Box2d& obs_box,
                                   double s_thresh, bool is_before);

  /** @brief Check if ADC is overlapping with the given obstacle box.
   * @param ADC's position.
   * @param Obstacle's box.
   * @param ADC's lateral buffer.
   * @return Whether ADC at that position is overlapping with the given
   * obstacle box.
   */
  bool IsADCOverlappingWithObstacle(const common::PathPoint& adc_path_point,
                                    const common::math::Box2d& obs_box,
                                    double l_buffer) const;

  /** @brief Find the vertical (s) gaps of the st-graph.
   * @param Vector of obstacle-t-edges
   * @param The existing minimum s edge.
   * @param The existing maximum s edge.
   * @return A list of available s gaps for ADC to go.
   */
  static std::vector<std::pair<double, double>> FindSGaps(
      const std::vector<std::tuple<int, double, double, double, std::string>>&
          obstacle_t_edges,
      double s_min, double s_max);

  /** @brief Based on obstacle position and prospective ADC position,
   * determine the obstacle decision.
   * @param Obstacle's minimum s.
   * @param Obstacle's maximum s.
   * @param ADC's prospective position.
   * @return The decision for the given obstacle.
   */
  static ObjectDecisionType DetermineObstacleDecision(double obs_s_min,
                                                      double obs_s_max,
                                                      double s);

  /** @brief Check if a given s falls within adc's low road right segment.
   * @param A certain S.
   * @return True if within; false otherwise.
   */
  bool IsSWithinADCLowRoadRightSegment(double s) const;

  // 解决马路牙子和边界障碍物用最小矩形方法生成的box，由于heading问题导致的与自车碰撞的问题，实际上polygon不影响自车行驶，
  // 思路是找距离自车最近的polygon的点，判断是否在ADC的bound内（加上buffer）
  bool IsStaticObstaclePolygonPointInAdcBoundary(
      const std::vector<TL::common::PathPoint>& adc_path_points,
      double l_buffer, const Obstacle& obstacle) const;

  /**
   * @brief Check if this is lane turn path
   *
   */
  void CheckIsLaneTurnPath();

  /**
   * @brief Check if ignore obstacle
   *
   * @param obstacle
   * @return true ignore obstacle
   * @return false do not ignore obstacle
   */
  bool CheckIfIgnoreObstacle(Obstacle* obstacle);

  /**
   * @brief Calculate obstacle lateral position
   * 
   * @param obstacle 
   * @return ObstacleLateralPosition 
   */
  ObstacleLateralPosition CalculateObstacleLateralPosition(
      const Obstacle& obstacle);

  /**
   * @brief Calculate obstacle longitudinal position
   * 
   * @param obstacle 
   * @return ObstacleLongitudinalPosition 
   */
  ObstacleLongitudinalPosition CalculateObstacleLongitudinalPosition(
      const Obstacle& obstacle);

  /**
   * @brief Check if obstacle is directly behind adc
   * 
   * @param obstacle 
   * @param lateral_position 
   * @param longitudinal_position 
   * @return true 
   * @return false 
   */
  bool CheckIfDirectlyBehind(
      const Obstacle& obstacle, const ObstacleLateralPosition& lateral_position,
      const ObstacleLongitudinalPosition& longitudinal_position);

  /**
   * @brief Check if obstacle trajectory has overlap with path caution region
   * 
   * @param obstacle 
   * @param direction 
   * @return true 
   * @return false 
   */
  bool CheckIfHasOverlapWithPathCaution(
      const Obstacle& obstacle,
      const PathData::PathCautionDirection& direction);

  /**
   * @brief Check if obstacle trajectory has overlap with path region
   * 
   * @param obstacle 
   * @return true 
   * @return false 
   */
  bool CheckIfHasOverlapWithPath(const Obstacle& obstacle);

  /**
   * @brief Calculate path road right
   * 
   */
  void CalculatePathRoadRight();

  void CheckIfKeepRoadRight();

  void CalculatePathFrenetInfos();

  void CalculatePathCautionEnvelopes();

  void CalculateLaneTurnPathCautionEnvelopes(
      const PathData::RoadRightType& low_road_right_type,
      std::vector<PathData::PathCautionEnvelope>* caution_path_envelopes);

  void CalculateLaneCrossPathCautionEnvelopes(
      const PathData::RoadRightType& low_road_right_type,
      std::vector<PathData::PathCautionEnvelope>* caution_path_envelopes);

  /**
    * @brief Check if corner enter neighbor lane
    * 
    * @param xy_corner xy position of corner
    * @param sl_corner sl position of corner
    * @param lane_type 
    * @param threshold 
    * @param exclude_lane_ids 
    * @return true  corner has enter neighbor lane
    * @return false  corner has enter neighbor lane
    */
  bool CheckIfEnterNeighborLane(
      const Vec2d& xy_corner, const common::SLPoint& sl_corner,
      ReferenceLineInfo::LaneType lane_type, double threshold,
      const std::set<std::string>& exclude_lane_ids) const;

  double planning_time_ = 0.0;
  double planning_distance_ = 0.0;
  common::VehicleParam vehicle_param_;
  double adc_path_init_s_ = 0.0;
  ReferenceLineInfo* reference_line_info_ = nullptr;
  const ReferenceLineInfo* adc_reference_line_info_ = nullptr;
  // A vector of sorted obstacle's t-edges:
  //  (is_starting_t, t, s_min, s_max, obs_id).
  std::vector<std::tuple<int, double, double, double, std::string>>
      obs_t_edges_;
  int obs_t_edges_idx_ = 0;

  std::unordered_map<std::string, STBoundary> obs_id_to_st_boundary_;
  std::unordered_map<std::string, ObjectDecisionType> obs_id_to_decision_;

  std::vector<std::tuple<std::string, STBoundary, Obstacle*>>
      candidate_clear_zones_;

  std::unordered_map<std::string, STBoundary>
      obs_id_to_alternative_st_boundary_;

  std::mutex lock_0_;
  std::mutex lock_1_;
  std::mutex lock_2_;
  std::mutex lock_3_;
  std::mutex lock_4_;

  std::vector<std::pair<double, double>> adc_low_road_right_segments_;

  History* history_ = nullptr;

  const Frame* frame_ = nullptr;
  const Frame* last_frame_ = nullptr;
  const STBoundsDeciderConfig* st_bounds_config_ = nullptr;
  bool is_lane_turn_path_ = false;
  std::vector<common::math::LineSegment2d> path_dir_line_segments_;
  std::vector<common::math::LineSegment2d> normal_line_segments_;
  bool is_avp_mode_ = false;
  bool keep_left_road_right_ = false;
  bool keep_right_road_right_ = false;
};

}  // namespace planning
}  // namespace TL
