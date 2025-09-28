#ifndef PLANNING_COMMON_REFERENCE_LINE_INFO_H
#define PLANNING_COMMON_REFERENCE_LINE_INFO_H

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
 **/

#include <cstdint>
#pragma once

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "common/thread/thread_pool.h"
#include "map/hdmap/hdmap_common.h"
#include "map/hdmap/path.h"
#include "planning/common/path/path_data.h"
#include "planning/common/path_boundary.h"
#include "planning/common/path_decision.h"
#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/speed/speed_data.h"
#include "planning/common/st_graph_data.h"
#include "planning/common/trajectory/discretized_trajectory.h"
#include "planning/pnc_map/new_framework/pnc_map_b.h"
#include "planning/pnc_map/route_segments.h"
#include "planning/proto/lattice_structure.pb.h"
#include "proto/common/drive_state.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/planning/decision.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {
using TL::common::thread::ThreadPool;
// PathBoundPoint contains: (s, l_min, l_max).
using PathBoundPoint = std::tuple<double, double, double>;
// PathBound contains a vector of PathBoundPoints.
using PathBound = std::vector<PathBoundPoint>;
// <interactive_obs_id>, <adc_yield_overtake, <adc_behavior, obs_behavior>>
using InteractiveDecision = std::pair<
    std::string,
    std::pair<ObjectDecisionType, std::pair<InteractiveObsBehavior::Behavior,
                                            InteractiveObsBehavior::Behavior>>>;

/**
 * @class ReferenceLineInfo
 * @brief ReferenceLineInfo holds all data for one reference line.
 */
class ReferenceLineInfo {
 public:
  enum class LaneType { LeftForward, LeftReverse, RightForward, RightReverse };
  ReferenceLineInfo() = default;

  ReferenceLineInfo(
      const std::shared_ptr<common::VehicleState>& vehicle_state_ptr,
      common::TrajectoryPoint adc_planning_point,
      const std::shared_ptr<ReferenceLine>& reference_line,
      hdmap::RouteSegments segments);

  bool Init(const std::vector<const std::shared_ptr<Obstacle>*>& obstacles,
            const TL::functionmanager::FunctionManagerIn& fct_manager_input);

  bool GetProjection();

  bool AddObstacles(
      const std::vector<const std::shared_ptr<Obstacle>*>& obstacles);
  Obstacle* AddObstacle(const std::shared_ptr<Obstacle>& obstacle);

  bool GetObstacleProjection(Obstacle* mutable_obstacle);

  const common::VehicleState& vehicle_state() const {
    return *vehicle_state_ptr_;
  }

  PathDecision* path_decision();
  const PathDecision& path_decision() const;

  const ReferenceLine& reference_line() const;
  std::shared_ptr<ReferenceLine> mutable_reference_line();

  double SDistanceToDestination(bool solution_through = false) const;
  bool ReachedDestination() const;

  void SetTrajectory(const DiscretizedTrajectory& trajectory);
  const DiscretizedTrajectory& trajectory() const;

  double Cost() const { return cost_; }

  void AddCost(double cost) { cost_ += cost; }

  void SetCost(double cost) { cost_ = cost; }

  double PriorityCost() const { return priority_cost_; }

  void SetPriorityCost(double cost) { priority_cost_ = cost; }

  // For lattice planner'speed planning target
  void SetLatticeStopPoint(const StopPoint& stop_point);
  void SetLatticeCruiseSpeed(double speed);

  const PlanningTarget& planning_target() const { return planning_target_; }

  void SetCruiseSpeed(double speed) { cruise_speed_ = speed; }

  double GetCruiseSpeed() const { return cruise_speed_; }

  double GetMaxSpeed() const { return max_speed_; }

  void SetMaxSpeed(double max_speed) { max_speed_ = max_speed; }

  double GetMaxDeceleration() const { return max_deceleration_; }

  void SetMaxDeceleration(double max_deceleration) {
    max_deceleration_ = fmin(max_deceleration_, max_deceleration);
  }

  double maxAcceleration() const { return max_acceleration_; }

  void SetMaxAcceleration(double max_acceleration) {
    max_acceleration_ = fmax(max_acceleration_, max_acceleration);
  }

  double GetLonCtrlTime() const { return longitud_ctrl_set_time_; }

  void SetLonCtrlTime(double longitud_ctrl_set_time) {
    longitud_ctrl_set_time_ = longitud_ctrl_set_time;
  }

  double GetLonCtrlMinDis() const { return longitud_ctrl_min_dis_; }

  void SetLonCtrlMinDis(double longitud_ctrl_min_dis) {
    longitud_ctrl_min_dis_ = longitud_ctrl_min_dis;
  }

  hdmap::LaneInfoConstPtr LocateLaneInfo(double s) const;

  bool GetNeighborLaneInfo(ReferenceLineInfo::LaneType lane_type, double s,
                           hdmap::Id* ptr_lane_id,
                           double* ptr_lane_width) const;

  hdmap::LaneInfoConstPtr GetNeighborLaneInfo(
      ReferenceLineInfo::LaneType lane_type, double s) const;

  /**
   * @brief check if current reference line is started from another reference
   *line info line. The method is to check if the start point of current
   *reference line is on previous reference line info.
   * @return returns true if current reference line starts on previous reference
   *line, otherwise false.
   **/
  bool IsStartFrom(const ReferenceLineInfo& previous_reference_line_info) const;

  planning_internal::Debug* mutable_debug() { return &debug_; }

  const planning_internal::Debug& debug() const { return debug_; }

  LatencyStats* mutable_latency_stats() { return &latency_stats_; }

  const LatencyStats& latency_stats() const { return latency_stats_; }

  const PathData& path_data() const;
  const PathData& fallback_path_data() const;
  const SpeedData& speed_data() const;
  PathData* mutable_path_data();
  PathData* mutable_fallback_path_data();
  SpeedData* mutable_speed_data();

  const RSSInfo& rss_info() const;
  RSSInfo* mutable_rss_info();
  // aggregate final result together by some configuration
  bool CombinePathAndSpeedProfile(
      double relative_time, double start_s,
      DiscretizedTrajectory* discretized_trajectory);

  const SLBoundary& AdcSlBoundary() const { return adc_sl_boundary_; }

  std::string PathSpeedDebugString() const;

  /**
   * Check if the current reference line is a change lane reference line, i.e.,
   * ADC's current position is not on this reference line.
   */
  bool IsChangeLanePath() const;

  /**
   * Check if the current reference line is the neighbor of the vehicle
   * current position
   */
  bool IsNeighborLanePath() const;

  /**
   * Set if the vehicle can drive following this reference line
   * A planner need to set this value to true if the reference line is OK
   */
  void SetDrivable(bool drivable);
  bool IsDrivable() const;

  void ExportEngageAdvice(common::EngageAdvice* engage_advice,
                          PlanningContext* planning_context) const;

  const hdmap::RouteSegments& Lanes() const;
  hdmap::RouteSegments* GetMutableLanes();
  std::list<hdmap::Id> TargetLaneId() const;

  void ExportDecision(DecisionResult* decision_result,
                      PlanningContext* planning_context,
                      functionmanager::TaPilotMode ta_pilot_mode) const;

  void SetJunctionRightOfWay(double junction_s, bool is_protected) const;

  ADCTrajectory::RightOfWayStatus GetRightOfWayStatus() const;

  hdmap::Lane::LaneTurn GetPathTurnType(double s) const;

  bool GetIntersectionRightofWayStatus(
      const hdmap::PathOverlap& pnc_junction_overlap) const;

  double OffsetToOtherReferenceLine() const {
    return offset_to_other_reference_line_;
  }

  void SetOffsetToOtherReferenceLine(const double offset) {
    offset_to_other_reference_line_ = offset;
  }

  const std::vector<PathBoundary>& GetCandidatePathBoundaries() const;

  void SetCandidatePathBoundaries(
      std::vector<PathBoundary>&& candidate_path_boundaries);

  void SetGuideLineBound(const PathBound& guide_line_bound) {
    guide_line_bound_ = guide_line_bound;
  }

  const PathBound& GetGuideLineBound() const { return guide_line_bound_; }

  void SetAstarSearchBound(const PathBound& astar_search_bound) {
    astar_search_bound_ = astar_search_bound;
  }

  const PathBound& GetAstarSearchBound() const { return astar_search_bound_; }

  void SetRefLineOffsetBound(const PathBound& ref_line_offset_bound) {
    ref_line_offset_bound_ = ref_line_offset_bound;
  }

  const PathBound& GetRefLineOffsetBound() const {
    return ref_line_offset_bound_;
  }

  const std::vector<std::pair<common::math::LineSegment2d, double>>&
  GetConcernedFSSegments() const {
    return concerned_fs_segments_;
  }

  void SetConcernedFSSegments(
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          concerned_fs_segments) {
    concerned_fs_segments_ = concerned_fs_segments;
  }

  const std::vector<PathData>& GetCandidatePathData() const;

  void SetCandidatePathData(std::vector<PathData>&& candidate_path_data);

  Obstacle* GetBlockingObstacle() const { return blocking_obstacle_; }

  void SetBlockingObstacle(const std::string& blocking_obstacle_id);

  bool is_path_lane_borrow() const { return is_path_lane_borrow_; }

  void set_is_path_lane_borrow(const bool is_path_lane_borrow) {
    is_path_lane_borrow_ = is_path_lane_borrow;
  }

  void set_is_on_reference_line() { is_on_reference_line_ = true; }

  bool IsOnReferenceLine() const { return is_on_reference_line_; }

  void SetIsTargetReferenceLineInfo(
      const bool is_target_reference_line_info = true) {
    is_target_reference_line_info_ = is_target_reference_line_info;
  }

  bool IsTargetReferenceLineInfo() const {
    return is_target_reference_line_info_;
  }

  void SetIsHistoryTrace(const bool is_history_trace = true) {
    is_history_trace_ = is_history_trace;
  }

  bool IsHistoryTrace() const { return is_history_trace_; }

  void SetPathBoundsDeciderResolution(
      const double path_bounds_decider_resolution) {
    path_bounds_decider_resolution_ = path_bounds_decider_resolution;
  }

  double PathBoundsDeciderResolution() const {
    return path_bounds_decider_resolution_;
  }

  uint32_t GetPriority() const { return reference_line_->GetPriority(); }

  void SetPriority(uint32_t priority) {
    reference_line_->SetPriority(priority);
  }

  void set_trajectory_type(
      const ADCTrajectory::TrajectoryType trajectory_type) {
    trajectory_type_ = trajectory_type;
  }

  ADCTrajectory::TrajectoryType trajectory_type() const {
    return trajectory_type_;
  }

  StGraphData* mutable_st_graph_data() { return &st_graph_data_; }

  const StGraphData& st_graph_data() const { return st_graph_data_; }

  // different types of overlaps that can be handled by different scenarios.
  enum OverlapType {
    CLEAR_AREA = 1,
    CROSSWALK = 2,
    OBSTACLE = 3,
    PNC_JUNCTION = 4,
    SIGNAL = 5,
    STOP_SIGN = 6,
    YIELD_SIGN = 7,
  };

  const std::vector<std::pair<OverlapType, hdmap::PathOverlap>>&
  FirstEncounteredOverlaps() const {
    return first_encounter_overlaps_;
  }

  int GetPnCJunction(double s, hdmap::PathOverlap* pnc_junction_overlap) const;

  std::vector<common::SLPoint> GetAllStopDecisionSLPoint() const;

  void SetTurnSignal(const common::VehicleSignal::TurnSignal& turn_signal);
  void SetEmergencyLight();

  void set_path_reusable(const bool path_reusable) {
    path_reusable_ = path_reusable;
  }

  bool path_reusable() const { return path_reusable_; }

  void SetTowingPoints(
      std::vector<std::tuple<double, double, double, std::string>>* const
          towing_points) {
    towing_points_.swap(*towing_points);
  }

  std::vector<std::tuple<double, double, double, std::string>> GetTowingPoints()
      const {
    return towing_points_;
  }

  void SetTowingLine(std::vector<double>* const towing_line) {
    towing_line_.swap(*towing_line);
  }

  std::vector<double> GetTowingLine() const { return towing_line_; }

  const std::unordered_set<std::string>& GetBigCarSet() const {
    return big_car_set_;
  }

  std::unordered_set<std::string>* GetMutableBigCarSet() {
    return &big_car_set_;
  }

  const std::unordered_set<std::string>& GetOvertakeBigCarSet() const {
    return overtake_big_car_set_;
  }

  std::unordered_set<std::string>* GetMutableOvertakeBigCarSet() {
    return &overtake_big_car_set_;
  }

  std::vector<InteractiveDecision>* GetMutableAdcObsBehavior() {
    return &adc_obs_behavior_pair_vec_;
  }

  const std::vector<InteractiveDecision>& GetAdcObsBehavior() const {
    return adc_obs_behavior_pair_vec_;
  }

  /**
   * @brief Get the Route Segments object
   * 
   * @return const hdmap::RouteSegments& 
   */
  const hdmap::RouteSegments& GetRouteSegments() { return lanes_; }

  /**
   * @brief Set the Lon Follow Obs Id object
   * 
   * @param follow_obs_perception_id 
   */
  void SetLonFollowObsId(const std::string& follow_obs_id) {
    follow_obs_id_ = follow_obs_id;
  }

  /**
   * @brief Get the Lon Follow Obs Id object
   * 
   * @return  std::string  
   */
  std::string GetLonFollowObsId() const { return follow_obs_id_; }

  const common::SLPoint& GetAdcSLPoint() const { return adc_sl_point_; }

  /**
   * @brief Set the Front Ramp object
   * 
   * @param front_ramp 
   */
  void SetFrontRamp(const hdmap::LaneRangeInfo& front_ramp) {
    front_ramp_ = front_ramp;
  }

  /**
   * @brief Get the Front Ramp object
   * 
   * @return const hdmap::LaneRangeInfo& 
   */
  const hdmap::LaneRangeInfo& GetFrontRamp() const { return front_ramp_; }

  /**
   * @brief Set the Front Tunnel object
   * 
   * @param front_tunnel 
   */
  void SetFrontTunnel(const hdmap::LaneRangeInfo& front_tunnel) {
    front_tunnel_ = front_tunnel;
  }

  /**
   * @brief Get the Front Tunnel object
   * 
   * @return const hdmap::LaneRangeInfo& 
   */
  const hdmap::LaneRangeInfo& GetFrontTunnel() const { return front_tunnel_; }

 private:
  void InitFirstOverlaps();

  bool CheckChangeLane() const;

  void SetTurnSignalBasedOnLaneTurnType(
      common::VehicleSignal* vehicle_signal) const;

  void ExportVehicleSignal(common::VehicleSignal* vehicle_signal,
                           functionmanager::TaPilotMode ta_pilot_mode) const;

  bool IsIrrelevantObstacle(const Obstacle& obstacle);

  void MakeDecision(DecisionResult* decision_result,
                    PlanningContext* planning_context) const;

  int MakeMainStopDecision(DecisionResult* decision_result) const;

  void MakeMainMissionCompleteDecision(DecisionResult* decision_result,
                                       PlanningContext* planning_context) const;

  void MakeEStopDecision(DecisionResult* decision_result) const;

  void SetObjectDecisions(ObjectDecisions* object_decisions) const;

  bool AddObstacleHelper(const std::shared_ptr<Obstacle>& obstacle);

  bool GetFirstOverlap(const std::vector<hdmap::PathOverlap>& path_overlaps,
                       hdmap::PathOverlap* path_overlap);

 private:
  static std::unordered_map<std::string, bool> junction_right_of_way_map_;
  std::shared_ptr<common::VehicleState> vehicle_state_ptr_;
  const common::TrajectoryPoint adc_planning_point_;
  std::shared_ptr<ReferenceLine> reference_line_;

  /**
   * @brief this is the number that measures the goodness of this reference
   * line. The lower the better.
   */
  double cost_ = 0.0;

  bool is_drivable_ = true;

  PathDecision path_decision_;

  std::mutex lock_add_obs_;
  std::mutex lock_add_obs_build_;

  Obstacle* blocking_obstacle_ = nullptr;

  std::vector<PathBoundary> candidate_path_boundaries_;
  PathBound guide_line_bound_;
  PathBound astar_search_bound_;
  PathBound ref_line_offset_bound_;
  std::vector<PathData> candidate_path_data_;

  PathData path_data_;
  PathData fallback_path_data_;
  SpeedData speed_data_;

  DiscretizedTrajectory discretized_trajectory_;

  RSSInfo rss_info_;

  /**
   * @brief SL boundary of stitching point (starting point of plan trajectory)
   * relative to the reference line
   */
  SLBoundary adc_sl_boundary_;

  planning_internal::Debug debug_;
  LatencyStats latency_stats_;

  hdmap::RouteSegments lanes_;

  bool is_on_reference_line_ = false;

  bool is_path_lane_borrow_ = false;

  ADCTrajectory::RightOfWayStatus status_ = ADCTrajectory::UNPROTECTED;

  double offset_to_other_reference_line_ = 0.0;

  double priority_cost_ = 0.0;

  PlanningTarget planning_target_;

  ADCTrajectory::TrajectoryType trajectory_type_ = ADCTrajectory::UNKNOWN;

  /**
   * Overlaps encountered in the first time along the reference line in front of
   * the vehicle
   */
  std::vector<std::pair<OverlapType, hdmap::PathOverlap>>
      first_encounter_overlaps_;

  /**
   * @brief Data generated by speed_bounds_decider for constructing st_graph for
   * different st optimizer
   */
  StGraphData st_graph_data_;

  common::VehicleSignal vehicle_signal_;

  double cruise_speed_ = 0.0;

  double max_speed_ = FLAGS_planning_upper_speed_limit;

  double max_deceleration_ = 0.0;

  double max_acceleration_ = 0.0;

  double longitud_ctrl_set_time_ = FLAGS_follow_time_desired;

  double longitud_ctrl_min_dis_ = 8.0;

  bool path_reusable_ = false;

  bool is_target_reference_line_info_ = false;

  bool is_history_trace_ = false;

  double path_bounds_decider_resolution_ = 0.5;

  // Towing Points: (s, left towing_l, right towing_l, obs id)
  std::vector<std::tuple<double, double, double, std::string>> towing_points_;

  // Towing Line: (towing l)
  std::vector<double> towing_line_;

  std::unordered_set<std::string> big_car_set_;

  std::unordered_set<std::string> overtake_big_car_set_;

  std::vector<InteractiveDecision> adc_obs_behavior_pair_vec_;
  std::string follow_obs_id_;
  common::SLPoint adc_sl_point_;

  std::vector<std::pair<common::math::LineSegment2d, double>>
      concerned_fs_segments_;

  hdmap::LaneRangeInfo front_ramp_{nullptr, 0.0, 0.0,
                                   std::numeric_limits<double>::infinity(),
                                   TL::hdmap::LaneType::RAMP};
  hdmap::LaneRangeInfo front_tunnel_{nullptr, 0.0, 0.0,
                                     std::numeric_limits<double>::infinity(),
                                     TL::hdmap::LaneType::TUNNEL_LANE};
  // DISALLOW_COPY_AND_ASSIGN(ReferenceLineInfo);
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_REFERENCE_LINE_INFO_H
