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

#pragma once

#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "planning/common/ego_info.h"
#include "planning/common/indexed_queue.h"
#include "planning/common/obstacle.h"
#include "planning/common/open_space_info.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/trajectory/publishable_trajectory.h"
#include "planning/localview/local_view.h"
#include "planning/reference_line/reference_line_provider.h"
#include "planning/tasks/deciders/obstacles_decider/obstacle_selector/obstacles_info_updater.h"

#include "planning/proto/planning_config.pb.h"
#include "proto/common/types.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/planning/pad_msg.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/planning/planning_internal.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {

enum ForceRplanType {
  NO_FORCE_REPLAN = 0,
  LAT_FORCE_REPLAN = 1,
  LON_FORCE_REPLAN = 2,
  LAT_LON_FORCE_REPLAN = 3,
};

/**
 * @class Frame
 *
 * @brief Frame holds all data for one planning cycle.
 */

class Frame {
 public:
  explicit Frame(uint32_t sequence_num);

  Frame(uint32_t sequence_num, const std::shared_ptr<LocalView>& local_view,
        common::TrajectoryPoint planning_start_point,
        common::VehicleState vehicle_state,
        ReferenceLineProvider* reference_line_provider);

  Frame(uint32_t sequence_num, const std::shared_ptr<LocalView>& local_view,
        const common::TrajectoryPoint& planning_start_point,
        const common::VehicleState& vehicle_state);

  virtual ~Frame() = default;

  const common::TrajectoryPoint& PlanningStartPoint() const;

  void SetPlanningStartPoint(const common::TrajectoryPoint& new_start_point) {
    planning_start_point_ = new_start_point;
  }

  common::Status Init(
      const common::VehicleState* vehicle_state,
      const std::list<std::shared_ptr<ReferenceLine>>& reference_lines,
      const std::list<hdmap::RouteSegments>& segments,
      const std::vector<routing::LaneWaypoint>& future_route_waypoints,
      const EgoInfo* ego_info);

  common::Status InitForOpenSpace(const common::VehicleState* vehicle_state,
                                  const EgoInfo* ego_info);

  uint32_t SequenceNum() const;

  std::string DebugString() const;

  const PublishableTrajectory& ComputedTrajectory() const;

  static void RecordInputDebug(planning_internal::Debug* debug);

  const std::list<ReferenceLineInfo>& reference_line_info() const;
  std::list<ReferenceLineInfo>* mutable_reference_line_info();

  Obstacle* Find(const std::string& id);

  const ReferenceLineInfo* FindDriveReferenceLineInfo();

  const ReferenceLineInfo* FindTargetReferenceLineInfo();

  const ReferenceLineInfo* FindFailedReferenceLineInfo();

  const ReferenceLineInfo* DriveReferenceLineInfo() const;

  std::vector<const std::shared_ptr<Obstacle>*> obstacles() const;

  std::shared_ptr<Obstacle> CreateStopObstacle(
      ReferenceLineInfo* reference_line_info, const std::string& obstacle_id,
      double obstacle_s);

  std::shared_ptr<Obstacle> CreateStopObstacle(const std::string& obstacle_id,
                                               const std::string& lane_id,
                                               double lane_s);

  std::shared_ptr<Obstacle> CreateStaticObstacle(
      ReferenceLineInfo* reference_line_info, const std::string& obstacle_id,
      double obstacle_start_s, double obstacle_end_s);

  const ObstaclesInfo& GetObstacleInfo() const { return obstacles_info_; }

  void SetObstacleInfo(const planning::ObstaclesInfo& obstacles_info) {
    obstacles_info_ = obstacles_info;
  }

  bool Rerouting(PlanningContext* planning_context);

  const common::VehicleState& vehicle_state() const;

  /**
   * @brief Check is vehicle stand still
   *
   * @return true
   * @return false
   */
  inline bool IsVehicleStandStill() const {
    constexpr double kStandStillSpd = 0.05;
    return fabs(vehicle_state_.linear_velocity()) <= kStandStillSpd;
  }

  static void AlignPredictionTime(
      double planning_start_time,
      prediction::PredictionObstacles* prediction_obstacles);

  void set_current_frame_planned_trajectory(
      const std::shared_ptr<ADCTrajectory>& current_frame_planned_trajectory) {
    current_frame_planned_trajectory_ = current_frame_planned_trajectory;
  }

  std::shared_ptr<ADCTrajectory> current_frame_planned_trajectory() const {
    return current_frame_planned_trajectory_;
  }

  void set_current_frame_planned_path(
      const std::shared_ptr<DiscretizedPath>& current_frame_planned_path) {
    current_frame_planned_path_ = current_frame_planned_path;
  }

  std::shared_ptr<DiscretizedPath> current_frame_planned_path() const {
    return current_frame_planned_path_;
  }

  bool is_near_destination() const { return is_near_destination_; }

  void set_local_free_space_info_vec(
      const std::vector<TL::perception::FreeSpaceOutArray>&
          tmp_local_free_space_info) {
    local_free_space_info_vec_ = tmp_local_free_space_info;
  }

  const std::vector<TL::perception::FreeSpaceOutArray>&
  get_local_free_space_info_vec() const {
    return local_free_space_info_vec_;
  }

  void set_world_free_space_info_vec(
      const std::vector<TL::perception::FreeSpaceOutArray>&
          tmp_world_free_space_info) {
    world_free_space_info_vec_ = tmp_world_free_space_info;
  }

  const std::vector<TL::perception::FreeSpaceOutArray>&
  get_world_free_space_info_vec() const {
    return world_free_space_info_vec_;
  }

  std::array<common::math::Vec2d, 4> get_parking_lot_vertices() const {
    return parking_lot_vertices_;
  }

  void set_parking_lot_vertices(
      const std::array<common::math::Vec2d, 4>& parking_lot_vertices) {
    parking_lot_vertices_ = parking_lot_vertices;
  }

  /**
   * @brief Adjust reference line priority according to actual road conditions
   * @id_to_priority lane id and reference line priority mapping relationship
   */
  void UpdateReferenceLinePriority(
      const std::map<std::string, uint32_t>& id_to_priority);

  const LocalView& local_view() const { return *local_view_; }

  ThreadSafeIndexedObstacles* GetObstacleList() { return &obstacles_; }

  const OpenSpaceInfo& open_space_info() const { return open_space_info_; }

  OpenSpaceInfo* mutable_open_space_info() { return &open_space_info_; }

  perception::TrafficLight GetSignal(const std::string& traffic_light_id) const;
  perception::TrafficLight GetSignalByTurnType(
      const hdmap::Lane::LaneTurn& turn_type) const;

  static const DrivingAction& GetPadMsgDrivingAction() {
    return pad_msg_driving_action_;
  }

  const functionmanager::MachineStateType& GetMachineStateType() const {
    return cur_state_machine_;
  }

  void SetMachineStateType(const functionmanager::MachineStateType& state_type);

  bool GetIsStateChange() const { return is_state_change_; }

  void SetIsStateChange(bool is_state_change);

  void SetGLobalTrafficLightColor(
      perception::Color global_traffic_light_color) {
    global_traffic_light_color_ = global_traffic_light_color;
  }

  void SetTargetGear(const soc::Chassis::GearPosition& gear);

  const soc::Chassis::GearPosition& GetTargetGear() const;

  const std::vector<routing::LaneWaypoint>& GetFutureRouteWayPoints() const {
    return future_route_waypoints_;
  }

  bool GetIsLateralReplan() const { return is_lateral_replan_; }

  void SetIsLateralReplan(const bool is_lateral_replan) {
    is_lateral_replan_ = is_lateral_replan;
  }

  bool GetIsLongitudinalReplan() const { return is_longitudinal_replan_; }

  void SetIsLongitudinalReplan(const bool is_longitudinal_replan) {
    is_longitudinal_replan_ = is_longitudinal_replan;
  }

  bool GetIsSpeedFallback() const { return is_speed_fallback_; }

  void SetIsSpeedFallback(const bool is_speed_fallback) {
    is_speed_fallback_ = is_speed_fallback;
  }

  bool GetIsCIPVSpeedFallback() const { return is_cipv_speed_fallback_; }

  void SetIsCIPVSpeedFallback(const bool is_cipv_speed_fallback) {
    is_cipv_speed_fallback_ = is_cipv_speed_fallback;
  }

  bool GetIsMergeStopFallback() const { return is_merge_stop_fallback_; }

  void SetIsMergeStopFallback(const bool is_merge_stop_fallback) {
    is_merge_stop_fallback_ = is_merge_stop_fallback;
  }

  bool GetIsStopFallback() const { return is_stop_fallback_; }

  void SetIsStopFallback(const bool is_stop_fallback) {
    is_stop_fallback_ = is_stop_fallback;
  }

  /**
   * @brief Set the Path Valid object
   * 
   * @param is_path_vaild 
   */
  void SetPathValid(const bool is_path_vaild) {
    is_path_vaild_ = is_path_vaild;
  }

  /**
   * @brief Is Path Valid
   * 
   * @return true 
   * @return false 
   */
  bool IsPathValid() const { return is_path_vaild_; }

  /**
   * @brief SetIsDecByRamp speed dec by ramp
   */
  void SetIsDecByRamp(const bool is_dec_by_ramp) {
    is_dec_by_ramp_ = is_dec_by_ramp;
  }

  /**
   * @brief GetIsDecByRamp speed dec by ramp
   * 
   */
  bool GetIsDecByRamp() const { return is_dec_by_ramp_; }

  /**
   * @brief Get reference line provider
   * 
   * @return const ReferenceLineProvider* 
   */
  const ReferenceLineProvider* GetReferenceLineProvider() const {
    return reference_line_provider_;
  }

  /**
   * @brief 
   * 
   * @param obs_segments_pair 
   * @return TL::common::Status 
   */
  TL::common::Status OpenSpaceCollisionCheck(
      const std::vector<std::pair<common::math::LineSegment2d, double>>*
          obs_segments_pair = nullptr);

  /**
   * @brief Set the Is Standstill object
   * 
   * @param is_standstill 
   */
  void SetSpeedPlanStandstill(const bool is_speed_plan_standstill) {
    is_speed_plan_standstill_ = is_speed_plan_standstill;
  }

  /**
   * @brief 
   * 
   * @return true 
   * @return false 
   */
  bool IsSpeedPlanStandstill() const { return is_speed_plan_standstill_; }

  /**
   * @brief Set the Force Rplan Type object
   * 
   * @param force_replan_type 
   */
  void SetForceRplanType(const ForceRplanType& force_replan_type) {
    force_replan_type_ = force_replan_type;
  }

  /**
   * @brief Get the Force Rplan Type object
   * 
   * @return const ForceRplanType& 
   */
  ForceRplanType GetForceRplanType() const { return force_replan_type_; }

  /**
   * @brief Set the Lon Stop Obs Id object
   * 
   * @param id 
   */
  void SetLonStopObsId(const std::string& id) { lon_stop_obs_id_ = id; }

  /**
   * @brief 
   * 
   * @return int32_t 
   */
  const std::string& LonStopObsId() const { return lon_stop_obs_id_; }

  void SetObsGameIntention(const std::unordered_map<int, GameObstacleIntention>&
                               game_obs_intention) {
    game_obs_intention_ = game_obs_intention;
  }

  const std::unordered_map<int, GameObstacleIntention>& GetObsGameIntention()
      const {
    return game_obs_intention_;
  }

  void SetStitchingTrajectoryPoints(
      const RepeatedPtrField<TrajectoryPoint>& sti_traj_points) {
    if (stitching_trajectory_points_ptr_ != nullptr) {
      *stitching_trajectory_points_ptr_ = sti_traj_points;
    }
  }

  void SetStitchingTrajectoryPointsPtr(
      RepeatedPtrField<TrajectoryPoint>* sti_traj_points_ptr) {
    stitching_trajectory_points_ptr_ = sti_traj_points_ptr;
  }

  RepeatedPtrField<TrajectoryPoint>* GetStitchingTrajectoryPointsPtr() const {
    return stitching_trajectory_points_ptr_;
  }

  /**
   * @brief Set the Ego Prediction Trajectory
   */
  void SetEgoPredTrajectory();

  /**
   * @brief Get the Ego Prediction Trajectory
   * 
   * @return shared_ptr<RepeatedPtrField<Trajectory>>
   */
  const RepeatedPtrField<prediction::Trajectory>& GetEgoPredTrajectory() const {
    return ego_prediction_trajectories_;
  }

 private:
  common::Status InitFrameData(const common::VehicleState* vehicle_state,
                               const EgoInfo* ego_info);

  bool CreateReferenceLineInfo(
      const std::list<std::shared_ptr<ReferenceLine>>& reference_lines,
      const std::list<hdmap::RouteSegments>& segments);

  /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if
   * such obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
  const Obstacle* FindCollisionObstacle(const EgoInfo* ego_info) const;

  /**
   * @brief create a static virtual obstacle
   */
  std::shared_ptr<Obstacle> CreateStaticVirtualObstacle(
      const std::string& id, const common::math::Box2d& box);
  /**
   * @brief 
   * 
   * @param obstacle 
   */
  void AddObstacle(const std::shared_ptr<Obstacle>& obstacle);
  /**
   * @brief 
   * 
   */

  void ReadTrafficLights();
  /**
   * @brief 
   * 
   */

  void ReadPadMsgDrivingAction();
  /**
   * @brief 
   * 
   */
  static void ResetPadMsgDrivingAction();

  void MakeObstacleDecisionForAccMode();

 private:
  // @brief open_space end configuration in order of x, y, heading and speed.
  // Speed is set to be always zero now for parking   world frame
  bool is_near_destination_ = false;

  /// AVP freespace
  std::vector<TL::perception::FreeSpaceOutArray> local_free_space_info_vec_;
  std::vector<TL::perception::FreeSpaceOutArray> world_free_space_info_vec_;

  // static DrivingAction pad_msg_driving_action_;
  static DrivingAction pad_msg_driving_action_;
  uint32_t sequence_num_ = 0;
  std::shared_ptr<LocalView> local_view_ = nullptr;
  std::shared_ptr<hdmap::HDMap> hdmap_ = nullptr;
  common::TrajectoryPoint planning_start_point_;
  common::VehicleState vehicle_state_;
  std::list<ReferenceLineInfo> reference_line_info_;
  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
  const ReferenceLineInfo* drive_reference_line_info_ = nullptr;

  ThreadSafeIndexedObstacles obstacles_;

  std::unordered_map<std::string, perception::TrafficLight> traffic_lights_;

  perception::Color global_traffic_light_color_ =
      TL::perception::Color::UNKNOWN;

  // current frame published trajectory
  std::shared_ptr<ADCTrajectory> current_frame_planned_trajectory_;

  // current frame path for future possible speed fallback
  std::shared_ptr<DiscretizedPath> current_frame_planned_path_;

  const ReferenceLineProvider* reference_line_provider_ = nullptr;

  std::array<common::math::Vec2d, 4> parking_lot_vertices_;
  OpenSpaceInfo open_space_info_;

  std::vector<routing::LaneWaypoint> future_route_waypoints_;

  functionmanager::MachineStateType cur_state_machine_ =
      functionmanager::MachineStateType::INITIAL_TYPE;
  bool is_state_change_ = false;

  soc::Chassis::GearPosition target_gear_ = soc::Chassis::GEAR_DRIVE;

  bool is_lateral_replan_ = false;

  bool is_longitudinal_replan_ = false;

  bool is_speed_fallback_ = false;
  bool is_cipv_speed_fallback_ = false;
  bool is_merge_stop_fallback_ = false;
  bool is_stop_fallback_ = false;

  bool is_dec_by_ramp_ = false;

  bool is_path_vaild_ = true;

  bool is_speed_plan_standstill_ = false;

  ObstaclesInfo obstacles_info_;

  std::unordered_map<int, GameObstacleIntention> game_obs_intention_ = {};

  std::string lon_stop_obs_id_;

  RepeatedPtrField<TrajectoryPoint>* stitching_trajectory_points_ptr_ = nullptr;
  ForceRplanType force_replan_type_ = NO_FORCE_REPLAN;

  RepeatedPtrField<prediction::Trajectory> ego_prediction_trajectories_;
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 public:
  FrameHistory();
};

}  // namespace planning
}  // namespace TL
