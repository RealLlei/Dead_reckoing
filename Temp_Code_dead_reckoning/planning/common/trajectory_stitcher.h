#ifndef PLANNING_COMMON_TRAJECTORY_STITCHER_H
#define PLANNING_COMMON_TRAJECTORY_STITCHER_H

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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "planning/common/frame.h"
#include "planning/common/trajectory/publishable_trajectory.h"
#include "planning/reference_line/reference_line.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_model_config.pb.h"
#include "proto/common/vehicle_state.pb.h"

namespace TL {
namespace planning {

class TrajectoryStitcher {
 public:
  TrajectoryStitcher() = delete;
  /**
   * @brief
   *
   * @param vehicle_state
   * @param central_lane_marker
   * @param previous_central_lane_marker
   * @param prev_trajectory
   */
  static void TransformLastPublishedTrajectory(
      const common::VehicleState& vehicle_state,
      PublishableTrajectory* prev_trajectory);
  /**
   * @brief 轨迹拼接
   *
   * @param vehicle_state
   * @param current_timestamp
   * @param vehicle_state_time
   * @param preserved_points_num
   * @param replan_by_offset
   * @param fct_input
   * @param vehicle_model_config
   * @param fsm_stage_type
   * @param prev_trajectory
   * @param replan_reason first:true表示控制偏差大；second:replan reason
   * @return ::google::protobuf::RepeatedPtrField<common::TrajectoryPoint>
   */
  static ::google::protobuf::RepeatedPtrField<common::TrajectoryPoint>
  ComputeStitchingTrajectory(
      const common::VehicleState& vehicle_state,
      const std::shared_ptr<common::VehicleState>& prev_vehicle_state,
      double current_timestamp, double planning_cycle_time,
      size_t preserved_points_num, bool replan_by_offset, bool is_forward,
      const functionmanager::FunctionManagerIn& fct_input,
      const common::VehicleModelConfig& vehicle_model_config,
      const functionmanager::AvpFctOut::FsmStageType& fsm_stage_type,
      PublishableTrajectory* prev_trajectory,
      std::pair<int, std::string>* replan_reason,
      const ForceRplanType& force_replan_type);
  /**
   * @brief replan轨迹
   *
   * @param vehicle_state_time
   * @param vehicle_state
   * @param vehicle_model_config
   * @return ::google::protobuf::RepeatedPtrField<common::TrajectoryPoint>
   */
  static ::google::protobuf::RepeatedPtrField<common::TrajectoryPoint>
  ComputeReinitStitchingTrajectory(
      double vehicle_state_time, const common::VehicleState& vehicle_state,
      const common::VehicleModelConfig& vehicle_model_config);
  /**
   * @brief
   *
   * @param vehicle_time
   * @param vehicle_state
   * @return common::TrajectoryPoint
   */
  static common::TrajectoryPoint ComputeTrajectoryPointFromVehicleState(
      double vehicle_state_time, const common::VehicleState& vehicle_state);
  /**
   * @brief
   *
   * @param x
   * @param y
   * @param matched_trajectory_point
   * @return std::pair<double, double>
   */
  static std::pair<double, double> ComputePositionProjection(
      bool is_forward, double x, double y,
      const common::TrajectoryPoint& matched_trajectory_point);
  /**
   * @brief use by open space partation, calculate stitch point index
   *
   * @param x  ego x
   * @param y  eog y
   * @param pre_trajectory prev trajectory
   * @param index
   * @return true  match success
   * @return false match failed
   */
  static bool CalculateStitchingIndex(
      double x, double y, const PublishableTrajectory& pre_trajectory,
      size_t* index);

  /**
 * @brief only use position stitch. used by openspace avp
 *
 * @param preserved_points_num
 * @param offset_time
 * @param current_timestamp
 * @param vehicle_state
 * @param vehicle_model_config
 * @param prev_trajectory
 * @return ::google::protobuf::RepeatedPtrField<common::TrajectoryPoint>
 */
  static ::google::protobuf::RepeatedPtrField<common::TrajectoryPoint>
  ComputeStitchingTrajectoryOnlyByPose(
      size_t preserved_points_num, double offset_time, double current_timestamp,
      bool is_forward, const common::VehicleState& vehicle_state,
      const common::VehicleModelConfig& vehicle_model_config,
      PublishableTrajectory* prev_trajectory);
  static void TrajectoryBus2Earth(const common::VehicleState& vehicle_state,
                                  PublishableTrajectory* prev_trajectory);
  static void TrajectoryEarth2Bus(const common::VehicleState& vehicle_state,
                                  PublishableTrajectory* prev_trajectory);
  /**
   * @brief 
   * 
   * @param curr_pos_from_last 
   * @param vehicle_state 
   * @param prev_trajectory 
   * @param vehicle_model_config 
   * @param offset_time 
   * @return ::google::protobuf::RepeatedPtrField<common::TrajectoryPoint> 
   */
  static ::google::protobuf::RepeatedPtrField<common::TrajectoryPoint>
  ComputeStitchingTrajectoryByTransfromLast(
      const TrajectoryPoint& base_point,
      const common::VehicleState& vehicle_state,
      PublishableTrajectory* prev_trajectory,
      const common::VehicleModelConfig& vehicle_model_config,
      double offset_time, double current_timestamp);
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_TRAJECTORY_STITCHER_H
