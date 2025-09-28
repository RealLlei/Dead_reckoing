/******************************************************************************
 * Copyright 2017 The TL Authors. All Rights Reserved.
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
#include <vector>

#include "planning/localview/local_view.h"

/**
 * @namespace TL::planning
 * @brief TL::planning
 */
namespace TL {
namespace planning {

/**
 * @class SimControl
 * @brief A module that simulates a 'perfect control' algorithm, which assumes
 * an ideal world where the car can be perfectly placed wherever the planning
 * asks it to be, with the expected speed, acceleration, etc.
 */
class SimControl {
 public:
  /**
   * @brief Constructor of SimControl.
   * @param map_service the pointer of MapService.
   */
  SimControl();

  bool IsEnabled() const { return enabled_; }

  /**
   * @brief setup callbacks and timer
   * @param set_start_point initialize localization.
   */
  void Init();

  void InitStartPoint(double start_velocity = 0.0,
                      double start_acceleration = 0.0);

  /**
   * @brief Starts the timer to publish simulated localization and chassis
   * messages.
   */
  void Start();

  /**
   * @brief Stops the timer.
   */
  void Stop();

  /**
   * @brief Resets the internal state.
   */
  void Reset();

  void Process(const std::shared_ptr<LocalView>& local_view,
               std::shared_ptr<hdmap::HDMap> map_ptr);

  void OnPlanning(const std::shared_ptr<const ADCTrajectory>& adc_trajectory);
  void OnRoutingResponse(
      const std::shared_ptr<const routing::RoutingResponse>& routing);

 private:
  /**
   * @brief Predict the next adc_trajectory point using perfect control model
   */
  bool PerfectControlModel(
      const std::shared_ptr<const TL::planning::ADCTrajectory>&,
      TL::common::TrajectoryPoint* point,
      TL::soc::Chassis::GearPosition* gear_position);

  void PublishChassis(const TL::common::TrajectoryPoint& traj_point,
                      TL::soc::Chassis::GearPosition gear_position);

  void PublishLocalization(const TL::common::TrajectoryPoint& point);

  void PublishDummyPerception(
      const std::shared_ptr<const TL::planning::ADCTrajectory>&
          adc_trajectory_ptr,
      std::shared_ptr<hdmap::HDMap> map_ptr);

  /**
    * @brief the scenario for test static obstacle towing
    */
  static void AddStaticObsatclesBothAndSingleSide(
      const std::shared_ptr<TL::perception::PerceptionObstacles>&
          perception);

  /**
    * @brief the scenario for test static obstacle towing
    */
  static void AddStaticObsatclesSideBySide(
      const std::shared_ptr<TL::perception::PerceptionObstacles>&
          perception);

  /**
   * @brief init start point when use local hdmap
   * @param start_velocity start velocity
   * @param start_acceleration start acceleration
   */
  void InitStartPointForHDMap(double start_velocity, double start_acceleration);

  /**
   * @brief load waypoint, init start point when use ehp
   * @param start_velocity start velocity
   * @param start_acceleration start acceleration
   * @return true:init start point successed, false:load waypoint failed
   */
  bool InitStartPointForEHP(double start_velocity, double start_acceleration);

  // Reset the start point, which can be a dummy point on the map, a current
  // localization pose, or a start position received from the routing module.
  void SetStartPoint(const TL::common::TrajectoryPoint& point);

  void Freeze();

  void ClearPlanning();

  void InternalReset();

  // Time interval of the timer, in milliseconds.
  static constexpr double kSimControlIntervalMs = 10;
  static constexpr double kSimPredictionIntervalMs = 100;

  // The index of the previous and next point with regard to the
  // current_trajectory.
  int prev_point_index_ = 0;
  int next_point_index_ = 0;

  // Whether there's a planning received after the most recent routing.
  bool received_planning_ = false;

  // Whether planning has requested a re-routing.
  bool re_routing_triggered_ = false;

  // Whether the sim control is enabled.
  bool enabled_ = false;

  // Whether start point is initialized from actual localization data
  bool start_point_from_localization_ = false;

  // Whether to send dummy predictions
  bool send_dummy_perception_ = true;

  // The header of the routing planning is following.
  std::shared_ptr<const routing::RoutingResponse> current_routing_;

  TL::common::TrajectoryPoint prev_point_;
  TL::common::TrajectoryPoint next_point_;

  common::Pose adc_position_;

  double time_stamp_prev = 0;

  std::mutex mutex_;
  std::shared_ptr<LocalView> local_view_;
  std::shared_ptr<ADCTrajectory> adc_trajectory_;
  bool is_started_ = false;
  double current_lon_ = 0.0;
  int planning_zone_ = 0;
};

}  // namespace planning
}  // namespace TL
