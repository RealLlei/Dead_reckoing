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

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "common/math/math_utils.h"
#include "planning/common/frame.h"
#include "planning/common/obstacle.h"
#include "planning/tasks/task.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

class SpeedDecider : public Task {
 public:
  SpeedDecider(const TaskConfig& config,
               const std::shared_ptr<DependencyInjector>& injector);

  common::Status Execute(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

 private:
  enum STLocation {
    ABOVE = 1,
    BELOW = 2,
    CROSS = 3,
  };

  static STLocation GetSTLocation(const PathDecision* path_decision,
                                  const SpeedData& speed_profile,
                                  const STBoundary& st_boundary);

  static bool CheckKeepClearCrossable(const PathDecision* path_decision,
                                      const SpeedData& speed_profile,
                                      const STBoundary& keep_clear_st_boundary);

  static bool CheckKeepClearBlocked(const PathDecision* path_decision,
                                    const Obstacle& keep_clear_obstacle);

  /**
   * @brief check if the ADC should follow an obstacle by examing the
   *StBoundary of the obstacle.
   * @param boundary The boundary of the obstacle.
   * @return true if the ADC believe it should follow the obstacle, and
   *         false otherwise.
   **/
  static bool CheckIsFollow(const Obstacle& obstacle,
                            const STBoundary& boundary);

  bool CheckStopForPedestrian(const Obstacle& obstacle) const;

  bool CreateStopDecision(const Obstacle& obstacle,
                          ObjectDecisionType* stop_decision,
                          double stop_distance) const;

  /**
   * @brief create follow decision based on the boundary
   **/
  bool CreateFollowDecision(const Obstacle& obstacle,
                            ObjectDecisionType* follow_decision) const;

  /**
   * @brief create yield decision based on the boundary
   **/
  bool CreateYieldDecision(const Obstacle& obstacle,
                           ObjectDecisionType* yield_decision) const;

  /**
   * @brief create overtake decision based on the boundary
   **/
  bool CreateOvertakeDecision(const Obstacle& obstacle,
                              ObjectDecisionType* overtake_decision) const;

  common::Status MakeObjectDecision(const SpeedData& speed_profile,
                                    PathDecision* path_decision) const;

  static void AppendIgnoreDecision(Obstacle* obstacle);

  /**
   * @brief "too close" is determined by whether ego vehicle will hit the front
   * obstacle if the obstacle drive at current speed and ego vehicle use some
   * reasonable deceleration
   **/
  bool IsFollowTooClose(const Obstacle& obstacle) const;

  /**
   * @brief Check is speed fallback
   * 
   * @param reference_line_info 
   * @return true 
   * @return false 
   */
  bool IsFallbackWithTTC(const ReferenceLineInfo* reference_line_info,
                         Frame* frame) const;

  /**
   * @brief Check is speed fallback
   * 
   * @param reference_line_info 
   * @return true 
   * @return false 
   */
  bool IsFallbackWithST(const ReferenceLineInfo* reference_line_info) const;

  /**
   * @brief Following distance limit table
   **/
  double GetFollowDistanceLimit(const double speed) const {
    const auto speed_index = common::math::Clamp(
        static_cast<int>(speed), 0,
        static_cast<int>(follow_distance_limit_table_.size() - 1));
    return follow_distance_limit_table_.at(speed_index);
  }

  /**
   * @brief Overtaking distance limit table
   **/
  double GetOvertakeDistanceLimit(const double speed) const {
    const auto speed_index = common::math::Clamp(
        static_cast<int>(speed), 0,
        static_cast<int>(overtake_distance_limit_table_.size() - 1));
    return overtake_distance_limit_table_.at(speed_index);
  }

  /**
   * @brief Setting the ultimate driving boundary
   **/
  void SetSDrivableBoundary(
      const SpeedData& speed_data,
      std::vector<std::pair<double, double>>* st_drivable) const;
  /**
   * @brief Obtaining the ultimate driving boundary
   **/
  void GetSDrivableBoundary(
      const SpeedData& speed_data,
      std::vector<std::pair<double, double>>* st_drivable) const;

  /**
   * @brief Warning for overtake too closely
   **/
  bool FallBackOvertake(
      const SpeedData& speed_data,
      const std::vector<STPoint>& fallback_overtake_points,
      double fallback_overtake_min,
      const std::vector<std::pair<double, double>>& st_drivable) const;
  /**
   * @brief Warning for horizontal and vertical 
   **/
  bool FallBackLatlon(
      const std::vector<ObsPointDescription>& trajectory_envelope,
      const FrenetFramePath& frenet_frame_path) const;
  /**
   * @brief Warning for following too closely
   **/
  bool FallBackFollow(
      const SpeedData& speed_data,
      const std::vector<STPoint>& fallback_follow_points,
      double fallback_follow_min, const prediction::Trajectory& trajectory,
      const std::vector<std::pair<double, double>>& st_drivable) const;
  /**
   * @brief Calculate if curvatureSpeed is dangerous
   **/
  bool IfCurvatureSpeedDangerous(const ReferenceLineInfo* reference_line_info,
                                 const Frame* frame) const;
  /**
   * @brief Calculate for cursteeringwheelangle
   **/
  double CalculateCurSteeringWheelAngle(const double kappa) const;
  /**
   * @brief Calculate for ttc
   **/
  bool CalculateTTC(const Obstacle& obstacle,
                    const ReferenceLineInfo* reference_line_info,
                    const Frame* frame) const;
  /**
   * @brief Calculate for staticObstacle
   **/
  bool CalculateTTCWithStaticObstacle(
      const Obstacle& obstacle,
      const ReferenceLineInfo* reference_line_info) const;
  /**
   * @brief Calculate for dynamicObstacle
   **/
  bool CalculateTTCWithDynamicObstacle(
      const Obstacle& obstacle, const ReferenceLineInfo* reference_line_info,
      const Frame* frame) const;
  /**
   * @brief Calculate extremedec is dangerous
   **/
  static bool CalculateExtremeDecIsDangerous(
      const Obstacle& obstacle, const ReferenceLineInfo* reference_line_info);

  SpeedDeciderConfig speed_decider_config_;
  SLBoundary adc_sl_boundary_;
  common::TrajectoryPoint init_point_;
  const ReferenceLine* reference_line_ = nullptr;
  bool is_forward_path_ = true;
  common::VehicleParam vehicle_param_;
  // 改动
  std::array<double, 10> follow_distance_limit_table_ = {};
  std::array<double, 20> overtake_distance_limit_table_ = {};
  static constexpr double kFollowDistanceMin = 2.5;
  static constexpr double kFollowDistanceIndex = 0.25;
  static constexpr double kOvertakeDistanceMax = 6.0;
  static constexpr double kOvertakeDistanceIndex = 0.1;
  static constexpr double kFollowspeedds = 0.5;
  static constexpr double kBreakDec = -3;
  static constexpr double kAcc = 1.0;
  static constexpr double kMaxv = 31.0;
  static constexpr double kConeDec = -2.0;
  static constexpr double kDrivableBoundarydt = 1.0;
  static constexpr double kTTCRange = 1.4;
  static constexpr size_t kDisSatisFactionPoint = 20;
  static constexpr double kExtremeDec = 3.8;
};

}  // namespace planning
}  // namespace TL
