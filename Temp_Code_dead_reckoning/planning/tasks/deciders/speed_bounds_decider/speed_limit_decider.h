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
 *   @file
 **/

#pragma once

#include <deque>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include "common/math/line_segment2d.h"
#include "common/status/status.h"
#include "planning/common/dependency_injector.h"
#include "planning/common/frame.h"
#include "planning/common/obstacle.h"
#include "planning/common/open_space_info.h"
#include "planning/common/path/path_data.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/speed_limit.h"
#include "planning/localview/local_view.h"

#include "planning/proto/task_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

using TL::common::TrajectoryPoint;

/**
   * @brief PedestrianTrajectoryPoint
   */
struct PedestrianPoint {
  double mean_x = 0.0;
  double mean_y = 0.0;
  double sigma_x = 0.001;
  double sigma_y = 0.001;
  double product_sigma_xy = 0.001;
  double double_correlation = 0.0;
  double temp2 = 0.001;
};

/**
 * @brief SpeedLimitSegment 
 * 
 */
struct SpeedLimitSegment {
  double start_s = 0.0;
  double end_s = 0.0;
  double ave_l = 0;
  double speed_limit = FLAGS_speed_upper_bound;
};

class SpeedLimitDecider {
 public:
  /**
  * @brief Construct a new Speed Limit Decider object
  * 
  * @param config 
  * @param reference_line 
  * @param path_data 
  */
  SpeedLimitDecider() = default;
  common::Status Init(const SpeedBoundsDeciderConfig& config,
                      const ReferenceLine& reference_line,
                      const PathData& path_data);
  /**
   * @brief Destroy the Speed Limit Decider object
   * 
   */
  virtual ~SpeedLimitDecider() = default;
  /**
   * @brief Get the Speed Limits object
   * 
   * @param obstacles 
   * @param reference_line_info 
   * @param init_point 
   * @param local_view 
   * @param centric_accel_calibration_table
   * @param frame
   * @param speed_limit_data 
   * @return common::Status 
   */
  virtual common::Status GetSpeedLimits(
      const IndexedList<std::string, Obstacle>& obstacles,
      const ReferenceLineInfo& reference_line_info,
      const TrajectoryPoint& init_point, const LocalView& local_view,
      const std::pair<std::vector<double>, std::vector<double>>&
          centric_accel_calibration_table,
      Frame* frame, SpeedLimit* speed_limit_data);

 private:
  // FRIEND_TEST(SpeedLimitDeciderTest, get_centric_acc_limit);
  /**
   * @brief Get the Centric Acc Limit object
   * 
   * @param kappa 
   * @return double 
   */
  [[nodiscard]] double GetCentricAccLimit(double kappa) const;
  /**
   * @brief Get the Avg Kappa object
   * 
   * @param path_points 
   * @param kappa 
   */
  void GetAvgKappa(const std::vector<common::PathPoint>& path_points,
                   std::vector<double>* kappa) const;
  /**
    * @brief Get the Nudge Speed Limit object
    * 
    * @param frenet_frame_point 
    * @param obstacles 
    * @param map_speed_limit 
    * @return double 
    */
  [[nodiscard]] double GetNudgeSpeedLimit(
      const common::FrenetFramePoint& frenet_frame_point,
      const IndexedList<std::string, Obstacle>& obstacles,
      double map_speed_limit) const;

  /**
   * @brief get limit speed by freespace key points
   *
   * @param path_point
   * @param freespace_outs
   * @return double limit speed
   */
  [[nodiscard]] double GetFreeSpaceSpeedLimit(
      const common::PathPoint& path_point,
      const std::unordered_map<uint32_t, perception::PerceptionObstacle_Type>&
          box_id_type,
      const std::vector<FreeSpaceSegment>& freespace_segments, bool is_forward,
      double cruise_speed);
  /**
   * @brief get limit speed by steer angle diff
   *
   * @param kappa
   * @param real_steer_angle
   * @param cruise_speed
   * @return double limit speed
   */
  [[nodiscard]] double GetAvpSteerDiffSpeedLimit(double kappa,
                                                 double real_steer_angle,
                                                 double cruise_speed);
  /**
    * @brief Get the Pedestrian Speed Limit object
    * 
    * @param path_point 
    * @param obstacles 
    * @param point_s_interval
    * @param is_avp_mode TRUE in avp mode   
    * @return double 
    */
  [[nodiscard]] double GetPedestrianSpeedLimit(
      const common::PathPoint& path_point, double point_s_interval,
      const std::vector<PedestrianPoint>& pedestrians_point,
      bool is_avp_mode) const;
  /**
    * @brief 
    * 
    * @param path_point 
    * @param next_path_point 
    * @return * double 
    */
  [[nodiscard]] double GetDKappaSpeedLimit(
      const common::PathPoint& path_point,
      const common::PathPoint& next_path_point) const;

  /**
   * @brief load curvature max lateral acc avp reverse
   * 
   * @return true 
   * @return false 
   */
  bool LoadCentricAccelCalibrationTableReverse();

  void GetConeSpeedLimitSegments(const ReferenceLineInfo& reference_line_info);

  double GetConeSpeedLimit(const common::PathPoint& path_point) const;

  /**
   * @brief load curvature max lateral acc avp forward
   * 
   * @return true 
   * @return false 
   */
  bool LoadCentricAccelCalibrationTableForward();
  static double EgoMovingStep(const LocalView& local_view);
  void SpeedLimitByModeChange(double path_s, double delta_s, bool is_acc_mode,
                              functionmanager::PerceptionSubState current_state,
                              SpeedLimitInfo* speed_limit_info);
  SpeedBoundsDeciderConfig speed_bounds_config_;
  ReferenceLine reference_line_;
  PathData path_data_;
  TL::common::VehicleParam vehicle_param_;
  std::vector<std::vector<double>> state_all_;
  std::pair<std::vector<double>, std::vector<double>>
      centric_accel_calibration_table_avp_reverse_;
  std::pair<std::vector<double>, std::vector<double>>
      centric_accel_calibration_table_avp_forward_;
  DiscretizedPath discretized_path_last_;
  SpeedLimit curvature_speed_limit_data_last_;
  functionmanager::PerceptionSubState last_state_ =
      functionmanager::SUB_INITIAL_TYPE;
  double active_time_ = 0.0;
  double current_centric_accel_ = 5.0;
  bool is_acc_mode_last_ = false;
  bool discretized_path_last_valid_ = false;
  int mode_change_count_ = 0;
  bool mode_change_valid_ = false;
  ::TL::soc::Chassis_DrivingMode last_function_status_ =
      soc::Chassis::AUTO_SPEED_ONLY;

  SpeedLimitSegment left_cone_speed_limit_segment_;
  SpeedLimitSegment right_cone_speed_limit_segment_;
  bool left_cone_speed_limit_mode_ = false;
  bool right_cone_speed_limit_mode_ = false;
  std::vector<double> cone_l_conf_;
  std::vector<double> cone_v_conf_;
  static constexpr double kEffectiveCone = 5.0;
};

}  // namespace planning
}  // namespace TL
