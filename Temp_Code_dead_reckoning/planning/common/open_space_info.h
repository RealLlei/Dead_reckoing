#ifndef PLANNING_COMMON_OPEN_SPACE_INFO_H
#define PLANNING_COMMON_OPEN_SPACE_INFO_H
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
 * @file
 **/
#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/math/box2d.h"
#include "common/math/double_type.h"
#include "common/math/line_segment2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "map/hdmap/hdmap_util.h"
#include "map/hdmap/path.h"
#include "planning/common/indexed_queue.h"
#include "planning/common/obstacle.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/trajectory/discretized_trajectory.h"
#include "planning/common/trajectory/publishable_trajectory.h"
#include "planning/open_space/coarse_path_generator/node3d.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/map/map_id.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/planning/planning_internal.pb.h"
#include "proto/planning/planning_status.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {
using typename TL::planning_internal::AvpSpeedPlanCollisionInfo;
// using typename TL::planning_internal::OpenSpaceDebug;

using TrajGearPair =
    std::pair<DiscretizedTrajectory, soc::Chassis::GearPosition>;
using PathGearPair = std::pair<DiscretizedPath, soc::Chassis::GearPosition>;

// destregion: xy polygon, from_ang, to_ang
using DestRegionWithAng = std::tuple<common::math::Polygon2d, double, double>;

enum ParkingScenarioType {
  DEFAULT_TYPE = 0,
  LEFT_VERTICAL_PARKING_IN = 1,
  RIGHT_VERTICAL_PARKING_IN = 2,
  LEFT_LATERAL_PARKING_IN = 3,
  RIGHT_LATERAL_PARKING_IN = 4,
  LEFT_OBLIQUE_PARKING_IN = 5,
  RIGHT_OBLIQUE_PARKING_IN = 6,
  LEFT_VERTICAL_PARKING_OUT = 7,
  RIGHT_VERTICAL_PARKING_OUT = 8,
  FORWARD_VERTICAL_PARKING_OUT = 9,
  BACKWARD_VERTICAL_PARKING_OUT = 10,
  LEFT_LATERAL_PARKING_OUT = 11,
  RIGHT_LATERAL_PARKING_OUT = 12,
  LEFT_OBLIQUE_PARKING_OUT = 13,
  RIGHT_OBLIQUE_PARKING_OUT = 14,
  FORWARD_OBLIQUE_PARKING_OUT = 15,
  BACKWARD_OBLIQUE_PARKING_OUT = 16,
  REVERSE_TURN = 20,
  LEFT_U_TYPE_TURN = 21,
  RIGHT_U_TYPE_TURN = 22,
  FREESPACE_FORWARD_EXPLORATION = 23,
  CONTROL_CALIBRATION_MODE = 100
};

struct PartitionedPath {
  size_t path_idx = 0;
  size_t point_idx = 0;
  bool path_shift = false;
  std::vector<PathGearPair> path_set;
  planning_internal::PathUpdateStatus::PathType path_type =
      planning_internal::PathUpdateStatus::DEFAULT;
  uint32_t replan_status = 0;
};

enum SpaceStructure {
  DEFAULT = 0,
  VER_PARK_LOT = 1,
  LAT_PARK_LOT = 2,
};

enum UseGeometry {
  // only use hybrid a star
  NOT_USE = 0,
  // only use precise pose
  ONLY_USE = 1,
  // first use precise pose, hybrid a star if necessary
  USE_FIRST = 2,
  // use precise angle if hybrid a star fails
  USE_LAST = 3,
  // first use precise pose, then both hybrid a star and  precise angle if precise pose fails
  USE_FIRST_LAST = 4,
  // first use precise pose, then  precise angle if necessary
  USE_BOTH = 5,
};

enum GeometryConnectionType {
  DEFAULT_GEOMETRY_TYPE = 0,
  CYCLE_STRAIGHT = 1,
  STRAIGHT_CYCLE = 2,
  BOTH_TYPE = 3,
};

enum UseGeometryPurpose {
  DEFAULTPURPOSE = 0,
  PRECISEPOSE = 1,
  PRECISEANGLE = 2,
};

enum ParkDirection {
  NODIRECTION = 0,
  PARKIN = 1,
  LEFTPARKOUT = 2,
  RIGHTPARKOUT = 3,
  FORWARDPARKOUT = 4,
};

enum ScenarioDiffcultyType {
  NORMAL_SCENARIO = 0,
  DEADEND_SCENARIO = 1,
  NARROW_SPOT_SCENARIO = 2,
  NARROW_PASSAGE_SCENARIO = 4,
};

struct GeometryStrategy {
  UseGeometry use_geometry = UseGeometry::NOT_USE;
  std::vector<std::pair<double, double>> longitudal_bound;
  std::vector<UseGeometryPurpose> use_purpose;
  std::vector<GeometryConnectionType> geometry_path_type;
  bool consider_kappa_diff = false;
};

struct CollisionFreeSearchStrategy {
  bool replan_due_to_collision = false;
  double collision_free_dist = 0.0;
  common::PathPoint collision_path_point;
};

struct TraceAdjustSearchStrategy {
  bool is_trace_adjust = false;
  DiscretizedPath trace_path;
  std::vector<double> xy_bounds;
  double target_s = 0.0;
  double finish_l_threshold = 0.0;
  double finish_theta_threshold = 0.0;

  std::string DebugString() const {
    std::string ans;
    absl::StrAppend(&ans, " \n trace_adjust_strategy is_trace_adjust = ",
                    is_trace_adjust ? "True" : "False");
    absl::StrAppend(&ans, " \n trace_adjust_strategy xy_bounds = ");
    for (const auto& xy_bound : xy_bounds) {
      absl::StrAppend(&ans, xy_bound, " ");
    }
    absl::StrAppend(&ans, " \n trace_adjust_strategy trace_path Length ",
                    trace_path.Length());
    absl::StrAppend(&ans, " \n trace_adjust_strategy target_s = ", target_s);
    absl::StrAppend(&ans, " \n trace_adjust_strategy finish_l_threshold = ",
                    finish_l_threshold);
    absl::StrAppend(&ans, " \n trace_adjust_strategy finish_theta_threshold = ",
                    finish_theta_threshold);
    return ans;
  }
};

struct PathSearchStrategy {
  // add soft cost if plan from start; hard constrain if plan from end
  // 0 no limit; 1: forward; -1: backward
  int init_path_direction = 0;
  // limit start point steer
  bool limit_init_steer_margin = false;
  // search direction
  bool is_plan_from_start = true;
  // 0 no limit; 1: cut off same direction; -1: cut off opposite direction
  int cut_off_strategy = 0;
  // 0 default structure; 1: lat parking lot
  SpaceStructure space_structure = SpaceStructure::DEFAULT;
  ParkDirection park_direction = ParkDirection::NODIRECTION;
  bool use_larger_curvature = false;
  bool enable_init_kappa_cost = false;
  GeometryStrategy use_geometry_strategy;
  CollisionFreeSearchStrategy collision_free_search_strategy;
  bool is_dead_end_scenario = false;
  bool is_narrow_passage_scenario = false;
  TraceAdjustSearchStrategy trace_adjust_search_strategy;
  // temp: record nns scenario flag for hybrid search
  bool is_nns_adjust_senario = false;
  // temp solution, set a reference line in this structure
  ReferenceLine reference_line;

  std::string DebugString() const {
    std::string ans = absl::StrCat(
        "PathSearchStrategy: \n init_path_direction = ", init_path_direction,
        " \n limit_init_steer_margin = ",
        limit_init_steer_margin ? "True" : "False",
        " \n is_plan_from_start = ", is_plan_from_start ? "True" : "False",
        " \n cut_off_strategy = ", cut_off_strategy,
        " \n space_structure = ", space_structure,
        " \n use_larger_curvature = ", use_larger_curvature ? "True" : "False",
        " \n enable_init_kappa_cost = ",
        enable_init_kappa_cost ? "True" : "False",
        " \n use_geometry_strategy use_geometry = ",
        use_geometry_strategy.use_geometry,
        " \n collision_free_search_strategy replan_due_to_collision = ",
        collision_free_search_strategy.replan_due_to_collision ? "True"
                                                               : "False",
        " \n collision_free_search_strategy collision_free_dist = ",
        collision_free_search_strategy.collision_free_dist);
    for (const auto& long_bound : use_geometry_strategy.longitudal_bound) {
      absl::StrAppend(&ans,
                      " \n use_geometry_strategy longitudal_bound first = ",
                      long_bound.first,
                      " \n use_geometry_strategy longitudal_bound second = ",
                      long_bound.second);
    }
    for (const auto& geometry_purpose : use_geometry_strategy.use_purpose) {
      absl::StrAppend(&ans, " \n use geometry purpose: ", geometry_purpose);
    }
    for (const auto& geometry_path_type :
         use_geometry_strategy.geometry_path_type) {
      absl::StrAppend(&ans, " \n use_geometry_strategy_geometry_path_type: ",
                      geometry_path_type);
    }
    absl::StrAppend(&ans, " \n  is_dead_end_scenario: ",
                    static_cast<int>(is_dead_end_scenario));
    absl::StrAppend(&ans, " \n  is_narrow_passage_scenario: ",
                    static_cast<int>(is_narrow_passage_scenario));
    absl::StrAppend(&ans, trace_adjust_search_strategy.DebugString());
    return ans;
  }

  void Reset() {
    init_path_direction = 0;
    limit_init_steer_margin = false;
    is_plan_from_start = true;
    cut_off_strategy = 0;
    space_structure = SpaceStructure::DEFAULT;
    use_larger_curvature = false;
    use_geometry_strategy.use_geometry = UseGeometry::NOT_USE;
    use_geometry_strategy.longitudal_bound.clear();
    use_geometry_strategy.geometry_path_type.clear();
    use_geometry_strategy.use_purpose.clear();
    collision_free_search_strategy.replan_due_to_collision = false;
    collision_free_search_strategy.collision_free_dist = 0.0;
    trace_adjust_search_strategy.is_trace_adjust = false;
    is_nns_adjust_senario = false;
    reference_line = ReferenceLine();
  }
} __attribute__((aligned(128)));

struct PathStrategy {
  // coarse path strategy
  PathSearchStrategy path_search_strategy;
  // smooth path constrain
  // 1 : forward; 0: stop; -1: backward
  int init_moving_direction{0};
  bool disable_search{false};
};

struct OpenSpaceEnvStructuredInfo {
  bool is_out_roi = false;
  bool is_parking_inwards = false;
  bool is_in_nns_adjust_scenario = false;
  uint32_t parking_scenario_diffculty_type =
      ScenarioDiffcultyType::NORMAL_SCENARIO;
  ParkingScenarioType parking_scenario_type = ParkingScenarioType::DEFAULT_TYPE;
} __attribute__((aligned(8)));

// NOLINTBEGIN
struct OpenSpacePathInfo {
  PathStrategy path_strategy;
  // coordinate info
  double rotate_angle{0.0};
  // use env info for park search strategy
  OpenSpaceEnvStructuredInfo open_space_env_structured_info;
  common::math::Vec2d origin;
  // obstacle info
  std::vector<std::pair<common::math::LineSegment2d, double>>
      obstacles_segments_vec;
  // Low fs obs_segement for approcahing & Pre-check
  std::vector<std::pair<common::math::LineSegment2d, double>>
      low_fs_obstacles_segments_vec;
  // roi_xy_boundary; x_min, x_max, y_min, y_max
  std::vector<double> roi_xy_boundary;
  TL::common::Status status = TL::common::Status::OK();
  // default warm start path
  DiscretizedPath trace_path;
  // reference line
  ReferenceLine reference_line;
  // start point
  common::PathPoint start_point;
  // end point
  common::PathPoint end_point;
  // dest region： polygon and angle range
  DestRegionWithAng dest_region_with_angle;
} __attribute__((aligned(128)));

// NOLINTEND
// key is pathid (spot id if park in scene); value is path input info
using OpenSpacePathInfoMap = std::unordered_map<int, OpenSpacePathInfo>;

struct OpenSpacePathInput {
  int path_id{-1};
  uint32_t replan_status = 0;
  double rotate_angle{0.0};
  TL::common::math::Vec2d translate_origin;
  common::PathPoint start_point;
  common::PathPoint end_pose;
  std::vector<double> xy_bounds;
  std::vector<std::pair<common::math::LineSegment2d, double>>
      obstacles_segments_vec;
  DiscretizedPath warm_start_path;
  DestRegionWithAng dest_region_with_angle;
  PathStrategy path_strategy;
} __attribute__((aligned(128)));

struct OpenSpacePathOutput {
  uint32_t replan_status = 0;
  bool need_collision_free_smooth{true};
  bool has_smoothed{false};
  std::string error_msg;
  std::vector<PathGearPair> partitioned_path;  // input and output
  planning_internal::PathUpdateStatus::PathType path_type;

  void Reset() {
    need_collision_free_smooth = true;
    has_smoothed = false;
    error_msg.clear();
    partitioned_path.clear();
    path_type = planning_internal::PathUpdateStatus::DEFAULT;
  }
} __attribute__((aligned(64)));

struct FreeSpaceSegment {
  int index_in_freespace_out = -1;
  int index_in_keypoints = -1;
  bool isLinkObjFusion = false;
  uint32_t obstacleId = 0;
  perception::FreeSpaceOut::ClassType cls_type =
      perception::FreeSpaceOut::UNKOWN_CLASS;
  perception::FreeSpaceOut::SensorType sensor_type =
      perception::FreeSpaceOut::CAMERA;
  perception::FreeSpaceOut::HeightType height_type =
      perception::FreeSpaceOut::UNKOWN_HEIGHT;
  common::math::LineSegment2d segment;
};

class OpenSpaceInfo {
 public:
  OpenSpaceInfo()
      : open_space_path_info_id_(-1),
        publishable_trajectory_data_(
            std::make_pair(PublishableTrajectory(common::Clock::NowInSeconds(),
                                                 DiscretizedTrajectory()),
                           soc::Chassis::GEAR_NONE)) {
    open_space_path_info_map_[open_space_path_info_id_] = OpenSpacePathInfo();
  }

  ~OpenSpaceInfo() = default;

  int open_space_path_info_id() const { return open_space_path_info_id_; }

  void set_open_space_path_info_id(int id) {
    open_space_path_info_id_ = id;
    if (open_space_path_info_map_.find(id) == open_space_path_info_map_.end()) {
      open_space_path_info_map_[id] = OpenSpacePathInfo();
    }
  }

  double open_space_pre_stop_fence_s() const {
    return open_space_pre_stop_fence_s_;
  }

  void set_open_space_pre_stop_fence_s(const double s) {
    open_space_pre_stop_fence_s_ = s;
  }

  /**
   * @brief Set the open space wheel mask box in enu coordinate
   *
   * @param wheel_mask_ptr wheel mask ptr
   */
  void set_open_space_wheel_mask_box(
      const common::math::Box2d& wheel_mask_box) {
    open_space_wheel_mask_box_ = wheel_mask_box;
  }

  /**
   * @brief  get  wheel mask enu ptr
   *
   * @return open_space_wheel_mask_enu_
   */
  const common::math::Box2d& open_space_wheel_mask_box() const {
    return open_space_wheel_mask_box_;
  }

  /**
   * @brief Set  consider wheel mask flag
   *
   * @param is_consider_wheel_mask
   */
  void set_is_consider_wheel_mask(const bool is_consider_wheel_mask) {
    is_consider_wheel_mask_ = is_consider_wheel_mask;
  }

  /**
   * @brief clear speed bump segments
   *
   * @param 
   */
  void clear_speed_bump_segments() { speed_bump_segments_.clear(); }

  /**
   * @brief Set the speed bump segments in enu coordinate
   *
   * @param speed_bump_ptr speed bump ptr
   */
  void add_speed_bump_segments(
      const common::math::LineSegment2d& speed_bump_segment) {
    speed_bump_segments_.emplace_back(speed_bump_segment);
  }

  /**
   * @brief  get speed bumps enu ptr
   *
   * @return speed_bumps_enu_
   */
  const std::vector<common::math::LineSegment2d>& speed_bump_segments() const {
    return speed_bump_segments_;
  }

  /**
   * @brief  return consider wheel mask flag
   *
   * @return true
   * @return false
   */
  bool is_consider_wheel_mask() const { return is_consider_wheel_mask_; }

  bool is_on_open_space_trajectory() const {
    return is_on_open_space_trajectory_;
  }

  void set_is_on_open_space_trajectory(const bool flag) {
    is_on_open_space_trajectory_ = flag;
  }

  const PartitionedPath& path_result() const { return path_result_; }

  PartitionedPath* mutable_path_result() { return &path_result_; }

  bool destination_reached() const { return destination_reached_; }

  void set_destination_reached(const bool flag) { destination_reached_ = flag; }

  const PartitionedPath& partitioned_paths() const {
    return partitioned_paths_;
  }

  PartitionedPath* mutable_partitioned_paths() { return &partitioned_paths_; }

  bool is_partitioned_paths_valid() const {
    const auto& path_set = partitioned_paths_.path_set;
    const auto& path_type = partitioned_paths_.path_type;
    if (path_type == planning_internal::PathUpdateStatus::TRACE_PATH ||
        path_type == planning_internal::PathUpdateStatus::CRUISE_PATH ||
        path_set.empty()) {
      return false;
    }
    // not stop path
    static const double kEps = 1e-3;
    return path_set.front().first.back().s() > kEps;
  }

  const PathGearPair& chosen_partitioned_path() const {
    // TODO(Runxin): export to chart
    return chosen_partitioned_path_;
  }

  PathGearPair* mutable_chosen_partitioned_path() {
    return &chosen_partitioned_path_;
  }

  std::pair<int, int>* mutable_chosen_partitioned_path_idx() {
    return &chosen_partitioned_path_idx_;
  }

  const std::pair<int, int>& chosen_partitioned_path_idx() const {
    return chosen_partitioned_path_idx_;
  }

  /**
   * @brief get mutable speed optimizer trajectory
   *
   * @return TrajGearPair*
   */
  TrajGearPair* mutable_speed_optimizer_trajectory() {
    return &speed_optimizer_trajectory_;
  }

  /**
   * @brief get const speed optimizer trajectory
   *
   * @return const TrajGearPair&
   */
  const TrajGearPair& speed_optimizer_trajectory() const {
    return speed_optimizer_trajectory_;
  }

  /**
   * @brief Set the speed optimizer trajectory object
   *
   * @param traj_gear_pair
   */
  void set_speed_optimizer_trajectory(const TrajGearPair& traj_gear_pair) {
    speed_optimizer_trajectory_ = traj_gear_pair;
  }

  std::pair<PublishableTrajectory, soc::Chassis::GearPosition>*
  mutable_publishable_trajectory_data() {
    return &publishable_trajectory_data_;
  }

  const std::pair<PublishableTrajectory, soc::Chassis::GearPosition>&
  publishable_trajectory_data() const {
    return publishable_trajectory_data_;
  }

  // TODO(QiL, Jinyun) refactor and merge this with debug
  common::TrajectoryPoint* mutable_future_collision_point() {
    return &future_collision_point_;
  }

  const common::TrajectoryPoint& future_collision_point() const {
    return future_collision_point_;
  }

  // TODO(QiL, Jinyun): refactor open_space_info vs debug

  TL::planning_internal::Debug* mutable_debug() { return debug_; }

  void set_debug(TL::planning_internal::Debug* debug) { debug_ = debug; }

  const TL::planning_internal::Debug& debug() const { return *debug_; }

  LatencyStats* mutable_latency_stats() { return &time_latency_; }

  const LatencyStats& latency_stats() const { return time_latency_; }

  TL::planning_internal::Debug debug_instance() const {
    return debug_instance_;
  }

  TL::planning_internal::Debug* mutable_debug_instance() {
    return &debug_instance_;
  }

  void sync_debug_instance() {
    // Remove existing obstacle vectors to prevent repeating obstacle
    // vectors.
    if (!debug_->planning_data().open_space().obstacles().empty()) {
      debug_instance_.mutable_planning_data()
          ->mutable_open_space()
          ->clear_obstacles();
    }

    if (!debug_->planning_data()
             .open_space()
             .warm_start_path()
             .warm_start_path_points()
             .empty()) {
      debug_instance_.mutable_planning_data()
          ->mutable_open_space()
          ->clear_warm_start_path();
    }
    if (!debug_->planning_data().open_space().smoothed_path().empty()) {
      debug_instance_.mutable_planning_data()
          ->mutable_open_space()
          ->clear_smoothed_path();
    }
    if (!debug_->planning_data()
             .open_space()
             .partition_smoothed_path()
             .empty()) {
      debug_instance_.mutable_planning_data()
          ->mutable_open_space()
          ->clear_partition_smoothed_path();
    }
    if (!debug_->planning_data().open_space().multi_search_info().empty()) {
      debug_instance_.mutable_planning_data()
          ->mutable_open_space()
          ->clear_multi_search_info();
    }

    debug_instance_.MergeFrom(*debug_);
  }

  void RecordDebug(TL::planning_internal::Debug* ptr_debug);

  void set_is_stop_path(const bool is_stop_path) {
    is_stop_path_ = is_stop_path;
  }

  bool get_is_stop_path() const { return is_stop_path_; }

  planning_internal::VehicleFollowError* mutable_vehicle_follow_error() {
    return &vehicle_follow_error_;
  }

  const planning_internal::VehicleFollowError& get_vehicle_follow_error()
      const {
    return vehicle_follow_error_;
  }

  planning_internal::VehicleFollowError*
  mutable_vehicle_to_current_end_error() {
    return &vehicle_to_current_end_error_;
  }

  const planning_internal::VehicleFollowError&
  get_vehicle_to_current_end_error() const {
    return vehicle_to_current_end_error_;
  }

  /**
   * @brief get the pointer of replan reason
   *
   * @return std::string* replan_reason pointer
   */
  std::string* mutable_replan_reason() { return &replan_reason_; }

  /**
   * @brief Get the replan reason object

   * @return const std::string& replan reason
   */
  const std::string& get_replan_reason() const { return replan_reason_; }

  /**
   * @brief Generate stop acceleration based on vehicle state
   *
   * @param vehicle_state
   * @return double stop acceleration
   */
  static inline double StopAcc(const common::VehicleState& vehicle_state) {
    double brake_acceleration = 0;
    switch (vehicle_state.gear()) {
      case soc::Chassis::GEAR_DRIVE:
        brake_acceleration = (1 == common::math::double_type::Compare(
                                       vehicle_state.linear_velocity(), 0.0))
                                 ? -FLAGS_open_space_standstill_acceleration
                                 : 0.0;
        break;
      case soc::Chassis::GEAR_REVERSE:
        brake_acceleration = (-1 == common::math::double_type::Compare(
                                        vehicle_state.linear_velocity(), 0.0))
                                 ? FLAGS_open_space_standstill_acceleration
                                 : 0.0;
        break;
      case soc::Chassis::GEAR_NEUTRAL:
      case soc::Chassis::GEAR_PARKING:
      case soc::Chassis::GEAR_LOW:
      case soc::Chassis::GEAR_INVALID:
      case soc::Chassis::GEAR_NONE:
        break;
    }
    return brake_acceleration;
  }

  /**
   * @brief 
   * 
   * @param open_space_status_ptr 
   * @param update_replan_status 
   * @return true 
   * @return false 
   */
  static void UpdateReplanStatus(
      const TL::planning::OpenSpaceStatus::Replan& update_replan_status,
      TL::planning::OpenSpaceStatus* open_space_status_ptr);

  /**
   * @brief 
   * 
   * @param open_space_status_ptr 
   * @param update_replan_status 
   * @return true 
   * @return false 
   */
  static void ResetReplanStatus(
      TL::planning::OpenSpaceStatus* open_space_status_ptr) {
    if (nullptr != open_space_status_ptr) {
      open_space_status_ptr->set_replan(0);
    }
  }

  /**
   * @brief Set the replan triggered by speed plan object
   *
   * @param flag
   */
  void set_replan_triggered_by_speed_plan(const bool flag) {
    replan_triggered_by_speed_plan_ = flag;
  }

  /**
   * @brief get replan triggered by speed plan
   *
   * @return true
   * @return false
   */
  bool replan_triggered_by_speed_plan() const {
    return replan_triggered_by_speed_plan_;
  }

  void set_current_path_has_collision_risk(const bool flag) {
    current_path_has_collision_risk_ = flag;
  }

  bool current_path_has_collision_risk() const {
    return current_path_has_collision_risk_;
  }

  /**
 * @brief 
 * 
 * @param is_history_entered 
 * @return true 
 * @return false 
 */
  bool entered_lateral_parking_slot(bool is_history_entered) const {
    bool is_entered_special_domain = false;
    if (is_history_entered) {
      is_entered_special_domain =
          adc_bottom_dist_ <=
          FLAGS_park_in_lateral_leave_parking_domain_bottom_dist_threhold;
    } else {
      is_entered_special_domain =
          (adc_bottom_dist_ <=
           FLAGS_park_in_lateral_ignore_bottom_dist_threhold);
    }
    return is_entered_special_domain;
  }

  /**
  * @brief Set the adc bottom dist 
  * 
  * @param adc_bottom_dist 
  */
  void set_adc_bottom_dist(const double adc_bottom_dist) {
    adc_bottom_dist_ = adc_bottom_dist;
  }

  /**
  * @brief Get the adc bottom dist object
  * 
  * @return double 
  */
  double get_adc_bottom_dist() const { return adc_bottom_dist_; }

  /**
   * @brief
   *
   * @return double
   */
  double global_publish_timestamp() const { return global_publish_timestamp_; }

  /**
   * @brief Set the global publish timestamp object
   *
   * @param publish_timestamp
   */
  void set_global_publish_timestamp(double publish_timestamp) {
    global_publish_timestamp_ = publish_timestamp;
  }

  /**
   * @brief Set the is gear changed object
   *
   * @param flag
   */
  void set_is_gear_changed(bool flag) { is_gear_changed_ = flag; }

  /**
   * @brief is gear changed
   *
   * @return true
   * @return false
   */
  bool is_gear_changed() const { return is_gear_changed_; }

  /**
   * @brief get speed plan collision info
   *
   * @return const AvpSpeedPlanCollisionInfo&
   */
  const AvpSpeedPlanCollisionInfo& speed_plan_collision_info() const {
    return speed_plan_collision_info_;
  }

  /**
   * @brief get mutable speed plan collision info
   *
   * @return AvpSpeedPlanCollisionInfo*
   */
  AvpSpeedPlanCollisionInfo* mutable_speed_plan_collision_info() {
    return &speed_plan_collision_info_;
  }

  /**
   * @brief get is stop near wheel mask
   *
   * @return true
   * @return false
   */
  bool is_stop_near_wheel_mask() const {
    return speed_plan_collision_info_.is_stop_near_wheel_mask();
  }

  /**
   * @brief get speed task interactive stage
   *
   * @return SpeedTaskInteractiveStage
   */
  AvpSpeedPlanCollisionInfo::SpeedTaskInteractiveStage
  speed_task_interactive_stage() const {
    return speed_plan_collision_info_.speed_task_inter_stage();
  }

  std::vector<size_t>* mutable_under_spot_low_fs_idxs() {
    return &under_spot_low_fs_idxs_;
  }

  std::vector<size_t> under_spot_low_fs_idxs() const {
    return under_spot_low_fs_idxs_;
  }

  std::vector<size_t>* mutable_high_curb_fs_idxs() {
    return &high_curb_fs_idxs_;
  }

  std::vector<size_t> high_curb_fs_idxs() const { return high_curb_fs_idxs_; }

  std::vector<std::pair<size_t, std::vector<size_t>>>*
  mutable_ignore_fs_idxs() {
    return &ignore_fs_idxs_;
  }

  std::vector<std::pair<size_t, std::vector<size_t>>> ignore_fs_idxs() const {
    return ignore_fs_idxs_;
  }

  const std::vector<common::PathPoint>& spd_limit_points() const {
    return spd_limit_points_;
  }

  std::vector<common::PathPoint>* mutable_spd_limit_points() {
    return &spd_limit_points_;
  }

  const OpenSpacePathInfoMap& open_space_path_info_map() const {
    return open_space_path_info_map_;
  }

  OpenSpacePathInfoMap* mutable_open_space_path_info_map() {
    return &open_space_path_info_map_;
  }

  const OpenSpacePathInfo& open_space_path_info() const {
    return open_space_path_info_map_.at(open_space_path_info_id_);
  }

  OpenSpacePathInfo* mutbale_open_space_path_info() {
    return &(open_space_path_info_map_.at(open_space_path_info_id_));
  }

  void set_stuck_scenario_roi(const std::vector<double>& roi) {
    stuck_scenario_roi_ = roi;
  }

  const std::vector<double>& stuck_scenario_roi() const {
    return stuck_scenario_roi_;
  }

  void set_is_stuck_roi_updated(const bool updated) {
    is_stuck_roi_updated_ = updated;
  }

  bool is_stuck_roi_updated() const { return is_stuck_roi_updated_; }

 private:
  double open_space_pre_stop_fence_s_ = 0.0;
  TL::planning_internal::Debug* debug_{};
  double global_publish_timestamp_ = 0.0;
  double adc_bottom_dist_ = 0;
  std::vector<size_t> under_spot_low_fs_idxs_;
  std::vector<size_t> high_curb_fs_idxs_;
  std::vector<std::pair<size_t, std::vector<size_t>>> ignore_fs_idxs_;
  std::vector<common::PathPoint> spd_limit_points_;
  DiscretizedPath apa_trace_path_;
  int open_space_path_info_id_{-1};
  PathGearPair chosen_partitioned_path_;
  std::string replan_reason_;
  TrajGearPair speed_optimizer_trajectory_;
  PartitionedPath path_result_;
  PartitionedPath partitioned_paths_;
  std::pair<PublishableTrajectory, soc::Chassis::GearPosition>
      publishable_trajectory_data_;
  planning_internal::VehicleFollowError vehicle_follow_error_;
  planning_internal::VehicleFollowError vehicle_to_current_end_error_;
  TL::planning_internal::Debug debug_instance_;
  common::TrajectoryPoint future_collision_point_;
  LatencyStats time_latency_;
  AvpSpeedPlanCollisionInfo speed_plan_collision_info_;
  common::math::Box2d open_space_wheel_mask_box_;
  std::vector<common::math::LineSegment2d> speed_bump_segments_;
  std::pair<int, int> chosen_partitioned_path_idx_{-1, -1};
  std::vector<double> stuck_scenario_roi_;
  bool is_stop_path_ = false;
  bool is_on_open_space_trajectory_ = false;
  bool destination_reached_ = false;
  bool replan_triggered_by_speed_plan_ = false;
  bool current_path_has_collision_risk_ = false;
  bool is_consider_wheel_mask_ = false;
  bool is_gear_changed_ = false;
  bool is_stuck_roi_updated_ = false;
  OpenSpacePathInfoMap open_space_path_info_map_;
};

// LCOV_EXCL_START
// @brief class for record hybrid a star node expansion and finally searched
// path
class ExpansionInfo {
 public:
  explicit ExpansionInfo(const common::PathPoint& start_point);
  ~ExpansionInfo() = default;
  /**
 * @brief 
 * 
 * @param iter_num 
 * @param current_node_id 
 * @param next_node 
 */
  void load_extension_node_info(double iter_num,
                                const std::string& current_node_id,
                                const std::shared_ptr<Node3d>& next_node);
  /**
 * @brief 
 * 
 * @param iter_num 
 * @param current_node_id 
 * @param astar_start_time 
 * @param current_node 
 */
  void load_current_node_info(double iter_num,
                              const std::string& current_node_id,
                              double astar_start_time,
                              const std::shared_ptr<Node3d>& current_node);
  /**
 * @brief 
 * 
 * @param obstacles_segments_vec 
 */
  void load_extension_environment_info(
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec);

  /**
   * @brief record searched path
   *
   * @param result_x searched path x
   * @param result_y searched path y
   * @param result_phi searched path phi
   */
  void load_coarse_path_info(const std::vector<double>& result_x,
                             const std::vector<double>& result_y,
                             const std::vector<double>& result_phi);

 private:
  // file names for one shoot debug
  std::string current_node_filename_;
  std::string extension_filename_;
  std::string path_filename_;
  std::string environment_filename_;
};

// LCOV_EXCL_STOP

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_OPEN_SPACE_INFO_H
