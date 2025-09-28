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

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "common/status/status.h"
#include "planning/common/open_space_info.h"
#include "planning/common/path/discretized_path.h"
#include "planning/tasks/optimizers/path_optimizer.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {
struct pair_comp_ {
  // pair : path_index, match_point_index, iou
  bool operator()(
      const std::pair<std::pair<size_t, size_t>, double>& left,
      const std::pair<std::pair<size_t, size_t>, double>& right) const {
    return left.second <= right.second;
  }
};

// pair : match_point_index, iou
struct comp_ {
  bool operator()(const std::pair<size_t, double>& left,
                  const std::pair<size_t, double>& right) {
    return left.second <= right.second;
  }
};

using PointsMatchPriorityQueue =
    std::priority_queue<std::pair<size_t, double>,
                        std::vector<std::pair<size_t, double>>, comp_>;

using PointsMatchWithPathPriorityQueue = std::priority_queue<
    std::pair<std::pair<size_t, size_t>, double>,
    std::vector<std::pair<std::pair<size_t, size_t>, double>>, pair_comp_>;

enum OpenSpacePathDecision {
  UNKOWN = 0,
  TASK_FINISH = 1,
  TRACK_ABNORMAL = 2,
  NO_VALID_PATH = 3,
  CHOOSE_NEW_PATH = 4,
  CHOOSE_HISTORY_PATH = 5,
  PREPARE_FINISH = 6,
};

enum OpenSpacePathExcutableStatus {
  // set for control module
  EXCUTABLE = 0,                      // default value
  TOO_SHORT_TO_LAUNCH = 1,            // path is too short to launch
  TOO_SHORT_TO_BRAKE = 2,             // path is too short to brake
  LARGE_STEER_RATE = 4,               // steering angle change to fast
  LARGE_YAW_ERROR_IN_GEAR_SHIFT = 8,  // yaw error to large to track
  COLLISION_RISK = 16,                // collision risk
};

struct AdcStatus {
  double lat_error = INFINITY;
  double lon_error = INFINITY;
  double dist_error = INFINITY;
  double heading_error = INFINITY;
  double frozen_duration = INFINITY;
  bool is_stand_still = false;
  bool is_almost_stand_still = false;
  bool is_collision_near_target = false;
  bool is_reach_wheel_mask = false;
  bool is_block_by_car = false;
  bool is_block_by_curb = false;
  bool is_block_by_other_fs = false;
  bool is_heading_reach = false;
  bool is_distance_reach = false;
  bool is_lon_reach = false;
  bool is_lat_reach = false;
  bool is_over_time = false;
  bool is_blocked_over_time = false;
  bool is_lateral_park_in = false;
  bool is_uss_spot = false;
  bool is_execute_last_part_path = false;

  std::string DebugString() const {
    return absl::StrCat(
        "lat_error: ", std::to_string(lat_error),
        "\n lon_error: ", std::to_string(lon_error),
        "\n dist_error: ", std::to_string(dist_error),
        "\n heading_error: ", std::to_string(heading_error),
        "\n frozen_duration: ", std::to_string(frozen_duration),
        "\n is_stand_still: ", std::to_string(static_cast<int>(is_stand_still)),
        "\n is_almost_stand_still: ",
        std::to_string(static_cast<int>(is_almost_stand_still)),
        "\n is_collision_near_target: ",
        std::to_string(static_cast<int>(is_collision_near_target)),
        "\n is_reach_wheel_mask: ",
        std::to_string(static_cast<int>(is_reach_wheel_mask)),
        "\n is_block_by_car: ",
        std::to_string(static_cast<int>(is_block_by_car)),
        "\n is_block_by_curb: ",
        std::to_string(static_cast<int>(is_block_by_curb)),
        "\n is_block_by_other_fs: ",
        std::to_string(static_cast<int>(is_block_by_other_fs)),
        "\n is_heading_reach: ",
        std::to_string(static_cast<int>(is_heading_reach)),
        "\n is_distance_reach: ",
        std::to_string(static_cast<int>(is_distance_reach)),
        "\n is_lon_reach: ", std::to_string(static_cast<int>(is_lon_reach)),
        "\n is_lat_reach: ", std::to_string(static_cast<int>(is_lat_reach)),
        "\n is_lateral_park_in: ",
        std::to_string(static_cast<int>(is_lateral_park_in)),
        "\n is_uss_spot: ", std::to_string(static_cast<int>(is_uss_spot)),
        "\n is_execute_last_part_path: ",
        std::to_string(static_cast<int>(is_execute_last_part_path)));
  }
};

struct OpenSpacePathDeciderStatus {
  uint8_t gear_shift_num{0};
  uint32_t executable_status{0};
  double collision_distance = INFINITY;

  /**
  * @brief Reset all variables to their initial values.
  */
  void Reset() {
    gear_shift_num = 0;     // Reset gear shift number
    executable_status = 0;  // Reset executable status
    collision_distance = INFINITY;
  }

  std::string DebugString() const {
    return "gear_shift_num: " + std::to_string(gear_shift_num) +
           " executable_status: " + std::to_string(executable_status) +
           " collision_distance: " + std::to_string(collision_distance);
  }
};

using AlternativePath =
    std::vector<std::pair<PartitionedPath, OpenSpacePathDeciderStatus>>;

enum FinishType {
  ONRUN = 0,
  FAIL = 1,
  SUCCESS = 2,
  TBD = 3,
};

class OpenSpacePathPartition : public PathOptimizer {
 public:
  OpenSpacePathPartition(const TaskConfig& config,
                         const std::shared_ptr<DependencyInjector>& injector);

  ~OpenSpacePathPartition() override = default;
  common::Status Reset() override;

 private:
  common::Status Process() override;

  // meaningless method to mathch path optimizer task class
  TL::common::Status Process(const SpeedData& speed_data,
                                const ReferenceLine& reference_line,
                                const common::TrajectoryPoint& init_point,
                                const bool path_reusable,
                                PathData* const path_data) override {
    UNUSED(speed_data);
    UNUSED(reference_line);
    UNUSED(init_point);
    UNUSED(path_reusable);
    UNUSED(path_data);
    return TL::common::Status::OK();
  }

  /**
   * @brief 
   * 
   */
  void UpdateParam();

  /**
   * @brief 
   * 
   */
  void UpdateStatusBasedPartitionResult();

  /**
   * @brief 
   * 
   * @param openspace_path_decision_ptr 
   * @param choose_path_ptr 
   */
  void UpdatePathDecision(OpenSpacePathDecision* openspace_path_decision_ptr,
                          PartitionedPath* choose_path_ptr);

  /**
   * @brief 
   * 
   * @param partition_path_ptr 
   * @param executable_status_ptr 
   * @return true 
   * @return false 
   */
  bool PathMatch(PartitionedPath* partition_path_ptr,
                 uint32_t* executable_status_ptr);

  /**
   * @brief Get the Path Match Idx object
   * 
   * @param partition_path 
   * @param path_idx 
   * @param path_point_idx_ptr 
   * @return true 
   * @return false 
   */
  bool GetPathMatchIdx(const PartitionedPath& partition_path, size_t path_idx,
                       size_t* path_point_idx_ptr);

  /**
   * @brief 
   * 
   * @param path_point 
   * @param gear 
   * @return true 
   * @return false 
   */
  bool PointMatch(const common::PathPoint& path_point,
                  const soc::Chassis::GearPosition& gear);

  /**
   * @brief 
   * 
   * @param match_path_idx 
   * @param match_point_idx 
   * @param partition_path_ptr 
   * @param executable_status_ptr 
   */
  void UpdatePathMatchIdx(size_t match_path_idx, size_t match_point_idx,
                          PartitionedPath* partition_path_ptr,
                          uint32_t* executable_status_ptr);

  /**
   * @brief 
   * 
   * @param path 
   */
  void UpdateHistoryPath(const PartitionedPath& path) { history_path_ = path; }

  /**
   * @brief 
   * 
   * @param partition_path_set 
   * @return int 
   */
  int32_t PathDecider(const AlternativePath& alternative_path);

  /**
   * @brief 
   * 
   * @param replan_status 
   * @return true 
   * @return false 
   */
  bool IsUseSpeedWarnReplan(uint32_t replan_status);

  /**
   * @brief 
   * 
   * @param partitioned_paths 
   * @param current_path_index 
   * @param closest_path_point_index 
   * @param chosen_partitioned_path 
   */
  void AdjustRelativeS(const std::vector<PathGearPair>& partitioned_paths,
                       size_t current_path_index,
                       size_t closest_path_point_index,
                       PathGearPair* chosen_partitioned_path);

  /**
   * @brief Get the Complete Path Size object
   * 
   * @param partitioned_paths 
   * @return size_t 
   */
  static size_t GetCompletePathSize(
      const std::vector<PathGearPair>& partitioned_paths);

  /**
   * @brief Only Display current gear(partition) path percent and rest dis to HMI
   * 
   * @param path 
   */
  void UpdateParkDisplay(PartitionedPath* path);

  /**
   * @brief calculate pose error
   * 
   * @param cur_x 
   * @param cur_y 
   * @param cur_theta 
   * @param target_x 
   * @param target_y 
   * @param target_theta 
   * @param pose_error 
   */
  static void CalculatePoseError(
      double cur_x, double cur_y, double cur_theta, double target_x,
      double target_y, double target_theta,
      planning_internal::VehicleFollowError* pose_error);

  /**
   * @brief judge is pre_p , curr_p , next_p is same direction
   *
   * @param pre_p
   * @param curr_p
   * @param next_p
   * @param proj_p
   * @return true
   * @return false
   */
  static bool IsAddStartPointToPath(const common::PathPoint& pre_p,
                                    const common::PathPoint& curr_p,
                                    const common::PathPoint& next_p,
                                    common::PathPoint* proj_p);

  /**
   * @brief Get the Last Cyble Pub Path object
   * 
   * @param previous_frame 
   * @param partition_path_ptr 
   * @return true 
   * @return false 
   */
  static bool GetLastCyclePubPath(const Frame* previous_frame,
                                  PartitionedPath* partition_path_ptr);

  /**
   * @brief 
   * 
   * @param partitioned_paths_ptr 
   */
  static void ResetPartitionPath(PartitionedPath* partitioned_paths_ptr) {
    if (nullptr != partitioned_paths_ptr) {
      partitioned_paths_ptr->path_idx = 0;
      partitioned_paths_ptr->point_idx = 0;
      partitioned_paths_ptr->path_shift = false;
      partitioned_paths_ptr->path_set.clear();
      partitioned_paths_ptr->path_type =
          planning_internal::PathUpdateStatus::DEFAULT;
    }
  }

  /**
   * @brief Set the Stop Path object
   * 
   * @param pub_gear 
   * @param reserved_partitioned_path 
   * @param partitioned_paths_ptr 
   */
  void SetStopPath(const soc::Chassis::GearPosition& pub_gear,
                   const PartitionedPath& reserved_partitioned_paths,
                   PartitionedPath* partitioned_paths_ptr);

  /**
   * @brief 
   * 
   * @param chosen_path
   * @return true 
   * @return false 
   */
  bool IsTaskFinish(const PartitionedPath& chosen_path);

  /**
   * @brief 
   * 
   * @return true 
   * @return false 
   */
  bool IsTrackAbnormal();

  /**
   * @brief 
   * 
   * @param track_point 
   * @param gear 
   * @return true 
   * @return false 
   */
  bool IsYawTrackAbnormal(const common::PathPoint& track_point,
                          const soc::Chassis::GearPosition& gear);

  /**
   * @brief 
   * 
   * @param partition_path 
   * @param lon_dis_to_gear_shift_point 
   * @param next_path_idx_ptr 
   * @param next_path_point_idx_ptr 
   * @param executable_status_ptr 
   * @return true 
   * @return false 
   */
  bool AbleToGearShift(const PartitionedPath& partition_path,
                       double lon_dis_to_gear_shift_point,
                       size_t* next_path_idx_ptr,
                       size_t* next_path_point_idx_ptr,
                       uint32_t* executable_status_ptr);

  /**
   * @brief 
   * 
   * @param 
   * @return longitudinal_offset_threshold 
   */
  double SetGearShiftDis();

  /**
   * @brief 
   * 
   * @param path 
   * @param next_path_point_idx 
   * @param curr_p 
   * @param project_point 
   */
  static void ProjectOnPath(const DiscretizedPath& path,
                            size_t next_path_point_idx,
                            const common::PathPoint& curr_p,
                            common::PathPoint* project_point);

  /**
   * @brief 
   * 
   * @param vehicle_state
   * @param chosen_path
   * @param adc_status
   */
  void GetAdcStatus(const common::VehicleState& vehicle_state,
                    const PartitionedPath& chosen_path,
                    AdcStatus* adc_status_ptr);

  /**
   * @brief 
   * 
   * @param vehicle_state
   * @param end_pose_enu
   * @param adc_status
   */
  void GetAdcPosStatus(const common::VehicleState& vehicle_state,
                       const common::PathPoint& end_pose_enu,
                       AdcStatus* adc_status_ptr);

  /**
   * @brief 
   * 
   * @param vehicle_state
   * @param end_pose_enu
   * @param adc_status
   */
  void GetAdcHeadingStatus(const common::VehicleState& vehicle_state,
                           const common::PathPoint& end_pose_enu,
                           AdcStatus* adc_status_ptr);

  /**
   * @brief 
   * 
   * @param adc_status
   */
  void UpdateFinishStatusBasedOnStatus(const AdcStatus& adc_status);

  /**
   * @brief 
   * 
   * @param adc_status
   */
  void UpdateReplanInfoBasedOnStatus(const AdcStatus& adc_status);

  /**
   * @brief 
   * 
   * @return true 
   * @return false 
   */
  bool IsEndReplanTriggered();

  /**
   * @brief 
   * 
   * @param vehicle_state 
   * @param end_pose_enu 
   * @return true 
   * @return false 
   */
  bool IsCollisionNearTarget(const common::VehicleState& vehicle_state,
                             const common::PathPoint& end_pose_enu);

  /**
   * @brief 
   * 
   * @param end_pose_enu 
   * @param is_block_by_curb 
   * @param is_block_by_car 
   * @param is_block_by_other_fs
   */
  void IsBlockByCurbOrCar(const common::PathPoint& end_pose_enu,
                          bool* is_block_by_curb, bool* is_block_by_car,
                          bool* is_block_by_other_fs);

  /**
   * @brief 
   * 
   * @param end_pose_enu 
   * @param block_area_ptr 
   */
  void GetBlockArea(const common::PathPoint& end_pose_enu,
                    common::math::Polygon2d* block_area_ptr);

  /**
   * @brief 
   * 
   * @return true 
   * @return false 
   */
  bool IsReachWheelMask();

  /**
   * @brief 
   * 
   * @param vehicle_state 
   * @param dest_region_with_angle 
   * @return true 
   * @return false 
   */
  bool IsSatisfyParkOutFinishCondition(
      const common::VehicleState& vehicle_state,
      const DestRegionWithAng& dest_region_with_angle);

  /**
   * @brief 
   * 
   * @param vehicle_state 
   * @param dest_region_with_angle 
   * @return true 
   * @return false 
   */
  bool IsVehicleReachDestinationZone(
      const common::VehicleState& vehicle_state,
      const DestRegionWithAng& dest_region_with_angle);

  /**
   * @brief Check vehicle is on road or not
   * 
   * @return true 
   * @return false 
   */
  bool IsVehOnRoad();

  /**
   * @brief 
   * 
   * @param openspace_path_decision 
   * @param current_partitioned_path
   * @param path_point_index
   * @param chosen_partitioned_path 
   */
  void UpdateInfoForPreFinishCondition(
      const OpenSpacePathDecision& openspace_path_decision,
      PathGearPair* chosen_partitioned_path);

  /**
   * @brief 
   * 
   * @return true 
   * @return false 
   */
  static bool GetEndPointSLInCurrentPath(
      const std::vector<PathGearPair>& partitioned_path, size_t path_idx,
      common::SLPoint* end_point_sl_ptr);

  /**
   * @brief is adc executing last part of path
   * 
   * @param chosen_path 
   * @return true 
   * @return false 
   */
  bool IsAdcExecuteLastPartPath(const PartitionedPath& chosen_path);

  /**
   * @brief Update Path Excutable Status, only check new path currently
   * 
   * @param path 
   * @param executable_status_ptr 
   */
  void UpdatePathExcutableStatus(const PartitionedPath& path,
                                 uint32_t* executable_status_ptr);

  /**
   * @brief Update Collision Distance for path, currently only check match part path
   * 
   * @param path 
   * @param collision_distance_ptr 
   */
  void UpdateCollisionDistance(const PartitionedPath& path,
                               double* collision_distance_ptr);
  /**
   * @brief is path can be executable by adc
   * 
   * @param path 
   * @return true 
   * @return false 
   */
  bool IsPathTooShortToBrake(const PartitionedPath& path);

  /**
   * @brief 
   * 
   * @param path 
   * @return true 
   * @return false 
   */
  bool IsPathSteerRateLarge(const PartitionedPath& path);

  /**
   * @brief Get the Replan Status Based Excutable Status object
   * 
   * @param open_space_path_decider_status 
   * @param replan_status_ptr 
   */
  static void GetReplanStatusBasedExcutableStatus(
      const OpenSpacePathDeciderStatus& open_space_path_decider_status,
      uint32_t* replan_status_ptr) {
    if (nullptr == replan_status_ptr) {
      return;
    }
    const uint32_t executable_status =
        open_space_path_decider_status.executable_status;
    if ((executable_status & TOO_SHORT_TO_LAUNCH) != 0) {
      *replan_status_ptr +=
          static_cast<uint32_t>(OpenSpaceStatus::FREEZE_NEAR_END);
    }
    if ((executable_status & LARGE_YAW_ERROR_IN_GEAR_SHIFT) != 0) {
      *replan_status_ptr +=
          static_cast<uint32_t>(OpenSpaceStatus::YAW_TRACK_ABNORMAL);
    }
  }

  /**
  * @brief 
  * 
  * @return FinishType 
  */
  FinishType FinishCheck() {
    FinishType ret = ONRUN;
    switch (task_finish_status_) {
      case planning_internal::OpenSpaceDebug::REACH_TARGET:
      case planning_internal::OpenSpaceDebug::COLLISION_FINISH:
      case planning_internal::OpenSpaceDebug::REACH_WHEEL_MASK:
      case planning_internal::OpenSpaceDebug::BLOCK_BY_CURB_IN_SPOT:
      case planning_internal::OpenSpaceDebug::BLOCK_BY_CAR_IN_SPOT: {
        ret = SUCCESS;
        break;
      }
      case planning_internal::OpenSpaceDebug::OVER_TIME: {
        ret = FAIL;
        break;
      }
      case planning_internal::OpenSpaceDebug::UNKNOWN: {
        ret = TBD;
        break;
      }
      default: {
        break;
      }
    }
    return ret;
  }

  /**
   * @brief 
   * 
   * @param chosen_path
   * @return common::PathPoint 
   */
  common::PathPoint TaskTargetPose(const PartitionedPath& chosen_path);

  /**
   * @brief 
   * 
   * @param open_space_path_info
   * @return true
   * @return false
   */
  bool IsMirrorFold(
      const TL::planning::OpenSpacePathInfo& open_space_path_info);

  /**
   * @brief 
   * 
   * @param parking_scenario_type
   * @return true
   * @return false
   */
  static bool IsVerticalParkOut(
      const ParkingScenarioType& parking_scenario_type) {
    return (parking_scenario_type == LEFT_VERTICAL_PARKING_OUT ||
            parking_scenario_type == RIGHT_VERTICAL_PARKING_OUT ||
            parking_scenario_type == FORWARD_VERTICAL_PARKING_OUT ||
            parking_scenario_type == LEFT_OBLIQUE_PARKING_OUT ||
            parking_scenario_type == RIGHT_OBLIQUE_PARKING_OUT ||
            parking_scenario_type == FORWARD_OBLIQUE_PARKING_OUT);
  }

  OpenSpacePathPartitionConfig open_space_path_partition_config_;
  std::pair<TL::soc::Chassis_GearPosition, u_short> gear_shift_ = {
      TL::soc::Chassis::GEAR_PARKING, 0};
  common::VehicleParam vehicle_param_;
  PartitionedPath history_path_;
  planning_internal::OpenSpaceDebug::FinishStatus task_finish_status_ =
      planning_internal::OpenSpaceDebug::UNKNOWN;
  std::string path_decision_debug_;
  double ego_x_ = 0.0;
  double ego_y_ = 0.0;
  double ego_theta_ = 0.0;
  common::PathPoint start_point_;
  bool is_warm_start_ = false;
  double frozen_near_end_time_ = 0.0;
  double frozen_time_ = 0.0;
  bool is_veh_reach_destination_ = false;
  bool is_veh_prefinish_brake_saftisfied_ = false;
  bool is_clipped_ = false;
  double yaw_track_abnormal_start_time_ = 0.0;
  common::SLPoint end_point_sl_;
  common::PathPoint clipped_point_;
  bool is_mirror_fold_ = false;
};
}  // namespace planning
}  // namespace TL
