/******************************************************************************
 * Copyright 2019 The TL Authors. All Rights Reserved.
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
#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "planning/common/open_space_info.h"
#include "planning/common/path/discretized_path.h"
#include "planning/proto/open_space_task_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/types.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/planning/decider_debug.pb.h"
#include "proto/planning/planning_internal.pb.h"

namespace TL {
namespace planning {

using common::math::Box2d;
using perception::FreeSpaceOutArray;
using perception::PerceptionObstacle_Type;
using planning::OpenSpaceSpeedOptimizerConfig;

class PathHandle {
 public:
  using FsPointInfo = std::tuple<const common::Point2D*, int>;

  struct CollisionInfo {
    struct ObstacleCollisionInfo {
      bool is_collision = false;
      size_t collision_index = std::numeric_limits<size_t>::max();
      int collision_obstacle_id = 0;
      PerceptionObstacle_Type collision_obstacle_type =
          PerceptionObstacle_Type::PerceptionObstacle_Type_UNKNOWN;

      void Init(size_t path_size) {
        is_collision = false;
        collision_index = path_size - 1;
        collision_obstacle_id = 0;
        collision_obstacle_type =
            PerceptionObstacle_Type::PerceptionObstacle_Type_UNKNOWN;
      }
    };

    struct FreeSpaceCollisionInfo {
      bool is_collision = false;
      size_t collision_index = std::numeric_limits<size_t>::max();
      FreeSpaceSegment freespace_segment;
      common::Point2D freespace_flu_point;

      void Init(size_t path_size) {
        is_collision = false;
        collision_index = path_size - 1;
      }
    };

    ObstacleCollisionInfo static_obstacle_collision_info;
    ObstacleCollisionInfo moving_obstacle_collision_info;
    ObstacleCollisionInfo outside_wheelmask_obstacle_collision_info;
    FreeSpaceCollisionInfo freespace_collision_info;
    bool is_collision = false;
    size_t first_collision_index = std::numeric_limits<size_t>::max();
    AvpSpeedPlanCollisionInfo::CollisionType collision_type =
        AvpSpeedPlanCollisionInfo::NO_COLLISION;
    double stop_reserve_distance = 0.0;
    double curr_collision_distance = std::numeric_limits<double>::max();

    void Init(size_t path_size) {
      static_obstacle_collision_info.Init(path_size);
      moving_obstacle_collision_info.Init(path_size);
      outside_wheelmask_obstacle_collision_info.Init(path_size);
      freespace_collision_info.Init(path_size);
      is_collision = false;
      first_collision_index = path_size - 1;
      collision_type = AvpSpeedPlanCollisionInfo::NO_COLLISION;
      stop_reserve_distance = 0.0;
      curr_collision_distance = std::numeric_limits<double>::max();
    }
  };

 public:
  explicit PathHandle(const OpenSpaceSpeedOptimizerConfig& config);
  ~PathHandle() = default;

  /**
   * @brief cut off path use wheel mask + obstacle + freespace, update collision debug info
   * 
   * @param path  complete path
   * @param obstacles 
   * @param freespace_out_array 
   * @param vehicle_state 
   * @param is_vehicle_still 
   * @param is_forward 
   * @param is_rpa_direct_mode
   * @param speed_bound_info 
   * @param is_mirror_fold 
   * @param open_space_info 
   * @param candidate_path    output cutoffed path
   * @param interactive_stage  output
   * @param mutable_open_space_info output
   * @return ** std::string  if msg.empty()-> success else failed
   */
  std::string Process(
      const DiscretizedPath& path,
      const std::vector<const std::shared_ptr<Obstacle>*>& obstacles,
      const std::shared_ptr<const FreeSpaceOutArray>& freespace_out_array,
      const common::VehicleState& vehicle_state, bool is_vehicle_still,
      bool is_forward, bool is_rpa_direct_mode,
      const OpenSpaceSpeedOptimizerConfig::SpeedBoundInfo& speed_bound_info,
      bool is_mirror_fold, const OpenSpaceInfo& open_space_info,
      DiscretizedPath* candidate_path,
      AvpSpeedPlanCollisionInfo::SpeedTaskInteractiveStage* interactive_stage,
      OpenSpaceInfo* mutable_open_space_info);

  /**
   * @brief Get the Speed Limits object
   * 
   * @return * const std::vector<double>& 
   */
  const std::vector<double>& GetSpeedLimits() const { return speed_limits_; }

  /**
   * @brief Get the Speed Limit Unit S object
   * 
   * @return * double 
   */
  double GetSpeedLimitUnitS() const { return speed_limit_unit_s_; }

 private:
  /**
  * @brief init info
  * 
  * @return * void 
  */
  void Init(const planning::OpenSpaceEnvStructuredInfo&
                open_space_env_structured_info);

  /**
   * @brief cut off path by wheel mask info
   * 
   * @param path 
   * @param is_forward 
   * @param is_parking_inwards
   * @param is_consider_wheel_mask 
   * @param wheel_mask_box 
   * @param new_path 
   * @return true 
   * @return false 
   */
  bool CutOffPathByWheelMask(const DiscretizedPath& path, bool is_forward,
                             bool is_parking_inwards,
                             bool is_consider_wheel_mask,
                             const Box2d& wheel_mask_box,
                             DiscretizedPath* new_path);

  /**
   * @brief update static obstacle segments vector
   * 
   * @return * void 
   */
  void UpdateStaticObstacleSegments();

  /**
   * @brief update moving obstacle boxs vector 
   * 
   * @return * void 
   */
  void UpdateMovingObstacleBoxs();

  /**
   * @brief update freespace info to kinds of vector by type/ is_lidar/
   * 
   * @param path 
   * @param freespace_out_array 
   * @param under_spot_low_fs_idxs 
   * @param high_curb_fs_idxs 
   * @param ignore_fs_idxs
   * @param max_s 
   * @param min_s 
   * @param max_l 
   * @param min_l 
   * @return * void 
   */
  void UpdateFreeSpaceInfo(
      const DiscretizedPath& path,
      const std::shared_ptr<const FreeSpaceOutArray>& freespace_out_array,
      const std::vector<size_t>& under_spot_low_fs_idxs,
      const std::vector<size_t>& high_curb_fs_idxs,
      const std::vector<std::pair<size_t, std::vector<size_t>>>&
          ignore_fs_idxs);

  /**
   * @brief update valid obstacle info, use sl fliter
   * 
   * @param path 
   * @param obstacles 
   * @param freespace_out_array 
   * @param under_spot_low_fs_idxs 
   * @param high_curb_fs_idxs 
   * @param ignore_fs_idxs
   * @param is_forward 
   * @param is_rpa_direct_mode
   * @return * void 
   */
  void UpdateValidObstacleInfo(
      const DiscretizedPath& path,
      const std::vector<const std::shared_ptr<Obstacle>*>& obstacles,
      const std::shared_ptr<const FreeSpaceOutArray>& freespace_out_array,
      const std::vector<size_t>& under_spot_low_fs_idxs,
      const std::vector<size_t>& high_curb_fs_idxs,
      const std::vector<std::pair<size_t, std::vector<size_t>>>& ignore_fs_idxs,
      bool is_forward, bool is_rpa_direct_mode);

  /**
  * @brief update collision buffer info
  * 
  * @param is_forward  
  * @return * OpenSpaceSpeedOptimizerConfig::CollisionBufferInfo 
  */
  OpenSpaceSpeedOptimizerConfig::CollisionBufferInfo UpdateCollisionBuffer(
      bool is_forward) const;

  /**
   * @brief calculate lateral control diff 
   * 
   * @param vehicle_state 
   * @param path_p 
   * @param left_lateral_buffer 
   * @param right_lateral_buffer 
   * @return * void 
   */
  static void CalLateralBufferByControlDiff(
      const common::VehicleState& vehicle_state,
      const common::PathPoint& path_p, double* left_lateral_buffer,
      double* right_lateral_buffer);

  /**
   * @brief is path collision with static obstacle
   * 
   * @param path 
   * @param is_forward
   * @param collision_info 
   * @return true  collision
   * @return false not collision
   */
  bool IsCollisionWithStaticObstacle(
      const DiscretizedPath& path, bool is_forward,
      CollisionInfo::ObstacleCollisionInfo* collision_info);

  /**
   * @brief is path collision with moving obstacle
   * 
   * @param path 
   * @param collision_info 
   * @return true collision
   * @return false not collision
   */
  bool IsCollisionWithMovingObstacle(
      const DiscretizedPath& path, bool is_forward,
      CollisionInfo::ObstacleCollisionInfo* collision_info);

  /**
   * @brief is path collision with static obstacle
   * 
   * @param path 
   * @param collision_buffer 
   * @param collision_info
   * @param left_control_diff 
   * @param right_control_diff
   * @return true  collision
   * @return false not collision
   */
  bool IsCollisionWithOutsideWheelMaskObstacle(
      const DiscretizedPath& path,
      const OpenSpaceSpeedOptimizerConfig::CollisionBufferInfo&
          collision_buffer,
      double left_control_diff, double right_control_diff,
      CollisionInfo::ObstacleCollisionInfo* collision_info);

  /**
   * @brief is path collision with freespaces
   * 
   * @param path 
   * @param collision_buffer 
   * @param left_control_diff 
   * @param right_control_diff 
   * @param is_use_middle_buffer 
   * @param path_type 
   * @param collision_info 
   * @param cur_path_idx
   * @return true 
   * @return false 
   */
  bool IsCollisionWithFreeSpaceSegment(
      const DiscretizedPath& path,
      const OpenSpaceSpeedOptimizerConfig::CollisionBufferInfo&
          collision_buffer,
      double left_control_diff, double right_control_diff,
      bool is_use_middle_buffer, bool is_mirror_fold,
      const planning_internal::PathUpdateStatus::PathType& path_type,
      size_t cur_path_idx,
      CollisionInfo::FreeSpaceCollisionInfo* collision_info);

  /**
   * @brief collision ifno decision by static/moving/freespace collision info
   * 
   * @param collision_info 
   * @return * void 
   */
  void CollisionInfoDecision(CollisionInfo* collision_info);

  /**
   * @brief  update collision info
   * 
   * @param path 
   * @param obstacles 
   * @param freespace_out_array 
   * @param under_spot_low_fs_idxs 
   * @param high_curb_fs_idxs
   * @param ignore_fs_idxs
   * @param vehicle_state 
   * @param is_forward 
   * @param is_rpa_direct_mode
   * @param partitioned_paths
   * @param collision_info 
   * @return true 
   * @return false 
   */
  bool UpdateCollisionInfo(
      const DiscretizedPath& path,
      const std::vector<const std::shared_ptr<Obstacle>*>& obstacles,
      const std::shared_ptr<const FreeSpaceOutArray>& freespace_out_array,
      const std::vector<size_t>& under_spot_low_fs_idxs,
      const std::vector<size_t>& high_curb_fs_idxs,
      const std::vector<std::pair<size_t, std::vector<size_t>>>& ignore_fs_idxs,
      const common::VehicleState& vehicle_state, bool is_forward,
      bool is_rpa_direct_mode, bool is_mirror_fold,
      const PartitionedPath& partitioned_paths, CollisionInfo* collision_info);

  /**
   * @brief udpate interactive stage
   * 
   * @param is_vehicle_still 
   * @param is_rpa_direct_mode
   * @param collision_info 
   * @param interactive_stage 
   * @return * void 
   */
  void UpdateInteractiveStage(
      bool is_vehicle_still, bool is_rpa_direct_mode,
      const CollisionInfo& collision_info,
      AvpSpeedPlanCollisionInfo::SpeedTaskInteractiveStage* interactive_stage);

  void UpdateIsUseMiddleBuffer(
      const AvpSpeedPlanCollisionInfo::SpeedTaskInteractiveStage&
          interactive_stage,
      bool curr_is_forward,
      const planning_internal::PathUpdateStatus::PathType& path_type);

  /**
   * @brief cutoff path by collision info
   * 
   * @param path 
   * @param collision_info 
   * @param candidate_path 
   * @return true 
   * @return false 
   */
  static bool CutOffPathByCollisionInfo(const DiscretizedPath& path,
                                        const CollisionInfo& collision_info,
                                        DiscretizedPath* candidate_path);

  /**
   * @brief calculate limit speed by s
   * 
   * @param s 
   * @param speed_bound_info
   * @return * double 
   */
  static double CalLimitSpeedByS(
      double s,
      const OpenSpaceSpeedOptimizerConfig::SpeedBoundInfo& speed_bound_info);

  /**
   * @brief update speed limits
   * 
   * @param path 
   * @param speed_bound_info
   * @param limit_speed_path_points
   * @return * void 
   */
  void UpdateSpeedLimits(
      const DiscretizedPath& path,
      const OpenSpaceSpeedOptimizerConfig::SpeedBoundInfo& speed_bound_info,
      bool is_forward,
      const std::vector<common::PathPoint>& limit_speed_path_points);

  void SmoothSpeedLimits();

  /**
   * @brief update collision info
   * 
   * @param collision_info 
   * @param freespace_out_array 
   * @param interactive_stage 
   * @param future_collision_point 
   * @param is_vehicle_still 
   * @param mutable_open_space_info 
   * @return * void 
   */
  void UpdateDebugInfo(
      const CollisionInfo& collision_info,
      const AvpSpeedPlanCollisionInfo::SpeedTaskInteractiveStage&
          interactive_stage,
      const common::PathPoint& future_collision_point, bool is_vehicle_still,
      double wheel_mask_distance, OpenSpaceInfo* mutable_open_space_info);

  /**
   * @brief update collision risk count
   * 
   * @param path 
   * @param path_type 
   * @param cur_path_idx 
   * @param is_mirror_fold
   * @param is_forward 
   * @return * void 
   */
  void UpdatePathCollisionRiskCount(
      const DiscretizedPath& path,
      const planning_internal::PathUpdateStatus::PathType& path_type,
      size_t cur_path_idx, bool is_mirror_fold, bool is_forward);

  /**
   * @brief update real_time lateral buffer
   * 
   * @param collision_buffer
   * @param is_use_middle_buffer
   * @param is_narrow_spot_scenario
   * @param is_lat_park_in
   * @param is_lateral_park_out
   * @param path_type
   * @param cur_path_idx
   * @param lateral_buffer_for_vehicle
   * @param lateral_buffer_for_not_vehicle
   * @param lateral_buffer_for_low_fs
   * @return * void 
   */
  static void UpdateLateralBuffer(
      const OpenSpaceSpeedOptimizerConfig::CollisionBufferInfo&
          collision_buffer,
      bool is_use_middle_buffer, bool is_narrow_spot_scenario,
      bool is_lat_park_in, bool is_lat_park_out,
      const planning_internal::PathUpdateStatus::PathType& path_type,
      size_t cur_path_idx,
      std::pair<double, double>* lateral_buffer_for_vehicle,
      std::pair<double, double>* lateral_buffer_for_not_vehicle,
      std::pair<double, double>* lateral_buffer_for_low_fs);

 private:
  const OpenSpaceSpeedOptimizerConfig& config_;

  // obstacle info
  const size_t max_moving_obs_size_ = 50;
  const size_t max_static_obs_size_ = 50;
  const size_t max_uss_obs_size_ = 50;
  const size_t max_wheelmask_obs_size_ = 50;

  size_t moving_obs_size_ = 0;
  std::vector<const Obstacle*> moving_obs_ptrs_;

  size_t static_obs_size_ = 0;
  std::vector<const Obstacle*> static_obs_ptrs_;

  size_t uss_obs_size_ = 0;
  std::vector<const Obstacle*> uss_obs_ptrs_;

  size_t wheelmask_obs_size_ = 0;
  std::vector<const Obstacle*> wheelmask_obs_ptrs_;

  const double predict_unit_t_ = 0.2;
  const size_t predict_box_size_ = 0;
  std::vector<std::vector<common::math::Box2d>> moving_obs_boxs_;

  std::vector<std::pair<common::math::LineSegment2d, double>>
      static_vehicle_segments_;
  std::vector<std::pair<int, size_t>> static_vehicle_segment_count_;
  std::vector<std::pair<common::math::LineSegment2d, double>>
      static_pedestrian_segments_;
  std::vector<std::pair<int, size_t>> static_pedestrian_segment_count_;
  std::vector<std::pair<common::math::LineSegment2d, double>>
      static_other_segments_;
  std::vector<std::pair<int, size_t>> static_other_segment_count_;

  std::vector<FreeSpaceSegment> not_lidar_not_vehicle_fs_segments_;
  std::vector<FreeSpaceSegment> not_lidar_vehicle_fs_segments_;
  std::vector<FreeSpaceSegment> lidar_vehicle_fs_segments_;
  std::vector<FreeSpaceSegment> lidar_not_vehicle_fs_segments_;
  std::vector<FreeSpaceSegment> low_height_fs_segments_;
  std::vector<FreeSpaceSegment> high_height_curb_fs_segments_;

  std::vector<std::pair<common::math::LineSegment2d, double>>
      not_lidar_not_vehicle_fs_;
  std::vector<std::pair<common::math::LineSegment2d, double>>
      not_lidar_vehicle_fs_;
  std::vector<std::pair<common::math::LineSegment2d, double>> lidar_vehicle_fs_;
  std::vector<std::pair<common::math::LineSegment2d, double>>
      lidar_not_vehicle_fs_;
  std::vector<std::pair<common::math::LineSegment2d, double>> low_height_fs_;
  std::vector<std::pair<common::math::LineSegment2d, double>>
      high_height_curb_fs_;

  std::vector<std::pair<common::math::LineSegment2d, double>>
      all_freespace_segments_;

  bool pre_wheel_mask_valid_ = false;

  const double speed_limit_unit_s_ = 0.2;
  std::vector<double> speed_limits_;

  double wait_replan_start_time_ = common::Clock::NowInSeconds();
  double wait_replan_to_init_time_ = common::Clock::NowInSeconds();
  double wait_obstacle_start_time_ = common::Clock::NowInSeconds();

  int wait_obstacle_count_ = 0;
  int bigger_buffer_safe_count_ = 0;
  bool is_use_middle_buffer_ = false;

  bool is_narrow_spot_scenario_ = false;
  bool is_lateral_park_in_ = false;
  bool is_lateral_park_out_ = false;
  bool is_vertical_park_in_ = false;
  bool is_nns_adjust_ = false;
  std::pair<bool, bool> last_is_forward_{false, false};

  int current_path_has_collision_count_ = 0;
  size_t current_path_collision_index_ = 0;
};

}  // namespace planning
}  // namespace TL
