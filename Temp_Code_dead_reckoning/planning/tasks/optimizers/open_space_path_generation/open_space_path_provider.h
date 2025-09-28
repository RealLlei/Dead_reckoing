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

#include <sys/types.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/status/status.h"
#include "common/time/clock.h"
#include "planning/common/open_space_info.h"
#include "planning/common/path/discretized_path.h"
#include "planning/open_space/open_space_thread/open_space_thread_manager.h"
#include "planning/tasks/optimizers/open_space_path_generation/open_space_path_generator.h"
#include "planning/tasks/optimizers/open_space_path_generation/open_space_path_smoother.h"
#include "planning/tasks/optimizers/path_optimizer.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_model_config.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

enum PlanThreadStatus {
  OFF = 1,
  RUNNING = 2,
  // UPDATE_SUC = 3,
  // UPDATE_FAIL = 4,
};

class OpenSpacePathProvider : public PathOptimizer {
 public:
  OpenSpacePathProvider(const TaskConfig& config,
                        const std::shared_ptr<DependencyInjector>& injector);

  ~OpenSpacePathProvider() override = default;

  // void Stop();

  TL::common::Status Reset() override;

  double GetSubThreadCostTimeMs() const { return thread_cost_time_ * 1000; }

 private:
  TL::common::Status Process() override;

  /**
   * @brief meaningless method to mathch path optimizer task class
   * 
   * @param speed_data 
   * @param reference_line 
   * @param init_point 
   * @param path_reusable 
   * @param path_data 
   * @return TL::common::Status 
   */
  TL::common::Status Process(const SpeedData& /*speed_data*/,
                                const ReferenceLine& /*reference_line*/,
                                const common::TrajectoryPoint& /*init_point*/,
                                bool /*path_reusable*/,
                                PathData* /*path_data*/) override {
    return TL::common::Status::OK();
  }

  /**
   * @brief  planning on path thread
   * 
   * @return TL::common::Status 
   */
  TL::common::Status PlanningOnPathThread();

  /**
   * @brief 
   * 
   * @param optimized_path_ptr 
   * @param update_path_segment_size_ptr 
   * @param path_status 
   */
  void UpdatePathStatus(
      PartitionedPath* optimized_path_ptr, size_t* update_path_segment_size_ptr,
      planning_internal::PathUpdateStatus::UpdateStatus* path_status);

  /**
   * @brief 
   * 
   * @return true 
   * @return false 
   */
  bool HasValidHistoryPath();
  /**
   * @brief 
   * 
   * @param optimized_path 
   * @param update_path_segment_size_ptr 
   * @param update_path_status_ptr 
   * @return true 
   * @return false 
   */
  bool CheckPathValid(const std::vector<PathGearPair>& optimized_path,
                      size_t* update_path_segment_size_ptr,
                      planning_internal::PathUpdateStatus::UpdateStatus*
                          update_path_status_ptr);
  /**
   * @brief 
   * 
   * @return TL::common::Status 
   */
  TL::common::Status PreCheck();

  // void GeneratePathThread();

  /**
   * @brief 
   * 
   */
  void UpdateReplanInfo();

  /**
   * @brief 
   * 
   * @param open_space_path_info 
   * @param open_space_path_input 
   * @param open_space_path_output 
   */
  void LoadOptimizerData(const OpenSpacePathInfo& open_space_path_info,
                         OpenSpacePathInput* open_space_path_input,
                         OpenSpacePathOutput* open_space_path_output);

  /**
   * @brief Set the Warm Start Path object
   * 
   * @param start_point 
   * @param apa_trace_path 
   * @param warm_start_path_ptr 
   */
  void SetWarmStartPath(const common::PathPoint& start_point,
                        const DiscretizedPath& apa_trace_path,
                        DiscretizedPath* warm_start_path_ptr) const;
  /**
   * @brief 
   * 
   */
  void UpdateCurTaskReplanStatus();

  /**
   * @brief 
   * 
   */
  void ResetSpliceTraj() {
    splice_path_data_.first.clear();
    splice_path_data_.second = soc::Chassis::GEAR_NONE;
  }

  /**
   * @brief Set the Path Strategy object
   * 
   * @param open_space_path_info 
   * @param splice_path_size 
   * @param start_point 
   * @param start_gear 
   * @param dest_region_with_angle 
   * @param trace_path
   * @param path_strategy 
   */
  void SetPathStrategy(const OpenSpacePathInfo& open_space_path_info,
                       size_t splice_path_size,
                       const common::PathPoint& start_point,
                       const soc::Chassis::GearPosition& start_gear,
                       const DestRegionWithAng& dest_region_with_angle,
                       const DiscretizedPath& trace_path,
                       PathStrategy* path_strategy);

  /**
   * @brief
   * 
   * @param open_space_env_structured_info 
   * @param trace_path 
   * @param start_point 
   * @param is_lon_reach 
   * @param is_lat_reach 
   */
  void GetAdcStatusOnTracePath(const ParkingScenarioType& parking_scenario_type,
                               const DiscretizedPath& trace_path,
                               const common::PathPoint& start_point,
                               bool* is_lon_reach, bool* is_lat_reach);

  /**
   * @brief 
   * 
   * @param start_point 
   * @param open_space_env_structured_info 
   * @param trace_path 
   * @param path_strategy 
   */
  void UseTracePathStrategy(
      const common::PathPoint& start_point,
      const OpenSpaceEnvStructuredInfo& open_space_env_structured_info,
      const DiscretizedPath& trace_path, PathStrategy* path_strategy);

  /**
 * @brief 
 * 
 * @param start_point 
 * @param dest_region_with_angle 
 * @param parking_scenario_type 
 * @param is_plan_from_start 
 * @param space_structure 
 * @param park_direction 
 */
  void PlanDirectionDecision(const common::PathPoint& start_point,
                             const DestRegionWithAng& dest_region_with_angle,
                             const ParkingScenarioType& parking_scenario_type,
                             bool* is_plan_from_start,
                             SpaceStructure* space_structure,
                             ParkDirection* park_direction);

  /**
   * @brief 
   * 
   * @param open_space_env_structured_info 
   * @param is_plan_from_start 
   * @param cut_off_strategy 
   */

  static void SteerCutoffStrategy(
      const OpenSpaceEnvStructuredInfo& open_space_env_structured_info,
      bool is_plan_from_start, int* cut_off_strategy);

  /**
 * @brief 
 * 
 * @param parking_scenario_type 
 * @param plan_from_start 
 * @param init_move_direction 
 * @param use_geometry_strategy 
 */
  void UseGeometryStrategy(const ParkingScenarioType& parking_scenario_type,
                           bool plan_from_start, int init_move_direction,
                           GeometryStrategy* use_geometry_strategy);

  /**
   * @brief 
   * 
   * @param use_larger_curvature 
   */
  void KappaStrategy(bool* use_larger_curvature);

  /**
 * @brief 
 * 
 * @param plan_from_start 
 * @param space_structure 
 * @param collision_free_search_strategy 
 */
  void LocalCollisionFreeSearchStrategy(
      bool plan_from_start, const SpaceStructure& space_structure,
      CollisionFreeSearchStrategy* collision_free_search_strategy);

  /**
 * @brief 
 * 
 * @param trace_path 
 * @param path_search_strategy 
 * @param trace_adjust_search_strategy 
 */
  static void LocalTraceAdjustSearchStrategy(
      const DiscretizedPath& trace_path,
      PathSearchStrategy* path_search_strategy,
      TraceAdjustSearchStrategy* trace_adjust_search_strategy);

  /**
 * @brief used to constrain change rate of kappa
 * 
 * @param splice_path_size 
 * @param start_gear 
 * @param move_direction 
 */
  static void InitMoveDirectionStrategy(
      size_t splice_path_size, const soc::Chassis::GearPosition& start_gear,
      int* move_direction);
  /**
   * @brief used to constrain start point move direction
   * 
   * @param open_space_env_structured_info
   * @param splice_path_size 
   * @param start_gear 
   * @param init_move_direction 
   */
  void ForceInitDirectionStrategy(
      const OpenSpaceEnvStructuredInfo& open_space_env_structured_info,
      size_t splice_path_size, const soc::Chassis::GearPosition& start_gear,
      int* init_gear_direction) const;

  /**
    * @brief used to judge if front or back lot is empty when lateral parking
    *
    */
  bool HasExtraSpaceNearby();

  void PrePlan();

  OpenSpacePathInput open_space_path_input_;
  OpenSpacePathOutput open_space_path_output_;

  //  std::shared_ptr<OpenSpacePathGenerator> path_generator_ptr_;
  //  std::shared_ptr<OpenSpacePathSmoother> path_smoother_ptr_;
  //  std::future<void> task_future_;
  //  std::atomic<bool> is_generation_thread_stop_{false};
  //  std::atomic<bool> data_ready_{false};
  //  std::atomic<bool> is_reach_precise_target_{false};
  //  std::atomic<uint64_t> thread_cost_time_{0};
  //  std::atomic<uint64_t> thread_start_time_{0};
  //  std::atomic<uint64_t> no_valid_path_start_time_{0};
  //  std::atomic<PlanThreadStatus> plan_thread_status_{PlanThreadStatus::OFF};
  //  std::mutex open_space_mutex_;
  //  std::condition_variable open_space_cv_;

  std::unique_ptr<common::PathPoint> init_adc_point_;
  common::VehicleModelConfig vehicle_model_config_;
  planning_internal::OpenSpaceDebug optimizer_debug_;
  std::vector<std::pair<int, planning_internal::OpenSpaceDebug>>
      optimizer_multi_debugs_;
  PathGearPair splice_path_data_;
  uint32_t replan_status_ = 0;
  bool is_reach_precise_target_{false};
  bool is_entered_special_domain_{false};
  double thread_cost_time_{0.0};
  double thread_start_time_ = common::Clock::NowInSeconds();
  double no_valid_path_start_time_ = common::Clock::NowInSeconds();
  PlanThreadStatus plan_thread_status_{PlanThreadStatus::OFF};

  OpenSpaceThreadManager<planning::OpenSpacePathProviderConfig,
                         OpenSpacePathGenerator, OpenSpacePathSmoother,
                         OpenSpacePathInput, OpenSpacePathOutput,
                         planning_internal::OpenSpaceDebug>
      open_space_thread_manager_;
};

}  // namespace planning
}  // namespace TL
