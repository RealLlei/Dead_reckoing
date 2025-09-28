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

#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "common/status/status.h"
#include "planning/common/dependency_injector.h"
#include "planning/common/frame.h"
#include "planning/common/open_space_info.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/path/path_data.h"
#include "planning/common/trajectory/discretized_trajectory.h"
#include "planning/tasks/optimizers/open_space_speed_optimizer/path_handle.h"
#include "planning/tasks/optimizers/open_space_speed_optimizer/st_sample_curves.h"
#include "planning/tasks/optimizers/speed_optimizer.h"

#include "planning/proto/open_space_task_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/types.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/fsm/avp_fct.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/planning/planning_internal.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {
using typename TL::planning_internal::AvpSpeedPlanCollisionInfo;
using typename TL::planning_internal::STSampleDebug;
using typename soc::Chassis;

/**
 * @brief optimizer opne space traj when collision
 *
 */
class OpenSpaceSpeedOptimizer : public SpeedOptimizer {
 public:
  /**
   * @brief Construct a new Open Space Speed Optimizer object
   *
   * @param config
   * @param injector
   */
  OpenSpaceSpeedOptimizer(const TaskConfig& config,
                          const std::shared_ptr<DependencyInjector>& injector);
  /**
   * @brief reset openspaceoptimizer
   *
   * @return TL::common::Status
   */
  TL::common::Status Reset() override;

 private:
  /**
   * @brief base class virtual function, need override
   *
   * @param path_data
   * @param init_point
   * @param speed_data
   * @return TL::common::Status
   */
  // LCOV_EXCL_START
  TL::common::Status Process(const PathData& path_data,
                                const common::TrajectoryPoint& init_point,
                                SpeedData* const speed_data) override {
    UNUSED(path_data);
    UNUSED(init_point);
    UNUSED(speed_data);
    return common::Status::OK();
  }

  // LCOV_EXCL_STOP

  /**
   * @brief opensapce speed optimizer procss
   *
   * @param frame
   * @return TL::common::Status
   */
  TL::common::Status Process(Frame* frame) override;

  /**
   * @brief generate emergency stop trajecoty, same point, v = 0, a = max
   *
   * @param trajectory_gear_ptr output trajectory
   * @return true
   * @return false
   */
  void GenerateStopTrajectory(TrajGearPair* trajectory_gear_ptr);

  /**
* @brief generate t shape speed profile in path length
*
* @param st_sample_params
* @param candidate_path
* @param traj_gear_ptr
* @return true success
* @return false failed
*/
  bool GenerateBackUpTrajectory(const StSampleParams& sample_params,
                                const DiscretizedPath& candidate_path,
                                TrajGearPair* traj_gear_ptr);
  /**
* @brief combine path data and speed data
*
* @param is_forward
* @param path_points path data
* @param speed_points speed data
* @param discretized_trajectory_ptr output trajectory
* @return true success
* @return false failed
*/
  bool CombinePathAndSpeed(bool is_forward, const DiscretizedPath& path_points,
                           const SpeedData& speed_points,
                           DiscretizedTrajectory* discretized_trajectory_ptr);

  /**
   * @brief speed plan pre check, if return !msg.empty(), no need speed plan
   * 
   * @param path 
   * @param gear 
   * @param is_stop_path 
   * @return * std::string 
   */
  static std::string SpeedPlanPreCheck(const DiscretizedPath& path,
                                       const Chassis::GearPosition& gear,
                                       bool is_stop_path);

  /**
   * @brief update speed plan input info
   * 
   * @param gear 
   * @param start_point 
   * @param is_rpa_mode
   * @return * void 
   */
  void UpdateSpeedPlanInputInfo(const Chassis::GearPosition& gear,
                                TrajectoryPoint* start_point,
                                bool* is_rpa_direct_mode);

  /**
   * @brief update st_sample_params_
   *
   * @param start_point
   * @param is_forward
   * @param end_s
   * @param start_s default 0
   * @param end_v default 0
   */
  void UpdateSampleParams(const common::TrajectoryPoint& start_point,
                          bool is_forward, double end_s, double start_s = 0.0,
                          double end_v = 0.0);

  /**
   * @brief  sample st curves
   *
   * @param sample_params
   * @return true success
   * @return false failed
   */
  bool SampleStCurves(const StSampleParams& sample_params);

  /**
   * @brief Get the Best Curve Idx object
   *
   * @param best_idx
   * @return true
   * @return false
   */
  bool GetBestCurveIdx(int* best_idx);

  /**
   * @brief sample trajectory
   *
   * @param sample_params
   * @param candidate_path
   * @param traj_gear
   * @return true sample trajectory success
   * @return false sample trajectory failed
   */
  bool SampleTrajectory(const StSampleParams& sample_params,
                        const DiscretizedPath& candidate_path,
                        TrajGearPair* traj_gear);

  /**
   * @brief generate trajectory
   *
   * @param sample_params
   * @param candidate_path
   * @param traj_gear_ptr
   * @param msg
   * @return true success
   * @return false failed
   */
  bool GenerateTrajectory(const StSampleParams& sample_params,
                          const DiscretizedPath& candidate_path,
                          TrajGearPair* traj_gear_ptr, std::string* msg);

  /**
   * @brief update st debug info
   * 
   * @param start_v 
   * @param start_acc 
   * @param total_s 
   * @param end_s 
   * @param actual_v 
   * @param actual_acc 
   * @param speed_limits 
   * @param speed_limit_unit_s 
   * @return * void 
   */
  void UpdateStDebugInfo(double start_v, double start_acc, double total_s,
                         double end_s, double actual_v, double actual_acc,
                         const std::vector<double>& speed_limits,
                         double speed_limit_unit_s);

  /**
   * @brief
   *
   */
  void RecordDebug();

  /**
   * @brief init interactive stage in process begin
   *
   * @param gear
   */
  void InitInteractiveStage(const Chassis::GearPosition& gear);

  /**
   * @brief cal diff time between curr and last frame
   *
   * @return * double
   */
  double CalDiffTimeFromLast();

 private:
  PathHandle path_handle_;
  bool is_forward_ = true;
  const double trajectory_unit_t_ = 0.1;

  AvpSpeedPlanCollisionInfo::SpeedTaskInteractiveStage interactive_stage_ =
      AvpSpeedPlanCollisionInfo::INIT;

  // sample params
  OpenSpaceSpeedOptimizerConfig::SpeedBoundInfo speed_bound_info_;
  StSampleParams st_sample_params_;
  StSampleCurves st_sample_curves_;
  std::shared_ptr<StCurve> last_curve_;
  std::vector<std::pair<size_t, double>> min_costs_;

  // record debug info
  STSampleDebug st_debug_info_;
};

}  // namespace planning
}  // namespace TL
