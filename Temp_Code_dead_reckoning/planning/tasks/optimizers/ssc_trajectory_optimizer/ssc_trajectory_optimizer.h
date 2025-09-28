/*
 * Copyright (c) TL Technologies Co., Ltd. 2023. All rights reserved.
 * Description:  guide_line_path_optimizer.h
 */

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/status/status.h"
#include "planning/math/discretized_points_smoothing/ipopt_pos_optimize_smoother.h"
#include "planning/tasks/optimizers/path_speed_optimizer.h"
#include "planning/tasks/optimizers/ssc_trajectory_optimizer/bezier_spline_generator/bezier_spline.h"
#include "planning/tasks/optimizers/ssc_trajectory_optimizer/ssc_map/ssc_map.h"
#include "planning/proto/planning_config.pb.h"
#include "planning/proto/task_config.pb.h"

namespace TL::planning {

class SscTrajectoryOptimizer : public PathSpeedOptimizer {
 public:
  SscTrajectoryOptimizer(const TaskConfig& config,
                         const std::shared_ptr<DependencyInjector>& injector);

  ~SscTrajectoryOptimizer() override = default;

  //   /**
  //    * @brief Initialize the planner with config path
  //    */
  //   bool Init(const std::string& config_path);

 private:
  /**
   * @brief Process
   * 
   * @param speed_data 
   * @param reference_line 
   * @param init_point 
   * @param path_reusable 
   * @param path_data 
   * @return common::Status 
   */
  common::Status Process(const ReferenceLine& reference_line,
                         const common::TrajectoryPoint& init_point,
                         PathData* path_data, SpeedData* speed_data) override;

  bool RunOptimization();

  static bool CorridorFeasibilityCheck(
      const std::vector<SpatioTemporalSemanticCubeNd<2>>& corridor);

  bool CombinePathAndSpeedData(
      std::vector<game_common::Vehicle>* forward_trajectory,
      std::unordered_map<int, std::vector<game_common::Vehicle>>*
          surround_forward_trajectories);

  /**
   * @brief RecordDebugInfo
   * 
   * @param path_data 
   * @param debug_name 
   * @param reference_line_info 
   */
  static void RecordDebugInfo(const PathData& path_data,
                              const std::string& debug_name,
                              ReferenceLineInfo* reference_line_info);

  SscTrajectoryOptimizerConfig optimizer_config_;
  std::shared_ptr<SscMap> ssc_map_;
  game_common::FrenetState init_frenet_state_;
  std::vector<BezierSpline<5, 2>> optimized_trajectories_;
  std::vector<std::vector<FsVehicle>> fs_forward_trajectory_vec_;
  std::vector<std::unordered_map<int, std::vector<FsVehicle>>>
      fs_surround_forward_trajectories_vec_;
};

}  // namespace TL::planning
