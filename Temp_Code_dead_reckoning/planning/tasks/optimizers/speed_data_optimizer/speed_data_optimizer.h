/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_data_optimizer.h
 **/

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/caches/nudge_obstacle_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/speed_data_generator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_scenario_manager.h"
#include "planning/tasks/optimizers/speed_optimizer.h"

#include "planning/proto/task_config.pb.h"
#include "proto/planning/planning_internal.pb.h"

namespace TL {
namespace planning {

/**
 * @class SpeedDataOptimizer
 * @brief this class is used to generate speed data
 */
class SpeedDataOptimizer : public SpeedOptimizer {
 public:
  SpeedDataOptimizer(const TaskConfig& config,
                     const std::shared_ptr<DependencyInjector>& injector);

 private:
  /**
   * @brief Create speed scenario manager
   *
   */
  void CreateScenarioManager();

  std::shared_ptr<SpeedDataGenerator> SelectGenerator(
      const std::shared_ptr<SpeedScenario>& scenario);

  /**
   * @brief vt sample optimizer main process
   *
   * @param path_data path planning result
   * @param init_point planning start point
   * @param speed_data vt sample optimizer result
   * @return common::Status
   */
  common::Status Process(const PathData& path_data,
                         const common::TrajectoryPoint& init_point,
                         SpeedData* speed_data) override;

  /**
   * @brief print speed limit info from 0m to length
   *
   * @param length
   * @return
   */
  void RecordSpeedLimit(double length);

  /**
   * @brief print obstacles info
   *
   */
  void RecordObstacles();

  /**
   * @brief record debug info
   *
   * @param path_data
   */
  void RecordDebug(const PathData& path_data);

  // optimizer config
  SpeedDataOptimizerConfig optimizer_config_;
  // cost table and cost used to accelerate query performance
  SpeedCache cache_;
  // speed data generator
  std::shared_ptr<SpeedDataGenerator> forward_generator_;
  std::shared_ptr<SpeedDataGenerator> reverse_generator_;
  // speed scenario manager
  std::shared_ptr<SpeedScenarioManager> scenario_manager_;
};

}  // namespace planning
}  // namespace TL
