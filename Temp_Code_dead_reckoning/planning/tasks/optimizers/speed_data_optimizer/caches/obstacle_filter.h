/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_cost.h
 **/

#pragma once

#include <utility>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/caches/st_obstacle_cache.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {
namespace speed_evaluator {

/**
 * @brief ObstacleFilter is used to filter dynamic obstacle 
 * 
 */
class ObstacleFilter final {
 public:
  enum class CellState { NOT_VISITED = 0, DRIVABLE = 1, NOT_DRIVABLE = 2 };

  /**
   * @brief Main process
   *
   * @param config vt sample config
   * @param obstacle_caches obstacle caches
   */
  void Process(const SpeedCacheConfig& config,
               std::vector<STObstacleCache>* obstacle_caches);  // NOLINT

 private:
  /**
   * @brief Use BFS to find drivable cell
   * 
   */
  void VisitCell();

  std::vector<std::vector<CellState>> grid_;
  int s_count_ = 0;
  int t_count_ = 0;
  const std::vector<std::pair<int, int>> steps_ = {{0, 1}, {1, 1}, {1, 0}};
};

}  // namespace speed_evaluator
}  // namespace planning
}  // namespace TL
