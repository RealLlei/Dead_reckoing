/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_cost.h
 **/
#include "planning/tasks/optimizers/speed_data_optimizer/caches/obstacle_filter.h"

#include <algorithm>
#include <cstdint>
#include <queue>

namespace TL {
namespace planning {
namespace speed_evaluator {

void ObstacleFilter::Process(
    const SpeedCacheConfig& config,
    std::vector<STObstacleCache>* const obstacle_caches) {
  if (obstacle_caches == nullptr) {
    return;
  }

  const auto new_s_count =
      static_cast<int>(ceil(config.obstacle_filter_max_s() /
                            config.obstacle_filter_unit_s())) +
      1;
  const auto new_t_count =
      static_cast<int>(ceil(config.obstacle_filter_max_t() /
                            config.obstacle_filter_unit_t())) +
      1;
  if (s_count_ != new_s_count || t_count_ != new_t_count) {
    s_count_ = new_s_count;
    t_count_ = new_t_count;
    grid_.assign(t_count_,
                 std::vector<CellState>(s_count_, CellState::NOT_VISITED));
  }

  for (auto& row : grid_) {
    std::fill(row.begin(), row.end(), CellState::NOT_VISITED);
  }

  for (int t_index = 0; t_index < t_count_; ++t_index) {
    for (const auto& obstacle_cache : *obstacle_caches) {
      const auto& obstacle_info = obstacle_cache.GetObstacleInfoAtTime(
          t_index * config.obstacle_filter_unit_t());
      const auto start_s_index =
          std::max(0, static_cast<int>(round(obstacle_info.s_lower /
                                             config.obstacle_filter_unit_s())));
      const auto end_s_index = std::min(
          s_count_, static_cast<int>(round(obstacle_info.s_upper /
                                           config.obstacle_filter_unit_s())) +
                        1);
      for (int s_index = start_s_index; s_index < end_s_index; ++s_index) {
        grid_[t_index][s_index] = CellState::NOT_DRIVABLE;
      }
    }
  }

  VisitCell();

  for (auto& obstacle_cache : *obstacle_caches) {
    obstacle_cache.SetIsIgnore(true);
    const auto start_t_index =
        std::max(0, static_cast<int>(round(obstacle_cache.GetMinT() /
                                           config.obstacle_filter_unit_t())));
    const auto end_t_index = std::min(
        t_count_, static_cast<int>(round(obstacle_cache.GetMaxT() /
                                         config.obstacle_filter_unit_t())) +
                      1);
    for (int t_index = start_t_index; t_index < end_t_index; ++t_index) {
      const auto& obstacle_info = obstacle_cache.GetObstacleInfoAtTime(
          t_index * config.obstacle_filter_unit_t());
      const auto start_s_index =
          static_cast<int>(
              round(obstacle_info.s_lower / config.obstacle_filter_unit_s())) -
          1;
      const auto end_s_index =
          static_cast<int>(
              round(obstacle_info.s_upper / config.obstacle_filter_unit_s())) +
          1;
      if ((t_index == 0 && start_s_index == -1) ||
          (start_s_index >= 0 && start_s_index < s_count_ &&
           grid_[t_index][start_s_index] == CellState::DRIVABLE) ||
          (end_s_index >= 0 && end_s_index < s_count_ &&
           grid_[t_index][end_s_index] == CellState::DRIVABLE)) {
        obstacle_cache.SetIsIgnore(false);
        break;
      }
    }
  }
}

void ObstacleFilter::VisitCell() {
  std::queue<int> s_indexs;
  std::queue<int> t_indexs;
  s_indexs.push(0);
  t_indexs.push(0);
  while (!s_indexs.empty() && s_indexs.size() == t_indexs.size()) {
    const auto size = s_indexs.size();
    for (std::size_t i = 0; i < size; ++i) {
      const auto s_index = s_indexs.front();
      const auto t_index = t_indexs.front();
      s_indexs.pop();
      t_indexs.pop();

      for (const auto& step : steps_) {
        const auto new_s_index = s_index + step.first;
        const auto new_t_index = t_index + step.second;

        if (new_s_index < 0 || new_s_index >= s_count_ || new_t_index < 0 ||
            new_t_index >= t_count_ ||
            grid_[new_t_index][new_s_index] != CellState::NOT_VISITED) {
          continue;
        }

        grid_[new_t_index][new_s_index] = CellState::DRIVABLE;
        s_indexs.push(new_s_index);
        t_indexs.push(new_t_index);
      }
    }
  }
}

}  // namespace speed_evaluator
}  // namespace planning
}  // namespace TL
