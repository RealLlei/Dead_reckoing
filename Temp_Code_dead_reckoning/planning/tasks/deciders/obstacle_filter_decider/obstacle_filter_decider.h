
/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Description:  obstacle_filter_decider.h
 */

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "planning/tasks/deciders/decider.h"

namespace TL {
namespace planning {

class ObstacleFilterDecider : public Decider {
 public:
  ObstacleFilterDecider(const TaskConfig& config,
                        const std::shared_ptr<DependencyInjector>& injector);
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

 private:
  /**
   * @brief 筛选在guide_line_bounds中的静态障碍物
   * 
   * @param path_bounds 上游某块给出
   * @param obstacle 
   * @param reference_line_info 
   * @param path_bounds_resolution 
   */
  void FilterStaticObstacle(double path_bounds_resolution,
                            const PathBound& path_bounds,
                            const Obstacle& obstacle,
                            ReferenceLineInfo* reference_line_info) const;

  /**
   * @brief 检查边是不是在给定的bounds内（检查两个点）
   * 
   * @param path_bounds 
   * @param check_s 
   * @param check_start_l 
   * @param check_end_l 
   * @param path_bounds_resolution 
   * @return true：不在给定的bounds内 
   */
  bool CheckEdgeIsOutBounds(bool is_pedestrian,
                            const PathBound& path_bounds, double check_s,
                            double check_start_l, double check_end_l,
                            double path_bounds_resolution) const;

  /**
   * @brief 检查障碍物是不是堵塞bounds
   * 
   * @param path_bounds 
   * @param obstacle 
   * @param path_bounds_resolution 
   * @return true：堵塞 
   */
  static bool CheckObstacleIsBlockedBounds(const PathBound& path_bounds,
                                           const Obstacle& obstacle,
                                           double path_bounds_resolution);

  /**
   * @brief 检查点是不是在给定的bounds内（检查一个点）
   * 
   * @param path_bounds 
   * @param check_s 
   * @param check_start_l 
   * @param check_end_l 
   * @param path_bounds_resolution 
   * @return true：不在给定的bounds内 
   */
  bool CheckPointIsOutBounds(bool is_pedestrian,
                             const PathBound& path_bounds, double check_s,
                             double check_l,
                             double path_bounds_resolution) const;

  /**
   * @brief 筛选在guide_line_bounds中的动态障碍物
   * 
   * @param path_bounds 上游某块给出
   * @param obstacle 
   * @param reference_line_info 
   * @param path_bounds_resolution 
   */
  void FilterDynamicObstacle(double path_bounds_resolution,
                             const PathBound& path_bounds,
                             const Obstacle& obstacle,
                             ReferenceLineInfo* reference_line_info) const;
};
}  // namespace planning
}  // namespace TL
