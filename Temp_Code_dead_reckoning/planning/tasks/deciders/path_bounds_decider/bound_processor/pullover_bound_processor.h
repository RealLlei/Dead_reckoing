/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path pullover bound processor
 * Author: ROC
 */

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "planning/tasks/deciders/path_bounds_decider/bound_processor/bound_processor.h"
#include "planning/tasks/deciders/path_bounds_decider/bound_processor/process_bound.h"
#include "planning/tasks/deciders/path_bounds_decider/obs_processor/obs_static_processor.h"

#include "planning/proto/planning_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/planning/path_bound.pb.h"

namespace TL {
namespace planning {

class PullOverBoundProcessor : public BoundProcessor {
 public:
  PullOverBoundProcessor(const std::shared_ptr<DependencyInjector>& injector,
                         const TaskConfig& config);

  common::Status Process(ReferenceLineInfo* reference_line_info,
                         PathBound* path_bound, Frame* frame,
                         std::vector<LaneType>* lane_type_pool) override;

  /** @brief Refine the boundary based on the road-info.
   *  The returned boundary is with respect to the lane-center (NOT the
   *  reference_line), though for most of the times reference_line's
   *  deviation from lane-center is negligible.
   */
  bool GetBoundaryFromRoads(
      const ReferenceLineInfo& reference_line_info,
      std::vector<std::tuple<double, double, double>>* path_bound);

  /**
   * @brief Convert Boundary S Axis From Lane Center To Ref Line
   * 
   * @param reference_line_info 
   * @param path_bound 
   */
  static void ConvertBoundarySAxisFromLaneCenterToRefLine(
      const ReferenceLineInfo& reference_line_info,
      std::vector<std::tuple<double, double, double>>* path_bound);

  /** @brief Update left boundary by lane_left_width
   *   This is for normal pull-over, which uses lane boundary as left boundary
   *   and road_boundary for right boundary
   */
  void UpdatePullOverBoundaryByLaneBoundary(
      const ReferenceLineInfo& reference_line_info,
      std::vector<std::tuple<double, double, double>>* path_bound);

  /**
   * @brief Is Point Within Path Bound
   * 
   * @param reference_line_info 
   * @param x 
   * @param y 
   * @param path_bound 
   * @return int 
   */
  static int IsPointWithinPathBound(
      const ReferenceLineInfo& reference_line_info, double x, double y,
      const std::vector<std::tuple<double, double, double>>& path_bound);

  /**
   * @brief Search Pull Over Position
   * 
   * @param frame 
   * @param reference_line_info 
   * @param path_bound 
   * @param pull_over_configuration 
   * @return true 
   * @return false 
   */
  bool SearchPullOverPosition(
      const Frame& frame, const ReferenceLineInfo& reference_line_info,
      const std::vector<std::tuple<double, double, double>>& path_bound,
      std::tuple<double, double, double, int>* pull_over_configuration);

  /**
   * @brief Find Emergency Pull Over S
   * 
   * @param reference_line_info 
   * @param pull_over_s 
   * @return true 
   * @return false 
   */
  bool FindEmergencyPullOverS(const ReferenceLineInfo& reference_line_info,
                              double* pull_over_s);
  /**
   * @brief Find Destination Pull Over S
   * 
   * @param frame 
   * @param reference_line_info 
   * @param path_bound 
   * @param pull_over_s 
   * @return true 
   * @return false 
   */
  bool FindDestinationPullOverS(
      const Frame& frame, const ReferenceLineInfo& reference_line_info,
      const std::vector<std::tuple<double, double, double>>& path_bound,
      double* pull_over_s);

 private:
  std::shared_ptr<ProcessBound> process_bound_;
  std::shared_ptr<ObsStaticProcessor> obs_static_process_;
};
}  // namespace planning
}  // namespace TL
