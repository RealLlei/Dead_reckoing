/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path bounds decider
 */

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "planning/tasks/deciders/decider.h"
#include "planning/tasks/deciders/path_bounds_decider/bound_processor/bound_processor.h"
#include "planning/tasks/deciders/path_bounds_decider/util/path_info.h"

#include "planning/proto/planning_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/planning/path_bound.pb.h"

namespace TL {
namespace planning {

class PathBoundsDecider : public Decider {
 public:
  PathBoundsDecider(const TaskConfig& config,
                    const std::shared_ptr<DependencyInjector>& injector);

 private:
  /** @brief Every time when Process function is called, it will:
   *        1.Must generate fallback bound or return error.
   *        2.Sequentially generate pullver, lane change,lane keep bound,
   *          once generate one of them,return OK.
   *        3.Save the candidate to reference_line_info
   */
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

  /**
   * @brief Fallback path bound process
   * 
   * @param frame 
   * @param reference_line_info 
   * @param candidate_path_boundaries 
   * @param lane_type_pool
   * @return true 
   * @return false 
   */
  bool FallbackPathBoundProcess(
      Frame* frame, ReferenceLineInfo* reference_line_info,
      std::vector<PathBoundary>* candidate_path_boundaries,
      std::vector<LaneType>* lane_type_pool);

  /**
   * @brief Universal path bound process
   * 
   * @param frame 
   * @param reference_line_info 
   * @param candidate_path_boundaries 
   * @param lane_type_pool
   * @return true 
   * @return false 
   */
  bool UniversalPathBoundProcess(
      Frame* frame, ReferenceLineInfo* reference_line_info,
      std::vector<PathBoundary>* candidate_path_boundaries,
      std::vector<LaneType>* lane_type_pool);

  /**
   * @brief Pull over process
   * 
   * @param frame 
   * @param reference_line_info 
   * @param candidate_path_boundaries
   * @param lane_type_pool
   * @return true 
   * @return false 
   */
  bool PullOverProcess(Frame* frame, ReferenceLineInfo* reference_line_info,
                       std::vector<PathBoundary>* candidate_path_boundaries,
                       std::vector<LaneType>* lane_type_pool);

  /**
   * @brief Lane change process
   * 
   * @param frame 
   * @param reference_line_info 
   * @param candidate_path_boundaries 
   * @param lane_type_pool
   * @return true 
   * @return false 
   */
  bool LaneChangeProcess(Frame* frame, ReferenceLineInfo* reference_line_info,
                         std::vector<PathBoundary>* candidate_path_boundaries,
                         std::vector<LaneType>* lane_type_pool);

  /**
   * @brief Lane borrow info list switch
   * 
   * @param reference_line_info 
   * @param lane_borrow_info_list 
   * @return true 
   * @return false 
   */
  bool LaneBorrowSwitch(
      const ReferenceLineInfo& reference_line_info,
      std::vector<PathInfo::LaneBorrowInfo>* lane_borrow_info_list);

  /**
   * @brief generate lane keep path bound according to lane borrow information 
   * 
   * @param lane_borrow_info_list lane borrow information
   * @param frame current frame
   * @param reference_line_info reference line info
   * @param candidate_path_boundaries candidate path boundaries
   * @param lane_type_pool
   * @return true 
   * @return false 
   */
  bool LaneKeepProcess(
      const std::vector<PathInfo::LaneBorrowInfo>& lane_borrow_info_list,
      Frame* frame, ReferenceLineInfo* reference_line_info,
      std::vector<PathBoundary>* candidate_path_boundaries,
      std::vector<LaneType>* lane_type_pool);

  /**
   * @brief !!Function not used!!
   *   Remove redundant path bounds in the following manner:
   *   - if "left" is contained by "right", remove "left"; vice versa.
   * 
   * @param candidate_path_boundaries 
   */
  static void RemoveRedundantPathBoundaries(
      std::vector<PathBoundary>* candidate_path_boundaries);

  /**
   * @brief !!Function not used!!
   * 
   * @param lhs 
   * @param rhs 
   * @return true 
   * @return false 
   */
  static bool IsContained(const std::vector<std::pair<double, double>>& lhs,
                          const std::vector<std::pair<double, double>>& rhs);

  std::shared_ptr<BoundProcessor> lane_keep_bound_processor_;
  std::shared_ptr<BoundProcessor> lane_change_bound_processor_;
  std::shared_ptr<BoundProcessor> pull_over_bound_processor_;
  std::shared_ptr<BoundProcessor> fallback_bound_processor_;
};
}  // namespace planning
}  // namespace TL
