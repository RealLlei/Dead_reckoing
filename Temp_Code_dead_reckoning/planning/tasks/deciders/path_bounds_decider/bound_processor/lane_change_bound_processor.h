/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path lane change bound processor
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
#include "planning/tasks/deciders/path_bounds_decider/bound_processor/lane_change_heuristic_generator.h"
#include "planning/tasks/deciders/path_bounds_decider/bound_processor/process_bound.h"
#include "planning/tasks/deciders/path_bounds_decider/obs_processor/obs_static_processor.h"
#include "planning/proto/planning_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/planning/path_bound.pb.h"

namespace TL {
namespace planning {

class LaneChangeBoundProcessor : public BoundProcessor {
 public:
  LaneChangeBoundProcessor(const std::shared_ptr<DependencyInjector>& injector,
                           const TaskConfig& config);

  /** @brief The lane change path boundary generation considers the ADC itself
   *   and other static environments:
   *   - ADC's position (lane-changing considerations)
   *   - lane info
   *   - static obstacles
   *   The philosophy is: static environment must be and can only be taken
   *   care of by the path planning.
   * 
   * @param reference_line_info
   * @param lane_borrow_info: which lane to borrow.
   * @param path bound: a vector for every point(s,l_min,l_max) in path boundary
   * @param lane_type_pool: a vector for reusable lane type 
   * @return common::Status
   */
  common::Status Process(ReferenceLineInfo* reference_line_info,
                         PathBound* path_bound, Frame* frame,
                         std::vector<LaneType>* lane_type_pool) override;

  /**
   * @brief Blocking obstacle id init process
   *
   * @param blocking_obstacle_id
   */
  bool BlockingIDInit(std::string* blocking_obstacle_id) override;

  /**
   * @brief Get the Lane Change Start Status object
   *
   * @param reference_line_info
   * @param lane_change_start_sl fitst: s value, second: l value.
   */
  void GetLaneChangeStartStatus(
      const ReferenceLineInfo& reference_line_info,
      std::pair<double, double>* lane_change_start_sl);

  /**
   * @brief if adc doesn't arrive at lane_change_start_position(start position exist), 
   *        adjust bound with consideration of adc's position,lanes and buffers before lane change start s,
   *        otherwise, just return
   * 
   * @param reference_line_info 
   * @param path_bound 
   */
  void GetBoundaryFromLaneChangeForbiddenZone(
      const ReferenceLineInfo& reference_line_info, PathBound* path_bound,
      std::pair<double, double>* lane_change_start_sl);

  /**
    * @brief Calculate a distance for lane change since lane change start s
    *        adjust the boundary after this distance 
    * 
    * @param reference_line_info 
    * @param path_bound 
    */
  void GetBoundaryForLaneChangeConstraint(
      const Frame& frame, const ReferenceLineInfo& reference_line_info,
      PathBound* path_bound,
      const std::pair<double, double>& lane_change_start_sl);

  bool CheckCrossSolidLine(double lane_change_length,
                           const ReferenceLine& reference_line,
                           const PathBound& path_bound);

  /**
     * @brief !!Function not used!!
     *        Calculate forbid lane change index and adjust boundary before forbidden zone 
     *        with considering LaneType,adc's width and buffer.
     *               
     * @param reference_line_info 
     * @param path_boundaries 
     */
  void GetBoundaryFromLaneBoundaryType(
      const ReferenceLineInfo& reference_line_info, PathBound* path_bound);

  /**
     * @brief !!Function not used!!
     *        Adjust the boundary according to the lateral displacement.
     * 
     * @param path_boundaries 
     */
  void ProcessPathboundFromStartPoint(PathBound* path_boundaries);

 private:
  std::shared_ptr<ProcessBound> process_bound_;
  std::shared_ptr<ObsStaticProcessor> obs_static_process_;
  LaneChangeHeuristicGenerator lc_heuristic_generator_;

  double lane_change_length_ = 0.0;
  common::Point3D last_lane_change_end_position_;

  bool is_cross_solid_line_ = false;
  bool cross_solid_line_checked_ = false;
  double adjusted_lane_change_length_ = 0.0;
  std::string* blocking_obstacle_id_ = nullptr;
};
}  // namespace planning
}  // namespace TL
