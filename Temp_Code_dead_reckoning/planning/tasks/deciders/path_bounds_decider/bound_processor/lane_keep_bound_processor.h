/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path lane keep bound processor
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
#include "planning/tasks/deciders/path_bounds_decider/obs_processor/obs_dynamic_processor.h"
#include "planning/tasks/deciders/path_bounds_decider/obs_processor/obs_static_processor.h"
#include "planning/tasks/deciders/path_bounds_decider/util/path_info.h"
#include "planning/tasks/deciders/path_bounds_decider/util/py_plot.h"

#include "planning/proto/planning_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/planning/path_bound.pb.h"

namespace TL {
namespace planning {

class LaneKeepBoundProcessor : public BoundProcessor {
 public:
  LaneKeepBoundProcessor(const std::shared_ptr<DependencyInjector>& injector,
                         const TaskConfig& config);

  /**
   * @brief The lane keep path boundary generation considers the ADC itself
   *   and other environments:
   *   - ADC's position (lane-borrow considerations)
   *   - lane info
   *   - static obstacles
   *   - dynamic obstacles
   *   The philosophy is: static and dynamic(big car) environment must be and can only be taken
   *   care of by the path planning.
   * 
   * @param reference_line_info 
   * @param path_bound: a vector for every point(s,l_min,l_max) in path boundary 
   * @param frame 
   * @param lane_type_pool: a vector for reusable lane type 
   * @return common::Status 
   */
  common::Status Process(ReferenceLineInfo* reference_line_info,
                         PathBound* path_bound, Frame* frame,
                         std::vector<LaneType>* lane_type_pool) override;

  /**
   * @brief Bound init
   * 
   * @param lane_borrow_info 
   * @param blocking_obstacle_id 
   * @param borrow_lane_type 
   * @return true 
   * @return false 
   */
  bool BoundInit(const PathInfo::LaneBorrowInfo& lane_borrow_info,
                 std::string* blocking_obstacle_id,
                 std::string* borrow_lane_type) override;

  /**
   * @brief Towing Points Filter Process
   * 
   * @param reference_line_info
   * @param towing_points 
   * @param towing_line 
   */
  void TowingPointsFilterProcess(ReferenceLineInfo* reference_line_info,
                                 TowingPointsInfo* towing_points,
                                 std::vector<double>* towing_line);

 private:
  /**
   * @brief GetGuideLineBoundsFromFreeSpace.
   * @param reference_line_info 
   * @param is_astar_bound 
   * @param guide_line_bound 
   */
  void ProcessBoundWithFreeSpace(const ReferenceLineInfo& reference_line_info,
                                 Frame* frame, PathBound* path_bound);

  /**
   * @brief GetFreeSpaceLineSegments.
   * @return true: 成功.
   */
  static bool ConvertFreeSpaceToLineSegments(
      Frame* frame,
      std::vector<common::math::LineSegment2d>* freespace_line_segments);

  /**
   * @brief CalculateFreeSpaceFlag.
   * @param line_segment 
   * @param common_point 
   * @param triangle_height_uint 
   * @param half_bottom_edge_width
   */
  static bool CalculateFreeSpaceFlag(
      const common::math::LineSegment2d& line_segment,
      const TL::planning::ReferencePoint& common_point,
      const common::math::Vec2d& triangle_height_uint,
      double half_bottom_edge_width);

  /**
   * @brief DoubleTriangleConstructor.
   * @param common_point
   * @param triangle_height 
   * @param bottom_angle
   * @param up_polygon 
   * @param down_polygon 
   * @return true: 成功.
   */
  static bool DoubleTriangleConstructor(const ReferencePoint& common_point,
                                        double triangle_height,
                                        double bottom_angle,
                                        common::math::Polygon2d* up_triangle,
                                        common::math::Polygon2d* down_triangle);
// #define BOUNDARY_PLOT
#ifdef BOUNDARY_PLOT
  static void PrepareLineAndTriData(
      Frame* frame,
      std::vector<common::math::LineSegment2d>* const freespace_line_segments,
      const std::vector<common::math::Polygon2d>& triangles_vec,
      const common::math::Box2d& vehicle_line);
  static void PrepareBoundaryData(const ReferenceLineInfo& reference_line_info,
                                  Frame* frame, PathBound* path_bound, int i);
  static void PlotData();
  static std::unique_ptr<PyPlot> py_plot_;
#endif
  PathInfo::LaneBorrowInfo lane_borrow_info_ =
      PathInfo::LaneBorrowInfo::NO_BORROW;
  std::string* blocking_obstacle_id_ = nullptr;
  std::string* borrow_lane_type_ = nullptr;
  std::shared_ptr<ProcessBound> process_bound_;
  std::shared_ptr<ObsStaticProcessor> obs_static_process_;
  std::shared_ptr<ObsDynamicProcessor> obs_dynamic_processor_;
};
}  // namespace planning
}  // namespace TL
