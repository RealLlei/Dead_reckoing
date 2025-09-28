/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path info
 * Author: ROC
 */

#pragma once

#include <array>
#include <limits>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "planning/common/reference_line_info.h"

#include "proto/planning/path_bound.pb.h"

namespace TL {
namespace planning {

constexpr static double kLimitStartPointdl = 0.2;
// TODO(all): Update extra tail point base on vehicle speed.
constexpr static int kNumExtraTailBoundPoint = 20;
constexpr static double kPulloverLonSearchCoeff = 1.5;
constexpr static double kPulloverLatSearchCoeff = 1.25;
constexpr static double KAlmostZero1ENegtive3 = 1e-3;
constexpr static double KAlmostZero1ENegtive5 = 1e-5;
constexpr static double KAlmostZero1ENegtive6 = 1e-6;
constexpr static double KAlmostZero1ENegtive8 = 1e-8;
constexpr static double kMinObstacleArea = 1e-4;

struct ObstacleEdge {
  double obstacle_edge_start_s;
  double obstacle_edge_end_s;
  double obstacle_edge_l_min;
  double obstacle_edge_l_max;
  std::string obstacle_id;
  const Obstacle* obstacle;
};

// Towing Points: (s, left towing_l, right towing_l, obs id)
using TowingPointsInfo =
    std::vector<std::tuple<double, double, double, std::string>>;

struct LaneType {
  bool is_left_line_solid = false;
  bool is_left_line_virtual = false;
  bool is_right_line_solid = false;
  bool is_right_line_virtual = false;
  bool is_curved_road = false;
};

class PathInfo {
 public:
  enum class LaneBorrowInfo {
    LEFT_BORROW,
    NO_BORROW,
    RIGHT_BORROW,
  };

  struct LanesAndADCPathBoundDebugInfoInput {
    const PathBoundType* path_bound_type;
    const LaneBorrowInfo* lane_borrow_info;
    const PathBound* path_bound;
    ReferenceLineInfo* reference_line_info;
    bool borrowing_reverse_lane;
    const std::vector<double>* curr_neighbor_lane_widths;
    const std::vector<double>* ADC_speed_buffers;
    const std::vector<double>* ADC_buffers;
    const std::vector<double>* curr_left_bound_lanes;
    const std::vector<double>* curr_right_bound_lanes;
    const std::vector<double>* curr_left_bounds;
    const std::vector<double>* curr_right_bounds;
    const std::vector<double>* curr_lane_left_widths;
    const std::vector<double>* curr_lane_right_widths;
  };

  static void LanesAndADCPathBoundDebugInfo(
      const LanesAndADCPathBoundDebugInfoInput& input);

  static void PathBoundDebugInfo(const PathBoundType& path_bound_type,
                                 const PathBound& path_bound,
                                 ReferenceLineInfo* reference_line_info);

  static void RecordDebugInfo(
      const std::vector<std::tuple<double, double, double>>& path_boundaries,
      const std::string& debug_name, ReferenceLineInfo* reference_line_info);

  /** @brief Print out the path bounds for debugging purpose.
   */
  static void PathBoundsDebugString(
      const std::vector<std::tuple<double, double, double>>& path_boundaries);

  /** @brief Print out the path bounds for debugging purpose.
   */
  static void PathBoundsDebugString(
      const PathBound& path_boundaries,
      const ReferenceLineInfo& reference_line_info, double adc_frenet_l);

  /**
   * @brief Print out the towing line for debugging purpose.
   * 
   * @param path_boundaries
   * @param towing_line
   * @param reference_line_info
   */
  static void TowingLineDebugInfo(const PathBound& path_boundaries,
                                  const std::vector<double>& towing_line,
                                  ReferenceLineInfo* reference_line_info);
};
}  // namespace planning
}  // namespace TL
