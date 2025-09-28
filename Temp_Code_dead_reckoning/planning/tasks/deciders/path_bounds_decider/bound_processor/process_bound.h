/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path bound process
 * Author: ROC
 */

#pragma once

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "planning/common/dependency_injector.h"
#include "planning/common/reference_line_info.h"

#include "planning/tasks/deciders/path_bounds_decider/util/path_info.h"

namespace TL {
namespace planning {
class ProcessBound {
 public:
  ProcessBound(const std::shared_ptr<DependencyInjector>& injector,
               const TaskConfig& config);
  ~ProcessBound() = default;

  /**
    * @brief Initial some private variables for the following calculate 
    * 
    * @param path_bound 
    * @param injector 
    * @return true: normal
    * @return false: frame or reference line is nullptr
    */
  bool InitPathBounds(Frame* frame, ReferenceLineInfo* reference_line_info);

  /**
    * @brief Initial path bound to be infinite at every spot 
    * 
    * @param path_bound 
    * @param injector 
    * @return true: normal
    * @return false: path bound is empty
    */
  bool InitPathBoundary(PathBound* path_bound,
                        const std::shared_ptr<DependencyInjector>& injector);

  /**
   * @brief Refine the boundary based on lane-info and ADC's location.
   *   It will comply to the lane-boundary. However, if the ADC itself
   *   is out of the given lane(s), it will adjust the boundary
   *   accordingly to include ADC's current position.
   * 
   * @param lane_borrow_info: left borrow,right borrow,no borrow
   * @param path_bound_type: fallback,lane change,lane keep,pullover
   * @param path_bound: a vector for every point(s,l_min,l_max) in path boundary
   * @param borrow_lane_type:  "forward","reverse"
   * @param ADC_buffer: default 0.15 
   * @param lane_type_pool a vector for reusable lane type 
   * @param is_fallback_lanechange: always true
   * @return true 
   * @return false 
   */
  bool GetBoundaryFromLanesAndADC(
      const PathInfo::LaneBorrowInfo& lane_borrow_info,
      const PathBoundType& path_bound_type, PathBound* path_bound,
      std::string* borrow_lane_type, double ADC_buffer,
      std::vector<LaneType>* lane_type_pool,
      bool is_fallback_lanechange = false);

  /**
   * @brief check whether lane_type_pool is empty, if it is, fill it;
   *        otherwise, return directly to reuse it.
   * @param path_bound: a vector for every point(s,l_min,l_max) in path boundary
   * @param lane_type_pool: a vector for reusable lane type 
  */
  void CheckLaneTypePool(const PathBound& path_bound,
                         std::vector<LaneType>* lane_type_pool);

  // This function not used and has calculation error in it.
  double EvaluateLaneChangeDistance(
      const ReferenceLineInfo& reference_line_info,
      TL::common::VehicleState* vehicle_state_ptr) const;

  /**
   * @brief Update the path_boundary at "idx" with considering buffer and adc width
   *      It also checks if ADC is blocked (lmax < lmin).
   * 
   * @param idx: The current index of the path_bounds
   * @param left_bound: The minimum left boundary (l_max)
   * @param right_bound: The maximum right boundary (l_min)
   * @param path_boundaries: The path_boundaries (its content at idx will be updated)
   * @param lane_type lane type
   * @param is_left_lane_bound: Is the left bound comes from lane boundary
   * @param is_right_lane_bound: Is the right bound comes from lane boundary
   * @return true: normal
   * @return false: block
   */
  bool UpdatePathBoundaryWithBuffer(size_t idx, double left_bound,
                                    double right_bound,
                                    PathBound* path_boundaries,
                                    const LaneType& lane_type,
                                    bool is_left_lane_bound = false,
                                    bool is_right_lane_bound = false);

  /**
   * @brief Update the path_boundary at "idx" with considering buffer and adc width
   *      It also checks if ADC is blocked (lmax < lmin).
   * 
   * @param idx: The current index of the path_bounds
   * @param left_bound: The minimum left boundary (l_max)
   * @param right_bound: The maximum right boundary (l_min)
   * @param path_boundaries: The path_boundaries (its content at idx will be updated)
   * @param reference_line_info
   * @param is_left_lane_bound: Is the left bound comes from lane boundary
   * @param is_right_lane_bound: Is the right bound comes from lane boundary
   * @return true: normal
   * @return false: block
   */
  bool UpdatePathBoundaryWithBuffer(
      size_t idx, double left_bound, double right_bound,
      PathBound* path_boundaries, const ReferenceLineInfo& reference_line_info,
      bool is_left_lane_bound = false, bool is_right_lane_bound = false);

  /**
   * @brief 
   * 
   * @param idx 
   * @param left_bound 
   * @param right_bound 
   * @param path_boundaries 
   * @return true 
   * @return false 
   */
  bool UpdateObstaclePathBoundaryWithBuffer(size_t idx, double left_bound,
                                            double right_bound,
                                            PathBound* path_boundaries);

  /**
    * @brief check whether the lane boundary type of check_s, check_s's forward 
    * and check_s's backward is solid.
    * 
    * @param reference_line_info 
    * @param check_s: current s to check
    * @param lane_type: first: 1.is left lane solid, 2.is left lane virtual, second: 1.is right lane solid, 2. is right lane virtual
    * @return true: normal 
    * @return false: Get projection points from reference line failed.
    */
  bool CheckLaneSolidType(const ReferenceLineInfo& reference_line_info,
                          double check_s, LaneType* lane_type);

  /**
   * @brief !!Function not used!!
   *  Refine the boundary based on the lane-info.
   *  The returned boundary is with respect to the lane-center (NOT the
   *  reference_line), though for most of the times reference_line's
   *  deviation from lane-center is negligible.
   * 
   * @param reference_line_info 
   * @param lane_borrow_info 
   * @param path_bound 
   * @param borrow_lane_type 
   * @return true 
   * @return false 
   */
  bool GetBoundaryFromLanes(const ReferenceLineInfo& reference_line_info,
                            const PathInfo::LaneBorrowInfo& lane_borrow_info,
                            PathBound* path_bound,
                            std::string* borrow_lane_type) const;

  /**
   * @brief switch traj_point from rear axe to front axe
   * 
   * @param traj_point：Cartesian 
   * @return common::TrajectoryPoint: Cartesian 
   */
  static common::TrajectoryPoint InferFrontAxeCenterFromRearAxeCenter(
      const common::TrajectoryPoint& traj_point);

  /**
   * @brief !!Function not used!!
   *  Refine the boundary based on the ADC position and velocity.
   *  The returned boundary is with respect to the lane-center (NOT the
   *  reference_line), though for most of the times reference_line's
   *  deviation from lane-center is negligible.
   * 
   * @param reference_line_info 
   * @param ADC_extra_buffer 
   * @param path_bound 
   * @return true 
   * @return false 
   */
  bool GetBoundaryFromADC(const ReferenceLineInfo& reference_line_info,
                          double ADC_extra_buffer, PathBound* path_bound) const;

  /**
   * @brief check whether the borrow direction lane boundary is solid 
   * 
   * @param reference_line_info 
   * @param check_s 
   * @param lane_borrow_info: RIGHT_BORROW, LEFT_BORROW, NO_BORROW
   * @return true: not solid 
   * @return false: is solid 
   */
  static bool CheckLaneBoundaryType(
      const ReferenceLineInfo& reference_line_info, double check_s,
      const PathInfo::LaneBorrowInfo& lane_borrow_info);

  /**
   * @brief if lane_borrow_info is unknown, ensure lane_borrow_info in this override
   * 
   * @param reference_line_info 
   * @param check_s 
   * @param lane_change_type: RIGHT, LEFT, FORWARD
   * @return call override function  
   */
  static bool CheckLaneBoundaryType(
      const ReferenceLineInfo& reference_line_info, double check_s,
      const routing::ChangeLaneType& lane_change_type);

  /**
   * @brief compare adc_frenet_l and 0 to determine the lane_change_type
   * 
   * @param adc_frenet_l 
   * @return routing::ChangeLaneType: adc_frenet_l>0:LEFT, adc_frenet_l<0: RIGHT, equals:FORWARD
   */
  static routing::ChangeLaneType JudgeLaneChangeType(double adc_frenet_l);

  /** @brief calculate the length half the adc width
   *  @return The length of half the adc width
   */
  static double GetBufferBetweenADCCenterAndEdge();

  /** @brief !!Function just be called by an unused function!!
   *         Update the path_boundary at "idx", It also checks if
             ADC is blocked (lmax < lmin).
   *  @param idx: The current index of the path_bounds
   *  @param left_bound: The minimum left boundary (l_max)
   *  @param right_bound: The maximum right boundary (l_min)
   *  @param path_boundaries: its content at idx will be updated
   *  @return If path is good, true; if path is blocked, false.
   */
  static bool UpdatePathBoundary(size_t idx, double left_bound,
                                 double right_bound,
                                 PathBound* path_boundaries);

  /**
   * @brief Trim the path bounds starting at the idx where path is blocked.
   * 
   * @param path_blocked_idx: the index of blocking point 
   * @param path_boundaries 
   */
  static void TrimPathBounds(int path_blocked_idx, PathBound* path_boundaries);

  /**
   * @brief Trim the towing line starting at the idx where path is blocked.
   * 
   * @param path_blocked_idx: the index of blocking point 
   * @param towing_points 
   */
  static void TrimTowingPoints(int path_blocked_idx,
                               TowingPointsInfo* towing_points);

  /**
   * @brief reverse the path bound to adapt to TBA
   *
   * @param pnc_length look forward distance
   * @param path_bound regular or fallback path_bound
   */
  void ReversePathBound(double pnc_length, PathBound* path_bound);

  /**
   * @brief Filter Path Bound Peaks
   * 
   * @param path_bound
   */
  void FilterPathBoundPeaks(PathBound* path_bound);

  /**
   * @brief shrink the path bound at the end of the path
   *
   * @param shrink_distance the point after which the path bound is to shrink
   * @param shrink_end_distance the point before which the path bound is to shrink
   * @param path_bound regular or fallback path_bound
   * @return true succeeded to shrink
   * @return false failed to shrink
   */
  bool ShrinkPathBoundAtEnd(double shrink_distance, double shrink_end_distance,
                            PathBound* path_bound);

  /**
   * @brief delete the obstacle edge which is out if considered scope
   * 
   * @param processing_edge_idx: the index of all the in scope edges 
   * @param right_bounds 
   * @param left_bounds 
   * @param obstacle_edges 
   * @param curr_s 
   */
  void DeleteOutOfScopeObstacle(
      std::vector<ObstacleEdge>* obstacle_edges,
      std::vector<size_t>* processing_edge_idx,
      std::multiset<std::pair<double, std::string>, std::greater<>>*
          right_bounds,
      std::multiset<std::pair<double, std::string>>* left_bounds,
      double curr_s) const;

  /**
   * @brief Steering Wheel Speed Limit Bound Process
   * 
   * @param path_bound 
   * @return true 
   * @return false 
   */
  bool SteeringWheelSpeedLimitBoundProcess(PathBound* path_bound);

  /**
   * @brief judge whether has obstacle behind adc in the neighbor lane
   * 
   */
  void IsClearToExpandLaneBound();

  /**
   * @brief judge whether allow virtual lane bound
   * 为了保证R500弯道压线能刹住，非锥桶目标优先考虑停车
   * @param path_boundaries 
   * @param obstacle_edges 
   */
  void IsAllowVirtualLaneBound(const PathBound& path_boundaries,
                               const std::vector<ObstacleEdge>& obstacle_edges);

  /**
   * @brief judge whether allow expend lane bound
   * 
   * @param path_boundaries 
   * @param obstacle_edges 
   */
  void IsAllowExpendLaneBound(const PathBound& path_boundaries,
                              const std::vector<ObstacleEdge>& obstacle_edges);

  double GetAdcFrenetS() const { return adc_frenet_s_; }

  double GetAdcFrenetL() const { return adc_frenet_l_; }

  double GetAdcFrenetSd() const { return adc_frenet_sd_; }

  double GetAdcFrenetLd() const { return adc_frenet_ld_; }

  double GetStartPointDl() const { return start_point_dl_; }

  double GetAdcLaneWidth() const { return adc_lane_width_; }

  double GetAdcDefaultLaneWidth() const { return adc_default_lane_width_; }

  double GetAdcLaneLeftWidth() const { return adc_lane_left_width_; }

  double GetAdcLaneRightWidth() const { return adc_lane_right_width_; }

  const Frame* GetFrame() const { return frame_; }

  Frame* GetMutableFrame() const { return frame_; }

  const ReferenceLineInfo* GetReferenceLineInfo() const {
    return reference_line_info_;
  }

  ReferenceLineInfo* GetMutableReferenceLineInfo() {
    return reference_line_info_;
  }

  const TL::planning::LaneBoundProcess& GetLaneBoundConf() const {
    return lane_bound_conf_;
  }

  const TL::planning::ObsTowingProcess& GetObsTowingConf() const {
    return obs_towing_conf_;
  }

  /**
   * @brief Check camera lane boundary type
   * 
   * @param is_right_lane
   * @return true 
   * @return false 
   */
  bool CheckCameraLaneBoundaryType(bool is_right_lane);

  bool CheckIfInJunction() const;

  void SetIsAllowExpandLeftLaneBound(bool is_allow_expand_left_lane_bound) {
    is_allow_expand_left_lane_bound_ = is_allow_expand_left_lane_bound;
  }

  void SetIsAllowExpandRightLaneBound(bool is_allow_expand_right_lane_bound) {
    is_allow_expand_right_lane_bound_ = is_allow_expand_right_lane_bound;
  }

  void SetIsAllowLeftVirtualLaneBound(bool is_allow_left_virtual_lane_bound) {
    is_allow_left_virtual_lane_bound_ = is_allow_left_virtual_lane_bound;
  }

  void SetIsAllowRightVirtualLaneBound(bool is_allow_right_virtual_lane_bound) {
    is_allow_right_virtual_lane_bound_ = is_allow_right_virtual_lane_bound;
  }

  double adc_frenet_s_ = 0.0;
  double adc_frenet_l_ = 0.0;
  double adc_frenet_sd_ = 0.0;
  double adc_frenet_ld_ = 0.0;
  double start_point_dl_ = 0.0;
  double adc_lane_width_ = 0.0;
  double adc_default_lane_width_ = 0.0;
  double adc_lane_left_width_ = 0.0;
  double adc_lane_right_width_ = 0.0;
  double adc_l_to_lane_center_ = 0.0;
  bool is_clear_to_expand_left_lane_bound_ = true;
  bool is_clear_to_expand_right_lane_bound_ = true;
  bool is_allow_expand_left_lane_bound_ = false;
  bool is_allow_expand_right_lane_bound_ = false;
  bool is_allow_left_virtual_lane_bound_ = true;
  bool is_allow_right_virtual_lane_bound_ = true;

  Frame* frame_ = nullptr;
  ReferenceLineInfo* reference_line_info_ = nullptr;
  std::shared_ptr<DependencyInjector> injector_;
  const TaskConfig config_;
  TL::planning::LaneBoundProcess lane_bound_conf_;
  TL::planning::ObsTowingProcess obs_towing_conf_;
};
}  // namespace planning
}  // namespace TL
