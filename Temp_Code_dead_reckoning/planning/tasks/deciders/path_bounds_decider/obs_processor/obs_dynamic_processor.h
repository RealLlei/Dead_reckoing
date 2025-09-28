/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning obstacle dynamic processor
 */

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "planning/common/dependency_injector.h"
#include "planning/common/frame.h"
#include "planning/tasks/deciders/path_bounds_decider/bound_processor/process_bound.h"
#include "planning/tasks/deciders/path_bounds_decider/obs_processor/dynamic_buffer_processor.h"
#include "planning/tasks/deciders/path_bounds_decider/obs_processor/obs_processor.h"
#include "planning/tasks/deciders/path_bounds_decider/util/path_info.h"

#include "proto/planning/path_bound.pb.h"

namespace TL {
namespace planning {

constexpr static double kExtendTime = 0.3;
constexpr static double kMinExtendLength = 5.0;
constexpr static double kMaxExtendLength = 10.0;
// speed knot info contains: (knot_t, knot_s)
using SpeedKnotInfo = std::vector<std::pair<double, double>>;
// SLBoundary Info contains: (SLBoundary, is_overlap_with_adc, obstacle_id)
using SLBoundaryInfo = std::tuple<SLBoundary, bool, std::string>;

class ObsDynamicProcessor : public ObsProcessor {
 public:
  ObsDynamicProcessor(const std::shared_ptr<DependencyInjector>& injector,
                      const TaskConfig& config);
  ~ObsDynamicProcessor() override = default;

  /**
   * @brief sort the dynamic obstacles and adjust path bound with the
   * consideration of dynamic obstacles
   *   !!! it is only valid for big car !!!
   *
   * @param reference_line_info
   * @param frame
   * @param path_boundaries
   * @param blocking_obstacle_id
   * @param towing_points s, left towing_l, right towing_l, obs id
   * @param is_enable_towing_process
   * @return true: normal
   * @return false: adjust path bound with dynamic obstacles failed
   */
  bool Process(ReferenceLineInfo* reference_line_info, Frame* frame,
               std::vector<std::tuple<double, double, double>>* path_boundaries,
               std::string* blocking_obstacle_id,
               TowingPointsInfo* towing_points,
               bool is_enable_towing_process) override;

  /**
   * @brief !!Function not used!!
   *
   * @param reference_line_info
   * @param name
   * @param time_diff_ms
   */
  static void RecordElapseTimeInfo(ReferenceLineInfo* reference_line_info,
                                   const std::string& name,
                                   double time_diff_ms);

  /**
   * @brief !!Function not be called!!
   *
   * @param ADC_buffer
   */
  void DynamicObsInit(double ADC_buffer);

  /**
   * @brief use ratio to calculate a value between a and b
   * 
   * @param a lower value
   * @param b upper value
   * @param ratio 
   * @return double: the result
   */
  static double LookUpValue(double a, double b, double ratio);

 private:
  using ObsProcessor::Process;
  /**
   * @brief calculate extend length from start point to avoid boundary
   * constraint too much. initial is_avoid_big_car_ to false,clear big_car_ids_
   */
  bool Init();

  /**
   * @brief 1.Screen out the obstacles to be considered and sort them
   *          sorted regulation :
   *            sort with lower_s ,small to large
   *            if lower_s are equal ,smaller upper_s first.
   *        2.Calculate towing line
   * 
   * @param indexed_obstacle 
   * @param path_boundaries 
   * @param dynamic_obstacles_edges 
   * @param towing_points 
   * @param towing_line 
   */
  void SortDynamicObstaclesFromSweepLine(
      const IndexedList<std::string, Obstacle>& indexed_obstacle,
      PathBound* path_boundaries,
      std::vector<ObstacleEdge>* dynamic_obstacles_edges,
      TowingPointsInfo* towing_points);

  /**
   * @brief Find every path_matched_speed_point within optimize_length
   *      put their s and t in speed_knots_info
   *
   * @param path_boundaries
   * @param speed_knots_info: a vector of speed_knots_info(s && t)
   * @return true: normal
   * @return false: fail to prepare for finding path_matched_speed_points
   */
  bool GetSampleKnots(const PathBound* path_boundaries,
                      SpeedKnotInfo* speed_knots_info);

  /**
   * @brief According last_speed_data initial speed_data
   *
   * @param speed_data: a vector of SpeedPoint
   * @return true: normal
   * @return false: prepare to initial failed
   * @return MockLonTrajectory1d(function): depend on the return of
   * MockLonTrajectory1d
   */
  bool InitSpeedData(SpeedData* speed_data);

  /**
   * @brief !!Function not used!!
   *      Filter out the obstacle that we do not need to consider
   *      only ensure exist big car can return true
   *
   * @param obstacle
   * @param considering_length: the length in our consideration
   * @return true: exist a big car
   * @return false: any other
   */
  bool IsConsideringDynamicObstacle(const Obstacle* obstacle,
                                    double considering_length);
  /**
   * @brief Determine whether the obstacle is not beyond the lane
   *
   * @param sl_boundary: obs's sl_boundary
   * @return true: is beyond the lane
   * @return false: within the lane
   */
  bool ObstacleIsWithinLane(const SLBoundary* sl_boundary);

  /**
   * @brief Determine whether the obstacle is in the same direction with adc
   *
   * @param obstacle
   * @return true: the difference of heading directions between obs and adc <
   * M_PI / 2.0
   * @return false: otherwise
   */
  bool IsSameDirectionObstacleFromADC(const Obstacle* obstacle) const;

  /**
   * @brief Determine whether the obstacle beyond adc's lane too much(more than
   * 0.1), or the lane width is too narrow to consider a dynamic obstacle
   *
   * @param obstacle
   * @return true: otherwise
   * @return false: the lane width is too narrow or beyond adc's lane too much
   */
  bool IsObstacleDeadAheadADCWithBuffer(const Obstacle* obstacle) const;

  /**
   * @brief Determine whether the road has reasonable curvature(kappa <= 0.009)
   *
   * @param obstacle
   * @return true: otherwise
   * @return false: one of start_point and end_point's curvature > 0.009
   */
  bool IsObstacleInReasonableCurvatureRoad(const Obstacle* obstacle) const;

  /**
   * @brief Use linear interpolation to find an approximate point 
   *        to descript the obstacle
   * 
   * @param tp0_iter lower value
   * @param tp1_iter upper value
   * @param obstacle_sl_boundary_ptr_point the result of obstacle description
   * @param t time for estimation
   */
  static void InterpolatePointUsingLinearApproximation(
      const std::vector<ObsPointDescription>::const_iterator& tp0_iter,
      const std::vector<ObsPointDescription>::const_iterator& tp1_iter,
      std::pair<std::vector<ObsPointDescription>::const_iterator,
                ObsPointDescription>* obstacle_sl_boundary_ptr_point,
      double t);

  /**
   * @brief To Calculate the Obstacle SL Boundary point at target time
   * 
   * @param estimation_time the target time
   * @param start_iter start point for finding the matched ObsPointDescription
   * @param obstacle_trajectory_envelope a vector of ObsPointDescription
   * @param obstacle_sl_boundary_ptr_point the result of obstacle description
   */
  static void GetObstacleSLBoundaryPointAtTime(
      double estimation_time,
      const std::vector<ObsPointDescription>::const_iterator& start_iter,
      const std::vector<ObsPointDescription>& obstacle_trajectory_envelope,
      std::pair<std::vector<ObsPointDescription>::const_iterator,
                ObsPointDescription>* obstacle_sl_boundary_ptr_point);

  /**
   * @brief According to the trajectory of the obstacle,
   *        calculate and record the obstacle boundary
   *        each time the obstacle overlaps the path boundary
   *
   * @param speed_data: a vector of (t,s)
   * @param obstacle
   * @param obs_sl_boundary
   * @return std::vector<SLBoundaryInfo>: the obstacle's boundary at the time
   * overlaped with path bound
   */
  void CalculateSLBoundary(const SpeedKnotInfo& speed_data,
                           const Obstacle& obstacle,
                           std::vector<SLBoundaryInfo>* obs_sl_boundary);

  /**
   * @brief Combine infomation:
   *          is_start_s(1,is the start s of the boundary  0,is the end s of the
   * boundary), s, l_min, l_max, obs_id) with consideration of longitude and
   * lateral buffer.
   *
   * @param sl_boundaries
   * @param obstacle
   * @param obstacle_edges_process_bound: for bound process
   * @param obstacle_edges_process_towing_line: for towing line process
   * @return true
   */
  bool CalculateObstacleEdges(
      const std::vector<SLBoundaryInfo>& sl_boundaries,
      const Obstacle& obstacle,
      std::vector<ObstacleEdge>* obstacle_edges_process_bound,
      std::vector<ObstacleEdge>* obstacle_edges_process_towing_line);

  /**
   * @brief calculate the Dynamic Obstacle Buffer or use default value
   *
   * @param sl_boundary
   * @param obstacle
   * @return double: buffer
   */
  double GetDynamicObstacleBuffer(const SLBoundaryInfo& sl_boundary,
                                  const Obstacle& obstacle);

  /**
   * @brief calculate start s and end s,
   *       after change dynamic obstacle to static obstacle
   * 
   * @param obstacle_edges_process_towing_line 
   * @param path_boundaries 
   * @param towing_points 
   */
  void ProcessObstacleEdgesForSetTowingPoints(
      const std::vector<ObstacleEdge>& obstacle_edges_process_towing_line,
      PathBound* path_boundaries, TowingPointsInfo* towing_points);

  /**
   * @brief prepare to call TowingPointsBigCarProcess
   * 
   * @param path_boundaries 
   * @param cur_obs_s_edges: pair of start s and end s
   * @param cur_obs_l_edges: pair of start l and end l
   * @param obstacle_id 
   * @param towing_points 
   */
  void ToCallTowingProcess(const std::pair<double, double>& cur_obs_s_edges,
                           const std::pair<double, double>& cur_obs_l_edges,
                           const std::string& obstacle_id,
                           PathBound* path_boundaries,
                           TowingPointsInfo* towing_points);

  /**
   * @brief According to obstacle_edges to adjust path boundaries,
   *     if the dynamic obstacle's boundary overlaped with path boundary
   *     adjust path boundary and determine whether there is blocking
   *
   * @param obstacle_edges: a vector of (is_start_s, s, l_min, l_max, id)
   * @param path_boundaries
   * @param blocking_obstacle_id: which obstacle caused block
   * @return true: normal
   * @return false: there is no path boundaries or obstacle edges
   */
  bool ConstructBoundaryFromDynamicObstacles(
      std::vector<ObstacleEdge>* obstacle_edges, PathBound* path_boundaries,
      std::string* blocking_obstacle_id);

  /**
   * @brief Calculate a buffer for dynammic obstacles
   *        But, this function is unuse now.
   * @param path_boundary
   * @param obstacle_edges: all the edges for considered dynamic obstacles
   * @param path_boundary_index: the index of current boundary point
   * @param obstacle_edge_index: the index of current obstacle's egde
   * @param is_left_obstacle: always false
   * @return const double: 0.0
   */
  double DynamicBuffer(const PathBound& path_boundary,
                       const std::vector<ObstacleEdge>& obstacle_edges,
                       size_t path_boundary_index, size_t obstacle_edge_index,
                       bool is_left_obstacle) const;

  /**
   * @brief !!Function not used!!
   *
   * @param current_obstacle_edge_index
   * @param opposite_obstacle_edge_index
   * @param obstacle_edges
   * @return true
   * @return false
   */
  bool IsScaleBufferByOppositeObstacles(
      std::size_t current_obstacle_edge_index,
      std::size_t opposite_obstacle_edge_index,
      const std::vector<ObstacleEdge>& obstacle_edges) const;

  /**
   * @brief calculate the value of half adc_width
   *
   * @return const double: half adc_width
   */
  static double GetBufferBetweenADCCenterAndEdge();

  /**
   * @brief Adjust path boundary with consideration of buffer,
   *        update the center_line after determine the new path boundary,
   *        check whether blocking after adjustment
   *
   * @param idx: the index of path boundary's point
   * @param left_bound: obstacle's left bound
   * @param right_bound: obstacle's right bound
   * @param path_boundaries
   * @param center_line: the center of path bound
   * @return true: normal
   * @return false: there is no path boundaries or obstacle edges
   */
  bool UpdatePathBoundaryAndCenterLineWithBuffer(size_t idx, double left_bound,
                                                 double right_bound,
                                                 PathBound* path_boundaries,
                                                 double* center_line);

  /**
   * @brief Loose the path boundary with consideration of lateral accelarate.
   *      !!! But this function looks useless in this scope.
   *
   * @param idx
   * @param path_boundaries
   * @param left_bound: obstacle's left bound
   * @param right_bound: obstacle's right bound
   */
  void ProcessPathboundFromStartPoint(size_t idx,
                                      const PathBound* path_boundaries,
                                      double* left_bound, double* right_bound);

  /**
   * @brief Trim the path boundaries form the blocking place
   *
   * @param path_blocked_idx: the place blocked
   * @param path_boundaries
   */
  static void TrimPathBounds(int path_blocked_idx, PathBound* path_boundaries);

  /**
   * @brief Set the obstacle_edges info to dynamic_obstacle_constraint_debug and
   * generate relavent debug info
   *
   * @param obstacle_edges: a vector of sorted dynamic obstacles
   * (is_start_s,s,l_min,l_max,id)
   * @param obs_id: block obstacle id
   * @param origin_path_boundaries: tmp_path_boundaries(=path_boundaries)
   * @param path_boundaries
   */
  static void DebugInfo(const std::vector<ObstacleEdge>* obstacle_edges,
                        const std::string* obs_id,
                        const PathBound* origin_path_boundaries,
                        const PathBound* path_boundaries);

  /**
   * @brief add origin_path_bound info and path_bound info to
   * dynamic_obstacle_constraint_debug
   *
   * @param dynamic_obstacle_constraint_debug：message object
   * @param origin_path_bound: =path_bound
   * @param path_bound
   */
  static void PathBoundDebugInfo(
      DynamicObstacleConstraintDebug dynamic_obstacle_constraint_debug,
      const PathBound* origin_path_bound, const PathBound* path_bound);

  /**
   * @brief Big Car towing points process
   * 
   * @param path_boundaries 
   * @param towing_points 
   * @param obstacle_id 
   * @param cur_obs_edges: pair of lower s and upper s
   * @param is_bigcar_left_nudge 
   * @param delta_v: speed diffrence between adc and obstacle
   * @param dynamic_obs_expect_towing_l
   */
  void TowingPointsBigCarProcess(const std::pair<double, double>& cur_obs_edges,
                                 const std::string& obstacle_id,
                                 PathBound* path_boundaries,
                                 TowingPointsInfo* towing_points,
                                 bool is_bigcar_left_nudge, double delta_v,
                                 double dynamic_obs_expect_towing_l);

  /**
   * @brief !!Function not used!!
   * 
   * @param obstacle_edges 
   * @param big_car_idx 
   */
  void NudgeBigCarFilterProcess(std::vector<ObstacleEdge>* obstacle_edges,
                                int big_car_idx);

  /**
   * @brief !!Function not used!!
   * 
   * @param cur_obstacle_edge 
   * @param other_dynamic_obstacle_edge 
   * @param filter_length 
   * @return true 
   * @return false 
   */
  bool AnotherSideHasDynamicObstacle(
      const ObstacleEdge& cur_obstacle_edge,
      std::vector<ObstacleEdge>* other_dynamic_obstacle_edge,
      double filter_length);
  // std::pair<double, double> base_big_car_s_area);

  /**
   * @brief !!Function not used!!
   * 
   * @param obstacle_edge 
   * @return double 
   */
  double CalculateFilterLength(const ObstacleEdge& obstacle_edge);

  bool CheckIfNudgeObstacle(const Obstacle& obstacle);

  double ADC_buffer_ = 0.0;
  double extend_length_ = 5.0;
  bool is_avoid_big_car_ = false;
  common::SLPoint planning_start_point_sl_;
  DynamicBufferProcessor dynamic_obs_dynamic_buffer_calculate_;
  std::vector<std::string> big_car_ids_;
};
}  // namespace planning
}  // namespace TL
