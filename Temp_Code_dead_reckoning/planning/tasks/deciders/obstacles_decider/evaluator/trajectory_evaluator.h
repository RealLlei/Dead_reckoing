/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  trajectory_evaluator.h
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "planning/common/reference_line_info.h"
#include "planning/common/trajectory/discretized_trajectory.h"
#include "planning/proto/planning_config.pb.h"

namespace TL {
namespace planning {

class TrajectoryEvaluator {
  // normal use
  using TrajectoryCost =
      std::pair<DiscretizedTrajectory, std::array<double, 8>>;

 public:
  explicit TrajectoryEvaluator(ObstaclesDeciderConfig config);
  virtual ~TrajectoryEvaluator() = default;

  /**
   * @brief MakeObstacleDecision.
   * @param discretized_trajectories 
   * @param decision_obstacles  
   * @param dangerous_obstacles
   * @param nearest_cutin_obs_id  
   * @param follow_obs_id
   * @param reference_line_info
   */
  void MakeObstacleDecision(
      const std::vector<DiscretizedTrajectory>& discretized_trajectories,
      const std::unordered_set<std::string>& decision_obstacles,
      const std::unordered_set<std::string>& dangerous_obstacles,
      const std::string& nearest_cutin_obs_id, const std::string& follow_obs_id,
      const std::unordered_map<int, double>& cutin_estimation_infos,
      ReferenceLineInfo* reference_line_info);

  /**
   * @brief CalculateVirtualWallS.
   * @param nearest_cutin_obs_id 
   * @param follow_obs_id 
   * @param reference_line_info 
   */
  static double CalculateVirtualWallS(
      const std::string& nearest_cutin_obs_id, const std::string& follow_obs_id,
      const ReferenceLineInfo& reference_line_info);
  /**
   * @brief PruningStrategy.
   * @param discretized_trajectory 
   * @param decision_obstacle 
   * @param virtual_wall_s 
   * @param reference_line_info 
   */
  bool PruningStrategy(const DiscretizedTrajectory& discretized_trajectory,
                       const Obstacle& decision_obstacle,
                       const double& virtual_wall_s,
                       const ReferenceLineInfo& reference_line_info);

  /**
   * @brief DebugAvoidAlongsideDecision.
   * @param reference_line_info 
   */
  void DebugAvoidAlongsideDecision(ReferenceLineInfo* reference_line_info);

 private:
  /**
   * @brief ClearCache.
   * @param reference_line_info 
   */
  void ClearCache(const ReferenceLineInfo& reference_line_info);
  /**
   * @brief LateralStableCost.
   * @param trajectory 
   * @param perception_obs_id  
   * @return LateralStableCost
   */
  double LateralStableCost(const DiscretizedTrajectory& trajectory,
                           const int32_t& perception_obs_id);

  /**
   * @brief LongitudinalStableCost.
   * @param trajectory 
   * @param perception_obs_id  
   * @return LongitudinalStableCost
   */
  double LongitudinalStableCost(const DiscretizedTrajectory& trajectory,
                                const int32_t& perception_obs_id);

  /**
   * @brief CalculateInteractionInfo.
   * @param reference_line_info 
   * @param trajectory  
   * @param decision_obstacle_id
   * @param dangerous_obstacles
   * @param parallel_time
   * @param obs_adc_min_distance
   * @param is_collision_trajectory
   */
  void CalculateInteractionInfo(
      const ReferenceLineInfo& reference_line_info,
      const DiscretizedTrajectory& trajectory,
      const std::string& decision_obstacle_id,
      const std::set<std::string>& dangerous_obstacles, double* parallel_time,
      double* obs_adc_min_distance, bool* is_collision_trajectory) const;

  /**
   * @brief CalculateObsAdcMinDistance.
   * @param reference_line_info 
   * @param trajectory  
   * @param dangerous_obstacles
   * @param obs_adc_min_distance
   * @return false: collision 
   */
  static bool CalculateObsAdcMinDistance(
      const ReferenceLineInfo& reference_line_info,
      const DiscretizedTrajectory& trajectory,
      const std::unordered_set<std::string>& dangerous_obstacles,
      double* obs_adc_min_distance);
  /**
   * @brief CalculateDistanceFilter.
   * @param reference_line_info 
   * @param adc_sl_boundary  
   * @param obs_sl_boundary
   * @param trajectory_end_s
   * @param s_distance
   * @return true: filter 
   */
  static bool CalculateDistanceFilter(
      const ReferenceLineInfo& reference_line_info,
      const SLBoundary& adc_sl_boundary, const SLBoundary& obs_sl_boundary,
      double relative_time, double trajectory_end_s, double s_distance);
  /**
   * @brief CalculateObsAdcSlDistance.
   * @param adc_sl_boundary 
   * @param obs_sl_boundary  
   * @param s_distance
   * @param l_distance
   */
  static bool CalculateObsAdcSlDistance(const SLBoundary& adc_sl_boundary,
                                        const SLBoundary& obs_sl_boundary,
                                        double* s_distance, double* l_distance);
  /**
   * @brief CalculateObsAdcSlBoundary.
   * @param adc_trajectory_point 
   * @param trajectory_envelopes  
   * @param adc_sl_boundary
   * @param obs_sl_boundary
   */
  static bool CalculateObsAdcSlBoundary(
      const TrajectoryPoint& adc_trajectory_point,
      const std::vector<ObsPointDescription>& trajectory_envelopes,
      SLBoundary* adc_sl_boundary, SLBoundary* obs_sl_boundary);

  /**
   * @brief CalculateParallelTime.
   * @param reference_line_info 
   * @param trajectory  
   * @param decision_obstacle_id
   * @return parallel time 
   */
  static double CalculateParallelTime(
      const ReferenceLineInfo& reference_line_info,
      const DiscretizedTrajectory& trajectory,
      const std::string& decision_obstacle_id);

  /**
   * @brief cost计算.
   * @param reference_line_info 
   * @param trajectory  
   * @param dangerous_obstacles
   * @param is_collision_trajectory
   * @return cost值.
   */
  std::array<double, 8> Evaluate(
      const ReferenceLineInfo& reference_line_info,
      const DiscretizedTrajectory& trajectory,
      const std::unordered_set<std::string>& dangerous_obstacles,
      bool* is_collision_trajectory) const;

  /**
   * @brief CalculateCostMapNotIncludeInteraction.
   * @param discretized_trajectories 
   * @param dangerous_obstacles
   * @param reference_line_info
   */
  void CalculateCostMapNotIncludeInteraction(
      const std::vector<DiscretizedTrajectory>& discretized_trajectories,
      const std::unordered_set<std::string>& dangerous_obstacles,
      const ReferenceLineInfo& reference_line_info);

  /**
   * @brief DoDecision.
   * @param decision_obstacle 
   * @param reference_line_info
   */
  void DoDecision(const Obstacle& decision_obstacle,
                  ReferenceLineInfo* reference_line_info);

  /**
   * @brief CalculateInteractionCost.
   * @param discretized_trajectories 
   * @param decision_obstacle
   * @param nearest_cutin_obs_id 
   * @param follow_obs_id
   * @param reference_line_info
   */
  void CalculateInteractionCost(
      const std::vector<DiscretizedTrajectory>& discretized_trajectories,
      const Obstacle& decision_obstacle,
      const std::string& nearest_cutin_obs_id, const std::string& follow_obs_id,
      const ReferenceLineInfo& reference_line_info);

  /**
   * @brief SaveDebugInfo.
   * @param debug_obs_id 
   */
  void SaveDebugInfo(const std::string& debug_obs_id);

  /**
   * @brief LatOffsetCost.
   * @param trajectory 
   * @return cost值.
   */
  double LatOffsetCost(const DiscretizedTrajectory& trajectory) const;

  /**
   * @brief LonComfortCost.
   * @param trajectory 
   * @return cost值.
   */
  double LonComfortCost(const DiscretizedTrajectory& trajectory) const;

  /**
   * @brief EfficiencyCost.
   * @param trajectory 
   * @param ref_speed  
   * @return cost值.
   */
  double EfficiencyCost(const DiscretizedTrajectory& trajectory,
                        double ref_speed) const;

  /**
   * @brief ParallelTimeCost.
   * @param parallel_time 
   * @return cost值.
   */
  double ParallelTimeCost(double parallel_time) const;

  /**
   * @brief SafetyCost.
   * @param adc_obs_min_distance 
   * @return cost值.
   */
  double SafetyCost(double adc_obs_min_distance) const;

  struct CostComparator {
    bool operator()(const TrajectoryCost& left,
                    const TrajectoryCost& right) const {
      return left.second.back() > right.second.back();
    }
  };

  std::priority_queue<TrajectoryCost, std::vector<TrajectoryCost>,
                      CostComparator>
      cost_queue_;

  ObstaclesDeciderConfig config_;
  std::unordered_map<std::string, TrajectoryCost> debug_info_;
  std::unordered_map<size_t, std::array<double, 8>> cost_map_;
  std::unordered_map<std::string, std::pair<int32_t, DiscretizedTrajectory>>
      last_frame_decision_trajectory_;

  std::unordered_map<int, double> cutin_estimation_infos_;
};

}  // namespace planning
}  // namespace TL
