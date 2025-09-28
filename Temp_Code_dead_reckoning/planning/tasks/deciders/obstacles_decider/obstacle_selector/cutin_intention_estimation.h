/*
 * Copyright (c) 2023 TL Technologies Co., Ltd. All rights reserved.
 */

#pragma once

#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "planning/common/obstacle.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/reference_line_info.h"
#include "planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {
class CutinIntentionEstimation {
 public:
  CutinIntentionEstimation() = default;
  ~CutinIntentionEstimation() = default;
  /**
   * @brief GetDiscretizedPath.
   * @param cutin_path 
   * @param regression_path 
   * @param reference_line 
   * @param obstacle 
   * @param cutin_discretized_path 
   * @param regression_discretized_path 
   * @return true: calculate success.
   */
  bool GetDiscretizedPath(const QuinticPolynomialCurve1d& cutin_path,
                          const QuinticPolynomialCurve1d& regression_path,
                          const ReferenceLine& reference_line,
                          const Obstacle& obstacle,
                          DiscretizedPath* cutin_discretized_path,
                          DiscretizedPath* regression_discretized_path);

  /**
   * @brief CutinIntentionDebug.
   * @param history_cutin_path 
   * @param history_regression_path 
   * @param obstacle_id 
   * @param prob 
   * @param cutin_prior_prob 
   * @param cutin_likelihood 
   * @param regression_likelihood 
   * @param reference_line_info 
   */
  static void CutinIntentionDebug(
      const DiscretizedPath& history_cutin_path,
      const DiscretizedPath& history_regression_path,
      const std::string& obstacle_id, double prob, double cutin_prior_prob,
      double cutin_likelihood, double regression_likelihood,
      ReferenceLineInfo* reference_line_info);

  /**
   * @brief NaiveBayesProcessObstacle.
   * @param consider_obstacle 
   * @param obstacle_cutin_intentions 
   * @param reference_line_info 
   */
  void NaiveBayesProcessObstacle(
      const Obstacle& consider_obstacle,
      std::unordered_map<int, double>* obstacle_cutin_intentions,
      ReferenceLineInfo* reference_line_info);

  /**
   * @brief NaiveBayesCutinProbEstimation.
   * @param consider_obstacles 
   * @param reference_line_info
   * @return frist is obstacle id, second is cutin prob.
   */
  std::unordered_map<int, double> NaiveBayesCutinProbEstimate(
      const std::unordered_set<int>& consider_obstacles,
      ReferenceLineInfo* reference_line_info);

  /**
   * @brief CalculateGoalPoint.
   * @param reference_line_info 
   * @param init_state
   * @param consider_obstacle 
   * @param cutin_point
   * @param regression_point
   * @param get_map_info_time
   * @return true: calculate success.
   */
  static bool CalculateGoalPoint(
      const ReferenceLineInfo& reference_line_info,
      const std::pair<std::array<double, 3UL>, std::array<double, 3UL>>&
          init_state,
      const Obstacle& consider_obstacle, common::SLPoint* cutin_point,
      common::SLPoint* regression_point, double* get_map_info_time);

  /**
   * @brief CalculateCutinIntention.
   * @param cutin_prior_prob 
   * @param cutin_likelihood
   * @param regression_likelihood 
   * @return cutin prob.
   */
  static double CalculateCutinIntention(double cutin_prior_prob,
                                        double cutin_likelihood,
                                        double regression_likelihood);

  /**
   * @brief CalculatePriorProbability.
   * @param reference_line_info 
   * @param obstacle
   * @return cutin prior prob.
   */
  static double CalculatePriorProbability(
      const ReferenceLineInfo& reference_line_info, const Obstacle& obstacle);

  /**
   * @brief SampleIntentionPath.
   * @param init_state 
   * @param cutin_point
   * @param regression_point 
   * @param cutin_path
   * @param regression_path
   * @return true: calculate success.
   */
  static bool SampleIntentionPath(
      const std::pair<std::array<double, 3UL>, std::array<double, 3UL>>&
          init_state,
      const common::SLPoint& cutin_point,
      const common::SLPoint& regression_point,
      QuinticPolynomialCurve1d* cutin_path,
      QuinticPolynomialCurve1d* regression_path);

  /**
   * @brief IsLeftOrRightObstacle.
   * @param consider_obstacle 
   * @param is_left_obstacle
   * @param is_right_obstacle 
   * @return true: calculate success.
   */
  static bool IsLeftOrRightObstacle(const Obstacle& consider_obstacle,
                                    bool* is_left_obstacle,
                                    bool* is_right_obstacle);

  /**
   * @brief CalculateSampleStartPoint.
   * @param reference_line 
   * @param consider_obstacle 
   * @return true: calculate success.
   */
  bool CalculateSampleStartPoint(const ReferenceLine& reference_line,
                                 const Obstacle& consider_obstacle);

  /**
   * @brief CalculateInitState.
   * @param reference_line_info 
   * @param consider_obstacle 
   * @param init_state 
   * @return true: calculate success.
   */
  bool CalculateInitState(
      const ReferenceLineInfo& reference_line_info,
      const Obstacle& consider_obstacle,
      std::pair<std::array<double, 3UL>, std::array<double, 3UL>>* init_state);

  /**
   * @brief CalculateLikelihood.
   * @param reference_line 
   * @param history_path_data 
   * @param obstacle 
   * @param is_cutin_path 
   * @return likelihood.
   */
  double CalculateLikelihood(const ReferenceLine& reference_line,
                             const DiscretizedPath& history_path_data,
                             const Obstacle& obstacle, bool is_cutin_path);

  /**
   * @brief Debug.
   * @param path_data 
   * @param path_label 
   * @param reference_line_info 
   */
  static void Debug(const DiscretizedPath& path_data,
                    const std::string& path_label,
                    ReferenceLineInfo* reference_line_info);

  /**
   * @brief NormalDistributionCDF.
   * @param x 
   * @param mean 
   * @param standard_deviation 
   * @return prob.
   */
  static double NormalDistributionCDF(double x, double mean,
                                      double standard_deviation);

 private:
  common::TrajectoryPoint sample_start_point_;
  std::unordered_map<
      int, std::pair<std::queue<DiscretizedPath>, std::queue<DiscretizedPath>>>
      history_path_queue_;
};

}  // namespace planning
}  // namespace TL
