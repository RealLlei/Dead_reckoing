/*
 * Copyright (c) 2022 TL Technologies Co., Ltd. All rights reserved.
 */

#pragma once
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "planning/common/reference_line_info.h"
#include "planning/tasks/deciders/obstacles_decider/obstacle_selector/obstacles_info_updater.h"
#include "planning/proto/planning_config.pb.h"

namespace TL {
namespace planning {
using BehaviorPair = std::pair<InteractiveObsBehavior::Behavior,
                               InteractiveObsBehavior::Behavior>;

class BehaviorCost {
 public:
  BehaviorCost() = default;

  // calculate cost of the interactive obs
  double CalculateCost(
      const ReferenceLineInfo& reference_line_info,
      const BehaviorPair& adc_obs_behavior,
      const InteractiveObsBehavior::Behavior& last_adc_behavior,
      const std::string& interactive_obs_id,
      const std::unordered_map<std::string, ObstacleInfo>& obstacles_info,
      const std::pair<std::string, std::string>& leading_obs_id,
      std::pair<bool, double> adc_merge_info);
  // only consider adc drive style
  static double GetDriveStyleCost();
  // traffic rule cost including right of way and dashed or solid line
  static double GetTrafficRuleCost(double adc_a, double obs_a,
                                   bool adc_in_dying_road);
  // safety cost
  double GetSafetyCost(
      const PathDecision& path_decision, double adc_s, double adc_v,
      double adc_a, double obs_a, const std::string& interactive_obs_id,
      const std::unordered_map<std::string, ObstacleInfo>& obstacles_info,
      const std::pair<std::string, std::string>& leading_obs_id,
      std::pair<bool, double> adc_merge_info);
  static bool JudgeLeadingObsCollision(int first_time_index, double first_v,
                                       double merge_point_s,
                                       const Obstacle* leading_obstacle);
  // {adc_s, adc_arrival_time, adc_v, adc_v_new, adc_a};
  static void LongitudinalSFunction(std::array<double, 5> obs_info,
                                    std::vector<double>* obs_s_vec);
  static bool JudgeFrontCollision(const std::vector<double>& first_one,
                                  const PathDecision& path_decision,
                                  const std::string& leading_obs_id);
  // get rear car and front car longitudinal safe distance
  static double GetRSSLgtSafeDistance(double v_r, double v_f);
  // get ego_speed upper limit
  static double GetRSSLgtSafeVelocity(double v_f, double lgt_distance_abs);
  // time of driving delta_s
  static double CalArrivalTime(double curr_v, double curr_a, double* new_v,
                               double delta_s, double curr_arrival_time);
  // comfort cost
  static double GetComfortCost(double adc_a, double obs_a);
  // efficiency cost
  static double GetEfficiencyCost(
      double adc_a, double time_gap, double obs_a,
      const ReferenceLineInfo& reference_line_info,
      const std::string& interactive_obs_id,
      const std::unordered_map<std::string, ObstacleInfo>& obstacles_info);
  // consistency cost
  static double GetConsistencyCost(
      const InteractiveObsBehavior::Behavior& adc_behavior,
      const InteractiveObsBehavior::Behavior& last_adc_behavior);

  bool GetAdcIsLatecomer() const { return adc_is_latecomer_; }

 private:
  bool adc_is_latecomer_ = true;
};

class BehaviorEvaluator {
 public:
  BehaviorEvaluator() = default;

  // evaluate adc and obs behavior combination via costs
  // @return true means yield and false means overtake
  bool Evaluate(
      ReferenceLineInfo* reference_line_info,
      const std::string& interactive_obs_id,
      const std::unordered_map<std::string, ObstacleInfo>& obstacles_info,
      const std::pair<std::string, std::string>& leading_obs_id,
      std::pair<bool, double> adc_merge_info, BehaviorPair* adc_obs_behavior,
      double* min_cost);

  // <cost, adc_is_latecomer>
  using CostInfo = std::pair<double, bool>;

  struct CostComparator {
    bool operator()(const std::pair<BehaviorPair, CostInfo>& left,
                    const std::pair<BehaviorPair, CostInfo>& right) const {
      return left.second.first < right.second.first;
    }
  };

 private:
  // last_cycle_adc_behavior
  InteractiveObsBehavior::Behavior last_adc_behavior_ =
      InteractiveObsBehavior::ConstantSpeed;
};

}  // namespace planning
}  // namespace TL
