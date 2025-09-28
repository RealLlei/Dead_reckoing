/*
 * Copyright (c) 2022 TL Technologies Co., Ltd. All rights reserved.
 */

#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "planning/tasks/deciders/decider.h"
#include "planning/tasks/deciders/obstacles_decider/conflict_resolution/behavior_conflict_resolution.h"
#include "planning/tasks/deciders/obstacles_decider/conflict_resolution/conflict_resolution.h"
#include "planning/tasks/deciders/obstacles_decider/evaluator/behavior_evaluator.h"
#include "planning/tasks/deciders/obstacles_decider/evaluator/trajectory_evaluator.h"
#include "planning/tasks/deciders/obstacles_decider/obstacle_selector/obstacle_selector.h"
#include "planning/tasks/deciders/obstacles_decider/obstacle_selector/obstacles_info_updater.h"
#include "planning/tasks/deciders/obstacles_decider/sampler/trajectory_generator.h"
#include "proto/planning/decider_debug.pb.h"

namespace TL::planning {
using InteractiveDecision = std::pair<
    std::string,
    std::pair<ObjectDecisionType, std::pair<InteractiveObsBehavior::Behavior,
                                            InteractiveObsBehavior::Behavior>>>;

class ObstaclesDecider : public Decider {
 public:
  ObstaclesDecider(const TaskConfig& config,
                   const std::shared_ptr<DependencyInjector>& injector);
  /**
   * @brief ComputeInitFrenetState.
   * @param matched_point
   * @param cartesian_state 
   * @param ptr_s 
   * @param ptr_d 
   */
  static void ComputeInitFrenetState(const common::PathPoint& matched_point,
                                     const TrajectoryPoint& cartesian_state,
                                     std::array<double, 3>* ptr_s,
                                     std::array<double, 3>* ptr_d);

 private:
  /**
   * @brief CutinProbEstimation.
   * @param decision_obstacles 
   * @param reference_line_info 
   * @param cutin_infos
   */
  void CutinProbEstimate(
      const std::unordered_set<std::string>& decision_obstacles,
      ReferenceLineInfo* reference_line_info,
      std::unordered_map<int, double>* cutin_infos);

  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;
  /*
   * @brief ToDiscretizedReferenceLine.
   * @param ref_points
   * @return vector<common::PathPoint>.
 */
  std::vector<common::PathPoint> ToDiscretizedReferenceLine(
      const std::vector<ReferencePoint>& ref_points) const;

  /**
   * @brief IgnoreObstacle.
   * @param reference_line_info
   */
  void SetIgnoreObstacle(ReferenceLineInfo* reference_line_info);

  // set adc longitudinal action including constant speed、accelerate、decelerate
  void MakeInteractiveObsDecision(ReferenceLineInfo* reference_line_info);
  // ignore obstacles in front of cipv
  void MakeIgnoreDecision(PathDecision* path_decision);
  common::Status MakeAvoidAlongsideDecision(
      Frame* frame, ReferenceLineInfo* reference_line_info);
  std::unique_ptr<TrajectoryGenerator> trajectory_generator_ = nullptr;
  std::unique_ptr<TrajectoryEvaluator> trajectory_evaluator_ = nullptr;
  std::unique_ptr<ConflictResolution> conflict_resolution_ = nullptr;
  std::unique_ptr<ObstacleSelector> obstacle_selector_ = nullptr;

  ObstaclesInfo obs_info_;
  ObstaclesDeciderDebug* mutable_obstacles_decider_debug_ = nullptr;

  BehaviorEvaluator behavior_evaluator_;
  BehaviorConflictResolution behavior_conflict_resolution_;

  void ObstaclesDeciderDebugInfo();

  bool ignore_obs_before_cipv_ = false;
  bool enable_debug_avoid_alongside_decision_ = false;
  bool enable_debug_obstacles_info_ = false;

  bool is_auto_mode_ = false;
  double start_time_avoid_alongside_ = std::numeric_limits<double>::max();
  CutinIntentionEstimation cutin_intention_estimation_;
};
}  // namespace TL::planning
