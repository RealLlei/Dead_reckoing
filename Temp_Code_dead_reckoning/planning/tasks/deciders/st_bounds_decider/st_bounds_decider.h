/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "planning/common/frame.h"
#include "planning/tasks/deciders/decider.h"
#include "planning/tasks/deciders/st_bounds_decider/st_driving_limits.h"
#include "planning/tasks/deciders/st_bounds_decider/st_guide_line.h"
#include "planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"
#include "planning/tasks/deciders/st_bounds_decider/slt_obstacles_processor.h"
#include "planning/tasks/deciders/st_bounds_decider/obstacle_intention_processor.h"
#include "planning/proto/planning_config.pb.h"
#include "planning/proto/task_config.pb.h"
#include "planning/tasks/deciders/st_bounds_decider/obstacle_filter.h"

namespace TL {
namespace planning {

constexpr double kSTBoundsDeciderResolution = 0.1;
constexpr double kSTPassableThreshold = 3.0;

class STBoundsDecider : public Decider {
 public:
  STBoundsDecider(const TaskConfig& config,
                  const std::shared_ptr<DependencyInjector>& injector);

 private:
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

  void DebugForValidStBoundary();

  bool InitSTBoundsDecider(const Frame& frame,
                           ReferenceLineInfo* reference_line_info);

  //  common::Status GenerateFallbackSTBound(
  //      std::vector<std::tuple<double, double, double>>* const st_bound,
  //      std::vector<std::tuple<double, double, double>>* const vt_bound);

  common::Status GenerateRegularSTBound(
      std::vector<std::tuple<double, double, double>>* st_bound,
      std::vector<std::tuple<double, double, double>>* vt_bound,
      std::vector<std::pair<double, double>>* st_guide_line);

  static void RemoveInvalidDecisions(
      std::pair<double, double> driving_limit,
      std::vector<
          std::pair<std::tuple<double, double, double>,
                    std::vector<std::pair<std::string, ObjectDecisionType>>>>*
          available_choices);

  static bool CheckValidDecisions(
      std::pair<double, double> driving_limit,
      const std::vector<
          std::pair<std::tuple<double, double, double>,
                    std::vector<std::pair<std::string, ObjectDecisionType>>>>&
          available_choices);

  static void RankDecisions(
      double s_guide_line, std::pair<double, double> driving_limit,
      std::vector<
          std::pair<std::tuple<double, double, double>,
                    std::vector<std::pair<std::string, ObjectDecisionType>>>>*
          available_choices);

  static void RecordSTGraphDebug(
      const std::vector<STBoundary>& st_graph_data,
      const std::vector<std::tuple<double, double, double>>& st_bound,
      const std::vector<std::pair<double, double>>& st_guide_line,
      planning_internal::STGraphDebug* st_graph_debug);

  STBoundsDeciderConfig st_bounds_config_;
  STGuideLine st_guide_line_;
  STDrivingLimits st_driving_limits_;
  STObstaclesProcessor st_obstacles_processor_;
  SLTObstaclesProcessor slt_obstacles_processor_;
  ObstacleFilter obs_fillter_;
  ObstacleIntentionProcessor obstacle_intention_processor_;
};

}  // namespace planning
}  // namespace TL
