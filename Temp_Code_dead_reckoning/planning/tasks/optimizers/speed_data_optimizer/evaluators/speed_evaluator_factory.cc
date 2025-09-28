/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file speed_Stage_factory.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator_factory.h"

#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/lane_change_ongoing_evaluator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/lane_change_prefinish_evaluator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/lane_continue_evaluator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/lane_keep_evaluator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/lane_merge_cruise_evaluator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/lane_merge_stop_evaluator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/reverse_gear_evaluator.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

TL::common::util::Factory<
    SpeedEvaluatorConfig::EvaluatorType, SpeedEvaluator,
    SpeedEvaluator* (*)(const SpeedEvaluatorConfig& config),
    std::unordered_map<SpeedEvaluatorConfig::EvaluatorType,
                       SpeedEvaluator* (*)(const SpeedEvaluatorConfig& config),
                       std::hash<int>>>
    SpeedEvaluatorFactory::speed_evaluator_factory_;

void SpeedEvaluatorFactory::Init() {
  // deciders
  speed_evaluator_factory_.Register(
      SpeedEvaluatorConfig::LANE_KEEP_EVALUATOR,
      [](const SpeedEvaluatorConfig& config) -> SpeedEvaluator* {
        return new LaneKeepEvaluator(config);
      });
  speed_evaluator_factory_.Register(
      SpeedEvaluatorConfig::LANE_CHANGE_ONGOING_EVALUATOR,
      [](const SpeedEvaluatorConfig& config) -> SpeedEvaluator* {
        return new LaneChangeOngoingEvaluator(config);
      });
  speed_evaluator_factory_.Register(
      SpeedEvaluatorConfig::LANE_CHANGE_PREFINISH_EVALUATOR,
      [](const SpeedEvaluatorConfig& config) -> SpeedEvaluator* {
        return new LaneChangePrefinishEvaluator(config);
      });
  speed_evaluator_factory_.Register(
      SpeedEvaluatorConfig::LANE_MERGE_CRUISE_EVALUATOR,
      [](const SpeedEvaluatorConfig& config) -> SpeedEvaluator* {
        return new LaneMergeCruiseEvaluator(config);
      });
  speed_evaluator_factory_.Register(
      SpeedEvaluatorConfig::LANE_MERGE_STOP_EVALUATOR,
      [](const SpeedEvaluatorConfig& config) -> SpeedEvaluator* {
        return new LaneMergeStopEvaluator(config);
      });
  speed_evaluator_factory_.Register(
      SpeedEvaluatorConfig::LANE_CONTINUE_EVALUATOR,
      [](const SpeedEvaluatorConfig& config) -> SpeedEvaluator* {
        return new LaneContinueEvaluator(config);
      });
  speed_evaluator_factory_.Register(
      SpeedEvaluatorConfig::REVERSE_GEAR_EVALUATOR,
      [](const SpeedEvaluatorConfig& config) -> SpeedEvaluator* {
        return new ReverseGearEvaluator(config);
      });
}

std::unique_ptr<SpeedEvaluator> SpeedEvaluatorFactory::CreateSpeedEvaluator(
    const SpeedEvaluatorConfig& speed_evaluator_config) {
  return speed_evaluator_factory_.CreateObject(
      speed_evaluator_config.evaluator_type(), speed_evaluator_config);
}

}  // namespace planning
}  // namespace TL
