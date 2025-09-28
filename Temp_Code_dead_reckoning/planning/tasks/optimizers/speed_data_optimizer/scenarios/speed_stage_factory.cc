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
 * @file speed_stage_factory.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_stage_factory.h"

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_change/lane_change_ongoing_stage.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_change/lane_change_prefinish_stage.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_continue/lane_continue_stage.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_keep/lane_keep_stage.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_merge/lane_merge_cruise_stage.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_merge/lane_merge_stop_stage.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/reverse_gear/reverse_gear_stage.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

TL::common::util::Factory<
    SpeedStageConfig::StageType, SpeedStage,
    SpeedStage* (*)(const SpeedStageConfig& config),
    std::unordered_map<SpeedStageConfig::StageType,
                       SpeedStage* (*)(const SpeedStageConfig& config),
                       std::hash<int>>>
    SpeedStageFactory::speed_stage_factory_;

void SpeedStageFactory::Init() {
  // deciders
  speed_stage_factory_.Register(
      SpeedStageConfig::LANE_KEEP_STAGE,
      [](const SpeedStageConfig& config) -> SpeedStage* {
        return new LaneKeepStage(config);
      });
  speed_stage_factory_.Register(
      SpeedStageConfig::LANE_CHANGE_ONGOING_STAGE,
      [](const SpeedStageConfig& config) -> SpeedStage* {
        return new LaneChangeOngoingStage(config);
      });
  speed_stage_factory_.Register(
      SpeedStageConfig::LANE_CHANGE_PREFINISH_STAGE,
      [](const SpeedStageConfig& config) -> SpeedStage* {
        return new LaneChangePrefinishStage(config);
      });

  speed_stage_factory_.Register(
      SpeedStageConfig::LANE_MERGE_CRUISE_STAGE,
      [](const SpeedStageConfig& config) -> SpeedStage* {
        return new LaneMergeCruiseStage(config);
      });
  speed_stage_factory_.Register(
      SpeedStageConfig::LANE_MERGE_STOP_STAGE,
      [](const SpeedStageConfig& config) -> SpeedStage* {
        return new LaneMergeStopStage(config);
      });
  speed_stage_factory_.Register(
      SpeedStageConfig::LANE_CONTINUE_STAGE,
      [](const SpeedStageConfig& config) -> SpeedStage* {
        return new LaneContinueStage(config);
      });
  speed_stage_factory_.Register(
      SpeedStageConfig::REVERSE_GEAR_STAGE,
      [](const SpeedStageConfig& config) -> SpeedStage* {
        return new ReverseGearStage(config);
      });
}

std::unique_ptr<SpeedStage> SpeedStageFactory::CreateSpeedStage(
    const SpeedStageConfig& speed_stage_config) {
  return speed_stage_factory_.CreateObject(speed_stage_config.stage_type(),
                                           speed_stage_config);
}

}  // namespace planning
}  // namespace TL
