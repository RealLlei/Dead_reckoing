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
 * @file speed_scenario_factory.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_scenario_factory.h"

#include <string>
#include <vector>

#include "common/file/file.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/util/util.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_change/lane_change_scenario.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_continue/lane_continue_scenario.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_keep/lane_keep_scenario.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_merge/lane_merge_scenario.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/reverse_gear/reverse_gear_scenario.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

TL::common::util::Factory<
    SpeedScenarioConfig::ScenarioType, SpeedScenario,
    SpeedScenario* (*)(const SpeedScenarioConfig& config),
    std::unordered_map<SpeedScenarioConfig::ScenarioType,
                       SpeedScenario* (*)(const SpeedScenarioConfig& config),
                       std::hash<int>>>
    SpeedScenarioFactory::speed_scenario_factory_;

std::unordered_map<SpeedScenarioConfig::ScenarioType, SpeedScenarioConfig,
                   std::hash<int>>
    SpeedScenarioFactory::default_speed_scenario_config_map_;

void SpeedScenarioFactory::Init() {
  // deciders
  speed_scenario_factory_.Register(
      SpeedScenarioConfig::LANE_KEEP_SCENARIO,
      [](const SpeedScenarioConfig& config) -> SpeedScenario* {
        return new LaneKeepScenario(config);
      });
  speed_scenario_factory_.Register(
      SpeedScenarioConfig::LANE_CHANGE_SCENARIO,
      [](const SpeedScenarioConfig& config) -> SpeedScenario* {
        return new LaneChangeScenario(config);
      });
  speed_scenario_factory_.Register(
      SpeedScenarioConfig::LANE_MERGE_SCENARIO,
      [](const SpeedScenarioConfig& config) -> SpeedScenario* {
        return new LaneMergeScenario(config);
      });
  speed_scenario_factory_.Register(
      SpeedScenarioConfig::LANE_CONTINUE_SCENARIO,
      [](const SpeedScenarioConfig& config) -> SpeedScenario* {
        return new LaneContinueScenario(config);
      });
  speed_scenario_factory_.Register(
      SpeedScenarioConfig::REVERSE_GEAR_SCENARIO,
      [](const SpeedScenarioConfig& config) -> SpeedScenario* {
        return new ReverseGearScenario(config);
      });

  boost::filesystem::path path(FLAGS_default_speed_scenario_config_dir);
  std::vector<std::string> files;
  TL::planning::util::GetFilesByPath(path, &files);
  for (const auto& file : files) {
    SpeedScenarioConfig default_speed_scenario_config;
    if (TL::common::GetProtoFromFile(file, &default_speed_scenario_config)) {
      default_speed_scenario_config_map_.emplace(
          default_speed_scenario_config.scenario_type(),
          default_speed_scenario_config);
    }
  }
}

std::unique_ptr<SpeedScenario> SpeedScenarioFactory::CreateSpeedScenario(
    const SpeedScenarioConfig& speed_scenario_config) {
  SpeedScenarioConfig merged_config;
  const auto iter = default_speed_scenario_config_map_.find(
      speed_scenario_config.scenario_type());
  if (iter != default_speed_scenario_config_map_.end()) {
    merged_config.CopyFrom(iter->second);
  }
  merged_config.MergeFrom(speed_scenario_config);
  return speed_scenario_factory_.CreateObject(
      speed_scenario_config.scenario_type(), merged_config);
}

}  // namespace planning
}  // namespace TL
