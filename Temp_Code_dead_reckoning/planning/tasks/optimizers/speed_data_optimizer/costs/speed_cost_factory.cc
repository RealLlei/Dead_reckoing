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
 * @file speed_cost_factory.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost_factory.h"

#include "common/file/file.h"
#include "planning/common/planning_gflags.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/accel_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/collision_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/curve_priority_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/gap_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/jerk_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/low_speed_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/nudge_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/obstacle_expected_distance_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/obstacle_safe_distance_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/over_speed_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/stop_distance_cost.h"

namespace TL {
namespace planning {

TL::common::util::Factory<
    SpeedCostConfig::SpeedCostType, SpeedCost,
    SpeedCost* (*)(const SpeedCostConfig& config),
    std::unordered_map<SpeedCostConfig::SpeedCostType,
                       SpeedCost* (*)(const SpeedCostConfig& config),
                       std::hash<int>>>
    SpeedCostFactory::speed_cost_factory_;

std::unordered_map<SpeedCostConfig::SpeedCostType, SpeedCostConfig,
                   std::hash<int>>
    SpeedCostFactory::default_speed_cost_config_map_;

void SpeedCostFactory::Init() {
  // deciders
  speed_cost_factory_.Register(SpeedCostConfig::ACCEL_COST,
                               [](const SpeedCostConfig& config) -> SpeedCost* {
                                 return new AccelCost(config);
                               });
  speed_cost_factory_.Register(SpeedCostConfig::CURVE_PRIORITY_COST,
                               [](const SpeedCostConfig& config) -> SpeedCost* {
                                 return new CurvePriorityCost(config);
                               });
  speed_cost_factory_.Register(SpeedCostConfig::GAP_COST,
                               [](const SpeedCostConfig& config) -> SpeedCost* {
                                 return new GapCost(config);
                               });
  speed_cost_factory_.Register(SpeedCostConfig::JERK_COST,
                               [](const SpeedCostConfig& config) -> SpeedCost* {
                                 return new JerkCost(config);
                               });
  speed_cost_factory_.Register(SpeedCostConfig::LOW_SPEED_COST,
                               [](const SpeedCostConfig& config) -> SpeedCost* {
                                 return new LowSpeedCost(config);
                               });
  speed_cost_factory_.Register(
      SpeedCostConfig::OBSTACLE_EXPECTED_DISTANCE_COST,
      [](const SpeedCostConfig& config) -> SpeedCost* {
        return new ObstacleExpectedDistanceCost(config);
      });
  speed_cost_factory_.Register(SpeedCostConfig::OBSTACLE_SAFE_DISTANCE_COST,
                               [](const SpeedCostConfig& config) -> SpeedCost* {
                                 return new ObstacleSafeDistanceCost(config);
                               });
  speed_cost_factory_.Register(SpeedCostConfig::OVER_SPEED_COST,
                               [](const SpeedCostConfig& config) -> SpeedCost* {
                                 return new OverSpeedCost(config);
                               });
  speed_cost_factory_.Register(SpeedCostConfig::NUDGE_COST,
                               [](const SpeedCostConfig& config) -> SpeedCost* {
                                 return new NudgeCost(config);
                               });
  speed_cost_factory_.Register(SpeedCostConfig::STOP_DISTANCE_COST,
                               [](const SpeedCostConfig& config) -> SpeedCost* {
                                 return new StopDistanceCost(config);
                               });
  speed_cost_factory_.Register(SpeedCostConfig::COLLISION_COST,
                               [](const SpeedCostConfig& config) -> SpeedCost* {
                                 return new CollisionCost(config);
                               });

  DefaultSpeedCostConfig default_speed_cost_configs;
  if (TL::common::GetProtoFromFile(fLS::FLAGS_default_speed_cost_config_file,
                                      &default_speed_cost_configs)) {
    for (const auto& speed_cost_config :
         default_speed_cost_configs.speed_cost_config()) {
      default_speed_cost_config_map_.emplace(
          speed_cost_config.speed_cost_type(), speed_cost_config);
    }
  }
}

std::unique_ptr<SpeedCost> SpeedCostFactory::CreateSpeedCost(
    const SpeedCostConfig& speed_cost_config) {
  SpeedCostConfig merged_config;
  const auto iter =
      default_speed_cost_config_map_.find(speed_cost_config.speed_cost_type());
  if (iter != default_speed_cost_config_map_.end()) {
    merged_config.CopyFrom(iter->second);
  }
  merged_config.MergeFrom(speed_cost_config);
  return speed_cost_factory_.CreateObject(speed_cost_config.speed_cost_type(),
                                          merged_config);
}

}  // namespace planning
}  // namespace TL
