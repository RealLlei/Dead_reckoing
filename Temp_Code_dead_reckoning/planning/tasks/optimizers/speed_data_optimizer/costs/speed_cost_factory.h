/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @file speed_cost_factory.h
 **/

#pragma once

#include <memory>
#include <unordered_map>

#include "common/util/factory.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost.h"
#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

class SpeedCostFactory {
 public:
  static void Init();
  static std::unique_ptr<SpeedCost> CreateSpeedCost(
      const SpeedCostConfig& speed_cost_config);

 private:
  static TL::common::util::Factory<
      SpeedCostConfig::SpeedCostType, SpeedCost,
      SpeedCost* (*)(const SpeedCostConfig& config),
      std::unordered_map<SpeedCostConfig::SpeedCostType,
                         SpeedCost* (*)(const SpeedCostConfig& config),
                         std::hash<int>>>
      speed_cost_factory_;
  static std::unordered_map<SpeedCostConfig::SpeedCostType, SpeedCostConfig,
                            std::hash<int>>
      default_speed_cost_config_map_;
};

}  // namespace planning
}  // namespace TL
