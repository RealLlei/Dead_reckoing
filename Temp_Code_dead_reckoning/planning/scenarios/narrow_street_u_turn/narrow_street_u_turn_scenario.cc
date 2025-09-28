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
#include "planning/scenarios/narrow_street_u_turn/narrow_street_u_turn_scenario.h"

namespace TL {
namespace planning {
namespace scenario {
namespace narrow_street_u_turn {

std::unique_ptr<Stage> NarrowStreetUTurnScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  UNUSED(stage_config);
  UNUSED(injector);
  return nullptr;
}

}  // namespace narrow_street_u_turn
}  // namespace scenario
}  // namespace planning
}  // namespace TL
