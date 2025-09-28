/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_change_scenario.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_change/lane_change_scenario.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

LaneChangeScenario::LaneChangeScenario(const SpeedScenarioConfig& config)
    : ForwardGearScenario(config),
      config_(config.lane_change_scenario_config()) {}

}  // namespace planning
}  // namespace TL
