/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_continue_scenario.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_continue/lane_continue_scenario.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

LaneContinueScenario::LaneContinueScenario(const SpeedScenarioConfig& config)
    : ForwardGearScenario(config),
      config_(config.lane_continue_scenario_config()) {}

}  // namespace planning
}  // namespace TL
