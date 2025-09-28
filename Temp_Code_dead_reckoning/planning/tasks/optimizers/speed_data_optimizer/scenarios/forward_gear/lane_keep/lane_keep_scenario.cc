/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_keep_scenario.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_keep/lane_keep_scenario.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

LaneKeepScenario::LaneKeepScenario(const SpeedScenarioConfig& config)
    : ForwardGearScenario(config),
      config_(config.lane_keep_scenario_config()) {}

}  // namespace planning
}  // namespace TL
