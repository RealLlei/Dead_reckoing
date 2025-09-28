/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_change_scenario.h
 **/

#pragma once

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/forward_gear_scenario.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class LaneChangeScenario
 * @brief This class defines the lane change scenario
 */
class LaneChangeScenario : public ForwardGearScenario {
 public:
  explicit LaneChangeScenario(const SpeedScenarioConfig& config);

 private:
  LaneChangeScenarioConfig config_;
};

}  // namespace planning
}  // namespace TL
