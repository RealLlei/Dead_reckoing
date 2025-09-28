/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_continue_scenario.h
 **/

#pragma once

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/forward_gear_scenario.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class LaneContinueScenario
 * @brief This class defines the lane continue scenario
 */
class LaneContinueScenario : public ForwardGearScenario {
 public:
  explicit LaneContinueScenario(const SpeedScenarioConfig& config);

 private:
  LaneContinueScenarioConfig config_;
};

}  // namespace planning
}  // namespace TL
