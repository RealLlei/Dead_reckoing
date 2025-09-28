/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_merge_scenario.h
 **/

#pragma once

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "planning/tasks/deciders/st_bounds_decider/slt_obstacles_processor.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/forward_gear_scenario.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class LaneMergeScenario
 * @brief This class defines the lane merge scenario
 */
class LaneMergeScenario : public ForwardGearScenario {
 public:
  explicit LaneMergeScenario(const SpeedScenarioConfig& config);

 private:
  LaneMergeScenarioConfig config_;
  SLTObstaclesProcessor slt_obstacles_processor_;
};

}  // namespace planning
}  // namespace TL
