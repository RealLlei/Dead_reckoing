/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_merge_scenario.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_merge/lane_merge_scenario.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>

#include "common/file/log.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

LaneMergeScenario::LaneMergeScenario(const SpeedScenarioConfig& config)
    : ForwardGearScenario(config),
      config_(config.lane_merge_scenario_config()) {}

}  // namespace planning
}  // namespace TL
