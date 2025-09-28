/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file reverse_gear_stage.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/reverse_gear/reverse_gear_stage.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

ReverseGearStage::ReverseGearStage(const SpeedStageConfig& config)
    : SpeedStage(config), config_(config.reverse_gear_stage_config()) {}

}  // namespace planning
}  // namespace TL
