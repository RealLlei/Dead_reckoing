/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file forward_gear_speed_data_generator.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/generators/forward_gear_speed_data_generator.h"

namespace TL {
namespace planning {

ForwardGearSpeedDataGenerator::ForwardGearSpeedDataGenerator(
    const SpeedDataGeneratorConfig& config)
    : SpeedDataGenerator(config) {}

}  // namespace planning
}  // namespace TL
