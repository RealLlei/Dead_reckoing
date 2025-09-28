/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file reverse_gear_speed_data_generator.h
 **/

#pragma once

#include "planning/tasks/optimizers/speed_data_optimizer/generators/speed_data_generator.h"
#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class ReverseGearSpeedDataGenerator
 * @brief this class is used to generate speed_data in reverse gear scenario
 */
class ReverseGearSpeedDataGenerator : public SpeedDataGenerator {
 public:
  explicit ReverseGearSpeedDataGenerator(
      const SpeedDataGeneratorConfig& config);
};

}  // namespace planning
}  // namespace TL
