/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file reverse_gear_stage.h
 **/

#pragma once

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_stage.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class ReverseGearStage
 * @brief This class defines the reverse gear stage
 */
class ReverseGearStage : public SpeedStage {
 public:
  explicit ReverseGearStage(const SpeedStageConfig& config);

 private:
  ReverseGearStageConfig config_;
};

}  // namespace planning
}  // namespace TL
