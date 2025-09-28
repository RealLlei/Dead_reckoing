/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_keep_stage.h
 **/

#pragma once

#include "planning/common/frame.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_stage.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class LaneKeepStage
 * @brief This class defines the lane keep stage
 */
class LaneKeepStage : public SpeedStage {
 public:
  explicit LaneKeepStage(const SpeedStageConfig& config);

  bool PreProcess(Frame* frame,
                  ReferenceLineInfo* reference_line_info) override;

 private:
  /**
  * @brief 
  * 
  * @param reference_line_info 
  */
  void CheckIfIgnoreBackObstacles(
      ReferenceLineInfo* reference_line_info) override;

 private:
  LaneKeepStageConfig config_;
};

}  // namespace planning
}  // namespace TL
