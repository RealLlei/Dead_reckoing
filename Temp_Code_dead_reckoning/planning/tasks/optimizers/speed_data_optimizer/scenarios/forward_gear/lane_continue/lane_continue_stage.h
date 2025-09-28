/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_continue_stage.h
 **/

#pragma once

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "planning/common/frame.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_stage.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class LaneContinueStage
 * @brief This class defines the lane continue stage
 */
class LaneContinueStage : public SpeedStage {
 public:
  explicit LaneContinueStage(const SpeedStageConfig& config);

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

  /**
   * @brief Get merge obstacle
   * 
   * @param reference_line_info 
   */
  static void GetMergeObstacles(ReferenceLineInfo* reference_line_info);

 private:
  LaneContinueStageConfig config_;
};

}  // namespace planning
}  // namespace TL
