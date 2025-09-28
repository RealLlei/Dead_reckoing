/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_change_ongoing_stage.h
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
 * @class LaneChangeOngoingStage
 * @brief This class defines the lane change ongoing stage
 */
class LaneChangeOngoingStage : public SpeedStage {
 public:
  explicit LaneChangeOngoingStage(const SpeedStageConfig& config);

  bool PreProcess(Frame* frame,
                  ReferenceLineInfo* reference_line_info) override;

  bool PostProcess(Frame* frame, ReferenceLineInfo* reference_line_info,
                   const common::TrajectoryPoint& init_point,
                   const SpeedCache& cache,
                   const std::shared_ptr<SpeedDataGenerator>& generator,
                   SpeedData* speed_data) override;

  bool FinishStage(Frame* frame, ReferenceLineInfo* reference_line_info,
                   const common::TrajectoryPoint& init_point,
                   const SpeedCache& cache,
                   const std::shared_ptr<SpeedDataGenerator>& generator,
                   SpeedData* speed_data) override;

 private:
  /**
  * @brief 
  * 
  * @param reference_line_info 
  */
  void CheckIfIgnoreBackObstacles(
      ReferenceLineInfo* reference_line_info) override;

 private:
  LaneChangeOngoingStageConfig config_;
};

}  // namespace planning
}  // namespace TL
