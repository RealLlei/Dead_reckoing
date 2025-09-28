/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_merge_stop_stage.h
 **/

#pragma once

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "planning/common/frame.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/forward_gear_speed_data_generator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_stage.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class LaneMergeStopStage
 * @brief This class defines the lane merge stop staged
 */
class LaneMergeStopStage : public SpeedStage {
 public:
  explicit LaneMergeStopStage(const SpeedStageConfig& config);

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
   * @param frame 
   * @param reference_line_info 
   * @param init_point 
   * @param cache 
   * @param generator 
   * @return true 
   * @return false 
   */
  static bool CheckEndMergeStop(
      Frame* frame, ReferenceLineInfo* reference_line_info,
      const common::TrajectoryPoint& init_point, const SpeedCache& cache,
      const std::shared_ptr<ForwardGearSpeedDataGenerator>& generator);

 private:
  LaneMergeStopStageConfig config_;
  static constexpr double kMergeStopFallbackSMin = 10.0;
  static constexpr double kMergeStopFallbackSMax = 60.0;
  static constexpr double kMergeStopFallbackA = -2.0;
};

}  // namespace planning
}  // namespace TL
