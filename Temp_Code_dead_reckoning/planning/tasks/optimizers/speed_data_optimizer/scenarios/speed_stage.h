/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_stage.h
 **/

#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "planning/common/dependency_injector.h"
#include "planning/common/frame.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/util/common.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/speed_data_generator.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

using CandidateSpeedCurveCostResults =
    std::vector<std::pair<std::size_t, std::vector<SpeedCurveCostResult>*>>;

/**
 * @class SpeedStage
 * @brief This class defines the speed stage
 */
class SpeedStage {
 public:
  explicit SpeedStage(const SpeedStageConfig& config);
  virtual ~SpeedStage() = default;

  /**
   * @brief Pre process before speed plan
   * 
   * @param frame 
   * @param reference_line_info 
   * @return true 
   * @return false 
   */
  virtual bool PreProcess(Frame* frame, ReferenceLineInfo* reference_line_info);

  /**
   * @brief Main Process to generate speed data
   * 
   * @param injector 
   * @param frame current frame
   * @param reference_line_info current reference line info
   * @param init_point current planning start point
   * @param cache speed cache
   * @param generator speed data generator
   * @param speed_data output speed data
   * @return true 
   * @return false 
   */
  virtual bool Process(const std::shared_ptr<DependencyInjector>& injector,
                       Frame* frame, ReferenceLineInfo* reference_line_info,
                       const common::TrajectoryPoint& init_point,
                       SpeedCache* cache,
                       const std::shared_ptr<SpeedDataGenerator>& generator,
                       SpeedData* speed_data);

  /**
   * @brief Post process after speed plan
   * 
   * @param frame current frame
   * @param reference_line_info current reference line info
   * @param init_point current planning start point
   * @param cache speed cache
   * @param generator speed data generator 
   * @param speed_data output speed data 
   * @return true 
   * @return false 
   */
  virtual bool PostProcess(Frame* frame, ReferenceLineInfo* reference_line_info,
                           const common::TrajectoryPoint& init_point,
                           const SpeedCache& cache,
                           const std::shared_ptr<SpeedDataGenerator>& generator,
                           SpeedData* speed_data);

  /**
   * @brief Check if stage finish
   * 
   * @param frame current frame
   * @param reference_line_info current reference line info
   * @param init_point current planning start point
   * @param cache speed cache
   * @param generator speed data generator 
   * @param speed_data output speed data 
   * @return true 
   * @return false 
   */
  virtual bool FinishStage(Frame* frame, ReferenceLineInfo* reference_line_info,
                           const common::TrajectoryPoint& init_point,
                           const SpeedCache& cache,
                           const std::shared_ptr<SpeedDataGenerator>& generator,
                           SpeedData* speed_data);

  const SpeedStageConfig& GetStageConfig() const { return stage_config_; }

  const std::shared_ptr<SpeedEvaluator>& GetSpeedEvaluator() const {
    return evaluator_;
  }

  /**
   * @brief 
   * 
   * @return std::string 
   */
  std::string GetObsDecisionTag() const { return obs_decision_tag_; }

 private:
  /**
  * @brief 
  * 
  * @param reference_line_info 
  */
  virtual void CheckIfIgnoreBackObstacles(
      ReferenceLineInfo* reference_line_info);

 private:
  std::shared_ptr<SpeedEvaluator> evaluator_;
  SpeedStageConfig stage_config_;
  const std::string obs_decision_tag_ = "speed_stage";
};

}  // namespace planning
}  // namespace TL
