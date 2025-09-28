/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_data_generator.h
 **/

#pragma once

#include <memory>

#include "planning/common/dependency_injector.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"
#include "proto/planning/planning_internal.pb.h"

namespace TL {
namespace planning {

/**
 * @class VtSampleOptimizer
 * @brief this class is used to generate speed_data using vt sample method
 */
class SpeedDataGenerator {
 public:
  explicit SpeedDataGenerator(const SpeedDataGeneratorConfig& config);
  virtual ~SpeedDataGenerator() = default;

  /**
   * @brief use vt sample generator to generate speed data
   * 
   * @param injector 
   * @param frame 
   * @param reference_line_info 
   * @param init_point 
   * @param cache 
   * @param evaluator 
   * @param speed_data 
   * @return true 
   * @return false 
   */
  virtual bool Process(const std::shared_ptr<DependencyInjector>& injector,
                       Frame* frame, ReferenceLineInfo* reference_line_info,
                       const common::TrajectoryPoint& init_point,
                       SpeedCache* cache,
                       const std::shared_ptr<SpeedEvaluator>& evaluator,
                       SpeedData* speed_data) = 0;

  /**
   * @brief use vt sample generator to generate fallback speed data
   * 
   * @param reference_line_info 
   * @param init_point 
   * @param cache 
   * @param evaluator 
   * @param speed_data 
   * @return true 
   * @return false 
   */
  virtual bool GenerateFallbackSpeedData(
      const ReferenceLineInfo* reference_line_info,
      const common::TrajectoryPoint& init_point, const SpeedCache& cache,
      const std::shared_ptr<SpeedEvaluator>& evaluator,
      SpeedData* speed_data) = 0;
};

}  // namespace planning
}  // namespace TL
