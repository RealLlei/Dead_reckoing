/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file forward_gear_speed_data_generator.h
 **/

#pragma once

#include <memory>

#include "planning/tasks/optimizers/speed_data_optimizer/generators/speed_data_generator.h"
#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class ForwardGearSpeedDataGenerator
 * @brief this class is used to generate speed_data in forward gear scenario
 */
class ForwardGearSpeedDataGenerator : public SpeedDataGenerator {
 public:
  explicit ForwardGearSpeedDataGenerator(
      const SpeedDataGeneratorConfig& config);

  /**
   * @brief ignore back obstacle, use vt sample generator to generate speed data
   * 
   * @param frame 
   * @param reference_line_info 
   * @param init_point 
   * @param cache 
  * @param evaluator 
   * @param speed_data 
   * @return true 
   * @return false 
   */
  virtual bool GeneratorSpeedDataWithoutBackObstacle(
      const Frame* frame, const ReferenceLineInfo* reference_line_info,
      const common::TrajectoryPoint& init_point, const SpeedCache& cache,
      const std::shared_ptr<SpeedEvaluator>& evaluator,
      SpeedData* speed_data) = 0;

  /**
   * @brief use vt sample generator to generate merge stop speed data
   * 
   * @param frame 
   * @param reference_line_info 
   * @param init_point 
   * @param cache 
   * @param evaluator 
   * @param speed_data 
   * @return true 
   * @return false 
   */
  virtual bool GeneratorMergeStopSpeedData(
      const Frame* frame, const ReferenceLineInfo* reference_line_info,
      const common::TrajectoryPoint& init_point, const SpeedCache& cache,
      const std::shared_ptr<SpeedEvaluator>& evaluator,
      SpeedData* speed_data) = 0;

  /**
   * @brief Get the Best Curve object
   * 
   * @return const std::shared_ptr<SpeedCurve>& 
   */
  const std::shared_ptr<SpeedCurve>& GetBestCurve() const {
    return best_curve_;
  }

  /**
   * @brief 
   * 
   * @return true 
   * @return false 
   */
  bool StartMergeStop() const { return start_merge_stop_; }

  /**
   * @brief Set the Start Merge Stop object
   * 
   * @param start_merge_stop 
   */
  void SetStartMergeStop(const bool start_merge_stop) {
    start_merge_stop_ = start_merge_stop;
  }

 public:
  std::shared_ptr<SpeedCurve> best_curve_ = nullptr;

 private:
  bool start_merge_stop_ = false;
};

}  // namespace planning
}  // namespace TL
