/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_stage.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_stage.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>

#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator_factory.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

SpeedStage::SpeedStage(const SpeedStageConfig& config) : stage_config_(config) {
  if (config.has_speed_evaluator_config()) {
    evaluator_ = SpeedEvaluatorFactory::CreateSpeedEvaluator(
        config.speed_evaluator_config());
  }
}

bool SpeedStage::PreProcess(Frame* frame,
                            ReferenceLineInfo* reference_line_info) {
  UNUSED(frame);
  UNUSED(reference_line_info);
  return true;
}

bool SpeedStage::Process(const std::shared_ptr<DependencyInjector>& injector,
                         Frame* frame, ReferenceLineInfo* reference_line_info,
                         const common::TrajectoryPoint& init_point,
                         SpeedCache* const cache,
                         const std::shared_ptr<SpeedDataGenerator>& generator,
                         SpeedData* speed_data) {
  if (frame == nullptr || reference_line_info == nullptr || cache == nullptr ||
      generator == nullptr || speed_data == nullptr) {
    AERROR << "input is null, speed stage process failed";
    return false;
  }

  if (!generator->Process(injector, frame, reference_line_info, init_point,
                          cache, evaluator_, speed_data)) {
    AERROR << "speed data generate failed";
    return false;
  }

  return true;
}

bool SpeedStage::PostProcess(
    Frame* frame, ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const std::shared_ptr<SpeedDataGenerator>& generator,
    SpeedData* speed_data) {
  UNUSED(frame);
  UNUSED(reference_line_info);
  UNUSED(init_point);
  UNUSED(cache);
  UNUSED(generator);
  UNUSED(speed_data);
  return true;
}

bool SpeedStage::FinishStage(
    Frame* frame, ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const std::shared_ptr<SpeedDataGenerator>& generator,
    SpeedData* speed_data) {
  UNUSED(frame);
  UNUSED(reference_line_info);
  UNUSED(init_point);
  UNUSED(cache);
  UNUSED(generator);
  UNUSED(speed_data);
  return false;
}

void SpeedStage::CheckIfIgnoreBackObstacles(
    ReferenceLineInfo* reference_line_info) {
  UNUSED(reference_line_info);
}

}  // namespace planning
}  // namespace TL
