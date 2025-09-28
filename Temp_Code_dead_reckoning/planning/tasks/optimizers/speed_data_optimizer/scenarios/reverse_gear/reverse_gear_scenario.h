/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file reverse_gear_scenario.h
 **/

#pragma once

#include <memory>

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_scenario.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class ReverseGearScenario
 * @brief This class defines the reverse gear scenario
 */
class ReverseGearScenario : public SpeedScenario {
 public:
  explicit ReverseGearScenario(const SpeedScenarioConfig& config);

 private:
  bool PreProcess(const std::shared_ptr<DependencyInjector>& injector,
                  Frame* frame, ReferenceLineInfo* reference_line_info,
                  common::TrajectoryPoint* init_point,
                  SpeedCache* cache) override;

  bool PostProcess(Frame* frame, ReferenceLineInfo* reference_line_info,
                   const common::TrajectoryPoint& init_point, SpeedCache* cache,
                   const std::shared_ptr<SpeedDataGenerator>& generator,
                   SpeedData* speed_data) override;

  bool CheckIfKeepStandStill(Frame* frame,
                             const ReferenceLineInfo& reference_line_info,
                             SpeedCache* cache,
                             common::TrajectoryPoint* init_point) override;

  bool GenerateStandStillSpeedData(const common::TrajectoryPoint& init_point,
                                   SpeedData* speed_data) override;
};

}  // namespace planning
}  // namespace TL
