/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  valet_parking_scenario.h
 */

#pragma once

#include <memory>

#include "common/util/factory.h"
#include "planning/scenarios/scenario.h"

namespace TL {
namespace planning {
namespace scenario {
namespace valet_parking {

struct ValetParkingContext {
  ScenarioValetParkingConfig scenario_config;
};

class ValetParkingScenario : public Scenario {
 public:
  ValetParkingScenario(const ScenarioConfig& config,
                       const ScenarioContext* context,
                       const std::shared_ptr<DependencyInjector>& injector)
      : Scenario(config, context, injector) {}

  void Init() override;

  std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config,
      const std::shared_ptr<DependencyInjector>& injector) override;

  ValetParkingContext* GetContext() { return &context_; }

  void UpdateRegistry(const ScenarioStatus::StageType& last_stage_type);

 private:
  static void RegisterStages();

  bool GetScenarioConfig();

  static TL::common::util::Factory<
      ScenarioStatus::StageType, Stage,
      Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
                 const std::shared_ptr<DependencyInjector>& injector)>
      s_stage_factory_;

  bool init_ = false;
  ValetParkingContext context_;
  ScenarioStatus::StageType last_stage_type_ = ScenarioStatus::NO_STAGE;
};

}  // namespace valet_parking
}  // namespace scenario
}  // namespace planning
}  // namespace TL
