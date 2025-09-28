/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  valet_parking_scenario.cc
 */

#include "planning/scenarios/valet_parking/valet_parking_scenario.h"

#include "common/file/log.h"
#include "planning/scenarios/valet_parking/stage_valet_parking_cruise.h"
#include "planning/scenarios/valet_parking/stage_valet_parking_parking.h"

namespace TL {
namespace planning {
namespace scenario {
namespace valet_parking {

TL::common::util::Factory<
    ScenarioStatus::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
               const std::shared_ptr<DependencyInjector>& injector)>
    ValetParkingScenario::s_stage_factory_;

void ValetParkingScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();
  if (!GetScenarioConfig()) {
    AERROR << "fail to get scenario specific config";
    return;
  }

  init_ = true;
}

void ValetParkingScenario::RegisterStages() {
  if (!s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      ScenarioStatus::VALET_PARKING_CRUISE,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new ValetParkingStageCruise(config, injector);
      });
  s_stage_factory_.Register(
      ScenarioStatus::VALET_PARKING_PARKING,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new ValetParkingStageParking(config, injector);
      });
}

std::unique_ptr<Stage> ValetParkingScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (s_stage_factory_.Empty()) {
    RegisterStages();
  }

  UpdateRegistry(last_stage_type_);
  auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(),
                                                 stage_config, injector);
  if (ptr) {
    ptr->SetContext(&context_);
    last_stage_type_ = stage_config.stage_type();
  }
  return ptr;
}

bool ValetParkingScenario::GetScenarioConfig() {
  if (!config_.has_valet_parking_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.valet_parking_config());
  return true;
}

void ValetParkingScenario::UpdateRegistry(
    const ScenarioStatus::StageType& last_stage_type) {
  UNUSED(last_stage_type);
  switch (last_stage_type_) {
    case ScenarioStatus::VALET_PARKING_CRUISE:
      s_stage_factory_.Unregister(ScenarioStatus::VALET_PARKING_CRUISE);
      s_stage_factory_.Register(
          ScenarioStatus::VALET_PARKING_CRUISE,
          [](const ScenarioConfig::StageConfig& config,
             const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
            return new ValetParkingStageCruise(config, injector);
          });
      break;
    case ScenarioStatus::VALET_PARKING_PARKING:
      s_stage_factory_.Unregister(ScenarioStatus::VALET_PARKING_PARKING);
      s_stage_factory_.Register(
          ScenarioStatus::VALET_PARKING_PARKING,
          [](const ScenarioConfig::StageConfig& config,
             const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
            return new ValetParkingStageParking(config, injector);
          });
      break;
    default:
      break;
  }
}

}  // namespace valet_parking
}  // namespace scenario
}  // namespace planning
}  // namespace TL
