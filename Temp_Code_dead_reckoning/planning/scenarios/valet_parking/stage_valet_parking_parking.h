/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  stage_valet_parking_parking.h
 */

#pragma once

#include <memory>
#include <utility>

#include "planning/scenarios/stage.h"

#include "proto/fsm/avp_fct.pb.h"

namespace TL {
namespace planning {
namespace scenario {
namespace valet_parking {

class ValetParkingStageParking : public Stage {
 public:
  ValetParkingStageParking(const ScenarioConfig::StageConfig& config,
                           const std::shared_ptr<DependencyInjector>& injector)
      : Stage(config, injector) {}

 private:
  std::pair<Stage::StageStatus, common::Status> Process(
      const common::TrajectoryPoint& planning_init_point,
      Frame* frame) override;

  void SetParkingType(const functionmanager::AvpFctIn& avp_in);

  bool IsParkingBrakeCondition(const functionmanager::AvpFctIn& avp_in);

  bool IsReadyToFinishStage(Frame* frame);

  Stage::StageStatus FinishStage() { return Stage::FINISHED; }  // NOLINT

  ScenarioValetParkingConfig scenario_config_;
  bool is_stage_over_ = false;
};

}  // namespace valet_parking
}  // namespace scenario
}  // namespace planning
}  // namespace TL
