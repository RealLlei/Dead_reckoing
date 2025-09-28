/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_change_prefinish_stage.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_change/lane_change_prefinish_stage.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

LaneChangePrefinishStage::LaneChangePrefinishStage(
    const SpeedStageConfig& config)
    : SpeedStage(config),
      config_(config.lane_change_prefinish_stage_config()) {}

bool LaneChangePrefinishStage::PreProcess(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (frame == nullptr || reference_line_info == nullptr) {
    AERROR << "input is error, LaneChangeScenario::PreProcess failed";
    return false;
  }

  CheckIfIgnoreBackObstacles(reference_line_info);

  return true;
}

void LaneChangePrefinishStage::CheckIfIgnoreBackObstacles(
    ReferenceLineInfo* const reference_line_info) {
  if (reference_line_info == nullptr) {
    return;
  }
  const auto adc_start_s = reference_line_info->AdcSlBoundary().start_s();
  auto* path_decision = reference_line_info->path_decision();
  if (path_decision == nullptr) {
    return;
  }
  ObjectDecisionType ignore_decision;
  ignore_decision.mutable_ignore();
  static constexpr double kObstacleSpeedCoefficient = 2.5;
  static constexpr double kEgoSpeedCoefficient = 2.0;
  static constexpr double kBackwardMinSafeDistanceOnSameDirection = 10.0;
  const double ego_v =
      std::fabs(reference_line_info->vehicle_state().linear_velocity());
  // 换道的时候如果没有slt，车屁股之前的不处理
  for (const auto* obs : path_decision->obstacles().Items()) {
    if (obs == nullptr || obs->IsVirtual() || obs->IsStatic() ||
        obs->IsIgnore() || obs->PerceptionSLBoundary().end_s() > adc_start_s ||
        obs->GetPathSLTBoundary().IsEmpty()) {
      continue;
    }
    const double backward_safe_distance =
        std::fmax(planning::util::LaneChangeBackWardSafeDistance(
                      kObstacleSpeedCoefficient, kEgoSpeedCoefficient,
                      obs->speed(), ego_v),
                  kBackwardMinSafeDistanceOnSameDirection);
    ADEBUG << " obs id : " << obs->Id()
           << " backward_safe_distance :" << backward_safe_distance
           << "real dis : "
           << adc_start_s - obs->PerceptionSLBoundary().end_s();
    if (adc_start_s - obs->PerceptionSLBoundary().end_s() <
        backward_safe_distance) {
      continue;
    }

    auto* obs_ptr = path_decision->Find(obs->Id());
    if (obs_ptr == nullptr) {
      continue;
    }
    obs_ptr->SetLongitudinalDecision(GetObsDecisionTag(), ignore_decision);
  }
}

}  // namespace planning
}  // namespace TL
