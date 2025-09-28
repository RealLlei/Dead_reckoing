/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_keep_stage.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_keep/lane_keep_stage.h"
#include <algorithm>
#include "map/hdmap/path.h"
#include "planning/localview/local_view.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "proto/fsm/nnp_fct.pb.h"

namespace TL {
namespace planning {

LaneKeepStage::LaneKeepStage(const SpeedStageConfig& config)
    : SpeedStage(config), config_(config.lane_keep_stage_config()) {}

bool LaneKeepStage::PreProcess(Frame* frame,
                               ReferenceLineInfo* reference_line_info) {
  if (frame == nullptr || reference_line_info == nullptr) {
    return false;
  }

  const auto is_avp_mode =
      frame->local_view().HasFunctionManagerIn() &&
      frame->local_view().GetFunctionManagerIn()->ta_pilot_mode() ==
          functionmanager::AVP;
  if (!is_avp_mode) {
    CheckIfIgnoreBackObstacles(reference_line_info);
  }

  return true;
}

void LaneKeepStage::CheckIfIgnoreBackObstacles(
    ReferenceLineInfo* const reference_line_info) {
  if (reference_line_info == nullptr || !config_.completely_ignore_back_obs()) {
    return;
  }
  const auto adc_front_s = reference_line_info->AdcSlBoundary().end_s();
  const auto adc_back_s = reference_line_info->AdcSlBoundary().start_s();
  const auto& junction_overlaps =
      reference_line_info->reference_line().map_path().junction_overlaps();
  const auto adc_in_junction =
      std::any_of(junction_overlaps.begin(), junction_overlaps.end(),
                  [&](const hdmap::PathOverlap& junction) {
                    return adc_front_s > (junction.start_s - 50) &&
                           adc_back_s < junction.end_s;
                  });
  if (adc_in_junction) {
    return;
  }
  auto* path_decision = reference_line_info->path_decision();
  if (path_decision == nullptr) {
    return;
  }
  ObjectDecisionType ignore_decision;
  ignore_decision.mutable_ignore();
  // 只忽略后向进图了的目标
  for (const auto* obs : path_decision->obstacles().Items()) {
    if (obs == nullptr || obs->IsVirtual() || obs->IsStatic() ||
        obs->Perception().type() ==
            perception::PerceptionObstacle::PEDESTRIAN ||
        obs->IsIgnore() || obs->PerceptionSLBoundary().end_s() > adc_front_s ||
        obs->path_st_boundary().IsEmpty() ||
        obs->GetPathSLTBoundary().IsEmpty()) {
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
