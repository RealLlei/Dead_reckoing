/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_continue_stage.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_continue/lane_continue_stage.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>

#include "common/file/log.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

LaneContinueStage::LaneContinueStage(const SpeedStageConfig& config)
    : SpeedStage(config), config_(config.lane_continue_stage_config()) {}

bool LaneContinueStage::PreProcess(Frame* frame,
                                   ReferenceLineInfo* reference_line_info) {
  if (frame == nullptr || reference_line_info == nullptr) {
    return false;
  }

  CheckIfIgnoreBackObstacles(reference_line_info);
  GetMergeObstacles(reference_line_info);

  return true;
}

void LaneContinueStage::CheckIfIgnoreBackObstacles(
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
        obs->IsIgnore() || obs->PerceptionSLBoundary().end_s() > adc_back_s ||
        obs->path_st_boundary().IsEmpty() ||
        obs->GetPathSLTBoundary().IsEmpty() || obs->GetHasIntention()) {
      continue;
    }

    auto* obs_ptr = path_decision->Find(obs->Id());
    if (obs_ptr == nullptr) {
      continue;
    }
    obs_ptr->SetLongitudinalDecision(GetObsDecisionTag(), ignore_decision);
  }
}

void LaneContinueStage::GetMergeObstacles(
    ReferenceLineInfo* reference_line_info) {

  const auto& adc_map_common_info =
      reference_line_info->reference_line().GetAdcMapCommonInfo();
  if (adc_map_common_info.first_merge_dir !=
          hdmap::MergeDirection::FROM_LEFT_MERGE &&
      adc_map_common_info.first_merge_dir !=
          hdmap::MergeDirection::FROM_RIGHT_MERGE) {
    return;
  }

  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle == nullptr || obstacle->IsVirtual() || obstacle->IsStatic()) {
      continue;
    }
    const auto& perception_sl_boundary = obstacle->PerceptionSLBoundary();
    if (obstacle->PerceptionSLBoundary().end_s() <
        reference_line_info->AdcSlBoundary().start_s()) {
      continue;
    }

    const auto center_l =
        (perception_sl_boundary.start_l() + perception_sl_boundary.end_l()) *
        0.5;
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    if (!reference_line_info->reference_line().GetLaneWidth(
            center_l, &lane_left_width, &lane_right_width)) {
      continue;
    }

    if ((adc_map_common_info.first_merge_dir ==
             hdmap::MergeDirection::FROM_LEFT_MERGE &&
         center_l < lane_left_width) ||
        (adc_map_common_info.first_merge_dir ==
             hdmap::MergeDirection::FROM_RIGHT_MERGE &&
         center_l > -lane_right_width)) {
      continue;
    }

    auto* mutable_obstacle =
        reference_line_info->path_decision()->Find(obstacle->Id());
    if (mutable_obstacle != nullptr) {
      mutable_obstacle->SetIsMergeObstacle(true);
      AERROR << "[merge_obstacle]id:" << mutable_obstacle->Id();
    }
  }
}

}  // namespace planning
}  // namespace TL
