/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_scenario_manager.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_scenario_manager.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>

#include "common/file/log.h"
#include "map/hdmap/hdmap_util.h"
#include "map/hdmap/path.h"
#include "planning/common/path/discretized_path.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_scenario.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_scenario_factory.h"
#include "planning/proto/speed_evaluator_config.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {

SpeedScenarioManager::SpeedScenarioManager(
    const SpeedScenarioManagerConfig& config)
    : config_(config) {
  for (const auto& speed_scenario_config : config.speed_scenario_config()) {
    scenarios_.emplace(
        speed_scenario_config.scenario_type(),
        SpeedScenarioFactory::CreateSpeedScenario(speed_scenario_config));
  }
}

bool SpeedScenarioManager::Update(
    const std::shared_ptr<DependencyInjector>& injector, Frame* const frame,
    ReferenceLineInfo* const reference_line_info, SpeedCache* const cache) {
  if (injector == nullptr || frame == nullptr ||
      reference_line_info == nullptr || cache == nullptr ||
      cache->GetMutableBasicCache() == nullptr) {
    return false;
  }

  auto* basic_cache = cache->GetMutableBasicCache();
  if (basic_cache != nullptr) {
    basic_cache->SetIsLastChangeLanePath(basic_cache->GetIsChangeLanePath());
    basic_cache->SetIsChangeLaneReturnPath(
        CheckIfLaneChangeReturn(injector, *frame, *reference_line_info));
    basic_cache->SetIsChangeLanePath(reference_line_info->IsChangeLanePath() &&
                                     frame->reference_line_info().size() > 1 &&
                                     !basic_cache->GetIsChangeLaneReturnPath());
  }

  const auto scenario_type = SelectScenario(frame, reference_line_info, *cache);
  const auto iter = scenarios_.find(scenario_type);
  if (iter == scenarios_.end()) {
    return false;
  }

  if (current_scenario_type_ != scenario_type || current_scenario_ == nullptr) {
    current_scenario_type_ = scenario_type;
    current_scenario_ = iter->second;
    if (current_scenario_ == nullptr || !current_scenario_->Init()) {
      return false;
    }
  }

  ADEBUG << "current_scenario_type:"
         << SpeedScenarioConfig::ScenarioType_Name(current_scenario_type_);
  return true;
}

SpeedScenarioConfig::ScenarioType SpeedScenarioManager::SelectScenario(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    const SpeedCache& cache) {
  auto default_scenario_type = SpeedScenarioConfig::LANE_KEEP_SCENARIO;

  if (frame == nullptr || frame->GetReferenceLineProvider() == nullptr ||
      frame->GetReferenceLineProvider()->GetPncMap() == nullptr ||
      reference_line_info == nullptr) {
    return default_scenario_type;
  }

  // check if reverse gear
  if (!reference_line_info->path_data().frenet_frame_path().is_forward_path() ||
      frame->vehicle_state().gear() ==
          soc::Chassis::GearPosition::Chassis_GearPosition_GEAR_REVERSE) {
    return SpeedScenarioConfig::REVERSE_GEAR_SCENARIO;
  }

  // check if lane change
  if (cache.GetBasicCache().GetIsChangeLanePath()) {
    return SpeedScenarioConfig::LANE_CHANGE_SCENARIO;
  }

  // check if lane change return
  if (cache.GetBasicCache().GetIsChangeLaneReturnPath()) {
    return SpeedScenarioConfig::LANE_KEEP_SCENARIO;
  }

  // get lane merge info
  const auto& adc_map_common_info =
      reference_line_info->reference_line().GetAdcMapCommonInfo();
  ADEBUG << "merge_dir:"
         << static_cast<int>(adc_map_common_info.first_merge_dir)
         << ", distance:" << adc_map_common_info.dis_first_merge_begin_point;

  // check if lane merge
  if ((adc_map_common_info.first_merge_dir ==
           hdmap::MergeDirection::MERGE_TO_LEFT ||
       adc_map_common_info.first_merge_dir ==
           hdmap::MergeDirection::MERGE_TO_RIGHT) &&
      adc_map_common_info.dis_first_merge_begin_point <
          config_.lane_merge_start_distance()) {
    return SpeedScenarioConfig::LANE_MERGE_SCENARIO;
  }

  // // check if lane continue
  if ((adc_map_common_info.first_merge_dir ==
           hdmap::MergeDirection::FROM_LEFT_MERGE ||
       adc_map_common_info.first_merge_dir ==
           hdmap::MergeDirection::FROM_RIGHT_MERGE) &&
      adc_map_common_info.dis_first_merge_begin_point <
          config_.lane_continue_start_distance()) {
    return SpeedScenarioConfig::LANE_CONTINUE_SCENARIO;
  }

  return default_scenario_type;
}

bool SpeedScenarioManager::CheckIfLaneChangeReturn(
    const std::shared_ptr<DependencyInjector>& injector, const Frame& frame,
    const ReferenceLineInfo& reference_line_info) {
  static double target_l = 0.0;
  static bool in_lane_change_return = false;

  // check if lane change return begin
  if (!in_lane_change_return) {
    if (injector != nullptr && injector->planning_context() != nullptr &&
        injector->planning_context()
                ->planning_status()
                .change_lane()
                .status() ==
            TL::planning::ChangeLaneStatus::CHANGE_LANE_FAILED) {
      in_lane_change_return = true;
      target_l = reference_line_info.reference_line().GetADCWaypoint().l;
      ADEBUG << "lane change return begin";
    }
    return in_lane_change_return;
  }

  // check if this reference line is lane change return target
  const auto& reference_line_infos = frame.reference_line_info();
  const auto iter = std::min_element(
      reference_line_infos.begin(), reference_line_infos.end(),
      [&](const auto& reference_line_info1, const auto& reference_line_info2) {
        return fabs(reference_line_info1.reference_line().GetADCWaypoint().l -
                    target_l) <
               fabs(reference_line_info2.reference_line().GetADCWaypoint().l -
                    target_l);
      });
  if (iter == reference_line_infos.end() || &(*iter) != &reference_line_info) {
    in_lane_change_return = false;
    ADEBUG << "lane change return end";
    return in_lane_change_return;
  }

  target_l = iter->reference_line().GetADCWaypoint().l;

  // check if lane change return finish
  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  const auto& reference_line = reference_line_info.reference_line();
  for (const auto& frenet_frame_point :
       reference_line_info.path_data().frenet_frame_path()) {
    if (!reference_line.GetLaneWidth(frenet_frame_point.s(), &left_lane_width,
                                     &right_lane_width)) {
      continue;
    }
    if ((frenet_frame_point.l() > 0.0 &&
         frenet_frame_point.l() > left_lane_width / 2.0) ||
        (frenet_frame_point.l() < 0.0 &&
         -frenet_frame_point.l() > right_lane_width / 2.0)) {
      return in_lane_change_return;
    }
  }

  in_lane_change_return = false;
  return in_lane_change_return;
}

}  // namespace planning
}  // namespace TL
