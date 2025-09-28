/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_scenario.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_scenario.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>

#include "common/file/log.h"
#include "common/util/macros.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_stage_factory.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

SpeedScenario::SpeedScenario(const SpeedScenarioConfig& config)
    : scenario_config_(config), current_stage_(nullptr) {
  const auto& origin_speed_stage_configs = config.speed_stage_config();
  auto* speed_stage_configs = scenario_config_.mutable_speed_stage_config();
  if (speed_stage_configs == nullptr) {
    return;
  }
  speed_stage_configs->Clear();

  for (const auto& origin_speed_stage_config : origin_speed_stage_configs) {
    const auto iter =
        std::find_if(speed_stage_configs->begin(), speed_stage_configs->end(),
                     [&](const auto& speed_stage_config) {
                       return speed_stage_config.stage_type() ==
                              origin_speed_stage_config.stage_type();
                     });
    if (iter != speed_stage_configs->end()) {
      iter->MergeFrom(origin_speed_stage_config);
    } else {
      speed_stage_configs->Add()->CopyFrom(origin_speed_stage_config);
    }
  }

  for (const auto& speed_stage_config : scenario_config_.speed_stage_config()) {
    if (current_stage_ == nullptr) {
      current_stage_ = SpeedStageFactory::CreateSpeedStage(speed_stage_config);
      stages_.emplace(speed_stage_config.stage_type(), current_stage_);
    } else {
      stages_.emplace(speed_stage_config.stage_type(),
                      SpeedStageFactory::CreateSpeedStage(speed_stage_config));
    }
  }
}

bool SpeedScenario::Init() {
  const auto& speed_stage_configs = scenario_config_.speed_stage_config();
  if (speed_stage_configs.empty()) {
    AERROR << "no stage configs";
    return false;
  }
  const auto iter = stages_.find(speed_stage_configs.at(0).stage_type());
  if (iter == stages_.end()) {
    AERROR << "can not find init stage";
    return false;
  }
  current_stage_ = iter->second;
  is_enter_scenario_ = true;
  return true;
}

bool SpeedScenario::Process(
    const std::shared_ptr<DependencyInjector>& injector, Frame* frame,
    ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, SpeedCache* const cache,
    const std::shared_ptr<SpeedDataGenerator>& generator,
    SpeedData* speed_data) {
  if (frame == nullptr || reference_line_info == nullptr || cache == nullptr ||
      speed_data == nullptr || current_stage_ == nullptr) {
    AERROR << "input is null, speed scenario process failed";
    return false;
  }

  if (is_enter_scenario_) {
    LoadAccelLimitTable(cache);
    LoadJerkLimitTable(cache);
    is_enter_scenario_ = false;
  }

  // scenario preprocess
  auto cur_init_point = init_point;
  if (!PreProcess(injector, frame, reference_line_info, &cur_init_point,
                  cache)) {
    AERROR << "current scenario preprocess failed";
    return false;
  }
  if (cache != nullptr) {
    frame->SetLonStopObsId(cache->GetBasicCache().GetBlockObstacleId());
  }

  speed_data->SetLowRoadRightEndS(cache->GetBasicCache().GetLowRoadRightEndS());

  // check if start after stop / if use standstill curve
  if (CheckIfKeepStandStill(frame, *reference_line_info, cache,
                            &cur_init_point)) {
    GenerateStandStillSpeedData(cur_init_point, speed_data);
    ADEBUG << "use standstill curve : " << cache->GetIsLastFallback();
    return true;
  }

  // stage process
  if (!current_stage_->Process(injector, frame, reference_line_info,
                               cur_init_point, cache, generator, speed_data)) {
    AERROR << "current stage process failed";
    return false;
  }

  // check if current stage finish
  if (current_stage_->FinishStage(frame, reference_line_info, cur_init_point,
                                  *cache, generator, speed_data)) {
    const auto& stage_config = current_stage_->GetStageConfig();
    if (!stage_config.has_next_stage_type()) {
      return true;
    }
    const auto iter = stages_.find(stage_config.next_stage_type());
    if (iter == stages_.end()) {
      AERROR << "can not find next stage";
      return false;
    }
    current_stage_ = iter->second;
    return true;
  }

  // scenario postprocess
  if (!PostProcess(frame, reference_line_info, cur_init_point, cache, generator,
                   speed_data)) {
    AERROR << "current scenario post process failed";
    return false;
  }
  for (const auto& safe_cache : cache->GetSafeSTObstacleCaches()) {
    if (safe_cache->GetEnableCollisionCheck()) {
      speed_data->SetSpeedDeciderIntentionID(safe_cache->GetObstacle()->Id());
    }
  }
  return true;
}

void SpeedScenario::LoadAccelLimitTable(SpeedCache* const cache) {
  if (cache == nullptr || cache->GetMutableBasicCache() == nullptr ||
      scenario_config_.accel_limit_calibration_info().empty()) {
    return;
  }

  auto* basic_cache = cache->GetMutableBasicCache();
  if (basic_cache == nullptr) {
    return;
  }

  const auto& accel_calibration_infos =
      scenario_config_.accel_limit_calibration_info();
  std::vector<double> accel_speeds(accel_calibration_infos.size(), 0.0);
  std::vector<double> accels(accel_calibration_infos.size(), 0.0);
  for (int i = 0; i < accel_calibration_infos.size(); i++) {
    accel_speeds.at(i) = accel_calibration_infos.at(i).speed();
    accels.at(i) = accel_calibration_infos.at(i).accel();
  }

  const auto& decel_calibration_infos =
      scenario_config_.decel_limit_calibration_info();
  std::vector<double> decel_speeds(decel_calibration_infos.size(), 0.0);
  std::vector<double> decels(decel_calibration_infos.size(), 0.0);
  for (int i = 0; i < decel_calibration_infos.size(); i++) {
    decel_speeds.at(i) = decel_calibration_infos.at(i).speed();
    decels.at(i) = decel_calibration_infos.at(i).decel();
  }

  basic_cache->SetAccelLimitTable(accel_speeds, accels, decel_speeds, decels);
}

void SpeedScenario::LoadJerkLimitTable(SpeedCache* const cache) {
  if (cache == nullptr || cache->GetMutableBasicCache() == nullptr ||
      scenario_config_.accel_limit_calibration_info().empty()) {
    return;
  }

  auto* basic_cache = cache->GetMutableBasicCache();
  if (basic_cache == nullptr) {
    return;
  }

  const auto& jerk_calibration_infos =
      scenario_config_.jerk_limit_calibration_info();
  std::vector<double> speeds(jerk_calibration_infos.size(), 0.0);
  std::vector<double> jerks(jerk_calibration_infos.size(), 0.0);
  for (int i = 0; i < jerk_calibration_infos.size(); i++) {
    speeds.at(i) = jerk_calibration_infos.at(i).speed();
    jerks.at(i) = jerk_calibration_infos.at(i).jerk();
  }

  basic_cache->SetJerkLimitTable(speeds, jerks);
}

}  // namespace planning
}  // namespace TL
