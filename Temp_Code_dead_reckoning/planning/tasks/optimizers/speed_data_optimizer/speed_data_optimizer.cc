/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_data_optimizer.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/speed_data_optimizer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <ios>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/match.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/file/log.h"
#include "common/status/status.h"
#include "common/util/macros.h"
#include "planning/common/frame.h"
#include "planning/common/path/frenet_frame_path.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/nudge_cost.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/speed_data_generator_factory.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {

using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::TrajectoryPoint;

SpeedDataOptimizer::SpeedDataOptimizer(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : SpeedOptimizer(config, injector),
      optimizer_config_(config.speed_data_optimizer_config()),
      cache_(config.speed_data_optimizer_config().speed_cache_config()) {
  CreateScenarioManager();

  forward_generator_ = SpeedDataGeneratorFactory::CreateSpeedDataGenerator(
      optimizer_config_.forward_speed_data_generator_config());
  reverse_generator_ = SpeedDataGeneratorFactory::CreateSpeedDataGenerator(
      optimizer_config_.reverse_speed_data_generator_config());
}

void SpeedDataOptimizer::CreateScenarioManager() {
  if (optimizer_config_.has_speed_scenario_manager_config()) {
    scenario_manager_ = std::make_shared<SpeedScenarioManager>(
        optimizer_config_.speed_scenario_manager_config());
  } else {
    scenario_manager_ =
        std::make_shared<SpeedScenarioManager>(SpeedScenarioManagerConfig());
  }
}

std::shared_ptr<SpeedDataGenerator> SpeedDataOptimizer::SelectGenerator(
    const std::shared_ptr<SpeedScenario>& scenario) {
  const auto scenario_type = scenario->GetScenarioConfig().scenario_type();
  if (scenario_type == SpeedScenarioConfig::REVERSE_GEAR_SCENARIO) {
    return reverse_generator_;
  }
  return forward_generator_;
}

Status SpeedDataOptimizer::Process(const PathData& path_data,
                                   const TrajectoryPoint& init_point,
                                   SpeedData* const speed_data) {
  if (frame_ == nullptr || reference_line_info_ == nullptr ||
      path_data.discretized_path().empty() ||
      path_data.frenet_frame_path().empty() || speed_data == nullptr ||
      scenario_manager_ == nullptr) {
    const std::string msg = "input error, SpeedDataOptimizer::Process failed";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_VTSAMPLEOPTIMIZER_ERROR, msg);
  }

  // select scenario
  scenario_manager_->Update(injector_, frame_, reference_line_info_, &cache_);

  // get current scenario
  auto current_scenario = scenario_manager_->GetMutableCurrentScenario();
  if (current_scenario == nullptr) {
    const std::string msg = "current scenario is nullptr";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_VTSAMPLEOPTIMIZER_ERROR, msg);
  }

  // process
  if (!current_scenario->Process(
          injector_, frame_, reference_line_info_, init_point, &cache_,
          SelectGenerator(current_scenario), speed_data)) {
    const std::string msg = "current scenario process failed";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_VTSAMPLEOPTIMIZER_ERROR, msg);
  }

  // record debug
  RecordDebug(path_data);

  // for lane change or lane borrow, use common fallback
  if (speed_data->GetIsFallback() &&
      ((cache_.GetBasicCache().GetIsChangeLanePath() &&
        current_scenario->GetCurrentStage() != nullptr &&
        current_scenario->GetCurrentStage()->GetStageConfig().stage_type() ==
            SpeedStageConfig::LANE_CHANGE_ONGOING_STAGE) ||
       absl::StrContains(path_data.path_label(), "regular/lane_keep/left") ||
       absl::StrContains(path_data.path_label(), "regular/lane_keep/right"))) {
    const std::string msg =
        "for " + path_data.path_label() +
        ", when normal vt sample curves collision, use common fallback";
    AERROR << msg;
    return common::Status(
        common::ErrorCode::PLANNER_CRUISING_VTSAMPLEOPTIMIZER_ERROR, msg);
  }

  return common::Status::OK();
}

void SpeedDataOptimizer::RecordSpeedLimit(const double length) {
  auto* speed_limit_point = reference_line_info_->mutable_debug()
                                ->mutable_planning_data()
                                ->mutable_speed_data_optimizer_debug()
                                ->mutable_speed_limit_point();
  if (speed_limit_point == nullptr) {
    return;
  }

  const auto& speed_limit_cache = cache_.GetSpeedLimitCache();
  constexpr auto s_unit = 1.0;
  auto count = static_cast<int>(round(length / s_unit)) + 1;
  for (int i = 0; i < count; ++i) {
    const auto s = i * s_unit;
    const auto& speed_limit_info = speed_limit_cache.GetPositionSpeedLimit(s);
    ADEBUG << FIXED << SETPRECISION(3) << "s:" << s
           << ", map_speed_limit:" << speed_limit_info.map_speed_limit
           << ", curvature_speed_limit:"
           << speed_limit_info.curvature_speed_limit
           << ", decision_speed_limit:" << speed_limit_info.decision_speed_limit
           << ", cruise_speed_limit:" << reference_line_info_->GetCruiseSpeed();
    auto* point = speed_limit_point->Add();
    if (point != nullptr) {
      point->set_s(s);
      point->set_map_speed_limit(speed_limit_info.map_speed_limit);
      point->set_curvature_speed_limit(speed_limit_info.curvature_speed_limit);
      point->set_decision_speed_limit(speed_limit_info.decision_speed_limit);
      point->set_allow_over_speed(speed_limit_info.allow_over_speed);
      point->set_origin_map_speed_limit(
          speed_limit_info.origin_map_speed_limit);
      point->set_cruise_speed_limit(reference_line_info_->GetCruiseSpeed());
    }
  }
}

void SpeedDataOptimizer::RecordObstacles() {
  const auto& obstacles =
      reference_line_info_->path_decision()->obstacles().Items();
  for (const auto* obstacle : obstacles) {
    const auto& boundary = obstacle->path_st_boundary();
    ADEBUG << "id:" << obstacle->Id() << ", min_t:" << boundary.min_t()
           << ", max_t:" << boundary.max_t() << ", min_s:" << boundary.min_s()
           << ", max_s:" << boundary.max_s();
  }

  auto* nudge_infos = reference_line_info_->mutable_debug()
                          ->mutable_planning_data()
                          ->mutable_speed_data_optimizer_debug()
                          ->mutable_nudge_info();
  if (nudge_infos == nullptr) {
    return;
  }

  for (const auto& nudge_obstacle_cache : cache_.GetNudgeObstacleCaches()) {
    auto* nudge_info = nudge_infos->Add();
    const auto* obstacle = nudge_obstacle_cache.GetObstacle();
    if (nudge_info == nullptr || obstacle == nullptr) {
      continue;
    }
    nudge_info->set_id(obstacle->Id());
    nudge_info->set_target_nudge_state(
        SpeedCacheConfig::NudgeState_descriptor()
            ->FindValueByNumber(nudge_obstacle_cache.GetTargetNudgeState())
            ->name());
    nudge_info->set_current_nudge_state(
        SpeedCacheConfig::NudgeState_descriptor()
            ->FindValueByNumber(nudge_obstacle_cache.GetCurrentNudgeState())
            ->name());
  }
}

void SpeedDataOptimizer::RecordDebug(const PathData& path_data) {
  // record speed limit
  RecordSpeedLimit(path_data.discretized_path().Length());

  // print obstacle
  RecordObstacles();

  auto* speed_data_optimizer_debug = reference_line_info_->mutable_debug()
                                         ->mutable_planning_data()
                                         ->mutable_speed_data_optimizer_debug();

  // record follow time
  speed_data_optimizer_debug->set_follow_time(cache_.GetFollowTime());

  // record stop s
  speed_data_optimizer_debug->set_expected_stop_s(
      cache_.GetBasicCache().GetExpectedStopS());

  // record scenario
  const auto& current_scenario = scenario_manager_->GetCurrentScenario();
  if (current_scenario != nullptr) {
    speed_data_optimizer_debug->set_scenario_type(
        SpeedScenarioConfig::ScenarioType_descriptor()
            ->FindValueByNumber(
                current_scenario->GetScenarioConfig().scenario_type())
            ->name());
    const auto& current_stage = current_scenario->GetCurrentStage();
    if (current_stage != nullptr) {
      speed_data_optimizer_debug->set_stage_type(
          SpeedStageConfig::StageType_descriptor()
              ->FindValueByNumber(current_stage->GetStageConfig().stage_type())
              ->name());
    }
  }
}

}  // namespace planning
}  // namespace TL
