/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_change_ongoing_stage.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_change/lane_change_ongoing_stage.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>

#include "common/file/log.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/forward_gear_speed_data_generator.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

LaneChangeOngoingStage::LaneChangeOngoingStage(const SpeedStageConfig& config)
    : SpeedStage(config), config_(config.lane_change_ongoing_stage_config()) {}

bool LaneChangeOngoingStage::PreProcess(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (frame == nullptr || reference_line_info == nullptr) {
    AERROR << "input is error, LaneChangeScenario::PreProcess failed";
    return false;
  }

  CheckIfIgnoreBackObstacles(reference_line_info);

  return true;
}

bool LaneChangeOngoingStage::PostProcess(
    Frame* frame, ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const std::shared_ptr<SpeedDataGenerator>& generator,
    SpeedData* speed_data) {

  if (frame == nullptr || reference_line_info == nullptr ||
      speed_data == nullptr ||
      frame->vehicle_state().driving_mode() !=
          soc::Chassis::COMPLETE_AUTO_DRIVE) {
    return true;
  }

  const auto forward_gear_generator =
      std::dynamic_pointer_cast<ForwardGearSpeedDataGenerator>(generator);
  if (forward_gear_generator == nullptr) {
    return true;
  }

  // if adc do not follow back obstacle, do not cancel lane change
  const auto& obstacle_caches = cache.GetSafeSTObstacleCaches();
  if (std::none_of(obstacle_caches.begin(), obstacle_caches.end(),
                   [](const auto* obstacle_cache) {
                     return obstacle_cache->GetSTObstacleLocation() ==
                                STObstacleLocation::ABOVE &&
                            !obstacle_cache->GetIsFront();
                   })) {
    return true;
  }

  SpeedData speed_data_without_back_obstacle;
  forward_gear_generator->GeneratorSpeedDataWithoutBackObstacle(
      frame, reference_line_info, init_point, cache, GetSpeedEvaluator(),
      &speed_data_without_back_obstacle);

  return speed_data->GetMinAccel() > config_.lane_change_accel_threshold() ||
         speed_data->GetMinAccel() >
             speed_data_without_back_obstacle.GetMinAccel() +
                 config_.lane_change_delta_accel_threshold_for_rear_obstacle();
}

bool LaneChangeOngoingStage::FinishStage(
    Frame* frame, ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const std::shared_ptr<SpeedDataGenerator>& generator,
    SpeedData* speed_data) {
  UNUSED(init_point);
  UNUSED(cache);
  UNUSED(generator);
  UNUSED(speed_data);
  if (frame == nullptr || reference_line_info == nullptr) {
    return false;
  }
  const auto& discretized_path =
      reference_line_info->path_data().discretized_path();
  const auto& frenet_frame_path =
      reference_line_info->path_data().frenet_frame_path();
  if (discretized_path.empty() || frenet_frame_path.empty()) {
    return false;
  }

  const auto& discretized_point = discretized_path.front();
  const auto& frenet_point = frenet_frame_path.front();
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  const auto theta = (frenet_point.l() > 0)
                         ? (discretized_point.theta() - M_PI_2)
                         : (discretized_point.theta() + M_PI_2);

  common::math::Vec2d front_wheel(
      discretized_point.x() + 0.5 * vehicle_param.width() * cos(theta) +
          vehicle_param.wheel_base() * cos(discretized_point.theta()),
      discretized_point.y() + 0.5 * vehicle_param.width() * sin(theta) +
          vehicle_param.wheel_base() * sin(discretized_point.theta()));

  common::SLPoint sl_point;
  if (!reference_line_info->reference_line().XYToSL(front_wheel, &sl_point)) {
    return false;
  }

  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!reference_line_info->reference_line().GetLaneWidth(
          sl_point.s(), &lane_left_width, &lane_right_width)) {
    return false;
  }

  ADEBUG << "frenet_point.l():" << sl_point.l()
         << ", lane_left_width:" << lane_left_width
         << ", lane_right_width:" << lane_right_width;

  const auto dl = (frenet_point.l() > 0) ? (lane_left_width - sl_point.l())
                                         : (sl_point.l() + lane_right_width);
  return dl > 0;
}

void LaneChangeOngoingStage::CheckIfIgnoreBackObstacles(
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
