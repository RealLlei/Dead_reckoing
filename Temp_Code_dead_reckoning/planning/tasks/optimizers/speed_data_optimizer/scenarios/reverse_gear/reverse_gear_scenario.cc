/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file reverse_gear_scenario.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/reverse_gear/reverse_gear_scenario.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

ReverseGearScenario::ReverseGearScenario(const SpeedScenarioConfig& config)
    : SpeedScenario(config) {}

bool ReverseGearScenario::PreProcess(
    const std::shared_ptr<DependencyInjector>& injector, Frame* const frame,
    ReferenceLineInfo* const reference_line_info,
    common::TrajectoryPoint* const init_point, SpeedCache* const cache) {
  if (frame == nullptr || reference_line_info == nullptr ||
      init_point == nullptr) {
    return false;
  }

  // adjust init speed and accel
  init_point->set_v(fabs(init_point->v()));
  init_point->set_a(-init_point->a());

  cache->Init(injector, *frame, reference_line_info, *init_point);

  // stage preprocess
  const auto& current_stage = GetCurrentStage();
  if (current_stage == nullptr ||
      !current_stage->PreProcess(frame, reference_line_info)) {
    AERROR << "current stage preprocess failed";
    return false;
  }

  // init cache
  cache->InitObstacle(injector, *frame, *reference_line_info, *init_point);

  return true;
}

bool ReverseGearScenario::PostProcess(
    Frame* frame, ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, SpeedCache* cache,
    const std::shared_ptr<SpeedDataGenerator>& generator,
    SpeedData* speed_data) {
  if (frame == nullptr || reference_line_info == nullptr || cache == nullptr ||
      generator == nullptr || speed_data == nullptr) {
    return false;
  }

  // stage postprocess
  const auto& current_stage = GetCurrentStage();
  if (current_stage == nullptr ||
      !current_stage->PostProcess(frame, reference_line_info, init_point,
                                  *cache, generator, speed_data)) {
    AERROR << "current stage post process failed";
    return false;
  }

  // adjust speed data speed and accel
  for (auto& speed_point : *speed_data) {
    speed_point.set_v(-speed_point.v());
    speed_point.set_a(-speed_point.a());
  }

  cache->SetIsLastFallback(speed_data->GetIsFallback() &&
                           !frame->IsVehicleStandStill());

  return true;
}

bool ReverseGearScenario::CheckIfKeepStandStill(
    Frame* const frame, const ReferenceLineInfo& reference_line_info,
    SpeedCache* const cache, common::TrajectoryPoint* const init_point) {
  if (init_point == nullptr || frame == nullptr || cache == nullptr) {
    AERROR << "input data is null";
    return true;
  }

  if (frame->IsVehicleStandStill() &&
      cache->GetBasicCache().GetExpectedStopS() >
          GetScenarioConfig().adc_start_distance_threshold_after_stop()) {
    init_point->set_v(fmax(init_point->v(), 0.0));
    init_point->set_a(fmax(init_point->a(), 0.0));
    ADEBUG << "start after stop";
    return false;
  }

  if (std::isinf(cache->GetBasicCache().GetExpectedStopS())) {
    return false;
  }

  const auto* last_frame = cache->GetBasicCache().GetLastFrame();
  if (last_frame == nullptr) {
    return false;
  }

  const auto* last_reference_line_info = last_frame->DriveReferenceLineInfo();
  return last_reference_line_info != nullptr &&
         last_reference_line_info->speed_data().TotalLength() < 1.0 &&
         last_reference_line_info->path_data()
                 .frenet_frame_path()
                 .is_forward_path() == reference_line_info.path_data()
                                           .frenet_frame_path()
                                           .is_forward_path() &&
         frame->IsVehicleStandStill();
}

bool ReverseGearScenario::GenerateStandStillSpeedData(
    const common::TrajectoryPoint& init_point, SpeedData* const speed_data) {
  if (speed_data == nullptr) {
    return false;
  }

  const auto a = fmin(GetScenarioConfig().standstill_accel(), -1e-6);
  const auto max_decel_t = -init_point.v() / a;
  const auto max_s = 0.5 * init_point.v() * max_decel_t;
  common::SpeedPoint speed_point;
  const auto t_count = static_cast<int>(std::round(
      FLAGS_trajectory_time_length / FLAGS_trajectory_time_resolution));
  for (int i = 0; i <= t_count; ++i) {
    const auto t = i * FLAGS_trajectory_time_resolution;
    if (t < max_decel_t) {
      speed_point.set_s(init_point.v() * t + 0.5 * a * t * t);
      speed_point.set_v(-(init_point.v() + a * t));
    } else {
      speed_point.set_s(max_s);
      speed_point.set_v(0.0);
    }
    speed_point.set_t(t);
    speed_point.set_a(-a);
    speed_point.set_da(0.0);
    speed_data->push_back(speed_point);
  }

  return true;
}

}  // namespace planning
}  // namespace TL
