/*
 * Copyright (c) 2021 TL
 *
 * Author: Liu Bei
 */
#include "planning/self_simulator/self_simulator.h"

#include "common/configs/config_gflags.h"
#include "planning/self_simulator/sim_dummy_obs/obs_manager.h"

namespace TL {
namespace planning {
using TL::perception::PerceptionObstacles;

SelfSimulator::SelfSimulator(/* args */) {}

SelfSimulator::~SelfSimulator() {}

void SelfSimulator::Init() {
  sim_control_.Init();
}

void SelfSimulator::Start() {
  sim_control_.Start();
}

void SelfSimulator::Stop() {
  sim_control_.Stop();
}

void SelfSimulator::Reset() {
  sim_control_.Reset();
}

void SelfSimulator::Process(const std::shared_ptr<LocalView>& local_view,
                            std::shared_ptr<hdmap::HDMap> map_ptr) {
  sim_control_.Process(local_view, map_ptr);
}

void SelfSimulator::InjectorPerceptionObs(
    const std::shared_ptr<LocalView>& local_view,
    std::shared_ptr<hdmap::HDMap> map_ptr) {
  if (!FLAGS_enable_planning_injector_obs) {
    return;
  }
  auto perception = std::make_shared<PerceptionObstacles>(
      *local_view->GetPerceptionObstacles());
  double delta_time = 0;
  if (time_stamp_prev_ < 0.01) {
    time_stamp_prev_ = common::Clock::NowInSeconds();
  } else {
    delta_time = common::Clock::NowInSeconds() - time_stamp_prev_;
    time_stamp_prev_ = common::Clock::NowInSeconds();
  }
  auto adc_trajectory_ptr = std::make_shared<ADCTrajectory>();
  if (local_view->HasADCTrajectory()) {
    adc_trajectory_ptr->CopyFrom(*local_view->GetPerceptionObstacles());
  }

  common::PathPoint adc_position;
  auto& pose = local_view->GetLocalization()->pose();
  adc_position.set_x(pose.position().x());
  adc_position.set_y(pose.position().y());
  if (adc_trajectory_ptr) {
    TL::simdummy::PerceptionObsManager::getInstance().Update(
        delta_time, adc_trajectory_ptr,
        std::make_shared<common::PathPoint>(adc_position), perception,
        local_view, map_ptr);
  }
  local_view->SetPerceptionObstaclesPtr(perception);
  local_view->SetPerceptionObstaclesMinieyePtr(perception);
}

bool SelfSimulator::UpdateRoutingResponse(
    const std::shared_ptr<const routing::RoutingResponse>& routing_response) {
  sim_control_.OnRoutingResponse(routing_response);
  return true;
}

void SelfSimulator::UpdatePlanning(
    const std::shared_ptr<const ADCTrajectory>& adc_trajectory) {
  sim_control_.OnPlanning(adc_trajectory);
}

}  // namespace planning
}  // namespace TL
