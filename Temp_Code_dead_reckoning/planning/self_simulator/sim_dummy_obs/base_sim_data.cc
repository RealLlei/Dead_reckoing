/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#include <memory>
#include <utility>

#include "planning/self_simulator/sim_dummy_obs/base_sim_data.h"

namespace TL {
namespace simdummy {
DummyObsInputDataBase::DummyObsInputDataBase() {
  x1 = 1;
  y2 = 2;
}

void DummyObsInputDataBase::Update(
    double t_delta_time,
    std::shared_ptr<const TL::planning::ADCTrajectory> t_traj,
    std::shared_ptr<common::PathPoint> adc_position,
    std::shared_ptr<TL::perception::PerceptionObstacles> t_perception,
    std::shared_ptr<TL::planning::LocalView> local_view,
    std::shared_ptr<hdmap::HDMap> map_ptr) {
  delta_time = t_delta_time;
  current_trajectory_ = t_traj;
  adc_position_ = std::move(adc_position);
  perception = std::move(t_perception);
  local_view_ = std::move(local_view);
  map_ptr_ = map_ptr;
}
}  // namespace simdummy
}  // namespace TL
