/*
 * Copyright (c) 2021 TL
 *
 * Author: Liu Bei
 */
#pragma once

#include <memory>

#include "planning/localview/local_view.h"
#include "planning/self_simulator/sim_control/sim_control.h"

namespace TL {
namespace planning {

class SelfSimulator {
 public:
  SelfSimulator(/* args */);
  ~SelfSimulator();
  void Init();

  void Start();

  void Stop();

  void Reset();

  void Process(const std::shared_ptr<LocalView>& local_view,
               std::shared_ptr<hdmap::HDMap> map_ptr);

  void InjectorPerceptionObs(const std::shared_ptr<LocalView>& local_view,
                             std::shared_ptr<hdmap::HDMap> map_ptr);

  void UpdatePlanning(
      const std::shared_ptr<const ADCTrajectory>& adc_trajectory);
  bool UpdateRoutingResponse(
      const std::shared_ptr<const routing::RoutingResponse>& routing_response);

 private:
  SimControl sim_control_;
  double time_stamp_prev_ = 0;
};

}  // namespace planning
}  // namespace TL
