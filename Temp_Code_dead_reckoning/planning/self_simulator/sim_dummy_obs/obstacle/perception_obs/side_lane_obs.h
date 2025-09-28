/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#pragma once

#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/perception_obstacle_base.h"

namespace TL {
namespace simdummy {
class SideObs : public PerceptionObstacleBase {
 public:
  virtual bool StartDisplay(const DummyObsInputDataBase& data);

  virtual bool StopDisplay(const DummyObsInputDataBase& data);

  virtual void UpdateState(const DummyObsInputDataBase& data);
};
}  // namespace simdummy
}  // namespace TL
