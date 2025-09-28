/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#pragma once

#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/perception_obstacle_base.h"

namespace TL {
namespace simdummy {
class RelativePositionObstacle : public PerceptionObstacleBase {
 public:
  explicit RelativePositionObstacle(int id, double s, double l, double v)
      : init_id_(id), init_s_(s), init_l_(l), init_v_(v) {}

  virtual bool StartDisplay(const DummyObsInputDataBase& data);

  virtual bool StopDisplay(const DummyObsInputDataBase& data);

  virtual void UpdateState(const DummyObsInputDataBase& data);

 private:
  int init_id_;
  double init_s_;
  double init_l_;
  double init_heading_;
  double init_v_;
};
}  // namespace simdummy
}  // namespace TL
