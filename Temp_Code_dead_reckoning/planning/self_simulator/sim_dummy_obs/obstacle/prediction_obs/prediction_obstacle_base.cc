/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#include "planning/self_simulator/sim_dummy_obs/obstacle/prediction_obs/prediction_obstacle_base.h"

namespace TL {
namespace simdummy {
void PredictionObstacleBase::SetState(int32_t id, double length_, double width,
                                      double height, double x, double y,
                                      double theta, double v, boost::any type) {
  return;
}

void PredictionObstacleBase::Update(double x, double y, double theta,
                                    double v) {
  return;
}

void PredictionObstacleBase::Update(double delta_time, double theta) {
  return;
}

}  // namespace simdummy
}  // namespace TL
