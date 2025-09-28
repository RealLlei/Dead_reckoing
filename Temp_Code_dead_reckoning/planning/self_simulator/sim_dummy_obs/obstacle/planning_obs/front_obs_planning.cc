/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#include "planning/self_simulator/sim_dummy_obs/obstacle/planning_obs/front_obs_planning.h"

namespace TL {
namespace simdummy {
bool FrontObs_planning::StartDisplay(const DummyObsInputDataBase& data) {
  return false;
}

bool FrontObs_planning::StopDisplay(const DummyObsInputDataBase& data) {
  SetIsNeededRemove(false);
  return false;
}

void FrontObs_planning::UpdateState(const DummyObsInputDataBase& data) {
  return;
}
}  // namespace simdummy
}  // namespace TL
