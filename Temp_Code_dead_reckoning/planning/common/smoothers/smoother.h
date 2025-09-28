#ifndef PLANNING_COMMON_SMOOTHERS_SMOOTHER_H
#define PLANNING_COMMON_SMOOTHERS_SMOOTHER_H

/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include "common/status/status.h"
#include "planning/common/frame.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/planning/decision.pb.h"

namespace TL {
namespace planning {

class Smoother {
 public:
  Smoother() = default;
  virtual ~Smoother() = default;

  static TL::common::Status Smooth(const FrameHistory* frame_history,
                                      const Frame* current_frame,
                                      ADCTrajectory* current_trajectory_pb);

 private:
  static bool IsCloseStop(const common::VehicleState& vehicle_state,
                          const MainStop& main_stop);
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_SMOOTHERS_SMOOTHER_H
