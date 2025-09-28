/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 *   @file
 **/

#pragma once

#include <vector>

#include "planning/common/speed/speed_data.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {

// TODO(jiacheng): currently implemented a constant velocity model for
// guide-line. Upgrade it to a constant acceleration model.
class STGuideLine {
 public:
  STGuideLine() = default;
  virtual ~STGuideLine() = default;

  void Init(double desired_v);

  void Init(double desired_v,
            const ::google::protobuf::RepeatedPtrField<common::TrajectoryPoint>&
                speed_reference);

  double GetGuideSFromT(double t);

  void UpdateBlockingInfo(double t, double s_block, bool is_lower_block);

 private:
  // Variables for simple guide-line calculation.
  double t0_ = 0.0;
  double s0_ = 0.0;
  double v0_ = 0.0;
  // St guideline from upstream modules
  SpeedData guideline_speed_data_;
};

}  // namespace planning
}  // namespace TL
