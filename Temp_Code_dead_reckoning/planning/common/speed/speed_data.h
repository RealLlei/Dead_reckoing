#ifndef PLANNING_COMMON_SPEED_SPEED_DATA_H
#define PLANNING_COMMON_SPEED_SPEED_DATA_H

/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include <limits>
#pragma once

#include <set>
#include <string>
#include <utility>
#include <vector>

#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

class SpeedData : public std::vector<common::SpeedPoint> {
 public:
  SpeedData() = default;

  virtual ~SpeedData() = default;

  explicit SpeedData(std::vector<common::SpeedPoint> speed_points);

  void AppendSpeedPoint(double s, double time, double v, double a, double da);

  bool EvaluateByTime(double time, common::SpeedPoint* speed_point) const;

  // Assuming spatial traversed distance is monotonous, which is the case for
  // current usage on city driving scenario
  bool EvaluateByS(double s, common::SpeedPoint* speed_point) const;

  double TotalTime() const;

  // Assuming spatial traversed distance is monotonous
  double TotalLength() const;

  virtual std::string DebugString() const;

  double GetMaxAccel() const;

  double GetMinAccel() const;

  bool GetIsFallback() const { return is_fallback_; }

  void SetIsFallback(bool is_fallback) { is_fallback_ = is_fallback; }

  const std::set<std::string>& GetSpeedDeciderIntentionID() const {
    return set_of_speed_decider_intention_id_;
  }

  void SetSpeedDeciderIntentionID(
      const std::string& speed_decider_intention_id) {
    set_of_speed_decider_intention_id_.insert(speed_decider_intention_id);
  }

  /**
  * @brief Set the Low Road Right End S object
  * 
  * @param low_road_right_end_s 
  */
  void SetLowRoadRightEndS(double low_road_right_end_s) {
    low_road_right_end_s_ = low_road_right_end_s;
  }

  /**
  * @brief Get the Low Road Right End S object
  * 
  * @return double 
  */
  double GetLowRoadRightEndS() const { return low_road_right_end_s_; }

 private:
  bool is_fallback_ = false;
  double low_road_right_end_s_ = std::numeric_limits<double>::lowest();
  std::set<std::string> set_of_speed_decider_intention_id_;
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_SPEED_SPEED_DATA_H
