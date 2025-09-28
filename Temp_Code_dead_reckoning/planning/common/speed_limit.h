#ifndef PLANNING_COMMON_SPEED_LIMIT_H
#define PLANNING_COMMON_SPEED_LIMIT_H

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
 * @file speed_limit.h
 **/

#pragma once
#include <deque>
#include <utility>
#include <vector>
#include "common/math/double_type.h"
#include "planning/common/planning_gflags.h"
#include "proto/map/map_road.pb.h"

namespace TL {
namespace planning {

struct SpeedLimitInfo {
  double map_speed_limit = 0.0;
  double curvature_speed_limit = 0.0;
  double decision_speed_limit = 0.0;
  double pedestrian_speed_limit = 0.0;
  double speed_limit = 0.0;
  double origin_map_speed_limit = 0.0;
  bool allow_over_speed = false;
  double cone_speed_limit = 0.0;
};

class SpeedLimit {
 public:
  SpeedLimit() = default;

  void AppendSpeedLimit(double s, double v);

  void AppendSpeedLimitInfo(double s, const SpeedLimitInfo& info);

  const std::vector<std::pair<double, double>>& speed_limit_points() const;

  const std::vector<std::pair<double, SpeedLimitInfo>>&
  speed_limit_info_points() const {
    return speed_limit_info_points_;
  }

  std::vector<std::pair<double, SpeedLimitInfo>>*
  GetMutableSpeedLimitInfoPoints() {
    return &speed_limit_info_points_;
  }

  double GetSpeedLimitByS(double s) const;

  const SpeedLimitInfo& GetSpeedLimitInfoByS(double s) const;

  void Clear();

  /**
   * @brief use slide window to calcaulate min curvature speed limit within
   * window size in front
   *
   * @param smooth_start_index
   * @param window_size
   */
  void SmoothCurvatureSpeedLimit(int smooth_start_index, int window_size);

 private:
  static double GetSpeedLimitByS(
      const std::vector<std::pair<double, double>>& points, double s);

  // use a vector to represent speed limit
  // the first number is s, the second number is v
  // It means at distance s from the start point, the speed limit is v.
  std::vector<std::pair<double, double>> speed_limit_points_;
  std::vector<std::pair<double, SpeedLimitInfo>> speed_limit_info_points_;

  // TL::hdmap::RoadSection::Type curr_road_type_ =
  //     TL::hdmap::RoadSection::Type::RoadSection_Type_UNKNOWN;
  // bool allow_over_speed_ = false;
  // bool adc_is_tunnel_lane_ = false;
  // double allow_max_over_map_speed_ = FLAGS_planning_upper_speed_limit;
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_SPEED_LIMIT_H
