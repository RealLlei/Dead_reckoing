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
 * @file speed_limit.cc
 **/

#include "planning/common/speed_limit.h"

#include <algorithm>

#include "common/file/log.h"
#include "common/math/math_utils.h"
#include "proto/map/map_road.pb.h"

namespace TL {
namespace planning {

void SpeedLimit::AppendSpeedLimit(const double s, const double v) {
  if (!speed_limit_points_.empty()) {
    DCHECK_GE(s, speed_limit_points_.back().first);
  }
  speed_limit_points_.emplace_back(s, v);
}

void SpeedLimit::AppendSpeedLimitInfo(const double s,
                                      const SpeedLimitInfo& info) {
  if (!speed_limit_info_points_.empty()) {
    DCHECK_GE(s, speed_limit_info_points_.back().first);
  }
  speed_limit_info_points_.emplace_back(s, info);
}

const std::vector<std::pair<double, double>>& SpeedLimit::speed_limit_points()
    const {
  return speed_limit_points_;
}

double SpeedLimit::GetSpeedLimitByS(const double s) const {
  return GetSpeedLimitByS(speed_limit_points_, s);
}

const SpeedLimitInfo& SpeedLimit::GetSpeedLimitInfoByS(const double s) const {
  CHECK_GE(speed_limit_info_points_.size(), 2U);
  DCHECK_GE(s, speed_limit_info_points_.front().first);

  auto compare_s = [](const auto& point, const auto s) {
    return point.first < s;
  };

  auto it_lower =
      std::lower_bound(speed_limit_info_points_.begin(),
                       speed_limit_info_points_.end(), s, compare_s);

  if (it_lower == speed_limit_info_points_.end()) {
    return (it_lower - 1)->second;
  }
  return it_lower->second;
}

double SpeedLimit::GetSpeedLimitByS(
    const std::vector<std::pair<double, double>>& points, const double s) {
  CHECK_GE(points.size(), 2U);
  DCHECK_GE(s, points.front().first);

  auto compare_s = [](const std::pair<double, double>& point, const double s) {
    return point.first < s;
  };

  auto it_lower = std::lower_bound(points.begin(), points.end(), s, compare_s);

  if (it_lower == points.end()) {
    return (it_lower - 1)->second;
  }
  return it_lower->second;
}

void SpeedLimit::Clear() {
  speed_limit_points_.clear();
  speed_limit_info_points_.clear();
}

void SpeedLimit::SmoothCurvatureSpeedLimit(const int smooth_start_index,
                                           const int window_size) {
  const auto unsmooth_points = speed_limit_info_points_;
  std::deque<int> indexs;
  const auto smooth_end_index =
      static_cast<int>(unsmooth_points.size()) + window_size;
  for (int i = std::max(smooth_start_index, 0); i < smooth_end_index; ++i) {
    if (i < static_cast<int>(unsmooth_points.size())) {
      while (!indexs.empty() &&
             unsmooth_points[i].second.curvature_speed_limit <=
                 unsmooth_points[indexs.back()].second.curvature_speed_limit) {
        indexs.pop_back();
      }
      indexs.push_back(i);
    }

    if (indexs.front() < i - 2 * window_size) {
      indexs.pop_front();
    }

    const auto start_index = i - window_size;
    if (start_index >= 0) {
      speed_limit_info_points_[start_index].second.curvature_speed_limit =
          unsmooth_points[indexs.front()].second.curvature_speed_limit;
    }
  }
}

}  // namespace planning
}  // namespace TL
