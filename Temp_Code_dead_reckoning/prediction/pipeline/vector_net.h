/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *implied. See the License for the specific language governing
 *permissions and limitations under the License.
 *****************************************************************************/

#pragma once

#include <algorithm>
#include <deque>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/math/linear_interpolation.h"
#include "common/util/point_factory.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/prediction/common/prediction_gflags.h"

#include "proto/prediction/vector_net.pb.h"

namespace TL {
namespace prediction {

using FeatureVector = std::vector<std::vector<std::vector<double>>>;
using PidVector = std::vector<std::vector<double>>;

enum ATTRIBUTE_TYPE {
  ROAD,
  LANE_UNKOWN,
  LANE_DOTTED_YELLOW,
  LANE_DOTTED_WHITE,
  LANE_SOLID_YELLOW,
  LANE_SOLID_WHITE,
  LANE_DOUBLE_YELLOW,
  LANE_CURB,
  JUNCTION,
  CROSSWALK,
};

enum BOUNDARY_TYPE {
  UNKNOW,
  NORMAL,
  LEFT_BOUNDARY,
  RIGHT_BOUNDARY,
};

const std::map<ATTRIBUTE_TYPE, double> attribute_map{
    {ROAD, 0.0},
    {LANE_UNKOWN, 1.0},
    {LANE_DOTTED_YELLOW, 2.0},
    {LANE_DOTTED_WHITE, 3.0},
    {LANE_SOLID_YELLOW, 4.0},
    {LANE_SOLID_WHITE, 5.0},
    {LANE_DOUBLE_YELLOW, 6.0},
    {LANE_CURB, 7.0},
    {JUNCTION, 8.0},
    {CROSSWALK, 9.0},
};

const std::map<BOUNDARY_TYPE, double> boundary_map{
    {UNKNOW, 0.0},
    {NORMAL, 1.0},
    {LEFT_BOUNDARY, 2.0},
    {RIGHT_BOUNDARY, 3.0},
};

const std::map<hdmap::LaneBoundaryType::Type, ATTRIBUTE_TYPE> lane_attr_map{
    {hdmap::LaneBoundaryType::UNKNOWN, LANE_UNKOWN},
    {hdmap::LaneBoundaryType::DOTTED_YELLOW, LANE_DOTTED_YELLOW},
    {hdmap::LaneBoundaryType::DOTTED_WHITE, LANE_DOTTED_WHITE},
    {hdmap::LaneBoundaryType::SOLID_YELLOW, LANE_SOLID_YELLOW},
    {hdmap::LaneBoundaryType::SOLID_WHITE, LANE_SOLID_WHITE},
    {hdmap::LaneBoundaryType::DOUBLE_YELLOW, LANE_DOUBLE_YELLOW},
    {hdmap::LaneBoundaryType::CURB, LANE_CURB},
};

class VectorNet {
 public:
  VectorNet() = default;

  ~VectorNet() = default;

  static bool Query(const common::PointENU& center_point,
                    VectorNetFeature* const vector_net_ptr);
  static bool AppendRoutingLaneGraph(
      const std::set<std::string>& routing_lane_set,
      VectorNetFeature* const vector_net_ptr);

 private:
  static bool GetOnePolyline(
      const common::PointENU& center_point,
      const TL::hdmap::LaneInfoConstPtr& lane,
      std::priority_queue<
          std::pair<double, std::pair<TL::hdmap::LaneInfoConstPtr,
                                      std::vector<std::pair<double, double>>>>>*
          valid_lane_queue,
      size_t* points_size);

  static void GetLaneQueue(
      const std::vector<hdmap::LaneInfoConstPtr>& lanes,
      std::vector<std::deque<hdmap::LaneInfoConstPtr>>* const lane_deque_ptr);

  static void AddConnectionInfo(const std::string& lane1_id,
                                const std::string& lane2_id,
                                VectorNetFeature* const vector_net_ptr);
  static void SaveValidLane(
      VectorNetFeature* const vector_net_ptr,
      const std::pair<TL::hdmap::LaneInfoConstPtr,
                      std::vector<std::pair<double, double>>>& lane_info);
  static void GetLanes(const common::PointENU& center_point,
                       VectorNetFeature* const vector_net_ptr);
};

}  // namespace prediction
}  // namespace TL
