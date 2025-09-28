/******************************************************************************
 * Copyright 2021 The TL Authors. All Rights Reserved.
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
#include "planning/prediction/pipeline/vector_net.h"

#include <cmath>
#include <limits>
#include <unordered_set>
#include <utility>

#include "common/file/file.h"

namespace TL {
namespace prediction {
bool VectorNet::GetOnePolyline(
    const common::PointENU& center_point,
    const TL::hdmap::LaneInfoConstPtr& lane,
    std::priority_queue<
        std::pair<double, std::pair<TL::hdmap::LaneInfoConstPtr,
                                    std::vector<std::pair<double, double>>>>>*
        valid_lane_queue,
    size_t* points_size) {
  std::vector<common::PointENU> lane_points;
  size_t lane_points_size = 0;
  for (const auto& segment : lane->lane().central_curve().segment()) {
    const auto& points = segment.line_segment().point();
    auto segment_points_size = points.size();
    for (int i = 0; i < segment_points_size; ++i) {
      lane_points.emplace_back(points.at(i));
    }
    lane_points_size += segment_points_size;
  }

  if (lane_points_size < 2) {
    ADEBUG << "lane id:[" << lane->id().id() << "] has " << lane_points_size
           << " points, cannot generate a polyline.";
    return false;
  }

  std::vector<double> lane_s(1, 0.0);
  for (size_t i = 1; i < lane_points_size; ++i) {
    double current_s =
        std::hypot(lane_points.at(i).x() - lane_points.at(i - 1).x(),
                   lane_points.at(i).y() - lane_points.at(i - 1).y()) +
        lane_s.at(i - 1);
    // if (std::fabs(current_s) < FLAGS_road_distance) {
    //   lane_s.emplace_back(current_s);
    // }
    lane_s.emplace_back(current_s);
  }

  std::vector<std::pair<double, double>> points;
  double cur_length = 0.0;
  double points_interval =
      std::max(lane_s.back() / FLAGS_polyline_length, FLAGS_point_distance);
  auto it_lower = std::lower_bound(lane_s.begin(), lane_s.end(), cur_length);
  while (it_lower != lane_s.end()) {
    if (it_lower == lane_s.begin()) {
      points.emplace_back(lane_points.at(0).x(), lane_points.at(0).y());
    } else {
      const auto& distance = std::distance(lane_s.begin(), it_lower);
      points.emplace_back(
          common::math::lerp(lane_points.at(distance - 1).x(),
                             lane_s[distance - 1], lane_points.at(distance).x(),
                             lane_s[distance], cur_length),
          common::math::lerp(lane_points.at(distance - 1).y(),
                             lane_s[distance - 1], lane_points.at(distance).y(),
                             lane_s[distance], cur_length));
    }
    // condition:  distance to end < point_distance
    if (cur_length + points_interval > lane_s.back()) {
      points.emplace_back(lane_points.rbegin()->x(), lane_points.rbegin()->y());
      // cur_length += (lane_s.back() - cur_length);
      it_lower = lane_s.end();
    } else {
      cur_length += points_interval;
      it_lower = std::lower_bound(lane_s.begin(), lane_s.end(), cur_length);
    }
  }
  if (points.size() < 2) {
    ADEBUG << "lane id:[" << lane->id().id()
           << "] cannot generate correct polyline points";
    return false;
  }
  auto start_point = points[0];
  auto end_point = points[points.size() - 1];
  auto distance = std::hypot(
      (start_point.first + end_point.first) / 2 - center_point.x(),
      (start_point.second + end_point.second) / 2 - center_point.y());

  if (valid_lane_queue->size() < FLAGS_polyline_num) {
    valid_lane_queue->emplace(distance, std::make_pair(lane, points));
    *points_size += points.size();
  } else if (valid_lane_queue->top().first > distance) {
    *points_size -= valid_lane_queue->top().second.second.size();
    valid_lane_queue->pop();
    valid_lane_queue->emplace(distance, std::make_pair(lane, points));
    *points_size += points.size();
  }
  return true;
}

bool VectorNet::Query(const common::PointENU& center_point,
                      VectorNetFeature* const vector_net_ptr) {
  if (vector_net_ptr == nullptr) {
    return false;
  }
  vector_net_ptr->clear_map_polyline();
  GetLanes(center_point, vector_net_ptr);
  return true;
}

bool VectorNet::AppendRoutingLaneGraph(
    const std::set<std::string>& routing_lane_set,
    VectorNetFeature* const vector_net_ptr) {
  if (nullptr == vector_net_ptr) {
    return false;
  }
  auto map_polyline_num = vector_net_ptr->map_polyline_size();
  int count = 0;
  for (int i = 0; i < map_polyline_num; ++i) {
    const auto& lane = vector_net_ptr->map_polyline(i);
    std::string lane_id = lane.lane_id();
    if (routing_lane_set.find(lane_id) != routing_lane_set.end()) {
      vector_net_ptr->add_routing_lane_id(lane_id);
      count += 1;
    }
  }
  if (0 == count) {
    ADEBUG << "No routing lane info is successfully added!";
    return false;
  }
  return true;
}

void VectorNet::AddConnectionInfo(const std::string& lane1_id,
                                  const std::string& lane2_id,
                                  VectorNetFeature* const vector_net_ptr) {
  auto* connection = vector_net_ptr->add_connection();
  connection->set_polyline1_id(lane1_id);
  connection->set_polyline2_id(lane2_id);
}

void VectorNet::SaveValidLane(
    VectorNetFeature* const vector_net_ptr,
    const std::pair<TL::hdmap::LaneInfoConstPtr,
                    std::vector<std::pair<double, double>>>& lane_info) {
  const auto& lane = lane_info.first;
  const auto& lane_points = lane_info.second;
  size_t point_size = lane_points.size();
  auto* one_polyline = vector_net_ptr->add_map_polyline();
  one_polyline->set_lane_id(lane->id().id());
  one_polyline->set_lane_type(lane->lane().type());
  for (size_t i = 1; i < point_size; ++i) {
    auto* one_vector = one_polyline->add_vector();

    one_vector->mutable_start()->set_x(lane_points[i - 1].first);
    one_vector->mutable_start()->set_y(lane_points[i - 1].second);

    one_vector->mutable_end()->set_x(lane_points[i].first);
    one_vector->mutable_end()->set_y(lane_points[i].second);

    // TODO(xk): add lane feature to one_vector.feature()
  }
}

void VectorNet::GetLanes(const common::PointENU& center_point,
                         VectorNetFeature* const vector_net_ptr) {
  std::vector<TL::hdmap::LaneInfoConstPtr> lanes;
  TL::hdmap::HDMapUtil::MapForPrediction().GetLanes(
      center_point, FLAGS_road_distance, &lanes);
  size_t points_size = 0;
  std::priority_queue<
      std::pair<double, std::pair<TL::hdmap::LaneInfoConstPtr,
                                  std::vector<std::pair<double, double>>>>>
      valid_lane_queue;
  std::unordered_map<std::string, TL::hdmap::LaneInfoConstPtr>
      valid_lane_set;
  for (const auto& lane : lanes) {
    if (!GetOnePolyline(center_point, lane, &valid_lane_queue, &points_size)) {
      continue;
    }
  }

  while (points_size > FLAGS_point_num) {
    size_t point_num = valid_lane_queue.top().second.second.size();
    points_size -= point_num;
    valid_lane_queue.pop();
  }

  while (!valid_lane_queue.empty()) {
    const auto& lane_info = valid_lane_queue.top().second;
    SaveValidLane(vector_net_ptr, lane_info);
    valid_lane_set.emplace(lane_info.first->id().id(), lane_info.first);
    valid_lane_queue.pop();
  }

  for (auto lane_info = valid_lane_set.begin();
       lane_info != valid_lane_set.end(); ++lane_info) {
    // no removal iteration
    for (int index = 0; index < lane_info->second->lane().successor_id_size();
         index++) {
      auto id = lane_info->second->lane().successor_id(index).id();
      if (valid_lane_set.find(id) != valid_lane_set.end()) {
        AddConnectionInfo(lane_info->first, id, vector_net_ptr);
      }
    }
    // no removal iteration
    for (int index = 0; index < lane_info->second->lane().predecessor_id_size();
         index++) {
      auto id = lane_info->second->lane().predecessor_id(index).id();
      if (valid_lane_set.find(id) != valid_lane_set.end()) {
        AddConnectionInfo(lane_info->first, id, vector_net_ptr);
      }
    }
  }
}

}  // namespace prediction
}  // namespace TL
