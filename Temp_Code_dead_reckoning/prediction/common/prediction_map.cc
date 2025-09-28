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

#include "planning/prediction/common/prediction_map.h"
#include <algorithm>
#include <limits>
#include <tuple>
#include <unordered_set>
#include <utility>
#include "planning/prediction/common/prediction_gflags.h"

namespace TL {
namespace prediction {

using TL::common::math::Polygon2d;
using TL::common::math::Vec2d;
using TL::hdmap::HDMapUtil;
using TL::hdmap::JunctionInfo;
using TL::hdmap::LaneInfo;
using TL::hdmap::MapPathPoint;
using TL::hdmap::OverlapInfo;
using TL::hdmap::PNCJunctionInfo;
using TL::hdmap::RoadSection;

bool PredictionMap::Ready() {
  return HDMapUtil::MapPtrForPrediction() != nullptr;
}

std::string PredictionMap::MapModuleName() {
  return HDMapUtil::MapForPrediction().GetMapHeader().header().frame_id();
}

Eigen::Vector2d PredictionMap::PositionOnLane(
    const std::shared_ptr<const LaneInfo>& lane_info, const double s) {
  common::PointENU point = lane_info->GetSmoothPoint(s);
  return {point.x(), point.y()};
}

double PredictionMap::HeadingOnLane(
    const std::shared_ptr<const LaneInfo>& lane_info, const double s) {
  return lane_info->Heading(s);
}

double PredictionMap::CurvatureOnLane(const std::string& lane_id,
                                      const double s) {
  std::shared_ptr<const hdmap::LaneInfo> lane_info = LaneById(lane_id);
  if (lane_info == nullptr) {
    AERROR << "Null lane_info ptr found";
    return 0.0;
  }
  return lane_info->Curvature(s);
}

double PredictionMap::LaneTotalWidth(
    const std::shared_ptr<const hdmap::LaneInfo>& lane_info, const double s) {
  double left = 0.0;
  double right = 0.0;
  lane_info->GetWidth(s, &left, &right);
  return left + right;
}

std::shared_ptr<const LaneInfo> PredictionMap::LaneById(
    const std::string& str_id) {
  const auto& lane_info =
      HDMapUtil::MapForPrediction().GetLaneById(hdmap::MakeMapId(str_id));
  if (lane_info == nullptr || lane_info->total_length() < 0.1) {
    return nullptr;
  }
  return lane_info;
}

std::shared_ptr<const RoadSection> PredictionMap::GetLaneSection(
    const std::string& lane_id) {
  auto lane_info_ptr = LaneById(lane_id);
  if (nullptr == lane_info_ptr) {
    return nullptr;
  }
  const auto& section_id = lane_info_ptr->section_id().id();

  auto road_ptr =
      HDMapUtil::MapForPrediction().GetRoadById(lane_info_ptr->road_id());
  if (nullptr == road_ptr) {
    return nullptr;
  }

  const auto& sections = road_ptr->sections();
  for (const auto& section : sections) {
    if (section.id().id() == section_id) {
      return std::make_shared<RoadSection>(section);
    }
  }
  return nullptr;
}

bool PredictionMap::IsSectionLaneFromLeft2Right(
    const hdmap::RoadSection& section) {
  if (section.lane_id().empty()) {
    return false;
  }

  for (int i = 0; i < section.lane_id_size() - 1; ++i) {
    const auto lane_ptr = LaneById(section.lane_id(i).id());
    if (nullptr == lane_ptr ||
        lane_ptr->lane().right_neighbor_forward_lane_id().empty()) {
      continue;
    }
    for (const auto& right_lane_id :
         lane_ptr->lane().right_neighbor_forward_lane_id()) {
      if (right_lane_id.id() == section.lane_id(i + 1).id()) {
        return true;
      }
    }
  }

  return false;
}

std::string PredictionMap::GetRoadSectionCombinedId(
    const std::shared_ptr<const hdmap::LaneInfo>& lane_info) {
  if (nullptr == lane_info || lane_info->section_id().id().empty() ||
      lane_info->road_id().id().empty()) {
    return "";
  }
  return lane_info->road_id().id() + "," + lane_info->section_id().id();
}

bool PredictionMap::GetRoadAndSectionIdFromCombinedId(
    const std::string& combined_id, std::string* road_id,
    std::string* section_id) {
  if (combined_id.empty() || nullptr == road_id || nullptr == section_id) {
    return false;
  }
  (*road_id).clear();
  (*section_id).clear();

  // 找到所有逗号的位置
  std::vector<size_t> commaPositions = {};
  for (size_t pos = combined_id.find(','); pos != std::string::npos;
       pos = combined_id.find(',', pos)) {
    commaPositions.push_back(pos);
  }
  if (commaPositions.empty()) {
    return false;
  }
  *road_id = combined_id.substr(0, commaPositions.back());
  *section_id =
      combined_id.substr(commaPositions.back() + 1, combined_id.size());
  return !((*road_id).empty() || (*section_id).empty());
}

std::shared_ptr<const JunctionInfo> PredictionMap::JunctionById(
    const std::string& str_id) {
  return HDMapUtil::MapForPrediction().GetJunctionById(
      hdmap::MakeMapId(str_id));
}

std::shared_ptr<const OverlapInfo> PredictionMap::OverlapById(
    const std::string& str_id) {
  return HDMapUtil::MapForPrediction().GetOverlapById(hdmap::MakeMapId(str_id));
}

bool PredictionMap::GetProjection(
    const Eigen::Vector2d& pos,
    const std::shared_ptr<const LaneInfo>& lane_info, double* s, double* l) {
  if (lane_info == nullptr) {
    return false;
  }
  return lane_info->GetProjection({pos.x(), pos.y()}, s, l);
}

bool PredictionMap::GetProjectionWithIndex(
    const Eigen::Vector2d& pos,
    const std::shared_ptr<const LaneInfo>& lane_info, double* s, double* l,
    int* index) {
  if (lane_info == nullptr) {
    return false;
  }
  return lane_info->GetProjection({pos.x(), pos.y()}, s, l, index);
}

bool PredictionMap::GetProjection(const Eigen::Vector2d& position,
                                  const std::vector<std::string>& lane_ids,
                                  double* s, double* l, int* idx) {
  *s = 0.0;
  *l = 0.0;
  *idx = 0;

  if (nullptr == s || nullptr == l || nullptr == idx || lane_ids.empty()) {
    return false;
  }

  int min_idx = 0;
  double min_l = std::numeric_limits<double>::infinity();
  double min_s = std::numeric_limits<double>::infinity();
  for (int i = 0; i < lane_ids.size(); ++i) {
    const auto lane_info_ptr = LaneById(lane_ids.at(i));
    if (nullptr == lane_info_ptr) {
      continue;
    }
    if (lane_info_ptr->GetProjection({position.x(), position.y()}, s, l)) {
      if (std::isgreaterequal(*s, 0.0) &&
          std::islessequal(*s, lane_info_ptr->total_length())) {
        *idx = i;
        return true;
      }
      if (std::isless(std::fabs(*l), std::fabs(min_l))) {
        min_l = *l;
        min_s = *s;
        min_idx = i;
      }
    }
  }

  *l = min_l;
  *s = min_s;
  *idx = min_idx;

  return true;
}

bool PredictionMap::GetLaneSequenceS(const std::vector<std::string>& lane_ids,
                                     int idx, double s, double* seq_s) {
  *seq_s = 0.0;
  if (lane_ids.empty() || idx > lane_ids.size()) {
    return false;
  }

  for (int i = 0; i < idx; ++i) {
    const auto lane_info_ptr = LaneById(lane_ids.at(i));
    if (nullptr == lane_info_ptr) {
      continue;
    }
    *seq_s += lane_info_ptr->total_length();
  }
  *seq_s += s;

  return true;
}

bool PredictionMap::HasNearbyLane(const double x, const double y,
                                  const double radius) {
  common::PointENU point_enu;
  point_enu.set_x(x);
  point_enu.set_y(y);
  std::vector<std::shared_ptr<const LaneInfo>> lanes;
  HDMapUtil::MapForPrediction().GetLanes(point_enu, radius, &lanes);
  return (!lanes.empty());
}

bool PredictionMap::ProjectionFromLane(
    const std::shared_ptr<const LaneInfo>& lane_info, const double s,
    MapPathPoint* path_point) {
  if (lane_info == nullptr) {
    return false;
  }
  const common::PointENU point = lane_info->GetSmoothPoint(s);
  const double heading = HeadingOnLane(lane_info, s);
  path_point->set_x(point.x());
  path_point->set_y(point.y());
  path_point->set_heading(heading);
  return true;
}

bool PredictionMap::IsVirtualLane(const std::string& lane_id) {
  std::shared_ptr<const LaneInfo> lane_info =
      HDMapUtil::MapForPrediction().GetLaneById(hdmap::MakeMapId(lane_id));
  if (lane_info == nullptr) {
    return false;
  }
  const hdmap::Lane& lane = lane_info->lane();
  bool left_virtual = lane.has_left_boundary() &&
                      lane.left_boundary().has_virtual_() &&
                      lane.left_boundary().virtual_();
  bool right_virtual = lane.has_right_boundary() &&
                       lane.right_boundary().has_virtual_() &&
                       lane.right_boundary().virtual_();
  return left_virtual && right_virtual;
}

bool PredictionMap::OnVirtualLane(const Eigen::Vector2d& point,
                                  const double radius) {
  std::vector<std::shared_ptr<const LaneInfo>> lanes;
  common::PointENU hdmap_point;
  hdmap_point.set_x(point[0]);
  hdmap_point.set_y(point[1]);
  HDMapUtil::MapForPrediction().GetLanes(hdmap_point, radius, &lanes);

  bool hasVirtualLane = std::any_of(
      lanes.begin(), lanes.end(),
      [](const auto& lane) { return IsVirtualLane(lane->id().id()); });

  return hasVirtualLane;
}

int PredictionMap::GetNearestLane(const double x, const double y,
                                  std::shared_ptr<const LaneInfo>* nearest_lane,
                                  double* nearest_s, double* nearest_l) {
  common::PointENU hdmap_point;
  hdmap_point.set_x(x);
  hdmap_point.set_y(y);
  return HDMapUtil::MapForPrediction().GetNearestLane(hdmap_point, nearest_lane,
                                                      nearest_s, nearest_l);
}

void PredictionMap::OnLane(
    std::vector<std::string>* prev_lanes, const Eigen::Vector2d& point,
    const std::tuple<double, double, bool, int, double, double, double>& params,
    std::vector<std::shared_ptr<const LaneInfo>>* lanes) {
  double heading = std::get<0>(params);
  double radius = std::get<1>(params);
  bool on_lane = std::get<2>(params);
  int max_num_lane = std::get<3>(params);
  double max_lane_angle_diff = std::get<4>(params);
  double length = std::get<5>(params);
  double width = std::get<6>(params);

  bool use_length_width = length > 0. && width > 0.;

  std::vector<std::shared_ptr<const LaneInfo>> candidate_lanes;

  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  std::vector<std::string> lane_id_past_tmp;
  std::vector<std::string> lane_id_past_tmp_init;
  if (HDMapUtil::MapForPrediction().GetLanesWithHeading(
          hdmap_point, radius, heading, max_lane_angle_diff,
          &candidate_lanes) != 0) {
    return;
  }

  std::vector<std::pair<std::shared_ptr<const LaneInfo>, double>> lane_pairs;
  std::vector<std::pair<std::shared_ptr<const LaneInfo>, double>>
      lane_pairs_tmp;

  for (const auto& candidate_lane : candidate_lanes) {
    if (candidate_lane == nullptr) {
      continue;
    }

    bool is_obs_on_lane = false;
    if (use_length_width) {
      common::math::Box2d box({point.x(), point.y()}, heading, length, width);
      for (const auto corner : box.GetAllCorners()) {
        is_obs_on_lane = candidate_lane->IsOnLane(corner);
        if (is_obs_on_lane) {
          break;
        }
      }
    } else {
      is_obs_on_lane = candidate_lane->IsOnLane({point.x(), point.y()});
    }

    if (on_lane && !is_obs_on_lane) {
      continue;
    }
    // if (!IsIdenticalLane(candidate_lane, prev_lanes) &&
    //     !IsSuccessorLane(candidate_lane, prev_lanes) &&
    //     !IsLeftNeighborLane(candidate_lane, prev_lanes) &&
    //     !IsRightNeighborLane(candidate_lane, prev_lanes)) {
    //   continue;
    // }

    double distance = 0.0;
    common::PointENU nearest_point =
        candidate_lane->GetNearestPoint({point.x(), point.y()}, &distance);
    double nearest_point_heading = PathHeading(candidate_lane, nearest_point);
    double diff =
        std::fabs(common::math::AngleDiff(heading, nearest_point_heading));
    if (diff <= max_lane_angle_diff) {
      lane_pairs_tmp.emplace_back(candidate_lane, diff);

      // according to history position to decide current lanes
      std::string lane_id = candidate_lane->id().id();
      lane_id_past_tmp_init.push_back(lane_id);
      // if previous lane is included in current lanes, then keep the past lane;

      if (std::find(prev_lanes->begin(), prev_lanes->end(), lane_id) !=
          prev_lanes->end()) {
        lane_pairs.emplace_back(candidate_lane, diff);
        lane_id_past_tmp.push_back(lane_id);
        continue;
      }
      bool is_curr_lane = false;
      // if current lane is the successor lane of previous lane, then current
      // lane changes to the successor lane;
      for (const auto& predecessor_lane_id :
           candidate_lane->lane().predecessor_id()) {
        if (std::find(prev_lanes->begin(), prev_lanes->end(),
                      predecessor_lane_id.id()) != prev_lanes->end()) {
          // flag_current_success_past = true;
          lane_pairs.emplace_back(candidate_lane, diff);
          lane_id_past_tmp.push_back(lane_id);
          is_curr_lane = true;
          break;
        }
      }
      if (is_curr_lane) {
        continue;
      }

      // if current lane is the left lane of previous lane, then current lane
      // changes to the left lane;
      // only judge when type is not EMERGENCY_LANE
      if (candidate_lane->lane().has_type() &&
          candidate_lane->lane().type() == hdmap::Lane::EMERGENCY_LANE) {
        continue;
      }
      for (const auto& left_lane_id :
           candidate_lane->lane().left_neighbor_forward_lane_id()) {
        if (std::find(prev_lanes->begin(), prev_lanes->end(),
                      left_lane_id.id()) != prev_lanes->end()) {
          lane_pairs.emplace_back(candidate_lane, diff);
          lane_id_past_tmp.push_back(lane_id);
          is_curr_lane = true;
          break;
        }
      }
      if (is_curr_lane) {
        continue;
      }
      // if current lane is the right lane of previous lane, then current lane
      // changes to the right lane;
      for (const auto& right_lane_id :
           candidate_lane->lane().right_neighbor_forward_lane_id()) {
        if (std::find(prev_lanes->begin(), prev_lanes->end(),
                      right_lane_id.id()) != prev_lanes->end()) {
          lane_pairs.emplace_back(candidate_lane, diff);
          lane_id_past_tmp.push_back(lane_id);
          break;
        }
      }
    }
  }
  if (lane_pairs_tmp.empty()) {
    return;
  }

  *prev_lanes = lane_id_past_tmp;
  // if the current lanes has no relation with the previous lane，then update
  // current lane
  if (lane_pairs.empty()) {
    lane_pairs = lane_pairs_tmp;
    *prev_lanes = lane_id_past_tmp_init;
  }

  std::sort(lane_pairs.begin(), lane_pairs.end(),
            [](const std::pair<std::shared_ptr<const LaneInfo>, double>& p1,
               const std::pair<std::shared_ptr<const LaneInfo>, double>& p2) {
              return p1.second < p2.second;
            });

  int count = 0;
  for (const auto& lane_pair : lane_pairs) {
    bool is_repeated_lane = false;
    // 匝道附近有重复的lane，要过滤
    for (const std::shared_ptr<const LaneInfo>& tmp_lane_ptr : *lanes) {
      if (nullptr == lane_pair.first) {
        break;
      }
      double epsilon = common::math::kMathEpsilon;
      if (nullptr == tmp_lane_ptr ||
          std::fabs(tmp_lane_ptr->points().front().x() -
                    lane_pair.first->points().front().x()) > epsilon ||
          std::fabs(tmp_lane_ptr->points().back().y() -
                    lane_pair.first->points().back().y()) > epsilon ||
          std::fabs(tmp_lane_ptr->points().front().x() -
                    lane_pair.first->points().front().x()) > epsilon ||
          std::fabs(tmp_lane_ptr->points().back().y() -
                    lane_pair.first->points().back().y()) > epsilon) {
        continue;
      }
      is_repeated_lane = true;
      break;
    }
    if (is_repeated_lane) {
      continue;
    }

    lanes->push_back(lane_pair.first);
    ++count;
    if (count >= max_num_lane) {
      break;
    }
  }
}

std::shared_ptr<const LaneInfo> PredictionMap::GetMostLikelyCurrentLane(
    const common::PointENU& position, const double radius, const double heading,
    const double angle_diff_threshold) {
  std::vector<std::shared_ptr<const LaneInfo>> candidate_lanes;
  if (HDMapUtil::MapForPrediction().GetLanesWithHeading(
          position, radius, heading, angle_diff_threshold, &candidate_lanes) !=
      0) {
    return nullptr;
  }
  double min_angle_diff = 2.0 * M_PI;
  std::shared_ptr<const LaneInfo> curr_lane_ptr = nullptr;
  for (const auto& candidate_lane : candidate_lanes) {
    if (!candidate_lane->IsOnLane({position.x(), position.y()})) {
      continue;
    }
    double distance = 0.0;
    common::PointENU nearest_point = candidate_lane->GetNearestPoint(
        {position.x(), position.y()}, &distance);
    double nearest_point_heading = PathHeading(candidate_lane, nearest_point);
    double angle_diff =
        std::fabs(common::math::AngleDiff(heading, nearest_point_heading));
    if (angle_diff < min_angle_diff) {
      min_angle_diff = angle_diff;
      curr_lane_ptr = candidate_lane;
    }
  }
  return curr_lane_ptr;
}

bool PredictionMap::IsProjectionApproximateWithinLane(
    const Eigen::Vector2d& ego_position, const std::string& lane_id) {
  auto ptr_lane = LaneById(lane_id);
  if (nullptr == ptr_lane) {
    return false;
  }
  const auto& lane_points = ptr_lane->points();
  if (lane_points.size() < 2) {
    return false;
  }

  const auto& start_point = lane_points.front();
  const auto& end_point = lane_points.back();

  auto lane_vec = end_point - start_point;

  auto approx_lane_length = lane_vec.Length();
  if (approx_lane_length < 1.0e-3) {
    return false;
  }

  auto dist_vec =
      common::math::Vec2d(ego_position.x(), ego_position.y()) - start_point;

  auto projection_length = dist_vec.InnerProd(lane_vec) / approx_lane_length;

  return !std::signbit(projection_length) &&
         projection_length <= approx_lane_length;
}

bool PredictionMap::NearJunction(const Eigen::Vector2d& point,
                                 const double radius) {
  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  std::vector<std::shared_ptr<const JunctionInfo>> junctions;
  HDMapUtil::MapForPrediction().GetJunctions(hdmap_point, radius, &junctions);
  return !junctions.empty();
}

bool PredictionMap::IsPointInJunction(
    const double x, const double y,
    const std::shared_ptr<const JunctionInfo>& junction_info_ptr) {
  if (junction_info_ptr == nullptr) {
    return false;
  }
  const Polygon2d& polygon = junction_info_ptr->polygon();
  return polygon.IsPointIn({x, y});
}

std::vector<std::shared_ptr<const JunctionInfo>> PredictionMap::GetJunctions(
    const Eigen::Vector2d& point, const double radius) {
  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  std::vector<std::shared_ptr<const JunctionInfo>> junctions;
  HDMapUtil::MapForPrediction().GetJunctions(hdmap_point, radius, &junctions);
  return junctions;
}

std::vector<std::shared_ptr<const PNCJunctionInfo>>
PredictionMap::GetPNCJunctions(const Eigen::Vector2d& point,
                               const double radius) {
  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  std::vector<std::shared_ptr<const PNCJunctionInfo>> junctions;
  HDMapUtil::MapForPrediction().GetPNCJunctions(hdmap_point, radius,
                                                &junctions);
  return junctions;
}

bool PredictionMap::InJunction(double x, double y, double radius) {
  Eigen::Vector2d point(x, y);
  return InJunction(point, radius);
}

bool PredictionMap::InJunction(const Eigen::Vector2d& point,
                               const double radius) {
  auto junction_infos = GetJunctions(point, radius);
  if (junction_infos.empty()) {
    return false;
  }
  for (const auto& junction_info : junction_infos) {
    if (junction_info == nullptr || !junction_info->junction().has_polygon()) {
      continue;
    }
    std::vector<Vec2d> vertices;
    for (const auto& point : junction_info->junction().polygon().point()) {
      vertices.emplace_back(point.x(), point.y());
    }
    if (vertices.size() < 3) {
      continue;
    }
    Polygon2d junction_polygon{vertices};
    if (junction_polygon.IsPointIn({point.x(), point.y()})) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsLaneInJunction(
    const std::shared_ptr<const LaneInfo>& lane_info,
    const std::string& junction_id) {
  if (lane_info == nullptr) {
    return false;
  }
  if (!lane_info->id().has_id()) {
    return false;
  }

  for (const auto& over_id : lane_info->lane().overlap_id()) {
    auto overlap_info_ptr = PredictionMap::OverlapById(over_id.id());
    if (overlap_info_ptr == nullptr) {
      continue;
    }

    for (const auto& object : overlap_info_ptr->overlap().object()) {
      if (!object.has_junction_overlap_info()) {
        continue;
      }
      if (object.has_id() && object.id().has_id() &&
          junction_id == object.id().id()) {
        return true;
      }
    }
  }

  return false;
}

double PredictionMap::PathHeading(
    const std::shared_ptr<const LaneInfo>& lane_info,
    const common::PointENU& point) {
  double s = 0.0;
  double l = 0.0;
  if (lane_info->GetProjection({point.x(), point.y()}, &s, &l)) {
    return HeadingOnLane(lane_info, s);
  }
  return M_PI;
}

bool PredictionMap::SmoothPointFromLane(const std::string& id, const double s,
                                        const double l, Eigen::Vector2d* point,
                                        double* heading) {
  if (point == nullptr || heading == nullptr) {
    return false;
  }
  std::shared_ptr<const LaneInfo> lane = LaneById(id);
  if (lane == nullptr) {
    return false;
  }
  common::PointENU hdmap_point = lane->GetSmoothPoint(s);
  *heading = lane->Heading(s);
  point->x() = hdmap_point.x() - std::sin(*heading) * l;
  point->y() = hdmap_point.y() + std::cos(*heading) * l;
  return true;
}

void PredictionMap::NearbyLanesByCurrentLanes(
    const Eigen::Vector2d& point, const double heading, const double radius,
    const std::vector<std::shared_ptr<const LaneInfo>>& lanes,
    const int max_num_lane, const double extra_s,
    std::vector<std::shared_ptr<const LaneInfo>>* nearby_lanes) {
  if (lanes.empty()) {
    std::vector<std::string> prev_lanes;
    std::tuple<double, double, bool, int, double, double, double> params =
        std::make_tuple(heading, radius, false, max_num_lane,
                        FLAGS_max_lane_angle_diff, -1.0, -1.0);

    OnLane(&prev_lanes, point, params, nearby_lanes);
  } else {
    std::unordered_set<std::string> lane_ids;
    std::vector<std::shared_ptr<const LaneInfo>> left_nearby_lanes;
    std::vector<std::shared_ptr<const LaneInfo>> right_nearby_lanes;
    for (const auto& lane_ptr : lanes) {
      if (lane_ptr == nullptr) {
        continue;
      }
      for (const auto& lane_id :
           lane_ptr->lane().left_neighbor_forward_lane_id()) {
        const std::string& id = lane_id.id();
        if (lane_ids.find(id) != lane_ids.end()) {
          continue;
        }
        std::shared_ptr<const LaneInfo> nearby_lane = LaneById(id);
        if (nearby_lane == nullptr) {
          continue;
        }
        double s = -1.0;
        double l = 0.0;
        GetProjection(point, nearby_lane, &s, &l);
        if (s < 0.0 || s >= nearby_lane->total_length() + extra_s ||
            std::fabs(l) > radius) {
          continue;
        }
        lane_ids.insert(id);
        bool is_lane_id_in_currentlanes = false;
        // 如果lane_ptr已经在current_lanes里了，就不需要再添加到nearby_lanes里了
        for (const auto& lane_ptr : lanes) {
          if (lane_ptr->id().id() == id) {
            is_lane_id_in_currentlanes = true;
            break;
          }
        }
        if (!is_lane_id_in_currentlanes) {
          left_nearby_lanes.push_back(nearby_lane);
        }
      }
      for (const auto& lane_id :
           lane_ptr->lane().right_neighbor_forward_lane_id()) {
        const std::string& id = lane_id.id();
        if (lane_ids.find(id) != lane_ids.end()) {
          continue;
        }
        std::shared_ptr<const LaneInfo> nearby_lane = LaneById(id);
        if (nearby_lane == nullptr) {
          continue;
        }
        double s = -1.0;
        double l = 0.0;
        GetProjection(point, nearby_lane, &s, &l);
        if (s < 0.0 || s >= nearby_lane->total_length() + extra_s ||
            std::fabs(l) > radius) {
          continue;
        }
        lane_ids.insert(id);
        bool is_lane_id_in_currentlanes = false;
        // 如果lane_ptr已经在current_lanes里了，就不需要再添加到nearby_lanes里了
        for (const auto& lane_ptr : lanes) {
          if (lane_ptr->id().id() == id) {
            is_lane_id_in_currentlanes = true;
            break;
          }
        }
        if (!is_lane_id_in_currentlanes) {
          right_nearby_lanes.push_back(nearby_lane);
        }
      }
    }
    // ----3----|----6----1,3,4,6为nearby lane, 2和5为current lane
    // ----2----|----5----搜索时，若1不满足条件, 则nearby_lanes为3,6,4,
    // ----1----|----4----因数量限制只获取3,6, 都为左边车道, 因此做如下修改
    int left_lanes_size = static_cast<int>(left_nearby_lanes.size());
    int right_lanes_size = static_cast<int>(right_nearby_lanes.size());
    int idx = 0;
    for (; idx < std::min(left_lanes_size, right_lanes_size); ++idx) {
      nearby_lanes->push_back(left_nearby_lanes[idx]);
      nearby_lanes->push_back(right_nearby_lanes[idx]);
    }
    while (idx < left_lanes_size) {
      nearby_lanes->push_back(left_nearby_lanes[idx++]);
    }
    while (idx < right_lanes_size) {
      nearby_lanes->push_back(right_nearby_lanes[idx++]);
    }
  }
}

std::shared_ptr<const LaneInfo> PredictionMap::GetLeftNeighborLane(
    const std::shared_ptr<const LaneInfo>& ptr_ego_lane,
    const Eigen::Vector2d& ego_position, const double threshold) {
  std::vector<std::string> neighbor_ids;
  for (const auto& lane_id :
       ptr_ego_lane->lane().left_neighbor_forward_lane_id()) {
    neighbor_ids.push_back(lane_id.id());
  }

  return GetNeighborLane(ptr_ego_lane, ego_position, neighbor_ids, threshold);
}

std::shared_ptr<const LaneInfo> PredictionMap::GetRightNeighborLane(
    const std::shared_ptr<const LaneInfo>& ptr_ego_lane,
    const Eigen::Vector2d& ego_position, const double threshold) {
  std::vector<std::string> neighbor_ids;
  for (const auto& lane_id :
       ptr_ego_lane->lane().right_neighbor_forward_lane_id()) {
    neighbor_ids.push_back(lane_id.id());
  }

  return GetNeighborLane(ptr_ego_lane, ego_position, neighbor_ids, threshold);
}

std::shared_ptr<const LaneInfo> PredictionMap::GetNeighborLane(
    const std::shared_ptr<const LaneInfo>& ptr_ego_lane,
    const Eigen::Vector2d& ego_position,
    const std::vector<std::string>& neighbor_lane_ids, const double threshold) {
  double ego_s = 0.0;
  double ego_l = 0.0;
  GetProjection(ego_position, ptr_ego_lane, &ego_s, &ego_l);

  double s_diff_min = std::numeric_limits<double>::max();
  std::shared_ptr<const LaneInfo> ptr_lane_min = nullptr;

  for (const auto& lane_id : neighbor_lane_ids) {
    std::shared_ptr<const LaneInfo> ptr_lane = LaneById(lane_id);
    if (ptr_lane == nullptr) {
      continue;
    }
    double s = -1.0;
    double l = 0.0;
    GetProjection(ego_position, ptr_lane, &s, &l);

    double s_diff = std::fabs(s - ego_s);
    if (s_diff < s_diff_min) {
      s_diff_min = s_diff;
      ptr_lane_min = ptr_lane;
    }
  }

  if (s_diff_min > threshold) {
    return nullptr;
  }
  return ptr_lane_min;
}

std::vector<std::string> PredictionMap::NearbyLaneIds(
    const Eigen::Vector2d& point, const double radius) {
  std::vector<std::string> lane_ids;
  std::vector<std::shared_ptr<const LaneInfo>> lanes;
  common::PointENU hdmap_point;
  hdmap_point.set_x(point[0]);
  hdmap_point.set_y(point[1]);
  HDMapUtil::MapForPrediction().GetLanes(hdmap_point, radius, &lanes);
  lane_ids.reserve(lanes.size());
  for (const auto& lane : lanes) {
    lane_ids.push_back(lane->id().id());
  }
  return lane_ids;
}

bool PredictionMap::IsLeftNeighborLane(
    const std::shared_ptr<const LaneInfo>& target_lane,
    const std::shared_ptr<const LaneInfo>& curr_lane) {
  if (curr_lane == nullptr) {
    return true;
  }
  if (target_lane == nullptr) {
    return false;
  }
  if (!target_lane->id().has_id() || !curr_lane->id().has_id()) {
    return true;
  }
  bool hasMatchingLeftLane =
      std::any_of(curr_lane->lane().left_neighbor_forward_lane_id().begin(),
                  curr_lane->lane().left_neighbor_forward_lane_id().end(),
                  [target_lane](const auto& left_lane_id) {
                    return target_lane->id().id() == left_lane_id.id();
                  });

  return hasMatchingLeftLane;
}

bool PredictionMap::IsLeftNeighborLane(
    const std::shared_ptr<const LaneInfo>& target_lane,
    const std::vector<std::shared_ptr<const LaneInfo>>& lanes) {
  if (lanes.empty()) {
    return true;
  }
  bool hasMatchingLeftNeighbor =
      std::any_of(lanes.begin(), lanes.end(), [target_lane](const auto& lane) {
        return IsLeftNeighborLane(target_lane, lane);
      });

  return hasMatchingLeftNeighbor;
}

int PredictionMap::GetLeftNeighborLaneIdx(
    const std::shared_ptr<const hdmap::LaneInfo>& target_lane,
    const LaneSequence& lane_sequence) {
  for (int i = 0; i < lane_sequence.lane_segment_size(); ++i) {
    std::shared_ptr<const LaneInfo> lane =
        PredictionMap::LaneById(lane_sequence.lane_segment(i).lane_id());
    if (nullptr == lane) {
      return -1;
    }
    if (IsLeftNeighborLane(target_lane, lane)) {
      return i;
    }
  }
  return -1;
}

bool PredictionMap::IsRightNeighborLane(
    const std::shared_ptr<const LaneInfo>& target_lane,
    const std::shared_ptr<const LaneInfo>& curr_lane) {
  if (curr_lane == nullptr) {
    return true;
  }
  if (target_lane == nullptr) {
    return false;
  }
  bool hasMatchingRightLane =
      std::any_of(curr_lane->lane().right_neighbor_forward_lane_id().begin(),
                  curr_lane->lane().right_neighbor_forward_lane_id().end(),
                  [target_lane](const auto& right_lane_id) {
                    return target_lane->id().id() == right_lane_id.id();
                  });

  return hasMatchingRightLane;
}

bool PredictionMap::IsRightNeighborLane(
    const std::shared_ptr<const LaneInfo>& target_lane,
    const std::vector<std::shared_ptr<const LaneInfo>>& lanes) {
  if (lanes.empty()) {
    return true;
  }
  bool hasMatchingRightNeighbor =
      std::any_of(lanes.begin(), lanes.end(), [target_lane](const auto& lane) {
        return IsRightNeighborLane(target_lane, lane);
      });

  return hasMatchingRightNeighbor;
}

int PredictionMap::GetRightNeighborLaneIdx(
    const std::shared_ptr<const LaneInfo>& target_lane,
    const LaneSequence& lane_sequence) {
  for (int i = 0; i < lane_sequence.lane_segment_size(); ++i) {
    std::shared_ptr<const LaneInfo> lane =
        PredictionMap::LaneById(lane_sequence.lane_segment(i).lane_id());
    if (nullptr == lane) {
      return -1;
    }
    if (IsRightNeighborLane(target_lane, lane)) {
      return i;
    }
  }
  return -1;
}

bool PredictionMap::IsSuccessorLane(
    const std::shared_ptr<const LaneInfo>& target_lane,
    const std::shared_ptr<const LaneInfo>& curr_lane) {
  if (curr_lane == nullptr) {
    return false;
  }
  if (target_lane == nullptr) {
    return false;
  }
  if (!target_lane->id().has_id() || !curr_lane->id().has_id()) {
    return false;
  }
  bool hasMatchingSuccessor =
      std::any_of(curr_lane->lane().successor_id().begin(),
                  curr_lane->lane().successor_id().end(),
                  [target_lane](const auto& successor_lane_id) {
                    return target_lane->id().id() == successor_lane_id.id();
                  });

  return hasMatchingSuccessor;
}

bool PredictionMap::IsSuccessorLane(
    const std::shared_ptr<const LaneInfo>& target_lane,
    const std::vector<std::shared_ptr<const LaneInfo>>& lanes) {
  if (lanes.empty()) {
    return true;
  }
  bool isSuccessorlane =
      std::any_of(lanes.begin(), lanes.end(),
                  [&](const std::shared_ptr<const LaneInfo>& lane) {
                    return IsSuccessorLane(target_lane, lane);
                  });
  return isSuccessorlane;
}

bool PredictionMap::IsPredecessorLane(
    const std::shared_ptr<const LaneInfo>& target_lane,
    const std::shared_ptr<const LaneInfo>& curr_lane) {
  if (curr_lane == nullptr) {
    return true;
  }
  if (target_lane == nullptr) {
    return false;
  }
  if (!target_lane->id().has_id() || !curr_lane->id().has_id()) {
    return false;
  }
  bool hasPredecessorLane =
      std::any_of(curr_lane->lane().predecessor_id().begin(),
                  curr_lane->lane().predecessor_id().end(),
                  [&](const auto& predecessor_lane_id) {
                    return target_lane->id().id() == predecessor_lane_id.id();
                  });

  return hasPredecessorLane;
}

bool PredictionMap::IsPredecessorLane(
    const std::shared_ptr<const LaneInfo>& target_lane,
    const std::vector<std::shared_ptr<const LaneInfo>>& lanes) {
  if (lanes.empty()) {
    return true;
  }
  bool hasPredecessorLane = std::any_of(
      lanes.begin(), lanes.end(),
      [&](const auto& lane) { return IsPredecessorLane(target_lane, lane); });

  return hasPredecessorLane;
}

bool PredictionMap::IsIdenticalLane(
    const std::shared_ptr<const LaneInfo>& other_lane,
    const std::shared_ptr<const LaneInfo>& curr_lane) {
  if (curr_lane == nullptr || other_lane == nullptr) {
    return true;
  }
  if (!other_lane->id().has_id() || !curr_lane->id().has_id()) {
    return true;
  }
  return other_lane->id().id() == curr_lane->id().id();
}

bool PredictionMap::IsIdenticalLane(
    const std::shared_ptr<const LaneInfo>& other_lane,
    const std::vector<std::shared_ptr<const LaneInfo>>& lanes) {
  if (lanes.empty()) {
    return true;
  }
  bool hasIdenticalLane = std::any_of(
      lanes.begin(), lanes.end(),
      [&](const auto& lane) { return IsIdenticalLane(other_lane, lane); });

  return hasIdenticalLane;
}

int PredictionMap::LaneTurnType(const std::string& lane_id) {
  std::shared_ptr<const LaneInfo> lane = LaneById(lane_id);
  if (lane != nullptr) {
    return static_cast<int>(lane->lane().turn());
  }
  return 1;
}

std::vector<std::shared_ptr<const LaneInfo>> PredictionMap::GetNearbyLanes(
    const common::PointENU& position, const double nearby_radius) {
  std::vector<std::shared_ptr<const LaneInfo>> nearby_lanes;

  if (!position.has_x() || !position.has_y() || nearby_radius < 0.0) {
    AERROR << "Invalid position or nearby_radius.";
    return nearby_lanes;
  }

  HDMapUtil::MapForPrediction().GetLanes(position, nearby_radius,
                                         &nearby_lanes);
  return nearby_lanes;
}

std::shared_ptr<const LaneInfo> PredictionMap::LaneWithSide(
    const std::vector<std::shared_ptr<const LaneInfo>>& lane_infos,
    const bool is_left_first) {

  int size = static_cast<int>(lane_infos.size());
  if (size <= 0) {
    return nullptr;
  }
  if (1 == size) {
    return lane_infos[0];
  }
  std::shared_ptr<const hdmap::LaneInfo> selected_lane_info;
  if (is_left_first) {
    selected_lane_info = lane_infos[size - 1];
    if (selected_lane_info->lane().right_neighbor_forward_lane_id_size() > 0 &&
        selected_lane_info->lane().right_neighbor_forward_lane_id(0).id() ==
            lane_infos[size - 2]->lane().id().id()) {
      return selected_lane_info;
    }
  } else {
    selected_lane_info = lane_infos[0];
    if (selected_lane_info->lane().left_neighbor_forward_lane_id_size() > 0 &&
        selected_lane_info->lane().left_neighbor_forward_lane_id(0).id() ==
            lane_infos[1]->lane().id().id()) {
      return selected_lane_info;
    }
  }

  return nullptr;
}

std::shared_ptr<const LaneInfo> PredictionMap::LaneWithSmallestAverageCurvature(
    const std::vector<std::shared_ptr<const LaneInfo>>& lane_infos) {
  if (lane_infos.empty()) {
    AERROR << "Lane Vector is empty.";
    return nullptr;
  }
  size_t sample_size = FLAGS_sample_size_for_average_lane_curvature;
  std::shared_ptr<const hdmap::LaneInfo> selected_lane_info = lane_infos[0];
  if (selected_lane_info == nullptr) {
    AERROR << "Lane Vector first element: selected_lane_info is nullptr.";
    return nullptr;
  }
  double smallest_curvature =
      AverageCurvature(selected_lane_info->id().id(), sample_size);
  for (size_t i = 1; i < lane_infos.size(); ++i) {
    const std::shared_ptr<const hdmap::LaneInfo>& lane_info = lane_infos[i];
    if (lane_info == nullptr) {
      AWARN << "Lane vector element: one lane_info is nullptr.";
      continue;
    }
    double curvature = AverageCurvature(lane_info->id().id(), sample_size);
    if (curvature < smallest_curvature) {
      smallest_curvature = curvature;
      selected_lane_info = lane_info;
    }
  }
  return selected_lane_info;
}

std::shared_ptr<const LaneInfo> PredictionMap::LaneWithSmallestAngleDiff(
    const std::shared_ptr<const LaneInfo>& curr_lane_info,
    const std::vector<std::shared_ptr<const LaneInfo>>& lane_infos,
    size_t sample_size) {
  if (nullptr == curr_lane_info || lane_infos.empty()) {
    AERROR << "Lane Vector is empty.";
    return nullptr;
  }
  double curr_lane_end_angle =
      curr_lane_info->Heading(curr_lane_info->total_length());
  std::shared_ptr<const hdmap::LaneInfo> selected_lane_info = nullptr;
  double min_angle_diff = 2.0 * M_PI;
  for (const auto& lane_info : lane_infos) {
    if (lane_info == nullptr) {
      AWARN << "Lane vector element: one lane_info is nullptr.";
      continue;
    }
    double lane_length = lane_info->total_length();
    double s_gap = lane_length / static_cast<double>(sample_size);
    double angle_diff = 0.0;
    for (size_t i = 0; i < sample_size; ++i) {
      double s = s_gap * static_cast<double>(i);
      double sample_angle = lane_info->Heading(s);
      angle_diff +=
          std::fabs(common::math::AngleDiff(curr_lane_end_angle, sample_angle));
    }
    angle_diff /= static_cast<double>(sample_size);
    if (std::isless(angle_diff, min_angle_diff)) {
      min_angle_diff = angle_diff;
      selected_lane_info = lane_info;
    }
  }
  return selected_lane_info;
}

double PredictionMap::AverageCurvature(const std::string& lane_id,
                                       const size_t sample_size) {
  CHECK_GT(sample_size, 0U);
  std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr =
      PredictionMap::LaneById(lane_id);
  if (lane_info_ptr == nullptr) {
    return 0.0;
  }
  double lane_length = lane_info_ptr->total_length();
  double s_gap = lane_length / static_cast<double>(sample_size);
  double curvature_sum = 0.0;
  for (size_t i = 0; i < sample_size; ++i) {
    double s = s_gap * static_cast<double>(i);
    curvature_sum += std::abs(PredictionMap::CurvatureOnLane(lane_id, s));
  }
  return curvature_sum / static_cast<double>(sample_size);
}

int PredictionMap::GetLaneInSequenceIndex(const std::string& lane_id,
                                          const LaneSequence& lane_sequence) {
  for (int i = 0; i < lane_sequence.lane_segment_size(); ++i) {
    if (lane_sequence.lane_segment(i).lane_id() == lane_id) {
      return i;
    }
  }
  return -1;
}

bool PredictionMap::IsTwoSequenceHaveSameLane(
    const LaneSequence& lane_sequence, const LaneSequence& lane_sequence2) {
  for (const auto& lane_seg : lane_sequence.lane_segment()) {
    if (!lane_seg.has_lane_id()) {
      continue;
    }

    for (const auto& lane_seg2 : lane_sequence2.lane_segment()) {
      if (!lane_seg2.has_lane_id()) {
        continue;
      }

      if (lane_seg.lane_id() == lane_seg2.lane_id()) {
        return true;
      }
    }
  }
  return false;
}

bool PredictionMap::IsDrivableLane(const hdmap::Lane::LaneType& lane_type) {
  // Note that EMERGENCY_LANE is not included here,
  // sometimes Emergency_lane is drivable,sometimes not
  return lane_type != hdmap::Lane::NONE && lane_type != hdmap::Lane::SHOULDER &&
         lane_type != hdmap::Lane::PARKING &&
         lane_type != hdmap::Lane::SIDEWALK &&
         lane_type != hdmap::Lane::INVALID_LANE &&
         lane_type != hdmap::Lane::EMERGENCY_LANE;
}

bool PredictionMap::IsDrivableLaneSequence(const LaneSequence& lane_seq) {
  for (const auto& lane_seg : lane_seq.lane_segment()) {
    std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr =
        PredictionMap::LaneById(lane_seg.lane_id());
    if (nullptr == lane_info_ptr) {
      return false;
    }
    if (IsDrivableLane(lane_info_ptr->lane().type())) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsTwoSequenceSame(const LaneSequence& lane_sequence,
                                      const LaneSequence& lane_sequence2) {
  if (lane_sequence.lane_segment_size() != lane_sequence2.lane_segment_size()) {
    return false;
  }

  for (int i = 0; i < lane_sequence.lane_segment_size(); ++i) {
    const LaneSegment& lane_seg = lane_sequence.lane_segment(i);
    const LaneSegment& lane_seg2 = lane_sequence2.lane_segment(i);
    if (!lane_seg.has_lane_id() || !lane_seg2.has_lane_id()) {
      return false;
    }

    if (lane_seg.lane_id() != lane_seg2.lane_id()) {
      return false;
    }
  }

  return true;
}

// curr_seq successor is target_seq
bool PredictionMap::IsTwoSequenceConnected(const LaneSequence& target_seq,
                                           const LaneSequence& curr_seq) {
  if (target_seq.lane_segment_size() <= 0 ||
      curr_seq.lane_segment_size() <= 0) {
    return false;
  }

  const auto& curr_seg = curr_seq.lane_segment().rbegin();
  std::shared_ptr<const hdmap::LaneInfo> curr_lane =
      LaneById(curr_seg->lane_id());

  const auto& target_seg = target_seq.lane_segment(0);
  std::shared_ptr<const hdmap::LaneInfo> target_lane =
      LaneById(target_seg.lane_id());

  return IsSuccessorLane(target_lane, curr_lane);
}

// curr_seq contains target_seq
bool PredictionMap::IsSequenceContainsAnother(const LaneSequence& target_seq,
                                              const LaneSequence& curr_seq) {
  if (target_seq.lane_segment_size() <= 0 ||
      curr_seq.lane_segment_size() <= 0) {
    return false;
  }

  const auto& first_lane = target_seq.lane_segment(0);
  int idx = GetLaneInSequenceIndex(first_lane.lane_id(), curr_seq);
  if (idx < 0) {
    return false;
  }

  int remain_size = curr_seq.lane_segment_size() - idx;
  if (remain_size < target_seq.lane_segment_size()) {
    return false;
  }

  for (int j = 1; j < target_seq.lane_segment_size(); ++j) {
    if (target_seq.lane_segment(j).lane_id() !=
        curr_seq.lane_segment(j + idx).lane_id()) {
      return false;
    }
  }

  return true;
}

// target_seq extends curr_seq
bool PredictionMap::IsSequenceExtendsAnother(const LaneSequence& target_seq,
                                             const LaneSequence& curr_seq) {
  if (target_seq.lane_segment_size() <= 0 ||
      curr_seq.lane_segment_size() <= 0) {
    return false;
  }

  const auto& first_lane = target_seq.lane_segment(0);
  int idx = GetLaneInSequenceIndex(first_lane.lane_id(), curr_seq);
  if (idx < 0) {
    return false;
  }

  int remain_size = curr_seq.lane_segment_size() - idx;
  if (remain_size >= target_seq.lane_segment_size()) {
    return false;
  }

  for (int j = 1; j < remain_size; ++j) {
    if (target_seq.lane_segment(j).lane_id() !=
        curr_seq.lane_segment(j + idx).lane_id()) {
      return false;
    }
  }

  return true;
}

bool PredictionMap::GetProjLanePointHeading(const LaneSequence& lane_sequence,
                                            double* heading) {
  *heading = 0.0;

  if (lane_sequence.has_adc_lane_segment_idx()) {
    int lane_seg_start_idx = lane_sequence.adc_lane_segment_idx();
    if (!lane_sequence.lane_segment(lane_seg_start_idx).lane_point().empty()) {
      int lane_point_start_idx =
          lane_sequence.lane_segment(lane_seg_start_idx).adc_lane_point_idx();
      if (lane_point_start_idx >=
          lane_sequence.lane_segment(lane_seg_start_idx).lane_point_size()) {
        lane_point_start_idx =
            lane_sequence.lane_segment(lane_seg_start_idx).lane_point_size() -
            1;
      }
      const LanePoint& start_lane_point =
          lane_sequence.lane_segment(lane_seg_start_idx)
              .lane_point(lane_point_start_idx);
      *heading = start_lane_point.heading();
      return true;
    }
  }
  if (lane_sequence.has_lane_s()) {
    if (lane_sequence.lane_segment_size() <= 0) {
      return false;
    }
    const auto& seg = lane_sequence.lane_segment(0);
    if (seg.lane_point_size() <= 0) {
      return false;
    }
    *heading = seg.lane_point(0).heading();
    return true;
  }

  return false;
}

bool PredictionMap::GetProjLanePointWidth(const LaneSequence& lane_sequence,
                                          double* width) {
  *width = 0.0;

  if (lane_sequence.has_adc_lane_segment_idx()) {
    int lane_seg_start_idx = lane_sequence.adc_lane_segment_idx();
    if (!lane_sequence.lane_segment(lane_seg_start_idx).lane_point().empty()) {
      int lane_point_start_idx =
          lane_sequence.lane_segment(lane_seg_start_idx).adc_lane_point_idx();
      if (lane_point_start_idx >=
          lane_sequence.lane_segment(lane_seg_start_idx).lane_point_size()) {
        lane_point_start_idx =
            lane_sequence.lane_segment(lane_seg_start_idx).lane_point_size() -
            1;
      }
      const LanePoint& start_lane_point =
          lane_sequence.lane_segment(lane_seg_start_idx)
              .lane_point(lane_point_start_idx);
      *width = start_lane_point.width();
      return true;
    }
  }
  if (lane_sequence.lane_segment_size() <= 0) {
    return false;
  }
  const auto& seg = lane_sequence.lane_segment(0);
  if (seg.lane_point_size() <= 0) {
    return false;
  }
  *width = seg.lane_point(0).width();

  return true;
}

bool PredictionMap::IsPolyInLaneSequenceLatRange(
    const LaneSequence& lane_sequence, double lat_dist = 0.0) {
  double width = 0.0;
  if (!GetProjLanePointWidth(lane_sequence, &width)) {
    return false;
  }
  double lat_range = width / 2.0 + lat_dist;
  double min_l = lane_sequence.poly_min_l();
  double max_l = lane_sequence.poly_max_l();

  return std::fabs(max_l) < lat_range || std::fabs(min_l) < lat_range;
}

bool PredictionMap::IsEmergencyLaneSequence(const LaneSequence& lane_sequence) {
  for (const auto& lane_seg : lane_sequence.lane_segment()) {
    std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr =
        PredictionMap::LaneById(lane_seg.lane_id());
    if (nullptr == lane_info_ptr) {
      return false;
    }
    if (lane_info_ptr->lane().type() == hdmap::Lane::EMERGENCY_LANE) {
      return true;
    }
  }

  return false;
}

bool PredictionMap::IsInMergeLaneSequence(const LaneGraph& lane_graph) {
  bool has_merge_seq = std::any_of(
      lane_graph.lane_sequence().begin(), lane_graph.lane_sequence().end(),
      [](const auto& lane_seq) { return lane_seq.has_merge_lane_idx(); });

  return has_merge_seq;
}

bool PredictionMap::IsLaneLineSolid(
    bool left_side, double s,
    const std::shared_ptr<const LaneInfo>& lane_info) {
  if (nullptr == lane_info) {
    return false;
  }
  const auto& waypoint = hdmap::LaneWaypoint(lane_info, s);

  if (left_side) {
    const auto& boundary_type = LeftBoundaryType(waypoint);
    if (boundary_type != hdmap::LaneBoundaryType::UNKNOWN &&
        boundary_type != hdmap::LaneBoundaryType::DOTTED_WHITE &&
        boundary_type != hdmap::LaneBoundaryType::DOTTED_YELLOW) {
      return true;
    }
  } else {
    const auto& boundary_type = RightBoundaryType(waypoint);
    if (boundary_type != hdmap::LaneBoundaryType::UNKNOWN &&
        boundary_type != hdmap::LaneBoundaryType::DOTTED_WHITE &&
        boundary_type != hdmap::LaneBoundaryType::DOTTED_YELLOW) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::GetLaneIdFromLaneSequenceByS(
    const LaneSequence& lane_sequence, double s, std::string* lane_id) {
  *lane_id = "";
  double tmp_s = s;
  for (const auto& lane_seg : lane_sequence.lane_segment()) {
    double lane_len = lane_seg.total_length();
    if (std::isgreaterequal(tmp_s, 0.0) && std::islessequal(tmp_s, lane_len)) {
      *lane_id = lane_seg.lane_id();
      return true;
    }
    tmp_s -= lane_len;
  }
  return false;
}

bool PredictionMap::LaneSequenceContainsEgoLaneSequence(
    const LaneSequence& lane_sequence, const LaneGraph& lane_graph) {
  for (const auto& ego_seq : lane_graph.lane_sequence()) {
    if (!ego_seq.vehicle_on_lane()) {
      continue;
    }
    for (const auto& ego_lane : ego_seq.lane_segment()) {
      if (std::any_of(lane_sequence.lane_segment().begin(),
                      lane_sequence.lane_segment().end(),
                      [&](const auto& lane_seg) {
                        return lane_seg.lane_id() == ego_lane.lane_id();
                      })) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace prediction
}  // namespace TL
