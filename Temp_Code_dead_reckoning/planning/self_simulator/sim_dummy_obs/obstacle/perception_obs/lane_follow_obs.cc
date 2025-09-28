/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */
#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/lane_follow_obs.h"

#include <algorithm>
#include <list>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "common/file/log.h"
#include "common/math/vec2d.h"

namespace TL {
namespace simdummy {

// Custom helper functions for sorting purpose.
bool HeadingIsAtLeft(const std::vector<double>& heading1,
                     const std::vector<double>& heading2, const size_t idx) {
  if (idx >= heading1.size() || idx >= heading2.size()) {
    return true;
  }
  const double angle =
      TL::common::math::NormalizeAngle(heading1[idx] - heading2[idx]);
  if (angle > 0.0) {
    return true;
  } else if (angle < 0.0) {
    return false;
  }
  return HeadingIsAtLeft(heading1, heading2, idx + 1);
}

int ConvertTurnTypeToDegree(const hdmap::Lane& lane) {
  // Sanity checks.
  if (!lane.has_turn()) {
    return 0;
  }

  // Assign a number to measure how much it is bent to the left.
  switch (lane.turn()) {
    case hdmap::Lane::NO_TURN:
      return 0;
    case hdmap::Lane::LEFT_TURN:
      return 1;
    case hdmap::Lane::U_TURN:
      return 2;
    default:
      break;
  }

  return -1;
}

bool IsAtLeft(std::shared_ptr<const hdmap::LaneInfo> lane1,
              std::shared_ptr<const hdmap::LaneInfo> lane2) {
  if (lane1->lane().has_turn() && lane2->lane().has_turn() &&
      lane1->lane().turn() != lane2->lane().turn()) {
    int degree_to_left_1 = ConvertTurnTypeToDegree(lane1->lane());
    int degree_to_left_2 = ConvertTurnTypeToDegree(lane2->lane());
    return degree_to_left_1 > degree_to_left_2;
  }
  return HeadingIsAtLeft(lane1->headings(), lane2->headings(), 0);
}

void LaneFollowObs::ConstructLaneSequence(
    const bool search_forward_direction, const double accumulated_s,
    const double curr_lane_seg_s,
    std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr,
    const int graph_search_horizon, const bool consider_lane_split,
    std::list<prediction::LaneSegment>* const lane_segments,
    prediction::LaneGraph* const lane_graph_ptr) const {
  // Sanity checks.
  if (lane_info_ptr == nullptr) {
    AERROR << "Invalid lane.";
    return;
  }
  if (graph_search_horizon < 0) {
    AERROR << "The lane search has already reached the limits";
    AERROR << "Possible map error found!";
    return;
  }

  // Create a new lane_segment based on the current lane_info_ptr.
  double curr_s =
      curr_lane_seg_s >= 0.0 ? curr_lane_seg_s : lane_info_ptr->total_length();
  prediction::LaneSegment lane_segment;
  lane_segment.set_adc_s(curr_s);
  lane_segment.set_lane_id(lane_info_ptr->id().id());
  lane_segment.set_total_length(lane_info_ptr->total_length());
  if (search_forward_direction) {
    lane_segment.set_start_s(curr_s);
    lane_segment.set_end_s(std::fmin(curr_s + length_ - accumulated_s,
                                     lane_info_ptr->total_length()));
  } else {
    lane_segment.set_start_s(
        std::fmax(0.0, curr_s - (length_ - accumulated_s)));
    lane_segment.set_end_s(curr_s);
  }
  if (search_forward_direction) {
    lane_segments->push_back(std::move(lane_segment));
  } else {
    lane_segments->push_front(std::move(lane_segment));
  }

  // End condition: if search reached the maximum search distance,
  // or if there is no more successor lane_segment.
  if (search_forward_direction) {
    if (lane_segments->back().end_s() < lane_info_ptr->total_length() ||
        lane_info_ptr->lane().successor_id().empty()) {
      prediction::LaneSequence* sequence = lane_graph_ptr->add_lane_sequence();
      for (const auto& it : *lane_segments) {
        *(sequence->add_lane_segment()) = it;
      }
      lane_segments->pop_back();
      return;
    }
  } else {
    if (lane_segments->front().start_s() > 0.0 ||
        lane_info_ptr->lane().predecessor_id().empty()) {
      prediction::LaneSequence* sequence = lane_graph_ptr->add_lane_sequence();
      for (const auto& it : *lane_segments) {
        *(sequence->add_lane_segment()) = it;
      }
      lane_segments->pop_front();
      return;
    }
  }

  // Otherwise, continue searching for subsequent lane_segments.
  double new_accumulated_s = 0.0;
  double new_lane_seg_s = 0.0;
  std::vector<std::shared_ptr<const hdmap::LaneInfo>> candidate_lanes;
  std::set<std::string> set_lane_ids;
  if (search_forward_direction) {
    new_accumulated_s = accumulated_s + lane_info_ptr->total_length() - curr_s;
    // Reundancy removal.
    for (const auto& successor_lane_id : lane_info_ptr->lane().successor_id()) {
      set_lane_ids.insert(successor_lane_id.id());
    }
    for (const auto& unique_id : set_lane_ids) {
      auto candidate_lane = map_ptr_->GetLaneById(hdmap::MakeMapId(unique_id));
      if (candidate_lane != nullptr) {
        candidate_lanes.push_back(candidate_lane);
      }
    }
    // Sort the successor lane_segments from left to right.
    std::sort(candidate_lanes.begin(), candidate_lanes.end(), IsAtLeft);
    // Based on other conditions, select what successor lanes should be used.
    // if (!consider_lane_split && !candidate_lanes.empty()) {
    //   candidate_lanes = {
    //       PredictionMap::LaneWithSmallestAverageCurvature(candidate_lanes)};
    // }
  } else {
    new_accumulated_s = accumulated_s + curr_s;
    new_lane_seg_s = -0.1;
    // Redundancy removal.
    for (const auto& predecessor_lane_id :
         lane_info_ptr->lane().predecessor_id()) {
      set_lane_ids.insert(predecessor_lane_id.id());
    }
    for (const auto& unique_id : set_lane_ids) {
      candidate_lanes.push_back(
          map_ptr_->GetLaneById(hdmap::MakeMapId(unique_id)));
    }
  }
  bool consider_further_lane_split = false;
  // Recursively expand lane-sequence.
  for (const auto& candidate_lane : candidate_lanes) {
    ConstructLaneSequence(search_forward_direction, new_accumulated_s,
                          new_lane_seg_s, candidate_lane,
                          graph_search_horizon - 1, consider_further_lane_split,
                          lane_segments, lane_graph_ptr);
  }
  if (search_forward_direction) {
    lane_segments->pop_back();
  } else {
    lane_segments->pop_front();
  }
}

void LaneFollowObs::BuildLaneGraph() {
  double x = percep_obs_.position().x();
  double y = percep_obs_.position().y();
  lane_graph_.Clear();

  common::PointENU ENU_point;
  ENU_point.set_x(x);
  ENU_point.set_y(y);
  std::vector<std::shared_ptr<const TL::hdmap::LaneInfo>> candidate_lanes;
  map_ptr_->GetLanes(ENU_point, 0.5, &candidate_lanes);

  if (candidate_lanes.empty() || candidate_lanes.front() == nullptr) {
    SetIsNeededRemove(true);
    setIsDisplay(false);
    return;
  }

  std::shared_ptr<const hdmap::LaneInfo> lane_info = candidate_lanes.front();

  current_lane_id_ = lane_info->lane().id().id();
  lane_info->GetProjection({x, y}, &current_s_, &current_l_);

  std::list<prediction::LaneSegment> lane_segments;
  double accumulated_s = 0.0;
  ConstructLaneSequence(true, accumulated_s, current_s_, lane_info, 20, false,
                        &lane_segments, &lane_graph_);
}

bool LaneFollowObs::StartDisplay(const DummyObsInputDataBase& data) {
  return true;
}

bool LaneFollowObs::StopDisplay(const DummyObsInputDataBase& data) {
  SetIsNeededRemove(false);
  return false;
}

void LaneFollowObs::UpdateState(const DummyObsInputDataBase& data) {
  // 匀速test
  map_ptr_ = data.map_ptr_;

  BuildLaneGraph();

  double delta_s = v_ * 0.1;
  current_s_ += delta_s;

  if (!is_display_) {
    return;
  }

  std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr =
      map_ptr_->GetLaneById(hdmap::MakeMapId(current_lane_id_));

  if (lane_info_ptr == nullptr) {
    SetIsNeededRemove(true);
    setIsDisplay(false);
    return;
  }

  int lane_segment_idx = 0;
  while (current_s_ > lane_info_ptr->total_length()) {
    lane_segment_idx++;
    if (!lane_graph_.lane_sequence().empty() &&
        lane_graph_.lane_sequence(0).lane_segment_size() > lane_segment_idx) {
      current_lane_id_ =
          lane_graph_.lane_sequence(0).lane_segment(lane_segment_idx).lane_id();
      current_s_ = current_s_ - lane_info_ptr->total_length();
      lane_info_ptr = map_ptr_->GetLaneById(hdmap::MakeMapId(current_lane_id_));
    } else {
      SetIsNeededRemove(true);
      setIsDisplay(false);
      return;
    }
  }

  common::PointENU hdmap_point = lane_info_ptr->GetSmoothPoint(current_s_);

  // 距离太远可以删了，正后面快撞了的的删除
  common::math::Vec2d obs_pos(hdmap_point.x(), hdmap_point.y());
  common::math::Vec2d ego_pos(data.adc_position_->x(), data.adc_position_->y());
  double adc_s = 0.0;
  double adc_l = 0.0;
  lane_info_ptr->GetProjection(
      {data.adc_position_->x(), data.adc_position_->y()}, &adc_s, &adc_l);
  if (adc_s < current_s_ + 5.0 && adc_s > current_s_ - 8.0 &&
      fabs(adc_l - current_l_) < 2.0) {
    SetIsNeededRemove(true);
    setIsDisplay(false);
    return;
  }

  double distance_to_ego = obs_pos.DistanceTo(ego_pos);
  if (distance_to_ego > 300) {
    SetIsNeededRemove(true);
    setIsDisplay(false);
    return;
  }

  double heading = lane_info_ptr->Heading(current_s_);

  percep_obs_.set_theta(heading);
  percep_obs_.mutable_position()->set_x(hdmap_point.x());
  percep_obs_.mutable_position()->set_y(hdmap_point.y());
  percep_obs_.mutable_position()->set_z(1);
  percep_obs_.mutable_velocity()->set_x(v_ * cos(heading));
  percep_obs_.mutable_velocity()->set_y(v_ * sin(heading));
}
}  // namespace simdummy
}  // namespace TL
