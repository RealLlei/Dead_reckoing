/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/navigation_lane_map_creator.h"
#include <cstdint>

#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
// #include "common/log.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/util/message_util.h"
#include "common/util/points_downsampler.h"
#include "common/util/util.h"
#include "planning/common/planning_gflags.h"
#include "planning/pnc_map/pnc_map.h"

#include "proto/common/types.pb.h"
#include "proto/map/map_lane.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {
using TL::common::util::DownsampleCurve;
using TL::hdmap::Lane;
using TL::common::util::operator+;  // NOLINT
using TL::common::Clock;
using TL::common::Status;

Status NavigationLaneMapCreator::Init() {
  change_type_diff_debounce_.ResetTime(3, 0, 0.1);
  return Status::OK();
}

NavigationLaneMapCreator::NavigationLaneMapCreator(
    const planning::PerceptionMapConfig& config)
    : config_(config) {}

void NavigationLaneMapCreator::SetChangeLaneType(
    const routing::PerceptionChangeLaneTypes& change_lane_type) {
  input_change_lane_types_ = change_lane_type;
}

bool NavigationLaneMapCreator::CreateMap(
    const std::list<NaviPathTuple>& navigation_path_list,
    navigation_hdmap::MapMsg* const map_msg) {
  auto* navigation_path = map_msg->mutable_navigation_path();
  auto* hdmap = map_msg->mutable_hdmap();
  const auto& lane_markers = map_msg->lane_marker();
  creat_map_time_.Clear();
  // If no navigation path is generated based on navigation lines, we try to
  // create map with "current_navi_path_tuple_" which is generated based on
  // perceived lane markers.
  if (navigation_path_list.empty()) {
    return false;
  }
  ChangeLaneTypeDecider(lane_markers);
  // double list_one_y =
  // std::get<3>(*navigation_path_list_.begin())->path().path_point().at(0).y();
  // double list_two_y =
  // std::get<3>(*(navigation_path_list_.begin()++))->path().path_point().at(0).y();

  FLAGS_navigation_hdmap_generate_left_boundray = true;
  for (auto iter = navigation_path_list.cbegin();
       iter != navigation_path_list.cend(); ++iter) {
    std::size_t index = std::distance(navigation_path_list.cbegin(), iter);
    if (!CreateSingleLaneMap(*iter, lane_markers, hdmap, navigation_path)) {
      AWARN << "Failed to generate lane: " << index;
      FLAGS_navigation_hdmap_generate_left_boundray = true;
      continue;
    }
    // double list_y = std::get<3>(*iter)->path().path_point().at(0).y();
    // double path_central_y =
    // (*navigation_path)["0_current"].path().path_point().at(0).y(); double
    // path_left_y = 0.0; if(index > 0){
    //   path_left_y =
    //   (*navigation_path)["0_left"].path().path_point().at(0).y();
    // }

    FLAGS_navigation_hdmap_generate_left_boundray = true;
    // The left border of the middle lane uses the right border of the left
    // lane.
    // int lane_index = static_cast<int>(index) - fail_num;
    // if (lane_index > 0) {
    //   auto* left_boundary =
    //       hdmap->mutable_lane(lane_index)->mutable_left_boundary();
    //   left_boundary->CopyFrom(hdmap->lane(lane_index - 1).right_boundary());
    //   auto* left_sample =
    //       hdmap->mutable_lane(lane_index)->mutable_left_sample();
    //   left_sample->CopyFrom(hdmap->lane(lane_index - 1).right_sample());
    // }
  }
  // Set routing info
  auto* routing = map_msg->mutable_routing();
  return SetRoutingAndRoad(routing, hdmap);
}

bool NavigationLaneMapCreator::SetRoutingAndRoad(
    TL::routing::RoutingResponse* inrouting, TL::hdmap::Map* hd_map) {
  // Set road boundary
  int lane_num = hd_map->lane_size();
  if (lane_num < 1) {
    return false;
  }
  ADEBUG << "The Lane number is: " << lane_num;
  auto* road = hd_map->add_road();
  road->mutable_id()->set_id("road_navigation");
  auto* section = road->add_section();
  for (int i = 0; i < lane_num; ++i) {
    auto* lane_id = section->add_lane_id();
    lane_id->CopyFrom(hd_map->lane(i).id());
  }
  auto* outer_polygon = section->mutable_boundary()->mutable_outer_polygon();
  auto* left_edge = outer_polygon->add_edge();
  left_edge->set_type(TL::hdmap::BoundaryEdge::LEFT_BOUNDARY);
  left_edge->mutable_curve()->CopyFrom(hd_map->lane(0).left_boundary().curve());

  auto* right_edge = outer_polygon->add_edge();
  right_edge->set_type(TL::hdmap::BoundaryEdge::RIGHT_BOUNDARY);
  right_edge->mutable_curve()->CopyFrom(
      hd_map->lane(lane_num - 1).right_boundary().curve());
  // Set routing info
  auto* routing_road = inrouting->add_road();
  routing_road->set_id(road->id().id());
  routing_road->mutable_passage()->Reserve(lane_num);
  for (int i = 0; i < lane_num; ++i) {
    auto* passage = routing_road->add_passage();
    auto change_lane_type = routing::ChangeLaneType::FORWARD;
    for (const auto& lanechange :
         current_change_lane_types_.per_change_lane_type()) {
      if (lanechange.id() == hd_map->lane(i).id().id()) {
        change_lane_type = lanechange.change_lane_type();
        break;
      }
    }
    passage->set_can_exit(true);
    passage->set_change_lane_type(change_lane_type);
    auto* segment = passage->add_segment();
    segment->set_id(hd_map->lane(i).id().id());
    segment->set_start_s(0.0);
    segment->set_end_s(hd_map->lane(i).length());

    auto adc_lane_segment_points =
        hd_map->lane(i).central_curve().segment().at(0).line_segment().point();
    common::PointENU start_point = adc_lane_segment_points.at(0);
    int max_index = adc_lane_segment_points.size() - 1;
    common::PointENU end_point = adc_lane_segment_points.at(max_index);
    auto* routing_request = inrouting->mutable_routing_request();
    routing::LaneWaypoint waypoint;
    waypoint.set_id(hd_map->lane(i).id().id());
    waypoint.mutable_pose()->set_x(start_point.x());
    waypoint.mutable_pose()->set_y(start_point.y());
    waypoint.set_s(0.0);
    routing_request->add_waypoint()->CopyFrom(waypoint);
    waypoint.set_s(hd_map->lane(i).length());
    waypoint.mutable_pose()->set_x(end_point.x());
    waypoint.mutable_pose()->set_y(end_point.y());
    routing_request->add_waypoint()->CopyFrom(waypoint);
  }

  // switch (current_change_lane_type_) {
  //   case routing::ChangeLaneType::LEFT:
  //     ADEBUG << "***********changetypeleft";
  //     for (int i = 0; i < lane_num; ++i) {
  //       if (absl::EndsWith(hd_map->lane(i).id().id(), "left")) {
  //         AddOtherPassage(hd_map, inrouting, i);
  //       }
  //     }
  //     break;
  //   case routing::ChangeLaneType::RIGHT:
  //     ADEBUG << "***********changetypeRIGHT";
  //     for (int i = 0; i < lane_num; ++i) {
  //       if (absl::EndsWith(hd_map->lane(i).id().id(), "right")) {
  //         AddOtherPassage(hd_map, inrouting, i);
  //       }
  //     }
  //     break;
  //   case routing::ChangeLaneType::FORWARD:
  //     ADEBUG << "***********changetypeFORWARD";
  //     break;
  //   default:
  //     break;
  // }

  common::util::FillHeader("from_navigation_routing", inrouting);
  // Set neighbor information for each lane
  if (lane_num < 2) {
    return true;
  }
  for (int i = 0; i < lane_num; ++i) {
    auto* lane = hd_map->mutable_lane(i);
    if (i > 0) {
      lane->add_left_neighbor_forward_lane_id()->CopyFrom(
          hd_map->lane(i - 1).id());
      ADEBUG << "Left neighbor is: " << hd_map->lane(i - 1).id().id();
    }
    if (i < lane_num - 1) {
      lane->add_right_neighbor_forward_lane_id()->CopyFrom(
          hd_map->lane(i + 1).id());
      ADEBUG << "Right neighbor is: " << hd_map->lane(i + 1).id().id();
    }
  }
  ADEBUG << "hdmap lane number " << hd_map->lane_size();
  return true;
}

bool NavigationLaneMapCreator::CreateSingleLaneMap(
    const NaviPathTuple& navi_path_tuple,
    const perception::LaneMarkers& lane_marker, hdmap::Map* const hdmap,
    google::protobuf::Map<std::string, navigation_hdmap::NavigationPath>* const
        navigation_path) {
  static uint8_t lane_id_count{0};
  lane_id_count++;
  UNUSED(lane_marker);
  CHECK_NOTNULL(hdmap);
  CHECK_NOTNULL(navigation_path);

  const auto& navi_path = std::get<3>(navi_path_tuple);
  const auto& path = navi_path->path();
  const auto& left_boundary_path = std::get<4>(navi_path_tuple);
  const auto& right_boundary_path = std::get<5>(navi_path_tuple);
  if (path.path_point_size() < 2) {
    AERROR << "The path length of line index is invalid";
    return false;
  }
  auto* lane = hdmap->add_lane();
  lane->mutable_id()->set_id(absl::StrCat(navi_path->path_priority(), "_",
                                          path.name(), "_", lane_id_count));
  (*navigation_path)[lane->id().id()] = *navi_path;
  // lane types
  lane->set_type(Lane::CITY_DRIVING);
  lane->set_turn(Lane::NO_TURN);
  lane->mutable_map_lane_type()->set_tunnel_lane(adc_is_in_tunnel_);

  // speed limit
  lane->set_speed_limit(hdmap_lane_speed_limit_ > 0.1
                            ? hdmap_lane_speed_limit_
                            : config_.default_speed_limit());

  // center line
  auto* curve_segment = lane->mutable_central_curve()->add_segment();
  curve_segment->set_heading(path.path_point(0).theta());
  curve_segment->set_length(path.path_point(path.path_point_size() - 1).s());
  lane->set_length(path.path_point(path.path_point_size() - 1).s());
  auto* line_segment = curve_segment->mutable_line_segment();

  // left boundary
  hdmap::LineSegment* left_segment = nullptr;
  if (FLAGS_navigation_hdmap_generate_left_boundray) {
    auto* left_boundary = lane->mutable_left_boundary();
    auto* left_boundary_type = left_boundary->add_boundary_type();
    left_boundary->set_virtual_(false);
    left_boundary_type->set_s(0.0);
    left_boundary_type->add_types(std::get<6>(navi_path_tuple).first);
    left_segment =
        left_boundary->mutable_curve()->add_segment()->mutable_line_segment();
  }

  // right boundary
  auto* right_boundary = lane->mutable_right_boundary();
  auto* right_boundary_type = right_boundary->add_boundary_type();
  right_boundary->set_virtual_(false);
  right_boundary_type->set_s(0.0);
  right_boundary_type->add_types(std::get<6>(navi_path_tuple).second);
  auto* right_segment =
      right_boundary->mutable_curve()->add_segment()->mutable_line_segment();

  const std::vector<double> lane_left_width{std::get<1>(navi_path_tuple)};
  const std::vector<double> lane_right_width{std::get<2>(navi_path_tuple)};
  ADEBUG << "lane[" << lane->id().id()
         << "]: left_width[end] = " << lane_left_width.back()
         << ", right_width[end] = " << lane_right_width.back();
  auto start_time = Clock::NowInMicroseconds();
  double index = 0;
  line_segment->mutable_point()->Reserve(path.path_point_size());
  for (int i = 0; i < path.path_point().size(); i++) {
    auto* point = line_segment->add_point();
    point->set_x(path.path_point().at(i).x());
    point->set_y(path.path_point().at(i).y());
    point->set_z(path.path_point().at(i).z());
    index = index + 1;
    if (FLAGS_navigation_hdmap_generate_left_boundray) {
      auto* left_sample = lane->add_left_sample();
      left_sample->set_s(path.path_point().at(i).s());
      left_sample->set_width(lane_left_width[i]);
      if (FLAGS_using_original_lanemarker_boundary &&
          i < left_boundary_path->point().size()) {
        left_segment->add_point()->CopyFrom(left_boundary_path->point().at(i));
      } else if (i < left_boundary_path->point().size()) {
        left_segment->add_point()->CopyFrom(
            *point +
            lane_left_width[i] * Vec2d::CreateUnitVec2d(
                                     path.path_point().at(i).theta() + M_PI_2));
      }
    }

    auto* right_sample = lane->add_right_sample();
    right_sample->set_s(path.path_point().at(i).s());
    right_sample->set_width(lane_right_width[i]);
    if (FLAGS_using_original_lanemarker_boundary &&
        i < right_boundary_path->point().size()) {
      right_segment->add_point()->CopyFrom(right_boundary_path->point().at(i));
    } else if (i < right_boundary_path->point().size()) {
      right_segment->add_point()->CopyFrom(
          *point +
          lane_right_width[i] *
              Vec2d::CreateUnitVec2d(path.path_point().at(i).theta() - M_PI_2));
    }
  }
  auto path_time = Clock::NowInMicroseconds();
  // DownsampleCurve(lane->mutable_central_curve());
  DownsampleCurve(lane->mutable_left_boundary()->mutable_curve());
  DownsampleCurve(lane->mutable_right_boundary()->mutable_curve());
  auto path_point_time = path_time - start_time;
  auto down_time = Clock::NowInMicroseconds() - path_time;
  auto* time = creat_map_time_.add_lane_time();
  time->set_name(lane->id().id());
  time->set_lane_point_time(path_point_time);
  time->set_down_time(down_time);
  time->set_lane_point_size(path.path_point().size());

  /*打印稀疏点信息
  AERROR << "----------------central_points_size------------ = "
         << lane->central_curve().segment(0).line_segment().point_size();
  for (const auto& point :
       lane->central_curve().segment(0).line_segment().point()) {
    ADEBUG << "central_point_x = " << point.x()
              << "; central_point_y = " << point.y() << ".";
  }
  AERROR
      << "----------------left_points_size------------ = "
      << lane->left_boundary().curve().segment(0).line_segment().point_size();
  for (const auto& point :
       lane->left_boundary().curve().segment(0).line_segment().point()) {
    ADEBUG << "left_point_x = " << point.x()
              << "; left_point_y = " << point.y() << ".";
  }
  AERROR
      << "----------------right_points_size------------ = "
      << lane->right_boundary().curve().segment(0).line_segment().point_size();
  for (const auto& point :
       lane->right_boundary().curve().segment(0).line_segment().point()) {
    ADEBUG << "right_point_x = " << point.x()
              << "; right_point_y = " << point.y() << ".";
  }*/
  return true;
}

void NavigationLaneMapCreator::AddOtherPassage(
    hdmap::Map* hd_map, routing::RoutingResponse* routing_response, int index) {
  routing_response->clear_routing_request();
  auto* passage = routing_response->mutable_road()->at(0).add_passage();
  passage->set_can_exit(false);
  passage->set_change_lane_type(routing::ChangeLaneType::FORWARD);
  auto* segment = passage->add_segment();
  segment->set_id(hd_map->lane(index).id().id());
  auto adc_lane_segment_points = hd_map->lane(index)
                                     .central_curve()
                                     .segment()
                                     .at(0)
                                     .line_segment()
                                     .point();
  common::PointENU start_point = adc_lane_segment_points.at(0);
  int max_index = adc_lane_segment_points.size() - 1;
  common::PointENU end_point = adc_lane_segment_points.at(max_index);
  auto* routing_request = routing_response->mutable_routing_request();
  routing::LaneWaypoint waypoint;
  waypoint.set_id(hd_map->lane(index).id().id());
  waypoint.mutable_pose()->set_x(start_point.x());
  waypoint.mutable_pose()->set_y(start_point.y());
  waypoint.set_s(0.0);
  segment->set_start_s(0.0);
  routing_request->add_waypoint()->CopyFrom(waypoint);
  waypoint.set_s(hd_map->lane(index).length());
  segment->set_end_s(hd_map->lane(index).length());
  waypoint.mutable_pose()->set_x(end_point.x());
  waypoint.mutable_pose()->set_y(end_point.y());
  routing_request->add_waypoint()->CopyFrom(waypoint);
}

void NavigationLaneMapCreator::ChangeLaneTypeDecider(
    const perception::LaneMarkers& lane_marker) {
  const bool is_left_change = lane_marker.is_lanechange_to_left();
  const bool is_right_change = lane_marker.is_lanechange_to_right();
  auto input_change_lane_type_of_vehicle_lane =
      routing::ChangeLaneType::FORWARD;
  auto current_change_lane_type_of_vehicle_lane =
      routing::ChangeLaneType::FORWARD;
  for (const auto& change_lane_type :
       input_change_lane_types_.per_change_lane_type()) {
    if (change_lane_type.id() == "0_current") {
      input_change_lane_type_of_vehicle_lane =
          change_lane_type.change_lane_type();
    }
  }
  for (const auto& change_lane_type :
       current_change_lane_types_.per_change_lane_type()) {
    if (change_lane_type.id() == "0_current") {
      current_change_lane_type_of_vehicle_lane =
          change_lane_type.change_lane_type();
    }
  }

  bool diff_lane_change_type = false;
  // 1.当前状态为None：输入换道类型不为直行且当前换道类型为直行时跳转到start状态，此时会在routing中放入换道类型
  // 2.当前状态为Start：换道完成或者输入的换道类型与当前不同持续3s，此时会进入换道结束状态
  // 3.当前状态为End：此状态持续20周期，然后跳回None继续下一个周期，防止换道后输入还保持几帧换道状态，同时也可能
  // 出现连续换道情况
  ADEBUG << "now change_lane_type_state: " << change_lane_type_state_;
  switch (change_lane_type_state_) {
    case ChangeLaneTypeState::None:
      if (input_change_lane_type_of_vehicle_lane ==
          routing::ChangeLaneType::LEFT) {
        change_lane_type_state_ = ChangeLaneTypeState::Left_Start;
      } else if (input_change_lane_type_of_vehicle_lane ==
                 routing::ChangeLaneType::RIGHT) {
        change_lane_type_state_ = ChangeLaneTypeState::Right_Start;
      }
      current_change_lane_types_ = input_change_lane_types_;
      break;
    case ChangeLaneTypeState::Left_Start:
      diff_lane_change_type = change_type_diff_debounce_.DealDebounce(
          current_change_lane_type_of_vehicle_lane !=
          input_change_lane_type_of_vehicle_lane);
      DealStartState(is_left_change || diff_lane_change_type);
      break;
    case ChangeLaneTypeState::Right_Start:
      diff_lane_change_type = change_type_diff_debounce_.DealDebounce(
          current_change_lane_type_of_vehicle_lane !=
          input_change_lane_type_of_vehicle_lane);
      DealStartState(is_right_change || diff_lane_change_type);
      break;
    case ChangeLaneTypeState::End:
      change_count_++;
      if (change_count_ > 20) {
        change_count_ = 0;
        change_lane_type_state_ = ChangeLaneTypeState::None;
      }
      break;
    default:
      break;
  }
  // AERROR << "change_lane_type_state: " << change_lane_type_state_;
  // AERROR << "cur_change_lane_types: " << current_change_lane_types_.DebugString();
  // AERROR << "input_change_lane_types: " << input_change_lane_types_.DebugString();
  change_lane_type_info_.mutable_cur_change_lane_types()->CopyFrom(
      current_change_lane_types_);
  change_lane_type_info_.mutable_input_change_lane_types()->CopyFrom(
      input_change_lane_types_);
  change_lane_type_info_.set_chang_lane_type_state(change_lane_type_state_);
  change_lane_type_info_.set_count(change_count_);
}

void NavigationLaneMapCreator::DealStartState(bool input) {
  if (input) {
    change_type_diff_debounce_.Reset();
    auto* per_change_lane_type =
        current_change_lane_types_.mutable_per_change_lane_type();
    for (auto& change_lane_type : *per_change_lane_type) {
      if (change_lane_type.id() == "0_current") {
        change_lane_type.set_change_lane_type(routing::ChangeLaneType::FORWARD);
      } else if (change_lane_type.id() == "11_left") {
        change_lane_type.set_change_lane_type(routing::ChangeLaneType::RIGHT);
      } else if (change_lane_type.id() == "12_right") {
        change_lane_type.set_change_lane_type(routing::ChangeLaneType::LEFT);
      }
    }
    change_lane_type_state_ = ChangeLaneTypeState::End;
  }
}
}  // namespace planning
}  // namespace TL
