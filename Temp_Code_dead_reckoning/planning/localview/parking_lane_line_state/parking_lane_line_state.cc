//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
#include "planning/localview/parking_lane_line_state/parking_lane_line_state.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "common/configs/config_gflags.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "map/hdmap/hdmap_common.h"
#include "map/hdmap/hdmap_util.h"
#include "proto/routing/poi.pb.h"

namespace TL {
namespace planning {
bool ParkingLanelineState::Init() {
  return true;
}

bool ParkingLanelineState::ReadMap(const std::shared_ptr<LocalView>& local_view,
                                   TL::hdmap::Map* hd_map_ptr) {
  // read hd map from file
  if (hd_map_ptr == nullptr) {
    AERROR << "hd_map_ptr is nullptr";
    return false;
  }

  // 1. get parking lot center
  std::vector<common::math::Vec2d> parking_spot_enu(4);
  const std::string getParkingLot = {
      GetParkingLotArrayWorld(local_view, &parking_spot_enu)};
  if (!getParkingLot.empty()) {
    return false;
  }
  double lt2rt_angle = {
      std::atan2(parking_spot_enu[3].y() - parking_spot_enu[0].y(),
                 parking_spot_enu[3].x() - parking_spot_enu[0].x())};

  hd_map_ptr->clear_parking_space();
  auto* parking_space_show = hd_map_ptr->add_parking_space();
  parking_space_show->mutable_id()->set_id(absl::StrCat("PS", "000"));
  for (auto& p : parking_spot_enu) {
    auto* parking_space_show_point =
        parking_space_show->mutable_polygon()->add_point();
    parking_space_show_point->set_x(p.x());
    parking_space_show_point->set_y(p.y());
    parking_space_show_point->set_z(0);
  }

  common::PointENU top_edge_center;
  top_edge_center.set_x((parking_spot_enu[0].x() + parking_spot_enu[3].x()) /
                        2.0);
  top_edge_center.set_y((parking_spot_enu[0].y() + parking_spot_enu[3].y()) /
                        2.0);

  // 2. get the neast lane from file
  hdmap::LaneInfoConstPtr nearest_lane;
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  if (!GetNearestLane(hdmap::HDMapUtil::BaseMapPtr(), top_edge_center,
                      lt2rt_angle, &nearest_lane, &nearest_s, &nearest_l)) {
    AERROR << "error in getting nearest lane";
    return false;
  }

  // 3. set lane to hd_map
  // TODO(jyw): build lane based on routing if ism
  hd_map_ptr->clear_lane();
  constexpr double kRoiLen = 100.0;
  // construct predecessor lane
  double predecessor_length = kRoiLen - nearest_s;
  std::vector<hdmap::LaneInfoConstPtr> predecessor_lanes{nearest_lane};
  while (predecessor_length > 0) {
    ADEBUG << "left predecessor_length is " << predecessor_length;
    hdmap::LaneInfoConstPtr& lane_tmp = predecessor_lanes.back();
    if (lane_tmp->lane().predecessor_id().empty()) {
      AERROR << "has no predecessor lane";
      break;
    }
    // get predecessor lane with min angle diff
    int min_ang_diff_idx = 0;
    if (lane_tmp->lane().predecessor_id_size() > 1) {
      double min_ang_diff = {M_PI};
      for (int i = 0; i < lane_tmp->lane().predecessor_id_size(); ++i) {
        const auto& lane_id = lane_tmp->lane().predecessor_id().at(i);
        hdmap::LaneInfoConstPtr predecessor_lane = {
            hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(lane_id)};
        if (predecessor_lane == nullptr) {
          continue;
        }
        double ang_diff = fabs(common::math::AngleDiff(
            predecessor_lane->Heading(predecessor_lane->accumulate_s().back()),
            predecessor_lane->Heading(predecessor_lane->accumulate_s().back() /
                                      2.0)));
        if (ang_diff < min_ang_diff) {
          min_ang_diff = ang_diff;
          min_ang_diff_idx = i;
        }
      }
    }
    const auto& lane_id =
        lane_tmp->lane().predecessor_id().at(min_ang_diff_idx);
    hdmap::LaneInfoConstPtr predecessor_lane = {
        hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(lane_id)};
    predecessor_lanes.emplace_back(predecessor_lane);
    predecessor_length -= predecessor_lane->lane().length();
  }
  for (size_t i = predecessor_lanes.size() - 1; i > 0; --i) {
    auto* lane = hd_map_ptr->add_lane();
    lane->CopyFrom(predecessor_lanes[i]->lane());
    AINFO << "predecessor_lane id " << predecessor_lanes[i]->lane().id().id();
  }

  auto* lane = hd_map_ptr->add_lane();
  lane->CopyFrom(nearest_lane->lane());
  ADEBUG << "nearest_lane_id " << nearest_lane->lane().id().id();
  lane->set_type(hdmap::Lane::CITY_DRIVING);
  lane->set_turn(hdmap::Lane::NO_TURN);
  lane->set_speed_limit(FLAGS_default_cruise_low_speed);

  double successor_length = kRoiLen - nearest_lane->lane().length() + nearest_s;
  std::vector<hdmap::LaneInfoConstPtr> successor_lanes{nearest_lane};
  while (successor_length > 0) {
    ADEBUG << "left successor_length is " << successor_length;
    hdmap::LaneInfoConstPtr& lane_tmp = successor_lanes.back();
    if (lane_tmp->lane().successor_id().empty()) {
      AERROR << "has no successor lane";
      break;
    }
    // get successor lane with min angle diff
    int min_ang_diff_idx = 0;
    if (lane_tmp->lane().successor_id_size() > 1) {
      double min_ang_diff = {M_PI};
      for (int i = 0; i < lane_tmp->lane().successor_id_size(); ++i) {
        const auto& lane_id = lane_tmp->lane().successor_id().at(i);
        hdmap::LaneInfoConstPtr successor_lane =
            hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(lane_id);
        if (successor_lane == nullptr) {
          continue;
        }
        double ang_diff = fabs(common::math::AngleDiff(
            successor_lane->Heading(successor_lane->accumulate_s().front()),
            successor_lane->Heading(successor_lane->accumulate_s().back() /
                                    2.0)));
        if (ang_diff < min_ang_diff) {
          min_ang_diff = ang_diff;
          min_ang_diff_idx = i;
        }
      }
    }

    const auto& lane_id = lane_tmp->lane().successor_id().at(min_ang_diff_idx);
    hdmap::LaneInfoConstPtr successor_lane = {
        hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(lane_id)};
    successor_lanes.emplace_back(successor_lane);
    successor_length -= successor_lane->lane().length();
  }
  for (size_t i = 1; i < successor_lanes.size(); ++i) {
    auto* lane = hd_map_ptr->add_lane();
    lane->CopyFrom(successor_lanes[i]->lane());
    AINFO << "successor_lane id " << successor_lanes[i]->lane().id().id();
  }
  start_and_end_s_.first =
      hd_map_ptr->lane(0).central_curve().segment().at(0).s() -
      predecessor_length;
  start_and_end_s_.second =
      hd_map_ptr->lane(hd_map_ptr->lane_size() - 1)
          .central_curve()
          .segment()
          .at(0)
          .s() +
      hd_map_ptr->lane(hd_map_ptr->lane_size() - 1).length() + successor_length;

  return true;
}

std::string ParkingLanelineState::GetParkingLotArrayWorld(  // NOLINT
    const std::shared_ptr<LocalView>& local_view,
    std::vector<common::math::Vec2d>* const parking_spot_enu) {
  CHECK_EQ(parking_spot_enu->size(), 4U);
  int space_opt_id = -1;
  int selected_slot_id = -1;
  const int parking_lot_size =
      local_view->GetParkingLotOutArray()->parking_lots_size();

  std::string error_msg = {};
  if (!local_view->HasValidParkingLotOutArrayHeader() || parking_lot_size < 1) {
    error_msg = "has no praking lot";
    AERROR << error_msg;
    return error_msg;
  }

  if (local_view->GetParkingLotOutArray()->has_opt_parking_seq()) {
    space_opt_id = static_cast<int>(
        local_view->GetParkingLotOutArray()->opt_parking_seq());
  } else {
    error_msg = "has no opt parking lot id";
    AERROR << error_msg;
    return error_msg;
  }

  for (int i = 0; i < parking_lot_size; i++) {
    if (local_view->GetParkingLotOutArray()
            ->parking_lots()
            .at(i)
            .parking_seq() == static_cast<uint32_t>(space_opt_id)) {
      selected_slot_id = i;
      ADEBUG << "Space Perception give parking_slot's ID = "
             << selected_slot_id;
      break;
    }
  }
  if (selected_slot_id == -1) {
    error_msg = "The optimal parking lot is not given";
    AERROR << error_msg;
    return error_msg;
  }
  const auto& select_parking_lot =
      local_view->GetParkingLotOutArray()->parking_lots().at(selected_slot_id);
  if (select_parking_lot.pts_enu_size() < 4) {
    error_msg = "Space Perception give parking_slot's point size not 4 ";
    AERROR << error_msg;
    return error_msg;
  }

  for (int j = 0; j < select_parking_lot.pts_enu_size(); j++) {
    int idx = -1;
    switch (select_parking_lot.pts_enu().at(j).position()) {
      case TL::perception::PSPoint_Position_TOP_LEFT:
        idx = 0;
        break;
      case TL::perception::PSPoint_Position_BOTTOM_LEFT:
        idx = 1;
        break;
      case TL::perception::PSPoint_Position_BOTTOM_RIGHT:
        idx = 2;
        break;
      case TL::perception::PSPoint_Position_TOP_RIGHT:
        idx = 3;
        break;
      default:
        break;
    }
    if (idx > -1) {
      parking_spot_enu->at(idx) = {
          select_parking_lot.pts_enu().at(j).point().x(),
          select_parking_lot.pts_enu().at(j).point().y()};
    }
  }

  if (!common::math::Polygon2d::IsConvexPolygon(*parking_spot_enu)) {
    error_msg = "optimal parking spot is not convex";
    AERROR << error_msg;
  }
  return error_msg;
}

// TODO(jyw): add perception
std::string ParkingLanelineState::CreatMap(
    const std::shared_ptr<LocalView>& local_view,
    TL::hdmap::Map* hd_map_ptr) {
  // creat map for parking task
  std::vector<TL::common::PointENU> center_points;
  std::vector<TL::common::PointENU> left_boundary_points;
  std::vector<TL::common::PointENU> right_boundary_points;
  TL::common::PointENU point_central;
  TL::common::PointENU point_left;
  TL::common::PointENU point_right;
  std::vector<double> sample_s;
  // init by vehicle pose
  double track_angle = local_view->GetLocalization()->pose().heading();
  common::math::Vec2d lane_central_point{
      local_view->GetLocalization()->pose().position().x(),
      local_view->GetLocalization()->pose().position().y()};
  const double half_lane_width = FLAGS_open_space_lane_width;

  std::string creatMagErrorMsg = {};

  if (local_view->GetFunctionManagerIn()->fct_avp_in().has_sys_run_state() &&
      (local_view->GetFunctionManagerIn()->fct_avp_in().sys_run_state() !=
       functionmanager::AvpFctIn::PARKING)) {
    AINFO << "In direct move scenario, add only lane and road map info";
  } else {
    // In parking scenario,  first add park slot
    std::vector<common::math::Vec2d> parking_spot_enu(4);
    creatMagErrorMsg = GetParkingLotArrayWorld(local_view, &parking_spot_enu);
    if (!creatMagErrorMsg.empty()) {
      return creatMagErrorMsg;
    }
    // creart map based on parking lot
    hd_map_ptr->clear_parking_space();
    auto* parking_space_show = hd_map_ptr->add_parking_space();
    parking_space_show->mutable_id()->set_id(absl::StrCat("PS", "000"));
    for (auto& p : parking_spot_enu) {
      auto* parking_space_show_point =
          parking_space_show->mutable_polygon()->add_point();
      parking_space_show_point->set_x(p.x());
      parking_space_show_point->set_y(p.y());
      parking_space_show_point->set_z(0);
    }
    // modify track angle and central line based on projection
    double dis2parking_lot = {
        ((parking_spot_enu[3] + parking_spot_enu[0]) / 2.0)
            .DistanceTo(lane_central_point)};
    AINFO << "dis2parking_lot is " << dis2parking_lot
          << " FLAGS_parkingout_roi_length " << FLAGS_parkingout_roi_length;
    if (dis2parking_lot < FLAGS_parkingout_roi_length) {
      track_angle = (parking_spot_enu[3] - parking_spot_enu[0]).Angle();
      lane_central_point =
          half_lane_width *
              common::math::Vec2d::CreateUnitVec2d(track_angle + M_PI_2) +
          (parking_spot_enu[3] - parking_spot_enu[0]) / 2.0 +
          parking_spot_enu[0];
    }
  }
  // constrcut map
  for (int i = static_cast<int>(-1 * FLAGS_parkingout_roi_length);
       i <= FLAGS_parkingout_roi_length; i++) {
    auto tmp_x = lane_central_point.x() +
                 (i * cos(common::math::NormalizeAngle(track_angle)));
    auto tmp_y = lane_central_point.y() +
                 (i * sin(common::math::NormalizeAngle(track_angle)));
    point_central.set_x(tmp_x);
    point_central.set_y(tmp_y);
    point_central.set_z(0);
    point_left.set_x(
        tmp_x + half_lane_width *
                    (cos(common::math::NormalizeAngle(track_angle + M_PI_2))));
    point_left.set_y(
        tmp_y + half_lane_width *
                    (sin(common::math::NormalizeAngle(track_angle + M_PI_2))));

    point_right.set_x(
        tmp_x + half_lane_width *
                    (cos(common::math::NormalizeAngle(track_angle - M_PI_2))));
    point_right.set_y(
        tmp_y + half_lane_width *
                    (sin(common::math::NormalizeAngle(track_angle - M_PI_2))));
    center_points.emplace_back(point_central);
    left_boundary_points.emplace_back(point_left);
    right_boundary_points.emplace_back(point_right);
    sample_s.emplace_back(i + FLAGS_parkingout_roi_length);
  }

  // set lane in map
  hd_map_ptr->clear_lane();
  auto* lane = hd_map_ptr->add_lane();
  lane->mutable_id()->set_id(absl::StrCat("parking_lane"));
  lane->set_type(hdmap::Lane::CITY_DRIVING);
  lane->set_turn(hdmap::Lane::NO_TURN);
  lane->set_speed_limit(FLAGS_default_cruise_low_speed);

  // central line
  auto* curve_segment = lane->mutable_central_curve()->add_segment();
  curve_segment->set_heading(track_angle);
  curve_segment->set_length(sample_s.back());
  lane->set_length(sample_s.back());
  auto* line_segment = curve_segment->mutable_line_segment();

  // left boundary
  auto* left_boundary = lane->mutable_left_boundary();
  auto* left_boundary_type = left_boundary->add_boundary_type();
  left_boundary->set_virtual_(false);
  left_boundary_type->set_s(0.0);
  left_boundary_type->add_types(
      TL::hdmap::LaneBoundaryType_Type::LaneBoundaryType_Type_DOTTED_WHITE);
  auto* left_segment =
      left_boundary->mutable_curve()->add_segment()->mutable_line_segment();

  // right boundary
  auto* right_boundary = lane->mutable_right_boundary();
  auto* right_boundary_type = right_boundary->add_boundary_type();
  right_boundary->set_virtual_(false);
  right_boundary_type->set_s(0.0);
  right_boundary_type->add_types(
      TL::hdmap::LaneBoundaryType_Type::LaneBoundaryType_Type_DOTTED_WHITE);
  auto* right_segment =
      right_boundary->mutable_curve()->add_segment()->mutable_line_segment();

  for (size_t i = 0; i < center_points.size(); i++) {
    line_segment->add_point()->CopyFrom(center_points.at(i));
    left_segment->add_point()->CopyFrom(left_boundary_points.at(i));
    right_segment->add_point()->CopyFrom(right_boundary_points.at(i));

    auto* right_sample = lane->add_right_sample();
    right_sample->set_s(sample_s.at(i));
    right_sample->set_width(half_lane_width);

    auto* left_sample = lane->add_left_sample();
    left_sample->set_s(sample_s.at(i));
    left_sample->set_width(half_lane_width);
  }

  return creatMagErrorMsg;
}

bool ParkingLanelineState::SetRoutingAndRoad(  // NOLINT
    TL::routing::RoutingResponse* inrouting, TL::hdmap::Map* hd_map) {
  // Set road boundary
  int lane_num = hd_map->lane_size();
  ADEBUG << "The Parking Lane number is: " << lane_num;
  hd_map->clear_road();
  auto* road = hd_map->add_road();
  road->mutable_id()->set_id("road_park");
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
  hdmap::LaneInfo first_lane(hd_map->lane(0));
  hdmap::LaneInfo end_lane(hd_map->lane(lane_num - 1));
  const auto start_point = first_lane.GetSmoothPoint(start_and_end_s_.first);
  const auto end_point = end_lane.GetSmoothPoint(start_and_end_s_.second);
  auto* routing_request = inrouting->mutable_routing_request();
  routing::LaneWaypoint waypoint;
  waypoint.set_id(hd_map->lane(0).id().id());
  waypoint.mutable_pose()->set_x(start_point.x());
  waypoint.mutable_pose()->set_y(start_point.y());
  double start_s = start_and_end_s_.first;
  double end_s = start_s + 2 * FLAGS_parkingout_roi_length;
  waypoint.set_s(start_s);
  auto* passage = routing_road->add_passage();
  passage->set_can_exit(false);
  passage->set_change_lane_type(routing::ChangeLaneType::FORWARD);
  for (int i = 0; i < lane_num; ++i) {
    auto* segment = passage->add_segment();
    segment->set_id(hd_map->lane(i).id().id());
    segment->set_start_s(hd_map->lane(i).central_curve().segment().at(0).s());
    segment->set_end_s(hd_map->lane(i).central_curve().segment().at(0).s() +
                       hd_map->lane(i).length());
  }
  routing_request->add_waypoint()->CopyFrom(waypoint);
  waypoint.set_id(hd_map->lane(lane_num - 1).id().id());
  waypoint.set_s(end_s);
  waypoint.mutable_pose()->set_x(end_point.x());
  waypoint.mutable_pose()->set_y(end_point.y());
  routing_request->add_waypoint()->CopyFrom(waypoint);

  common::util::FillHeader("parking_routing", inrouting);
  return true;
}

Status ParkingLanelineState::ProcessBaiduSimulations(
    const std::shared_ptr<LocalView>& local_view) {
  std::string error_msg{};
  if (local_view == nullptr) {
    error_msg = "local view is null";
    return Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, error_msg);
  }
  // only worldsim
  if (parking_lot_array_enu_ptr_ == nullptr) {
    parking_lot_array_enu_ptr_ =
        std::make_shared<TL::perception::ParkingLotOutArray>();
    if (!hdmap::HDMapUtil::GetParkingLotFromMap(
            hdmap::HDMapUtil::BaseMapPtr(),
            local_view->GetLocalization()->pose().position(),
            parking_lot_array_enu_ptr_.get())) {
      error_msg = "get parking lot from map failed";
      AERROR << error_msg;
      return Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, error_msg);
    }
    ParkingPathPointSetPro traced_path;
    std::string error_msg{};
    if (GenerateTracedPathFromBaiduMap(hdmap::HDMapUtil::BaseMapPtr(),
                                       &traced_path, &error_msg)) {
      parking_lot_array_enu_ptr_->mutable_traced_path()->CopyFrom(traced_path);
    }
  }
  if (!SetParkingLot2Localview(local_view, &error_msg)) {
    return Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, error_msg);
  }

  if (!ReadMap(local_view, parking_hdmap_.mutable_hdmap())) {
    error_msg = "Read map failed.";
    AERROR << error_msg;
    return Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, error_msg);
  }

  routing_response_.Clear();
  if (!SetRoutingAndRoad(&routing_response_, parking_hdmap_.mutable_hdmap())) {
    error_msg = "build parking lane response fail ";
    ADEBUG << error_msg;
    return Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, error_msg);
  }
  return Status::OK();
}

Status ParkingLanelineState::ProcessBaiduRecordReplay(
    const std::shared_ptr<LocalView>& local_view) {
  std::string error_msg{};
  if (local_view == nullptr) {
    error_msg = "local view is null";
    return Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, error_msg);
  }
  AINFO << "check msg " << parking_hdmap_.has_hdmap() << " "
        << local_view->HasMapMsg();
  if (!parking_hdmap_.has_hdmap() && local_view->HasMapMsg()) {
    auto hd_map = std::make_shared<hdmap::HDMap>();
    hd_map->LoadMapFromProto(local_view->GetMapMsg()->hdmap());
    parking_hdmap_.mutable_hdmap()->CopyFrom(local_view->GetMapMsg()->hdmap());
    parking_lot_array_enu_ptr_ =
        std::make_shared<TL::perception::ParkingLotOutArray>();
    if (!hdmap::HDMapUtil::GetParkingLotFromMap(
            hd_map, local_view->GetLocalization()->pose().position(),
            parking_lot_array_enu_ptr_.get())) {
      error_msg = "get parking lot from map failed";
      AERROR << error_msg;
      return Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, error_msg);
    }
    ADEBUG << "lane id" << local_view->GetMapMsg()->hdmap().lane(0).id().id();
  }
  if (local_view->HasValidRoutingResponseHeader()) {
    routing_response_.CopyFrom(*local_view->GetRoutingResponse());
  }

  if (!SetParkingLot2Localview(local_view, &error_msg)) {
    return Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, error_msg);
  }
  return Status::OK();
}

Status ParkingLanelineState::ProcessRealCar(
    const std::shared_ptr<LocalView>& local_view) {
  std::string error_msg{};
  if (local_view == nullptr) {
    error_msg = "local view is null";
    return Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, error_msg);
  }
  if (FLAGS_enable_parking_state_sim && parking_lot_array_enu_ptr_ == nullptr &&
      !ReadParkingLotInfoFromConf(local_view)) {
    error_msg = "Perfect perception: fail to read config";
    AINFO << error_msg;
    return Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, error_msg);
  }

  if (!SetParkingLot2Localview(local_view, &error_msg)) {
    return Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, error_msg);
  }

  const std::string creatMapMsg = {
      CreatMap(local_view, parking_hdmap_.mutable_hdmap())};
  if (!creatMapMsg.empty()) {
    AERROR << creatMapMsg;
    return Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, creatMapMsg);
  }

  routing_response_.Clear();
  if (!SetRoutingAndRoad(&routing_response_, parking_hdmap_.mutable_hdmap())) {
    error_msg = "build parking lane response fail ";
    ADEBUG << error_msg;
    return Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, error_msg);
  }
  return Status::OK();
}

bool ParkingLanelineState::SetParkingLot2Localview(
    const std::shared_ptr<LocalView>& local_view, std::string* error_msg) {
  // set parking lot to local view
  if ((FLAGS_enable_parking_state_sim ||
       !local_view->HasValidParkingLotOutArrayHeader() ||
       !local_view->GetParkingLotOutArray()->header().has_seq()) &&
      !SetParkingLot(local_view)) {
    AERROR << "set parking lot failed";
  }
  if (FLAGS_enable_park_with_one_slot && local_view->HasFunctionManagerIn() &&
      local_view->GetFunctionManagerIn()->fct_avp_in().sys_run_state() ==
          functionmanager::AvpFctIn::PARKING &&
      local_view->HasValidParkingLotOutArrayHeader() &&
      local_view->GetParkingLotOutArray()->parking_lots_size() != 1) {
    *error_msg = "parking lot num is not 1 ";
    return false;
  }
  return true;
}

Status ParkingLanelineState::BuildLocalView(
    const std::shared_ptr<LocalView>& local_view) {
  if (!local_view->HasLocalization()) {
    const std::string msg = {"No localization"};
    AINFO << msg;
    return Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, msg);
  }
  // get parking lot in world coordinate
  bool use_baidu_mode = false;
#ifdef FOR_BAIDU_SIMULATION
  use_baidu_mode = true;
#endif
  Status cur_status = Status::OK();
  if (FLAGS_is_record_replay) {
    // baidu simulation record replay
    cur_status = ProcessBaiduRecordReplay(local_view);
  } else if (use_baidu_mode) {
    // baidu simulation
    cur_status = ProcessBaiduSimulations(local_view);
  } else {
    // real car
    cur_status = ProcessRealCar(local_view);
  }

  if (cur_status != Status::OK()) {
    return cur_status;
  }

  local_view->SetRoutingResponsePtr(
      std::make_shared<routing::RoutingResponse>(routing_response_));
  common::util::FillHeader("parking_map", &parking_hdmap_);
  common::util::FillHeader("parking_map",
                           parking_hdmap_.mutable_hdmap()->mutable_header());
  local_view->SetMapMsgPtr(
      std::make_shared<navigation_hdmap::MapMsg>(parking_hdmap_));
  return Status::OK();
}

bool ParkingLanelineState::ReadParkingLotInfoFromConf(
    const std::shared_ptr<LocalView>& local_view) {
  TL::perception::ParkingLotOutArray parking_lot;
  if (!TL::common::GetProtoFromFile(FLAGS_test_parking_file, &parking_lot)) {
    AERROR << "failed to load parking lot file: " << FLAGS_test_parking_file;
    return false;
  }
  if (!local_view->HasLocalization()) {
    AERROR << "No localization";
    return false;
  }
  const auto& local_position = local_view->GetLocalization();
  // trans to global coordinate
  parking_lot_array_enu_ptr_ =
      std::make_shared<TL::perception::ParkingLotOutArray>(parking_lot);
  for (int i = 0; i < parking_lot.parking_lots_size(); ++i) {
    for (int j = 0; j < parking_lot.parking_lots(i).pts_vrf_size(); j++) {
      auto parking_point_enu = common::math::RFUToENU(
          parking_lot.parking_lots(i).pts_vrf().at(j).point().x(),
          parking_lot.parking_lots(i).pts_vrf().at(j).point().y(),
          local_position->pose().position().x(),
          local_position->pose().position().y(),
          local_position->pose().heading());
      auto* pts_enu =
          parking_lot_array_enu_ptr_->mutable_parking_lots(i)->add_pts_enu();
      *pts_enu = parking_lot.parking_lots(i).pts_vrf().at(j);
      pts_enu->mutable_point()->set_x(parking_point_enu.first);
      pts_enu->mutable_point()->set_y(parking_point_enu.second);
    }
  }

  return true;
}

bool ParkingLanelineState::SetParkingLot(
    const std::shared_ptr<LocalView>& local_view) {
  if (nullptr == parking_lot_array_enu_ptr_) {
    AERROR << "parking lot array is nullptr";
    return false;
  }
  TL::perception::ParkingLotOutArray parking_lot_array =
      *parking_lot_array_enu_ptr_;
#ifdef FOR_BAIDU_SIMULATION
  for (auto& origin_park_lot : *parking_lot_array.mutable_parking_lots()) {
    for (auto& p_vrf : *origin_park_lot.mutable_pts_vrf()) {
      p_vrf.set_quality(TL::perception::PSPoint::HIGH);
    }
    for (auto& p_enu : *origin_park_lot.mutable_pts_enu()) {
      p_enu.set_quality(TL::perception::PSPoint::HIGH);
    }
    origin_park_lot.set_sensor_type(perception::ParkingLotOut::CAMERA);
  }
#endif
  common::util::FillHeader("parking_lots", &parking_lot_array);
  local_view->SetParkingLotOutArrayPtr(
      std::make_shared<TL::perception::ParkingLotOutArray>(
          parking_lot_array));
  return true;
}

bool ParkingLanelineState::GetNearestLane(  // NOLINT
    const std::shared_ptr<hdmap::HDMap>& map_ptr, const common::PointENU& point,
    double heading, hdmap::LaneInfoConstPtr* nearest_lane, double* nearest_s,
    double* nearest_l) {
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  int status_positive = {map_ptr->GetLanesWithHeading(
      point, 10, common::math::NormalizeAngle(heading), M_PI / 8.0, &lanes)};

  std::vector<hdmap::LaneInfoConstPtr> lanes_tmp;
  int status_negative = {map_ptr->GetLanesWithHeading(
      point, 10, common::math::NormalizeAngle(heading + M_PI), M_PI / 8.0,
      &lanes_tmp)};
  ADEBUG << "lanes size " << lanes.size() << " " << lanes_tmp.size();

  double min_distance = {std::numeric_limits<double>::infinity()};
  bool has_nearest_lane = false;
  double s_in_lane = 0.0;
  double l_in_lane = 0.0;
  if (status_positive >= 0 || status_negative >= 0) {
    lanes.insert(lanes.end(), lanes_tmp.begin(), lanes_tmp.end());
    for (const auto& lane : lanes) {
      ADEBUG << " pnc map lane id: " << lane->id().id();
      if (!lane->GetProjection({point.x(), point.y()}, &s_in_lane,
                               &l_in_lane)) {
        AERROR << "fail to get projection";
        continue;
      }
      ADEBUG << "s: " << s_in_lane << " l: " << l_in_lane;

      // Use large epsilon to allow projection diff
      constexpr double kEpsilon = 0.2;
      if (s_in_lane > (lane->total_length() + kEpsilon) ||
          (s_in_lane + kEpsilon) < 0.0) {
        ADEBUG << "s " << s_in_lane << " lane " << lane->total_length();
        continue;
      }
      const auto distance = fabs(l_in_lane);
      if (distance < min_distance) {
        min_distance = distance;
        *nearest_lane = lane;
        *nearest_s = s_in_lane;
        *nearest_l = l_in_lane;
        has_nearest_lane = true;
      }
    }
  } else {
    AWARN << "failed get nearest lane with heading, try to search without "
             "heading constrain";
    hdmap::LaneInfoConstPtr lane_tmp{};
    int status =
        map_ptr->GetNearestLane(point, &lane_tmp, &s_in_lane, &l_in_lane);
    *nearest_lane = lane_tmp;
    *nearest_s = s_in_lane;
    *nearest_l = l_in_lane;
    has_nearest_lane = status > -1;
  }

  return has_nearest_lane;
}

bool ParkingLanelineState::GenerateTracedPathFromBaiduMap(
    const std::shared_ptr<hdmap::HDMap>& map_ptr,
    ParkingPathPointSetPro* const traced_paths, std::string* const error_msg) {
  routing::POI poi;
  if (error_msg == nullptr) {
    AERROR << "error_msg is nullptr!";
    return false;
  }
  if (traced_paths == nullptr) {
    *error_msg = "traced_paths is nullptr!";
    return false;
  }
  if (!TL::common::GetProtoFromASCIIFile(hdmap::EndWayPointFile(), &poi)) {
    *error_msg = "can not get EndWayPointFile!";
    return false;
  }
  if (poi.landmark_size() != 1 || poi.landmark(0).waypoint_size() != 2) {
    *error_msg = "EndWayPointFile not valid!";
    return false;
  }
  const auto& start_pose = poi.landmark(0).waypoint(1).pose();
  const auto& end_pose = poi.landmark(0).waypoint(0).pose();
  const Vec2d start_point{start_pose.x(), start_pose.y()};
  const Vec2d end_point{end_pose.x(), end_pose.y()};
  ADEBUG << "start p x: " << start_point.x() << "start p y: " << start_point.y()
         << "end p x: " << end_point.x() << "end p y: " << end_point.y();
  constexpr double kLaneDistance = 2;
  std::vector<hdmap::LaneInfoConstPtr> start_lanes;
  std::vector<hdmap::LaneInfoConstPtr> end_lanes;
  hdmap::LaneInfoConstPtr start_lane;
  hdmap::LaneInfoConstPtr end_lane;
  double start_lane_s = 0.0;
  double start_lane_l = 0.0;
  double end_lane_s = 0.0;
  double end_lane_l = 0.0;
  if (map_ptr->GetLanes(end_pose, kLaneDistance, &end_lanes) != 0) {
    *error_msg = "can not get lanes around end point within 2 meters!";
    return false;
  }
  if (!IsLaneForPoint(end_lanes, end_point, &end_lane, &end_lane_s,
                      &end_lane_l)) {
    return false;
  }
  if (map_ptr->GetLanes(start_pose, kLaneDistance, &start_lanes) != 0) {
    *error_msg = "can not get lanes around start point within 2 meters!";
    return false;
  }
  if (!IsLaneForPoint(start_lanes, start_point, &start_lane, &start_lane_s,
                      &start_lane_l)) {
    return false;
  }
  constexpr double kLaneTotalLengthLimit = 500.0;
  std::queue<std::pair<std::vector<hdmap::Id>, double>> lane_info_queue;
  std::string start_lane_id{start_lane->id().id()};
  ADEBUG << " end size: " << end_lanes.size()
         << " start size: " << start_lanes.size();
  ADEBUG << " end_lane_id: " << end_lane->id().id();
  std::vector<hdmap::Id> target_lane_ids;
  lane_info_queue.push({{end_lane->id()}, end_lane->total_length()});
  while (!lane_info_queue.empty()) {
    const auto cur_id_vec = lane_info_queue.front().first;
    const double cur_total_len = lane_info_queue.front().second;
    lane_info_queue.pop();
    const hdmap::Id& cur_lane_id = cur_id_vec.back();
    const hdmap::LaneInfoConstPtr cur_lane = {
        map_ptr->GetLaneById(cur_lane_id)};
    if (cur_lane_id.id() == start_lane_id) {
      target_lane_ids = cur_id_vec;
      break;
    }
    for (int i = 0; i < cur_lane->lane().predecessor_id_size(); ++i) {
      const auto& next_lane_id = cur_lane->lane().predecessor_id().at(i);
      ADEBUG << " predecessor lane id : " << next_lane_id.id();
      hdmap::LaneInfoConstPtr next_lane = {map_ptr->GetLaneById(next_lane_id)};
      const double next_total_len = cur_total_len + next_lane->total_length();
      if (next_total_len > kLaneTotalLengthLimit) {
        continue;
      }
      std::string str_next_lane_id{next_lane_id.id()};
      std::vector<hdmap::Id> next_id_vec{cur_id_vec};
      next_id_vec.push_back(next_lane_id);
      lane_info_queue.emplace(next_id_vec, next_total_len);
    }
  }
  ADEBUG << " start_lane_id: " << start_lane->id().id();
  if (target_lane_ids.empty()) {
    *error_msg = "can not find target_lane_ids, return false! ";
    AERROR << *error_msg;
    return false;
  }
  // auto* p1 = traced_paths->Add();
  // p1->set_x(end_point.x());
  // p1->set_y(end_point.y());
  for (const auto& id : target_lane_ids) {
    const hdmap::LaneInfoConstPtr cur_lane =
        hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(id);
    for (int i = static_cast<int>(cur_lane->points().size()) - 2; i >= 0; --i) {
      if (cur_lane == end_lane &&
          cur_lane->accumulate_s()[i] > end_lane_s - 1e-6) {
        continue;
      }
      if (cur_lane == start_lane &&
          cur_lane->accumulate_s()[i] < start_lane_s + 1e-6) {
        break;
      }
      auto* cur_point = traced_paths->Add();
      cur_point->set_x(cur_lane->points()[i].x());
      cur_point->set_y(cur_lane->points()[i].y());
      uint32_t trace_path_gear = 4;  // gear D
      cur_point->set_gear(trace_path_gear);
      cur_point->set_yaw(cur_lane->headings()[i]);
      ADEBUG << " psh_x: " << cur_point->x() << ", psh_y: " << cur_point->y();
    }
  }
  if (traced_paths->size() < 2) {
    *error_msg = "traced_paths size less than 2";
    return false;
  }
  // auto* p2 = traced_paths->Add();
  // const auto start_p = start_lane->GetSmoothPoint(start_lane_s);
  // p2->set_x(start_p.x());
  // p2->set_y(start_p.y());
  // uint32_t trace_path_gear = 2;  // gear R
  // p1->set_gear(trace_path_gear);
  // trace_path_gear = 4;  // gear D
  // p2->set_gear(trace_path_gear);

  // p1->set_yaw(traced_paths->at(1).yaw());
  // p2->set_yaw(traced_paths->at(traced_paths->size() - 2).yaw());

  ADEBUG << "traced_path.size(): " << traced_paths->size();
  for (const auto& point : *traced_paths) {
    ADEBUG << ", x: " << point.x() << ", y: " << point.y()
           << ", theta: " << point.yaw();
  }
  return true;
}

bool ParkingLanelineState::IsLaneForPoint(
    const std::vector<hdmap::LaneInfoConstPtr>& lanes, const Vec2d& point,
    hdmap::LaneInfoConstPtr* const nearest_lane, double* const nearest_s,
    double* const nearest_l) {
  if (nearest_lane == nullptr) {
    return false;
  }
  if (nearest_s == nullptr) {
    return false;
  }
  if (nearest_l == nullptr) {
    return false;
  }
  double min_distance = 2;
  bool has_nearest_lane = false;
  double s_in_lane = 0.0;
  double l_in_lane = 0.0;

  for (const auto& lane : lanes) {
    ADEBUG << " pnc map lane id: " << lane->id().id();
    if (!lane->GetProjection(point, &s_in_lane, &l_in_lane)) {
      AERROR << "fail to get projection";
      continue;
    }
    ADEBUG << "s: " << s_in_lane << " l: " << l_in_lane;

    // Use large epsilonmin_distance to allow projection diff
    constexpr double kEpsilon = 0.2;
    if (s_in_lane > (lane->total_length() + kEpsilon) ||
        (s_in_lane + kEpsilon) < 0.0) {
      ADEBUG << "s " << s_in_lane << " lane " << lane->total_length();
      continue;
    }
    const auto distance = fabs(l_in_lane);
    if (distance < min_distance) {
      min_distance = distance;
      *nearest_lane = lane;
      *nearest_s = s_in_lane;
      *nearest_l = l_in_lane;
      has_nearest_lane = true;
    }
  }
  return has_nearest_lane;
}
}  // namespace planning
}  // namespace TL
