//  Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
#include "planning/localview/hdmap_avp_state/hdmap_avp_state.h"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "absl/strings/match.h"
#include "boost/cerrno.hpp"
#include "common/configs/vehicle_config_helper.h"
#include "common/file/log.h"
#include "common/math/path_matcher.h"
#include "common/util/point_factory.h"
#include "common/util/util.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/planning_gflags.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/common/types.pb.h"
#include "proto/hmi/avp.pb.h"
#include "proto/map/map.pb.h"
#include "proto/perception/perception_parking_lot.pb.h"
#include "proto/routing/poi.pb.h"

namespace TL {
namespace planning {

using AVPMapState = TL::hdmap::AVPSlamMap::AVPMapState;

HDMapAVPState::HDMapAVPState() {
  last_routing_request_ = std::make_shared<routing::RoutingRequest>();
#if (!defined(ISMDC) && !defined(ISORIN))
  if (FLAGS_is_record_replay) {
    hd_map_ = std::make_shared<hdmap::HDMap>();
  } else {
    hd_map_ = hdmap::HDMapUtil::LoadLocalMapPtr();
  }
  if (hd_map_) {
    hdmap::Map hd_map;
    hd_map_->GetMap(&hd_map);
    routing_.Init(hd_map);
    routing_.Start();
  }
#endif
}

bool HDMapAVPState::Init() {
  return true;
}

bool HDMapAVPState::SetRoutingInfo(
    TL::routing::RoutingResponse* const inrouting,
    std::string* const error_msg) {
  ACHECK(inrouting);
  ACHECK(error_msg);
  error_msg->clear();
  // Set routing info
  inrouting->clear_road();
  inrouting->clear_routing_request();

  std::pair<int, int> cur_idx = map_from_slam_.GetLaneIdx();
  ADEBUG << "The cur lane idx is: "
         << "[" << cur_idx.first << ", " << cur_idx.second << "]";

  for (int i = cur_idx.first; i <= cur_idx.second; ++i) {
    // add road
    auto* routing_road = inrouting->add_road();
    routing_road->set_id(absl::StrCat("road_hdmap_avp", i));

    auto* passage = routing_road->add_passage();
    passage->set_can_exit(false);
    passage->set_change_lane_type(routing::ChangeLaneType::FORWARD);

    // Set routing info
    auto* segment = passage->add_segment();
    const auto& cur_lane = map_from_slam_.GetLanes()[i];
    segment->set_id(cur_lane.id().id());
    int max_idx =
        cur_lane.central_curve().segment(0).line_segment().point_size();
    if (max_idx <= 0) {
      AERROR << "cur_lane is empty.";
      *error_msg = "cur_lane is empty.";
      return false;
    }
    segment->set_start_s(0.0);
    segment->set_end_s(cur_lane.length());

    if (i == cur_idx.second) {
      auto* routing_request = inrouting->mutable_routing_request();
      routing::LaneWaypoint waypoint;
      const auto& start_point = map_from_slam_.GetLanes()[cur_idx.first]
                                    .central_curve()
                                    .segment(0)
                                    .line_segment()
                                    .point(0);
      auto end_point =
          cur_lane.central_curve().segment(0).line_segment().point(max_idx - 1);
      if (i == map_from_slam_.GetLanes().size() - 1) {
        end_point.set_x(map_from_slam_.GetRoutingEndPoint().point3d().x());
        end_point.set_y(map_from_slam_.GetRoutingEndPoint().point3d().y());
        end_point.set_z(map_from_slam_.GetRoutingEndPoint().point3d().z());
      }
      const double end_s =
          (i == map_from_slam_.GetLanes().size() - 1)
              ? cur_lane.length() - FLAGS_hdmap_avp_path_extend_buffer
              : cur_lane.length();
      waypoint.set_id(map_from_slam_.GetLanes()[cur_idx.first].id().id());
      waypoint.mutable_pose()->set_x(start_point.x());
      waypoint.mutable_pose()->set_y(start_point.y());
      waypoint.set_s(0.0);
      routing_request->add_waypoint()->CopyFrom(waypoint);
      waypoint.set_id(cur_lane.id().id());
      waypoint.mutable_pose()->set_x(end_point.x());
      waypoint.mutable_pose()->set_y(end_point.y());
      waypoint.set_s(end_s);
      routing_request->add_waypoint()->CopyFrom(waypoint);

      ADEBUG << "waypoint_end:[" << waypoint.pose().x() << ","
             << waypoint.pose().y() << "]";
      common::util::FillHeader("hdmap_avp_routing", inrouting);
    }
  }
  return true;
}

bool HDMapAVPState::ProcessSimulation(
    const std::shared_ptr<LocalView>& local_view) {
  // 1.判断地图
  std::string message = {FLAGS_use_carla_read_map_simulation ? (" carla: ")
                                                             : (" baidu ")};
  if (hd_map_ == nullptr) {
    AERROR << message << " hamap is nullptr";
    return false;
  }
  if (!local_view->HasLocalization()) {
    AERROR << "No localization";
    return false;
  }
  if (parking_lot_array_enu_ptr_ == nullptr) {
    parking_lot_array_enu_ptr_ =
        std::make_shared<TL::perception::ParkingLotOutArray>();
    if (!hdmap::HDMapUtil::GetParkingLotFromMap(
            hd_map_, local_view->GetLocalization()->pose().position(),
            parking_lot_array_enu_ptr_.get())) {
      AERROR << message << " get parking lot from map failed";
      return false;
    }
  }
  perception::ParkingLotOutArray parking_lot_array =
      *parking_lot_array_enu_ptr_;
  common::util::FillHeader("parking_lots", &parking_lot_array);
  local_view->SetParkingLotOutArrayPtr(
      std::make_shared<TL::perception::ParkingLotOutArray>(
          parking_lot_array));
  if (!SetRoutingForBaiduSimulation(local_view, &routing_response_)) {
    AERROR << message << " routing is failed";
    return false;
  }
  if (hdmap_avp_ptr_ == nullptr) {
    hdmap_avp_ptr_ = std::make_shared<TL::navigation_hdmap::MapMsg>();
    hdmap::Map hd_map;
    hd_map_->GetMap(&hd_map);
    hdmap_avp_ptr_->mutable_hdmap()->Swap(&hd_map);
  }
  return true;
}

Status HDMapAVPState::ProcessBaiduSim(
    const std::shared_ptr<LocalView>& local_view) {
  if (!hd_map_) {
    return Status(ErrorCode::LOCALVIEW_FSM_HDMAPAVP_ERROR,
                  "failed to create hdmap");
  }
  if (!ProcessSimulation(local_view)) {
    const std::string msg = {
        "simulation mode :Read map and parking lots is fail"};
    AINFO << msg;
    return Status(ErrorCode::LOCALVIEW_FSM_HDMAPAVP_ERROR, msg);
  }

  local_view->SetRoutingResponsePtr(
      std::make_shared<routing::RoutingResponse>(routing_response_));
  local_view->SetMapMsgPtr(hdmap_avp_ptr_);
  local_view->SetHDMapPtr(hd_map_);

  return Status::OK();
}

Status HDMapAVPState::ProcessRealCarAndReplay(
    const std::shared_ptr<LocalView>& local_view) {
  const auto& fct_avp_in = local_view->GetFunctionManagerIn()->fct_avp_in();
  const auto& pose = local_view->GetLocalization()->pose();
  ADEBUG << "fct_mode:" << fct_avp_in.sys_mode()
         << "  run_state:" << fct_avp_in.sys_run_state()
         << "  avp_sim:" << FLAGS_enable_hdmap_avp_state_sim
         << "  pose  x:" << pose.position().x() << "  y:" << pose.position().y()
         << "  z:" << pose.position().z() << "  theta:" << pose.heading();

  // parking
  if (fct_avp_in.sys_mode() == functionmanager::AvpFctIn::LAPA &&
      fct_avp_in.sys_run_state() == functionmanager::AvpFctIn::PARKING) {
    if (FLAGS_enable_hdmap_avp_state_sim) {
      map_from_slam_.UpdateParkingLotOutArrayHeaderTime();
      local_view->SetParkingLotOutArrayPtr(
          std::make_shared<TL::perception::ParkingLotOutArray>(
              map_from_slam_.GetParkingLotOutArray()));
    }
    if (!local_view->HasValidParkingLotOutArrayHeader() ||
        local_view->GetParkingLotOutArray()->parking_lots().empty()) {
      const std::string msg = {"not valid parkinglotarray."};
      AERROR << msg;
      return Status(ErrorCode::LOCALVIEW_FSM_HDMAPAVP_ERROR, msg);
    }
    std::string error_msg;
    if (!AddParkingLotToMapAndRouting(local_view, &error_msg)) {
      AERROR << error_msg;
      return Status(ErrorCode::LOCALVIEW_FSM_HDMAPAVP_ERROR, error_msg);
    }
  }

  int cur_lane_id = 0;
  // cruising
  if (fct_avp_in.sys_command() != functionmanager::AvpFctIn::PARKINCONTROL) {
    auto fct_out = std::make_shared<functionmanager::FunctionManagerOut>();
    if (local_view->HasFunctionManagerOut()) {
      fct_out->CopyFrom(*local_view->GetFunctionManagerOut());
    }
    AVPMapState cur_map_state =
        (hdmap_avp_ptr_ == nullptr) ? AVPMapState::INIT_MAP
        : (map_from_slam_.IsMapRebuild(pose, fct_out, &cur_lane_id))
            ? AVPMapState::REBUILD_MAP
        : (map_from_slam_.IsLaneUpdate(pose)) ? AVPMapState::UPDATE_LANE
                                              : AVPMapState::DEFAULT;
    std::string error_msg;
    if (map_content_ == nullptr) {
#if defined(ISX86) || defined(FOR_BAIDU_SIMULATION)
      auto& manager = MapManage::getInstance(FLAGS_avp_map_path_file);
#else
      auto& manager = MapManage::getInstance();
#endif
      map_content_ = manager.getMapPlanning();
      if (map_content_ == nullptr) {
        error_msg = "read map file error.";
        AERROR << error_msg;
        return Status(ErrorCode::LOCALVIEW_FSM_HDMAPAVP_ERROR, error_msg);
      }
    }
    switch (cur_map_state) {
      case AVPMapState::INIT_MAP:
      case AVPMapState::REBUILD_MAP: {
        if (!map_from_slam_.ConstructMap(map_content_->map, map_content_->path,
                                         pose, cur_map_state, fct_out,
                                         &error_msg)) {
          AERROR << error_msg;
          return Status(ErrorCode::LOCALVIEW_FSM_HDMAPAVP_ERROR, error_msg);
        }
        TL::navigation_hdmap::MapMsg hdmap_avp;
        hdmap_avp.mutable_hdmap()->mutable_parking_space()->CopyFrom(
            map_from_slam_.GetMap().parking_space());
        hdmap_avp.mutable_hdmap()->mutable_lane()->CopyFrom(
            map_from_slam_.GetMap().lane());
        hdmap_avp.mutable_hdmap()->mutable_yield()->CopyFrom(
            map_from_slam_.GetMap().yield());
        hdmap_avp.mutable_hdmap()->mutable_speed_bump()->CopyFrom(
            map_from_slam_.GetMap().speed_bump());
        hdmap_avp.mutable_hdmap()->mutable_pillar()->CopyFrom(
            map_from_slam_.GetMap().pillar());
        // hdmap_avp.mutable_hdmap()->mutable_overlap()->CopyFrom(
        //     map_from_slam_.GetMap().overlap());
        if (!SetRoutingInfo(&routing_response_, &error_msg)) {
          return Status(ErrorCode::LOCALVIEW_FSM_HDMAPAVP_ERROR, error_msg);
        }
        hdmap_avp_ptr_ =
            std::make_shared<TL::navigation_hdmap::MapMsg>(hdmap_avp);
        common::util::FillHeader("hdmap_avp_map", hdmap_avp_ptr_.get());
        common::util::FillHeader(
            "hdmap_avp_map", hdmap_avp_ptr_->mutable_hdmap()->mutable_header());
        hd_map_ = hdmap::CreateMap(*hdmap_avp_ptr_);
        break;
      }
      case AVPMapState::UPDATE_LANE: {
        if (!map_from_slam_.ConstructMap(map_content_->map, map_content_->path,
                                         pose, cur_map_state, fct_out,
                                         &error_msg)) {
          AERROR << error_msg;
          return Status(ErrorCode::LOCALVIEW_FSM_HDMAPAVP_ERROR, error_msg);
        }
        hd_map_->UpdateMapFromProto(map_from_slam_.GetExtendMap(),
                                    map_from_slam_.GetShrinkMap(),
                                    "hdmap_avp_map");
        hdmap::Map hd_map;
        hd_map_->GetMap(&hd_map);
        hdmap_avp_ptr_->mutable_hdmap()->Swap(&hd_map);
        if (!SetRoutingInfo(&routing_response_, &error_msg)) {
          return Status(ErrorCode::LOCALVIEW_FSM_HDMAPAVP_ERROR, error_msg);
        }
        break;
      }
      case AVPMapState::DEFAULT:
        break;
      default:
        error_msg = "hdmap avp map unknown map state.";
        return Status(ErrorCode::LOCALVIEW_FSM_HDMAPAVP_ERROR, error_msg);
    }
    if (map_from_slam_.IsPoseNearEnd(pose, *local_view)) {
      error_msg = "cur pose near end.";
      return Status(ErrorCode::LOCALVIEW_FSM_HDMAPAVP_ERROR, error_msg);
    }
    local_view->SetFunctionManagerOutPtr(fct_out);
  }
  auto ptr_trajectory_pb = std::make_shared<ADCTrajectory>();
  if (local_view->HasADCTrajectory()) {
    ptr_trajectory_pb->CopyFrom(*local_view->GetADCTrajectory());
  }
  SendEndPoint2ADC(local_view, ptr_trajectory_pb.get());

  local_view->SetADCTrajectoryPtr(ptr_trajectory_pb);
  local_view->SetRoutingResponsePtr(
      std::make_shared<routing::RoutingResponse>(routing_response_));
  local_view->SetMapMsgPtr(hdmap_avp_ptr_);
  local_view->SetHDMapPtr(hd_map_);

  return Status::OK();
}

Status HDMapAVPState::BuildLocalView(
    const std::shared_ptr<LocalView>& local_view) {
  if (!local_view->HasLocalization()) {
    return Status(ErrorCode::LOCALVIEW_FSM_HDMAPAVP_ERROR,
                  "no localization estimate.");
  }
#if defined(ISMDC) || defined(ISORIN)
  constexpr uint32_t kVioAndWoDR = 8;
  if (local_view->GetLocalization()->location_state() != kVioAndWoDR) {
    const std::string msg =
        "location_state is " +
        std::to_string(local_view->GetLocalization()->location_state());
    return Status(ErrorCode::LOCALVIEW_FSM_HDMAPAVP_ERROR, msg);
  }
#endif
  bool baidu_worldsim = false;
#ifdef FOR_BAIDU_SIMULATION
  std::filesystem::path file_path(FLAGS_avp_map_path_file);
  if (!std::filesystem::exists(file_path)) {
    baidu_worldsim = true;
  }
#endif
  // baidu worldsim
  if (FLAGS_use_carla_read_map_simulation || baidu_worldsim) {
    return ProcessBaiduSim(local_view);
  }
  // real car, mcap replay, baidu logsim, local sim
  return ProcessRealCarAndReplay(local_view);
}

void HDMapAVPState::SendEndPoint2ADC(
    const std::shared_ptr<LocalView>& local_view,
    ADCTrajectory* const ptr_trajectory_pb) {
  if (ptr_trajectory_pb == nullptr) {
    AERROR << "ptr_trajectory_pb is nullptr.";
    return;
  }
  const auto& fct_avp_in = local_view->GetFunctionManagerIn()->fct_avp_in();
  const auto& pose = local_view->GetLocalization()->pose();
  auto* avp_to_hmi = ptr_trajectory_pb->mutable_avp_to_hmi();

  // set point_with_yaw
  avp_to_hmi->clear_point_with_yaw();
  hmi::PointWithYaw local_end_point;
  const auto& global_end_point = map_from_slam_.GetRoutingEndPoint().point3d();
  constexpr double kDestinationDis = 20.0;
  const double square_dis =
      std::pow(global_end_point.x() - pose.position().x(), 2) +
      std::pow(global_end_point.y() - pose.position().y(), 2);
  if (square_dis < kDestinationDis * kDestinationDis &&
      fct_avp_in.sys_command() != functionmanager::AvpFctIn::PARKINCONTROL) {
    std::pair<double, double> flu_point_temp = common::math::ENUToFLU(
        global_end_point.x(), global_end_point.y(), pose.position().x(),
        pose.position().y(), pose.heading());
    local_end_point.mutable_point3d()->set_x(flu_point_temp.first);
    local_end_point.mutable_point3d()->set_y(flu_point_temp.second);
    local_end_point.mutable_point3d()->set_z(global_end_point.z() -
                                             pose.position().z());
    local_end_point.set_yaw(map_from_slam_.GetRoutingEndPoint().yaw() -
                            pose.heading());
    avp_to_hmi->mutable_point_with_yaw()->CopyFrom(local_end_point);
  }
}

bool HDMapAVPState::AddParkingLotToMapAndRouting(
    const std::shared_ptr<LocalView>& local_view,
    std::string* const error_msg) {
  if (error_msg == nullptr) {
    return false;
  }
  if (!local_view->HasParkingLotOutArray()) {
    *error_msg = "not valid parkinglotarray.";
    return false;
  }
  if (!hdmap_avp_ptr_) {
    *error_msg = "failed to create hdmap";
    return false;
  }
  perception::ParkingLotOut perception_parking_lot;
  bool perception_parking_lot_is_found = false;
  TL::hdmap::ParkingSpace opt_parking_lot;
  const auto& parking_lot_out_array = local_view->GetParkingLotOutArray();
  for (const auto& parking_space : parking_lot_out_array->parking_lots()) {
    if (parking_space.parking_seq() ==
        parking_lot_out_array->opt_parking_seq()) {
      perception_parking_lot = parking_space;
      perception_parking_lot_is_found = true;
      break;
    }
  }
  if (!perception_parking_lot_is_found) {
    *error_msg = "perception parking lot is not found";
    return false;
  }
  opt_parking_lot.mutable_id()->set_id(
      "LAPA_" + std::to_string(perception_parking_lot.parking_seq()));
  if (perception_parking_lot.pts_enu_size() < 4) {
    *error_msg = "perception parking space size less than 4";
    return false;
  }
  for (int i = 0; i < 4; i++) {
    opt_parking_lot.mutable_polygon()->add_point();
  }
  for (int j = 0; j < perception_parking_lot.pts_enu_size(); j++) {
    int idx = -1;
    switch (perception_parking_lot.pts_enu().at(j).position()) {
      case TL::perception::PSPoint_Position_TOP_LEFT:
        idx = 0;
        break;
      case TL::perception::PSPoint_Position_TOP_RIGHT:
        idx = 1;
        break;
      case TL::perception::PSPoint_Position_BOTTOM_RIGHT:
        idx = 2;
        break;
      case TL::perception::PSPoint_Position_BOTTOM_LEFT:
        idx = 3;
        break;
      default:
        break;
    }
    if (idx > -1) {
      opt_parking_lot.mutable_polygon()->mutable_point()->at(idx).set_x(
          perception_parking_lot.pts_enu().at(j).point().x());
      opt_parking_lot.mutable_polygon()->mutable_point()->at(idx).set_y(
          perception_parking_lot.pts_enu().at(j).point().y());
    }
  }

  // add perception parking lot to map
  auto p_it = hdmap_avp_ptr_->hdmap().parking_space().begin();
  while (p_it != hdmap_avp_ptr_->hdmap().parking_space().end()) {
    const auto& parking_id = p_it->id().id();
    if (absl::StrContains(parking_id, "LAPA_")) {
      p_it =
          hdmap_avp_ptr_->mutable_hdmap()->mutable_parking_space()->erase(p_it);
    } else {
      p_it++;
    }
  }
  hdmap_avp_ptr_->mutable_hdmap()->add_parking_space()->CopyFrom(
      opt_parking_lot);
  hd_map_ = hdmap::CreateMap(*hdmap_avp_ptr_);

  // add perception parking lot id to routing
  auto* const parking_info =
      routing_response_.mutable_routing_request()->mutable_parking_info();
  parking_info->set_parking_space_id(opt_parking_lot.id().id());

  return true;
}

bool HDMapAVPState::SetRoutingForBaiduSimulation(
    const std::shared_ptr<LocalView>& local_view,
    routing::RoutingResponse* routing_response_ptr) {
  routing::POI poi;
  routing::RoutingRequest routing_request;
  routing_request.Clear();
  if (!common::GetProtoFromASCIIFile(hdmap::EndWayPointFile(),
                                     &poi)) {  // NOLINT
    AERROR << "Failed to load end way point file: " << hdmap::EndWayPointFile();
    return false;
  }
  if (!local_view->HasLocalization()) {
    AERROR << "No localization estimate.";
    return false;
  }

  if (poi.landmark_size() > 0) {
    for (int32_t j = 0; j < poi.landmark(0).waypoint_size(); j++) {
      routing_request.add_waypoint()->CopyFrom(poi.landmark(0).waypoint(j));
    }
    AINFO << "waypoint size = " << routing_request.waypoint_size();
  } else {
    AERROR << "No waypoint in poi";
    return false;
  }

  if (!common::util::IsProtoEqual(*last_routing_request_, routing_request)) {
    *last_routing_request_ = routing_request;
    AINFO << "update routing request";
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
    constexpr double kRadius = 0.3;
    TL::common::PointENU start_waypoint =
        common::util::PointFactory::ToPointENU(
            routing_request.waypoint(0).pose());
    hd_map_->GetLanes(start_waypoint, 20 * kRadius, &lanes);
    if (lanes.empty() && local_view->GetLocalization()->has_header()) {
      AINFO << "No lane found, use vehicle position instead start waypoint";
      // start waypoint is not on road, replace it by vehicle position
      routing_request.mutable_waypoint(0)->mutable_pose()->set_x(
          local_view->GetLocalization()->pose().position().x());
      routing_request.mutable_waypoint(0)->mutable_pose()->set_y(
          local_view->GetLocalization()->pose().position().y());
    }
    if (!routing_.Process(
            std::make_shared<routing::RoutingRequest>(routing_request),
            routing_response_ptr)) {
      AERROR << "Failed to process routing response";
      return false;
    }
    common::util::FillHeader("from_file_routing", routing_response_ptr);
  }

  if (FLAGS_use_carla_read_map_simulation) {
    common::util::FillHeader("from_file_routing", routing_response_ptr);
  }

  return true;
}

}  // namespace planning
}  // namespace TL
