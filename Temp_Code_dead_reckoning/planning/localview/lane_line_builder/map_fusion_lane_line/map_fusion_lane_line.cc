/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#include "planning/localview/lane_line_builder/map_fusion_lane_line/map_fusion_lane_line.h"

#include <algorithm>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/time/clock.h"
#include "common/util/message_util.h"
#include "common/utm_projection/utm_zone.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/localview/lane_line_builder/lane_line_base.h"
#include "planning/localview/local_view.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/st_obstacle_cache.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/map/map_id.pb.h"
#include "proto/map/navigation.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/routing/poi.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {

using functionmanager::MachineStateType;
using google::protobuf::uint32;
using TL::common::Status;
using TL::common::VehicleState;
using TL::hdmap::PassageAndVehilceRelation;  // NOLINT
using PairChanglaneType = std::pair<double, routing::PerceptionChangeLaneType>;

Status MapFusionLaneLine::Init(
    const std::shared_ptr<LocalViewData>& local_view_data) {
  local_view_data_ = local_view_data;
  inside_active_conditon_ =
      std::make_unique<InsideActiveCondition>(local_view_data);
  pnc_map_ = std::make_shared<hdmap::PncMap>();
  map_fusion_vec_.reserve(30);
  if (!FLAGS_is_record_replay) {
    hdmap_thread_ = std::thread(&MapFusionLaneLine::SetMapRouting, this);
  }
  return Status::OK();
}

Status MapFusionLaneLine::Init() {
  return Status::OK();
}

Status MapFusionLaneLine::Start() {
  return Status::OK();
}

void MapFusionLaneLine::Stop() {}

void MapFusionLaneLine::SetMapRouting() {
  pthread_setname_np(pthread_self(), "SetMapRouting");
  double time_prev = 0.0;
  while (!is_exit_) {
    std::shared_ptr<const navigation_hdmap::MapMsg> temp{nullptr};
    {
      std::unique_lock<std::mutex> hdmap_guard(hdmap_mutex_);
      mapmsg_cv_.wait(hdmap_guard, [&]() {
        return (common::Clock::NowInMicroseconds() - time_prev >
                    time_interval_ &&
                !map_fusion_vec_.empty()) ||
               is_exit_;
      });
      if (is_exit_) {
        return;
      }
      temp = map_fusion_vec_.back();
      map_fusion_vec_.clear();
    }

    hdmap_bak_ = std::make_shared<hdmap::HDMap>();
    hdmap_bak_->LoadMapFromProto(temp->hdmap());
    if (hdmap_bak_->Empty()) {
      AERROR << " lane_size: " << temp->hdmap().lane_size()
             << "HEADER:" << temp->hdmap().header().DebugString();
    }
    routing_response_bak_ =
        std::make_shared<routing::RoutingResponse>(temp->routing());

    {
      std::lock_guard<std::mutex> hdmap_guard(hdmap_mutex_);
      hdmap_ = hdmap_bak_;
      current_routing_response_ = routing_response_bak_;
    }
    time_prev = TL::common::Clock::NowInMicroseconds();
  }
}

bool MapFusionLaneLine::Process(
    const std::shared_ptr<LocalView>& local_view,
    functionmanager::FunctionManagerOut* const to_fct) {

  auto* realhdmap_debug = to_fct->mutable_real_hdmap_debug();
  std::string msg = " ";
  if (!FLAGS_is_record_replay &&
      (!local_view->HasMapMsg() || !local_view->GetMapMsg()->has_routing() ||
       (local_view->HasMapMsg() && local_view->GetMapMsg()->has_routing() &&
        local_view->GetMapMsg()->routing().has_header() &&
        local_view->GetMapMsg()->routing().header().has_frame_id() &&
        local_view->GetMapMsg()->routing().header().frame_id() !=
            "map_fusion_routing"))) {
    msg = "local_view has no map_fusion or routing.";
    AERROR << msg;
    return ReturnStatus(to_fct, false, msg);
  }
  if (FLAGS_is_record_replay &&
      (!local_view->HasRoutingResponse() ||
       (local_view->HasRoutingResponse() &&
        local_view->GetRoutingResponse()->has_header() &&
        local_view->GetRoutingResponse()->header().has_frame_id() &&
        local_view->GetRoutingResponse()->header().frame_id() !=
            "map_fusion_routing" &&
        local_view->GetRoutingResponse()->header().frame_id() !=
            "from_ehp_routing"))) {
    msg = "recrod replay no routing.";
    AERROR << msg;
    return ReturnStatus(to_fct, false, msg);
  }
  if (local_view->HasMapMsg() && local_view->GetMapMsg()->has_routing() &&
      local_view->GetMapMsg()->routing().header().has_status() &&
      local_view->GetMapMsg()->routing().header().status().has_error_code() &&
      local_view->GetMapMsg()->routing().header().status().error_code() !=
          common::ErrorCode::OK) {
    msg = "map_fusion time out.";
    AERROR << msg;
    return ReturnStatus(to_fct, false, msg);
  }
  if (local_view->HasMapMsg() && local_view->GetMapMsg()->has_map_type()) {
    const auto map_type = local_view->GetMapMsg()->map_type();
    is_map_type_change_ =
        (map_type == TL::navigation_hdmap::MapMsg_MapType_FUSION_NNP_MAP ||
         map_type == TL::navigation_hdmap::MapMsg_MapType_FUSION_NCP_MAP) &&
        his_fusion_maptype_ ==
            TL::navigation_hdmap::MapMsg_MapType_PERCEP_MAP;
    his_fusion_maptype_ = map_type;
  }
  if (FLAGS_is_record_replay) {
    if (local_view->HasMapMsg() && local_view->HasRoutingResponse()) {
      hdmap_local_ = std::make_shared<hdmap::HDMap>();
      hdmap_local_->LoadMapFromProto(local_view->GetMapMsg()->hdmap());
      routing_response_local_ = std::make_shared<routing::RoutingResponse>(
          *local_view->GetRoutingResponse());
    } else if (!hdmap_local_) {
      AERROR << "hdmap_local is nullptr.";
      return ReturnStatus(to_fct, false, msg);
    }
  } else {
    if (local_view->HasMapMsg()) {
      std::lock_guard<std::mutex> hdmap_guard(hdmap_mutex_);
      if ((local_view->HasMapMsg() && local_view->GetMapMsg()->has_map_type() &&
           local_view->GetMapMsg()->map_type() ==
               TL::navigation_hdmap::MapMsg_MapType_PERCEP_MAP) ||
          is_map_type_change_) {
        time_interval_ = FLAGS_load_perception_map_interval;
      } else {
        time_interval_ = FLAGS_load_nnp_ncp_map_interval;
      }
      map_fusion_vec_.emplace_back(local_view->GetMapMsg());
      hdmap_local_ = hdmap_;
      routing_response_local_ = current_routing_response_;
    }
    mapmsg_cv_.notify_one();
  }
  // 匹配mapfusion地图类型
  DealMaptype(local_view, to_fct);
  if (local_view->HasVehicleState()) {
    vehicle_state_ = local_view->GetVehicleState();
  } else {
    msg = "local_view VehicleState is error";
    AERROR << msg;
    return ReturnStatus(to_fct, false, msg);
  }

  if (hdmap_local_ == nullptr || vehicle_state_ == nullptr ||
      routing_response_local_ == nullptr ||
      (local_view->HasMapMsg() && local_view->GetMapMsg()->has_is_valid() &&
       !local_view->GetMapMsg()->is_valid())) {
    msg =
        "local_view hd_map/vehicle_state/routing_response_local_ == "
        "nullptr.";
    AERROR << msg;
    return ReturnStatus(to_fct, false, msg);
  }
  if (to_fct != nullptr && to_fct->has_fsm_state() &&
      to_fct->fsm_state() == functionmanager::HDMAP_TYPE) {
    AddVirtualLaneLine(local_view);
  }

  if (now_fusion_maptype_ == navigation_hdmap::MapMsg_MapType_PERCEP_MAP) {
    ADEBUG << "get change map_fusion";
    // 更新pncmap
    pnc_map_->UpdateHDMap(hdmap_local_);
    pnc_map_->UpdateReferenceLineInfoConfig(
        MachineStateType::HDMAP_TYPE, functionmanager::AvpFctOut::CRUISING,
        std::fabs(local_view->GetVehicleState()->linear_velocity()),
        local_view->GetFunctionManagerIn()
            ->fct_nnp_in()
            .longitud_ctrl_cruise_speedms());
    pnc_map_->UpdateRoutingResponse(*routing_response_local_);
    bool update_vehicle_state_success =
        pnc_map_->UpdateVehicleState(*local_view->GetVehicleState());
    const auto& adc_waypoint = pnc_map_->GetADCWaypoint();
    std::list<TL::hdmap::RouteSegments> segments;
    bool get_route_segments_success =
        pnc_map_->GetRouteSegments(*local_view->GetVehicleState(), &segments,
                                   local_view->GetFunctionManagerIn()
                                       ->fct_nnp_in()
                                       .longitud_ctrl_cruise_speedms());
    if (adc_waypoint.lane == nullptr || !update_vehicle_state_success ||
        !get_route_segments_success || segments.empty()) {
      std::string str_is_lane_nullptr =
          adc_waypoint.lane != nullptr ? "true" : "false";
      std::string str_update_vehicle_state_success =
          update_vehicle_state_success ? "true" : "false";
      std::string str_get_route_segments_success =
          get_route_segments_success ? "true" : "false";
      std::string is_segments_empty = segments.empty() ? "true" : "false";
      std::string out_ehp_message =
          "adc_waypoint.lane != nullptr: " + str_is_lane_nullptr +
          " ; update_vehicle_state_success: " +
          str_update_vehicle_state_success +
          " ; get_route_segments_success: " + str_get_route_segments_success +
          " ; is_segments_empty: " + is_segments_empty;
      realhdmap_debug->mutable_location_err_debug()->set_loc_err_reason(
          out_ehp_message);
      return ReturnLocalMapStatus(to_fct, false);
    }
    // 进行路口检查是否降级到跟车模式标志位
    Fusion2NolaneChecker(local_view);
    // 只要自车和临车道的passage长度足够地图就可用
    double adc_passage_length = -100.0;
    std::string left_id{" "};
    std::string right_id{" "};
    if (!adc_waypoint.lane->lane().left_neighbor_forward_lane_id().empty()) {
      left_id =
          adc_waypoint.lane->lane().left_neighbor_forward_lane_id().at(0).id();
    }
    if (!adc_waypoint.lane->lane().right_neighbor_forward_lane_id().empty()) {
      right_id =
          adc_waypoint.lane->lane().right_neighbor_forward_lane_id().at(0).id();
    }
    for (const auto& seg : segments) {
      for (const auto& lane : seg) {
        if (lane.lane->id().id() == adc_waypoint.lane->id().id() ||
            lane.lane->id().id() == left_id ||
            lane.lane->id().id() == right_id) {
          auto length = seg.GetAdcDistanceToRouteEndPoint();
          if (length > adc_passage_length) {
            adc_passage_length = length;
          }
        }
      }
    }
    constexpr double kMinPreTime = 7.0;
    constexpr double kMinPassageLength = 20.0;
    constexpr double kMaxPassageLength = 200.0;
    const double min_length = std::min(
        std::max(std::fabs(vehicle_state_->linear_velocity()) * kMinPreTime,
                 kMinPassageLength),
        kMaxPassageLength);
    if (adc_passage_length < min_length) {
      std::string out_ehp_message =
          "passage_length too low: " +
          std::to_string(std::floor(adc_passage_length)) + " < " +
          std::to_string(std::floor(min_length));
      realhdmap_debug->mutable_location_err_debug()->set_loc_err_reason(
          out_ehp_message);
      return ReturnLocalMapStatus(to_fct, false);
    }

    inside_active_conditon_->MapFusionProcess(local_view, pnc_map_,
                                              hdmap_local_, to_fct, true);
    return ReturnLocalMapStatus(to_fct, true);
  }

  const bool is_first =
      to_fct->hdmap_sub_state() == functionmanager::MAP_FUSION_TYPE ||
      local_view_data_->update_data()->is_nnp_drive_auto() || is_in_hdmap_;
  is_in_hdmap_ = is_first ? JudgeIsInMapContinuously(local_view, hdmap_local_)
                          : JudgeIsInMapFirstly(local_view, hdmap_local_);
  to_fct->mutable_nnp_fct_out()->set_is_in_hdmap(is_in_hdmap_);
  realhdmap_debug->set_real_is_in_hdmap(is_in_hdmap_);
  realhdmap_debug->set_update_routing_request(true);
  realhdmap_debug->set_update_routing_response(true);
  if (!is_in_hdmap_) {
    msg = "is_in_hdmap is false";
    AERROR << msg;
    return ReturnStatus(to_fct, false, msg);
  }

  // 更新pncmap

  pnc_map_->UpdateHDMap(hdmap_local_);
  pnc_map_->UpdateReferenceLineInfoConfig(
      MachineStateType::HDMAP_TYPE, functionmanager::AvpFctOut::CRUISING,
      std::fabs(local_view->GetVehicleState()->linear_velocity()),
      local_view->GetFunctionManagerIn()
          ->fct_nnp_in()
          .longitud_ctrl_cruise_speedms());
  pnc_map_->UpdateRoutingResponse(*routing_response_local_);
  pnc_map_->UpdateVehicleState(*local_view->GetVehicleState());
  const auto& adc_waypoint = pnc_map_->GetADCWaypoint();
  if (adc_waypoint.lane != nullptr) {
    to_fct->mutable_nnp_fct_out()->set_curr_lane_spd_km(
        static_cast<uint32>(adc_waypoint.lane->lane().speed_limit() * 3.6));
  }

  if (!SolveChangeLaneType(local_view)) {
    if (pnc_map_) {
      msg = pnc_map_->GetPncMapErrorInfo();
    }
    msg = msg + " ; " + " pnc map is fail!";
    AERROR << msg;
    return ReturnStatus(to_fct, false, msg);
  }
  GetAdcToOtherLaneTypeLen(to_fct);
  // map fusion下定位无故障
  to_fct->mutable_nnp_fct_out()
      ->mutable_nnp_statechange_conditions()
      ->set_location_err_state(0);
  inside_active_conditon_->MapFusionProcess(local_view, pnc_map_, hdmap_local_,
                                            to_fct, false);

  // ncp下先让激活条件满足，下发mcu ncp状态
  if (local_view->HasMapMsg() && local_view->GetMapMsg()->has_map_type() &&
      local_view->GetMapMsg()->map_type() ==
          TL::navigation_hdmap::MapMsg_MapType_FUSION_NCP_MAP) {
    auto* nnp_activation_conditions =
        to_fct->mutable_nnp_fct_out()->mutable_hd_map_active_status();
    nnp_activation_conditions->set_vehicle_not_in_forbidlane(true);
    nnp_activation_conditions->set_vehicle_not_in_reverselane(true);
    nnp_activation_conditions->set_vehicle_not_in_otherforbidarea(true);
    nnp_activation_conditions->set_appropriate_current_lane_curve(true);
    nnp_activation_conditions->set_appropriate_current_lane_headingerr(true);
    nnp_activation_conditions->set_appropriate_current_lane_width(true);
    nnp_activation_conditions->set_vehicle_in_hdmap(true);
    nnp_activation_conditions->set_valid_of_lane_localization(true);
    nnp_activation_conditions->set_valid_of_lane_routing(true);
    auto soc_03_val = to_fct->soc_2_fct_tbd_u32_03();
    to_fct->set_soc_2_fct_tbd_u32_03(soc_03_val | kIsNcpMode);
  }
  // to_fct->set_soc_2_fct_tbd_u32_04(0x13);
  // to_fct->set_fsm_sequence_num(0);
  // to_fct->set_fsm_state(functionmanager::MachineStateType::HDMAP_TYPE);
  // to_fct->set_hdmap_sub_state(functionmanager::MAP_FUSION_TYPE);
  return ReturnStatus(to_fct, true, msg);
}

void MapFusionLaneLine::AddVirtualLaneLine(
    const std::shared_ptr<LocalView>& local_view) {
  if (local_view == nullptr || !local_view->HasLocalization() ||
      pnc_map_ == nullptr || pnc_map_->GetADCWaypoint().lane == nullptr ||
      !local_view->HasLaneMarkers() ||
      (now_fusion_maptype_ !=
           TL::navigation_hdmap::MapMsg_MapType_FUSION_NNP_MAP &&
       now_fusion_maptype_ !=
           TL::navigation_hdmap::MapMsg_MapType_PERCEP_MAP)) {
    return;
  }
  const auto& lane_markers = local_view->GetLaneMarkers();
  const auto& adc_way_point_lane = pnc_map_->GetADCWaypoint().lane;
  bool next_left_is_copy =
      lane_markers->front_next_left_lane_marker().empty() &&
      lane_markers->has_front_left_lane_marker() &&
      lane_markers->front_left_lane_marker().lane_type() ==
          hdmap::LaneBoundaryType::DOTTED_WHITE &&
      lane_markers->has_front_left_road_edge() &&
      !adc_way_point_lane->lane().left_neighbor_forward_lane_id().empty();
  bool next_right_is_copy =
      lane_markers->front_next_right_lane_marker().empty() &&
      lane_markers->has_front_right_lane_marker() &&
      lane_markers->front_right_lane_marker().lane_type() ==
          hdmap::LaneBoundaryType::DOTTED_WHITE &&
      lane_markers->has_front_right_road_edge() &&
      !adc_way_point_lane->lane().right_neighbor_forward_lane_id().empty();
  if (!next_left_is_copy && !next_right_is_copy) {
    return;
  }
  perception::LaneMarkers lane_markers_current = *local_view->GetLaneMarkers();
  perception::LaneMarker next_left_lanemarker{};
  perception::LaneMarker next_right_lanemarker{};
  if (next_left_is_copy) {
    auto left_lane_id =
        adc_way_point_lane->lane().left_neighbor_forward_lane_id().at(0);
    double left_lane_width = GetLaneWidth(local_view, left_lane_id);
    if (left_lane_width > 0.01) {
      VirtualLaneLine(&next_left_lanemarker,
                      lane_markers->front_left_lane_marker(),
                      lane_markers->front_left_road_edge(), left_lane_width);

      lane_markers_current.add_front_next_left_lane_marker()->CopyFrom(
          next_left_lanemarker);
    }
  }
  if (next_right_is_copy) {
    auto right_lane_id =
        adc_way_point_lane->lane().right_neighbor_forward_lane_id().at(0);
    double right_lane_width = GetLaneWidth(local_view, right_lane_id);
    if (right_lane_width > 0.01) {
      VirtualLaneLine(&next_right_lanemarker,
                      lane_markers->front_right_lane_marker(),
                      lane_markers->front_right_road_edge(), -right_lane_width);
      lane_markers_current.add_front_next_right_lane_marker()->CopyFrom(
          next_right_lanemarker);
    }
  }
  local_view->SetLaneMarkersPtr(
      std::make_shared<perception::LaneMarkers>(lane_markers_current));
}

double MapFusionLaneLine::GetLaneWidth(
    const std::shared_ptr<LocalView>& local_view,
    const TL::hdmap::Id& lane_id) {
  const auto& left_lane = hdmap_local_->GetLaneById(lane_id);
  if (left_lane == nullptr) {
    return 0.0;
  }
  double s = 0.0;
  double l = 0.0;
  common::math::Vec2d point{0.0, 0.0};
  point.set_x(local_view->GetLocalization()->pose().position().x());
  point.set_y(local_view->GetLocalization()->pose().position().y());
  if (!left_lane->GetProjection(point, &s, &l)) {
    return 0.0;
  }
  return left_lane->GetWidth(s);
}

void MapFusionLaneLine::VirtualLaneLine(
    perception::LaneMarker* next_lane_marker,
    const perception::LaneMarker& lane_marker,
    const perception::LaneMarker& road_edge, double width) {
  next_lane_marker->CopyFrom(lane_marker);
  next_lane_marker->set_lane_type(
      hdmap::LaneBoundaryType_Type::LaneBoundaryType_Type_SOLID_WHITE);
  next_lane_marker->set_c0_position(std::min(lane_marker.c0_position() + width,
                                             road_edge.c0_position() - 0.2));
}

const std::shared_ptr<navigation_hdmap::MapMsg>& MapFusionLaneLine::GetMapMsg(
    bool refresh) {
  UNUSED(refresh);
  empty_map_msg_ = std::make_shared<navigation_hdmap::MapMsg>();
  common::util::FillHeader("from_file_hdmap", empty_map_msg_.get());
  return empty_map_msg_;  // NOLINT
}

void MapFusionLaneLine::GetAdcToOtherLaneTypeLen(
    functionmanager::FunctionManagerOut* const to_fct) {
  const auto& front_ramp_count_len =
      pnc_map_->CheckFrontRampAndLaneChangeCount();
  auto front_other_type_lane_and_len =
      pnc_map_->GetFrontOtherTypeLaneAndLength();
  double to_ramp_len = 1e-6;
  double to_mainroad_len = 1e-6;
  uint32_t to_ramp_lane_count = 0;
  if (pnc_map_->AdcInMainRoad() && front_ramp_count_len.first > 0) {
    to_ramp_len = front_ramp_count_len.second;
    to_ramp_lane_count = front_ramp_count_len.first;
  }
  const auto& front_lane = front_other_type_lane_and_len.first;
  if (front_lane != nullptr && front_lane->IsRampRoad()) {
    to_ramp_len = front_other_type_lane_and_len.second;
    to_ramp_lane_count = 0;
  }
  if (front_lane != nullptr && front_lane->IsMainRoad()) {
    to_mainroad_len = front_other_type_lane_and_len.second;
  }
  to_fct->mutable_nnp_fct_out()->set_nnp_d_distance2_onramp_sg(
      to_mainroad_len);  // NOLINT
  to_fct->mutable_nnp_fct_out()->set_nnp_d_distance2_downramp_sg(
      to_ramp_len);  // NOLINT
  to_fct->mutable_nnp_fct_out()->set_to_ramp_lane_count(to_ramp_lane_count);
}

bool MapFusionLaneLine::SolveChangeLaneType(
    const std::shared_ptr<LocalView>& local_view) {
  std::list<TL::hdmap::RouteSegments> segments;
  if (pnc_map_->GetRouteSegments(*local_view->GetVehicleState(), &segments,
                                 local_view->GetFunctionManagerIn()
                                     ->fct_nnp_in()
                                     .longitud_ctrl_cruise_speedms())) {
    pnc_map_->ProcessFollowUpOdd();
    pnc_map_->ProcessDistanceFromADCToOddStartEnd();
    return true;
  }
  AERROR << " pnc map is fail ";
  return false;
}

void MapFusionLaneLine::Fusion2NolaneChecker(
    const std::shared_ptr<LocalView>& local_view) {

  constexpr double kMinPreLength = 15.0;
  const auto& adc_waypoint = pnc_map_->GetADCWaypoint();
  if (adc_waypoint.lane == nullptr) {
    return;
  }
  auto adc_lane = adc_waypoint.lane;
  bool adc_road_is_virtual = adc_lane->lane().left_boundary().virtual_() &&
                             adc_lane->lane().right_boundary().virtual_();
  double now_adc_lane_length = adc_lane->total_length() - adc_waypoint.s;
  double lane_length = now_adc_lane_length;
  // 15米内能够找到后继与当前不一样的
  bool successor_lane_is_virtual = adc_road_is_virtual;
  double heading_err = 0.0;
  int successor_id_size = adc_lane->lane().successor_id().size();
  //
  auto DealSuccessor = [&](TL::hdmap::LaneInfoConstPtr* lane,
                           double length) {
    int count{0};
    while (length < kMinPreLength && count < 10) {
      count++;
      if ((*lane)->lane().successor_id().empty()) {
        break;
      }
      auto lane_id = (*lane)->lane().successor_id().at(0);
      ADEBUG << " success lane id: " << lane_id.id();
      *lane = hdmap_local_->GetLaneById(lane_id);
      if ((*lane) == nullptr) {
        break;
      }
      successor_lane_is_virtual = (*lane)->lane().left_boundary().virtual_() &&
                                  (*lane)->lane().right_boundary().virtual_();
      if (successor_lane_is_virtual != adc_road_is_virtual) {
        break;
      }
      length += (*lane)->overall_length();
    }
  };

  for (int i = 0; i < successor_id_size; ++i) {
    lane_length = now_adc_lane_length;
    if (lane_length > kMinPreLength) {
      double adc_pre_heading =
          adc_waypoint.lane->Heading(adc_waypoint.s + 10.0);
      heading_err = common::math::AngleDiff(
          local_view->GetVehicleState()->heading(), adc_pre_heading);
      break;
    }
    auto lane_id = adc_lane->lane().successor_id().at(i);
    auto lane = hdmap_local_->GetLaneById(lane_id);
    if (lane == nullptr) {
      continue;
    }
    lane_length += lane->overall_length();
    DealSuccessor(&lane, lane_length);
    successor_lane_is_virtual = lane != nullptr &&
                                lane->lane().left_boundary().virtual_() &&
                                lane->lane().right_boundary().virtual_();
    double adc_pre_heading =
        (lane != nullptr && lane->GetIsValidGeometry() &&
         lane->id().id() != adc_waypoint.lane->id().id())
            ? lane->Heading(0.1)
            : adc_waypoint.lane->Heading(adc_waypoint.s + 10.0);
    double err = common::math::AngleDiff(
        local_view->GetVehicleState()->heading(), adc_pre_heading);
    if (std::fabs(err) < std::fabs(heading_err) ||
        common::math::double_type::IsZero(heading_err)) {
      heading_err = err;
    }
  }
  // 切到跟车模式的条件 1.自车是实的，但是后继15m内有lane是虚的
  // 2. 自车是虚的，15m内未找到后继还是虚的
  local_view_data_->update_data()->set_is_adc_lane_virtual(adc_road_is_virtual);
  local_view_data_->update_data()->set_is_virtual_map2nolane_bl(
      successor_lane_is_virtual);
  local_view_data_->update_data()->set_adc_fusion_percep_heading_err(
      heading_err);
  local_view_data_->static_data()->set_tmp_1(true);
  ADEBUG << " adc_road_is_virtual: " << adc_road_is_virtual
         << " , successor_lane_is_virtual: " << successor_lane_is_virtual;
}

void MapFusionLaneLine::DealMaptype(
    const std::shared_ptr<LocalView>& local_view,
    functionmanager::FunctionManagerOut* to_fct) {
  if (!local_view->HasMapMsg()) {
    return;
  }
  const auto& map_msg = local_view->GetMapMsg();
  if (!(map_msg->hdmap().has_header() &&
        map_msg->hdmap().header().has_header() &&
        map_msg->hdmap().header().header().has_seq()) ||
      hdmap_local_ == nullptr ||
      !(hdmap_local_->GetMapHeader().has_header() &&
        hdmap_local_->GetMapHeader().header().has_seq())) {
    return;
  }
  auto map_type = map_msg->has_map_type()
                      ? map_msg->map_type()
                      : navigation_hdmap::MapMsg_MapType_PERCEP_MAP;
  map_types_.emplace_front(map_msg->hdmap().header().header().seq(), map_type);
  constexpr int kMaxMapTypeSize = 50;
  if (map_types_.size() > kMaxMapTypeSize) {
    map_types_.pop_back();
  }
  auto it = std::find_if(
      map_types_.begin(), map_types_.end(), [&](const MapTypePair& input) {
        return input.first == hdmap_local_->GetMapHeader().header().seq();
      });
  if (it != map_types_.end()) {
    now_fusion_maptype_ = it->second;
  }
  to_fct->set_localization_maptype(now_fusion_maptype_);
}

bool MapFusionLaneLine::ReturnStatus(
    functionmanager::FunctionManagerOut* to_fct, bool status,
    const std::string& msg) {
  if (to_fct == nullptr) {
    map_fusion_hdmap_debounce_.Reset();
    to_fct->set_map_fusion_hdmap_status(false);
    return false;
  }
  to_fct->mutable_real_hdmap_debug()
      ->mutable_location_err_debug()
      ->set_loc_err_reason(msg);
  per_fusion_map_status_ = map_fusion_hdmap_debounce_.DealDebounce(
      status && FLAGS_enable_mapfusion_mode);
  to_fct->set_map_fusion_hdmap_status(per_fusion_map_status_);
  to_fct->set_hdmap_status(per_fusion_map_status_ || to_fct->hdmap_status());
  auto soc_04_val = to_fct->soc_2_fct_tbd_u32_04();
  uint32_t map_fusion_status_out =
      per_fusion_map_status_ ? kMapfusionPilotSuccess : 0;
  to_fct->set_soc_2_fct_tbd_u32_04(soc_04_val | map_fusion_status_out);
  return status;
}

bool MapFusionLaneLine::ReturnLocalMapStatus(
    functionmanager::FunctionManagerOut* const to_fct, bool status) {  // NOLINT
  if (to_fct == nullptr) {
    map_fusion_hdmap_debounce_.Reset();
    to_fct->set_map_fusion_hdmap_status(false);
    return false;
  }
  per_fusion_map_status_ = map_fusion_hdmap_debounce_.DealDebounce(
      status && FLAGS_enable_mapfusion_mode);
  to_fct->set_map_fusion_hdmap_status(per_fusion_map_status_);
  to_fct->set_hdmap_status(per_fusion_map_status_ || to_fct->hdmap_status());

  bool perception_status = to_fct->perception_status();
  to_fct->set_perception_status(per_fusion_map_status_ || perception_status);
  auto soc_04_val = to_fct->soc_2_fct_tbd_u32_04();
  auto is_map_fusion_type =
      to_fct->hdmap_sub_state() == functionmanager::MAP_FUSION_TYPE;
  uint32_t map_fusion_status_out =
      (per_fusion_map_status_ && is_map_fusion_type) ? kMapfusionAdasSuccess
                                                     : 0;
  to_fct->set_soc_2_fct_tbd_u32_04(soc_04_val | map_fusion_status_out);
  return status;
}

}  // namespace planning
}  // namespace TL
