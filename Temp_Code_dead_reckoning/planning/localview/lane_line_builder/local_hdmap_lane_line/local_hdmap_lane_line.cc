/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#include "planning/localview/lane_line_builder/local_hdmap_lane_line/local_hdmap_lane_line.h"

#include <string>
#include <vector>

#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/status/status.h"
#include "common/util/message_util.h"
#include "common/utm_projection/utm_zone.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "map/hdmap/hdmap_util.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/routing/poi.pb.h"

namespace TL {
namespace planning {

using google::protobuf::uint32;
using TL::common::Status;
using TL::common::VehicleState;  // NOLINT

LocalHDMapLaneLine::LocalHDMapLaneLine()
    : local_hdmap_debounce_{1.0, 0.0, 0.1} {}

Status LocalHDMapLaneLine::Init() {
  return Status::OK();
}

Status LocalHDMapLaneLine::Init(
    const std::shared_ptr<LocalViewData>& local_view_data) {
#ifdef FOR_BAIDU_SIMULATION
  local_hdmap_debounce_.ResetTime(0.0, 0.0, 0.1);
#endif
  local_view_data_ = local_view_data;
  local_hd_map_ = hdmap::HDMapUtil::LoadLocalMapPtr();
  if (local_hd_map_ == nullptr && FLAGS_is_record_replay) {
    local_hd_map_ = CreateMap(std::make_shared<hdmap::Map>());
  }
  if (local_hd_map_) {
    landmark_index_ = 0;
    pnc_map_ = std::make_shared<hdmap::PncMap>(local_hd_map_);
    last_routing_request_ = std::make_shared<routing::RoutingRequest>();
    current_routing_request_ = std::make_shared<routing::RoutingRequest>();
    last_routing_response_ = std::make_shared<routing::RoutingResponse>();
    current_routing_response_ = std::make_shared<routing::RoutingResponse>();
    current_map_msg_ = std::make_shared<navigation_hdmap::MapMsg>();
    inside_active_conditon_ =
        std::make_unique<InsideActiveCondition>(local_view_data);

  } else {
    AERROR << " No mapfile can not init routing and init faild!";
    is_init_success_ = false;
    return Status(common::ErrorCode::LOCALVIEW_EHR_DATALOSS_ERROR,
                  "No mapfile can not init routing.");
  }
  if (!FLAGS_is_record_replay) {
    hdmap::Map hd_map;
    local_hd_map_->GetMap(&hd_map);
    routing_.Init(hd_map);
    routing_.Start();
    is_routing_running_ = true;
    task_routing_thread_ =
        std::thread(&LocalHDMapLaneLine::GenerateRoutingThread, this);
  }
  return Status::OK();
}

Status LocalHDMapLaneLine::Start() {
  return Status::OK();
}

void LocalHDMapLaneLine::Stop() {
  is_routing_running_ = false;
  routing_success_.store(false);
  if (!FLAGS_is_record_replay && task_routing_thread_.joinable()) {
    task_routing_thread_.join();
  }
}

bool LocalHDMapLaneLine::Process(
    const std::shared_ptr<LocalView>& local_view,
    functionmanager::FunctionManagerOut* const to_fct) {
  ADEBUG << "local hdmap Process!";
  // 初始化失败后此模式一直失败
  if (!is_init_success_) {
    AERROR << " No mapfile can not init local hdmap lane line!!! ";
    return ReturnStatus(to_fct, false);
  }

  functionmanager::HdmapSubState state_type = to_fct->hdmap_sub_state();
  if (to_fct->hdmap_sub_state() == functionmanager::LOCAL_HDMAP_TYPE) {
    is_in_hdmap_ = JudgeIsInMapContinuously(local_view, local_hd_map_);
  } else {
    is_in_hdmap_ = JudgeIsInMapFirstly(local_view, local_hd_map_);
  }
  to_fct->mutable_nnp_fct_out()->set_is_in_hdmap(is_in_hdmap_);
  {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    vehicle_state_ = local_view->GetVehicleState();
  }

  // return UpdateMap(local_view);
  bool update_routing_request = false;
  if (is_in_hdmap_) {
    if (FLAGS_is_record_replay) {
      update_routing_request = true;
    } else {
      update_routing_request = UpdateRoutingRequest(local_view, state_type);
    }
  } else {
    auto* localhdmap_debug = to_fct->mutable_local_hdmap_debug();
    localhdmap_debug->set_real_is_in_hdmap(is_in_hdmap_);
    localhdmap_debug->set_update_routing_request(update_routing_request);
    return ReturnStatus(to_fct, false);
  }

  // 在数据回放模式下，读取地图和routing的操作
  if (FLAGS_is_record_replay) {
    std::lock_guard<std::mutex> lock(routing_response_mutex_);
    if (local_view->HasValidRoutingResponseHeader() &&
        hdmap::PncMap::IsNewRouting(*last_routing_response_,
                                    *local_view->GetRoutingResponse())) {
      last_routing_response_ = std::make_shared<routing::RoutingResponse>(
          *local_view->GetRoutingResponse());
    }
    if (last_routing_response_->routing_request().waypoint().empty()) {
      return ReturnStatus(to_fct, false);
    }
  }

  bool update_routing_response = UpdateRoutingResponse();

  if (FLAGS_enable_from_file_response) {
    // if ((current_routing_response_ != nullptr) &&
    //     (!current_routing_response_->road().empty()) &&
    //     (from_file_routing_response_ == nullptr)) {
    //   if (common::SetProtoToASCIIFile((*current_routing_response_),
    //                                   absl::StrCat("/opt/usr/col/planning", "/",
    //                                                "yanshi_respnse.txt"))) {
    //     ADEBUG << " demo_out response to txt";
    //   }
    // }
    // if (from_file_routing_response_ == nullptr) {
    //   routing::RoutingResponse tmp_response;
    //   if (common::GetProtoFromASCIIFile(
    //           absl::StrCat("/opt/usr/col/planning", "/", "yanshi_respnse.txt"),
    //           &tmp_response)) {
    //     if (!tmp_response.road().empty()) {
    //       from_file_routing_response_ =
    //           std::make_shared<routing::RoutingResponse>(tmp_response);
    //       AINFO << " mdc demo_get response from txt";
    //     }
    //   }
    // }

    ////// from file get response
    // if ((current_routing_response_ != nullptr) &&
    //     (!current_routing_response_->road().empty()) &&
    //     (from_file_routing_response_ == nullptr)) {
    //   if (common::SetProtoToASCIIFile(
    //           (*current_routing_response_),
    //           absl::StrCat(FLAGS_map_dir, "/", "yanshi_respnse.txt"))) {
    //     ADEBUG << " demo_out response to txt";
    //   }
    // }

    if (from_file_routing_response_ == nullptr) {
      routing::RoutingResponse tmp_response;
      if (common::GetProtoFromASCIIFile(
              absl::StrCat("FLAGS_map_dir", "/",
                           "default_routing_response.txt"),
              &tmp_response)) {
        if (!tmp_response.road().empty()) {
          from_file_routing_response_ =
              std::make_shared<routing::RoutingResponse>(tmp_response);
          ADEBUG << " demo_get response from txt";
        }
      }
    }

    if (from_file_routing_response_ != nullptr) {
      update_routing_response = true;
      ADEBUG << " current_routing_response road_size = "
             << current_routing_response_->road_size()
             << " from_file_routing_response road_size = "
             << from_file_routing_response_->road_size();
      current_routing_response_ = from_file_routing_response_;
    }
  }

  auto* localhdmap_debug = to_fct->mutable_local_hdmap_debug();
  localhdmap_debug->set_real_is_in_hdmap(is_in_hdmap_);
  localhdmap_debug->set_update_routing_request(update_routing_request);
  localhdmap_debug->set_update_routing_response(update_routing_response);
  bool valid_of_lane_routing = false;
  // routing成功后再更新车辆状态
  if (update_routing_response) {
    pnc_map_->UpdateHDMap(local_hd_map_);
    pnc_map_->UpdateReferenceLineInfoConfig(
        functionmanager::MachineStateType::HDMAP_TYPE,
        functionmanager::AvpFctOut::CRUISING,
        std::fabs(local_view->GetVehicleState()->linear_velocity()),
        local_view->GetFunctionManagerIn()
            ->fct_nnp_in()
            .longitud_ctrl_cruise_speedms());
    if (is_new_routing_responce_) {
      pnc_map_->UpdateRoutingResponse(*current_routing_response_);
    }
    pnc_map_->UpdateVehicleState(*local_view->GetVehicleState());

    if (!SolveChangeLaneType(local_view)) {
      return ReturnStatus(to_fct, false);
    }
    const auto& adc_waypoint = pnc_map_->GetADCWaypoint();
    if (adc_waypoint.lane != nullptr) {
      // 未在NNP地图中才使用NCP下的地图限速
      if (!to_fct->real_hdmap_debug().real_is_in_hdmap()) {
        to_fct->mutable_nnp_fct_out()->set_curr_lane_spd_km(
            static_cast<uint32>(adc_waypoint.lane->lane().speed_limit() * 3.6));
      }
      localhdmap_debug->set_adc_lane_id(adc_waypoint.lane->lane().id().id());
    }

    inside_active_conditon_->Process(local_view, pnc_map_, to_fct);
    valid_of_lane_routing = to_fct->nnp_fct_out()
                                .nnp_statechange_conditions()
                                .valid_of_lane_routing();
  }
  auto hdmap_status = is_in_hdmap_ && update_routing_request &&
                      update_routing_response && valid_of_lane_routing &&
                      FLAGS_enable_hdmap_ncp_mode;
  ADEBUG << "real is_in_hdmap: " << is_in_hdmap_
         << ", update_routing_request: " << update_routing_request
         << ", update_routing_response: " << update_routing_response
         << " valid_of_lane_routing = " << valid_of_lane_routing
         << " hdmap_status = " << hdmap_status;
  if (to_fct->real_hdmap_debug().real_is_in_hdmap()) {
    hdmap_status = false;
  }
  return ReturnStatus(to_fct, hdmap_status);
}

bool LocalHDMapLaneLine::UpdateRoutingRequest(
    const std::shared_ptr<LocalView>& local_view,
    const functionmanager::HdmapSubState& state_type) {
  need_first_routing_ =
      (need_first_routing_ || functionmanager::LOCAL_HDMAP_TYPE != state_type);

  ADEBUG << " need_first_routing_: " << need_first_routing_
         << " , state_type: " << state_type;
  std::lock_guard<std::mutex> lock(routing_request_mutex_);
  if (functionmanager::LOCAL_HDMAP_TYPE == state_type) {
    need_first_routing_ = false;
    landmark_index_ = 0;
  }
  if (need_first_routing_) {
    // routing from file at first time
    // common::util::FillHeader("from_file_needfirstrouting",
    // routing_response);
    return BuildFirstRouting(local_view);
  }
  // 更新其他来源的routing request
  if (!local_view->HasRoutingRequest()) {
    return true;
  }
  if ((local_view->HasRoutingRequest()) &&
      (local_view->GetRoutingRequest()->has_header() &&
       (!local_view->GetRoutingRequest()->waypoint().empty()))) {
    if (LaneLineBase::IsNewRoutingRequest(*local_view->GetRoutingRequest(),
                                          *current_routing_request_)) {
      current_routing_request_->Clear();
      for (const auto& single_point :
           local_view->GetRoutingRequest()->waypoint()) {
        current_routing_request_->add_waypoint()->CopyFrom(single_point);
      }
      if (!local_view->GetRoutingRequest()->has_header()) {
        common::util::FillHeader("dreamview_filtered_routing",
                                 current_routing_request_.get());
      } else {
        std::string new_header;
        new_header = local_view->GetRoutingRequest()->header().frame_id();
        common::util::FillHeader(new_header, current_routing_request_.get());
      }

      local_view->SetRoutingRequestPtr(current_routing_request_);
      ADEBUG << " dreamview_filtered_routing_Infor =  "
             << current_routing_request_->DebugString();
      return true;
    }
  }
  ADEBUG << " dreamview_filtered_routing_Infor is same";
  return true;
}

bool LocalHDMapLaneLine::UpdateRoutingResponse() {
#ifdef FOR_BAIDU_SIMULATION
  if ((!FLAGS_enable_planning_self_simulator) && (!FLAGS_is_record_replay)) {
    while (!routing_success_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
#endif
  {
    std::lock_guard<std::mutex> lock(routing_response_mutex_);
    if (last_routing_response_ == nullptr ||
        !last_routing_response_->has_header() || local_hd_map_->Empty()) {
      return false;
    }
    // 外部get返回current_routing_response_,last_routing_response_用于多线程间共享
    is_new_routing_responce_ = hdmap::PncMap::IsNewRouting(
        *last_routing_response_, *current_routing_response_);
    if (is_new_routing_responce_) {
      current_routing_response_ = last_routing_response_;
    }
    return true;
  }
}

bool LocalHDMapLaneLine::BuildFirstRouting(
    const std::shared_ptr<LocalView>& local_view) {
  routing::POI poi;
  current_routing_request_->Clear();
  if (!common::GetProtoFromASCIIFile(hdmap::EndWayPointFile(), &poi)) {
    return false;
  }
  if (!local_view->HasLocalization()) {
    return false;
  }

  if (local_view->GetLocalization()->has_header() &&
      local_view->GetLocalization()->header().frame_id() != "SelfSimulator") {
    auto* start_point = current_routing_request_->add_waypoint();
    start_point->mutable_pose()->set_x(
        local_view->GetLocalization()->pose().position().x());
    start_point->mutable_pose()->set_y(
        local_view->GetLocalization()->pose().position().y());
  }

  if (poi.landmark_size() > 0) {
    if (landmark_index_ >= poi.landmark_size()) {
      landmark_index_ = 0;
    }
    for (size_t j = 0; j < poi.landmark(landmark_index_).waypoint_size(); j++) {
      current_routing_request_->add_waypoint()->CopyFrom(
          poi.landmark(landmark_index_).waypoint(j));  // NOLINT
    }
  }

  common::util::FillHeader("from_file_need_first_routing_",
                           current_routing_request_.get());
  local_view->SetRoutingRequestPtr(current_routing_request_);
  return true;
}

const std::shared_ptr<navigation_hdmap::MapMsg>& LocalHDMapLaneLine::GetMapMsg(
    bool refresh) {
  const auto& hd_map_msg_header = local_hd_map_->GetMapHeader();
  if (refresh || updated_map_time_.empty() ||
      (hd_map_msg_header.has_date() &&
       updated_map_time_ != hd_map_msg_header.date())) {
    ADEBUG << "update hdmap header " << hd_map_msg_header.ShortDebugString()
           << " " << (updated_map_time_ != hd_map_msg_header.date());
    updated_map_time_ = hd_map_msg_header.date();
    current_map_msg_ = std::make_shared<navigation_hdmap::MapMsg>();
    current_map_msg_->mutable_hdmap()->mutable_header()->CopyFrom(
        hd_map_msg_header);
    return current_map_msg_;
  }

  empty_map_msg_ = std::make_shared<navigation_hdmap::MapMsg>();
  common::util::FillHeader("from_file_hdmap", empty_map_msg_.get());
  return empty_map_msg_;
}

bool LocalHDMapLaneLine::SolveChangeLaneType(
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
  ADEBUG << " pnc map is fail ";
  return false;
}

void LocalHDMapLaneLine::GenerateRoutingThread() {
  pthread_setname_np(pthread_self(), "GenerRouting");
  common::sub_thread_name = "_routing";
  if (FLAGS_enable_planning_self_simulator) {
    // 睡眠5s，这样前5s可以工作在感知模式，可以验证localview的感知模式是否工作，是否可以平滑切换
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }
  while (is_routing_running_) {
    static constexpr int32_t kSleepTime = 100;  // milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTime));
    if (!is_in_hdmap_) {
      ADEBUG << "current position is no valid";
      continue;
    }
    routing::RoutingRequest tmp_routing_request;
    {
      std::lock_guard<std::mutex> lock(routing_request_mutex_);
      if (!current_routing_request_ ||
          !current_routing_request_->has_header()) {
        continue;
      }
      if (!IsNewRoutingRequest(*current_routing_request_,
                               *last_routing_request_)) {
        continue;
      }
      ADEBUG << "current_routing_request_: "
             << current_routing_request_->DebugString();
      tmp_routing_request = *current_routing_request_;
    }
    need_first_routing_ = false;
    auto new_routing_response = std::make_shared<routing::RoutingResponse>();
    bool status = routing_.Process(
        std::make_shared<routing::RoutingRequest>(tmp_routing_request),
        new_routing_response.get());
    AINFO << "rerouting has already completed,and status is " << status;
    need_first_routing_ = !status;  // still need first routing when failed
    if (!status) {
      ADEBUG << "rerouting err status = " << status;
      {
        std::lock_guard<std::mutex> lock(routing_request_mutex_);
        landmark_index_ += 1;
      }
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(routing_request_mutex_);
      last_routing_request_->CopyFrom(*current_routing_request_);
    }
    {
      std::lock_guard<std::mutex> lock(routing_response_mutex_);
      last_routing_response_ = new_routing_response;
      routing_success_.store(true);
    }
  }
}

bool LocalHDMapLaneLine::ReturnStatus(
    functionmanager::FunctionManagerOut* const to_fct, bool status) {
  if (to_fct == nullptr) {
    local_hdmap_debounce_.Reset();
    return false;
  }
  auto hdmap_status = local_hdmap_debounce_.DealDebounce(status);
  to_fct->set_local_hdmap_status(hdmap_status);
  to_fct->set_hdmap_status(hdmap_status || to_fct->ehp_hdmap_status());
  return hdmap_status;
}

}  // namespace planning
}  // namespace TL
