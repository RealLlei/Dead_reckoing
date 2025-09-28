/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#include "planning/localview/lane_line_builder/real_hdmap_lane_line/real_hdmap_lane_line.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/match.h"
#include "absl/strings/numbers.h"
#include "absl/strings/strip.h"
#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/file/log.h"
#include "common/time/clock.h"
#include "common/util/message_util.h"
#include "common/util/util.h"
#include "common/utm_projection/coordinate_convertor.h"
#include "common/utm_projection/utm_zone.h"
#include "map/hdmap/hdmap.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/util/util.h"

#include "proto/fsm/function_manager.pb.h"
#include "proto/map/map_road.pb.h"
#include "proto/routing/poi.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {

using functionmanager::MachineStateType;
using google::protobuf::uint32;
using PairChanglaneType = std::pair<double, routing::PerceptionChangeLaneType>;
using TL::common::Status;
using TL::common::VehicleState;  // NOLINT
using TL::functionmanager::NNPSysState;
using TL::hdmap::PassageAndVehilceRelation;

RealHDMapLaneLine::RealHDMapLaneLine() : local_hdmap_debounce_{1.0, 0.0, 0.1} {}

Status RealHDMapLaneLine::Init() {
  return Status::OK();
}

Status RealHDMapLaneLine::Init(
    const std::shared_ptr<LocalViewData>& local_view_data) {
  hd_map_ = hdmap::HDMapUtil::LoadEhpMapPtr();
  hd_map_ehp_ = std::make_shared<hdmap::HDMap>();
  if (!hd_map_) {
    return Status(common::ErrorCode::LOCALVIEW_EHR_DATALOSS_ERROR,
                  "hd_map_ is nullptr.");
  }
  pnc_map_ = std::make_shared<hdmap::PncMap>(hd_map_);
  // is_ehr_running_.store(false);
  landmark_index_ = 0;
  last_routing_response_ = std::make_shared<routing::RoutingResponse>();
  current_routing_response_ = std::make_shared<routing::RoutingResponse>();
  current_map_msg_ = std::make_shared<navigation_hdmap::MapMsg>();
  inside_active_conditon_ =
      std::make_unique<InsideActiveCondition>(local_view_data);
  local_view_data_ = local_view_data;
  failure_location_checker_ = std::make_unique<FailureLocationChecker>();
  failure_location_checker_->Init();

  ehr_ = std::make_unique<TL::ehr::AmapEhrImpl>();
  is_ehr_running_ = true;
  task_ehr_thread_ = std::thread(&RealHDMapLaneLine::GenerateEHRThread, this);

  shrinked_map_protos_.clear();
  routing::POI poi;
  if (TL::common::GetProtoFromASCIIFile(hdmap::EndWayPointFile(), &poi)) {
    for (auto& landmark : *poi.mutable_landmark()) {
      if (landmark.name() == "ehp_navigation") {
        for (auto& waypoint : *landmark.mutable_waypoint()) {
          auto x = waypoint.mutable_pose()->x();
          auto y = waypoint.mutable_pose()->y();
          auto z = waypoint.mutable_pose()->z();
          if (std::abs(x) < 180) {
            int zone = x / 6 + 31;  // NOLINT
            common::coordinate_convertor::GCS2UTM(zone, &x, &y);
            waypoint.mutable_pose()->set_x(x);
            waypoint.mutable_pose()->set_y(y);
            waypoint.mutable_pose()->set_z(z);
            auto heading = (90.0 - waypoint.heading()) / 180 * M_PI;
            if (heading < -M_PI) {
              waypoint.set_heading(heading + 2 * M_PI);
            } else {
              waypoint.set_heading(heading);
            }
          }
        }
        ehp_navigation_landmark_.CopyFrom(landmark);
        break;
      }
    }
    if (!ehp_navigation_landmark_.has_name()) {
      return Status(common::ErrorCode::LOCALVIEW_EHR_DATALOSS_ERROR,
                    "no ehp_navigaton landmark");
    }
  }
  return Status::OK();
}

Status RealHDMapLaneLine::Start() {
  return Status::OK();
}

void RealHDMapLaneLine::Stop() {
  is_ehr_running_ = false;
  if (task_ehr_thread_.joinable()) {
    task_ehr_thread_.join();
  }
}  // namespace TL

bool RealHDMapLaneLine::Process(
    const std::shared_ptr<LocalView>& local_view,
    functionmanager::FunctionManagerOut* const to_fct) {
  ADEBUG << "real hdmap Process!";
  constexpr double kWasteTime = 10.0;
  // 在ehp模式下切utm zone的操作
  const double start_time_first = TL::common::Clock::NowInMicroseconds();
#ifdef ISORIN
  return ReturnStatus(to_fct, false);
#endif
  bool is_run_ehp = false;

#ifdef ISMDC
  is_run_ehp = true;
#endif
  if (FLAGS_enable_planning_self_simulator) {
    is_run_ehp = true;
  }
  if (is_run_ehp) {
    new_utm_zone_ = local_view->GetLocalization()->pose().using_utm_zone();
    if (new_utm_zone_ != previous_utm_zone_) {
      AINFO << "waiting for loading new zone map " << new_utm_zone_;
      auto new_localization = std::make_shared<localization::Localization>(
          *local_view->GetLocalization());
      auto new_perception = std::make_shared<perception::PerceptionObstacles>(
          *local_view->GetPerceptionObstacles());
      if (common::utm_zone::SetLocalizationAndPerceptionByZoneID(
              previous_utm_zone_, new_localization, new_perception)) {
        local_view->SetLocalizationPtr(new_localization);
        local_view->SetPerceptionObstaclesPtr(new_perception);
        auto new_vehicle_state = std::make_shared<common::VehicleState>();
        new_vehicle_state->CopyFrom(*local_view->GetVehicleState());
        common::VehicleStateProvider::SetNewZoneVehicleState(
            new_vehicle_state, previous_utm_zone_);
        local_view->SetVehicleStatePtr(new_vehicle_state);
      } else {
        return ReturnStatus(to_fct, false);
      }
    } else {
      if (hd_map_ != hd_map_ehp_) {
        // 外部get返回current_routing_response_,last_routing_response_用于多线程间共享
        std::lock_guard<std::mutex> lock(routing_response_mutex_);
        if (current_routing_response_ != last_routing_response_) {
          hd_map_ = hd_map_ehp_;
          AINFO << "change to new zone";
        }
      }
    }
  }

  const double start_time_second = TL::common::Clock::NowInMicroseconds();
  if ((std::fabs(start_time_second - start_time_first) > kWasteTime) &&
      (to_fct != nullptr)) {
    to_fct->mutable_nnp_fct_out()
        ->mutable_localview_time()
        ->set_map_state_date_time_set((start_time_second - start_time_first));
  }
  {
    std::lock_guard<std::mutex> lock(ehr_data_mutex_);
    if (FLAGS_using_record_ehp_data) {
      if (!local_view->HasMapStateData()) {
        ADEBUG << "has no record_ehp_data!!!";
        return false;
      }
      map_state_data_.CopyFrom(*local_view->GetMapStateData());
    } else {
      if (!map_state_data_.has_header()) {
        common::util::FillHeader("map_state_data",
                                 map_state_data_.mutable_header());
      }
      auto state_data = std::make_shared<MapStateData>();
      state_data->Swap(&map_state_data_);
      local_view->SetMapStateDataPtr(state_data);
    }
  }

  if (hd_map_ == nullptr) {
    AERROR << "hd_map_ == nullptr!!!";
    return ReturnStatus(to_fct, false);
  }
  const double start_time_three = TL::common::Clock::NowInMicroseconds();

  if ((std::fabs(start_time_three - start_time_second) > kWasteTime) &&
      (to_fct != nullptr)) {
    to_fct->mutable_nnp_fct_out()
        ->mutable_localview_time()
        ->set_map_state_date_time_copy(start_time_three - start_time_second);
  }

  // 在数据回放模式下，读取地图和routing的操作
  if (FLAGS_is_record_replay) {
    if (local_view->HasMapMsg() &&
        local_view->GetMapMsg()->hdmap().header().date() !=
            hd_map_->GetMapHeader().date()) {
      if (local_view->GetMapMsg()->header().frame_id() == "from_ehp_hdmap" &&
          local_view->GetMapMsg()
                  ->hdmap()
                  .header()
                  .projection()
                  .utm_zone_id() ==
              hd_map_->GetMapHeader().projection().utm_zone_id()) {
        TL::hdmap::Map tmp_hdmap = local_view->GetMapMsg()->hdmap();
        // std::string unvalid_lane;
        // for (int i = (tmp_hdmap.lane_size() / 3); i < tmp_hdmap.lane_size();
        //      i++) {
        //   unvalid_lane += tmp_hdmap.lane().at(i).id().id();
        //   unvalid_lane += "_";
        //   tmp_hdmap.mutable_lane()->Mutable(i)->clear_central_curve();
        //   tmp_hdmap.mutable_lane()->Mutable(i)->clear_left_boundary();
        //   tmp_hdmap.mutable_lane()->Mutable(i)->clear_right_boundary();
        //   tmp_hdmap.mutable_lane()->Mutable(i)->clear_left_sample();
        //   tmp_hdmap.mutable_lane()->Mutable(i)->clear_right_sample();
        // }
        // AINFO << " unvalid_lane = " << unvalid_lane;
        hd_map_->LoadMapFromProto(tmp_hdmap);
      } else {
        hd_map_ = std::make_shared<hdmap::HDMap>();
        hd_map_->LoadMapFromProto(local_view->GetMapMsg()->hdmap());
      }
      previous_utm_zone_ = hd_map_->GetMapHeader().projection().utm_zone_id();
    }
#ifdef ISMDC
    if (previous_utm_zone_ !=
        local_view->GetLocalization()->pose().using_utm_zone()) {
      auto new_localization = std::make_shared<localization::Localization>(
          *local_view->GetLocalization());
      auto new_perception = std::make_shared<perception::PerceptionObstacles>(
          *local_view->GetPerceptionObstacles());
      if (common::utm_zone::SetLocalizationAndPerceptionByZoneID(
              previous_utm_zone_, new_localization, new_perception)) {
        local_view->SetLocalizationPtr(new_localization);
        local_view->SetPerceptionObstaclesPtr(new_perception);
        auto new_vehicle_state = std::make_shared<common::VehicleState>();
        new_vehicle_state->CopyFrom(*local_view->GetVehicleState());
        common::VehicleStateProvider::SetNewZoneVehicleState(
            new_vehicle_state, previous_utm_zone_);
        local_view->SetVehicleStatePtr(new_vehicle_state);
      } else {
        return ReturnStatus(to_fct, false);
      }
    }
#endif
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
  const auto nnp_sys_state =
      local_view->HasFunctionManagerIn()
          ? local_view->GetFunctionManagerIn()->fct_nnp_in().nnp_sysstate()
          : NNPSysState::NNPS_PASSIVE;
  const bool is_drive_auto = (nnp_sys_state == NNPSysState::NNPS_ACTIVE ||
                              nnp_sys_state == NNPSysState::NNPS_OVERRIDE ||
                              nnp_sys_state == NNPSysState::NNPS_LAT_OVERRIDE ||
                              nnp_sys_state == NNPSysState::NNPS_LON_OVERRIDE);

  if (to_fct->hdmap_sub_state() == functionmanager::EHP_HDMAP_TYPE ||
      is_drive_auto || (is_in_hdmap_ && !FLAGS_enable_hdmap_nnp_mode)) {
    is_in_hdmap_ = JudgeIsInMapContinuously(local_view, hd_map_);
  } else {
    is_in_hdmap_ = JudgeIsInMapFirstly(local_view, hd_map_);
  }
  to_fct->mutable_nnp_fct_out()->set_is_in_hdmap(is_in_hdmap_);
  {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    vehicle_state_ = local_view->GetVehicleState();
  }
  bool update_routing_response = UpdateRoutingResponse();

  auto* realhdmap_debug = to_fct->mutable_real_hdmap_debug();
  realhdmap_debug->set_real_is_in_hdmap(is_in_hdmap_);
  realhdmap_debug->set_update_routing_request(true);
  realhdmap_debug->set_update_routing_response(update_routing_response);
  if (!is_in_hdmap_) {
    failure_location_checker_->Reset();
    return ReturnStatus(to_fct, false);
  }
  auto* loc_err_debug = realhdmap_debug->mutable_location_err_debug();
  if (current_routing_response_ != nullptr &&
      current_routing_response_->has_header() &&
      current_routing_response_->has_routing_request() &&
      current_routing_response_->routing_request().waypoint_size() > 0) {
    const auto waypoint =
        current_routing_response_->routing_request().waypoint().begin();
    realhdmap_debug->set_request_end_id(waypoint->id());
    realhdmap_debug->mutable_routing_header()->CopyFrom(
        current_routing_response_->header());
    realhdmap_debug->mutable_request_end_pose()->CopyFrom(waypoint->pose());
  }
  std::string out_ehp_message;
  if (current_routing_response_ != nullptr) {
    if (current_routing_response_->has_ehp_reason()) {
      out_ehp_message += current_routing_response_->ehp_reason();
    }
  }
  const double start_time_four = TL::common::Clock::NowInMicroseconds();

  if ((std::fabs(start_time_four - start_time_three) > kWasteTime) &&
      (to_fct != nullptr)) {
    to_fct->mutable_nnp_fct_out()
        ->mutable_localview_time()
        ->set_map_state_date_time_lock(start_time_four - start_time_three);
  }

  // routing成功后再更新车辆状态
  if (update_routing_response) {
    pnc_map_->UpdateHDMap(hd_map_);
    pnc_map_->UpdateReferenceLineInfoConfig(
        MachineStateType::HDMAP_TYPE, functionmanager::AvpFctOut::CRUISING,
        std::fabs(local_view->GetVehicleState()->linear_velocity()),
        local_view->GetFunctionManagerIn()
            ->fct_nnp_in()
            .longitud_ctrl_cruise_speedms());
    if (hdmap::PncMap::IsNewRouting(last_routing_,
                                    *current_routing_response_)) {
      pnc_map_->UpdateRoutingResponse(*current_routing_response_);
      last_routing_.CopyFrom(*current_routing_response_);
    }
    pnc_map_->UpdateVehicleState(*local_view->GetVehicleState());

    pnc_map_->UpdateEhpNavgationLandmark(ehp_navigation_landmark_);
    std::string adc_path_id = pnc_map_->GetAdcRoadId();
    std::string target_path_id = pnc_map_->GetTargetRoadId();
    {
      std::lock_guard<std::mutex> lock(adc_target_path_id_mutex_);
      adc_path_id_ = adc_path_id;
      target_path_id_ = target_path_id;
    }

    const auto& adc_waypoint = pnc_map_->GetADCWaypoint();
    if (adc_waypoint.lane != nullptr) {
      to_fct->mutable_nnp_fct_out()->set_curr_lane_spd_km(
          static_cast<uint32>(adc_waypoint.lane->lane().speed_limit() * 3.6));
    }
    if (!SolveChangeLaneType(local_view)) {
      if (pnc_map_) {
        auto error_string = pnc_map_->GetPncMapErrorInfo();
        out_ehp_message = out_ehp_message + " ; " + error_string;
        loc_err_debug->set_loc_err_reason(out_ehp_message);
      }
      failure_location_checker_->Process(local_view, pnc_map_, hd_map_, to_fct);
      return ReturnStatus(to_fct, false);
    }
    routing::PerceptionChangeLaneTypes change_lane_types;
    to_fct->mutable_nnp_fct_out()->mutable_change_lane_types()->CopyFrom(
        FLAGS_using_hdmap_lnaechange_type ? change_lane_types_
                                          : change_lane_types);
    // 拿取地图里的隧道类型视觉下使用
    if (adc_waypoint.lane != nullptr) {
      const auto& lane_type =
          pnc_map_->GetFrontLaneRangeInfo(800, hdmap::LaneType::TUNNEL_LANE);
      const auto is_near_tunnel =
          lane_type.start_s > 0.1 && lane_type.start_s < 710;
      const bool is_tunnel_lane =
          is_near_tunnel || (lane_type.start_lane != nullptr &&
                             lane_type.start_lane->id().id() ==
                                 adc_waypoint.lane->lane().id().id());

      to_fct->mutable_nnp_fct_out()->set_perception_adc_is_in_tunnel(
          is_tunnel_lane);
      if (is_near_tunnel && lane_type.start_lane != nullptr) {
        to_fct->mutable_nnp_fct_out()->set_curr_lane_spd_km(static_cast<uint32>(
            lane_type.start_lane->lane().speed_limit() * 3.6));
      }
    }

    GetAdcToOtherLaneTypeLen(to_fct);
    failure_location_checker_->Process(local_view, pnc_map_, hd_map_, to_fct);
    inside_active_conditon_->Process(local_view, pnc_map_, to_fct, true);
    static std::string speed_limit = "DownVisual_";
    if (absl::StartsWith(to_fct->odd_info().next_info(), speed_limit)) {
      absl::string_view info_raw(to_fct->odd_info().next_info());
      absl::ConsumePrefix(&info_raw, speed_limit);
      uint32 speed(60);
      if (absl::SimpleAtoi(info_raw, &speed)) {
        to_fct->mutable_nnp_fct_out()->set_curr_lane_spd_km(speed);
      }
    }
    ADEBUG << "is_lot_err = "
           << to_fct->nnp_fct_out()
                  .nnp_statechange_conditions()
                  .location_err_state();
  }
  const double start_time_five = TL::common::Clock::NowInMicroseconds();

  if ((std::fabs(start_time_five - start_time_four) > kWasteTime) &&
      (to_fct != nullptr)) {
    to_fct->mutable_nnp_fct_out()
        ->mutable_localview_time()
        ->set_solve_changelane_time(start_time_five - start_time_four);
  }

  // 视觉下匝道转到地图
  auto dis_downramp_sg = to_fct->nnp_fct_out().nnp_d_distance2_downramp_sg();
  auto dis_outof_odd_sg = to_fct->nnp_fct_out().nnp_d_distance_outof_odd_sg();
  if (hdmap_state_) {
    if (!FLAGS_enable_hdmap_nnp_mode &&
        !(dis_outof_odd_sg > 1 && dis_outof_odd_sg < 500)) {  // NOLINT
      hdmap_state_ = false;
    }

  } else {
    if (dis_downramp_sg > 5 && dis_downramp_sg < 10 && dis_outof_odd_sg > 350 &&
        dis_outof_odd_sg < 500 && !FLAGS_enable_hdmap_nnp_mode) {
      hdmap_state_ = true;
    }
  }

  loc_err_debug->set_loc_err_reason(out_ehp_message);
  const auto hdmap_status =
      is_in_hdmap_ && update_routing_response && hdmap_state_;
  to_fct->set_ori_ehp_hdmap_status(is_in_hdmap_ && update_routing_response);
  ADEBUG << "real is_in_hdmap: " << is_in_hdmap_
         << ", update_routing_response: " << update_routing_response;
  const double start_time_six = TL::common::Clock::NowInMicroseconds();

  if ((std::fabs(start_time_six - start_time_five) > kWasteTime) &&
      (to_fct != nullptr)) {
    to_fct->mutable_nnp_fct_out()
        ->mutable_localview_time()
        ->set_solve_routing_time(start_time_six - start_time_five);
  }

  return ReturnStatus(to_fct, hdmap_status);
}

void RealHDMapLaneLine::GetAdcToOtherLaneTypeLen(
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

bool RealHDMapLaneLine::UpdateRoutingResponse() {
  {
    std::lock_guard<std::mutex> lock(routing_response_mutex_);
    if (last_routing_response_ == nullptr ||
        !last_routing_response_->has_header() || hd_map_->Empty()) {
      return false;
    }
    // 外部get返回current_routing_response_,last_routing_response_用于多线程间共享
    current_routing_response_ = last_routing_response_;
  }
  return true;
}

const std::shared_ptr<navigation_hdmap::MapMsg>& RealHDMapLaneLine::GetMapMsg(
    bool refresh) {
  const auto& hd_map_msg_header = hd_map_->GetMapHeader();
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
  common::util::FillHeader("from_ehp_hdmap", empty_map_msg_.get());
  return empty_map_msg_;
}

bool RealHDMapLaneLine::UpdateEHPData(
    const std::shared_ptr<TL::ehp::EHP>& ehp_message,
    int* const received_ehp_count) {
  std::lock_guard<std::mutex> lock(ehp_data_mutex_);
  ehp_data_list_.push_back(ehp_message);
  int counter = 0;
  if (ehp_message->ehp_data_size() > 0) {
    counter =
        ehp_message->ehp_data(ehp_message->ehp_data_size() - 1).send_counter();
  }
  *received_ehp_count = counter;
  return true;
}

void RealHDMapLaneLine::GenerateEHRThread() {
  pthread_setname_np(pthread_self(), "GenerateEHR");
  if (FLAGS_enable_planning_self_simulator) {
    // 睡眠5s，这样前5s可以工作在感知模式，可以验证localview的感知模式是否工作，是否可以平滑切换
    std::this_thread::sleep_for(std::chrono::milliseconds(0));
  }
  std::list<TL::hdmap::Map> extended_map_protos;
  std::list<TL::hdmap::Map> shrinked_map_protos;
  bool reset_map = false;
  while (is_ehr_running_) {
    static constexpr int32_t kSleepTime = 10;  // milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTime));
    auto routing_response = std::make_shared<TL::routing::RoutingResponse>();
    bool start_reset_map = false;
    if (!SetMapStateData()) {
      ADEBUG << "Set data failed!!!";
      continue;
    }
    int new_zone = new_utm_zone_;
    {
      std::lock_guard<std::mutex> lock(ehp_data_mutex_);
      if (ehp_data_list_.empty()) {
        continue;
      }
      LoadMapStateData();

      for (int i = 0; i < ehp_data_list_.size(); i++) {
        // for(const auto& ehp_data:ehp_data_list_[i]->ehp_data()){
        //   AERROR << "resave send_counter: " << ehp_data.send_counter();
        // }

        if (i < ehp_data_list_.size() - 1 &&
            ehp_data_list_[i]->ehp_data_size() > 0 &&
            ehp_data_list_[i + 1]->ehp_data_size() > 0 &&
            ehp_data_list_[i]->ehp_data().begin()->send_counter() ==
                ehp_data_list_[i + 1]->ehp_data().begin()->send_counter()) {
          continue;
        }
        TL::hdmap::Map shrinked_map_proto;
        TL::hdmap::Map extended_map_proto;
        int counter = 0;
        {
          std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
          ehr_->SetVehicleState(vehicle_state_);
        }
        ehr_->CollectData(*ehp_data_list_[i], &counter, &shrinked_map_proto,
                          &start_reset_map);
        if (start_reset_map) {
          reset_map = true;
        }
        std::string adc_path_id;
        std::string target_path_id;
        {
          std::lock_guard<std::mutex> lock(adc_target_path_id_mutex_);
          adc_path_id = adc_path_id_;
          target_path_id = target_path_id_;
        }
        ehr_->StartBuild(&extended_map_proto, routing_response.get(),
                         adc_path_id, target_path_id, new_zone);
        ADEBUG << "ehp routing current: " << adc_path_id
               << " ,target: " << target_path_id << " routing road size "
               << routing_response->road_size();
        if (start_reset_map) {
          extended_map_protos.clear();
          shrinked_map_protos.clear();
          shrinked_map_protos_.clear();
        }

        SetShrinkMap(shrinked_map_proto, &shrinked_map_protos);

        if (extended_map_proto.lane_size() != 0) {
          std::string all_lane_ids;
          for (const auto& one : extended_map_proto.lane()) {
            all_lane_ids += (" " + one.id().id());
          }
          extended_map_protos.emplace_back(std::move(extended_map_proto));
        }
      }
      ehp_data_list_.clear();
    }
    if (!routing_response->has_header()) {
      ADEBUG << "new routing response road size empty, skip";
      continue;
    }
    if (new_zone != previous_utm_zone_ || hd_map_ehp_ == nullptr) {
      hd_map_ehp_ = std::make_shared<hdmap::HDMap>();
      ADEBUG << "loading new zone " << new_zone;
    }
    if (reset_map) {
      hd_map_ehp_ = std::make_shared<hdmap::HDMap>();
      ADEBUG << "reset map shared ptr";
      reset_map = false;
    }
    hd_map_ehp_->UpdateMapFromProto(extended_map_protos, shrinked_map_protos);
    {
      std::lock_guard<std::mutex> lock(routing_response_mutex_);
      last_routing_response_ = routing_response;
    }
    if (new_zone != previous_utm_zone_) {
      AINFO << "loading new zone success";
      previous_utm_zone_ = new_zone;
    }
    extended_map_protos.clear();
    shrinked_map_protos.clear();
  }
}

void RealHDMapLaneLine::SetShrinkMap(
    const TL::hdmap::Map& shrinked_map,
    std::list<TL::hdmap::Map>* shrinked_map_protos) {
  constexpr int kSectionNumber = 10;
  if (shrinked_map.road_size() > 0) {
    for (const auto& road : shrinked_map.road()) {
      if (road.section_size() < kSectionNumber) {
        TL::hdmap::Map temp_map;
        temp_map.add_road()->CopyFrom(road);
        shrinked_map_protos_.emplace_back(temp_map);
      } else {
        for (int i = 0; i < road.section_size();) {
          TL::hdmap::Map temp_map;
          auto* temp_road = temp_map.add_road();
          temp_road->mutable_id()->set_id(road.id().id());
          for (int j = i; j < road.section_size(); ++j) {
            temp_road->add_section()->CopyFrom(road.section()[j]);
            if (j - i >= kSectionNumber) {
              shrinked_map_protos_.emplace_back(temp_map);
              i = j + 1;
              break;
            }
            if (j == road.section_size() - 1) {
              shrinked_map_protos_.emplace_back(temp_map);
              i = j + 1;
            }
          }
        }
      }
    }
  }
  if (!shrinked_map_protos_.empty()) {
    shrinked_map_protos->emplace_back(shrinked_map_protos_.front());
    shrinked_map_protos_.pop_front();
  }
}

void RealHDMapLaneLine::LogMapStateData() {
  if (FLAGS_using_record_ehp_data) {
    return;
  }
  std::lock_guard<std::mutex> lock(ehr_data_mutex_);
  ADEBUG << "----ehp size = " << map_state_data_.ehp_data_size();
  constexpr int max_ehp_size = 100;
  if (map_state_data_.ehp_data_size() > max_ehp_size) {
    map_state_data_.Clear();
  }
  if (ehr_ != nullptr && hd_map_ehp_ != nullptr) {
    map_state_data_processor_.LogEhpData(ehp_data_list_, &map_state_data_);
    map_state_data_processor_.LogEhrAndMap(ehr_.get(), *hd_map_ehp_,
                                           &map_state_data_);
  }
  // other
  {
    std::lock_guard<std::mutex> lock(routing_response_mutex_);
    map_state_data_.mutable_last_routing_response()->CopyFrom(
        *last_routing_response_);
  }
  map_state_data_.set_previous_utm_zone(previous_utm_zone_);
  map_state_data_.set_new_utm_zone(new_utm_zone_);
  map_state_data_.set_is_ehr_running(is_ehr_running_);
}

void RealHDMapLaneLine::LoadMapStateData() {
  // 暂时注释掉map state log记录
  return;
  if (FLAGS_using_record_ehp_data) {
    return;
  }
  ADEBUG << "----ehp size = " << map_state_data_.ehp_data_size();
  constexpr int max_ehp_size = 100;

  if (map_state_data_.ehp_data_size() > max_ehp_size) {
    map_state_data_.Clear();
  }
  bool accepted_ehr_data = false;
  MapStateData tmp_map_state_data;
  if (ehr_ != nullptr) {
    map_state_data_processor_.LoadEhrData(ehr_.get(), &tmp_map_state_data,
                                          &accepted_ehr_data);
  }

  if (accepted_ehr_data) {
    std::lock_guard<std::mutex> lock(ehr_data_mutex_);
    map_state_data_.Swap(&tmp_map_state_data);
  }
}

bool RealHDMapLaneLine::SetMapStateData() {
  if (FLAGS_using_record_ehp_data) {
    std::lock_guard<std::mutex> lock(ehr_data_mutex_);
    if (hd_map_ehp_ == nullptr) {
      hd_map_ehp_ = std::make_shared<hdmap::HDMap>();
    }
    {
      std::lock_guard<std::mutex> lock(ehp_data_mutex_);
      if (!map_state_data_processor_.SetMapStateData(
              map_state_data_, &ehp_data_list_, ehr_.get(), hd_map_ehp_)) {
        return false;
      }
    }
    if (map_state_data_.has_last_routing_response()) {
      std::lock_guard<std::mutex> lock(routing_response_mutex_);
      last_routing_response_ = std::make_shared<routing::RoutingResponse>(
          map_state_data_.last_routing_response());
    }
    if (map_state_data_.has_previous_utm_zone()) {
      previous_utm_zone_ = map_state_data_.previous_utm_zone();
      new_utm_zone_ = map_state_data_.new_utm_zone();
      is_ehr_running_ = map_state_data_.is_ehr_running();
    }
    map_state_data_.Clear();
    if (hd_map_ehp_ == nullptr) {
      AERROR << "hd_map_ehp_ NULLPTR ";
      return false;
    }
    ADEBUG << "SetMapStateData END!";
  }
  return true;
}

bool RealHDMapLaneLine::SolveChangeLaneType(
    const std::shared_ptr<LocalView>& local_view) {
  change_lane_types_.Clear();
  std::list<TL::hdmap::RouteSegments> segments;
  if (pnc_map_->GetRouteSegments(*local_view->GetVehicleState(), &segments,
                                 local_view->GetFunctionManagerIn()
                                     ->fct_nnp_in()
                                     .longitud_ctrl_cruise_speedms())) {
    routing::PerceptionChangeLaneType init{};
    PairChanglaneType left_lane{0.0, init};
    PairChanglaneType current_lane{0.0, init};
    PairChanglaneType right_lane{0.0, init};
    for (const auto& segment : segments) {
      if (segment.GetPassageAndVehicleRelation() ==
          PassageAndVehilceRelation::Current_Passage) {
        std::get<1>(current_lane)
            .set_change_lane_type(segment.PerceptionNextAction());
        std::get<1>(current_lane).set_id("0_current");
        std::get<0>(current_lane) = segment.GetRouteSegmentLength();
      } else if (segment.GetPassageAndVehicleRelation() ==
                 PassageAndVehilceRelation::Left_Passage) {
        std::get<1>(left_lane).set_change_lane_type(
            segment.PerceptionNextAction());
        std::get<1>(left_lane).set_id("11_left");
        std::get<0>(left_lane) = segment.GetRouteSegmentLength();
      } else if (segment.GetPassageAndVehicleRelation() ==
                 PassageAndVehilceRelation::Right_Passage) {
        std::get<1>(right_lane)
            .set_change_lane_type(segment.PerceptionNextAction());
        std::get<1>(right_lane).set_id("12_right");
        std::get<0>(right_lane) = segment.GetRouteSegmentLength();
      }
    }
    // 如果左换道方向冲突需要根据长度修改
    if (std::get<1>(current_lane).change_lane_type() ==
            routing::ChangeLaneType::LEFT &&
        std::get<1>(left_lane).change_lane_type() ==
            routing::ChangeLaneType::RIGHT) {
      if (std::get<0>(current_lane) - std::get<0>(left_lane) > 10.0) {
        std::get<1>(current_lane)
            .set_change_lane_type(routing::ChangeLaneType::FORWARD);
      } else if (std::get<0>(left_lane) - std::get<0>(current_lane) > 10.0) {
        std::get<1>(left_lane).set_change_lane_type(
            routing::ChangeLaneType::FORWARD);
      }
    }
    // 如果右换道方向冲突需要根据长度修改
    if (std::get<1>(current_lane).change_lane_type() ==
            routing::ChangeLaneType::RIGHT &&
        std::get<1>(right_lane).change_lane_type() ==
            routing::ChangeLaneType::LEFT) {
      if (std::get<0>(current_lane) - std::get<0>(right_lane) > 10.0) {
        std::get<1>(current_lane)
            .set_change_lane_type(routing::ChangeLaneType::FORWARD);
      } else if (std::get<0>(right_lane) - std::get<0>(current_lane) > 10.0) {
        std::get<1>(right_lane)
            .set_change_lane_type(routing::ChangeLaneType::FORWARD);
      }
    }
    change_lane_types_.add_per_change_lane_type()->CopyFrom(
        std::get<1>(current_lane));
    change_lane_types_.add_per_change_lane_type()->CopyFrom(
        std::get<1>(right_lane));
    change_lane_types_.add_per_change_lane_type()->CopyFrom(
        std::get<1>(left_lane));
    pnc_map_->ProcessFollowUpOdd();
    pnc_map_->ProcessDistanceFromADCToOddStartEnd();
    return true;
  }
  ADEBUG << " pnc map is fail ";
  return false;
}

bool RealHDMapLaneLine::ReturnStatus(
    functionmanager::FunctionManagerOut* const to_fct, bool status) {
  if (to_fct == nullptr) {
    local_hdmap_debounce_.Reset();
    return false;
  }
  auto hdmap_status = local_hdmap_debounce_.DealDebounce(status);
  to_fct->set_ehp_hdmap_status(hdmap_status);
  return hdmap_status;
}

}  // namespace planning
}  // namespace TL
