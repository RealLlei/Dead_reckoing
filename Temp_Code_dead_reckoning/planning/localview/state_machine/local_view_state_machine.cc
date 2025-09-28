/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @LastEditors: Liu Bei
 *****************************************************************************/
#include "planning/localview/state_machine/local_view_state_machine.h"

#include <cstdint>
#include <list>
#include <memory>

#include "common/configs/config_gflags.h"
#include "common/time/clock.h"
#include "common/utm_projection/utm_zone.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "google/protobuf/stubs/logging.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/open_space_info.h"
#include "planning/common/util/util.h"
#include "planning/localview/lane_line_builder/local_hdmap_lane_line/local_hdmap_lane_line.h"

#include "common/file/log.h"
#include "proto/common/error_code.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
using AVPMapState = hdmap::AVPSlamMap::AVPMapState;
using TL::common::Clock;
using TL::soc::Chassis;

bool LocalViewStateFront::Init() {
  local_view_data_ = std::make_shared<LocalViewData>();
  perception_lane_line_ = std::make_unique<NavigationHdmap>();
  real_hdmap_lane_line_ = std::make_unique<RealHDMapLaneLine>();
#ifndef ISORIN
  local_hdmap_lane_line_ = std::make_unique<LocalHDMapLaneLine>();
#endif
  map_fusion_lane_line_ = std::make_unique<MapFusionLaneLine>();
  adaptive_cruise_ = std::make_unique<AdaptiveCruise>();
  ldw_ldp_core_ = std::make_unique<LdpLdwCore>();
  missile_mode_ = std::make_unique<missilelane::MissileMode>();
  perception_lane_line_->Init();
  perception_lane_line_->Start();
#ifndef ISORIN
  local_hdmap_lane_line_->Init(local_view_data_);
  local_hdmap_lane_line_->Start();
#endif
  real_hdmap_lane_line_->Init(local_view_data_);
  real_hdmap_lane_line_->Start();
  map_fusion_lane_line_->Init(local_view_data_);
  map_fusion_lane_line_->Start();
  missile_mode_->Init(local_view_data_);
  adaptive_cruise_->Init(perception_lane_line_->GetConfig(), local_view_data_);
  adaptive_cruise_->Start();
  ldw_ldp_core_->Init(perception_lane_line_->GetConfig());
  ldw_ldp_core_->Start();
  nnp_fct_out_ = std::make_shared<functionmanager::FunctionManagerOut>();
  return true;
}

bool LocalViewStateFront::Stop() {
  perception_lane_line_->Stop();
#ifndef ISORIN
  local_hdmap_lane_line_->Stop();
#endif
  real_hdmap_lane_line_->Stop();
  map_fusion_lane_line_->Stop();
  adaptive_cruise_->Stop();
  return true;
}

void LocalViewStateFront::SetState(const Status& status) {
  mode_status_ = status;
}

void LocalViewStateFront::SetLocalView(
    const std::shared_ptr<LocalView>& local_view) {
  local_view_ = local_view;
}

void LocalViewStateFront::SetMachineStateType(
    const functionmanager::MachineStateType& state_type) {
  cur_state_machine_ = state_type;
}

void LocalViewStateFront::SetHistoryMachineStateType(
    const functionmanager::MachineStateType& state_type) {
  history_state_machine_ = state_type;
}

const common::Status& LocalViewStateFront::GetState() const {
  return mode_status_;
}

const LocalView& LocalViewStateFront::GetLocalView() const {
  ACHECK(local_view_ != nullptr);
  return *local_view_;
}

const std::shared_ptr<LocalView>& LocalViewStateFront::GetMutableLocalView() {
  return local_view_;
}

const std::shared_ptr<LocalViewData>&
LocalViewStateFront::GetMutableLocalViewData() {
  return local_view_data_;
}

const functionmanager::MachineStateType&
LocalViewStateFront::GetMachineStateType() const {
  return cur_state_machine_;
}

const functionmanager::MachineStateType&
LocalViewStateFront::GetHistoryMachineStateType() const {
  return history_state_machine_;
}

common::Status LocalViewStateFront::BuildLocalView(
    const std::shared_ptr<LocalView>& local_view) {
  auto to_fct = std::make_shared<functionmanager::FunctionManagerOut>();
  VehicleFunctionStateUpdata(local_view);
  double time_start = Clock::NowInMicroseconds();
  SetFctAndLaneLineDebug(to_fct.get(), local_view);
  // 先计算纯地图模式和感知模式，得到他们的mapmsg和routing response
  bool is_generate_map_from_perception = false;
  bool is_generate_map_from_adaptive = false;
  const auto& vehicle_state_raw = local_view->GetVehicleState();
  // auto vehicle_state_raw =
  //     std::make_shared<common::VehicleState>();
  auto vehicle_state_ptr = std::make_shared<common::VehicleState>();
  vehicle_state_ptr->CopyFrom(*vehicle_state_raw);
  auto vehicle_state_vrf = std::make_shared<common::VehicleState>();
  vehicle_state_vrf->CopyFrom(*vehicle_state_raw);
  common::VehicleStateProvider::SetVehicleReferenceFrame(
      vehicle_state_vrf.get());
  if (local_view->GetFunctionManagerIn()->ta_pilot_mode() !=
      functionmanager::AVP) {
    auto* localview_time =
        to_fct->mutable_nnp_fct_out()->mutable_localview_time();
    const auto start_times_nnp = Clock::NowInMicroseconds();
    real_hdmap_lane_line_->Process(local_view, to_fct.get());
    nnp_fct_out_->CopyFrom(*to_fct);
    const auto start_times_per = Clock::NowInMicroseconds();
    localview_time->set_real_hdmap_time(start_times_per - start_times_nnp);
#ifndef ISORIN
    local_hdmap_lane_line_->Process(local_view, to_fct.get());
#endif
    adaptive_cruise_->KappaPreprocessing(local_view);
    bool map_s = map_fusion_lane_line_->Process(local_view, to_fct.get());
    ADEBUG << "map_s:" << map_s;
    local_view->SetVehicleStatePtr(vehicle_state_vrf);
    is_generate_map_from_perception =
        perception_lane_line_->Process(local_view, to_fct.get());
    const auto start_times_adp = Clock::NowInMicroseconds();
    localview_time->set_perception_map_time(start_times_adp - start_times_per);
    is_generate_map_from_adaptive =
        adaptive_cruise_->Process(local_view, to_fct.get());
    localview_time->set_adaptive_map_time(Clock::NowInMicroseconds() -
                                          start_times_adp);
    ldw_ldp_core_->SetLanemarkerDebug(
        perception_lane_line_->GetLanemarkerDebugPtr());
    ldw_ldp_core_->Process(local_view, to_fct.get());

    local_view->SetLanemarkersLaneLinePtr(
        perception_lane_line_->GetLanemarkerDebugPtr());
    missile_mode_->Process(local_view, to_fct.get());
    local_view->SetVehicleStatePtr(vehicle_state_ptr);
  }
  // 将视觉地图的mapmsg给localview，融合模式会用到
  // 将地图模式的HDMap给localveiw，状态机各种条件判断会需要
  local_view->SetHDMapPtr(real_hdmap_lane_line_->GetHDMapPtr());
  if (!FLAGS_is_record_replay && !to_fct->map_fusion_hdmap_status() &&
      !local_view->HasMapMsg()) {
    local_view->SetMapMsgPtr(perception_lane_line_->GetMapMsg(true));
  }
  // set control enbale
  local_view->SetFunctionManagerOutPtr(to_fct);
  to_fct->mutable_nnp_fct_out()->mutable_localview_time()->set_real_hdmap_time(
      Clock::NowInMicroseconds() - time_start);
  time_start = Clock::NowInMicroseconds();
  to_fct->mutable_avp_fct_out()->set_control_enable(false);

  ADEBUG << ", is_generate_map_from_perception = "
         << is_generate_map_from_perception
         << ",is_generate_map_from_adaptive = "
         << is_generate_map_from_adaptive;
  BuildLocalViewEvent localview_event;
  // NOLINTBEGIN
  boost::msm::back::state_machine<LocalViewStateFront>& state_machine =
      static_cast<boost::msm::back::state_machine<LocalViewStateFront>&>(*this);
  // NOLINTEND

  // 先给localview赋perception mode的routing，用于融合模式
  state_machine.SetLocalView(local_view);
  state_machine.SetState(
      Status(ErrorCode::LOCALVIEW_FSM_CONSTRUCT_ERROR, "default status"));
  ADEBUG << "local_state:" << cur_state_machine_ << "  "
         << cur_hdmap_sub_state_;
  // 运行状态机
  state_machine.process_event(localview_event);
  to_fct->CopyFrom(*local_view->GetFunctionManagerOut());
  // apa light up
  APALightUpNTP(local_view, to_fct);
  // after state_machine process, need set cur_perception_sub_state
  to_fct->set_soc_2_fct_tbd_u32_01(cur_perception_sub_state_);
  // NCP下LCC和ACC模式条件都满足
  if (cur_hdmap_sub_state_ == functionmanager::LOCAL_HDMAP_TYPE) {
    to_fct->set_soc_2_fct_tbd_u32_04(0x13);
  }
  DealMapFusionAction(to_fct.get(), local_view);
  to_fct->mutable_nnp_fct_out()
      ->mutable_localview_time()
      ->set_perception_map_time(Clock::NowInMicroseconds() - time_start);
  time_start = Clock::NowInMicroseconds();
  ADEBUG << "cur_state:" << cur_state_machine_
         << " , cur_perception_sub_state: " << cur_perception_sub_state_
         << " , cur_hdmap_sub_state: " << cur_hdmap_sub_state_;
  Status status = state_machine.GetState();
  if (local_view->GetFunctionManagerIn()->ta_pilot_mode() ==
      functionmanager::AVP) {
    to_fct->set_avp_status(status.ok());
    to_fct->mutable_avp_fct_out()->set_state_type(
        local_view->GetFunctionManagerIn()->fct_avp_in().sys_mode());
  }
  if (cur_state_machine_ != history_state_machine_) {
    fsm_sequence_num_++;
    previous_state_machine_ = history_state_machine_;
  }
  ADEBUG << "cur_state_machine_: " << cur_state_machine_
         << ", history_state_machine_: " << history_state_machine_
         << ", fsm_sequence_num_: " << fsm_sequence_num_
         << " get state:  " << status.ToString();
  to_fct->set_fsm_sequence_num(fsm_sequence_num_);
  to_fct->set_fsm_state(cur_state_machine_);
  to_fct->set_fsm_previous_state(previous_state_machine_);
  to_fct->set_perception_sub_state(cur_perception_sub_state_);
  to_fct->set_hdmap_sub_state(cur_hdmap_sub_state_);
  local_view->SetFunctionManagerOutPtr(to_fct);
  // 判断状态机结果是否有效
  if (!status.ok()) {
    history_state_machine_ = cur_state_machine_;
    return status;
  }
  // 有效后根据状态机结果选择对应的地图和routing更新到localview中,先是纯地图模式和视觉模式需要更新
  if (cur_state_machine_ == MachineStateType::HDMAP_TYPE) {
    if (cur_hdmap_sub_state_ == functionmanager::EHP_HDMAP_TYPE) {
      ADEBUG << " ------------- EHP HDMAP_TYPE ------------";
      local_view->SetRoutingResponsePtr(
          real_hdmap_lane_line_->GetRoutingResponse());
      if (history_state_machine_ != MachineStateType::HDMAP_TYPE) {
        local_view->SetMapMsgPtr(real_hdmap_lane_line_->GetMapMsg(true));
      } else {
        local_view->SetMapMsgPtr(real_hdmap_lane_line_->GetMapMsg(false));
      }
      local_view->SetHDMapPtr(real_hdmap_lane_line_->GetHDMapPtr());
    } else if (cur_hdmap_sub_state_ == functionmanager::LOCAL_HDMAP_TYPE) {
      ADEBUG << " ------------- LOCAL HDMAP_TYPE ------------";
#ifndef ISORIN
      local_view->SetRoutingResponsePtr(
          local_hdmap_lane_line_->GetRoutingResponse());
      if (history_state_machine_ != MachineStateType::HDMAP_TYPE) {
        local_view->SetMapMsgPtr(local_hdmap_lane_line_->GetMapMsg(true));
      } else {
        local_view->SetMapMsgPtr(local_hdmap_lane_line_->GetMapMsg(false));
      }
      local_view->SetHDMapPtr(local_hdmap_lane_line_->GetHDMapPtr());
#endif
    } else if (cur_hdmap_sub_state_ == functionmanager::MAP_FUSION_TYPE) {
      local_view->SetRoutingResponsePtr(
          map_fusion_lane_line_->GetRoutingResponse());
      local_view->SetHDMapPtr(map_fusion_lane_line_->GetHDMapPtr());
    }
  } else if (cur_state_machine_ == MachineStateType::PERCEPTION_TYPE) {
    ADEBUG << " ------------- PERCEPTION_TYPE ------------";
    // FLAGS_nolane_mode_first_for_simulation标志，跟车模式优先(只要有障碍
    // 物就会从地图和车道线模式转到跟车模式)
    local_view->SetVehicleStatePtr(vehicle_state_vrf);

    auto localization = std::make_shared<localization::Localization>();
    localization->mutable_pose()->mutable_position()->set_x(0.0);
    localization->mutable_pose()->mutable_position()->set_y(0.0);
    localization->mutable_pose()->mutable_position()->set_z(0.0);
    localization->mutable_pose()->set_heading(0.0);
    auto perception = std::make_shared<perception::PerceptionObstacles>(
        *local_view->GetPerceptionObstacles());
    common::utm_zone::TransPerceptionFromFLU2ENU(localization, perception);
    local_view->SetPerceptionObstaclesPtr(perception);

    local_view->SetRoutingResponsePtr(
        perception_lane_line_->GetRoutingResponse());
    switch (cur_perception_sub_state_) {
      case functionmanager::LANELINE_TYPE:
        local_view->SetRoutingResponsePtr(
            perception_lane_line_->GetRoutingResponse());
        local_view->SetMapMsgPtr(perception_lane_line_->GetMapMsg(true));
        break;
      case functionmanager::NOLANE_TYPE:
        local_view->SetRoutingResponsePtr(missile_mode_->GetRoutingResponse());
        local_view->SetMapMsgPtr(missile_mode_->GetMapMsg(true));
        break;
      case functionmanager::CRUISE_TYPE:
        local_view->SetRoutingResponsePtr(
            adaptive_cruise_->GetRoutingResponse());
        local_view->SetMapMsgPtr(adaptive_cruise_->GetMapMsg(true));
        break;
      default:
        break;
    }
  }
  history_state_machine_ = cur_state_machine_;
  // 其他泊车相关的模式，在状态机内部已经SetRoutingResponsePtr,
  // SetMapMsgPtr，并将运行状态机前面所置内容重置
  // 非纯地图模式则将MapMsg构造为HDmap更新到localview
  if (cur_state_machine_ != MachineStateType::HDMAP_TYPE &&
      cur_state_machine_ != MachineStateType::HDMAP_AVP_TYPE) {
    local_view->SetHDMapPtr(hdmap::CreateMap(*local_view->GetMapMsg()));
  }

  to_fct->mutable_nnp_fct_out()
      ->mutable_localview_time()
      ->set_adaptive_map_time(Clock::NowInMicroseconds() - time_start);
  // 判断routing是否有效
  if (!local_view->HasValidRoutingResponseHeader()) {
    const std::string msg =
        "RoutingResponse are not Ready, cur_state_machine: " +
        functionmanager::MachineStateType_Name(cur_state_machine_);
    AERROR << msg;
    return Status(ErrorCode::LOCALVIEW_FSM_CONSTRUCT_ERROR, msg);
  }
  // 判断map和routing是否匹配
  if (!FLAGS_is_record_replay &&
      !util::IsMatchedHdmapAndRouting(
          local_view->GetRoutingResponse()->header(),
          local_view->GetHDMapPtr()->GetMapHeader().header())) {
    const std::string msg =
        "Map and routing are not matched, cur_state_machine: " +
        functionmanager::MachineStateType_Name(cur_state_machine_);
    AERROR << msg;
    return Status(ErrorCode::LOCALVIEW_FSM_CONSTRUCT_ERROR, msg);
  }

  return Status::OK();
}

void LocalViewStateFront::APALightUpNTP(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<functionmanager::FunctionManagerOut>& to_fct) {
  constexpr uint32 kSlamMapLocation = 6;
  // 判定是否为ntp地图 not_ntp_map
  bool not_ntp_map = cur_state_machine_ != MachineStateType::HDMAP_AVP_TYPE;

  if (local_view->HasLocalization() &&
      local_view->GetLocalization()->rtk_status() == kSlamMapLocation) {
    // set control enbale
    bool is_light_up = not_ntp_map ? BuildNTPMap(local_view) : false;
    to_fct->mutable_avp_fct_out()->set_control_enable(is_light_up);
  } else {
    map_from_slam_.Init();
    is_slam_map_build_ = false;
  }
}

bool LocalViewStateFront::BuildNTPMap(
    const std::shared_ptr<LocalView>& local_view) {
  if (!local_view->HasSlamMapPose()) {
    return false;
  }
  const auto& pose = local_view->GetSlamMapPosePtr();
  auto fct_out = std::make_shared<functionmanager::FunctionManagerOut>();
  int lane_id = 0;
  AVPMapState cur_map_state =
      !is_slam_map_build_ ? AVPMapState::INIT_MAP
      : (map_from_slam_.IsMapRebuild(*pose, fct_out, &lane_id))
          ? AVPMapState::REBUILD_MAP
      : (map_from_slam_.IsLaneUpdate(*pose)) ? AVPMapState::UPDATE_LANE
                                             : AVPMapState::DEFAULT;

  if (cur_map_state != AVPMapState::DEFAULT) {
#ifdef ISX86
    auto& manager = MapManage::getInstance(FLAGS_avp_map_path_file);
#else
    auto& manager = MapManage::getInstance();
#endif
    const auto& map_content = manager.getMapPlanning();
    if (map_content == nullptr) {
      return false;
    }
    std::string error_msg;
    if (!map_from_slam_.ConstructMap(map_content->map, map_content->path, *pose,
                                     cur_map_state, fct_out, &error_msg)) {
      return false;
    }
  }
  if (map_from_slam_.IsPoseNearEnd(*pose, *local_view)) {
    return false;
  }

  is_slam_map_build_ = true;
  return local_view->HasChassis() &&
         local_view->GetChassis()->gear_location() !=
             soc::Chassis::GEAR_REVERSE;
}

bool LocalViewStateFront::UpdateEHPData(
    const std::shared_ptr<TL::ehp::EHP>& ehp_message,
    int* const received_ehp_count) {
  return real_hdmap_lane_line_->UpdateEHPData(ehp_message, received_ehp_count);
}

void LocalViewStateFront::SetFctAndLaneLineDebug(
    functionmanager::FunctionManagerOut* const to_fct,
    const std::shared_ptr<LocalView>& local_view) {
  nnp_fct_out_ = std::make_shared<functionmanager::FunctionManagerOut>();
  auto lane_line_debug = std::make_shared<LanemarkersLaneLine>();
  common::util::FillHeader("lanemarker decider debug", lane_line_debug.get());
  local_view->SetLanemarkersLaneLinePtr(lane_line_debug);
  // 在fct out中添加map_type
  if (local_view->HasMapMsg() && local_view->GetMapMsg()->has_map_type()) {
    to_fct->set_localization_maptype(local_view->GetMapMsg()->map_type());
  }
  to_fct->set_fsm_state(cur_state_machine_);
  to_fct->set_hdmap_sub_state(cur_hdmap_sub_state_);
  to_fct->set_perception_sub_state(cur_perception_sub_state_);
  to_fct->set_perception_status(false);
  to_fct->set_nolane_status(missile_mode_->NoLaneStatus());
  to_fct->set_laneline_status(false);
  to_fct->set_cruise_status(false);
  to_fct->set_hdmap_status(false);
  uint32_t kForceNnnp = FLAGS_enable_hdmap_nnp_mode ? 0x0 : 0x4;
  to_fct->set_soc_2_fct_tbd_u32_01(cur_perception_sub_state_);
  to_fct->set_soc_2_fct_tbd_u32_03(kForceNnnp);
  to_fct->mutable_nnp_fct_out()
      ->mutable_nnp_software_fault()
      ->set_planning_success(FLAGS_global_enable_nnp);
}

void LocalViewStateFront::DealMapFusionAction(
    functionmanager::FunctionManagerOut* to_fct,
    const std::shared_ptr<LocalView>& local_view) {
  ADEBUG << " HasMapMsg: " << local_view->HasMapMsg();
  constexpr uint32_t kLaneSuccessBl = 0x10;
  if (local_view->HasMapMsg()) {
    ADEBUG << " has_map_type: " << local_view->GetMapMsg()->has_map_type()
           << " , has_is_valid: " << local_view->GetMapMsg()->has_is_valid()
           << " ,map_type: " << local_view->GetMapMsg()->map_type()
           << " ,is_valid: " << local_view->GetMapMsg()->is_valid();
  }
  // 如果进入了巡航状态，将发到mcu的横向控制信号抑制
  if (cur_perception_sub_state_ == functionmanager::CRUISE_TYPE) {
    auto soc_04_val = to_fct->soc_2_fct_tbd_u32_04();
    to_fct->set_soc_2_fct_tbd_u32_04(soc_04_val & ~kLaneSuccessBl);
  }
  // 为统一mapfusion和原始跟车跟线的激活条件，在地图为mapfusion模式情况下，且NNP未激活，
  // 那么只要激活条件有一个不满足就将下发到mcu的PIOLT激活信号置为false
  if (local_view->HasMapMsg() && local_view->GetMapMsg()->has_map_type() &&
      local_view->GetMapMsg()->has_is_valid() &&
      local_view->GetMapMsg()->is_valid()) {
    auto* nnp_act =
        to_fct->mutable_nnp_fct_out()->mutable_nnp_activation_conditions();
    if (!(local_view_data_->update_data()->is_nnp_drive_auto() ||
          local_view_data_->update_data()->is_pilot_drive_auto()) &&
        cur_hdmap_sub_state_ == functionmanager::MAP_FUSION_TYPE &&
        !(nnp_act->vehicle_not_in_forbidlane() && nnp_act->vehicle_in_hdmap() &&
          nnp_act->valid_of_lane_localization() &&
          nnp_act->valid_of_lane_routing() &&
          nnp_act->vehicle_not_in_reverselane() &&
          nnp_act->appropriate_current_lane_curve() &&
          nnp_act->appropriate_current_lane_headingerr() &&
          nnp_act->appropriate_current_lane_width() &&
          nnp_act->vehicle_not_in_otherforbidarea())) {
      auto soc_04_val = to_fct->soc_2_fct_tbd_u32_04();
      to_fct->set_soc_2_fct_tbd_u32_04(soc_04_val & ~kLaneSuccessBl);
    }
  }

  if (local_view->HasMapMsg() && local_view->GetMapMsg()->has_map_type() &&
      local_view->GetMapMsg()->has_is_valid() &&
      local_view->GetMapMsg()->map_type() ==
          TL::navigation_hdmap::MapMsg_MapType_PERCEP_MAP &&
      local_view->GetMapMsg()->is_valid() &&
      local_view->GetMapMsg()->has_fault_level()) {
    auto fault_level = local_view->GetMapMsg()->fault_level();
    // 匝道或者高速互通外部降级
    const bool is_not_ramp_or_jct =
        (distance_outof_odd_last_ > 200 || distance_outof_odd_last_ < 0.1) &&
        (distance_downramp_last_ > 1000 || distance_downramp_last_ < 0.1) &&
        road_type_last_ != hdmap::RoadSection_Type_Ramp &&
        road_type_last_ != hdmap::RoadSection_Type_JCT &&
        road_type_last_ != hdmap::RoadSection_Type_SlipRoad;
    // 建图传过来localmapping模式且处于MAP_FUSION模式，且NNP处于激活状态，此时为内部降级不能修改标志位
    const bool is_hdmap_fault =
        (fault_level == 1 || fault_level == 0) &&
        local_view_data_->update_data()->is_nnp_drive_auto() &&
        cur_hdmap_sub_state_ == functionmanager::MAP_FUSION_TYPE &&
        is_not_ramp_or_jct;
    to_fct->mutable_nnp_fct_out()->set_is_in_hdmap(is_hdmap_fault);
    auto* nnp_activation_conditions =
        to_fct->mutable_nnp_fct_out()->mutable_nnp_activation_conditions();

    nnp_activation_conditions->set_vehicle_not_in_forbidlane(is_hdmap_fault);
    nnp_activation_conditions->set_vehicle_not_in_reverselane(is_hdmap_fault);
    nnp_activation_conditions->set_vehicle_not_in_otherforbidarea(
        is_hdmap_fault);
    nnp_activation_conditions->set_appropriate_current_lane_curve(
        is_hdmap_fault);
    nnp_activation_conditions->set_appropriate_current_lane_headingerr(
        is_hdmap_fault);
    nnp_activation_conditions->set_appropriate_current_lane_width(
        is_hdmap_fault);
    nnp_activation_conditions->set_vehicle_in_hdmap(is_hdmap_fault);
    nnp_activation_conditions->set_valid_of_lane_localization(is_hdmap_fault);
    nnp_activation_conditions->set_valid_of_lane_routing(is_hdmap_fault);
  }
  if (to_fct->has_nnp_fct_out() &&
      to_fct->nnp_fct_out().has_nnp_d_distance_outof_odd_sg() &&
      to_fct->nnp_fct_out().has_nnp_d_distance2_downramp_sg()) {
    distance_outof_odd_last_ =
        to_fct->nnp_fct_out().nnp_d_distance_outof_odd_sg();
    distance_downramp_last_ =
        to_fct->nnp_fct_out().nnp_d_distance2_downramp_sg();
  }
  if (to_fct->has_road_type()) {
    road_type_last_ = to_fct->road_type();
  }
}

void LocalViewStateFront::VehicleFunctionStateUpdata(
    const std::shared_ptr<LocalView>& local_view) {
  if (local_view->HasFunctionManagerIn()) {
    local_view_data_->Clear();
    const auto nnp_sys_state =
        local_view->GetFunctionManagerIn()->fct_nnp_in().nnp_sysstate();
    const auto pilot_sys_state =
        local_view->GetFunctionManagerIn()->fct_nnp_in().npilot_state();
    const auto acc_sys_state =
        local_view->GetFunctionManagerIn()->fct_nnp_in().acc_state();
    const bool is_nnp_active =
        (nnp_sys_state == functionmanager::NNPS_ACTIVE ||
         nnp_sys_state == functionmanager::NNPS_OVERRIDE ||
         nnp_sys_state == functionmanager::NNPS_LAT_OVERRIDE ||
         nnp_sys_state == functionmanager::NNPS_LON_OVERRIDE);
    local_view_data_->update_data()->set_is_nnp_drive_auto(is_nnp_active);
    local_view_data_->update_data()->set_is_nnp_npilot(
        nnp_sys_state == functionmanager::NNPS_NPILOT);
    const bool is_pilot_active =
        (pilot_sys_state == functionmanager::FctToNnpInput::PILOT_ACTIVE ||
         pilot_sys_state == functionmanager::FctToNnpInput::PILOT_SUSPEND);
    local_view_data_->update_data()->set_is_pilot_drive_auto(is_pilot_active);
    const auto drive_mode = local_view->GetChassis()->driving_mode();
    // 判断车辆是否一直处于自动驾驶状态，需要考虑横纵向都override的情形
    bool is_vehicle_active =
        (drive_mode == Chassis::AUTO_SPEED_ONLY ||
         drive_mode == Chassis::AUTO_STEER_ONLY ||
         drive_mode == Chassis::COMPLETE_AUTO_DRIVE ||
         acc_sys_state == functionmanager::FctToNnpInput::ACC_OVERRIDE ||
         pilot_sys_state == functionmanager::FctToNnpInput::PILOT_SUSPEND ||
         nnp_sys_state == functionmanager::NNPSysState::NNPS_OVERRIDE);
    local_view_data_->update_data()->set_is_vehicle_active(is_vehicle_active);
  }
}
}  // namespace planning
}  // namespace TL
