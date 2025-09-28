//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#pragma once
#include <memory>
#include <string>
#include <vector>

#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/util/message_util.h"
#if defined(ISORIN) || defined(FOR_BAIDU_SIMULATION)
#include "include/map_manage.h"
#else
#include "./map_manage.h"
#endif
#include "map/avp_slam_map/slam_to_map.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/planning_context.h"
#include "planning/localview/local_view.h"
#include "planning/localview/state_machine/base_state.h"
#include "planning/localview/state_machine/event.h"
#include "planning/routing/routing.h"

#include "proto/fsm/avp_fct.pb.h"
#include "proto/map/avp_map_origin.pb.h"
#include "proto/map/map.pb.h"

namespace TL {
namespace planning {
using google::protobuf::uint32;
using TL::common::ErrorCode;
using TL::common::Status;
#if defined(ISORIN) || defined(FOR_BAIDU_SIMULATION)
using TL::netaos::MapManage;
#else
using TL::MapManage;
#endif

class HDMapAVPState final : public BaseState {
 public:
  HDMapAVPState();
  ~HDMapAVPState() override = default;

  bool Init() override;

  /**
   * @brief Set the Routing Info object
   * 
   * @param inrouting 
   * @param error_msg 
   * @return true 
   * @return false 
   */
  bool SetRoutingInfo(TL::routing::RoutingResponse* inrouting,
                      std::string* error_msg);

  /**
   * @brief Buildlocalview
   *
   * @param local_view Localview
   * @return Status ERROR code and error message
   */
  common::Status BuildLocalView(
      const std::shared_ptr<LocalView>& local_view) override;

  /**
   * @brief 
   * 
   * @param pose 
   * @param cur_lane_id 
   * @param ptr_trajectory_pb 
   */
  void SendEndPoint2ADC(const std::shared_ptr<LocalView>& local_view,
                        ADCTrajectory* ptr_trajectory_pb);
  /**
   * @brief 
   * 
   * @param local_view 
   * @param error_msg 
   * @return true 
   * @return false 
   */
  bool AddParkingLotToMapAndRouting(
      const std::shared_ptr<LocalView>& local_view, std ::string* error_msg);

  /**
  * @brief Set the Routing For Baidu Simulation object
  * 
  * @param local_view 
  * @param routing_response_ptr 
  * @return true 
  * @return false 
  */
  bool SetRoutingForBaiduSimulation(
      const std::shared_ptr<LocalView>& local_view,
      routing::RoutingResponse* routing_response_ptr);

  /**
   * @brief 
   * 
   * @param local_view 
   * @return true 
   * @return false 
   */
  bool ProcessSimulation(const std::shared_ptr<LocalView>& local_view);

  /**
   * @brief 
   * 
   * @param local_view 
   * @return common::Status 
   */
  common::Status ProcessBaiduSim(const std::shared_ptr<LocalView>& local_view);

  /**
   * @brief 
   * 
   * @param local_view 
   * @return common::Status 
   */
  common::Status ProcessRealCarAndReplay(
      const std::shared_ptr<LocalView>& local_view);

  template <typename Event, typename StateMachine>
  void on_entry(const Event& event, const StateMachine& state_machine) {
    UNUSED(event);
    UNUSED(state_machine);
  }

  template <typename Event, typename StateMachine>
  void on_exit(const Event& event, const StateMachine& state_machine) {
    UNUSED(event);
    UNUSED(state_machine);
  }

  template <typename StateMachine>
  void ProcessBuildLocalView(BuildLocalViewEvent* event,
                             StateMachine* state_machine) {
    UNUSED(event);
    ACHECK(state_machine);
    if (!state_machine->GetLocalView().HasFunctionManagerIn()) {
      const std::string msg = {"function_manager_in_ptr is nullptr"};
      AERROR << msg;
      state_machine->SetMachineStateType(
          functionmanager::MachineStateType::HDMAP_AVP_TYPE);
      state_machine->SetState(
          Status(ErrorCode::LOCALVIEW_FSM_HDMAPAVP_ERROR, msg));
      return;
    }

    constexpr uint32 kLocalLocation = 8;
    const auto fmi = state_machine->GetLocalView().GetFunctionManagerIn();
    const bool is_localization_status_ok =
        state_machine->GetLocalView().HasLocalization()
            ? state_machine->GetLocalView()
                      .GetLocalization()
                      ->location_state() == kLocalLocation
            : false;
    if (fmi->fct_avp_in().sys_mode() != functionmanager::AvpFctIn::ISM &&
        fmi->fct_avp_in().sys_mode() != functionmanager::AvpFctIn::LAPA &&
        fmi->fct_avp_in().sys_mode() != functionmanager::AvpFctIn::NTP &&
        ((fmi->fct_avp_in().sys_mode() !=
              functionmanager::AvpFctIn::LOCALIZATION &&
          fmi->fct_avp_in().sys_mode() !=
              functionmanager::AvpFctIn::LOCALIZATION_BACKGROUND) ||
         !is_localization_status_ok)) {
      HDMapAVPToInitialEvent hatie;
      state_machine->SetHistoryMachineStateType(
          functionmanager::MachineStateType::HDMAP_AVP_TYPE);
      state_machine->process_event(hatie);
      AINFO << "---------HDMapAVP to Initial--------";
      return;
    }
    switch (fmi->fct_avp_in().sys_run_state()) {
      case functionmanager::AvpFctIn::STOP:
      case functionmanager::AvpFctIn::ERROR:
      case functionmanager::AvpFctIn::INVERTIBLE: {
        HDMapAVPToInitialEvent hatie;
        state_machine->SetHistoryMachineStateType(
            functionmanager::MachineStateType::HDMAP_AVP_TYPE);
        state_machine->process_event(hatie);
        AINFO << "---------HDMapAVP to Initial--------";
        return;
      } break;
      case functionmanager::AvpFctIn::QUIT: {
        const std::string msg = {
            "AVP Quit info: " +
            functionmanager::AvpFctIn_WarningInfoErrorType_Name(
                fmi->fct_avp_in().sys_warning_info())};
        AERROR << msg;
        state_machine->SetMachineStateType(
            functionmanager::MachineStateType::HDMAP_AVP_TYPE);
        state_machine->SetState(
            Status(ErrorCode::LOCALVIEW_FSM_INITIAL_ERROR, msg));
        return;
      } break;
      default:
        break;
    }

    auto status = BuildLocalView(state_machine->GetMutableLocalView());
    state_machine->SetState(status);
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::HDMAP_AVP_TYPE);
    AINFO << "-------- hdmap avp mode-------- status = " << status.ToString();
  }

  template <typename StateMachine>
  void ProcessInitialToHDMapAVPEvent(InitialToHDMapAVPEvent* event,
                                     StateMachine* state_machine) {
    UNUSED(event);
    ACHECK(state_machine);
    hdmap_avp_ptr_ = nullptr;
    map_from_slam_.Init();
    map_content_ = nullptr;
    routing_response_.Clear();
    parking_lot_array_enu_ptr_ = nullptr;

    auto status = BuildLocalView(state_machine->GetMutableLocalView());
    state_machine->SetState(status);
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::HDMAP_AVP_TYPE);
    ADEBUG << "---------hdmap avp mode from initial--------  status = "
           << status.ToString();
  }

 private:
  std::shared_ptr<hdmap::HDMap> hd_map_;
  std::shared_ptr<TL::navigation_hdmap::MapMsg> hdmap_avp_ptr_;
  std::shared_ptr<routing::RoutingRequest> last_routing_request_;
  std::shared_ptr<TL::perception::ParkingLotOutArray>
      parking_lot_array_enu_ptr_;

  routing::RoutingResponse routing_response_;
  common::PathPoint routing_end_point_;
  routing::Routing routing_;

  TL::hdmap::AVPSlamMap map_from_slam_;
  std::shared_ptr<perception_map::semanticmap::AvpTaskInfo> avp_park_task_{
      nullptr};
  std::shared_ptr<MapManage::MapContent> map_content_{nullptr};
};

// NOLINTBEGIN
SETACTIONSTRUCT(BuildHDMapAVPLocalViewAction, ProcessBuildLocalView);
SETACTIONSTRUCT(HDMapAVPFromInitialAction, ProcessInitialToHDMapAVPEvent);
// NOLINTEND
}  // namespace planning
}  // namespace TL
