
//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#pragma once

#include <memory>
#include <string>

#include <cmath>
#include "common/configs/config_gflags.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/util/util.h"
#include "planning/localview/state_machine/base_state.h"

namespace TL {
namespace planning {
using google::protobuf::uint32;
using TL::common::ErrorCode;
using TL::common::Status;

/**
 * @class InitialState
 * @brief 初始状态
 */
class InitialState : public BaseState {
 public:
  InitialState() = default;
  ~InitialState() override = default;

  bool Init() override;
  /**
   * @brief Buildlocalview
   *
   * @param local_view Localview
   * @return Status ERROR code and error message
   */
  Status BuildLocalView(const std::shared_ptr<LocalView>& local_view) override;

  /**
   * @brief 进入状态时调用
   * @param event 引起状态转移的事件
   * @param state_machine 状态机对象
   */
  template <typename Event, typename StateMachine>
  void on_entry(const Event& event, const StateMachine& state_machine) {
    UNUSED(event);
    UNUSED(state_machine);
    // TODO(fml)
  }

  /**
   * @brief 离开状态时调用
   * @param event 引起状态转移的事件
   * @param state_machine 状态机对象
   */
  template <typename Event, typename StateMachine>
  void on_exit(const Event& event, const StateMachine& state_machine) {
    UNUSED(event);
    UNUSED(state_machine);
    // TODO(fml)
  }

  /**
   * @brief 处理BuildLocalView事件
   * @param event 事件对象
   * @param state_machine 状态机对象
   */
  template <typename StateMachine>
  void ProcessBuildLocalView(BuildLocalViewEvent* event,
                             StateMachine* state_machine) {
    UNUSED(event);
    ACHECK(state_machine);
    if (!state_machine->GetLocalView().HasFunctionManagerIn() ||
        !state_machine->GetLocalView().HasValidLocalizationHeader()) {
      const std::string msg =
          "function_manager_in_ptr  or is nullptr or has no localization "
          "header";
      AERROR << msg;
      state_machine->SetMachineStateType(
          functionmanager::MachineStateType::INITIAL_TYPE);
      state_machine->SetState(
          Status(ErrorCode::LOCALVIEW_FSM_INITIAL_ERROR, msg));
      return;
    }

    bool available_avp_mode_wait_run_state = false;
    const auto fmi = state_machine->GetLocalView().GetFunctionManagerIn();
    bool has_opt_park_lot = false;
    bool is_vehicle_in_park = true;
    bool is_vehicle_standstill = false;
    if (state_machine->GetLocalView().HasParkingLotOutArray() &&
        state_machine->GetLocalView().HasVehicleState()) {
      has_opt_park_lot = util::IsHasOptParkLot(
          *state_machine->GetLocalView().GetParkingLotOutArray());
      is_vehicle_in_park = util::IsEgoInParkLot(
          *state_machine->GetLocalView().GetParkingLotOutArray(),
          common::math::Vec2d(
              state_machine->GetLocalView().GetVehicleState()->x(),
              state_machine->GetLocalView().GetVehicleState()->y()));
      is_vehicle_standstill = fabs(state_machine->GetLocalView()
                                       .GetVehicleState()
                                       ->linear_velocity()) <
                              FLAGS_initial_trans_to_park_speed_threshold;
    }
    bool use_baidu_mode = false;
#ifdef FOR_BAIDU_SIMULATION
    use_baidu_mode = true;
#endif
    if (fmi->ta_pilot_mode() == functionmanager::AVP) {
      AINFO << "fct_avp_in: " << fmi->fct_avp_in().DebugString();
      constexpr uint32 kLocalLocation = 8;
      constexpr uint32 kDrLocation = 21;
      constexpr int kDrLocationCount = 3;
      if (fmi->fct_avp_in().sys_mode() ==
              functionmanager::AvpFctIn_StateType_TBA &&
          state_machine->GetLocalView().HasLocalization() &&
          state_machine->GetLocalView().GetLocalization()->location_state() ==
              kDrLocation) {
        ++location_dr_count_;
      } else {
        location_dr_count_ = 0;
      }
      switch (fmi->fct_avp_in().sys_mode()) {
        case functionmanager::AvpFctIn_StateType_NOSTTYPE:
          available_avp_mode_wait_run_state = false;
          break;
        case functionmanager::AvpFctIn_StateType_APA:
        case functionmanager::AvpFctIn_StateType_RPA:
        case functionmanager::AvpFctIn_StateType_DAPA:
        case functionmanager::AvpFctIn_StateType_LAPA_MAPPING:
          if (fmi->fct_avp_in().sys_run_state() ==
                  functionmanager::AvpFctIn_SysRunState_PARKCONFIG ||
              fmi->fct_avp_in().sys_run_state() ==
                  functionmanager::AvpFctIn_SysRunState_PARKING ||
              fmi->fct_avp_in().sys_run_state() ==
                  functionmanager::AvpFctIn_SysRunState_STRAIGHTCONTROL ||
              (fmi->fct_avp_in().sys_run_state() ==
                   functionmanager::AvpFctIn_SysRunState_PARKSTART &&
               has_opt_park_lot && !is_vehicle_in_park &&
               is_vehicle_standstill)) {
            InitialToParkingEvent ipe;
            state_machine->SetHistoryMachineStateType(
                functionmanager::MachineStateType::INITIAL_TYPE);
            state_machine->process_event(ipe);
            AINFO << "---------Initial to Parking--------";
            return;
          }
          available_avp_mode_wait_run_state = true;
          break;
        case functionmanager::AvpFctIn_StateType_LAPA:
        case functionmanager::AvpFctIn_StateType_ISM:
        case functionmanager::AvpFctIn_StateType_NTP:
          if (fmi->fct_avp_in().sys_run_state() ==
                  functionmanager::AvpFctIn_SysRunState_CRUSING ||
              fmi->fct_avp_in().sys_run_state() ==
                  functionmanager::AvpFctIn_SysRunState_NNSING ||
              fmi->fct_avp_in().sys_run_state() ==
                  functionmanager::AvpFctIn_SysRunState_NTPING) {
            InitialToHDMapAVPEvent ihate;
            state_machine->SetHistoryMachineStateType(
                functionmanager::MachineStateType::INITIAL_TYPE);
            state_machine->process_event(ihate);
            AINFO << "---------Initial to HDMapAVP--------";
            return;
          }
          available_avp_mode_wait_run_state = true;
          break;
        case functionmanager::AvpFctIn::LOCALIZATION:
        case functionmanager::AvpFctIn::LOCALIZATION_BACKGROUND:
          if (state_machine->GetLocalView().HasLocalization() &&
              state_machine->GetLocalView()
                      .GetLocalization()
                      ->location_state() == kLocalLocation &&
              fmi->fct_avp_in().sys_run_state() ==
                  functionmanager::AvpFctIn::PARKSTART) {
            InitialToHDMapAVPEvent ihate;
            state_machine->SetHistoryMachineStateType(
                functionmanager::MachineStateType::INITIAL_TYPE);
            state_machine->process_event(ihate);
            AINFO << "---------Initial to HDMapAVP--------";
            return;
          }
          available_avp_mode_wait_run_state = false;
          break;
        case functionmanager::AvpFctIn_StateType_TBA:
          if (fmi->fct_avp_in().sys_run_state() ==
                  functionmanager::AvpFctIn_SysRunState_TBAING &&
              (location_dr_count_ >= kDrLocationCount || use_baidu_mode)) {
            location_dr_count_ = 0;
            InitialToHistoryTraceEvent ihte;
            state_machine->SetHistoryMachineStateType(
                functionmanager::MachineStateType::INITIAL_TYPE);
            state_machine->process_event(ihte);
            AINFO << "---------Initial to HistoryTrace--------";
            return;
          }
          available_avp_mode_wait_run_state = true;
          break;
        case functionmanager::AvpFctIn_StateType_AVP:
        default:
          const std::string msg = "Unexpected avp state:";
          AWARN << msg << fmi->fct_avp_in().DebugString();
          state_machine->SetMachineStateType(
              functionmanager::MachineStateType::INITIAL_TYPE);
          state_machine->SetState(
              Status(ErrorCode::LOCALVIEW_FSM_INITIAL_ERROR, msg));
          return;
      }
      if (available_avp_mode_wait_run_state) {
        const std::string msg = "avp_mode_wait_run_state";
        state_machine->SetMachineStateType(
            functionmanager::MachineStateType::INITIAL_TYPE);
        state_machine->SetState(
            Status(ErrorCode::LOCALVIEW_FSM_INITIAL_ERROR, msg));
        return;
      }
      const std::string msg = "avp system mode is not ready";
      state_machine->SetMachineStateType(
          functionmanager::MachineStateType::INITIAL_TYPE);
      state_machine->SetState(
          Status(ErrorCode::LOCALVIEW_FSM_INITIAL_ERROR, msg));
      return;
    }

    // 行车
    ADEBUG << "nolane_status: "
           << state_machine->GetMutableLocalView()
                  ->GetFunctionManagerOut()
                  ->nolane_status();
    if (state_machine->GetMutableLocalView()
            ->GetFunctionManagerOut()
            ->hdmap_status()) {
      if (FLAGS_use_hdmap_mode) {
        // 在车辆在地图中，强制使用高精地图模式已打开
        InitialToMapEvent initial_to_map_event;
        state_machine->SetHistoryMachineStateType(
            functionmanager::MachineStateType::INITIAL_TYPE);
        state_machine->process_event(initial_to_map_event);
      } else if (state_machine->GetMutableLocalView()
                     ->GetFunctionManagerOut()
                     ->perception_status()) {
        // 在车辆在地图中，强制使用高精地图模式未打开,此时转到融合模式
        ADEBUG << "in map and FLAGS_use_hdmap_mode = 0";
        InitialToFusionEvent initial_to_fusion_event;
        state_machine->SetHistoryMachineStateType(
            functionmanager::MachineStateType::INITIAL_TYPE);
        state_machine->process_event(initial_to_fusion_event);
        ADEBUG << "first to fused end.";
        return;
      }
    } else if (state_machine->GetMutableLocalView()
                   ->GetFunctionManagerOut()
                   ->perception_status() ||
               state_machine->GetMutableLocalView()
                   ->GetFunctionManagerOut()
                   ->cruise_status() ||
               state_machine->GetMutableLocalView()
                   ->GetFunctionManagerOut()
                   ->nolane_status()) {
      // BuildLocalViewEvent事件触发，感知模式下，不管强制使用高精地图标志是否打开，只要
      // 车辆不在地图中，就保持感知模式，无event发出
      InitialToPerceptionEvent initial_to_perception_event;
      state_machine->SetHistoryMachineStateType(
          functionmanager::MachineStateType::INITIAL_TYPE);
      state_machine->process_event(initial_to_perception_event);
      ADEBUG << "--------- Initial to perception mode not in "
                "hdmap--------";
      return;
    }
  }

  template <typename StateMachine>
  void ProcessParkingToInitialEvent(ParkingToInitialEvent* event,
                                    StateMachine* state_machine) {
    UNUSED(event);
    ACHECK(state_machine);

    state_machine->SetState(Status(ErrorCode::LOCALVIEW_FSM_INITIAL_ERROR,
                                   "initial from parking mode"));
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::INITIAL_TYPE);
    AINFO << "---------initial from parking mode--------";
  }

  template <typename StateMachine>
  void ProcessHistoryTraceToInitialEvent(HistoryTraceToInitialEvent* event,
                                         StateMachine* state_machine) {
    UNUSED(event);
    ACHECK(state_machine);

    state_machine->SetState(Status(ErrorCode::LOCALVIEW_FSM_INITIAL_ERROR,
                                   "initial from HistoryTrace mode"));
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::INITIAL_TYPE);
    AINFO << "---------initial from HistoryTrace mode--------";
  }

  template <typename StateMachine>
  void ProcessHDMapAVPToInitialEvent(HDMapAVPToInitialEvent* event,
                                     StateMachine* state_machine) {
    UNUSED(event);
    ACHECK(state_machine);

    state_machine->SetState(Status(ErrorCode::LOCALVIEW_FSM_INITIAL_ERROR,
                                   "initial from HDMapAVP mode"));
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::INITIAL_TYPE);
    AINFO << "---------initial from HDMapAVP mode--------";
  }

  template <typename StateMachine>
  void ProcessNnpToInitialEvent(NnpToInitialEvent* event,
                                StateMachine* state_machine) {
    UNUSED(event);
    ACHECK(state_machine);

    state_machine->SetState(Status(ErrorCode::LOCALVIEW_FSM_INITIAL_ERROR,
                                   "initial from nnp mode"));
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::INITIAL_TYPE);
    AINFO << "---------initial from nnp mode--------";
  }

 private:
  int location_dr_count_ = 0;
};

SETACTIONSTRUCT(BuildInitialLocalViewAction, ProcessBuildLocalView);
SETACTIONSTRUCT(InitialFromParkingAction, ProcessParkingToInitialEvent);
SETACTIONSTRUCT(InitialFromHistoryTraceAction,
                ProcessHistoryTraceToInitialEvent);
SETACTIONSTRUCT(InitialFromHDMapAVPAction, ProcessHDMapAVPToInitialEvent);
SETACTIONSTRUCT(InitialFromNnpAction, ProcessNnpToInitialEvent);

}  // namespace planning
}  // namespace TL
