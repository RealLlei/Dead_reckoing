//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#pragma once

#include <memory>
#include <string>

#include "common/configs/config_gflags.h"
#include "common/status/status.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/dependency_injector.h"
#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/state_machine/base_state.h"

namespace TL {
namespace planning {
using TL::common::ErrorCode;
using TL::common::Status;
using TL::planning::lanelineprocess::DebounceModule;

class PerceptionLaneLineState : public BaseState {
 public:
  PerceptionLaneLineState();
  ~PerceptionLaneLineState() override = default;

  bool Init() override;

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
    ADEBUG << "perception_on_entry";
    // navigation_hdmap_.Init();
    // navigation_hdmap_.Start();
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
    if (state_machine->GetLocalView().HasFunctionManagerIn()) {
      const auto& fmi = state_machine->GetLocalView().GetFunctionManagerIn();
      if (
          /*(nnp_sys_state == functionmanager::NNPSysState::NNPS_OFF ||
             nnp_sys_state == functionmanager::NNPSysState::NNPS_PASSIVE ||
             nnp_sys_state ==
                 functionmanager::NNPSysState::NNPS_FAIL_TmpUnavabl ||
             nnp_sys_state == functionmanager::NNPSysState::NNPS_FAIL_ServReq ||
             nnp_sys_state == functionmanager::NNPSysState::NNPS_TO) &&*/
          fmi->fct_avp_in().sys_mode() !=
          functionmanager::AvpFctIn_StateType_NOSTTYPE) {
        AINFO << "has no NNPSysState and has avp state, so return init state "
                 "from perception mode";
        cur_sub_state_ = functionmanager::SUB_INITIAL_TYPE;
        state_machine->SetPerceptionSubState(cur_sub_state_);
        NnpToInitialEvent nnp_to_initialstate_event;
        state_machine->process_event(nnp_to_initialstate_event);
        return;
      }
    }

    if (FLAGS_nolane_mode_first_for_simulation &&
        state_machine->GetMachineStateType() ==
            TL::functionmanager::MachineStateType::PERCEPTION_TYPE) {
      cur_sub_state_ = functionmanager::NOLANE_TYPE;
      state_machine->SetPerceptionSubState(cur_sub_state_);
      Status status(Status::OK());
      const std::string msg = "shield navigationperception";
      if (!state_machine->GetMutableLocalView()
               ->GetFunctionManagerOut()
               ->nolane_status()) {
        status = Status(ErrorCode::LOCALVIEW_FSM_PERCEPTION_ERROR, msg);
      }
      state_machine->SetState(status);
      AERROR << msg;
      return;
    }
    // 入地图条件判断
    local_view_data_ = state_machine->GetMutableLocalViewData();
    if (DealChangeModeConditions(state_machine->GetMutableLocalView())) {
      if (FLAGS_use_hdmap_mode) {
        // 在车辆在地图中，强制使用高精地图模式已打开
        PerceptionToMapWithFlagEvent per_to_map_event;
        state_machine->SetHistoryMachineStateType(
            functionmanager::MachineStateType::PERCEPTION_TYPE);
        cur_sub_state_ = functionmanager::SUB_INITIAL_TYPE;
        state_machine->SetPerceptionSubState(cur_sub_state_);
        AERROR << "to hdmap";
        state_machine->process_event(per_to_map_event);
        return;
      }
      if (state_machine->GetMutableLocalView()
              ->GetFunctionManagerOut()
              ->perception_status()) {
        // 在车辆在地图中，强制使用高精地图模式未打开,此时转到融合模式
        PerceptionToFusionEvent per_to_fusion_event;
        state_machine->SetHistoryMachineStateType(
            functionmanager::MachineStateType::PERCEPTION_TYPE);
        cur_sub_state_ = functionmanager::SUB_INITIAL_TYPE;
        state_machine->SetPerceptionSubState(cur_sub_state_);
        state_machine->process_event(per_to_fusion_event);
        AERROR << "first to fused end.";
        return;
      }
    }
    // BuildLocalViewEvent事件触发，感知模式下，不管强制使用高精地图标志是否打开，只要
    // 车辆不在地图中，就保持感知模式，无event发出
    // BuildLocalView(state_machine->GetMutableLocalView());
    state_machine->SetPerceptionSubState(cur_sub_state_);
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::PERCEPTION_TYPE);
    state_machine->SetState(GetSubStatus(state_machine->GetMutableLocalView()));
    ADEBUG << "---------perception mode--------";
    // TODO(fml)
  }

  /**
   * @brief 处理MapToPerceptionEvent事件
   * @param event 事件对象
   * @param state_machine 状态机对象
   */
  template <typename StateMachine>
  void ProcessMapToPerceptionEvent(MapToPerceptionEvent* event,
                                   StateMachine* state_machine) {
    UNUSED(event);
    if (FLAGS_nolane_mode_first_for_simulation) {
      state_machine->SetMachineStateType(
          functionmanager::MachineStateType::PERCEPTION_TYPE);
      cur_sub_state_ = functionmanager::NOLANE_TYPE;
      state_machine->SetPerceptionSubState(cur_sub_state_);
      state_machine->SetState(
          GetSubStatus(state_machine->GetMutableLocalView()));
      return;
    }
    is_nnp_auto_debounce_.Reset();
    local_view_data_ = state_machine->GetMutableLocalViewData();
    BuildLocalView(state_machine->GetMutableLocalView());
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::PERCEPTION_TYPE);
    state_machine->SetPerceptionSubState(cur_sub_state_);
    auto status = GetSubStatus(state_machine->GetMutableLocalView());
    state_machine->SetState(status);
    auto const& nnp_fct_out = state_machine->GetMutableLocalView()
                                  ->GetFunctionManagerOut()
                                  ->nnp_fct_out();
    is_change_dueto_internalreasons_ = nnp_fct_out.nnp_statechange_conditions()
                                           .is_change_dueto_internalreasons();
    initiative_degrate_bl_ = nnp_fct_out.initiative_degrate_bl();
    SetHdmapActionCondations(state_machine->GetMutableLocalView());
    ADEBUG << "---------perception mode--------"
           << " STATUS = " << status.ToString()
           << ", is_change_dueto_internalreasons_: "
           << is_change_dueto_internalreasons_;
    // TODO(fml)
  }

  /**
   * @brief 处理FusionToPerceptionEvent事件
   * @param event 事件对象
   * @param state_machine 状态机对象
   */
  template <typename StateMachine>
  void ProcessFusionToPerceptionEvent(FusionToPerceptionEvent* event,
                                      StateMachine* state_machine) {
    UNUSED(event);
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::PERCEPTION_TYPE);
    Status status(Status::OK());
    const std::string msg = "perception mode from fusion";
    if (!state_machine->GetMutableLocalView()
             ->GetFunctionManagerOut()
             ->perception_status()) {
      status = Status(ErrorCode::LOCALVIEW_FSM_PERCEPTION_ERROR, msg);
    }
    state_machine->SetState(status);
    AINFO << msg;
    // TODO(fml)
  }

  /**
   * @brief 处理InitialToPerceptionEvent事件
   * @param event 事件对象
   * @param state_machine 状态机对象
   */
  template <typename StateMachine>
  void ProcessInitialToPerceptionEvent(InitialToPerceptionEvent* event,
                                       StateMachine* state_machine) {
    UNUSED(event);
    if (FLAGS_nolane_mode_first_for_simulation) {
      state_machine->SetMachineStateType(
          functionmanager::MachineStateType::PERCEPTION_TYPE);
      cur_sub_state_ = functionmanager::NOLANE_TYPE;
      state_machine->SetPerceptionSubState(cur_sub_state_);
      state_machine->SetState(
          GetSubStatus(state_machine->GetMutableLocalView()));
      return;
    }
    local_view_data_ = state_machine->GetMutableLocalViewData();
    BuildLocalView(state_machine->GetMutableLocalView());
    SetHdmapActionCondations(state_machine->GetMutableLocalView());
    state_machine->SetPerceptionSubState(cur_sub_state_);
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::PERCEPTION_TYPE);
    state_machine->SetState(GetSubStatus(state_machine->GetMutableLocalView()));
    ADEBUG << "---------perception mode from initial--------";
    // TODO(fml)
  }

 private:
  void SubStateDecider(bool laneline_flag, bool nolane_flag, bool cruise_flag,
                       bool nolane_first_flag, bool speed_only_flag,
                       bool original_lat_override_flag);
  static bool DeciderObsBeforeVehicle(
      const std::shared_ptr<LocalView>& local_view);
  /**
   * @brief Get the Sub Status object
   *
   * @param local_view
   * @return Status error code and error message
   */
  Status GetSubStatus(const std::shared_ptr<LocalView>& local_view);
  bool DealChangeModeConditions(
      const std::shared_ptr<LocalView>& local_view);
  void SetHdmapActionCondations(
      const std::shared_ptr<LocalView>& local_view) const;
  functionmanager::PerceptionSubState cur_sub_state_ =
      functionmanager::SUB_INITIAL_TYPE;
  functionmanager::PerceptionSubState history_sub_state_ =
      functionmanager::SUB_INITIAL_TYPE;
  double sub_state_time_{0.0};
  bool history_laneline_flag_ = false;
  DebounceModule nolane_rise_debounce_;
  DebounceModule is_nnp_auto_debounce_{0.5, 0.0, 0.1};
  bool is_change_dueto_internalreasons_ = false;
  bool initiative_degrate_bl_{false};
};

SETACTIONSTRUCT(BuildPerceptionLocalViewAction, ProcessBuildLocalView);
SETACTIONSTRUCT(PerceptionFromFusionAction, ProcessFusionToPerceptionEvent);
SETACTIONSTRUCT(PerceptionFromMapAction, ProcessMapToPerceptionEvent);
SETACTIONSTRUCT(PerceptionFromInitialAction, ProcessInitialToPerceptionEvent);

}  // namespace planning
}  // namespace TL
