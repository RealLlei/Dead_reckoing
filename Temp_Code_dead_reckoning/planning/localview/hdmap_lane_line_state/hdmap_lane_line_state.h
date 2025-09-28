
//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#pragma once

#include <algorithm>
#include <atomic>
#include <future>
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/status/status.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/dependency_injector.h"
#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/util/util.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/state_machine/base_state.h"
#include "planning/localview/state_machine/event.h"
#include "planning/routing/routing.h"
#include "proto/fsm/function_manager.pb.h"

namespace TL {
namespace planning {
using TL::common::ErrorCode;
using TL::common::Status;
using TL::planning::lanelineprocess::DebounceModule;

class HDMapLaneLineState : public BaseState {
 public:
  HDMapLaneLineState();
  ~HDMapLaneLineState() override = default;
  bool Init() override;
  /**
   * @brief Buildlocalview
   *
   * @param local_view Localview
   * @return Status ERROR code and error message
   */
  Status BuildLocalView(const std::shared_ptr<LocalView>& local_view) override;

  Status BuildLocalView(
      const std::shared_ptr<LocalView>& local_view,
      const std::shared_ptr<functionmanager::FunctionManagerOut>& ehp_fct_out);

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
    if (state_machine->GetLocalView().HasFunctionManagerIn()) {
      const auto& fmi = state_machine->GetLocalView().GetFunctionManagerIn();
      if (
          /*(nnp_sys_state == functionmanager::NNPSysState::NNPS_OFF ||
             nnp_sys_state == functionmanager::NNPSysState::NNPS_PASSIVE ||
             nnp_sys_state ==
                 functionmanager::NNPSysState::NNPS_FAIL_TmpUnavabl ||
             nnp_sys_state == functionmanager::NNPSysState::NNPS_FAIL_ServReq)
             &&*/
          fmi->fct_avp_in().sys_mode() !=
          functionmanager::AvpFctIn_StateType_NOSTTYPE) {
        state_machine->SetMachineStateType(
            functionmanager::MachineStateType::HDMAP_TYPE);
        cur_sub_state_ = functionmanager::HD_INITIAL_TYPE;
        state_machine->SetHdmapSubState(cur_sub_state_);
        AINFO << "has no NNPSysState and has avp state, so return init state "
                 "from hdmap mode";
        NnpToInitialEvent nnp_to_initialstate_event;
        state_machine->process_event(nnp_to_initialstate_event);
        return;
      }
    }
    // 在FLAGS_use_fct_mode模式下，不会跳转到感知模式，无论成功与否
    if (FLAGS_use_fct_mode) {
      ADEBUG << " use_fct_mode.";
      state_machine->SetMachineStateType(
          functionmanager::MachineStateType::HDMAP_TYPE);
      Status status(Status::OK());
      if (!state_machine->GetMutableLocalView()
               ->GetFunctionManagerOut()
               ->hdmap_status()) {
        status = Status(ErrorCode::LOCALVIEW_FSM_HDMAP_ERROR, "use_fct_mode");
      }
      state_machine->SetState(status);
      return;
    }
    local_view_data_ = state_machine->GetMutableLocalViewData();
    BuildLocalView(state_machine->GetMutableLocalView(),
                   state_machine->GetEhpFctOut());

    if (!DealChangeModeConditions(state_machine->GetMutableLocalView())) {
      ADEBUG << "  -----map_to_perception----- ";
      MapToPerceptionEvent map_to_perception_event;
      state_machine->SetHistoryMachineStateType(
          functionmanager::MachineStateType::HDMAP_TYPE);
      cur_sub_state_ = functionmanager::HD_INITIAL_TYPE;
      state_machine->SetHdmapSubState(cur_sub_state_);
      state_machine->process_event(map_to_perception_event);
      return;
    }

    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::HDMAP_TYPE);
    state_machine->SetHdmapSubState(cur_sub_state_);
    state_machine->SetState(GetSubStatus(state_machine->GetMutableLocalView()));
    // force using  hdmap mode
    ADEBUG << "constructor is only using hdmap";
    ADEBUG << "---------force hdmap mode--------";

    // TODO(fml)
  }

  /**
   * @brief 处理PerceptionToMapWithFlagEvent事件
   * @param event 事件对象
   * @param state_machine 状态机对象
   */
  template <class StateMachine>
  void ProcessPerceptionToMapWithFlagEvent(PerceptionToMapWithFlagEvent* event,
                                           StateMachine* state_machine) {
    UNUSED(event);
    BuildLocalView(state_machine->GetMutableLocalView(),
                   state_machine->GetEhpFctOut());
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::HDMAP_TYPE);
    state_machine->SetHdmapSubState(cur_sub_state_);
    state_machine->SetState(GetSubStatus(state_machine->GetMutableLocalView()));
    AINFO << "---------force hdmap mode from perception--------";
    // TODO(fml)
  }

  /**
   * @brief 处理InitialToMapEvent事件
   * @param event 事件对象
   * @param state_machine 状态机对象
   */
  template <class StateMachine>
  void ProcessInitialToMapEvent(InitialToMapEvent* event,
                                StateMachine* state_machine) {
    UNUSED(event);
    AINFO << "ProcessInitialToMapEvent--";
    local_view_data_ = state_machine->GetMutableLocalViewData();
    BuildLocalView(state_machine->GetMutableLocalView(),
                   state_machine->GetEhpFctOut());
    SetNnpActiveCondations(state_machine->GetMutableLocalView());
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::HDMAP_TYPE);
    state_machine->SetHdmapSubState(cur_sub_state_);
    state_machine->SetState(GetSubStatus(state_machine->GetMutableLocalView()));
    ADEBUG << "constructor is only using hdmap";
    AINFO << "---------force hdmap mode--------";
    // TODO(fml)
  }

 private:
  void SubStateDecider(const std::shared_ptr<LocalView>& local_view);
  bool DealChangeModeConditions(const std::shared_ptr<LocalView>& local_view);
  Status GetSubStatus(const std::shared_ptr<LocalView>& local_view);
  void SetNnpActiveCondations(const std::shared_ptr<LocalView>& local_view);
  bool MissileModeDecider(const std::shared_ptr<LocalView>& local_view);
  functionmanager::HdmapSubState cur_sub_state_ =
      functionmanager::HD_INITIAL_TYPE;
  // functionmanager::HdmapSubState history_sub_state_ =
  //     functionmanager::HD_INITIAL_TYPE;
  // double sub_state_time_{0.0};
  DebounceModule is_ehp_mode_debounce_;
  DebounceModule is_local_mode_debounce_;
};

SETACTIONSTRUCT(BuildMapLocalViewAction, ProcessBuildLocalView);
SETACTIONSTRUCT(MapWithFlagAction, ProcessPerceptionToMapWithFlagEvent);
SETACTIONSTRUCT(MapFromInitialAction, ProcessInitialToMapEvent);

}  // namespace planning
}  // namespace TL
