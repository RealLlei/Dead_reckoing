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
 * @LastEditors: Xu Xin
 *****************************************************************************/

#pragma once

#include <memory>

#include "common/status/status.h"
#include "planning/common/planning_gflags.h"
#include "planning/localview/state_machine/base_state.h"

namespace TL {
namespace planning {
using TL::common::ErrorCode;
using TL::common::Status;

class HDMapPerceptionFusedState : public BaseState {
 public:
  HDMapPerceptionFusedState() = default;
  ~HDMapPerceptionFusedState() override = default;

  bool Init() override;
  /**
  * @brief 
  * 
  * @param local_view 
  * @return Status error code and error message
  */
  Status BuildLocalView(const std::shared_ptr<LocalView>& local_view) override;
  Status BuildLocalView(const std::shared_ptr<LocalView>& local_view,
                        const routing::ChangeLaneType change_lane_type);

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
      const auto fmi = state_machine->GetLocalView().GetFunctionManagerIn();
      auto nnp_sys_state = fmi->fct_nnp_in().nnp_sysstate();
      if ((nnp_sys_state == functionmanager::NNPSysState::NNPS_OFF ||
           nnp_sys_state == functionmanager::NNPSysState::NNPS_PASSIVE ||
           nnp_sys_state ==
               functionmanager::NNPSysState::NNPS_FAIL_TmpUnavabl ||
           nnp_sys_state == functionmanager::NNPSysState::NNPS_FAIL_ServReq) &&
          fmi->fct_avp_in().sys_mode() !=
              functionmanager::AvpFctIn_StateType_NOSTTYPE) {
        AINFO << "has no NNPSysState and has avp state, so return init state "
                 "from fused mode";
        NnpToInitialEvent nnp_to_initialstate_event;
        state_machine->process_event(nnp_to_initialstate_event);
        return;
      }
    }
    if (state_machine->GetMutableLocalView()
            ->GetFunctionManagerOut()
            ->hdmap_status() &&
        state_machine->GetMutableLocalView()
            ->GetFunctionManagerOut()
            ->perception_status()) {
      // BuildLocalView(state_machine->GetMutableLocalView(),
      //                state_machine->GetChangeLaneTypeFused());

      state_machine->SetMachineStateType(
          functionmanager::MachineStateType::HDMAP_PERCEPTION_FUSION);
      state_machine->SetState(Status::OK());
      AINFO << "----fused mode --------";
      return;
    } else {
      FusionToPerceptionEvent fusion_to_perception_event;
      state_machine->SetHistoryMachineStateType(
          functionmanager::MachineStateType::HDMAP_PERCEPTION_FUSION);
      state_machine->process_event(fusion_to_perception_event);
    }
  }

  /**
   * @brief 处理PerceptionToFusionEvent事件
   * @param event 事件对象
   * @param state_machine 状态机对象
   */
  template <typename StateMachine>
  void ProcessPerceptionToFusionEvent(PerceptionToFusionEvent* event,
                                      StateMachine* state_machine) {
    UNUSED(event);
    // BuildLocalView(state_machine->GetMutableLocalView(),
    //                state_machine->GetChangeLaneTypeFused());
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::HDMAP_PERCEPTION_FUSION);
    state_machine->SetState(Status::OK());
    AINFO << "----fused mode from Perception--------";
    return;
  }

  /**
   * @brief 处理InitialToFusionEvent事件
   * @param event 事件对象
   * @param state_machine 状态机对象
   */
  template <typename StateMachine>
  void ProcessInitialToFusionEvent(InitialToFusionEvent* event,
                                   StateMachine* state_machine) {
    UNUSED(event);
    // BuildLocalView(state_machine->GetMutableLocalView(),
    //                state_machine->GetChangeLaneTypeFused());
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::HDMAP_PERCEPTION_FUSION);
    state_machine->SetState(Status::OK());
    AINFO << "----fused mode from initial_state--------";
    return;
  }

 private:
  void JudgePassage(hdmap::Map* hd_map,
                    routing::RoutingResponse* routing_response,
                    routing::ChangeLaneType chgtype);
  void AddOtherPassage(hdmap::Map* hd_map,
                       routing::RoutingResponse* routing_response, int index);
  routing::ChangeLaneType current_change_lane_type_ =
      routing::ChangeLaneType::FORWARD;
};

SETACTIONSTRUCT(BuildFusedLocalViewAction, ProcessBuildLocalView);
SETACTIONSTRUCT(FusionFromPerceptionAction, ProcessPerceptionToFusionEvent);
SETACTIONSTRUCT(FusionFromInitialAction, ProcessInitialToFusionEvent);

}  // namespace planning
}  // namespace TL
