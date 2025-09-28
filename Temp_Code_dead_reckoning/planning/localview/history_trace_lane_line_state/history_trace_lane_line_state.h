//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#pragma once
#include <memory>
#include <string>
#include <vector>

#include "common/status/status.h"
#include "planning/common/planning_gflags.h"
#include "planning/localview/local_view.h"
#include "planning/localview/state_machine/base_state.h"
#include "planning/localview/state_machine/event.h"

#include "planning/proto/ipopt_pos_optimize_smoother_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/map/map.pb.h"

namespace TL::planning {
using TL::common::ErrorCode;
using TL::common::Status;

class HistoryTraceLanelineState : public BaseState {
  using ParkingPathPointSetPro =
      google::protobuf::RepeatedPtrField<TL::perception::ParkingPathPoint>;

 public:
  HistoryTraceLanelineState() {
    ACHECK(TL::common::GetProtoFromFile(
        FLAGS_ipopt_pos_optimize_smoother_config_file, &smoother_config_))
        << "Failed to ipopt_collision_free_smoother_config: "
        << FLAGS_ipopt_pos_optimize_smoother_config_file;
  }

  ~HistoryTraceLanelineState() override = default;

  bool Init() override;

  bool HistoryTraceCreatMap(const LocalView&, hdmap::Map* hd_map,
                            std::string* error_msg);

  bool SetRoutingAndRoad(TL::routing::RoutingResponse* inrouting,
                         TL::hdmap::Map* hd_map);

  bool ProcessLocalSim(const LocalView&, TL::hdmap::Map* hd_map,
                       std::string* error_msg);

  bool ProcessBaiduSim(const LocalView&, TL::hdmap::Map* hd_map,
                       std::string* error_msg);

  bool ProcessRealCar(const LocalView&, TL::hdmap::Map* hd_map,
                      std::string* error_msg);

  bool ConstructLane(const LocalView&, TL::hdmap::Map* hd_map,
                     std::string* error_msg);

  /**
   * @brief
   *
   * @param local_view
   * @return Status error code and error message
   */
  Status BuildLocalView(const std::shared_ptr<LocalView>& local_view) override;

  /**
   * @brief 
   * 
   * @param lanes 
   * @param point 
   * @param nearest_lane 
   * @param nearest_s 
   * @param nearest_l 
   * @return true 
   * @return false 
   */
  static bool IsLaneForPoint(const std::vector<hdmap::LaneInfoConstPtr>& lanes,
                             const common::math::Vec2d& point,
                             hdmap::LaneInfoConstPtr* nearest_lane,
                             double* nearest_s, double* nearest_l);
  /**
   * @brief 
   * 
   * @param map_ptr 
   * @param traced_paths 
   * @param error_msg 
   * @return true 
   * @return false 
   */
  static bool GenerateTracedPathFromBaiduMap(
      const std::shared_ptr<hdmap::HDMap>& map_ptr,
      ParkingPathPointSetPro* traced_paths, std::string* error_msg);

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
          functionmanager::MachineStateType::HISTORY_TRACE_TYPE);
      state_machine->SetState(
          Status(ErrorCode::LOCALVIEW_FSM_HISTORYTRACE_ERROR, msg));
      return;
    }

    const auto fmi = state_machine->GetLocalView().GetFunctionManagerIn();
    if (fmi->fct_avp_in().sys_mode() != functionmanager::AvpFctIn::TBA) {
      HistoryTraceToInitialEvent htie;
      state_machine->SetHistoryMachineStateType(
          functionmanager::MachineStateType::HISTORY_TRACE_TYPE);
      state_machine->process_event(htie);
      AINFO << "---------HistoryTrace to Initial--------";
      return;
    }
    switch (fmi->fct_avp_in().sys_run_state()) {
      case functionmanager::AvpFctIn::STOP:
      case functionmanager::AvpFctIn::ERROR: {
        HistoryTraceToInitialEvent htie;
        state_machine->SetHistoryMachineStateType(
            functionmanager::MachineStateType::HISTORY_TRACE_TYPE);
        state_machine->process_event(htie);
        AINFO << "---------HistoryTrace to Initial--------";
        return;
      } break;
      case functionmanager::AvpFctIn::QUIT: {
        const std::string msg = {
            "AVP Quit info: " +
            functionmanager::AvpFctIn_WarningInfoErrorType_Name(
                fmi->fct_avp_in().sys_warning_info())};
        AERROR << msg;
        state_machine->SetMachineStateType(
            functionmanager::MachineStateType::HISTORY_TRACE_TYPE);
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
        functionmanager::MachineStateType::HISTORY_TRACE_TYPE);
    ADEBUG << "-------- history trace mode-------- status = "
           << status.ToString();
  }

  template <typename StateMachine>
  void ProcessInitialToHistoryTraceEvent(InitialToHistoryTraceEvent* event,
                                         StateMachine* state_machine) {
    UNUSED(event);
    ACHECK(state_machine);
    global_path_.clear();
    tba_map_ptr_ = nullptr;
    routing_response_.Clear();
    auto status = BuildLocalView(state_machine->GetMutableLocalView());
    state_machine->SetState(status);
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::HISTORY_TRACE_TYPE);
    AINFO << "---------history trace mode from initial--------  status = "
          << status.ToString();
  }

 private:
  /**
   * @brief GlobalPathGenerator
   *
   * @param traced_path the history trace path
   * @return traced path is vaild or not
   */
  bool GlobalPathGenerator(const ParkingPathPointSetPro& traced_path,
                           std::string* error_msg);

  IpoptPosOptimizeSmootherConfig smoother_config_;
  std::vector<common::PathPoint> global_path_;
  std::shared_ptr<TL::navigation_hdmap::MapMsg> tba_map_ptr_;
  routing::RoutingResponse routing_response_;
};

// NOLINTBEGIN
SETACTIONSTRUCT(BuildHistoryTraceLocalViewAction, ProcessBuildLocalView);
SETACTIONSTRUCT(HistoryTraceFromInitialAction,
                ProcessInitialToHistoryTraceEvent);
// NOLINTEND
}  // namespace TL::planning
