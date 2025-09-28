//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/configs/vehicle_config_helper.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/util/message_util.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/util/util.h"
#include "planning/localview/local_view.h"
#include "planning/localview/state_machine/base_state.h"
#include "planning/localview/state_machine/event.h"
#include "proto/common/types.pb.h"
#include "proto/fsm/avp_fct.pb.h"
#include "proto/map/map.pb.h"
#include "proto/routing/poi.pb.h"

namespace TL {
namespace planning {
using TL::common::ErrorCode;
using TL::common::Status;
using ParkingPathPointSetPro =
    google::protobuf::RepeatedPtrField<perception::ParkingPathPoint>;

class ParkingLanelineState : public BaseState {
 public:
  ParkingLanelineState() = default;
  ~ParkingLanelineState() override = default;

  bool Init() override;
  bool ReadMap(const std::shared_ptr<LocalView>& local_view,
               TL::hdmap::Map* hd_map_ptr);

  /**
   * @brief creat parking lane map
   *
   * @param local_view local view
   * @param hd_map_ptr hd map msg pointer
   * @return std::string error msg
   */
  std::string CreatMap(const std::shared_ptr<LocalView>& local_view,
                       TL::hdmap::Map* hd_map_ptr);

  /**
   * @brief Get the Parking Lot Array World object
   *
   * @param local_view local view
   * @param parking_spot_enu parkint lot array in enu coordinate
   * @return std::string error msg
   */
  std::string GetParkingLotArrayWorld(
      const std::shared_ptr<LocalView>& local_view,
      std::vector<common::math::Vec2d>* parking_spot_enu);

  bool SetRoutingAndRoad(TL::routing::RoutingResponse* inrouting,
                         TL::hdmap::Map* hd_map);
  /**
   * @brief
   *
   * @param local_view
   * @return Status Errorcode and error message
   */
  Status BuildLocalView(const std::shared_ptr<LocalView>& local_view) override;

  /**
   * @brief 
   * 
   * @param local_view 
   * @return true 
   * @return false 
   */
  bool ReadParkingLotInfoFromConf(const std::shared_ptr<LocalView>& local_view);

  /**
   * @brief 
   * 
   * @param local_view 
   * @return Status 
   */
  Status ProcessBaiduSimulations(const std::shared_ptr<LocalView>& local_view);

  /**
   * @brief 
   * 
   * @param local_view 
   * @return Status 
   */
  Status ProcessBaiduRecordReplay(const std::shared_ptr<LocalView>& local_view);

  /**
   * @brief 
   * 
   * @param local_view 
   * @return Status 
   */
  Status ProcessRealCar(const std::shared_ptr<LocalView>& local_view);

  /**
   * @brief Set the Parking Lot 2 Localview object
   * 
   * @param local_view 
   * @param error_msg 
   * @return true 
   * @return false 
   */
  bool SetParkingLot2Localview(const std::shared_ptr<LocalView>& local_view,
                               std::string* error_msg);
  /**
   * @brief Set the Parking Lot object
   * 
   * @param local_view 
   * @return true 
   * @return false 
   */
  bool SetParkingLot(const std::shared_ptr<LocalView>& local_view);

  /**
   * @brief Get the Nearest Lane object
   * 
   * @param map_ptr 
   * @param point 
   * @param heading 
   * @param nearest_lane 
   * @param nearest_s 
   * @param nearest_l 
   * @return true 
   * @return false 
   */
  bool GetNearestLane(const std::shared_ptr<hdmap::HDMap>& map_ptr,
                      const common::PointENU& point, double heading,
                      hdmap::LaneInfoConstPtr* nearest_lane, double* nearest_s,
                      double* nearest_l);

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
                             const Vec2d& point,
                             hdmap::LaneInfoConstPtr* nearest_lane,
                             double* nearest_s, double* nearest_l);

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
          functionmanager::MachineStateType::APA_TYPE);
      state_machine->SetState(Status(ErrorCode::LOCALVIEW_FSM_APA_ERROR, msg));
      return;
    }

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
    if (fmi->fct_avp_in().sys_mode() != functionmanager::AvpFctIn::APA &&
        fmi->fct_avp_in().sys_mode() != functionmanager::AvpFctIn::RPA &&
        fmi->fct_avp_in().sys_mode() != functionmanager::AvpFctIn::DAPA &&
        fmi->fct_avp_in().sys_mode() !=
            functionmanager::AvpFctIn::LAPA_MAPPING) {
      ParkingToInitialEvent ptie;
      state_machine->SetHistoryMachineStateType(
          functionmanager::MachineStateType::APA_TYPE);
      state_machine->process_event(ptie);
      AINFO << "---------Parking to Initial--------";
      return;
    }
    switch (fmi->fct_avp_in().sys_run_state()) {
      case functionmanager::AvpFctIn::STOP:
      case functionmanager::AvpFctIn::ERROR: {
        ParkingToInitialEvent ptie;
        state_machine->SetHistoryMachineStateType(
            functionmanager::MachineStateType::APA_TYPE);
        state_machine->process_event(ptie);
        AINFO << "---------Parking to Initial--------";
        return;
      } break;
      case functionmanager::AvpFctIn::QUIT: {
        const std::string msg = {
            "AVP Quit info: " +
            functionmanager::AvpFctIn_WarningInfoErrorType_Name(
                fmi->fct_avp_in().sys_warning_info())};
        AERROR << msg;
        state_machine->SetMachineStateType(
            functionmanager::MachineStateType::APA_TYPE);
        state_machine->SetState(
            Status(ErrorCode::LOCALVIEW_FSM_INITIAL_ERROR, msg));
        return;
      } break;
      case functionmanager::AvpFctIn::PARKSTART: {
        if (fmi->fct_avp_in().sys_command() ==
            functionmanager::AvpFctIn::BRAKECONTROL) {
          break;
        }
        if (!has_opt_park_lot || !is_vehicle_standstill || is_vehicle_in_park) {
          ParkingToInitialEvent ptie;
          state_machine->SetHistoryMachineStateType(
              functionmanager::MachineStateType::APA_TYPE);
          state_machine->process_event(ptie);
          AINFO << "---------Parking to Initial--------";
          return;
        }
      } break;
      default:
        break;
    }

    auto status = BuildLocalView(state_machine->GetMutableLocalView());
    state_machine->SetState(status);
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::APA_TYPE);
    AINFO << "---------parking mode-------- status = " << status.ToString();
  }

  template <typename StateMachine>
  void ProcessInitialToParkingEvent(InitialToParkingEvent* event,
                                    StateMachine* state_machine) {
    UNUSED(event);
    ACHECK(state_machine);
    parking_lot_array_enu_ptr_ = nullptr;
    auto status = BuildLocalView(state_machine->GetMutableLocalView());
    state_machine->SetState(status);
    state_machine->SetMachineStateType(
        functionmanager::MachineStateType::APA_TYPE);
    AINFO << "---------parking mode from initial--------  status = "
          << status.ToString();
  }

 private:
  TL::navigation_hdmap::MapMsg parking_hdmap_;
  routing::RoutingResponse routing_response_;
  std::shared_ptr<TL::perception::ParkingLotOutArray>
      parking_lot_array_enu_ptr_ = nullptr;
  std::pair<double, double> start_and_end_s_{0, 0};
  common::math::Vec2d parkinglot_center_;
};

// NOLINTBEGIN
SETACTIONSTRUCT(BuildParkingLocalViewAction, ProcessBuildLocalView);
SETACTIONSTRUCT(ParkingFromInitialAction, ProcessInitialToParkingEvent);
// NOLINTEND
}  // namespace planning
}  // namespace TL
