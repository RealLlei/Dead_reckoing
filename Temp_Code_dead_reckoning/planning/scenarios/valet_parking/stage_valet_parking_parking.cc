/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  stage_valet_parking_parking.cc
 */

#include "planning/scenarios/valet_parking/stage_valet_parking_parking.h"

#include <string>
#include <utility>

#include "common/file/log.h"
#include "planning/common/open_space_info.h"
#include "planning/tasks/optimizers/open_space_speed_optimizer/open_space_speed_optimizer.h"

#include "proto/fsm/avp_fct.pb.h"
#include "proto/planning/planning_internal.pb.h"
#include "proto/planning/planning_status.pb.h"

namespace TL {
namespace planning {
namespace scenario {
namespace valet_parking {

using TL::common::ErrorCode;
using TL::common::Status;
using TL::functionmanager::AvpFctOut;
using TL::planning::AVPStatus;
using typename TL::planning_internal::AvpSpeedPlanCollisionInfo;

std::pair<Stage::StageStatus, Status> ValetParkingStageParking::Process(
    const common::TrajectoryPoint& /*planning_init_point*/, Frame* frame) {
  // Open space planning doesn't use planning_init_point from upstream because
  // of different stitching strategy
  auto* mutable_avp_fct_out = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_function_manager_out()
                                  ->mutable_avp_fct_out();
  mutable_avp_fct_out->set_stage_type(functionmanager::AvpFctOut::PARKING);
  const auto& vehicle_state = *(injector_->vehicle_state());
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  if (next_stage_ == ScenarioStatus::NO_STAGE) {
    GeneratePauseTrajectory(frame);
    frame->SetTargetGear(soc::Chassis::GEAR_PARKING);
    mutable_avp_fct_out->set_parking_status(
        TL::functionmanager::AvpFctOut::MISSIONFINISHED);
    ADEBUG << "enter scenario finish condition";
    return std::make_pair(FinishScenario(), Status::OK());
  }

  if (IsParkingBrakeCondition(
          frame->local_view().GetFunctionManagerIn()->fct_avp_in())) {
    AINFO << "Parking enter pause condition";
    GeneratePauseTrajectory(frame);
    return std::make_pair(StageStatus::RUNNING, Status::OK());
  }
  SetParkingType(frame->local_view().GetFunctionManagerIn()->fct_avp_in());
  if (injector_->planning_context()
          ->planning_status()
          .avp_status()
          .parking_type() == TL::planning::AVPStatus::NOSTATE) {
    const std::string msg = "Parking not start";
    AERROR << msg;
    frame->SetTargetGear(vehicle_state.gear());
    return std::make_pair(
        StageStatus::ERROR,
        Status(ErrorCode::CORE_PLANNING_PARKINGPARKINGSTAGE_ERROR, msg));
  }
  const auto ret = ExecuteTaskOnOpenSpace(frame);
  if (!ret.ok()) {
    AERROR << "ValetParkingStageParking planning error";
    frame->SetTargetGear(vehicle_state.gear());
    return std::make_pair(StageStatus::ERROR, ret);
  }

  const auto speed_task_interactive_stage =
      frame->open_space_info().speed_task_interactive_stage();
  switch (speed_task_interactive_stage) {
    case AvpSpeedPlanCollisionInfo::WAITOBSTACLE:
      mutable_avp_fct_out->set_parking_status(AvpFctOut::WAITOBSTACLE);
      break;
    case AvpSpeedPlanCollisionInfo::RUNNING:
      mutable_avp_fct_out->set_parking_status(AvpFctOut::RUNNING);
      break;
    default:
      mutable_avp_fct_out->set_parking_status(AvpFctOut::RUNNING);
  }

  if (IsReadyToFinishStage(frame)) {
    AINFO << "ValetParkingStageParking parking finish";
    const auto& fct_avp_in =
        frame->local_view().GetFunctionManagerIn()->fct_avp_in();
    switch (fct_avp_in.sys_mode()) {
      case functionmanager::AvpFctIn::ISM:
      case functionmanager::AvpFctIn::NTP:
        next_stage_ = ScenarioStatus::VALET_PARKING_CRUISE;
        return std::make_pair(StageStatus::FINISHED, Status::OK());
        break;
      case functionmanager::AvpFctIn::APA:
      case functionmanager::AvpFctIn::RPA:
      case functionmanager::AvpFctIn::DAPA:
      case functionmanager::AvpFctIn::LAPA_MAPPING:
      case functionmanager::AvpFctIn::LAPA:
        return std::make_pair(FinishScenario(), Status::OK());
        break;
      default:
        break;
    }
  }
  return std::make_pair(StageStatus::RUNNING, ret);
}

bool ValetParkingStageParking::IsParkingBrakeCondition(
    const functionmanager::AvpFctIn& avp_in) {
  if (avp_in.sys_run_state() ==
      functionmanager::AvpFctIn_SysRunState_STRAIGHTBRAKE) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_avp_status()
        ->set_parking_type(TL::planning::AVPStatus::NOSTATE);
    return true;
  }

  return avp_in.sys_run_state() ==
             functionmanager::AvpFctIn_SysRunState_PAUSE &&
         avp_in.sys_warning_info() !=
             functionmanager::AvpFctIn::WAIT_OBSTALE_0xA;
}

void ValetParkingStageParking::SetParkingType(
    const functionmanager::AvpFctIn& avp_in) {
  auto sys_command = avp_in.sys_command();
  auto* mutable_avp_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_avp_status();
  const bool has_parking_type = injector_->planning_context()
                                    ->planning_status()
                                    .avp_status()
                                    .has_parking_type();
  auto parking_type = has_parking_type ? injector_->planning_context()
                                             ->planning_status()
                                             .avp_status()
                                             .parking_type()
                                       : planning::AVPStatus::NOSTATE;
  switch (sys_command) {
    case functionmanager::AvpFctIn::SYSTEMON:
    case functionmanager::AvpFctIn::PARKINCONTROL:
      parking_type = planning::AVPStatus::PARKING_IN;
      break;
    case functionmanager::AvpFctIn::LEFTPARKOUTCONTROL:
      parking_type = planning::AVPStatus::PARKING_OUT_LEFT;
      break;
    case functionmanager::AvpFctIn::RIGHTPARKOUTCONTROL:
      parking_type = planning::AVPStatus::PARKING_OUT_RIGHT;
      break;
    case functionmanager::AvpFctIn::FRONTPARKOUTCONTROL:
      parking_type = planning::AVPStatus::PARKING_OUT_FRONT;
      break;
    case functionmanager::AvpFctIn::BACKPARKOUTCONTROL:
      parking_type = planning::AVPStatus::PARKING_OUT_BACK;
      break;
    case functionmanager::AvpFctIn::FORWARDCONTROL:
      parking_type = planning::AVPStatus::DIRECT_FORWARD;
      break;
    case functionmanager::AvpFctIn::TESTCONTROLMODE:
      parking_type = planning::AVPStatus::TEST_CONTROL_MODE;
      break;
    case functionmanager::AvpFctIn::BACKWARDCONTROL:
      parking_type = planning::AVPStatus::DIRECT_BACKWARD;
      break;
    case functionmanager::AvpFctIn::NNSCONTROL:
    case functionmanager::AvpFctIn::NTPCONTROL:
      parking_type = planning::AVPStatus::PARKING_OUT_NNS;
      break;
    case functionmanager::AvpFctIn::BRAKECONTROL: {
      if (avp_in.sys_mode() == functionmanager::AvpFctIn::RPA &&
          avp_in.sys_run_state() == functionmanager::AvpFctIn::PARKSTART) {
        parking_type = planning::AVPStatus::PARKING_IN;
      }
      break;
    }
    default:
      break;
  }
  mutable_avp_status->set_parking_type(parking_type);
  AINFO << "parking_type: "
        << TL::planning::AVPStatus::ParkingType_Name(parking_type);
}

bool ValetParkingStageParking::IsReadyToFinishStage(Frame* const frame) {
  const AVPStatus::ParkingType parking_type = injector_->planning_context()
                                                  ->planning_status()
                                                  .avp_status()
                                                  .parking_type();
  auto* avp_fct_out = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_function_manager_out()
                          ->mutable_avp_fct_out();
  if (!is_stage_over_) {
    switch (parking_type) {
      case TL::planning::AVPStatus::PARKING_OUT_NNS: {
        if (frame->open_space_info().destination_reached()) {
          is_stage_over_ = true;
          AINFO << "parking finished, ready to switch to cruise";
        }
        break;
      }
      case TL::planning::AVPStatus::PARKING_IN:
      case TL::planning::AVPStatus::PARKING_OUT_LEFT:
      case TL::planning::AVPStatus::PARKING_OUT_RIGHT:
      case TL::planning::AVPStatus::PARKING_OUT_FRONT:
      case TL::planning::AVPStatus::PARKING_OUT_BACK:
      case TL::planning::AVPStatus::DIRECT_FORWARD:
      case TL::planning::AVPStatus::DIRECT_BACKWARD: {
        if (frame->IsVehicleStandStill() &&
            frame->open_space_info().destination_reached()) {
          is_stage_over_ = true;
        }
        break;
      }
      default:
        break;
    }
  }

  if (is_stage_over_) {
    if (TL::planning::AVPStatus::PARKING_OUT_NNS == parking_type) {
      avp_fct_out->set_parking_status(
          TL::functionmanager::AvpFctOut::PARKINGFINISHED);
    } else {
      avp_fct_out->set_parking_status(
          TL::functionmanager::AvpFctOut::MISSIONFINISHED);
      AINFO << "avp mission finished";
    }
  }

  return is_stage_over_;
}

}  // namespace valet_parking
}  // namespace scenario
}  // namespace planning
}  // namespace TL
