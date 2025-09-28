/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/
#include <memory>
#include <string>

#include "common/util/message_util.h"
#include "planning/hmi/function_manager.h"

namespace TL {
namespace planning {

FunctionManager::FunctionManager() {
  can_nnp_hmi_ = std::make_unique<CanNnpHmi>();
  can_avp_hmi_ = std::make_unique<CanAvpHmi>();
  can_nnp_hmi_->Init();
  can_avp_hmi_->Init();
}

void FunctionManager::ProcessFctOutput(
    Frame* const frame, const std::shared_ptr<hdmap::PncMap>& pnc_map,
    const std::shared_ptr<DependencyInjector>& injector,
    const std::shared_ptr<ADCTrajectory>& trajectory_pb) {
  if (trajectory_pb == nullptr) {
    return;
  }
  auto* mutable_function_manager_out =
      trajectory_pb->mutable_function_manager_out();
  common::util::FillHeader("FunctionManager", mutable_function_manager_out);
  if (ta_pilot_mode_ == functionmanager::AVP) {
    can_avp_hmi_->ProcessFctOutput(frame, pnc_map, injector, trajectory_pb);
  } else {
    can_nnp_hmi_->ProcessFctOutput(frame, pnc_map, injector, trajectory_pb);
  }
}

void FunctionManager::ProcessFctInput(
    const std::shared_ptr<LocalView>& local_view) {
  if (local_view == nullptr) {
    ta_pilot_mode_ = functionmanager::NO_CONTROL;
    AERROR << "ptr is nullptr";
    return;
  }
  if (!local_view->HasFunctionManagerIn()) {
    ta_pilot_mode_ = functionmanager::NO_CONTROL;
    AERROR << "no FunctionManagerIn.";
    return;
  }

  ta_pilot_mode_ = local_view->GetFunctionManagerIn()->ta_pilot_mode();
  if (ta_pilot_mode_ == functionmanager::AVP) {
    can_avp_hmi_->ProcessFctInput(local_view);
  } else {
    can_nnp_hmi_->ProcessFctInput(local_view);
  }
}

void FunctionManager::SetSimulationFctValue(
    const std::shared_ptr<LocalView>& local_view,
    const functionmanager::FunctionManagerOut* function_manager_out) {
  auto fct_in = std::make_shared<functionmanager::FunctionManagerIn>();
  fct_in->CopyFrom(*local_view->GetFunctionManagerIn());
  const auto& fct_avp_in = fct_in->fct_avp_in();
#ifdef FOR_BAIDU_SIMULATION
  if (avp_sys_mode_ != functionmanager::AvpFctIn::NOSTTYPE &&
      avp_sys_cmd_ != functionmanager::AvpFctIn::NOCMDTYPE) {
    fct_in->mutable_fct_avp_in()->set_sys_mode(avp_sys_mode_);
    fct_in->mutable_fct_avp_in()->set_sys_command(avp_sys_cmd_);
  } else if (fct_avp_in.sys_command() ==
                 functionmanager::AvpFctIn::PARKINCONTROL ||
             fct_avp_in.sys_command() ==
                 functionmanager::AvpFctIn::LEFTPARKOUTCONTROL ||
             fct_avp_in.sys_command() ==
                 functionmanager::AvpFctIn::RIGHTPARKOUTCONTROL ||
             fct_avp_in.sys_command() ==
                 functionmanager::AvpFctIn::FRONTPARKOUTCONTROL ||
             fct_avp_in.sys_command() ==
                 functionmanager::AvpFctIn::BACKPARKOUTCONTROL ||
             fct_avp_in.sys_command() ==
                 functionmanager::AvpFctIn::TBACONTROL ||
             fct_avp_in.sys_command() ==
                 functionmanager::AvpFctIn::NNSCONTROL ||
             fct_avp_in.sys_command() ==
                 functionmanager::AvpFctIn::PARKINPOLIT ||
             fct_avp_in.sys_command() ==
                 functionmanager::AvpFctIn::NTPCONTROL) {
    avp_sys_mode_ = fct_avp_in.sys_mode();
    avp_sys_cmd_ = fct_avp_in.sys_command();
  }
  if (FLAGS_enable_lapa_simulation) {
    fct_in->mutable_fct_avp_in()->set_sys_mode(functionmanager::AvpFctIn::LAPA);
    fct_in->set_ta_pilot_mode(functionmanager::AVP);
  }
#else
  if (fct_avp_in.sys_mode() == functionmanager::AvpFctIn::NOSTTYPE) {
    avp_run_state_ = functionmanager::AvpFctIn::STOP;
  }
#endif
  UpdateAVPState(function_manager_out->avp_fct_out().parking_status(), fct_in);
  local_view->SetFunctionManagerInPtr(fct_in);
}

void FunctionManager::UpdateAVPState(  //NOLINT
    const functionmanager::AvpFctOut::ParkState& parkstatus,
    const std::shared_ptr<functionmanager::FunctionManagerIn>& fct_in) {
  if (avp_run_state_ == functionmanager::AvpFctIn::QUIT) {
    return;
  }
  switch (fct_in->fct_avp_in().sys_mode()) {
    case functionmanager::AvpFctIn::APA:
    case functionmanager::AvpFctIn::DAPA:
    case functionmanager::AvpFctIn::RPA:
    case functionmanager::AvpFctIn::LAPA_MAPPING: {
      switch (parkstatus) {
        case functionmanager::AvpFctOut::RUNNING: {
          if (fct_in->fct_avp_in().sys_command() ==
              functionmanager::AvpFctIn::BRAKECONTROL) {
            avp_run_state_ = functionmanager::AvpFctIn::STRAIGHTBRAKE;
          } else if (fct_in->fct_avp_in().sys_command() ==
                         functionmanager::AvpFctIn::FORWARDCONTROL ||
                     fct_in->fct_avp_in().sys_command() ==
                         functionmanager::AvpFctIn::BACKWARDCONTROL ||
                     fct_in->fct_avp_in().sys_command() ==
                         functionmanager::AvpFctIn::TESTCONTROLMODE) {
            avp_run_state_ = functionmanager::AvpFctIn::STRAIGHTCONTROL;
          } else if (fct_in->fct_avp_in().sys_command() !=
                         functionmanager::AvpFctIn::NOCMDTYPE &&
                     fct_in->fct_avp_in().sys_command() !=
                         functionmanager::AvpFctIn::SYSTEMON) {
            avp_run_state_ = functionmanager::AvpFctIn::PARKING;
          } else {
            avp_run_state_ = functionmanager::AvpFctIn::STOP;
          }
        } break;
        case functionmanager::AvpFctOut::MISSIONFINISHED:
          avp_run_state_ = functionmanager::AvpFctIn::PARKFINISH;
          break;
        case functionmanager::AvpFctOut::PLANNINGFAILED:
        case functionmanager::AvpFctOut::PARKINGNOSPACE:
        case functionmanager::AvpFctOut::COLLISION:
          avp_run_state_ = functionmanager::AvpFctIn::QUIT;
          break;
        case functionmanager::AvpFctOut::WAITOBSTACLE:
          avp_run_state_ = functionmanager::AvpFctIn::PAUSE;
          fct_in->mutable_fct_avp_in()->set_sys_warning_info(
              functionmanager::AvpFctIn::WAIT_OBSTALE_0xA);
          break;
        default:
          break;
      }
    } break;
    case functionmanager::AvpFctIn::TBA: {
      switch (parkstatus) {
        case functionmanager::AvpFctOut::RUNNING:
          avp_run_state_ = functionmanager::AvpFctIn::TBAING;
          break;
        case functionmanager::AvpFctOut::MISSIONFINISHED:
          avp_run_state_ = functionmanager::AvpFctIn::TBAFINISHED;
          break;
        case functionmanager::AvpFctOut::PLANNINGFAILED:
        case functionmanager::AvpFctOut::COLLISION:
          avp_run_state_ = functionmanager::AvpFctIn::QUIT;
          break;
        case functionmanager::AvpFctOut::WAITOBSTACLE:
          avp_run_state_ = functionmanager::AvpFctIn::PAUSE;
          fct_in->mutable_fct_avp_in()->set_sys_warning_info(
              functionmanager::AvpFctIn::WAIT_OBSTALE_0xA);
          break;
        default:
          break;
      }
    } break;
    case functionmanager::AvpFctIn::LAPA: {
      switch (parkstatus) {
        case functionmanager::AvpFctOut::RUNNING:
          if (fct_in->fct_avp_in().sys_run_state() ==
              functionmanager::AvpFctIn::CRUISESTART) {
            avp_run_state_ = fct_in->fct_avp_in().sys_run_state();
          } else if (avp_run_state_ != functionmanager::AvpFctIn::PARKING) {
            avp_run_state_ = functionmanager::AvpFctIn::CRUSING;
          }
          break;
        case functionmanager::AvpFctOut::MISSIONFINISHED:
          avp_run_state_ = functionmanager::AvpFctIn::PARKFINISH;
          break;
        case functionmanager::AvpFctOut::CRUISINGFINISHED:
          avp_run_state_ = functionmanager::AvpFctIn::PARKING;
          break;
        case functionmanager::AvpFctOut::PLANNINGFAILED:
        case functionmanager::AvpFctOut::PARKINGNOSPACE:
        case functionmanager::AvpFctOut::COLLISION:
          avp_run_state_ = functionmanager::AvpFctIn::QUIT;
          break;
        case functionmanager::AvpFctOut::WAITOBSTACLE:
          avp_run_state_ = functionmanager::AvpFctIn::PAUSE;
          fct_in->mutable_fct_avp_in()->set_sys_warning_info(
              functionmanager::AvpFctIn::WAIT_OBSTALE_0xA);
          break;
        default:
          break;
      }
      if (avp_run_state_ == functionmanager::AvpFctIn::PARKING) {
        fct_in->mutable_fct_avp_in()->set_sys_command(
            TL::functionmanager::AvpFctIn::PARKINCONTROL);
      }
    } break;
    case functionmanager::AvpFctIn::NTP: {
      switch (parkstatus) {
        case functionmanager::AvpFctOut::RUNNING:
          avp_run_state_ = functionmanager::AvpFctIn::NTPING;
          break;
        case functionmanager::AvpFctOut::MISSIONFINISHED:
          avp_run_state_ = functionmanager::AvpFctIn::NTPFINISHED;
          break;
        case functionmanager::AvpFctOut::PLANNINGFAILED:
          if (avp_run_state_ == functionmanager::AvpFctIn::NTPING) {
            avp_run_state_ = functionmanager::AvpFctIn::QUIT;
          }
          break;
        case functionmanager::AvpFctOut::PARKINGFINISHED:
          avp_run_state_ = functionmanager::AvpFctIn::CRUSING;
          break;
        case functionmanager::AvpFctOut::WAITOBSTACLE:
          avp_run_state_ = functionmanager::AvpFctIn::PAUSE;
          fct_in->mutable_fct_avp_in()->set_sys_warning_info(
              functionmanager::AvpFctIn::WAIT_OBSTALE_0xA);
          break;
        default:
          break;
      }
      if (avp_run_state_ == functionmanager::AvpFctIn::NTPING ||
          avp_run_state_ == functionmanager::AvpFctIn::CRUSING) {
        fct_in->mutable_fct_avp_in()->set_sys_command(
            TL::functionmanager::AvpFctIn::NTPCONTROL);
      }
    } break;
    case functionmanager::AvpFctIn::ISM: {
      switch (parkstatus) {
        case functionmanager::AvpFctOut::RUNNING:
          avp_run_state_ = functionmanager::AvpFctIn::NNSING;
          break;
        case functionmanager::AvpFctOut::MISSIONFINISHED:
          avp_run_state_ = functionmanager::AvpFctIn::NNSFINISHED;
          break;

        case functionmanager::AvpFctOut::PLANNINGFAILED:
        case functionmanager::AvpFctOut::COLLISION:
        case functionmanager::AvpFctOut::PARKINGNOSPACE:
          avp_run_state_ = functionmanager::AvpFctIn::QUIT;
          break;
        case functionmanager::AvpFctOut::WAITOBSTACLE:
          avp_run_state_ = functionmanager::AvpFctIn::PAUSE;
          fct_in->mutable_fct_avp_in()->set_sys_warning_info(
              functionmanager::AvpFctIn::WAIT_OBSTALE_0xA);
          break;
        default:
          break;
      }
      if (avp_run_state_ == functionmanager::AvpFctIn::NNSING ||
          avp_run_state_ == functionmanager::AvpFctIn::CRUSING) {
        fct_in->mutable_fct_avp_in()->set_sys_command(
            TL::functionmanager::AvpFctIn::NNSCONTROL);
      }
    } break;
    case functionmanager::AvpFctIn::LOCALIZATION:
    case functionmanager::AvpFctIn::LOCALIZATION_BACKGROUND: {
      avp_run_state_ = fct_in->fct_avp_in().sys_run_state();
    } break;
    default:
      break;
  }
  fct_in->mutable_fct_avp_in()->set_sys_run_state(avp_run_state_);
}
}  // namespace planning
}  // namespace TL
