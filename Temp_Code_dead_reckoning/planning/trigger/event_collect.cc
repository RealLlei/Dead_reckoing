/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/trigger/event_collect.h"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include "common/math/linear_interpolation.h"
#include "common/time/clock.h"
#include "common/util/util.h"
#include "planning/common/planning_gflags.h"
#include "planning/trigger/metric_collect.h"
#include "proto/fsm/avp_fct.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

namespace {
constexpr double kBaseSpeed = 3.6;
}  // namespace

using TL::common::Clock;

// NOLINTBEGIN
const std::unordered_set<functionmanager::AvpFctIn::WarningInfoErrorType>
    EventCollect::valid_avp_warininginfo_set_ = {
        functionmanager::AvpFctIn::GEAR_SHIFT_OVER_COUNTER_0x11,
        functionmanager::AvpFctIn::PASUE_OVER_COUNTER_0xB,
        functionmanager::AvpFctIn::ENVIRONMENT_ERROR_0x2F,
        functionmanager::AvpFctIn::SYSTEM_QUIT_0x1B,
        functionmanager::AvpFctIn::PLANNING_ERROR_0xE,
        functionmanager::AvpFctIn::PARKING_SPACE_ERROR_0x17,
        functionmanager::AvpFctIn::NTP_PARKING_ENVIRONMENTERROR_0x5D,
        functionmanager::AvpFctIn::NTP_SYSFAULT_0x4B,
        functionmanager::AvpFctIn::RELATED_SYSTEM_ERROR_0xD,
        functionmanager::AvpFctIn::NTP_RELATEDSYSTEMERROR_0x4C,
        functionmanager::AvpFctIn::VEHICLE_CRASHED_0x2B,
        functionmanager::AvpFctIn::NTP_VEHICLE_CRASHED_0x59,
        functionmanager::AvpFctIn::UNINTENDED_ERROR_0x3A,
        functionmanager::AvpFctIn::NTP_PARKING_HANDSHAKEOVERTIME_0x5F,
        functionmanager::AvpFctIn::NTP_PARKING_CRUSINGERROR_0x60,
        functionmanager::AvpFctIn::PAUSE_OVER_TIME_0xC,
        functionmanager::AvpFctIn::NTP_PARKING_DO_NOT_ACCELARATION_0X4E,
        functionmanager::AvpFctIn::NTP_PARKING_RELEASE_BRAKE_TOCONTINUE_0X54,
        functionmanager::AvpFctIn::NTP_PARKING_LATOVERRIDE_0x49,
        functionmanager::AvpFctIn::NTP_PARKING_EPBINTERVENE_0x58,
        functionmanager::AvpFctIn::NTP_PARKING_GEARINTERVENE_0x4A,
        functionmanager::AvpFctIn::DRIVER_INTERVENTION_0x12,
        functionmanager::AvpFctIn::PARKING_OVER_SPEED_0xF,
        functionmanager::AvpFctIn::NTP_PARKING_OVERSPEED_0x5A,
        functionmanager::AvpFctIn::SLOPE_OVER_RANGE_0x14,
        functionmanager::AvpFctIn::NTP_PARKIG_OVERSLOPE_QUIT_0X5B,
        functionmanager::AvpFctIn::VEHICLE_BLOCKED_0x2C,
        functionmanager::AvpFctIn::NTP_PARKING_VEHICLEBLOCKED_0x5C,
        functionmanager::AvpFctIn::NTP_Cruising_PAUSE_OVER_TIME_0x67};

// NOLINTEND

void EventCollect::Init(ADCTrajectory* const ptr_trajectory_pb) {
  ptr_trajectory_pb_ = ptr_trajectory_pb;
  emergency_ids_ = {2003, 2005, 2006, 2007, 2008, 2013, 2016, 2020, 2021,
                    2022, 2025, 2027, 2028, 2029, 2031, 2032, 2045, 2046};
}

// update trigger event from localview
void EventCollect::ProcessEventCollect(
    const std::shared_ptr<LocalView>& localview,
    ADCTrajectory* const ptr_trajectory_pb) {
  if (ptr_trajectory_pb == nullptr || localview == nullptr) {
    AERROR << "prev_trajectory is nullptr";
    return;
  }
  Init(ptr_trajectory_pb);
  is_ok_trigger_config_ = ValidNnpTriggerConfig(localview);
  if (UpdateCtrlErrTrigger(localview)) {
    return;
  }
  UpdateNnpTrigger(localview);
  UpdateAvpTrigger(localview);
}

bool EventCollect::ValidNnpTriggerConfig(  //NOLINT
    const std::shared_ptr<LocalView>& localview) {
  if (localview == nullptr || !localview->HasFunctionManagerIn()) {
    return false;
  }
  const auto& fct_in = localview->GetFunctionManagerIn();
  if (fct_in->has_trigger_config() &&
      !common::util::IsProtoEqual(fct_in->trigger_config(), trigger_config_)) {
    trigger_config_.CopyFrom(fct_in->trigger_config());
    emerger_steer_speeds_.clear();
    emerger_steer_rates_.clear();
    for (const auto& emerger_steer :
         trigger_config_.nnp_config().emerger_steer_config()) {
      emerger_steer_speeds_.emplace_back(emerger_steer.emerger_steer_speed());
      emerger_steer_rates_.emplace_back(emerger_steer.emerger_steer_rate());
    }
  }
  if (trigger_config_.has_nnp_config() && trigger_config_.has_avp_config()) {
    const auto& nnp_config = trigger_config_.nnp_config();
    return (nnp_config.has_collision_config() &&
            nnp_config.collision_config().has_collision_acc() &&
            nnp_config.collision_config().has_collision_speed() &&
            nnp_config.has_emerger_brake_config() &&
            nnp_config.emerger_brake_config().has_emerger_brake_speed() &&
            nnp_config.emerger_brake_config().has_emerger_brake_acc() &&
            nnp_config.emerger_steer_config_size() >= 4 &&
            nnp_config.has_lat_ctrl_err() && nnp_config.has_lon_ctrl_err() &&
            nnp_config.rapid_acc_config_size() >= 3);
  }
  return false;
}

bool EventCollect::UpdateCtrlErrTrigger(
    const std::shared_ptr<LocalView>& localview) {
  return UpdateCtrlFollowErrTrigger(localview) ||
         UpdateCtrlCommunicationErrTrigger(localview) ||
         UpdateCtrlInputCheckErrTrigger(localview);
}

bool EventCollect::UpdateCtrlFollowErrTrigger(
    const std::shared_ptr<LocalView>& localview) {
  if (localview == nullptr || !localview->HasFunctionManagerIn() ||
      !localview->HasADCTrajectory()) {
    return false;
  }
  const auto& fct_in = localview->GetADCTrajectory()->function_manager_in();
  std::bitset<32> ctrlfollowerr(fct_in.fct_2_soc_tbd_u32_05());
  ctrlfollowerr &= 0x1F000000;  // ctrlerr bit0~bit4
  if (ctrlfollowerr.any()) {
    AddEventTriger(EXCESSIVE_DEVIATION_TAKEOVER);
    return true;
  }
  return false;
}

bool EventCollect::UpdateCtrlCommunicationErrTrigger(
    const std::shared_ptr<LocalView>& localview) {
  if (localview == nullptr || !localview->HasFunctionManagerIn() ||
      !localview->HasADCTrajectory()) {
    return false;
  }
  const auto& fct_in = localview->GetADCTrajectory()->function_manager_in();
  std::bitset<32> ctrlcommunicationerr(fct_in.fct_2_soc_tbd_u32_05());
  ctrlcommunicationerr &= 0x40000000;  // ctrlerr bit6
  if (ctrlcommunicationerr.any()) {
    AddEventTriger(2039U);
    return true;
  }
  return false;
}

bool EventCollect::UpdateCtrlInputCheckErrTrigger(
    const std::shared_ptr<LocalView>& localview) {
  if (localview == nullptr || !localview->HasFunctionManagerIn() ||
      !localview->HasADCTrajectory()) {
    return false;
  }
  const auto& fct_in = localview->GetADCTrajectory()->function_manager_in();
  std::bitset<32> ctrlinputcheckerr(fct_in.fct_2_soc_tbd_u32_05());
  ctrlinputcheckerr &= 0x80000000;  // ctrlerr bit7
  if (ctrlinputcheckerr.any()) {
    AddEventTriger(2040U);
    return true;
  }
  return false;
}

void EventCollect::UpdateNnpTrigger(
    const std::shared_ptr<LocalView>& localview) {
  UpdateFunctionActiveStatus(localview);
  UpdateAEBOrFCWTrigger(localview);
  UpdateEmergencyBrakingTrigger(localview);
  UpdateEmergencySteeringTrigger(localview);
  UpdateDowngradeTrigger(localview);
  UpdateTakeOverRemindTrigger(localview);
  UpdateAlgorithmRequestTrigger(localview);
  UpdateCollisionTrigger(localview);
  UpdateTakeOverTrigger(localview);
  UpdateControlErrTrigger(localview);
  UpdateRapidAccTrigger(localview);
  UpdateTimeSaveEvent(localview);
  UpdateLatAndLonOverride(localview);
  UpdateLaneChangeFail(localview);
  UpdateBrake(localview);
  UpdateOutputTrajectoryError(localview);
  CheckGearReqCnt(localview);
}

void EventCollect::CheckGearReqCnt(
    const std::shared_ptr<LocalView>& localview) {
  if (!localview->HasChassis() || !localview->HasFunctionManagerIn()) {
    return;
  }

  const auto& ta_pilot_mode = localview->GetFunctionManagerIn()->ta_pilot_mode();
  if (ta_pilot_mode != functionmanager::TaPilotMode::AVP &&
      ta_pilot_mode != functionmanager::TaPilotMode::NNP &&
      ta_pilot_mode != functionmanager::TaPilotMode::NCP &&
      ta_pilot_mode != functionmanager::TaPilotMode::ADAS) {
      return;
  }

  const auto& chassis = localview->GetChassis();
  static int gear_d_cnt = 0;
  static double last_gear_req_timestamp = -1;
  static uint32_t last_gear_req_status = 0;
  if (last_gear_req_status == 5 &&
      chassis->switch_info().gear_position_req_st() != 5) {
    if (last_gear_req_timestamp < 0.0) {
      last_gear_req_timestamp = Clock::NowInSeconds();
    }
    gear_d_cnt++;
  }
  last_gear_req_status = chassis->switch_info().gear_position_req_st();
  if (Clock::NowInSeconds() - last_gear_req_timestamp < 4.0) {
    if (gear_d_cnt >= 3) {
      AddEventTriger(EUROPA_MANUAL_TRIGGER);
      last_gear_req_timestamp = -1;
      gear_d_cnt = 0;
    }
  } else {
    last_gear_req_timestamp = -1;
    gear_d_cnt = 0;
  }
}

void EventCollect::UpdateFunctionActiveStatus(
    const std::shared_ptr<LocalView>& localview) {
  if (localview == nullptr || !localview->HasFunctionManagerIn()) {
    return;
  }
  const auto& fct_in = localview->GetFunctionManagerIn();
  if (!fct_in->has_fct_nnp_in()) {
    return;
  }
  static double last_active_timestamp = 0;
  static bool last_function_status = false;
  const auto& nnp_sys_state = fct_in->fct_nnp_in().nnp_sysstate();
  const auto& adas_mode = fct_in->adas_mode();
  is_nnp_active_ =
      (nnp_sys_state == functionmanager::NNPSysState::NNPS_ACTIVE ||
       nnp_sys_state == functionmanager::NNPSysState::NNPS_OVERRIDE ||
       nnp_sys_state == functionmanager::NNPSysState::NNPS_LAT_OVERRIDE ||
       nnp_sys_state == functionmanager::NNPSysState::NNPS_LON_OVERRIDE);
  is_pilot_active_ = (adas_mode == functionmanager::PILOT);
  is_acc_active_ = (adas_mode == functionmanager::ACC);

  if (last_function_status && !(is_nnp_active_ || is_pilot_active_)) {
    last_active_timestamp = Clock::NowInSeconds();
  }
  is_function_active_ = is_nnp_active_ || is_pilot_active_ || is_acc_active_ ||
                        Clock::NowInSeconds() - last_active_timestamp < 0.3;
  last_function_status = is_nnp_active_ || is_pilot_active_ || is_acc_active_;
}

void EventCollect::UpdateAEBOrFCWTrigger(
    const std::shared_ptr<LocalView>& localview) {
  if (localview->HasFunctionManagerIn()) {
    const auto& fct_in = localview->GetFunctionManagerIn();
    if (fct_in->has_fct_nnp_in()) {
      const auto& nnp_in = fct_in->fct_nnp_in();
      static bool last_is_ok_aeb = false;
      static bool last_is_ok_fcw = false;
      bool is_ok_aeb =
          nnp_in.has_aeb() &&
          nnp_in.aeb().state() != functionmanager::WarningLevel::NO_WARNING &&
          nnp_in.aeb().id() != 0;
      // AEB事件 2001
      if (!last_is_ok_aeb && is_ok_aeb) {
        // AddEventTriger(2001U);
        AERROR << "- planning trigger event: AEB";
      }
      last_is_ok_aeb = is_ok_aeb;
      bool is_ok_fcw =
          nnp_in.has_fcw() &&
          nnp_in.fcw().state() == functionmanager::WarningLevel::LEVEL2 &&
          nnp_in.fcw().id() != 0;
      // FCW事件 2017
      if (!last_is_ok_fcw && is_ok_fcw) {
        // AddEventTriger(2017U);
        AERROR << "- planning trigger event: FCW";
      }
      last_is_ok_fcw = is_ok_fcw;
    }
  }
}

void EventCollect::UpdateEmergencyBrakingTrigger(
    const std::shared_ptr<LocalView>& localview) {
  if (localview == nullptr || !localview->HasFunctionManagerIn() ||
      !localview->HasADCTrajectory() || !localview->HasChassis()) {
    return;
  }
  if (!is_ok_trigger_config_) {
    return;
  }
  const auto& fct_out = localview->GetADCTrajectory()->function_manager_out();
  const auto& chassis = localview->GetChassis();
  const auto& nnp_trigger_config = trigger_config_.nnp_config();
  double speed_mps = 0.0;
  if (chassis->has_speed_mps()) {
    speed_mps = chassis->speed_mps();
  }
  double lon_acc = 0.0;
  if (fct_out.has_nnp_metric() && fct_out.nnp_metric().has_filter_acc_info()) {
    lon_acc = fct_out.nnp_metric().filter_acc_info().lon_acc();
  }
  static int emerger_brake_cnt = 0;
  static double last_emerger_brake_time = Clock::NowInSeconds();
  static constexpr int kEmergerBrakeTotalCnt = 3;
  if (is_function_active_ &&
      kBaseSpeed * speed_mps >
          nnp_trigger_config.emerger_brake_config().emerger_brake_speed() &&
      lon_acc < nnp_trigger_config.emerger_brake_config().emerger_brake_acc()) {
    if (Clock::NowInSeconds() - last_emerger_brake_time < 1.5) {
      emerger_brake_cnt++;
    } else {
      last_emerger_brake_time = Clock::NowInSeconds();
      emerger_brake_cnt = 1;
    }
    if (emerger_brake_cnt >= kEmergerBrakeTotalCnt) {
      last_emerger_brake_time = Clock::NowInSeconds();
      emerger_brake_cnt = 0;
      AddEventTriger(2002U);
    }
  } else if (!is_function_active_) {
    emerger_brake_cnt = 0;
  }
}

void EventCollect::UpdateEmergencySteeringTrigger(
    const std::shared_ptr<LocalView>& localview) {
  if (localview == nullptr || !localview->HasFunctionManagerIn() ||
      !localview->HasADCTrajectory() || !localview->HasChassis()) {
    return;
  }
  if (!is_ok_trigger_config_) {
    return;
  }
  const auto& chassis = localview->GetChassis();
  double speed_kmh = 0.0;
  if (chassis->has_speed_mps()) {
    speed_kmh = chassis->speed_mps() * kBaseSpeed;
  }
  double steer_rate = 0.0;
  const auto& nnp_metric =
      localview->GetADCTrajectory()->function_manager_out().nnp_metric();
  if (nnp_metric.has_filter_acc_info() &&
      nnp_metric.filter_acc_info().has_steer_rate()) {
    steer_rate = std::fabs(nnp_metric.filter_acc_info().steer_rate());
  }
  const auto& signal_light =
      localview->GetADCTrajectory()->decision().vehicle_signal().turn_signal();
  if (!is_function_active_ ||
      signal_light != common::VehicleSignal::TURN_NONE || speed_kmh < 10.0) {
    return;
  }
  const auto steer_rate_flag = common::math::InterpolationOne(
      speed_kmh, emerger_steer_speeds_, emerger_steer_rates_);
  if (steer_rate > steer_rate_flag) {
    AddEventTriger(DRIVER_EMERGENCY_STEERING);
  }
}

void EventCollect::UpdateOutputTrajectoryError(
    const std::shared_ptr<LocalView>& localview) {
  if (!localview->HasADCTrajectory()) {
    return;
  }
  static double last_error_code_timestamp = 0.0;
  static double last_is_avp_mode = 0.0;
  static double last_is_active_function = 0.0;
  const auto& trajectory = localview->GetADCTrajectory();
  static functionmanager::TaPilotMode last_ta_pilot_mode =
      functionmanager::TaPilotMode::NO_CONTROL;
  const auto& ta_pilot_mode = trajectory->function_manager_in().ta_pilot_mode();
  const auto& driver_mode = trajectory->function_manager_in().driver_mode();
  if (ta_pilot_mode == functionmanager::TaPilotMode::AVP &&
      driver_mode != functionmanager::DriveMode::AVP_LAT_LGT_ACTIVE &&
      driver_mode != functionmanager::DriveMode::AVP_LAT_LGT_QUIT &&
      driver_mode != functionmanager::DriveMode::AVP_TMP &&
      driver_mode != functionmanager::DriveMode::AVP_CRUSING) {
    return;
  }
  if (last_ta_pilot_mode != ta_pilot_mode &&
      (last_ta_pilot_mode == functionmanager::TaPilotMode::AVP ||
       ta_pilot_mode == functionmanager::TaPilotMode::AVP)) {
    last_is_avp_mode = Clock::NowInSeconds();
  }
  if (ta_pilot_mode != functionmanager::TaPilotMode::NO_CONTROL) {
    last_is_active_function = Clock::NowInSeconds();
  }
  last_ta_pilot_mode = ta_pilot_mode;
  if (trajectory->header().status().error_code() ==
          common::ErrorCode::LOCALVIEW_INVALID_INPUT_ERROR &&
      Clock::NowInSeconds() - last_is_active_function < 1.0) {
    AddEventTriger(2010U);
  } else if (Clock::NowInSeconds() - last_is_avp_mode > 5.0 &&
             Clock::NowInSeconds() - last_error_code_timestamp > 30.0 &&
             trajectory->has_header() && trajectory->header().has_status() &&
             trajectory->header().status().has_error_code() &&
             trajectory->header().status().error_code() !=
                 common::ErrorCode::LOCALVIEW_INVALID_INPUT_ERROR &&
             trajectory->header().status().error_code() !=
                 common::ErrorCode::LOCALVIEW_FSM_INITIAL_ERROR &&
             trajectory->header().status().error_code() !=
                 common::ErrorCode::OK) {
    AddEventTriger(2010U);
    last_error_code_timestamp = Clock::NowInSeconds();
  }
}

void EventCollect::UpdateDowngradeTrigger(
    const std::shared_ptr<LocalView>& localview) {
  if (localview == nullptr || !localview->HasFunctionManagerIn() ||
      !localview->HasADCTrajectory()) {
    return;
  }
  static bool last_nnp_status = false;
  const auto& fct_out = localview->GetADCTrajectory()->function_manager_out();
  const auto& odd_type = fct_out.odd_info().type();
  const auto& to_odd_len = fct_out.odd_info().to_end_len();
  const auto& odd = fct_out.odd_info().odd_type();
  static constexpr double kMinToOddLen = 100.0;
  bool is_odd = odd != routing::LaneWaypoint::SPECIAL_AREA &&
                (odd_type == routing::LaneWaypointType::ODD_END ||
                 ((odd_type == routing::LaneWaypointType::ODD_START ||
                   odd_type == routing::LaneWaypointType::ROUTE_BREAK) &&
                  to_odd_len < kMinToOddLen));
  const auto& fct_in = localview->GetFunctionManagerIn();
  bool is_fault =
      fct_in->has_nnp_switch_conditions() &&
      fct_in->nnp_switch_conditions().has_is_nnp_state_system_fault() &&
      fct_in->nnp_switch_conditions().is_nnp_state_system_fault();
  if (fct_in->has_fct_nnp_in()) {
    const auto& acc_state = fct_in->fct_nnp_in().acc_state();
    const auto& npilot_state = fct_in->fct_nnp_in().npilot_state();
    is_fault =
        (is_fault || acc_state == functionmanager::FctToNnpInput::ACC_FAULT ||
         npilot_state == functionmanager::FctToNnpInput::PILOT_FAULT);
    bool ctrl_err = fct_in->has_nnp_hmi_signals() &&
                    fct_in->nnp_switch_conditions().has_ctr_enable() &&
                    fct_in->nnp_switch_conditions().ctr_enable();
  }
  if (last_nnp_status && !is_nnp_active_) {
    if (is_odd) {  // 2004 odd降级
      AddEventTriger(ODD_DOWNDRAGE);
    }
  }
  last_nnp_status = is_nnp_active_;
}

void EventCollect::UpdateTakeOverRemindTrigger(
    const std::shared_ptr<LocalView>& localview) {
  if (!localview->HasADCTrajectory() || !localview->HasFunctionManagerIn()) {
    return;
  }
  if (!is_function_active_) {
    return;
  }
  static bool last_is_take_over_remind = false;
  const auto& fct_in = localview->GetFunctionManagerIn();
  const auto& fct_out = localview->GetADCTrajectory()->function_manager_out();
  const auto& nnp_rino_status = fct_out.nnp_fct_out().nnp_rino_status();
  std::bitset<32> nnp_lon_take_over(fct_out.soc_2_fct_tbd_u32_03());
  nnp_lon_take_over &= 0x01;
  bool is_exit_function =
      fct_in->has_nnp_hmi_signals() &&
      fct_in->nnp_hmi_signals().lateralctrhandoffreleasewarning();
  bool is_take_over_remind =
      (nnp_rino_status == functionmanager::NNPRinoStatus::TASK_TAKE_OVER_REQ ||
       nnp_lon_take_over.any() || is_exit_function);
  if (!last_is_take_over_remind && is_take_over_remind) {
    AddEventTriger(TAKEOVER_REMIND);
  }
  last_is_take_over_remind = is_take_over_remind;
}

void EventCollect::UpdateAlgorithmRequestTrigger(
    const std::shared_ptr<LocalView>& localview) {}

void EventCollect::UpdateCollisionTrigger(
    const std::shared_ptr<LocalView>& localview) {
  if (localview == nullptr || !localview->HasFunctionManagerIn() ||
      !localview->HasADCTrajectory() || !localview->HasChassis()) {
    return;
  }
  const auto& fct_out = localview->GetADCTrajectory()->function_manager_out();
  const auto& chassis = localview->GetChassis();
  const auto& nnp_trigger_config = trigger_config_.nnp_config();
  double speed_mps = 0.0;
  if (chassis->has_speed_mps()) {
    speed_mps = chassis->speed_mps();
  }
  uint32_t crash_status = 0;
  if (chassis->has_crash_status()) {
    crash_status = chassis->crash_status();
  }
  static int cycle = 0;
  static double last_speed = 0.0;
  // 在横向或纵向在150ms范围内，车速变化不小于8KM/H
  if (cycle == 2 && std::fabs(last_speed) > 1.0 &&
      std::fabs(speed_mps - last_speed) > 3.5) {
    cycle = 0;
    AddEventTriger(2045U);
  }
  // 在纵向在150ms范围内，车速变化不小于24km/h, 不可逆约束装置展开
  if (crash_status == 0x01 ||
      (cycle == 2 && std::fabs(speed_mps - last_speed) > 8.8)) {
    cycle = 0;
    AddEventTriger(2046U);
  }
  if (cycle == 0) {
    last_speed = speed_mps;
  }
  cycle = (cycle + 1) % 3;
  if (!is_ok_trigger_config_) {
    return;
  }
  double lon_acc = 0.0;
  if (fct_out.has_nnp_metric() && fct_out.nnp_metric().has_filter_acc_info()) {
    lon_acc = fct_out.nnp_metric().filter_acc_info().lon_acc();
  }
  if ((lon_acc < nnp_trigger_config.collision_config().collision_acc() &&
       speed_mps > nnp_trigger_config.collision_config().collision_speed()) ||
      crash_status == 0x01) {
    if (is_function_active_) {
      AddEventTriger(CLOUD_POSSIBLE_COLLISION_EVENT);
    }
  }
}

void EventCollect::UpdateTakeOverTrigger(
    const std::shared_ptr<LocalView>& localview) {
  if (localview == nullptr || !localview->HasFunctionManagerIn() ||
      !localview->HasADCTrajectory()) {
    return;
  }
  static bool last_is_ok = false;
  // static bool last_pilot_active_status = false;
  bool is_ok = is_function_active_ && !is_nnp_active_;
  const auto& fct_out = localview->GetADCTrajectory()->function_manager_out();
  const auto& odd_info = fct_out.odd_info();
  static constexpr double kMaxToOddLen = 500;  // m
  const auto& to_odd_len = odd_info.to_end_len();
  const auto& to_next_odd_len = odd_info.to_next_len();
  if (odd_info.has_odd_type() &&
      (odd_info.odd_type() == routing::LaneWaypoint::ROAD_END ||
       odd_info.odd_type() == routing::LaneWaypoint::SPECIAL_AREA) &&
      to_odd_len < kMaxToOddLen && to_odd_len > 0.01) {
    return;
  }
  bool is_odd_exit =
      (odd_info.has_next_info() && odd_info.next_info() == "QuitImmediate" &&
       to_next_odd_len < FLAGS_layer_function_quit_distance + 10);
  if (!is_odd_exit && !last_is_ok && is_ok && !is_pilot_active_ &&
      !is_acc_active_) {
    AddEventTriger(NNP_DRIVER_ACTIVE_TAKEOVER);
  }

  last_is_ok = is_ok;
  // last_pilot_active_status = is_pilot_active_;
}

void EventCollect::UpdateControlErrTrigger(
    const std::shared_ptr<LocalView>& localview) {
  if (localview == nullptr) {
    return;
  }
  // 2015 横纵向控制误差偏大
  static bool last_pose_err = true;
  static constexpr double kLatPoseErr = 0.5;    // nnp avp adas
  static constexpr double kNNpLonPoseErr = 10;  // nnp adas
  bool is_lat_active = false;
  bool is_lon_active = false;
  if (localview->HasFunctionManagerIn()) {
    const auto& drive_mode = localview->GetFunctionManagerIn()->driver_mode();
    is_lat_active =
        (drive_mode == functionmanager::DriveMode::NNP_LAT_LGT_ACTIVE) ||
        (drive_mode == functionmanager::DriveMode::NCP_LAT_LGT_ACTIVE) ||
        (drive_mode == functionmanager::DriveMode::ADAS_LAT_LGT_ACTIVE) ||
        (drive_mode ==
         functionmanager::DriveMode::ADAS_LAT_ACTIVE_LGT_OVERRIDE) ||
        (drive_mode ==
         functionmanager::DriveMode::NCP_LAT_ACTIVE_LGT_OVERRIDE) ||
        (drive_mode == functionmanager::DriveMode::NNP_LAT_ACTIVE_LGT_OVERRIDE);
    is_lon_active =
        (drive_mode == functionmanager::DriveMode::NNP_LAT_LGT_ACTIVE) ||
        (drive_mode == functionmanager::DriveMode::NCP_LAT_LGT_ACTIVE) ||
        (drive_mode == functionmanager::DriveMode::ADAS_LAT_LGT_ACTIVE) ||
        (drive_mode ==
         functionmanager::DriveMode::ADAS_LGT_ACTIVE_LAT_OVERRIDE) ||
        (drive_mode ==
         functionmanager::DriveMode::NCP_LGT_ACTIVE_LAT_OVERRIDE) ||
        (drive_mode == functionmanager::DriveMode::NNP_LGT_ACTIVE_LAT_OVERRIDE);
  }
  if (localview->HasMcuToSocPnc()) {
    const auto& traj_calc = localview->GetMcuToSocPnc()->trajcalc();
    const auto& lat_pose_err = traj_calc.trajcalc_lat_poserrcmd_sg();
    const auto& lon_pose_err = traj_calc.trajcalc_lon_poserrcmd_sg();
    bool cur_pose_err = ((is_lat_active && lat_pose_err > kLatPoseErr) ||
                         (is_lon_active && lon_pose_err > kNNpLonPoseErr));
    if (!last_pose_err && cur_pose_err) {  // 2015
      AddEventTriger(EXCESSIVE_DEVIATION_TAKEOVER);
    }
    last_pose_err = cur_pose_err;
  } else if (localview->HasMbdDebugFromMCU()) {
    const auto& traj_calc = localview->GetMbdDebugFromMCU()->traj_calc_debug();
    const auto& lat_pose_err = traj_calc.trajcalc_lat_poserrcmd();
    const auto& lon_pose_err = traj_calc.trajcalc_lon_poserrcmd();
    bool cur_pose_err =
        (lat_pose_err > kLatPoseErr || lon_pose_err > kNNpLonPoseErr) &&
        (is_nnp_active_ || is_pilot_active_);
    cur_pose_err =
        cur_pose_err || (lon_pose_err > kNNpLonPoseErr && is_acc_active_);
    if (!last_pose_err && cur_pose_err) {  // 2015
      AddEventTriger(EXCESSIVE_DEVIATION_TAKEOVER);
    }
    last_pose_err = cur_pose_err;
  }
  // 2023 ABS/ESC/TCS报错
  if (localview->Hasmcu_to_soc_DebugData() && is_function_active_) {
    const auto& mcu_to_soc = localview->Getmcu_to_soc_DebugData();
    const auto& tcs_active = mcu_to_soc->radptrin_veh_can().idb7_tcsactive();
    const auto& tcs_fail = mcu_to_soc->radptrin_veh_can().idb7_tcsfail();
    const auto& abs_active = mcu_to_soc->radptrin_veh_can().idb7_absactive();
    const auto& abs_fail = mcu_to_soc->radptrin_veh_can().idb7_absfail();
    const auto& esc_active = mcu_to_soc->radptrin_veh_can().idb7_escactive();
    const auto& esc_fail = mcu_to_soc->radptrin_veh_can().idb7_escfail();
    bool is_control_err = tcs_active != 0 || tcs_fail != 0 || abs_active != 0 ||
                          abs_fail != 0 || esc_active != 0 || esc_fail != 0;
    static bool last_is_control_err = false;
    if (!last_is_control_err && is_control_err) {
      AddEventTriger(CONTROL_FAULT_TAKEOVER);
    }
    last_is_control_err = is_control_err;
  }
}

void EventCollect::UpdateRapidAccTrigger(
    const std::shared_ptr<LocalView>& localview) {
  if (localview == nullptr || !localview->HasFunctionManagerIn() ||
      !localview->HasADCTrajectory() || !localview->HasChassis()) {
    return;
  }
  if (!is_ok_trigger_config_) {
    return;
  }
  const auto& fct_in = localview->GetFunctionManagerIn();
  const auto& fct_out = localview->GetADCTrajectory()->function_manager_out();
  const auto& chassis = localview->GetChassis();
  const auto& nnp_trigger_config = trigger_config_.nnp_config();
  const auto& avp_trigger_config = trigger_config_.avp_config();
  double speed_mps = 0.0;
  if (chassis->has_speed_mps()) {
    speed_mps = chassis->speed_mps();
  }
  double lon_acc = 0.0;
  if (fct_out.has_nnp_metric() && fct_out.nnp_metric().has_filter_acc_info()) {
    lon_acc = fct_out.nnp_metric().filter_acc_info().lon_acc();
  }
  const auto& lane_speed_limit = fct_in->fct_nnp_in().longitud_ctrl_setspeed();
  double acc_buff = nnp_trigger_config.rapid_acc_config(0).rapid_acc();
  if (speed_mps > nnp_trigger_config.rapid_acc_config(0).rapid_speed()) {
    acc_buff = nnp_trigger_config.rapid_acc_config(1).rapid_acc();
  }
  if (speed_mps > nnp_trigger_config.rapid_acc_config(2).rapid_speed()) {
    acc_buff = nnp_trigger_config.rapid_acc_config(2).rapid_acc();
  }
  if (fct_in->ta_pilot_mode() == functionmanager::TaPilotMode::AVP &&
      avp_trigger_config.has_rapid_acc()) {
    acc_buff = avp_trigger_config.rapid_acc();
  }
  if (is_function_active_ && kBaseSpeed * speed_mps > lane_speed_limit &&
      lon_acc > acc_buff) {
    // AddEventTriger(QUICKLY_ACC);
    AERROR << "- planning trigger event: QUICKLY_ACC";
  }
}

void EventCollect::UpdateBrake(const std::shared_ptr<LocalView>& localview) {
  if (!localview->HasADCTrajectory() || !localview->HasFunctionManagerIn()) {
    return;
  }
  const auto& nnp_active_status =
      localview->GetFunctionManagerIn()->fct_nnp_in().nnp_sysstate();
  if (localview->GetADCTrajectory()
          ->function_manager_out()
          .nnp_metric()
          .has_jerk_err_info()) {
    const auto& jerk_err_info = localview->GetADCTrajectory()
                                    ->function_manager_out()
                                    .nnp_metric()
                                    .jerk_err_info();
    static double last_braking_timestamp = -1.0;
    if (nnp_active_status == functionmanager::NNPSysState::NNPS_ACTIVE &&
        jerk_err_info.has_lon() &&
        (Clock::NowInSeconds() - last_braking_timestamp > 60.0)) {
      last_braking_timestamp = Clock::NowInSeconds();
      AddEventTriger(2036U);
    }
  }
}

void EventCollect::UpdateLaneChangeFail(
    const std::shared_ptr<LocalView>& localview) {
  if (!localview->HasADCTrajectory()) {
    return;
  }
  if (!is_function_active_) {
    return;
  }

  static bool last_is_current_opt_succeed = false;

  if (!localview->GetADCTrajectory()
           ->function_manager_out()
           .has_hmi_lane_change_debug()) {
    last_is_current_opt_succeed = false;
    return;
  }

  const auto& hmi_debug = localview->GetADCTrajectory()
                              ->function_manager_out()
                              .hmi_lane_change_debug();
  const auto& hmi_lane_change_status = localview->GetADCTrajectory()
                                           ->function_manager_out()
                                           .hmi_lane_change_debug()
                                           .hmi_lane_change_status();
  const auto& hmi_cancel_lane_change_reason =
      localview->GetADCTrajectory()
          ->function_manager_out()
          .hmi_lane_change_debug()
          .hmi_cancel_lane_change_reason();

  bool is_current_opt_succeed = hmi_debug.is_current_opt_succeed();

  if ((last_is_current_opt_succeed && !is_current_opt_succeed) ||
      (hmi_lane_change_status ==
           functionmanager::HmiChangeLaneStatus::HMI_CHANGE_LANE_CANCELED &&
       hmi_cancel_lane_change_reason ==
           functionmanager::CancelLaneChangeReason::
               SAFETY_DECIDER_CANCEL_LANE_CHANGE)) {
    AddEventTriger(2037U);
  }
  last_is_current_opt_succeed = is_current_opt_succeed;
}

void EventCollect::UpdateLatAndLonOverride(
    const std::shared_ptr<LocalView>& localview) {
  if (!localview->HasFunctionManagerIn() || !localview->HasADCTrajectory()) {
    return;
  }
  const auto& fct_out = localview->GetADCTrajectory()->function_manager_out();
  const auto& odd_info = fct_out.odd_info();
  static constexpr double kMaxToOddLen = 500;  // m
  const auto& to_odd_len = odd_info.to_end_len();
  if (odd_info.has_odd_type() &&
      (odd_info.odd_type() == routing::LaneWaypoint::ROAD_END ||
       odd_info.odd_type() == routing::LaneWaypoint::SPECIAL_AREA) &&
      to_odd_len < kMaxToOddLen && to_odd_len > 0.01) {
    return;
  }
  const auto& nnp_fct_in = localview->GetFunctionManagerIn();
  const auto& drive_mode = nnp_fct_in->driver_mode();
  static bool last_active_status = false;
  static double last_active_timestamp = -1.0;
  static double last_lat_active_timestamp = -1.0;
  static double last_lon_active_timestamp = -1.0;
  if (last_active_timestamp < 0.0 &&
      (drive_mode == functionmanager::DriveMode::NNP_LGT_ACTIVE_LAT_OVERRIDE ||
       drive_mode == functionmanager::DriveMode::NCP_LGT_ACTIVE_LAT_OVERRIDE ||
       drive_mode == functionmanager::DriveMode::NNP_LAT_ACTIVE_LGT_OVERRIDE ||
       drive_mode == functionmanager::DriveMode::NCP_LAT_ACTIVE_LGT_OVERRIDE ||
       drive_mode == functionmanager::DriveMode::NNP_LAT_LGT_ACTIVE ||
       drive_mode == functionmanager::DriveMode::NCP_LAT_LGT_ACTIVE)) {
    last_active_timestamp = Clock::NowInSeconds();
  } else if (drive_mode == functionmanager::DriveMode::NONE) {
    last_active_timestamp = -1.0;
  }
  bool is_ok_active = Clock::NowInSeconds() - last_active_timestamp > 5.0;
  if (is_ok_active && last_active_status &&
      (Clock::NowInSeconds() - last_lon_active_timestamp > 1.0) &&
      (drive_mode == functionmanager::DriveMode::NNP_LGT_ACTIVE_LAT_OVERRIDE ||
       drive_mode == functionmanager::DriveMode::NCP_LGT_ACTIVE_LAT_OVERRIDE)) {
    last_lon_active_timestamp = Clock::NowInSeconds();
    AddEventTriger(2034U);
  }
  if (is_ok_active && last_active_status &&
      (Clock::NowInSeconds() - last_lat_active_timestamp > 1.0) &&
      (drive_mode == functionmanager::DriveMode::NNP_LAT_ACTIVE_LGT_OVERRIDE ||
       drive_mode == functionmanager::DriveMode::NCP_LAT_ACTIVE_LGT_OVERRIDE)) {
    last_lat_active_timestamp = Clock::NowInSeconds();
    AddEventTriger(2035U);
  }
  last_active_status =
      (drive_mode == functionmanager::DriveMode::NNP_LAT_LGT_ACTIVE ||
       drive_mode == functionmanager::DriveMode::NCP_LAT_LGT_ACTIVE);
}

void EventCollect::UpdateTimeSaveEvent(
    const std::shared_ptr<LocalView>& localview) {
  if (!localview->HasChassis() || !localview->HasFunctionManagerIn() ||
      !localview->HasADCTrajectory()) {
    return;
  }
  // const auto& chassis = localview->GetChassis();
  const auto& fct_in = localview->GetFunctionManagerIn();
  static bool last_nnp_active_status = false;
  static bool last_ncp_active_status = false;
  static bool last_pilot_active_status = false;
  static bool last_acc_active_status = false;
  const auto& ta_pilot_mode = fct_in->ta_pilot_mode();
  const auto& adas_mode = fct_in->adas_mode();
  bool cur_nnp_active_status =
      (ta_pilot_mode == functionmanager::TaPilotMode::NNP);
  if (!last_nnp_active_status && cur_nnp_active_status) {
    AddEventTriger(NNP_ACTIVE);  // nnp首次激活或升级到nnp
  }
  last_nnp_active_status = cur_nnp_active_status;
  bool cur_ncp_active_status =
      (ta_pilot_mode == functionmanager::TaPilotMode::NCP);
  if (!last_ncp_active_status && cur_ncp_active_status) {
    AddEventTriger(NCP_ACTIVE);  // ncp首次激活或升级到ncp
  }
  last_ncp_active_status = cur_ncp_active_status;
  bool cur_pilot_active_status =
      (ta_pilot_mode == functionmanager::TaPilotMode::ADAS &&
       adas_mode == functionmanager::AdasMode::PILOT);
  if (!last_pilot_active_status && cur_pilot_active_status) {
    AddEventTriger(PILOT_ACTIVE);  // pilot首次激活或降级到pilot
  }
  last_pilot_active_status = cur_pilot_active_status;
  bool cur_acc_active_status =
      (ta_pilot_mode == functionmanager::TaPilotMode::ADAS &&
       adas_mode == functionmanager::AdasMode::ACC);
  if (!last_acc_active_status && cur_acc_active_status) {
    AddEventTriger(ACC_ACTIVE);  // acc首次激活或降级到acc
  }
  last_acc_active_status = cur_acc_active_status;

  bool cur_active_status = (cur_nnp_active_status || cur_ncp_active_status ||
                            cur_acc_active_status || cur_pilot_active_status);
  static double last_active_timestamp = 0.0;
  if (cur_active_status) {
    last_active_timestamp = Clock::NowInSeconds();
  }
  if (Clock::NowInSeconds() - last_active_timestamp > 0.3) {
    return;
  }
  const auto& trajectory = localview->GetADCTrajectory();
  int i = 0;
  double min_acc = std::numeric_limits<double>::max();
  for (const auto& point : trajectory->trajectory_point()) {
    if (point.relative_time() > 0.0) {
      if (i > 3) {
        break;
      }
      if (point.a() < min_acc) {
        min_acc = point.a();
      }
      i++;
    }
  }
  if (min_acc < -5.0) {
    // 有碰撞风险事件：AD 请求纵向减速度>5m/s2
    AddEventTriger(LOCAL_POSSIBLE_COLLISION_EVENT);
  }
}

void EventCollect::UpdateAvpTrigger(
    const std::shared_ptr<LocalView>& localview) {
  if (localview == nullptr) {
    AERROR << "localview is nullptr";
    return;
  }
  static constexpr double kTriggerIntervalTime = 5.0;
  static constexpr double kStandStillSpdThred = 0.05;
  static constexpr double kTriggerOneloopTime = 480;
  double cur_time = Clock::NowInSeconds();
  bool is_valid_avp_trigger = false;
  if (cur_time - trigger_time_ > kTriggerIntervalTime &&
      localview->HasFunctionManagerIn() &&
      localview->GetFunctionManagerIn()->has_fct_avp_in()) {
    const auto& sys_warning_info =
        localview->GetFunctionManagerIn()->fct_avp_in().sys_warning_info();
    if (valid_avp_warininginfo_set_.find(sys_warning_info) !=
        valid_avp_warininginfo_set_.end()) {
      if (sys_warning_info ==
              functionmanager::AvpFctIn::NTP_PARKING_DO_NOT_ACCELARATION_0X4E ||
          sys_warning_info == functionmanager::AvpFctIn::
                                  NTP_PARKING_RELEASE_BRAKE_TOCONTINUE_0X54 ||
          sys_warning_info ==
              functionmanager::AvpFctIn::NTP_PARKING_LATOVERRIDE_0x49 ||
          sys_warning_info ==
              functionmanager::AvpFctIn::NTP_PARKING_EPBINTERVENE_0x58 ||
          sys_warning_info ==
              functionmanager::AvpFctIn::NTP_PARKING_GEARINTERVENE_0x4A ||
          sys_warning_info ==
              functionmanager::AvpFctIn::DRIVER_INTERVENTION_0x12) {
        if (!localview->HasVehicleState()) {
          is_valid_avp_trigger = false;
          return;
        }
        const auto& vehicle_state = localview->GetVehicleState();
        is_valid_avp_trigger =
            ((vehicle_state->gear() == soc::Chassis::GEAR_REVERSE ||
              vehicle_state->gear() == soc::Chassis::GEAR_DRIVE) &&
             abs(vehicle_state->linear_velocity()) >= kStandStillSpdThred);
      } else {
        is_valid_avp_trigger = true;
      }
    }
    if (!is_valid_avp_trigger) {
      pre_warning_info_ = sys_warning_info;
    }
    if (!has_first_trigger_ && pre_warning_info_ != sys_warning_info &&
        is_valid_avp_trigger && Clock::NowInSeconds() < trigger_end_time_) {
      trigger_counter_++;
    }
    if (pre_warning_info_ != sys_warning_info &&
        Clock::NowInSeconds() > trigger_end_time_) {
      trigger_end_time_ = Clock::NowInSeconds() + kTriggerOneloopTime;
      trigger_counter_ = 1;
    }
    if (has_first_trigger_) {
      trigger_end_time_ = Clock::NowInSeconds() + kTriggerOneloopTime;
      trigger_counter_ = 1;
    }
    if (trigger_counter_ > 3) {
      return;
    }
    switch (sys_warning_info) {
      case functionmanager::AvpFctIn::GEAR_SHIFT_OVER_COUNTER_0x11:
        AddEventTriger(PARKING_NUM_EXCEED);
        has_first_trigger_ = false;
        pre_warning_info_ = sys_warning_info;
        break;
      case functionmanager::AvpFctIn::PASUE_OVER_COUNTER_0xB:
        AddEventTriger(MUTI_PARKING_TAKEOVER);
        has_first_trigger_ = false;
        pre_warning_info_ = sys_warning_info;
        break;
      case functionmanager::AvpFctIn::ENVIRONMENT_ERROR_0x2F:
      case functionmanager::AvpFctIn::SYSTEM_QUIT_0x1B:
      case functionmanager::AvpFctIn::PLANNING_ERROR_0xE:
      case functionmanager::AvpFctIn::PARKING_SPACE_ERROR_0x17:
      case functionmanager::AvpFctIn::NTP_PARKING_ENVIRONMENTERROR_0x5D:
      case functionmanager::AvpFctIn::NTP_SYSFAULT_0x4B:
        AddEventTriger(PARKING_CAPABILITY_FAILURE_TAKEOVER);
        has_first_trigger_ = false;
        pre_warning_info_ = sys_warning_info;
        break;
      case functionmanager::AvpFctIn::RELATED_SYSTEM_ERROR_0xD:
      case functionmanager::AvpFctIn::NTP_RELATEDSYSTEMERROR_0x4C:
        AddEventTriger(PARKING_FUCTION_FAILURE_TAKEOVER);
        has_first_trigger_ = false;
        pre_warning_info_ = sys_warning_info;
        break;
      case functionmanager::AvpFctIn::VEHICLE_CRASHED_0x2B:
      case functionmanager::AvpFctIn::NTP_VEHICLE_CRASHED_0x59:
        AddEventTriger(PARKING_COLLISION_EVENT);
        has_first_trigger_ = false;
        pre_warning_info_ = sys_warning_info;
        break;
      case functionmanager::AvpFctIn::UNINTENDED_ERROR_0x3A:
      case functionmanager::AvpFctIn::NTP_PARKING_HANDSHAKEOVERTIME_0x5F:
      case functionmanager::AvpFctIn::NTP_PARKING_CRUSINGERROR_0x60:
        AddEventTriger(PARKING_UNINTENDED_ERROR);
        has_first_trigger_ = false;
        pre_warning_info_ = sys_warning_info;
        break;
      case functionmanager::AvpFctIn::PAUSE_OVER_TIME_0xC:
        AddEventTriger(PARKING_PAUSE_OVER_TIME);
        has_first_trigger_ = false;
        pre_warning_info_ = sys_warning_info;
        break;
      case functionmanager::AvpFctIn::NTP_PARKING_DO_NOT_ACCELARATION_0X4E:
      case functionmanager::AvpFctIn::NTP_PARKING_RELEASE_BRAKE_TOCONTINUE_0X54:
      case functionmanager::AvpFctIn::NTP_PARKING_LATOVERRIDE_0x49:
      case functionmanager::AvpFctIn::NTP_PARKING_EPBINTERVENE_0x58:
      case functionmanager::AvpFctIn::NTP_PARKING_GEARINTERVENE_0x4A:
      case functionmanager::AvpFctIn::DRIVER_INTERVENTION_0x12: {
        if (!localview->HasVehicleState()) {
          AERROR << "localview has no vehiclestate";
          return;
        }
        const auto& vehicle_state = localview->GetVehicleState();
        if ((vehicle_state->gear() == soc::Chassis::GEAR_REVERSE ||
             vehicle_state->gear() == soc::Chassis::GEAR_DRIVE) &&
            abs(vehicle_state->linear_velocity()) >= kStandStillSpdThred) {
          AddEventTriger(PARKING_TAKEOVER_REQUEST_WITH_COLLISION_RISK);
          has_first_trigger_ = false;
          pre_warning_info_ = sys_warning_info;
        }
        break;
      }
      case functionmanager::AvpFctIn::PARKING_OVER_SPEED_0xF:
      case functionmanager::AvpFctIn::NTP_PARKING_OVERSPEED_0x5A:
        AddEventTriger(AVP_OVERSPEED);
        has_first_trigger_ = false;
        pre_warning_info_ = sys_warning_info;
        break;
      case functionmanager::AvpFctIn::SLOPE_OVER_RANGE_0x14:
      case functionmanager::AvpFctIn::NTP_PARKIG_OVERSLOPE_QUIT_0X5B:
        AddEventTriger(AVP_OVERSLOPE);
        has_first_trigger_ = false;
        pre_warning_info_ = sys_warning_info;
        break;
      case functionmanager::AvpFctIn::VEHICLE_BLOCKED_0x2C:
      case functionmanager::AvpFctIn::NTP_PARKING_VEHICLEBLOCKED_0x5C:
        AddEventTriger(AVP_VEHICLE_BLOCKED);
        has_first_trigger_ = false;
        pre_warning_info_ = sys_warning_info;
        break;
      case functionmanager::AvpFctIn::NTP_Cruising_PAUSE_OVER_TIME_0x67:
        AddEventTriger(NTP_PAUSE_OVER_TIME);
        has_first_trigger_ = false;
        pre_warning_info_ = sys_warning_info;
        break;
      default:
        break;
    }
    // todo: AVP_TAKEOVER_REQUEST
  }
}

void EventCollect::AddEventTriger(uint32_t type) {
  if (ptr_trajectory_pb_ == nullptr) {
    AERROR << "ptr_trajectory_pb_ is nullptr";
    return;
  }
  if (emergency_ids_.find(type) != emergency_ids_.end()) {
    if (last_emergency_trigger_.count(type) > 0 &&
        Clock::NowInSeconds() - last_emergency_trigger_.at(type) < 5.0) {
      return;
    }
    last_emergency_trigger_[type] = Clock::NowInSeconds();
  }
  auto* event_info =
      ptr_trajectory_pb_->mutable_event_trigger()->add_event_info();
  event_info->set_type(type);
  trigger_time_ = Clock::NowInSeconds();
}

}  // namespace planning
}  // namespace TL
