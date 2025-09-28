/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/hmi/lon_hmi/lon_hmi.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include "common/configs/config_gflags.h"
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/math_utils.h"
#include "common/time/clock.h"
#include "common/util/macros.h"
#include "google/protobuf/stubs/port.h"
#include "map/hdmap/path.h"
#include "planning/common/frame.h"
#include "planning/common/obstacle.h"
#include "planning/common/planning_gflags.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/perception/transport_element.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

// using TL::common::math::ConvertDisplaySpdToReal;
using common ::math::double_type::DefinitelyLessEqual;
using TL::functionmanager::NNPSysState;
using TL::perception::PerceptionObstacle;

// using common::math::double_type::DefinitelyLess;

void LonHmi::Init(const functionmanager::HmiConfig& hmi_config) {
  hmi_config_ = hmi_config;
  spd_adapt_.Init(hmi_config.speed_adapt_conf());
  acc_overtaking_assistance_.Init(hmi_config.acc_overtake_follow_time_conf());
}

int32 LonHmi::CruiseSpeedLimiter(int32 orin_speeed_km) {
  if (orin_speeed_km < hmi_config_.speed_adapt_conf().min_speed()) {
    return static_cast<int32>(hmi_config_.speed_adapt_conf().min_speed());
  }
  if (orin_speeed_km > hmi_config_.speed_adapt_conf().max_speed()) {
    return static_cast<int>(hmi_config_.speed_adapt_conf().max_speed());
  }
  return orin_speeed_km;
}

LonHmi::LonHmiScenario LonHmi::SelectCurrentScenario(
    functionmanager::FunctionManagerIn* const fct_in,
    const TL::soc::Chassis& chassis) {
  UNUSED(chassis);
  const auto& nnp_fct_in = fct_in->fct_nnp_in();
  const auto& nnp_state = nnp_fct_in.nnp_sysstate();
  const auto& acc_state = nnp_fct_in.acc_state();
  const auto acc_enable =
      acc_state != functionmanager::FctToNnpInput::ACC_OFF &&
      acc_state != functionmanager::FctToNnpInput::ACC_FAULT &&
      acc_state != functionmanager::FctToNnpInput::ACC_RESERVE;

  nnp_active_ = (nnp_state == NNPSysState::NNPS_ACTIVE ||
                 nnp_state == NNPSysState::NNPS_LAT_OVERRIDE ||
                 nnp_state == NNPSysState::NNPS_LON_OVERRIDE ||
                 nnp_state == NNPSysState::NNPS_OVERRIDE ||
                 nnp_state == NNPSysState::NNPS_TO);
  acc_active_ =
      (acc_state == functionmanager::FctToNnpInput::ACC_ACTIVE ||
       acc_state == functionmanager::FctToNnpInput::ACC_STANDSTILL_ACTIVE ||
       acc_state == functionmanager::FctToNnpInput::ACC_STANDSTILL_WAIT ||
       acc_state == functionmanager::FctToNnpInput::ACC_OVERRIDE) &&
      !nnp_active_;
  is_acc_resume_ = nnp_fct_in.is_acc_resume();
  LonHmiScenario current_scenario = ACC_NOT_STANDBY;
  if (!acc_enable) {
    current_scenario = ACC_NOT_STANDBY;
  }
  // acc accstandby
  if (acc_state == functionmanager::FctToNnpInput::ACC_STANDBY) {
    current_scenario = ACC_STANDBY;
  }
  // acc active
  if (acc_active_ && !last_acc_active_) {
    if (is_acc_first_active_) {
      is_acc_first_active_ = false;
    }
    current_scenario = is_acc_resume_ ? ACC_RESUME : ACC_ACTIVE;
  }
  //  ACC_ACTIVATING
  if (acc_active_ && last_acc_active_) {
    current_scenario = ACC_ACTIVATING;
  }
  // nnp active
  if (nnp_active_ && !last_nnp_active_) {
    current_scenario = NNP_ACTIVE;
  }
  //  NNP_ACTIVATING
  if (nnp_active_ && last_nnp_active_) {
    current_scenario = NNP_ACTIVATING;
  }
  return current_scenario;
}

void LonHmi::UpdateInput(functionmanager::FunctionManagerIn* const fct_in,
                         const functionmanager::FunctionManagerOut* fct_out,
                         const TL::common::VehicleState& vehicle_state,
                         const TL::control::McuToSocPnc& control_data) {
  if (fct_in == nullptr) {
    return;
  }
  acc_state_ = fct_in->fct_nnp_in().acc_state();
  tgtspdctgset_ = control_data.can_input_from_mcu().cdcs11_tgtspdctgset();
  tgtspddrftset_ =
      control_data.can_input_from_mcu().has_cdcs11_tgtspddrftset()
          ? static_cast<int>(
                control_data.can_input_from_mcu().cdcs11_tgtspddrftset() * 5 -
                30)
          : 0;
  ADEBUG << " tgtspdctgset_ : " << tgtspdctgset_
         << " tgtspddrftset_ : " << tgtspddrftset_;
  tgtspd_valid_ =
      tgtspddrftset_ >= hmi_config_.speed_adapt_conf().min_offset() &&
      tgtspddrftset_ <= hmi_config_.speed_adapt_conf().max_offset() &&
      tgtspddrftset_ != 0;
  drvoffalertswitch_ =
      control_data.can_input_from_mcu().cdcs11_drvoffalertswitch();
  // 代表着是车上的数据回灌的
  if (fct_in->fct_nnp_in().has_longitud_ctrl_cruise_speedms()) {
#ifdef FOR_BAIDU_SIMULATION
    ProcessBaiduSim(fct_in);
    spd_adapt_.UpdateInput(fct_in, fct_out, vehicle_state, tgtspdctgset_,
                           tgtspddrftset_);
#endif
    return;
  }
  const auto& chassis = vehicle_state.chassis();
  acc_overtaking_assistance_.Update(fct_in, chassis);
  if (FLAGS_enable_planning_self_simulator) {
    spd_adapt_.UpdateInput(fct_in, fct_out, vehicle_state, tgtspdctgset_,
                           tgtspddrftset_);
    ProcessLonFollowInfo(fct_in, chassis);
    return;
  }
  if (!fct_in->fct_nnp_in().has_usr_has_changed_cruise_spd()) {
    fct_in->mutable_fct_nnp_in()->set_usr_has_changed_cruise_spd(false);
  }
  if ((fct_in == nullptr || !fct_in->has_fct_nnp_in() ||
       !chassis.has_speed_display() || !chassis.has_switch_info() ||
       !chassis.switch_info().has_cruise_speed_add() ||
       !chassis.switch_info().has_cruise_speed_minus() ||
       !chassis.switch_info().has_cruise_distance_add() ||
       !chassis.switch_info().has_cruise_speed_minus())) {
    ProcessLonFollowInfo(fct_in, chassis);
    return;
  }
  if (!fct_in->fct_nnp_in().adcs_lon_speed_valid()) {
    // 说明不是回放模式，这里认为车上的数据是经过正确处理的，
    // 如果处理的不对，就需要查这部分的问题了
    speed_display_ = CruiseSpeedLimiter(chassis.speed_display());
    CheckUsrReset(fct_in, chassis);
    auto* mutable_nnp_fct = fct_in->mutable_fct_nnp_in();
    current_scenario_ = SelectCurrentScenario(fct_in, chassis);
    switch (current_scenario_) {
      case ACC_NOT_STANDBY:
        UpdateAccNotStandby(mutable_nnp_fct);
        break;
      case ACC_STANDBY:
        UpdateAccStandby(mutable_nnp_fct);
        break;
      case ACC_ACTIVE:
        UpdateAccActive(mutable_nnp_fct);
        break;
      case ACC_RESUME:
        UpdateResume(mutable_nnp_fct);
        break;
      case ACC_ACTIVATING:
        UpdateAccActivating(mutable_nnp_fct, chassis);
        break;
      case NNP_ACTIVE:
        UpdateNnpActive(mutable_nnp_fct);
        break;
      case NNP_ACTIVATING:
        UpdateNnpActivating(mutable_nnp_fct, chassis);
        break;
    }
  }
  fct_in->mutable_fct_nnp_in()->set_adcs_lon_speed_valid(true);
  spd_adapt_.UpdateInput(fct_in, fct_out, vehicle_state, tgtspdctgset_,
                         tgtspddrftset_);
  ProcessAccStandstill(fct_in, chassis);
  ProcessLonFollowInfo(fct_in, chassis);
}

void LonHmi::UpdateHmiCruiseSpeed(
    functionmanager::FunctionManagerOut* const fct_out) const {
  auto* mutable_fct_out = fct_out->mutable_nnp_fct_out();
  mutable_fct_out->set_tsr_speedsign(target_cruise_speed_);
}

void LonHmi::UpdateOutput(functionmanager::FunctionManagerOut* const fct_out,
                          const std::shared_ptr<hdmap::PncMap>& pnc_map,
                          planning::Frame* const frame,
                          functionmanager::SocToFctBus* const soc_to_fct_bus) {
  if (fct_out == nullptr || pnc_map == nullptr || frame == nullptr ||
      soc_to_fct_bus == nullptr) {
    return;
  }
  UpdateHmiCruiseSpeed(fct_out);
  ProcessStandstillHmi(fct_out, frame);
  spd_adapt_.UpdateOutput(fct_out, pnc_map, frame);
  use_mem_ = spd_adapt_.UseMemory();
  last_nnp_fct_out_.CopyFrom(fct_out->nnp_fct_out());
  is_speed_plan_standstill_ = frame->IsSpeedPlanStandstill();
  last_acc_active_ = acc_active_;
  last_nnp_active_ = nnp_active_;
  static const uint32_t kBroadcast = 0x400;
  uint32_t soc_03_val = 0x00;
  if (fct_out->has_soc_2_fct_tbd_u32_03()) {
    soc_03_val = fct_out->soc_2_fct_tbd_u32_03();
  }
  if (need_broadcast_) {
    fct_out->set_soc_2_fct_tbd_u32_03(soc_03_val | kBroadcast);
  }
  UpdateAccActiveObs(fct_out, frame);
  ProcessDriveOffAlert(soc_to_fct_bus, frame);
  CruiseDistanceDisplayReq(soc_to_fct_bus, fct_out);
  CheckForceReplan(frame);
  CheckStopObsDisappear(frame, fct_out);
}

void LonHmi::UpdateAccNotStandby(
    functionmanager::FctToNnpInput* const nnp_fct_in) {
  if (nnp_fct_in == nullptr) {
    return;
  }
  target_cruise_speed_ = acc_mem_cruise_speed_ > 0 ? acc_mem_cruise_speed_ : 30;
  lon_ctrl_time_level_ =
      acc_mem_ctrl_time_level_ > 0
          ? acc_mem_ctrl_time_level_
          : (lon_ctrl_set_dis_from_mem_ > 0 ? lon_ctrl_set_dis_from_mem_
                                            : kDefaultLonCtrlTimeLevel);
  usr_changed_cruise_speed_ = false;
  lon_ctrl_distance_display_req_ = false;
  nnp_fct_in->set_longitud_ctrl_setspeed(speed_display_);
  nnp_fct_in->set_usr_has_changed_cruise_spd(false);
  nnp_fct_in->set_longitud_ctrl_setdistance(lon_ctrl_time_level_);
}

void LonHmi::UpdateAccStandby(
    functionmanager::FctToNnpInput* const nnp_fct_in) {
  if (nnp_fct_in == nullptr) {
    return;
  }
  target_cruise_speed_ = acc_mem_cruise_speed_ > 0 ? acc_mem_cruise_speed_ : 30;
  lon_ctrl_time_level_ =
      acc_mem_ctrl_time_level_ > 0
          ? acc_mem_ctrl_time_level_
          : (lon_ctrl_set_dis_from_mem_ > 0 ? lon_ctrl_set_dis_from_mem_
                                            : kDefaultLonCtrlTimeLevel);
  usr_changed_cruise_speed_ = false;
  nnp_fct_in->set_longitud_ctrl_setspeed(speed_display_);
  nnp_fct_in->set_usr_has_changed_cruise_spd(false);
  nnp_fct_in->set_longitud_ctrl_setdistance(lon_ctrl_time_level_);
}

void LonHmi::UpdateResume(functionmanager::FctToNnpInput* const nnp_fct_in) {
  if (nnp_fct_in == nullptr) {
    return;
  }
  // 这代表着Mcu给了巡航速度，走的是mcu激活的逻辑
  const auto use_mcu_ctrl_setspeed =
      nnp_fct_in->longitud_ctrl_setspeed() >=
          hmi_config_.speed_adapt_conf().min_speed() &&
      nnp_fct_in->longitud_ctrl_setspeed() <=
          hmi_config_.speed_adapt_conf().max_speed();

  target_cruise_speed_ =
      use_mcu_ctrl_setspeed
          ? static_cast<int>(nnp_fct_in->longitud_ctrl_setspeed())
      : acc_mem_cruise_speed_ > 0 ? CruiseSpeedLimiter(acc_mem_cruise_speed_)
                                  : CruiseSpeedLimiter(speed_display_);
  lon_ctrl_time_level_ = acc_mem_ctrl_time_level_ > 0
                             ? acc_mem_ctrl_time_level_
                             : kDefaultLonCtrlTimeLevel;
  usr_changed_cruise_speed_ = false;
  lon_ctrl_distance_display_req_ = true;
  nnp_fct_in->set_longitud_ctrl_setspeed(target_cruise_speed_);
  nnp_fct_in->set_usr_has_changed_cruise_spd(true);
  nnp_fct_in->set_longitud_ctrl_setdistance(lon_ctrl_time_level_);
  acc_mem_ctrl_time_level_ = lon_ctrl_time_level_;
}

void LonHmi::UpdateAccActive(functionmanager::FctToNnpInput* const nnp_fct_in) {
  if (nnp_fct_in == nullptr) {
    return;
  }
  lon_ctrl_time_level_ =
      acc_mem_ctrl_time_level_ > 0
          ? acc_mem_ctrl_time_level_
          : (lon_ctrl_set_dis_from_mem_ > 0 ? lon_ctrl_set_dis_from_mem_
                                            : kDefaultLonCtrlTimeLevel);
  // 这代表着Mcu给了巡航速度，走的是mcu激活的逻辑
  const auto use_mcu_ctrl_setspeed =
      nnp_fct_in->longitud_ctrl_setspeed() >=
          hmi_config_.speed_adapt_conf().min_speed() &&
      nnp_fct_in->longitud_ctrl_setspeed() <=
          hmi_config_.speed_adapt_conf().max_speed();
  target_cruise_speed_ =
      last_nnp_active_ ? nnp_cruise_speed_
      : use_mcu_ctrl_setspeed
          ? static_cast<int>(nnp_fct_in->longitud_ctrl_setspeed())
          : CruiseSpeedLimiter(speed_display_);
  usr_changed_cruise_speed_ =
      last_nnp_active_ ? usr_changed_cruise_speed_ : false;
  nnp_fct_in->set_longitud_ctrl_setspeed(speed_display_);
  nnp_fct_in->set_usr_has_changed_cruise_spd(true);
  nnp_fct_in->set_longitud_ctrl_setdistance(lon_ctrl_time_level_);
  acc_mem_ctrl_time_level_ = lon_ctrl_time_level_;
  acc_mem_cruise_speed_ = target_cruise_speed_;
}

void LonHmi::UpdateAccActivating(
    functionmanager::FctToNnpInput* const nnp_fct_in,
    const TL::soc::Chassis& chassis) {
  CheckUsrAdjustCruiseSpeed(nnp_fct_in, chassis);
  UpdateLonCtrlTimeLevel(nnp_fct_in, chassis);
  nnp_fct_in->set_longitud_ctrl_setspeed(target_cruise_speed_);
  nnp_fct_in->set_usr_has_changed_cruise_spd(true);
  nnp_fct_in->set_longitud_ctrl_setdistance(lon_ctrl_time_level_);
  acc_mem_cruise_speed_ = target_cruise_speed_;
  acc_mem_ctrl_time_level_ = lon_ctrl_time_level_;
}

void LonHmi::UpdateNnpActive(functionmanager::FctToNnpInput* const nnp_fct_in) {
  if (nnp_fct_in == nullptr) {
    return;
  }
  // 这代表着Mcu给了巡航速度，走的是mcu激活的逻辑
  const auto use_mcu_ctrl_setspeed =
      nnp_fct_in->longitud_ctrl_setspeed() >=
          hmi_config_.speed_adapt_conf().min_speed() &&
      nnp_fct_in->longitud_ctrl_setspeed() <=
          hmi_config_.speed_adapt_conf().max_speed();
  target_cruise_speed_ =
      use_mcu_ctrl_setspeed
          ? static_cast<int>(nnp_fct_in->longitud_ctrl_setspeed())
      : last_nnp_fct_out_.has_adapt_cruise_speed_km() &&
              last_nnp_fct_out_.adapt_cruise_speed_km() >=
                  hmi_config_.speed_adapt_conf().min_speed() &&
              last_nnp_fct_out_.adapt_cruise_speed_km() <=
                  hmi_config_.speed_adapt_conf().max_speed()
          ? static_cast<int>(last_nnp_fct_out_.adapt_cruise_speed_km())
          : speed_display_;
  acc_mem_cruise_speed_ = target_cruise_speed_;
  lon_ctrl_time_level_ =
      acc_mem_ctrl_time_level_ > 0
          ? acc_mem_ctrl_time_level_
          : (lon_ctrl_set_dis_from_mem_ > 0 ? lon_ctrl_set_dis_from_mem_
                                            : kDefaultLonCtrlTimeLevel);
  usr_changed_cruise_speed_ =
      last_acc_active_ ? usr_changed_cruise_speed_ : false;
  lon_ctrl_distance_display_req_ = !last_acc_active_ && !last_nnp_active_;
  nnp_fct_in->set_longitud_ctrl_setspeed(target_cruise_speed_);
  nnp_fct_in->set_longitud_ctrl_setdistance(lon_ctrl_time_level_);
  nnp_fct_in->set_usr_has_changed_cruise_spd(usr_changed_cruise_speed_);
  acc_mem_ctrl_time_level_ = lon_ctrl_time_level_;
  nnp_cruise_speed_ = target_cruise_speed_;
}

void LonHmi::UpdateNnpActivating(
    functionmanager::FctToNnpInput* const nnp_fct_in,
    const TL::soc::Chassis& chassis) {
  if (last_nnp_fct_out_.has_need_speed_adapt() &&
      last_nnp_fct_out_.need_speed_adapt()) {
    target_cruise_speed_ = last_nnp_fct_out_.adapt_cruise_speed_km();
    usr_changed_cruise_speed_ = use_mem_ ? spd_adapt_.MemState() : false;
  } else {
    CheckUsrAdjustCruiseSpeed(nnp_fct_in, chassis);
  }
  UpdateLonCtrlTimeLevel(nnp_fct_in, chassis);
  acc_mem_cruise_speed_ = target_cruise_speed_;
  nnp_cruise_speed_ = target_cruise_speed_;
  nnp_fct_in->set_longitud_ctrl_setspeed(target_cruise_speed_);
  nnp_fct_in->set_usr_has_changed_cruise_spd(usr_changed_cruise_speed_);
  nnp_fct_in->set_longitud_ctrl_setdistance(lon_ctrl_time_level_);
  acc_mem_ctrl_time_level_ = lon_ctrl_time_level_;
}

void LonHmi::CheckUsrAdjustCruiseSpeed(
    functionmanager::FctToNnpInput* const nnp_fct_in,
    const TL::soc::Chassis& chassis) {
  if (nnp_fct_in == nullptr || nnp_fct_in->is_hands_off_warning_bl() ||
      (acc_active_ &&
       nnp_fct_in->acc_state() ==
           functionmanager::FctToNnpInput::ACC_STANDSTILL_WAIT)) {
    return;
  }

  static bool last_add = false;
  static bool last_minus = false;
  // 用户操作档杆调整巡航速度，下压档杆超过1s，把当前速度设定为仪表车速
  static int gear_d_cnt = 0;
  static bool need_change_speed = false;
  if (chassis.switch_info().gear_position_req_st() == 5) {
    gear_d_cnt++;
  } else {
    gear_d_cnt = 0;
  }
  if (gear_d_cnt > kGearCheckCnt) {
    need_change_speed = true;
  }
  if (need_change_speed && gear_d_cnt == 0) {
    const auto display_speed_imit = static_cast<int32>(
        nnp_fct_in->cdcs_info().cdcs_speed_limit().display_speed_imit());
    target_cruise_speed_ = display_speed_imit > 0
                               ? CruiseSpeedLimiter(display_speed_imit)
                               : speed_display_;
    usr_changed_cruise_speed_ = true;
    need_change_speed = false;
  }
  // 如果方控的模式不是默认的，比如说是在调后视镜或者hud等
  if (chassis.switch_info().swsm_mode() != 0 ||
      chassis.switch_info().sw_error() != 0 ||
      (chassis.switch_info().cruise_speed_add() == 0 &&
       chassis.switch_info().cruise_speed_minus() == 0)) {
    last_add = false;
    last_minus = false;
    return;
  }
  usr_changed_cruise_speed_ = true;
  const auto remainder = target_cruise_speed_ % 5;
  // 缓慢加 +1
  if (chassis.switch_info().cruise_speed_add() == 1 && !last_add) {
    target_cruise_speed_ += 1;
  }
  // 快加 +5
  if (chassis.switch_info().cruise_speed_add() == 2 && !last_add) {
    const auto offset = remainder == 0 ? 5 : (5 - remainder);
    target_cruise_speed_ += offset;
  }
  // 缓慢减 -1
  if (chassis.switch_info().cruise_speed_minus() == 1 && !last_minus) {
    target_cruise_speed_ -= 1;
  }
  // 快减 -5
  if (chassis.switch_info().cruise_speed_minus() == 2 && !last_minus) {
    const auto offset = remainder == 0 ? 5 : remainder;
    target_cruise_speed_ -= offset;
  }
  last_add = (chassis.switch_info().cruise_speed_add() == 1 ||
              chassis.switch_info().cruise_speed_add() == 2);
  last_minus = (chassis.switch_info().cruise_speed_minus() == 1 ||
                chassis.switch_info().cruise_speed_minus() == 2);
  target_cruise_speed_ = CruiseSpeedLimiter(target_cruise_speed_);
}

void LonHmi::UpdateLonCtrlTimeLevel(
    functionmanager::FctToNnpInput* const nnp_fct_in,
    const TL::soc::Chassis& chassis) {
  if (nnp_fct_in == nullptr || nnp_fct_in->is_hands_off_warning_bl()) {
    return;
  }
  lon_ctrl_distance_display_req_ = false;
  static bool last_add = false;
  static bool last_minus = false;
  // 如果方控的模式不是默认的，比如说是在调后视镜或者hud等
  if (chassis.switch_info().swsm_mode() != 0 ||
      chassis.switch_info().sw_error() != 0 ||
      (chassis.switch_info().cruise_distance_add() == 0 &&
       chassis.switch_info().cruise_distance_minus() == 0)) {
    last_add = false;
    last_minus = false;
    return;
  }
  const auto add = chassis.switch_info().swsm_a_cd_add_overtime() == 0 &&
                   chassis.switch_info().cruise_distance_add() == 1;
  if (add && !last_add && (lon_ctrl_time_level_ < kMaxLonCtrlTimeLevel)) {
    lon_ctrl_time_level_++;
  }
  const auto minus = chassis.switch_info().swsm_a_cd_minus_overtime() == 0 &&
                     chassis.switch_info().cruise_distance_minus() == 1;
  if (minus && !last_minus && (lon_ctrl_time_level_ > kMimLonCtrlTimeLevel)) {
    lon_ctrl_time_level_--;
  }
  lon_ctrl_distance_display_req_ = (add && !last_add) || (minus && !last_minus);
  last_add = add;
  last_minus = minus;
}

void LonHmi::ProcessLonFollowInfo(
    TL::functionmanager::FunctionManagerIn* const fct_input,
    const TL::soc::Chassis& chassis) {
  const auto& obs_follow_time_config = hmi_config_.obs_follow_time_conf();
  const auto& acc_overtake_follow_time_conf =
      hmi_config_.acc_overtake_follow_time_conf();
  double long_ctrl_set_time = obs_follow_time_config.default_time();
  double long_ctrl_min_dis = obs_follow_time_config.default_min_dis();
  double ratio =
      obs_follow_time_config.follow_ratio_by_driver_mode().default_ratio();
  const auto driving_mode = fct_input->fct_nnp_in().driving_mode();
  const auto is_acc_overtaking_assistance_start =
      GetAccOvertakingAssistanceStart();
  if (obs_follow_time_config.follow_ratio_by_driver_mode()
          .enable_driver_mode()) {
    switch (driving_mode) {
      case functionmanager::DrivingMode::UNKNOWN_MODE:
        ratio = obs_follow_time_config.follow_ratio_by_driver_mode()
                    .default_ratio();
        long_ctrl_set_time = obs_follow_time_config.level_3_time() * ratio;
        long_ctrl_min_dis = obs_follow_time_config.level_3_min_dis() * ratio;
        break;
      case functionmanager::DrivingMode::BASIC:
        ratio =
            obs_follow_time_config.follow_ratio_by_driver_mode().basic_ratio();
        long_ctrl_set_time = obs_follow_time_config.level_5_time() * ratio;
        long_ctrl_min_dis = obs_follow_time_config.level_5_min_dis() * ratio;
        break;
      case functionmanager::DrivingMode::NORMAL:
        ratio =
            obs_follow_time_config.follow_ratio_by_driver_mode().normal_ratio();
        long_ctrl_set_time = obs_follow_time_config.level_3_time() * ratio;
        long_ctrl_min_dis = obs_follow_time_config.level_3_min_dis() * ratio;
        break;
      case functionmanager::DrivingMode::RADICAL:
        ratio = obs_follow_time_config.follow_ratio_by_driver_mode()
                    .radical_ratio();
        long_ctrl_set_time = obs_follow_time_config.level_1_time() * ratio;
        long_ctrl_min_dis = obs_follow_time_config.level_1_min_dis() * ratio;
        break;
      default:
        ratio = obs_follow_time_config.follow_ratio_by_driver_mode()
                    .default_ratio();
        long_ctrl_set_time = obs_follow_time_config.level_3_time() * ratio;
        long_ctrl_min_dis = obs_follow_time_config.level_3_min_dis() * ratio;
        break;
    }
  }

  if (fct_input->has_fct_nnp_in() &&
      fct_input->fct_nnp_in().has_longitud_ctrl_setdistance()) {
    uint32_t dis_set = fct_input->fct_nnp_in().longitud_ctrl_setdistance();
    auto speed_level = dis_set;
    if (chassis.has_speed_display() &&
        chassis.speed_display() <= hmi_config_.speed_adapt_conf().min_speed()) {
      // 60以下按照10 -1档 ，20 - 2档， 30 -3档
      const auto speed_display = chassis.speed_display();
      if (!FLAGS_use_dynamic_time_gap) {
        speed_level = std::max(static_cast<int>(speed_display / 10), 1);
      }
    }
    dis_set = static_cast<uint32_t>(fmin(speed_level, dis_set));
    switch (dis_set) {
      case 1:
        long_ctrl_set_time = (is_acc_overtaking_assistance_start
                                  ? acc_overtake_follow_time_conf.level_1_time()
                                  : obs_follow_time_config.level_1_time()) *
                             ratio;
        long_ctrl_min_dis = obs_follow_time_config.level_1_min_dis() * ratio;
        break;
      case 2:
        long_ctrl_set_time = (is_acc_overtaking_assistance_start
                                  ? acc_overtake_follow_time_conf.level_2_time()
                                  : obs_follow_time_config.level_2_time()) *
                             ratio;
        long_ctrl_min_dis = obs_follow_time_config.level_2_min_dis() * ratio;
        break;
      case 3:
        long_ctrl_set_time = (is_acc_overtaking_assistance_start
                                  ? acc_overtake_follow_time_conf.level_3_time()
                                  : obs_follow_time_config.level_3_time()) *
                             ratio;
        long_ctrl_min_dis = obs_follow_time_config.level_3_min_dis() * ratio;
        break;
      case 4:
        long_ctrl_set_time = (is_acc_overtaking_assistance_start
                                  ? acc_overtake_follow_time_conf.level_4_time()
                                  : obs_follow_time_config.level_4_time()) *
                             ratio;
        long_ctrl_min_dis = obs_follow_time_config.level_4_min_dis() * ratio;
        break;
      case 5:
        long_ctrl_set_time = (is_acc_overtaking_assistance_start
                                  ? acc_overtake_follow_time_conf.level_5_time()
                                  : obs_follow_time_config.level_5_time()) *
                             ratio;
        long_ctrl_min_dis = obs_follow_time_config.level_5_min_dis() * ratio;
        break;
      default:
        break;
    }
  }
  fct_input->mutable_fct_nnp_in()->set_longitud_ctrl_time(long_ctrl_set_time);
  fct_input->mutable_fct_nnp_in()->set_longitud_ctrl_min_dis(long_ctrl_min_dis);
}

void LonHmi::ProcessAccStandstill(
    functionmanager::FunctionManagerIn* const fct_in,
    const TL::soc::Chassis& chassis) {
  if (fct_in == nullptr) {
    return;
  }
  UNUSED(chassis);
  need_broadcast_ = false;
  if (fct_in->fct_nnp_in().acc_state() !=
      functionmanager::FctToNnpInput::ACC_STANDSTILL_WAIT) {
    return;
  }

  fct_in->mutable_fct_nnp_in()->set_longitud_ctrl_cruise_speedms(-1.0);
  need_broadcast_ = !is_speed_plan_standstill_;
}

void LonHmi::ProcessBaiduSim(functionmanager::FunctionManagerIn* const fct_in) {
  static bool last_nnp_active = false;
  static bool mem_state = false;
  const auto& nnp_fct_in = fct_in->fct_nnp_in();
  const auto& nnp_state = nnp_fct_in.nnp_sysstate();
  static bool state_has_changed = false;

  const auto nnp_active = (nnp_state == NNPSysState::NNPS_ACTIVE ||
                           nnp_state == NNPSysState::NNPS_LAT_OVERRIDE ||
                           nnp_state == NNPSysState::NNPS_LON_OVERRIDE ||
                           nnp_state == NNPSysState::NNPS_OVERRIDE ||
                           nnp_state == NNPSysState::NNPS_TO);
  const auto curr_state = fct_in->fct_nnp_in().usr_has_changed_cruise_spd();
  if (!state_has_changed && last_nnp_active && nnp_active) {
    mem_state = curr_state;
  }
  // nnp状态切到其他，比如polit
  if (last_nnp_active && !nnp_active) {
    state_has_changed = true;
  }
  if (state_has_changed && !curr_state && mem_state) {
    fct_in->mutable_fct_nnp_in()->set_usr_has_changed_cruise_spd(mem_state);
  }

  last_nnp_active = nnp_active;
}

void LonHmi::ProcessStandstillHmi(
    functionmanager::FunctionManagerOut* const fct_out,
    planning::Frame* const frame) {
  if (frame == nullptr || fct_out == nullptr || nnp_active_) {
    return;
  }
  static bool last_broadcast = false;
  static bool broadcast = false;
  const auto* ref_info = frame->FindDriveReferenceLineInfo();
  if (ref_info == nullptr) {
    last_broadcast = false;
    broadcast = false;
    return;
  }
  const auto* front_obs = ref_info->path_decision().Find(frame->LonStopObsId());
  if (front_obs == nullptr || front_obs->IsVirtual()) {
    last_broadcast = false;
    broadcast = false;
    return;
  }
  static double broadcast_start_time = TL::common::Clock::NowInSeconds();
  const auto ped_broadcast = ProcessPedestrian(
      fct_out, front_obs, frame->IsSpeedPlanStandstill(), ref_info);
  const auto standstill_reusme_broadcast =
      ProcessStandstillReusme(fct_out, frame);

  if (!broadcast) {
    broadcast = ped_broadcast || standstill_reusme_broadcast;
  } else if (TL::common::Clock::NowInSeconds() - broadcast_start_time >
             kObsBroadcastTime) {
    broadcast = false;
  }
  if (!last_broadcast && broadcast) {
    broadcast_start_time = TL::common::Clock::NowInSeconds();
  }
  if (broadcast) {
    /*NOOBJECT = 0;
    PEDESTRIAN = 1;
    VEHICLE = 2;
    CYELIST = 3;
    UNKNOWN = 4;
  */
    uint32_t obs_type_broadcast = 0;
    if (front_obs->Perception().type() == PerceptionObstacle::PEDESTRIAN) {
      obs_type_broadcast = 1;
    } else if (front_obs->Perception().type() == PerceptionObstacle::BICYCLE ||
               front_obs->Perception().type() == PerceptionObstacle::CYCLIST) {
      obs_type_broadcast = 3;
    } else if (front_obs->Perception().type() == PerceptionObstacle::VEHICLE) {
      obs_type_broadcast = 2;
    } else {
      obs_type_broadcast = 4;
    }
    ADEBUG << " 有目标，无法起步";
    uint32_t soc_03_val = fct_out->has_soc_2_fct_tbd_u32_03()
                              ? fct_out->soc_2_fct_tbd_u32_03()
                              : 0x00;
    fct_out->set_soc_2_fct_tbd_u32_03(
        soc_03_val | kObsBroadcast |
        (static_cast<uint32_t>(obs_type_broadcast) << 12));
  }
  last_broadcast = broadcast;
}

bool LonHmi::ProcessPedestrian(
    functionmanager::FunctionManagerOut* const fct_out,
    const planning::Obstacle* stop_obs, const bool speed_plan_standstill,
    const ReferenceLineInfo* ref_info) {
  // 自车 --车 变成 自车 -- 行人 --车 的时候，前前车起步，提醒前边有目标，不能起步
  static std::string start_stop_obs_id;
  static PerceptionObstacle::Type start_stop_obs_type =
      PerceptionObstacle::UNKNOWN;
  static bool last_stand_still = false;
  static int32_t start_stop_obs_move_cnt = 0;
  if (fct_out == nullptr || stop_obs == nullptr || !speed_plan_standstill) {
    start_stop_obs_id = "";
    last_stand_still = false;
    last_stand_still = speed_plan_standstill;
    start_stop_obs_type = PerceptionObstacle::UNKNOWN;
    start_stop_obs_move_cnt = 0;
    return false;
  }
  // 进入standstill逻辑的时候，看前方目标是不是车，如果是车，继续等看看会不会变成行人
  if (!last_stand_still && speed_plan_standstill) {
    start_stop_obs_type = stop_obs->Perception().type();
    start_stop_obs_id = stop_obs->Id();
    ADEBUG << " 开始跟停了 , 障碍物类型 : " << start_stop_obs_type;
  }
  if (start_stop_obs_type != PerceptionObstacle::VEHICLE) {
    last_stand_still = speed_plan_standstill;
    start_stop_obs_move_cnt = 0;
    ADEBUG << " 开始跟停的不是车";
    return false;
  }
  if (acc_state_ != functionmanager::FctToNnpInput::ACC_STANDSTILL_WAIT &&
      acc_state_ != functionmanager::FctToNnpInput::ACC_STANDSTILL_ACTIVE) {
    last_stand_still = speed_plan_standstill;
    start_stop_obs_move_cnt = 0;
    return false;
  }
  last_stand_still = speed_plan_standstill;
  // 跟停目标变成了行人
  const auto front_ped =
      stop_obs->Perception().type() == PerceptionObstacle::PEDESTRIAN ||
      stop_obs->Perception().type() == PerceptionObstacle::BICYCLE ||
      stop_obs->Perception().type() == PerceptionObstacle::CYCLIST;
  if (!front_ped) {
    start_stop_obs_move_cnt = 0;
    return false;
  }
  ADEBUG << "跟停目标变成行人了";

  const auto* start_standstill_obs =
      ref_info->path_decision().Find(start_stop_obs_id);

  // 目标消失或者目标不再是静态目标

  const auto start_stop_obs_move =
      start_standstill_obs == nullptr ||
      (start_standstill_obs != nullptr && !start_standstill_obs->IsStatic());
  if (start_stop_obs_move) {
    start_stop_obs_move_cnt++;
  } else {
    start_stop_obs_move_cnt = 0;
  }
  // 起步后报3帧
  return start_stop_obs_move_cnt > 2 && start_stop_obs_move_cnt < 6;
}

bool LonHmi::ProcessStandstillReusme(
    functionmanager::FunctionManagerOut* const fct_out,
    const planning::Frame* frame) {
  if (frame == nullptr || fct_out == nullptr) {
    return false;
  }
  if (acc_state_ != functionmanager::FctToNnpInput::ACC_STANDSTILL_WAIT ||
      !frame->IsSpeedPlanStandstill()) {
    return false;
  }
  const auto& chassis = frame->vehicle_state().chassis();
  return chassis.switch_info().cruise_speed_add() == 2;
}

void LonHmi::UpdateAccActiveObs(
    functionmanager::FunctionManagerOut* const fct_out,
    planning::Frame* const frame) {
  if (fct_out == nullptr || frame == nullptr) {
    return;
  }
  double nearest_obs_dis = 1e5;
  std::string target_obs_id;
  const auto* reference_line_info = frame->FindDriveReferenceLineInfo();
  if (reference_line_info == nullptr) {
    return;
  }
  const auto& obstcales =
      reference_line_info->path_decision().obstacles().Items();
  const auto* blocking_obs = reference_line_info->GetBlockingObstacle();
  const auto lon_follow_obs_id = reference_line_info->GetLonFollowObsId();
  const auto& vehicle_state = reference_line_info->vehicle_state();
  const auto adc_fronts = reference_line_info->AdcSlBoundary().end_s();
  for (const auto& obs : obstcales) {
    if (obs == nullptr || obs->PerceptionSLBoundary().start_s() < adc_fronts ||
        obs->IsVirtual() || !obs->HasLongitudinalDecision() ||
        obs->LongitudinalDecision().has_ignore()) {
      continue;
    }

    if (blocking_obs != nullptr &&
        blocking_obs->PerceptionId() == obs->PerceptionId()) {
      const auto dis = obs->PerceptionSLBoundary().start_s() - adc_fronts;
      if (DefinitelyLessEqual(dis, nearest_obs_dis)) {
        nearest_obs_dis = dis;
        target_obs_id = obs->Id();
      }
      continue;
    }
    if (lon_follow_obs_id == obs->Id()) {
      const auto dis = obs->PerceptionSLBoundary().start_s() - adc_fronts;
      if (DefinitelyLessEqual(dis, nearest_obs_dis)) {
        nearest_obs_dis = dis;
        target_obs_id = obs->Id();
      }
      continue;
    }
    if (!obs->LongitudinalDecision().has_stop()) {
      continue;
    }
    const auto dis = obs->PerceptionSLBoundary().start_s() - adc_fronts;
    if (DefinitelyLessEqual(dis, nearest_obs_dis)) {
      nearest_obs_dis = dis;
      target_obs_id = obs->Id();
    }
  }
  static constexpr uint32_t kValue = 0x01;
  static constexpr double kMinDistance = 20.0;
  static constexpr double kPreviewTime = 5.0;
  const auto preview_distance =
      vehicle_state.linear_velocity() * kPreviewTime + kMinDistance;
  const auto& target_obs =
      reference_line_info->path_decision().Find(target_obs_id);
  uint32_t soc_04_val = fct_out->has_soc_2_fct_tbd_u32_04()
                            ? fct_out->soc_2_fct_tbd_u32_04()
                            : 0x00;
  const auto reverse_obs = target_obs != nullptr && !target_obs->IsStatic() &&
                           target_obs->Perception().has_velocity_flu() &&
                           target_obs->Perception().velocity_flu().x() < -1.0 &&
                           target_obs->Perception().has_theta_flu() &&
                           target_obs->Perception().theta_flu() < -M_PI_2;
  const auto cut_out_obs = target_obs != nullptr && !target_obs->IsStatic() &&
                           target_obs->Perception().has_theta_flu() &&
                           fabs(target_obs->Perception().theta_flu()) > M_PI_4;
  const auto is_vehicle =
      target_obs != nullptr &&
      target_obs->Perception().type() == PerceptionObstacle::VEHICLE;
  if (DefinitelyLessEqual(nearest_obs_dis, preview_distance) && !reverse_obs &&
      is_vehicle && !cut_out_obs) {
    fct_out->set_soc_2_fct_tbd_u32_04(soc_04_val | kValue);
  }
}

void LonHmi::ProcessDriveOffAlert(
    functionmanager::SocToFctBus* const soc_to_fct_bus,
    planning::Frame* const frame) {
  if (frame == nullptr || soc_to_fct_bus == nullptr ||
      !frame->vehicle_state().has_chassis()) {
    return;
  }
  if (0 == drvoffalertswitch_) {
    soc_to_fct_bus->mutable_soc_to_fct_bus_u8()->set_ldw_right_trigger_flag(
        0x0);
    soc_to_fct_bus->mutable_soc_to_fct_bus_u8()->set_ldw_left_lane(0x0);
    soc_to_fct_bus->mutable_soc_to_fct_bus_u8()->set_ldw_right_lane(0x0);
    return;
  }
  static double rolling_backwards_start_time = common::Clock::NowInSeconds();
  static bool last_rolling_backwards = false;
  static int32 alert_cnt = 0;
  static bool need_alert = false;
  static uint8_t alert_state = 0x0;
  const uint8_t error_code = 0x0;
  const auto& chassis = frame->vehicle_state().chassis();
  const auto acc_stand =
      functionmanager::FctToNnpInput::ACC_STANDSTILL_ACTIVE == acc_state_ ||
      functionmanager::FctToNnpInput::ACC_STANDSTILL_WAIT == acc_state_ ||
      (functionmanager::FctToNnpInput::ACC_ACTIVE == acc_state_ &&
       frame->IsVehicleStandStill());
  const auto door_status =
      chassis.door_status().fl_door() != soc::DoorStatus::CLOSED ||
      chassis.door_status().fr_door() != soc::DoorStatus::CLOSED ||
      chassis.door_status().rl_door() != soc::DoorStatus::CLOSED ||
      chassis.door_status().rr_door() != soc::DoorStatus::CLOSED ||
      chassis.hood_ajar_status().status() != soc::HoodAjarStatus::CLOSED ||
      chassis.back_door_status().status() != soc::BackDoorStatus::FULLY_CLOSED;
  const auto driver_buckle_status = chassis.driver_buckle_status().status() !=
                                    soc::DriverBuckleStatus::BUCKLED;
  const auto gear_status =
      chassis.gear_location() == soc::Chassis::GEAR_REVERSE;
  const auto rolling_backwards =
      chassis.wheel_speed().wheel_direction_fl() == soc::WheelSpeed::BACKWARD &&
      chassis.wheel_speed().wheel_direction_rl() == soc::WheelSpeed::BACKWARD &&
      chassis.wheel_speed().wheel_direction_fr() == soc::WheelSpeed::BACKWARD &&
      chassis.wheel_speed().wheel_direction_rr() == soc::WheelSpeed::BACKWARD;
  if (!rolling_backwards || (rolling_backwards && !last_rolling_backwards)) {
    rolling_backwards_start_time = common::Clock::NowInSeconds();
  }
  last_rolling_backwards = rolling_backwards;
  const auto& drv_off_alert_conf = hmi_config_.drv_off_alert_conf();
  const auto adc_rolling_backwards =
      rolling_backwards &&
      ((fabs(chassis.speed_mps()) <
            drv_off_alert_conf.rolling_backwards_speed_threshold() &&
        (common::Clock::NowInSeconds() - rolling_backwards_start_time) >
            drv_off_alert_conf.rolling_backwards_time_threshold()) ||
       common::math::double_type::DefinitelyGreaterEqual(
           fabs(chassis.speed_mps()),
           drv_off_alert_conf.rolling_backwards_speed_threshold()));
  if (acc_stand || door_status || driver_buckle_status || gear_status ||
      adc_rolling_backwards) {
    alert_cnt = 0;
    need_alert = false;
    alert_state = 0x0;
    // set 0x0
    soc_to_fct_bus->mutable_soc_to_fct_bus_u8()->set_ldw_right_trigger_flag(
        alert_state);
    soc_to_fct_bus->mutable_soc_to_fct_bus_u8()->set_ldw_left_lane(error_code);
    soc_to_fct_bus->mutable_soc_to_fct_bus_u8()->set_ldw_right_lane(0x0);
    stop_obs_id_ = 0;
    is_pre_stop_ = false;
    return;
  }
  const auto* reference_line_info = frame->FindDriveReferenceLineInfo();
  if (reference_line_info == nullptr) {
    alert_cnt = 0;
    need_alert = false;
    alert_state = 0x0;
    // set 0x0
    soc_to_fct_bus->mutable_soc_to_fct_bus_u8()->set_ldw_right_trigger_flag(
        alert_state);
    soc_to_fct_bus->mutable_soc_to_fct_bus_u8()->set_ldw_left_lane(error_code);
    soc_to_fct_bus->mutable_soc_to_fct_bus_u8()->set_ldw_right_lane(0x0);
    return;
  }
  auto has_targrt_obs = false;
  const auto obs_drv_off_alert =
      CheckObsDriveOffAlert(reference_line_info, frame, &has_targrt_obs);
  const auto traffic_light_alert =
      !has_targrt_obs && !obs_drv_off_alert &&
      CheckTrafficLightDriveOffAlert(reference_line_info, frame);

  // 信号在报文上发3帧，周期50ms，在soc上发2帧

  uint8_t lamp_mode = 0x0;
  if (obs_drv_off_alert || traffic_light_alert) {
    need_alert = true;
    alert_state = obs_drv_off_alert ? 0x01 : 0x02;
  }
  if (need_alert && alert_cnt < drv_off_alert_conf.alert_cnt()) {
    lamp_mode = alert_state != 0x0 ? 0x9 : 0x0;
    alert_cnt++;
  } else {
    need_alert = false;
    alert_cnt = 0;
    alert_state = 0x0;
  }
  soc_to_fct_bus->mutable_soc_to_fct_bus_u8()->set_ldw_right_trigger_flag(
      alert_state);
  soc_to_fct_bus->mutable_soc_to_fct_bus_u8()->set_ldw_left_lane(error_code);
  soc_to_fct_bus->mutable_soc_to_fct_bus_u8()->set_ldw_right_lane(lamp_mode);
}

bool LonHmi::CheckObsDriveOffAlert(const ReferenceLineInfo* reference_line_info,
                                   planning::Frame* const frame,
                                   bool* const has_targrt_obs) {
  // 当车停下来的时候看是看车前边的目标,8m内有静止目标，目标包括三轮车，两轮车和车辆
  static bool already_alert = false;
  *has_targrt_obs = false;
  const auto& drv_off_alert_conf = hmi_config_.drv_off_alert_conf();
  if (!drv_off_alert_conf.enable_obs_drv_off_alert() ||
      reference_line_info == nullptr || frame == nullptr ||
      has_targrt_obs == nullptr || !frame->IsVehicleStandStill()) {
    is_pre_stop_ = false;
    already_alert = false;
    stop_obs_id_ = 0;
    return false;
  }
  auto nearest_obs_dis = 1e2;
  bool has_stop_obs = false;
  std::string near_obs_id;
  const auto adc_fronts = reference_line_info->AdcSlBoundary().end_s();
  // 理论上应该是只看本车道内的目标,这个时候如果方向盘角度比较大，用了cruise，选不到正前方的目标怎么办
  for (const auto* obs :
       reference_line_info->path_decision().obstacles().Items()) {
    if (obs == nullptr || obs->IsVirtual() || !obs->HasLongitudinalDecision() ||
        obs->path_st_boundary().IsEmpty() ||
        obs->LongitudinalDecision().has_ignore() ||
        obs->PerceptionSLBoundary().start_s() < adc_fronts ||
        obs->PerceptionSLBoundary().start_s() >
            adc_fronts + drv_off_alert_conf.obs_speed_max_distance() ||
        (obs->Perception().has_theta_flu() &&
         fabs(common::math::NormalizeAngle(obs->Perception().theta_flu())) >
             M_PI_4) ||
        (obs->Perception().type() != PerceptionObstacle::VEHICLE &&
         obs->Perception().type() != PerceptionObstacle::BICYCLE &&
         obs->Perception().type() != PerceptionObstacle::CYCLIST)) {
      continue;
    }
    double middle_s = (obs->PerceptionSLBoundary().start_s() +
                       obs->PerceptionSLBoundary().end_s()) /
                      2;
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    reference_line_info->reference_line().GetLaneWidth(
        middle_s, &lane_left_width, &lane_right_width);
    if (obs->PerceptionSLBoundary().start_l() > lane_left_width ||
        obs->PerceptionSLBoundary().end_l() < -lane_right_width) {
      continue;
    }
    *has_targrt_obs = true;
    const auto obs_dis = obs->PerceptionSLBoundary().start_s() - adc_fronts;
    if (DefinitelyLessEqual(obs_dis, nearest_obs_dis)) {
      nearest_obs_dis = obs_dis;
      near_obs_id = obs->Id();
    }
    if (obs->LongitudinalDecision().has_stop()) {
      has_stop_obs = true;
    }
  }

  const auto* nearest_obs =
      reference_line_info->path_decision().Find(near_obs_id);
  // 车辆静止之后，看前边的最近的车是不是stop的，如果没有stop的，则认为不需要提醒
  if (nearest_obs != nullptr &&
      nearest_obs->LongitudinalDecision().has_stop()) {
    stop_obs_id_ = nearest_obs->PerceptionId();
    is_pre_stop_ = true;
  }
  if (!is_pre_stop_) {
    stop_obs_id_ = 0;
    return false;
  }
  const auto* stop_obs =
      reference_line_info->path_decision().FindPerceptionObstacle(stop_obs_id_);
  if (stop_obs == nullptr) {
    return false;
  }

  // 之前最近的目标stop，等没有stop的目标了，最近的目标速度满足阈值了，提醒
  auto dis = std::numeric_limits<double>::infinity();
  for (const auto* obs :
       reference_line_info->path_decision().obstacles().Items()) {
    if (obs->PerceptionId() == stop_obs_id_) {
      dis = obs->PerceptionSLBoundary().start_s() - adc_fronts;
    }
  }

  if (!has_stop_obs && stop_obs->has_velocity_flu() &&
      ObsSpeedStableGreater(stop_obs->velocity_flu().x(),
                            drv_off_alert_conf.obs_min_speed_threshold(),
                            drv_off_alert_conf.obs_max_speed_threshold()) &&
      dis > (drv_off_alert_conf.obs_distance_threshold()) && !already_alert) {
    already_alert = true;
    return true;
  }
  return false;
}

bool LonHmi::CheckTrafficLightDriveOffAlert(
    const ReferenceLineInfo* reference_line_info,
    planning::Frame* const frame) {
  static bool is_pre_red = false;
  static bool last_green = true;
  static double green_start_time = common::Clock::NowInSeconds();
  static bool already_alert = false;
  const auto& drv_off_alert_conf = hmi_config_.drv_off_alert_conf();
  if (!drv_off_alert_conf.enable_traffic_light_drv_off_alert() ||
      reference_line_info == nullptr || frame == nullptr ||
      !frame->IsVehicleStandStill()) {
    is_pre_red = false;
    last_green = true;
    already_alert = false;
    return false;
  }
  const auto& local_view = frame->local_view();
  if (!local_view.HasTrafficLightDetection() ||
      !local_view.GetTrafficLightDetection()->has_passable_info()) {
    last_green = true;
    already_alert = false;
    return false;
  }
  const auto lights = local_view.GetTrafficLightDetection()->passable_info();
  // todo get color
  if (!lights.has_straight()) {
    is_pre_red = false;
    last_green = true;
    already_alert = false;
    return false;
  }
  const auto light_color = lights.straight();
  const auto green = TL::perception::Color::GREEN == light_color;
  if (!green) {
    is_pre_red = true;
  }
  if (!is_pre_red) {
    is_pre_red = false;
    last_green = true;
    already_alert = false;
    return false;
  }
  if (!green || (!last_green && green)) {
    green_start_time = common::Clock::NowInSeconds();
  }
  last_green = green;
  if ((common::Clock::NowInSeconds() - green_start_time >
           drv_off_alert_conf.traffic_light_green_time_threshold() &&
       !already_alert)) {
    already_alert = true;
    return true;
  }
  return false;
}

void LonHmi::CruiseDistanceDisplayReq(
    functionmanager::SocToFctBus* const soc_to_fct_bus,
    functionmanager::FunctionManagerOut* const fct_out) const {
  if (soc_to_fct_bus == nullptr || fct_out == nullptr) {
    return;
  }
  fct_out->mutable_nnp_fct_out()->set_adcs_longitud_ctrl_setdistance(
      lon_ctrl_time_level_);
  // 板间是2个bit，需要拼接，最大值是5
  if (lon_ctrl_distance_display_req_) {
    if (lon_ctrl_time_level_ > kDefaultLonCtrlTimeLevel) {
      fct_out->mutable_nnp_fct_out()->set_spdadapt_comfirm_feedback(
          kDefaultLonCtrlTimeLevel);
      fct_out->mutable_nnp_fct_out()->set_paymode_confirm_feedback(
          lon_ctrl_time_level_ - kDefaultLonCtrlTimeLevel);
    } else {
      fct_out->mutable_nnp_fct_out()->set_spdadapt_comfirm_feedback(
          lon_ctrl_time_level_);
      fct_out->mutable_nnp_fct_out()->set_paymode_confirm_feedback(0);
    }
  } else {
    fct_out->mutable_nnp_fct_out()->set_spdadapt_comfirm_feedback(0);
    fct_out->mutable_nnp_fct_out()->set_paymode_confirm_feedback(0);
  }
  soc_to_fct_bus->mutable_soc_to_fct_bus_u8()->set_lane_line_status(
      static_cast<uint32>(lon_ctrl_distance_display_req_));
}

void LonHmi::ProcessTrafficJam(
    const functionmanager::FunctionManagerIn* fct_in) {
  // if (fct_in == nullptr) {
  //   return;
  // }
  // const auto& cdcs_traffic_jam =
  //     fct_in->fct_nnp_in().cdcs_info().cdcs_traffic_jam();
  // const auto road_speed_limit =
  //     fct_in->fct_nnp_in().cdcs_info().cdcs_speed_limit().display_speed_imit() >
  //             60
  //         ? fct_in->fct_nnp_in()
  //               .cdcs_info()
  //               .cdcs_speed_limit()
  //               .display_speed_imit()
  //         : 60;
  // const auto is_cdcs_traffic_jam =
  //     cdcs_traffic_jam.traffic_jam_type() ==
  //         functionmanager::CDCSTtafficJam::SLOW ||
  //     cdcs_traffic_jam.traffic_jam_type() ==
  //         functionmanager::CDCSTtafficJam::TRAFFIC_JAMMED ||
  //     cdcs_traffic_jam.traffic_jam_type() ==
  //         functionmanager::CDCSTtafficJam::TRAFFIC_JAMMED_SERIOUSLY;
  // if (is_cdcs_traffic_jam) {}
}

void LonHmi::CheckUsrReset(functionmanager::FunctionManagerIn* const fct_in,
                           const TL::soc::Chassis& chassis) {
  if ((chassis.reset_switch().factory_reset() == soc::ResetSwitch::RESET ||
       chassis.reset_switch().reset_all_setup() == soc::ResetSwitch::RESET)) {
    reset_ = true;
    acc_mem_ctrl_time_level_ = 0;
  }
  lon_ctrl_set_dis_from_mem_ =
      reset_ ? 0
             : static_cast<int32>(
                   fct_in->fct_nnp_in().longitud_ctrl_setdistance());
}

void LonHmi::CheckForceReplan(planning::Frame* const frame) {
  static int last_plan_standstill = -1;
  // 3度
  static constexpr double kMaxHeadingError = 0.05235987756;
  ForceRplanType force_replan_type = NO_FORCE_REPLAN;
  if (frame == nullptr || !frame->local_view().HasVehicleState() ||
      (acc_state_ != functionmanager::FctToNnpInput::ACC_STANDSTILL_WAIT &&
       acc_state_ != functionmanager::FctToNnpInput::ACC_STANDSTILL_ACTIVE) ||
      !frame->local_view().HasFunctionManagerIn() ||
      !frame->IsVehicleStandStill()) {
    last_plan_standstill = 0;
    return;
  }
  const auto& vehicle_state = frame->local_view().GetVehicleState();
  const auto& fct_in = frame->local_view().GetFunctionManagerIn();
  if (fct_in == nullptr || vehicle_state == nullptr ||
      fct_in->ta_pilot_mode() == functionmanager::AVP) {
    last_plan_standstill = 0;
    frame->SetForceRplanType(NO_FORCE_REPLAN);
    return;
  }
  const auto plan_standstill = frame->IsSpeedPlanStandstill();
  // 跟停的时候,判断自车的heading和init point的heading,误差大横向replan一下
  const auto heading_error = common::math::NormalizeAngle(
      fabs(vehicle_state->heading() -
           frame->PlanningStartPoint().path_point().theta()));
  if (plan_standstill && last_plan_standstill == 0 &&
      vehicle_state->driving_mode() == soc::Chassis::COMPLETE_AUTO_DRIVE &&
      heading_error > kMaxHeadingError) {
    ADEBUG << "stop heading_error : " << heading_error;
    frame->SetForceRplanType(LAT_FORCE_REPLAN);
  }
  last_plan_standstill = static_cast<int>(plan_standstill);
}

void LonHmi::CheckStopObsDisappear(
    planning::Frame* frame,
    functionmanager::FunctionManagerOut* const fct_out) {
  static constexpr double kMaxDistance = 5.0;
  bool near_obs_disappear = false;
  static int32_t last_near_obs_id = -1;
  int32_t curr_perception_near_obs_id = -1;
  std::string curr_near_obs_id;
  auto near_dis = std::numeric_limits<double>::max();
  if (frame == nullptr || fct_out == nullptr) {
    near_obs_disappear = false;
    last_near_obs_id = -1;
    return;
  }
  const auto* reference_line_info = frame->FindDriveReferenceLineInfo();
  if (reference_line_info == nullptr) {
    near_obs_disappear = false;
    last_near_obs_id = -1;
    return;
  }
  if ((acc_state_ != functionmanager::FctToNnpInput::ACC_STANDSTILL_ACTIVE &&
       acc_state_ != functionmanager::FctToNnpInput::ACC_STANDSTILL_WAIT) ||
      !frame->IsVehicleStandStill()) {
    near_obs_disappear = false;
    last_near_obs_id = -1;
    return;
  }
  const auto adc_front_s = reference_line_info->AdcSlBoundary().end_s();
  const auto adc_back_s = reference_line_info->AdcSlBoundary().start_s();
  for (const auto* obs :
       reference_line_info->path_decision().obstacles().Items()) {
    if (obs == nullptr || obs->IsVirtual() ||
        obs->Perception().type() != perception::PerceptionObstacle::VEHICLE ||
        obs->PerceptionSLBoundary().end_s() < adc_back_s ||
        obs->PerceptionSLBoundary().start_s() > (adc_front_s + 5) ||
        !reference_line_info->reference_line().IsOnLane(
            obs->PerceptionSLBoundary())) {
      continue;
    }
    const auto dis = obs->PerceptionSLBoundary().end_s() - adc_back_s;
    if (dis < near_dis) {
      curr_perception_near_obs_id = obs->PerceptionId();
      curr_near_obs_id = obs->Id();
      near_dis = dis;
    }
  }
  if (last_near_obs_id == -1 && curr_perception_near_obs_id == -1) {
    near_obs_disappear = false;
    last_near_obs_id = -1;
    return;
  }
  if (last_near_obs_id != -1 && curr_perception_near_obs_id == -1) {
    const auto* last_near_obs =
        reference_line_info->path_decision().FindPerceptionObstacle(
            last_near_obs_id);
    near_obs_disappear = (last_near_obs == nullptr);
    if (near_obs_disappear) {
      uint32_t soc_03_val = fct_out->has_soc_2_fct_tbd_u32_03()
                                ? fct_out->soc_2_fct_tbd_u32_03()
                                : 0x00;
      soc_03_val =
          soc_03_val | (static_cast<uint32_t>(near_obs_disappear) << 17);
      AERROR << "near_obs_disappear :  " << near_obs_disappear
             << " soc_03_val : " << soc_03_val
             << " 前方5m静止目标丢失，功能退出";
      fct_out->set_soc_2_fct_tbd_u32_03(soc_03_val);
    }

    last_near_obs_id = -1;
    return;
  }
  const auto* curr_near_obs =
      reference_line_info->path_decision().Find(curr_near_obs_id);
  if (curr_near_obs == nullptr || !curr_near_obs->IsStatic() ||
      curr_near_obs->speed() > 1.0) {
    near_obs_disappear = false;
    last_near_obs_id = -1;
    return;
  }
  last_near_obs_id = curr_perception_near_obs_id;
}
}  // namespace planning
}  // namespace TL
