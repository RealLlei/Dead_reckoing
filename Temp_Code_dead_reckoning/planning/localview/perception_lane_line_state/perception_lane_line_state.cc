//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#include "planning/localview/perception_lane_line_state/perception_lane_line_state.h"

#include <memory>
#include <string>

#include "common/util/message_util.h"
#include "map/hdmap/hdmap_util.h"

#include "proto/fsm/nnp_fct.pb.h"
#include "proto/map/navigation.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {
using TL::functionmanager::FctToNnpInput;
using TL::functionmanager::NNPSysState;
using TL::soc::Chassis;
constexpr double kMainLoopTime = 0.1;

PerceptionLaneLineState::PerceptionLaneLineState()
    : nolane_rise_debounce_{0.1, 0.0, kMainLoopTime} {}

bool PerceptionLaneLineState::Init() {
  return true;
}

Status PerceptionLaneLineState::BuildLocalView(
    const std::shared_ptr<LocalView>& local_view) {
  auto laneline_flag = local_view->GetFunctionManagerOut()->laneline_status();
  // auto nolane_flag = nolane_rise_debounce_.DealDebounce(
  auto nolane_flag = local_view->GetFunctionManagerOut()->nolane_status();
  auto cruise_flag = local_view->GetFunctionManagerOut()->cruise_status();
  const bool nolane_mode_first_flag = DeciderObsBeforeVehicle(local_view) ||
                                      FLAGS_nolane_mode_first_for_simulation;
  NNPSysState original_nnp_state = NNPSysState::NNPS_OFF;
  if (local_view->HasFunctionManagerIn()) {
    auto fct_in = local_view->GetFunctionManagerIn()->fct_nnp_in();
    original_nnp_state = fct_in.nnp_original_state();
  }

  const bool original_lat_override_flag =
      original_nnp_state == NNPSysState::NNPS_LAT_OVERRIDE;
  const auto drive_mode = local_view->GetChassis()->driving_mode();
  const bool drive_speed_only_flag = drive_mode == Chassis::AUTO_SPEED_ONLY;

  const auto is_turn_signal =
      local_view->HasChassis() &&
      local_view->GetChassis()->signal().turn_signal() !=
          common::VehicleSignal::TURN_NONE;

  is_change_dueto_internalreasons_ =
      is_change_dueto_internalreasons_ &&
      local_view_data_->update_data()->is_vehicle_active() &&
      (local_view->GetFunctionManagerOut()
               ->nnp_fct_out()
               .nnp_statechange_conditions()
               .location_err_state() != 2 ||
       is_turn_signal);
  initiative_degrate_bl_ =
      initiative_degrate_bl_ &&
      (local_view_data_->update_data()->is_nnp_npilot() ||
       local_view_data_->update_data()->is_nnp_drive_auto());
  // 内部降级情况下没有巡航模式
  // cruise_flag = !is_change_dueto_internalreasons_ && cruise_flag;
  ADEBUG << "original_nnp_state: " << original_nnp_state
         << ", drive_mode: " << drive_mode;
  if (!local_view_data_->update_data()->is_cruise_check()) {
    laneline_flag = false;
    nolane_flag = false;
  }
  SubStateDecider(laneline_flag, nolane_flag, cruise_flag,
                  nolane_mode_first_flag, drive_speed_only_flag,
                  original_lat_override_flag);
  if (cur_sub_state_ == functionmanager::CRUISE_TYPE) {
    is_change_dueto_internalreasons_ = false;
  }
  ADEBUG << "PER cur_sub_state: " << cur_sub_state_;
  history_laneline_flag_ =
      local_view->GetFunctionManagerOut()->laneline_status();
  history_sub_state_ = cur_sub_state_;
  return Status::OK();
}

void PerceptionLaneLineState::SubStateDecider(
    const bool laneline_flag, const bool nolane_flag, const bool cruise_flag,
    const bool nolane_first_flag, [[maybe_unused]] const bool speed_only_flag,
    const bool original_lat_override_flag) {
  ADEBUG << " laneline_flag: " << laneline_flag
         << ", nolane_flag: " << nolane_flag << ", cruise_flag: " << cruise_flag
         << ", FLAGS_not_using_nolane: " << FLAGS_not_using_nolane;
  constexpr double kMinTime = 0.5;
  switch (cur_sub_state_) {
    case functionmanager::SUB_INITIAL_TYPE:
      sub_state_time_ += kMainLoopTime;
      if (nolane_first_flag && !FLAGS_not_using_nolane &&
          nolane_flag) {  // NOLINT
        cur_sub_state_ = functionmanager::NOLANE_TYPE;
        sub_state_time_ = 0.0;
      } else if (laneline_flag) {
        cur_sub_state_ = functionmanager::LANELINE_TYPE;
        sub_state_time_ = 0.0;
      } else if (!FLAGS_not_using_nolane && nolane_flag) {
        cur_sub_state_ = functionmanager::NOLANE_TYPE;
        sub_state_time_ = 0.0;
      } else if (!FLAGS_not_using_cruise && cruise_flag) {
        cur_sub_state_ = functionmanager::CRUISE_TYPE;
        sub_state_time_ = 0.0;
      }
      break;
    case functionmanager::LANELINE_TYPE:
      sub_state_time_ += kMainLoopTime;
      if (sub_state_time_ < kMinTime && laneline_flag) {
        ADEBUG << "LANELINE sub_state_time" << sub_state_time_;
        break;
      }
      // 暂时屏蔽车辆由仅仅控制纵向时自动跳转到巡航模式
      // if (speed_only_flag && cruise_flag) {
      //   cur_sub_state_ = functionmanager::CRUISE_TYPE;
      //   sub_state_time_ = 0.0;
      //   break;
      // }
      if ((!laneline_flag || nolane_first_flag) && !FLAGS_not_using_nolane &&
          nolane_flag) {
        cur_sub_state_ = functionmanager::NOLANE_TYPE;
        sub_state_time_ = 0.0;
      } else if (!laneline_flag && !FLAGS_not_using_cruise && cruise_flag) {
        cur_sub_state_ = functionmanager::CRUISE_TYPE;
        sub_state_time_ = 0.0;
      } else if (!laneline_flag) {
        cur_sub_state_ = functionmanager::SUB_INITIAL_TYPE;
        sub_state_time_ = 0.0;
      }
      break;
    case functionmanager::NOLANE_TYPE:
      sub_state_time_ += kMainLoopTime;
      if (sub_state_time_ < kMinTime && nolane_flag) {
        ADEBUG << "NOLANE sub_state_time: " << sub_state_time_;
        break;
      }
      // 暂时屏蔽车辆由仅仅控制纵向时自动跳转到巡航模式
      // if (speed_only_flag && cruise_flag) {
      //   cur_sub_state_ = functionmanager::CRUISE_TYPE;
      //   sub_state_time_ = 0.0;
      //   break;
      // }
      if (laneline_flag && !nolane_first_flag) {
        cur_sub_state_ = functionmanager::LANELINE_TYPE;
        sub_state_time_ = 0.0;
      } else if (!nolane_flag && !FLAGS_not_using_cruise && cruise_flag) {
        cur_sub_state_ = functionmanager::CRUISE_TYPE;
        sub_state_time_ = 0.0;
      } else if (!nolane_flag) {
        cur_sub_state_ = functionmanager::SUB_INITIAL_TYPE;
        sub_state_time_ = 0.0;
      }
      break;
    case functionmanager::CRUISE_TYPE:
      sub_state_time_ += kMainLoopTime;
      if (sub_state_time_ < kMinTime && cruise_flag) {
        ADEBUG << "CRUISE sub_state_time: " << sub_state_time_;
        break;
      }
      if (original_lat_override_flag && cruise_flag) {
        break;
      }
      if (nolane_first_flag && !FLAGS_not_using_nolane &&
          nolane_flag) {  // NOLINT
        // cur_sub_state_ = functionmanager::NOLANE_TYPE;
        // sub_state_time_ = 0.0;
      } else if (laneline_flag) {
        cur_sub_state_ = functionmanager::LANELINE_TYPE;
        sub_state_time_ = 0.0;
      } else if (!FLAGS_not_using_nolane && nolane_flag) {
        // cur_sub_state_ = functionmanager::NOLANE_TYPE;
        // sub_state_time_ = 0.0;
      } else if (!cruise_flag) {
        cur_sub_state_ = functionmanager::SUB_INITIAL_TYPE;
        sub_state_time_ = 0.0;
      }
      break;
    default:
      break;
  }
}

bool PerceptionLaneLineState::DeciderObsBeforeVehicle(
    const std::shared_ptr<LocalView>& local_view) {
  bool has_obs_flag = false;
  const TL::common::VehicleState& vehicle_state =
      *local_view->GetVehicleState();
  const TL::perception::PerceptionObstacles& perception_obstacles =
      *local_view->GetPerceptionObstacles();
  const double max_front_distance = 20.0;
  const double max_lat_distance = 2.5;
  const double min_speed = 4.167;
  for (const auto& obs : perception_obstacles.perception_obstacle()) {
    if (obs.position_flu().x() > 0 &&
        obs.position_flu().x() < max_front_distance &&
        std::abs(obs.position_flu().y()) < max_lat_distance &&
        obs.velocity_flu().x() < min_speed) {
      has_obs_flag = true;
      break;
    }
  }
  const bool speed_flag = vehicle_state.linear_velocity() < min_speed;
  ADEBUG << "has_obs_flag: " << has_obs_flag << ", speed_flag: " << speed_flag;
  // return has_obs_flag && speed_flag;
  // 关闭低速优先跟车模式
  return false;
}

Status PerceptionLaneLineState::GetSubStatus(
    const std::shared_ptr<LocalView>& local_view) {
  auto fmo = local_view->GetFunctionManagerOut();
  bool status = false;
  switch (cur_sub_state_) {
    case functionmanager::LANELINE_TYPE:
      status = fmo->laneline_status();
      break;
    case functionmanager::NOLANE_TYPE:
      status = fmo->nolane_status();
      break;
    case functionmanager::CRUISE_TYPE:
      status = fmo->cruise_status();
      break;
    default:
      break;
  }
  if (status) {
    return Status::OK();
  }

  return Status(ErrorCode::LOCALVIEW_FSM_PERCEPTION_ERROR, "GetSubStatus fail");
}

bool PerceptionLaneLineState::DealChangeModeConditions(
    const std::shared_ptr<LocalView>& local_view) {
  // 将fct复制出来修改后重新放回local_view
  auto fct_out = *local_view->GetFunctionManagerOut();
  BuildLocalView(local_view);
  fct_out.mutable_nnp_fct_out()
      ->mutable_nnp_statechange_conditions()
      ->set_is_change_dueto_internalreasons(is_change_dueto_internalreasons_);
  fct_out.mutable_nnp_fct_out()->set_initiative_degrate_bl(
      initiative_degrate_bl_);
  const auto nnp_fct_out = fct_out.nnp_fct_out();
  auto* nnp_activation_conditions =
      fct_out.mutable_nnp_fct_out()->mutable_nnp_activation_conditions();
  auto ehp_hdmap_status = fct_out.ehp_hdmap_status();
  auto local_hdmap_status = fct_out.local_hdmap_status();
  auto map_fusion_status = fct_out.map_fusion_hdmap_status();
  ADEBUG << "ehp_hdmap_status:" << ehp_hdmap_status
         << " ,local_hdmap_status: " << local_hdmap_status
         << " ,map_fusion_status: " << map_fusion_status;
  if (ehp_hdmap_status) {
    nnp_activation_conditions->CopyFrom(nnp_fct_out.hd_map_active_status());
    nnp_activation_conditions->set_vehicle_in_hdmap(
        nnp_fct_out.hd_map_active_status().vehicle_in_hdmap() &&
        ehp_hdmap_status);
  } else if (local_hdmap_status) {
    nnp_activation_conditions->CopyFrom(nnp_fct_out.local_map_active_status());
    nnp_activation_conditions->set_vehicle_in_hdmap(
        nnp_fct_out.local_map_active_status().vehicle_in_hdmap() &&
        local_hdmap_status);
  } else if (map_fusion_status) {
    nnp_activation_conditions->CopyFrom(nnp_fct_out.hd_map_active_status());
    nnp_activation_conditions->set_vehicle_in_hdmap(
        nnp_fct_out.hd_map_active_status().vehicle_in_hdmap() &&
        map_fusion_status);
  }

  auto nnp_active_status =
      nnp_activation_conditions->vehicle_in_hdmap() &&
      nnp_activation_conditions->valid_of_lane_localization() &&
      nnp_activation_conditions->valid_of_lane_routing() &&
      nnp_activation_conditions->vehicle_not_in_forbidlane() &&
      nnp_activation_conditions->vehicle_not_in_reverselane() &&
      nnp_activation_conditions->vehicle_not_in_otherforbidarea() &&
      nnp_activation_conditions->appropriate_current_lane_curve() &&
      nnp_activation_conditions->appropriate_current_lane_headingerr() &&
      nnp_activation_conditions->appropriate_current_lane_width();
  auto nnp_grate_bl = nnp_activation_conditions->vehicle_in_hdmap() &&
                      nnp_activation_conditions->valid_of_lane_localization() &&
                      nnp_activation_conditions->valid_of_lane_routing();

  const bool is_nnp_auto = is_nnp_auto_debounce_.DealDebounce(
      local_view_data_->update_data()->is_nnp_drive_auto());
  const auto pilot_sys_state =
      local_view->GetFunctionManagerIn()->fct_nnp_in().npilot_state();
  const bool is_pilot_standby =
      pilot_sys_state == FctToNnpInput::PILOT_STANDBY ||
      local_view_data_->update_data()->is_pilot_drive_auto();
  auto nnp_main_switch_bl = local_view->GetFunctionManagerIn()
                                ->nnp_hmi_signals()
                                .fct_is_nnpmainswitch_bl();
  // nnp_active_onoffset/nnpsndstate: 1->off  2:on
  auto nnp_avtive_action = local_view->GetFunctionManagerIn()
                               ->nnp_hmi_signals()
                               .nnp_active_onoffset();
  // orin平台nnp自动开启开关是一直开启的无法关闭
  bool nnp_auto_onoffset = local_view->GetFunctionManagerIn()
                               ->nnp_switch_conditions()
                               .nnp_autoonoffset();
#ifdef ISORIN
  nnp_auto_onoffset = true;
  initiative_degrate_bl_ = false;
  is_change_dueto_internalreasons_ = false;
  nnp_main_switch_bl = true;
#endif
  auto nnp_psnd_state =
      local_view->GetFunctionManagerIn()->nnp_hmi_signals().nnpsndstate();
  bool initiative_grate_bl{false};
  if (local_view_data_->update_data()->is_pilot_drive_auto() &&
      (is_nnp_auto || local_view_data_->update_data()->is_nnp_npilot()) &&
      (nnp_avtive_action == functionmanager::NnpHmiSignals::On ||
       nnp_psnd_state == functionmanager::NnpHmiSignals::On)) {
    initiative_grate_bl = true;
  }
  fct_out.mutable_nnp_fct_out()->set_initiative_grate_bl(initiative_grate_bl);
  ADEBUG << "is odd region = "
         << nnp_fct_out.nnp_statechange_conditions().is_odd_region()
         << ", is_change_dueto_internalreasons_ = "
         << is_change_dueto_internalreasons_;
  // 地图模式能激活才会跳转到地图模式
  // 1.自仿真下激活条件满足，跳转到地图模式
  // 2.nnp主开关打开情况下：
  //   1）非内部降级，非主动降级情况下，nnp激活条件满足且自动升级开关打开的情况下，跳转到地图模式
  //   2）主动降级情况下，主动升级跳转到地图模式
  //   3）内部降级情况下，激活条件满足跳转到地图模式
  //   4）在非内部降级条件下，nnp处于激活状态则跳转到地图模式(为了防止某些情况下未跳转问题)
  // AERROR << "perception FLAGS_enable_planning_self_simulator: "
  //        << FLAGS_enable_planning_self_simulator
  //        << " ,nnp_active_status: " << nnp_active_status
  //        << " , nnp_main_switch_bl: " << nnp_main_switch_bl
  //        << " , nnp_auto_onoffset: " << nnp_auto_onoffset
  //        << ", is_pilot_active:" << is_pilot_active
  //        << ", is_pilot_standby:" << is_pilot_standby
  //        << " , initiative_degrate_bl_: " << initiative_degrate_bl_
  //        << " , is_nnp_auto: " << is_nnp_auto
  //        << " , nnp_grate_bl: " << nnp_grate_bl;
  bool is_change{false};
#ifdef FOR_BAIDU_SIMULATION
  if (nnp_activation_conditions->vehicle_in_hdmap()) {
    is_change = true;
  } else {
    AERROR << "vehicle NOT in hdmap is_change: " << is_change;
  }
  is_change = true;
#endif
  // PILOT激活情况下，必须处于非压线和道路宽度满足的情况下才跳转到建图
  const bool is_mapfusion_bl =
      map_fusion_status &&
      (local_view_data_->update_data()->is_pilot_drive_auto()
           ? (nnp_activation_conditions->appropriate_current_lane_width() &&
              nnp_activation_conditions->vehicle_not_in_otherforbidarea())
           : true);
  // PILOT激活情况下，必须处于非路口或者路口状态下跟车跟线都失败才能跳转到建图
  bool is_adc_lane_virtual =
      ((local_view_data_->update_data()->is_adc_lane_virtual() ||
        local_view_data_->update_data()->is_virtual_map2nolane_bl()) &&
       local_view->GetFunctionManagerOut()->nolane_status());
  if (is_adc_lane_virtual) {
    nnp_grate_bl = false;
    nnp_active_status = false;
  }
  if (FLAGS_enable_planning_self_simulator || FLAGS_is_record_replay ||
      FLAGS_is_mcap_replay) {
    is_change = nnp_grate_bl;
  } else if (nnp_main_switch_bl &&
             (((nnp_auto_onoffset ||
                !local_view_data_->update_data()->is_pilot_drive_auto()) &&
               nnp_active_status && is_pilot_standby &&
               !initiative_degrate_bl_ && !is_change_dueto_internalreasons_) ||
              (initiative_grate_bl && initiative_degrate_bl_) ||
              (nnp_active_status && is_change_dueto_internalreasons_) ||
              (is_nnp_auto && !is_change_dueto_internalreasons_ &&
               nnp_grate_bl) ||
              (nnp_grate_bl && is_mapfusion_bl && !local_hdmap_status))) {
    is_change = true;
  }

  if (!is_change && is_change_dueto_internalreasons_) {
    nnp_activation_conditions->set_vehicle_in_hdmap(true);
    nnp_activation_conditions->set_valid_of_lane_localization(true);
    nnp_activation_conditions->set_valid_of_lane_routing(true);
    nnp_activation_conditions->set_vehicle_not_in_forbidlane(true);
    nnp_activation_conditions->set_vehicle_not_in_otherforbidarea(true);
  }
  local_view->SetFunctionManagerOutPtr(
      std::make_shared<functionmanager::FunctionManagerOut>(fct_out));

  // (!FLAGS_enable_odd_area_internal_to_perception ||
  //  !is_change_dueto_internalreasons_);
  return is_change;
}

void PerceptionLaneLineState::SetHdmapActionCondations(
    const std::shared_ptr<LocalView>& local_view) const {
  auto fct_out = *local_view->GetFunctionManagerOut();
  const auto nnp_fct_out = fct_out.nnp_fct_out();
  auto* nnp_activation_conditions =
      fct_out.mutable_nnp_fct_out()->mutable_nnp_activation_conditions();
  auto ehp_hdmap_status = fct_out.ehp_hdmap_status();
  auto local_hdmap_status = fct_out.local_hdmap_status();
  if (ehp_hdmap_status) {
    nnp_activation_conditions->CopyFrom(nnp_fct_out.hd_map_active_status());
    nnp_activation_conditions->set_vehicle_in_hdmap(
        nnp_fct_out.hd_map_active_status().vehicle_in_hdmap() &&
        ehp_hdmap_status);
  } else if (local_hdmap_status) {
    nnp_activation_conditions->CopyFrom(nnp_fct_out.local_map_active_status());
    nnp_activation_conditions->set_vehicle_in_hdmap(
        nnp_fct_out.local_map_active_status().vehicle_in_hdmap() &&
        local_hdmap_status);
  }

  if (is_change_dueto_internalreasons_) {
    nnp_activation_conditions->set_vehicle_in_hdmap(true);
    nnp_activation_conditions->set_valid_of_lane_localization(true);
    nnp_activation_conditions->set_valid_of_lane_routing(true);
    nnp_activation_conditions->set_vehicle_not_in_forbidlane(true);
    nnp_activation_conditions->set_vehicle_not_in_otherforbidarea(true);
  }
  local_view->SetFunctionManagerOutPtr(
      std::make_shared<functionmanager::FunctionManagerOut>(fct_out));
}
}  // namespace planning
}  // namespace TL
