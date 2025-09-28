/******************************************************************************
 * Copyright 2018 The TL Authors. All Rights Reserved.
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
 * @LastEditors: Liu Bei
 *****************************************************************************/
#include "planning/localview/hdmap_lane_line_state/hdmap_lane_line_state.h"

#include <cmath>
#include <vector>

#include "common/file/file.h"
#include "common/util/message_util.h"
#include "common/util/point_factory.h"
#include "common/util/util.h"
#include "map/hdmap/hdmap_util.h"

#include "proto/fsm/function_manager.pb.h"
#include "proto/routing/poi.pb.h"

namespace TL {
namespace planning {

HDMapLaneLineState::HDMapLaneLineState()
    : is_ehp_mode_debounce_{2.0, 0.0, 0.1},
      is_local_mode_debounce_{2.0, 0.0, 0.1} {}

bool HDMapLaneLineState::Init() {
  return true;
}

/**
 * @brief build local view
 * 
 * @param local_view 
 * @return common::Status errorcode and error message
 */
common::Status HDMapLaneLineState::BuildLocalView(
    const std::shared_ptr<LocalView>& local_view) {
  ADEBUG << GetPtr(local_view);
  SubStateDecider(local_view);
  const std::string msg = "HDMapLaneLineState BuildLocalView failed";
  return Status::OK();
}

common::Status HDMapLaneLineState::BuildLocalView(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<functionmanager::FunctionManagerOut>& ehp_fct_out) {
  ADEBUG << GetPtr(local_view);
  SubStateDecider(local_view);

  // 在地图重合区域，EHP模式优先且EHP模式先运行，如果EHP和本地地图都成功了，需要还原回EHP的部分激活判断条件
  if (cur_sub_state_ == functionmanager::EHP_HDMAP_TYPE) {
    auto fct_out = *local_view->GetFunctionManagerOut();
    auto* nnp_fct_out = fct_out.mutable_nnp_fct_out();
    nnp_fct_out->mutable_nnp_statechange_conditions()->CopyFrom(
        ehp_fct_out->nnp_fct_out().nnp_statechange_conditions());
    fct_out.mutable_odd_info()->CopyFrom(ehp_fct_out->odd_info());
    fct_out.set_adc_passage_remain_len(ehp_fct_out->adc_passage_remain_len());
    nnp_fct_out->set_nnp_d_distance_outof_odd_sg(
        ehp_fct_out->nnp_fct_out().nnp_d_distance_outof_odd_sg());
    // nnp_fct_out->set_nnp_d_distance_into_odd_sg(
    //     ehp_fct_out->nnp_fct_out().nnp_d_distance_into_odd_sg());
    local_view->SetFunctionManagerOutPtr(
        std::make_shared<functionmanager::FunctionManagerOut>(fct_out));
  }

  const std::string msg = "HDMapLaneLineState BuildLocalView failed";
  return Status::OK();
}

bool HDMapLaneLineState::DealChangeModeConditions(
    const std::shared_ptr<LocalView>& local_view) {
  bool hdmap_fg = true;
  auto fct_out = *local_view->GetFunctionManagerOut();
  const auto nnp_fct_out = fct_out.nnp_fct_out();
  const bool odd_faild =
      nnp_fct_out.nnp_statechange_conditions().is_change_mode_by_odd_type();
  const int localization_state =
      nnp_fct_out.nnp_statechange_conditions().location_err_state();  // NOLINT
  const bool good_lane = fct_out.laneline_status();
  const bool is_not_fusionmap =
      fct_out.has_localization_maptype()
          ? fct_out.localization_maptype() ==
                TL::navigation_hdmap::MapMsg_MapType_INVALID
          : true;
  auto ehp_hdmap_status = fct_out.ehp_hdmap_status();
  auto local_hdmap_status = fct_out.local_hdmap_status();
  auto map_fusion_status = fct_out.map_fusion_hdmap_status();
  bool hdmap_status{false};
  bool hdmap_active_condations{false};
  bool hdmap_reduce_condations{false};
  auto* nnp_activation_conditions =
      fct_out.mutable_nnp_fct_out()->mutable_nnp_activation_conditions();
  if (cur_sub_state_ == functionmanager::LOCAL_HDMAP_TYPE) {
    nnp_activation_conditions->CopyFrom(nnp_fct_out.local_map_active_status());
    nnp_activation_conditions->set_vehicle_in_hdmap(
        nnp_fct_out.local_map_active_status().vehicle_in_hdmap() &&
        local_hdmap_status);
    hdmap_status = local_hdmap_status;
  } else if (cur_sub_state_ == functionmanager::EHP_HDMAP_TYPE) {
    nnp_activation_conditions->CopyFrom(nnp_fct_out.hd_map_active_status());
    nnp_activation_conditions->set_vehicle_in_hdmap(
        nnp_fct_out.hd_map_active_status().vehicle_in_hdmap() &&
        ehp_hdmap_status);
    hdmap_status = ehp_hdmap_status;
  } else if (cur_sub_state_ == functionmanager::MAP_FUSION_TYPE) {
    nnp_activation_conditions->CopyFrom(nnp_fct_out.hd_map_active_status());
    nnp_activation_conditions->set_vehicle_in_hdmap(
        nnp_fct_out.hd_map_active_status().vehicle_in_hdmap() &&
        map_fusion_status);
    hdmap_status = map_fusion_status;
  }
  hdmap_active_condations =
      nnp_activation_conditions->vehicle_not_in_forbidlane() &&
      nnp_activation_conditions->vehicle_not_in_reverselane() &&
      nnp_activation_conditions->vehicle_not_in_otherforbidarea() &&
      nnp_activation_conditions->appropriate_current_lane_curve() &&
      nnp_activation_conditions->appropriate_current_lane_headingerr() &&
      nnp_activation_conditions->appropriate_current_lane_width();
  hdmap_reduce_condations =
      nnp_activation_conditions->vehicle_in_hdmap() &&
      nnp_activation_conditions->valid_of_lane_localization() &&
      nnp_activation_conditions->valid_of_lane_routing();
  auto nnp_main_switch_bl = local_view->GetFunctionManagerIn()
                                ->nnp_hmi_signals()
                                .fct_is_nnpmainswitch_bl();
  // nnp_active_onoffset/nnpsndstate: 1->on  2:off
  auto nnp_avtive_action = local_view->GetFunctionManagerIn()
                               ->nnp_hmi_signals()
                               .nnp_active_onoffset();
  auto nnp_psnd_state =
      local_view->GetFunctionManagerIn()->nnp_hmi_signals().nnpsndstate();
  bool initiative_degrate_bl{false};
  // 地图模式有问题直接跳到视觉模式；地图模式没问题，如果打开了内部降级标志，则当处于不可行
  // 驶的ODD区域或者定位失效时，且车道线质量很好的情况下也会内部降级到视觉状态。

  if ((nnp_avtive_action == functionmanager::NnpHmiSignals::Off ||
       nnp_psnd_state == functionmanager::NnpHmiSignals::Off) &&
      local_view_data_->update_data()->is_pilot_drive_auto()) {
    initiative_degrate_bl = true;
  }
#ifdef ISORIN
  initiative_degrate_bl = false;
  nnp_main_switch_bl = true;
#endif
  // AERROR << "hdmap_status: " << hdmap_status
  //        << " ,FLAGS_enable_planning_self_simulator: "
  //        << FLAGS_enable_planning_self_simulator
  //        << " , FLAGS_enable_odd_area_internal_to_perception:"
  //        << FLAGS_enable_odd_area_internal_to_perception
  //        << " , nnp_main_switch_bl: " << nnp_main_switch_bl
  //        << " , hdmap_reduce_condations: " << hdmap_reduce_condations
  //        << " , initiative_degrate_bl: " << initiative_degrate_bl
  //        << " , nnp_sys_state: " << nnp_sys_state
  //        << " ,local_view_data_->update_data().is_nnp_drive_auto(): "
  //        << local_view_data_->update_data().is_nnp_drive_auto()
  //        << " , hdmap_active_condations:" << hdmap_active_condations;

#ifdef FOR_BAIDU_SIMULATION
  local_view->SetFunctionManagerOutPtr(
      std::make_shared<functionmanager::FunctionManagerOut>(fct_out));
  AERROR << "FOR_BAIDU_SIMULATION return local_hdmap_status: "
         << local_hdmap_status << ", cur_sub_state: " << cur_sub_state_;
  return hdmap_fg;
#endif
  // 检查是否应该在路口跟车模式优先
  if (MissileModeDecider(local_view)) {
    hdmap_status = false;
  }

  if (hdmap_status) {
    if (FLAGS_enable_planning_self_simulator ||
        !FLAGS_enable_odd_area_internal_to_perception || FLAGS_is_mcap_replay) {
      local_view->SetFunctionManagerOutPtr(
          std::make_shared<functionmanager::FunctionManagerOut>(fct_out));
      return hdmap_fg;
    }
    if (!nnp_main_switch_bl) {
      hdmap_fg = false;
    } else {
      if (local_view_data_->update_data()->is_nnp_drive_auto()) {
        if (((odd_faild || !hdmap_reduce_condations) &&
             FLAGS_enable_odd_area_internal_to_perception) ||
            initiative_degrate_bl) {
          fct_out.mutable_nnp_fct_out()
              ->mutable_nnp_statechange_conditions()
              ->set_is_change_dueto_internalreasons(localization_state == 1 &&
                                                    good_lane);
          fct_out.mutable_nnp_fct_out()->set_initiative_degrate_bl(
              initiative_degrate_bl);
          hdmap_fg = false;
        }
      } else {
        if (!hdmap_reduce_condations ||
            ((!hdmap_active_condations ||
              local_view_data_->update_data()->is_nnp_npilot()) &&
             is_not_fusionmap)) {
          fct_out.mutable_nnp_fct_out()->set_initiative_degrate_bl(
              local_view_data_->update_data()->is_nnp_npilot());
          hdmap_fg = false;
        }
      }
    }
  } else {
#ifdef FOR_BAIDU_SIMULATION
    AERROR << "hdmap_status: " << hdmap_status;
#endif
    bool is_change_dueto_internalreasons =
        ((localization_state == 2) &&
         local_view_data_->update_data()->is_nnp_drive_auto());
    fct_out.mutable_nnp_fct_out()
        ->mutable_nnp_statechange_conditions()
        ->set_is_change_dueto_internalreasons(is_change_dueto_internalreasons);
    hdmap_fg = false;
  }
  local_view->SetFunctionManagerOutPtr(
      std::make_shared<functionmanager::FunctionManagerOut>(fct_out));
  ADEBUG
      << "odd_faild = " << odd_faild
      << ", outof_odd = " << nnp_fct_out.nnp_d_distance_outof_odd_sg()
      << ", is_change_mode_by_odd_type = "
      << nnp_fct_out.nnp_statechange_conditions().is_change_mode_by_odd_type()
      << ", is_location_state = " << localization_state
      << ", good_lane = " << good_lane;
  return hdmap_fg;
}

void HDMapLaneLineState::SubStateDecider(
    const std::shared_ptr<LocalView>& local_view) {
  // ehp first
  bool ehp_hdmap_flag = local_view->GetFunctionManagerOut()->ehp_hdmap_status();
  bool local_hdmap_flag =
      local_view->GetFunctionManagerOut()->local_hdmap_status();
  bool map_fusion_flag =
      local_view->GetFunctionManagerOut()->map_fusion_hdmap_status();
  ADEBUG << "cur_sub_state_ " << cur_sub_state_ << " ehp_hdmap_flag "
         << ehp_hdmap_flag << " local_hdmap_flag " << local_hdmap_flag
         << " map_fusion_flag " << map_fusion_flag;
  switch (cur_sub_state_) {
    case functionmanager::HD_INITIAL_TYPE:
      if (map_fusion_flag) {
        cur_sub_state_ = functionmanager::MAP_FUSION_TYPE;
      } else if (ehp_hdmap_flag) {
        cur_sub_state_ = functionmanager::EHP_HDMAP_TYPE;
      } else if (local_hdmap_flag) {
        cur_sub_state_ = functionmanager::LOCAL_HDMAP_TYPE;
      }
      break;
    case functionmanager::LOCAL_HDMAP_TYPE:
      if (map_fusion_flag) {
        cur_sub_state_ = functionmanager::MAP_FUSION_TYPE;
      } else if (ehp_hdmap_flag) {
        cur_sub_state_ = functionmanager::EHP_HDMAP_TYPE;
      } else if (!local_hdmap_flag) {
        cur_sub_state_ = functionmanager::HD_INITIAL_TYPE;
      }
      break;
    case functionmanager::EHP_HDMAP_TYPE:
      if (!ehp_hdmap_flag && local_hdmap_flag) {
        cur_sub_state_ = functionmanager::LOCAL_HDMAP_TYPE;
      } else if (!ehp_hdmap_flag && !local_hdmap_flag) {
        cur_sub_state_ = functionmanager::HD_INITIAL_TYPE;
      }
      break;
    case functionmanager::MAP_FUSION_TYPE:
      if (!map_fusion_flag) {
        cur_sub_state_ = functionmanager::HD_INITIAL_TYPE;
      }
    default:
      break;
  }
}

Status HDMapLaneLineState::GetSubStatus(
    const std::shared_ptr<LocalView>& local_view) {
  auto fmo = local_view->GetFunctionManagerOut();
  bool status = false;
  switch (cur_sub_state_) {
    case functionmanager::LOCAL_HDMAP_TYPE:
      status = fmo->local_hdmap_status();
      break;
    case functionmanager::EHP_HDMAP_TYPE:
      status = fmo->ehp_hdmap_status();
      break;
    case functionmanager::MAP_FUSION_TYPE:
      status = fmo->map_fusion_hdmap_status();
      break;
    default:
      break;
  }
  if (status) {
    return Status::OK();
  }
#ifdef FOR_BAIDU_SIMULATION
  AERROR << "cur_sub_state_: " << cur_sub_state_
         << ", local_hdmap_status:" << fmo->local_hdmap_status();
#endif
  return Status(ErrorCode::LOCALVIEW_FSM_PERCEPTION_ERROR, "GetSubStatus fail");
}

void HDMapLaneLineState::SetNnpActiveCondations(
    const std::shared_ptr<LocalView>& local_view) {
  auto fct_out = *local_view->GetFunctionManagerOut();
  const auto nnp_fct_out = fct_out.nnp_fct_out();
  auto* nnp_activation_conditions =
      fct_out.mutable_nnp_fct_out()->mutable_nnp_activation_conditions();
  auto ehp_hdmap_status = fct_out.ehp_hdmap_status();
  auto local_hdmap_status = fct_out.local_hdmap_status();
  auto map_fusion_status = fct_out.map_fusion_hdmap_status();
  if (cur_sub_state_ == functionmanager::LOCAL_HDMAP_TYPE) {
    nnp_activation_conditions->CopyFrom(nnp_fct_out.local_map_active_status());
    nnp_activation_conditions->set_vehicle_in_hdmap(
        nnp_fct_out.local_map_active_status().vehicle_in_hdmap() &&
        local_hdmap_status);
  } else if (cur_sub_state_ == functionmanager::EHP_HDMAP_TYPE) {
    nnp_activation_conditions->CopyFrom(nnp_fct_out.hd_map_active_status());
    nnp_activation_conditions->set_vehicle_in_hdmap(
        nnp_fct_out.hd_map_active_status().vehicle_in_hdmap() &&
        ehp_hdmap_status);
  } else if (cur_sub_state_ == functionmanager::MAP_FUSION_TYPE) {
    nnp_activation_conditions->CopyFrom(nnp_fct_out.hd_map_active_status());
    nnp_activation_conditions->set_vehicle_in_hdmap(
        nnp_fct_out.hd_map_active_status().vehicle_in_hdmap() &&
        map_fusion_status);
  }
  local_view->SetFunctionManagerOutPtr(
      std::make_shared<functionmanager::FunctionManagerOut>(fct_out));
}

bool HDMapLaneLineState::MissileModeDecider(
    const std::shared_ptr<LocalView>& local_view) {
  bool using_nolane_first{false};
  /**
 * 路口跟车是否优先标志位FLAGS_using_missile_mode_level
 * 0：不主动降级到跟车模式
 * 1：路口优先降级到跟车模式
 * 2：路口有前车优先降级到跟车模式
 * 3：路口有前车且建图有折线问题优先降级到跟车模式
 */
  // 只有PILOT激活情况下，FLAGS_using_missile_mode_level
  constexpr double kMinHeadingErr = 0.06;
  auto laneline_status = local_view->GetFunctionManagerOut()->laneline_status();
  auto nolane_status = local_view->GetFunctionManagerOut()->nolane_status();
  const bool map2nolane_bl =
      local_view_data_->update_data()->is_virtual_map2nolane_bl();
  // const bool adc_lane_is_virtual =
  //     local_view_data_->update_data()->is_adc_lane_virtual();
  auto missile_mode_state =
      local_view_data_->update_data()->missile_mode_state();
  double heading_err = std::fabs(
      local_view_data_->update_data()->adc_fusion_percep_heading_err() -
      local_view_data_->update_data()->adc_target_obs_flupos_heading());
  const auto fusion_percep_heading_err =
      local_view_data_->update_data()->adc_fusion_percep_heading_err();
  const auto obs_heading =
      local_view_data_->update_data()->adc_target_obs_flupos_heading();
  const bool heading_err_diff =
      std::fabs(fusion_percep_heading_err - obs_heading) > kMinHeadingErr &&
      std::fabs(fusion_percep_heading_err) > kMinHeadingErr &&
      std::fabs(obs_heading) < kMinHeadingErr;
  const bool common_bl =
      local_view_data_->update_data()->is_pilot_drive_auto() && nolane_status &&
      !laneline_status && map2nolane_bl;
  if (FLAGS_using_missile_mode_level == 1) {
    using_nolane_first = common_bl;
  } else if (FLAGS_using_missile_mode_level == 2) {
    using_nolane_first = common_bl && missile_mode_state == 2;
  } else if (FLAGS_using_missile_mode_level == 3) {
    using_nolane_first =
        common_bl && missile_mode_state == 2 && heading_err_diff;
  }
  ADEBUG << "common_bl: " << common_bl
         << " , missile_mode_state: " << missile_mode_state
         << " ,heading_err: " << heading_err
         << " , map2nolane_bl: " << map2nolane_bl
         << " ,fusion_percep_heading_err: " << fusion_percep_heading_err
         << " ,obs_heading: " << obs_heading;
  return using_nolane_first;
}

}  // namespace planning
}  // namespace TL
