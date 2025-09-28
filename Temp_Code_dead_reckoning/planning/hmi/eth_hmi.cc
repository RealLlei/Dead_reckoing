/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning/hmi/eth_hmi.h"
#include <cstdint>
#ifdef ISORIN
#include "yaml-cpp/yaml.h"
#endif
#include <bitset>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <utility>
#include "common/configs/config_gflags.h"
#include "common/time/clock.h"
#include "map/hdmap/hdmap_common.h"
#include "nlohmann/json.hpp"
#include "planning/common/planning_gflags.h"
#include "planning/localview/local_view.h"
#include "proto/fsm/avp_fct.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/fsm/soc_to_mcu.pb.h"
#include "proto/hmi/nnp.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {
using Json = nlohmann::json;
using TL::common::Clock;
// using TL::common::VehicleSignal;
using TL::functionmanager::NNPSysState;
// using TL::hmi::LaneChangeType;
using functionmanager::ChangeLaneInfor;
using hmi::LaneChangeStatus;

EthHmi::EthHmi() {
  warning_ = std::make_unique<warning::Warning>();
  GetVersion();
  GetVehicleType();
  GetCarName();
  if (FLAGS_enable_planning_warning) {
    warning_->Init();
    warning_->Start();
  }
}

// 输出输出的二次处理
void EthHmi::ProcessEthHmi(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (local_view == nullptr || ptr_trajectory_pb == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  if (FLAGS_enable_planning_warning) {
    auto* warning_output =
        ptr_trajectory_pb->mutable_debug()->mutable_warning_output();
    auto warning_fault =
        ptr_trajectory_pb->debug().monitor_fault_debug().warning_fault();
    if (!FLAGS_enable_warning_fault_process) {
      warning_fault.Clear();
    }
    warning_->Process(warning_fault, local_view, warning_output);
    // 此处设计失误，只能拷贝一份，TODO 后续把这个拷贝去掉，轨迹上不用放两份一样的
    ptr_trajectory_pb->mutable_warning_status()->CopyFrom(
        warning_output->warning_status());
    LcaAudioPlayMapping(local_view, ptr_trajectory_pb);
    // WarningTest(ptr_trajectory_pb);
  }
  if (ptr_trajectory_pb->function_manager_in().ta_pilot_mode() !=
      functionmanager::TaPilotMode::AVP) {
    const auto& planning_fault =
        ptr_trajectory_pb->debug().monitor_fault_debug().planning_fault();
    auto* mutable_planning_fault =
        ptr_trajectory_pb->mutable_function_manager_out()
            ->mutable_nnp_fct_out()
            ->mutable_nnp_software_fault();
    mutable_planning_fault->set_planning_lat_err(planning_fault.lat());
    mutable_planning_fault->set_planning_lon_err(planning_fault.lon());
  }
  UpdateLaneChangeWarn(ptr_trajectory_pb);
  UpdateVehicleCfgToMcu(local_view, ptr_trajectory_pb);
  UpdateHmiData(local_view, ptr_trajectory_pb);
  CheckLaneChangeDirCameraLane(local_view, ptr_trajectory_pb);
  // UpdatePilotLaneChangeStatusToHmi(*localview->GetFunctionManagerIn(),
  //  ptr_trajectory_pb);
  if (FLAGS_map_data_to_hmi) {
    ExtractMapElement(local_view, ptr_trajectory_pb);
  }
  // CheckCameraLaneBoundaryType(localview, ptr_trajectory_pb);
  CheckStaticObstacle(ptr_trajectory_pb);
  ptr_trajectory_pb->mutable_function_manager_out()->set_version(version_);
  ptr_trajectory_pb->mutable_function_manager_out()
      ->mutable_nnp_metric()
      ->set_car_name(car_name_);
  if (vehicle_platform_type_ == "EP40") {
    const auto& soc_2_fct_tbd_u32_03 =
        ptr_trajectory_pb->function_manager_out().soc_2_fct_tbd_u32_03();
    auto* mutable_fct_out = ptr_trajectory_pb->mutable_function_manager_out();
    if (mutable_fct_out != nullptr) {
      std::bitset<32> value(soc_2_fct_tbd_u32_03);
      value |= 0x40000;
      mutable_fct_out->set_soc_2_fct_tbd_u32_03(value.to_ulong());
    }
  }
  // CheckLanechangeReason(ptr_trajectory_pb);
}

void EthHmi::UpdateVehicleCfgToMcu(  // NOLINT
    const std::shared_ptr<LocalView>& localview,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {

  if (ptr_trajectory_pb == nullptr) {
    return;
  }
  if (localview == nullptr || !localview->HasChassis()) {
    return;
  }

  auto* mutable_soc_u8 =
      ptr_trajectory_pb->mutable_soc_to_fct_bus()->mutable_soc_to_fct_bus_u8();
  auto platform_cfg =
      localview->GetChassis()->vehicle_cfg().central_control_platform();
  mutable_soc_u8->set_reserved05(platform_cfg);
}

void EthHmi::GetVersion() {
  std::string version_name = "/opt/app/1/version.json";
#ifdef ISORIN
  version_name = "/app/version.json";
#endif
  std::ifstream ifs(version_name);
  if (ifs.is_open()) {
    Json json;
    ifs >> json;
    ifs.close();
    const auto TL = json.find("HZ");
    if (TL != json.end() && TL->is_string()) {
      version_ = TL.value();
    }
  }
}

void EthHmi::GetVehicleType() {  // NOLINT
  std::ifstream ifs;
  std::string vehicle_platform_type = "EP41";
  ifs.open("/cfg/system/vehicle_flag.cfg", std::ios::in);
  if (ifs.is_open()) {
    std::getline(ifs, vehicle_platform_type);
    AERROR << "vehicle_platform_type is " << vehicle_platform_type;
  }
  ifs.close();
  vehicle_platform_type_ = vehicle_platform_type;
}

void EthHmi::GetCarName() {
#ifdef ISORIN
  std::string ve_advc_meta = "/opt/usr/bag_dv/ve_advc_meta.yaml";
  try {
    YAML::Node node = YAML::LoadFile(ve_advc_meta);
    if (node["CustomizedFields"]) {
      for (const auto& field : node["CustomizedFields"]) {
        if (field["FieldName"].as<std::string>() == "vehicle_alias") {
          car_name_ = field["FieldValue"].as<std::string>();
          break;
        }
      }
    }
  } catch (const YAML::Exception& e) {
    AWARN << "yaml file not found: " << e.what();
  }
#endif
}

void EthHmi::LcaAudioPlayMapping(  // NOLINT
    const std::shared_ptr<LocalView>& localview,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (ptr_trajectory_pb == nullptr) {
    return;
  }
  bool is_lca_lv2_warning = false;
  auto* mutable_soc_u8 =
      ptr_trajectory_pb->mutable_soc_to_fct_bus()->mutable_soc_to_fct_bus_u8();
  const auto& warning_status = ptr_trajectory_pb->warning_status();
  if (warning_status.lca_left_warning() == WarningLevel::LV2_WARNING) {
    is_lca_lv2_warning = true;
    mutable_soc_u8->set_reserved03(0x01);
  } else {
    mutable_soc_u8->set_reserved03(0x00);
  }
  if (warning_status.lca_right_warning() == WarningLevel::LV2_WARNING) {
    is_lca_lv2_warning = true;
    mutable_soc_u8->set_reserved04(0x01);
  } else {
    mutable_soc_u8->set_reserved04(0x00);
  }
  if (!localview->HasMcuToSocPnc() ||
      !localview->GetMcuToSocPnc()->has_can_input_from_mcu()) {
    mutable_soc_u8->set_enable_spd_adapt(0x00);
    return;
  }
  const auto& lca_steerwhlvibrat_onoffset =
      localview->GetMcuToSocPnc()
          ->can_input_from_mcu()
          .cdcs21_lcasteerwhlvibrat_onoffset();
  // lca 二级报警方向盘抖动
  if (lca_steerwhlvibrat_onoffset > 0 && is_lca_lv2_warning) {
    mutable_soc_u8->set_enable_spd_adapt(0x01);
  } else {
    mutable_soc_u8->set_enable_spd_adapt(0x00);
  }
}

void EthHmi::ExtractMapElement(  // NOLINT
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (local_view->HasHDMap() && local_view->HasLocalization() &&
      local_view->HasADCTrajectory()) {
    const auto& adc_point = local_view->GetLocalization()->pose();
    auto* mutable_cross_info_enu =
        ptr_trajectory_pb->mutable_nnp_hmi_output()->mutable_cross_info_enu();
    auto* mutable_cross_info_vrf =
        ptr_trajectory_pb->mutable_nnp_hmi_output()->mutable_cross_info_vrf();
    std::vector<hdmap::SignalInfoConstPtr> signals{};
    local_view->GetHDMapPtr()->GetSignals(adc_point.position(),
                                          FLAGS_map_radius, &signals);
    for (const auto& signal : signals) {
      if (signal == nullptr) {
        continue;
      }
      auto* mutable_stopline = mutable_cross_info_enu->add_stoplines();
      auto* mutable_stopline_vrf = mutable_cross_info_vrf->add_stoplines();
      mutable_stopline->set_id(signal->id().id());
      mutable_stopline_vrf->set_id(signal->id().id());
      if (signal->signal().stop_line_size() > 0) {
        const auto& stop_line = signal->signal().stop_line(0);
        if (stop_line.segment_size() > 0) {
          for (const auto& point :
               stop_line.segment(0).line_segment().point()) {
            auto* mutable_point = mutable_stopline->add_points();
            auto* mutable_point_vrf = mutable_stopline_vrf->add_points();
            const auto vector2d = TL::common::math::RotateVector2d(
                {point.x() - adc_point.position().x(),
                 point.y() - adc_point.position().y()},
                -adc_point.heading());
            mutable_point->set_x(point.x());
            mutable_point->set_y(point.y());
            mutable_point->set_z(0.0);
            mutable_point_vrf->set_x(vector2d.x());
            mutable_point_vrf->set_y(vector2d.y());
            mutable_point_vrf->set_z(0.0);
          }
        }
      }
    }
    std::vector<hdmap::CrosswalkInfoConstPtr> crosswalks{};
    local_view->GetHDMapPtr()->GetCrosswalks(adc_point.position(),
                                             FLAGS_map_radius, &crosswalks);
    for (const auto& crosswalk : crosswalks) {
      if (crosswalk == nullptr) {
        continue;
      }
      auto* mutable_crosswalk = mutable_cross_info_enu->add_crosswalks();
      mutable_crosswalk->set_id(crosswalk->id().id());
      auto* mutable_crosswalk_vrf = mutable_cross_info_vrf->add_crosswalks();
      mutable_crosswalk_vrf->set_id(crosswalk->id().id());
      for (const auto& point : crosswalk->crosswalk().polygon().point()) {
        auto* mutable_point = mutable_crosswalk->add_points();
        auto* mutable_point_vrf = mutable_crosswalk_vrf->add_points();
        const auto vector2d = TL::common::math::RotateVector2d(
            {point.x() - adc_point.position().x(),
             point.y() - adc_point.position().y()},
            -adc_point.heading());
        mutable_point->set_x(point.x());
        mutable_point->set_y(point.y());
        mutable_point->set_z(0.0);
        mutable_point_vrf->set_x(vector2d.x());
        mutable_point_vrf->set_y(vector2d.y());
        mutable_point_vrf->set_z(0.0);
      }
    }
  }
}

void EthHmi::WarningTest(  // NOLINT
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  static int warning_type_cnt = 0;
  auto* mutable_warning_status = ptr_trajectory_pb->mutable_warning_status();
  mutable_warning_status->set_dow_state(true);
  mutable_warning_status->set_lca_state(true);
  mutable_warning_status->set_fcta_state(true);
  mutable_warning_status->set_rcta_state(true);
  mutable_warning_status->set_rcw_state(true);
  WarningLevel warning_level = WarningLevel::LV2_WARNING;
  if (warning_type_cnt > 0 && warning_type_cnt < 20) {
    mutable_warning_status->set_dow_audio_play(true);
    mutable_warning_status->set_dow_left_warning(warning_level);
  } else if (warning_type_cnt > 20 && warning_type_cnt < 40) {
    mutable_warning_status->set_lca_left_warning(warning_level);
    mutable_warning_status->set_lca_right_warning(warning_level);
  } else if (warning_type_cnt > 40 && warning_type_cnt < 60) {
    mutable_warning_status->set_fcta_left_warning(warning_level);
    mutable_warning_status->set_fcta_right_warning(warning_level);
    mutable_warning_status->set_fcta_audio_play(AudioPlay::LEFT_WARNING_PLAY);
  } else if (warning_type_cnt > 60 && warning_type_cnt < 80) {
    mutable_warning_status->set_rcta_left_warning(warning_level);
    mutable_warning_status->set_rcta_right_warning(warning_level);
    mutable_warning_status->set_rcta_audio_play(AudioPlay::LEFT_WARNING_PLAY);
    warning_type_cnt = 0;
  }
  warning_type_cnt++;
}

void EthHmi::UpdatePilotLaneChangeStatusToHmi(  //NOLINT
    const TL::functionmanager::FunctionManagerIn& fct_in,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (!fct_in.has_fct_nnp_in() ||
      !fct_in.fct_nnp_in().has_pilot_lane_change_status()) {
    return;
  }
  const auto& pilot_lc_status = fct_in.fct_nnp_in().pilot_lane_change_status();
  auto* mutable_nnp_fct_out =
      ptr_trajectory_pb->mutable_function_manager_out()->mutable_nnp_fct_out();
  mutable_nnp_fct_out->set_lane_change_infor(pilot_lc_status.pilot_lc_infor());
  auto* mutable_hmi_out_state =
      ptr_trajectory_pb->mutable_nnp_hmi_output()->mutable_state();
  if (pilot_lc_status.pilot_lc_infor() ==
          functionmanager::ChangeLaneInfor::LANE_CHANGE_PENDING ||
      pilot_lc_status.pilot_lc_infor() ==
          functionmanager::ChangeLaneInfor::LANE_CHANGE_ONGOING) {
    mutable_hmi_out_state->set_lane_changed_type(
        hmi::LaneChangeType::NORMAL_CHANGE_LANE);
    bool warn_status = pilot_lc_status.pilot_lc_warning();
    if (pilot_lc_status.pilot_lc_dir() ==
        functionmanager::PilotLaneChangeDir::PILOT_DIR_LEFT) {
      mutable_hmi_out_state->set_lane_changed_status(
          warn_status ? LaneChangeStatus::LANE_CHANGE_DANGER_LEFT
                      : LaneChangeStatus::LEFT);
    } else if (pilot_lc_status.pilot_lc_dir() ==
               functionmanager::PilotLaneChangeDir::PILOT_DIR_RIGHT) {
      mutable_hmi_out_state->set_lane_changed_status(
          warn_status ? LaneChangeStatus::LANE_CHANGE_DANGER_RIGHT
                      : LaneChangeStatus::RIGHT);
    }
  }
}

void EthHmi::UpdateLaneChangeWarn(  //NOLINT
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (ptr_trajectory_pb == nullptr) {
    AERROR << "ptr_trajectory_pb is nullptr";
    return;
  }
  static double last_rcw_timestamp = 0.0;
  if (!ptr_trajectory_pb->has_function_manager_out() ||
      !ptr_trajectory_pb->function_manager_out().has_nnp_fct_out()) {
    return;
  }
  auto* mutable_nnp_fct_out =
      ptr_trajectory_pb->mutable_function_manager_out()->mutable_nnp_fct_out();
  const auto& warning_status = ptr_trajectory_pb->warning_status();
  if (warning_status.has_rcw_warning() &&
      warning_status.rcw_warning() == WarningLevel::LV2_WARNING) {
    last_rcw_timestamp = Clock::NowInSeconds();
  }
  if (Clock::NowInSeconds() - last_rcw_timestamp <
      FLAGS_rcw_double_flashing_time) {
    mutable_nnp_fct_out->set_light_request(
        functionmanager::LightReq::WARNING_LIGHT);
  }
}

void EthHmi::CheckLaneChangeDirCameraLane(  // NOLINT
    const std::shared_ptr<LocalView>& localview,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (!localview->HasLaneMarkers() || ptr_trajectory_pb == nullptr) {
    return;
  }
  const auto& lane_change_dir = ptr_trajectory_pb->function_manager_out()
                                    .nnp_fct_out()
                                    .lane_change_direction();
  const auto& lane_change_infor = ptr_trajectory_pb->function_manager_out()
                                      .nnp_fct_out()
                                      .lane_change_infor();
  static functionmanager::LaneChangeDir last_lane_change_dir =
      functionmanager::LaneChangeDir::NONE_DIR;
  static bool lane_change_has_target_lane = false;

  if (lane_change_dir != functionmanager::LaneChangeDir::NONE_DIR) {
    last_lane_change_dir = lane_change_dir;
  }
  if (lane_change_infor == functionmanager::ChangeLaneInfor::INFOR_NONE) {
    last_lane_change_dir = functionmanager::LaneChangeDir::NONE_DIR;
    lane_change_has_target_lane = false;
    return;
  }
  // pending过程中，突然没有目标车道，导致的变道取消无交互
  if (lane_change_has_target_lane) {
    return;
  }
  if (lane_change_infor ==
          functionmanager::ChangeLaneInfor::LANE_CHANGE_ONGOING ||
      lane_change_infor == functionmanager::ChangeLaneInfor::LANE_CHANGE_END) {
    return;
  }
  const auto& front_next_left_lane_markers =
      localview->GetLaneMarkers()->front_next_left_lane_marker();
  const auto& front_next_right_lane_markers =
      localview->GetLaneMarkers()->front_next_right_lane_marker();
  static constexpr double kMInViewRange = 0.1;
  auto* mutable_nnp_fct_out =
      ptr_trajectory_pb->mutable_function_manager_out()->mutable_nnp_fct_out();
  bool no_lane_maker =
      (last_lane_change_dir == functionmanager::LaneChangeDir::LEFT_DIR &&
       (front_next_left_lane_markers.empty() ||
        (!front_next_left_lane_markers.empty() &&
         front_next_left_lane_markers.at(0).view_range() < kMInViewRange))) ||
      (last_lane_change_dir == functionmanager::LaneChangeDir::RIGHT_DIR &&
       (front_next_right_lane_markers.empty() ||
        (!front_next_right_lane_markers.empty() &&
         front_next_right_lane_markers.at(0).view_range() < kMInViewRange)));
  // 变道方向没有lane marker 不进行任何变道交互
  if (no_lane_maker) {
    mutable_nnp_fct_out->set_lane_change_infor(
        functionmanager::ChangeLaneInfor::INFOR_NONE);
    mutable_nnp_fct_out->set_lane_change_direction(
        functionmanager::LaneChangeDir::NONE_DIR);
    mutable_nnp_fct_out->set_dclc_audio_play(
        functionmanager::DclcAudioPlay::DCLC_NONE);
    mutable_nnp_fct_out->set_lane_change_warning(
        functionmanager::LaneChangeWarn::NO_WARN);
    mutable_nnp_fct_out->set_lane_change_pending_alert(false);
  } else {
    lane_change_has_target_lane = true;
  }
}

void EthHmi::UpdateHmiChangeLaneState(  //NOLINT
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  const auto& nnp_fct_out =
      ptr_trajectory_pb->function_manager_out().nnp_fct_out();
  const auto& change_lane_dir = nnp_fct_out.lane_change_direction();
  // 换道状态
  auto* mutable_hmi_out_state =
      ptr_trajectory_pb->mutable_nnp_hmi_output()->mutable_state();
  auto* mutable_soc_to_mcu = ptr_trajectory_pb->mutable_soc_to_fct_bus();
  mutable_soc_to_mcu->mutable_soc_to_fct_bus_u8()->set_ldp_left_trigger_flag(0);
  if (!ptr_trajectory_pb->has_decision() ||
      !ptr_trajectory_pb->decision().has_vehicle_signal()) {
    mutable_hmi_out_state->set_lane_changed_type(hmi::LaneChangeType::UNKOWN);
  } else if (nnp_fct_out.lane_change_infor() ==
                 ChangeLaneInfor::LANE_CHANGE_ONGOING ||
             nnp_fct_out.lane_change_infor() ==
                 ChangeLaneInfor::LANE_CHANGE_PENDING) {
    mutable_hmi_out_state->set_lane_changed_type(
        hmi::LaneChangeType::NORMAL_CHANGE_LANE);
  }
  mutable_hmi_out_state->set_lane_changed_status(LaneChangeStatus::NO_ACTION);
  static constexpr int kSendCnt = 2;
  static int send_cnt = 0;
  // 换道类型 todo 细化 (根据座舱效果修改为只有左右变道和变道取消、变道完成)
  switch (nnp_fct_out.lane_change_infor()) {
    case ChangeLaneInfor::INFOR_NONE:
      mutable_hmi_out_state->set_lane_changed_status(
          LaneChangeStatus::NO_ACTION);
      break;

    case ChangeLaneInfor::LANE_CHANGE_START:
      break;
    case ChangeLaneInfor::LANE_CHANGE_PENDING:
    case ChangeLaneInfor::LANE_CHANGE_ONGOING:
      mutable_soc_to_mcu->mutable_soc_to_fct_bus_u8()
          ->set_ldp_left_trigger_flag(0x01);
      if (change_lane_dir == functionmanager::LaneChangeDir::LEFT_DIR) {
        mutable_hmi_out_state->set_lane_changed_status(LaneChangeStatus::LEFT);
      } else if (change_lane_dir == functionmanager::LaneChangeDir::RIGHT_DIR) {
        mutable_hmi_out_state->set_lane_changed_status(LaneChangeStatus::RIGHT);
      }
      break;
    case ChangeLaneInfor::LANE_CHANGE_END:
      send_cnt = kSendCnt;
      break;
    default:
      mutable_hmi_out_state->set_lane_changed_status(
          LaneChangeStatus::NO_ACTION);
      break;
  }
  const auto& fct_out = ptr_trajectory_pb->function_manager_out();
  const auto& fct_in = ptr_trajectory_pb->function_manager_in();
  const auto& is_lane_change_on_solid_lane =
      fct_out.hmi_lane_change_debug().is_lane_change_on_solid_lane();
  const auto& is_hands_off_warning_bl =
      fct_in.fct_nnp_in().is_hands_off_warning_bl();
  static ChangeLaneInfor last_lane_change_infor = ChangeLaneInfor::INFOR_NONE;
  static double start_lc_pending_timestamp = 0.0;
  static constexpr double kLaneChangeTimestamp = 0.5;
  if (last_lane_change_infor != ChangeLaneInfor::LANE_CHANGE_PENDING &&
      nnp_fct_out.lane_change_infor() == ChangeLaneInfor::LANE_CHANGE_PENDING) {
    start_lc_pending_timestamp = Clock::NowInSeconds();
  }
  last_lane_change_infor = nnp_fct_out.lane_change_infor();
  bool is_ok_warning_park =
      (nnp_fct_out.lane_change_infor() ==
           ChangeLaneInfor::LANE_CHANGE_PENDING &&
       !is_lane_change_on_solid_lane && !is_hands_off_warning_bl &&
       Clock::NowInSeconds() - start_lc_pending_timestamp >
           kLaneChangeTimestamp);
  const auto& alc_obs_hightlights =
      ptr_trajectory_pb->nnp_hmi_output().nnp_alc_obs_hightlight();
  for (const auto& alc_obs : alc_obs_hightlights) {
    if (hmi::NNPHmiOutput::ALC_LEFT == alc_obs.highlight_reason() &&
        is_ok_warning_park) {
      mutable_hmi_out_state->set_lane_changed_status(
          LaneChangeStatus::LANE_CHANGE_DANGER_LEFT);
      break;
    }
    if (hmi::NNPHmiOutput::ALC_RIGHT == alc_obs.highlight_reason() &&
        is_ok_warning_park) {
      mutable_hmi_out_state->set_lane_changed_status(
          LaneChangeStatus::LANE_CHANGE_DANGER_RIGHT);
      break;
    }
  }
  if (nnp_fct_out.lane_change_infor() != ChangeLaneInfor::LANE_CHANGE_ONGOING &&
      nnp_fct_out.lane_change_infor() != ChangeLaneInfor::LANE_CHANGE_START &&
      !is_ok_warning_park) {
    auto* mutable_nnp_fct_out =
        ptr_trajectory_pb->mutable_function_manager_out()
            ->mutable_nnp_fct_out();
    mutable_hmi_out_state->set_lane_changed_status(LaneChangeStatus::NO_ACTION);
    if (nnp_fct_out.lane_change_infor() !=
            ChangeLaneInfor::LANE_CHANGE_PENDING_ALERT &&
        nnp_fct_out.lane_change_infor() !=
            ChangeLaneInfor::LANE_CHANGE_CANCEL) {
      mutable_nnp_fct_out->set_lane_change_infor(
          functionmanager::ChangeLaneInfor::INFOR_NONE);
      mutable_nnp_fct_out->set_dclc_audio_play(
          functionmanager::DclcAudioPlay::DCLC_NONE);
    }
  }
  if (nnp_fct_out.lane_change_infor() == ChangeLaneInfor::LANE_CHANGE_CANCEL) {
    mutable_hmi_out_state->set_lane_changed_status(
        LaneChangeStatus::CANCEL_CHANGE_LANE);
  }

  if (send_cnt > 0) {
    auto* mutable_nnp_fct_out =
        ptr_trajectory_pb->mutable_function_manager_out()
            ->mutable_nnp_fct_out();
    if (mutable_nnp_fct_out != nullptr) {
      mutable_nnp_fct_out->set_lane_change_infor(
          functionmanager::ChangeLaneInfor::LANE_CHANGE_END);
    }
    mutable_hmi_out_state->set_lane_changed_status(
        LaneChangeStatus::FINISH_CHANGE_LANE);
    send_cnt--;
  }
}

void EthHmi::CheckStaticObstacle(  // NOLINT
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  const auto& is_lane_change_sidepass =
      ptr_trajectory_pb->function_manager_out()
          .hmi_lane_change_debug()
          .is_lane_change_sidepass_obstacle()
          .is_lane_change_sidepass_obstacle();
  const auto& lane_change_infor = ptr_trajectory_pb->function_manager_out()
                                      .nnp_fct_out()
                                      .lane_change_infor();
  auto* mutable_nnp_fct_out =
      ptr_trajectory_pb->mutable_function_manager_out()->mutable_nnp_fct_out();
  if (is_lane_change_sidepass && mutable_nnp_fct_out != nullptr) {
    if (lane_change_infor ==
        functionmanager::ChangeLaneInfor::LANE_CHANGE_PENDING) {
      mutable_nnp_fct_out->set_lane_change_infor(
          functionmanager::ChangeLaneInfor::INFOR_NONE);
    } else if (lane_change_infor ==
               functionmanager::ChangeLaneInfor::LANE_CHANGE_ONGOING) {
      mutable_nnp_fct_out->set_nnp_scenarios(
          functionmanager::NNPScenarios::STATIC_OBSTACLE);
      mutable_nnp_fct_out->set_dclc_audio_play(
          functionmanager::DclcAudioPlay::DCLC_NONE);
      mutable_nnp_fct_out->set_nnp_rino_status(
          functionmanager::NNPRinoStatus::RINO_NO_REQUEST);
    }
  }
}

void EthHmi::CheckCameraLaneBoundaryType(  // NOLINT
    const std::shared_ptr<LocalView>& localview,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (localview == nullptr || !localview->HasLaneMarkers() ||
      !localview->HasChassis() || ptr_trajectory_pb == nullptr) {
    return;
  }
  const auto& nnp_sys_state =
      ptr_trajectory_pb->function_manager_in().fct_nnp_in().nnp_sysstate();
  const auto& adas_mode = ptr_trajectory_pb->function_manager_in().adas_mode();
  const auto& nnp_hmi_signals =
      ptr_trajectory_pb->function_manager_in().nnp_hmi_signals();
  bool is_lat_lon_active = (nnp_sys_state == NNPSysState::NNPS_ACTIVE ||
                            nnp_sys_state == NNPSysState::NNPS_LON_OVERRIDE ||
                            adas_mode == functionmanager::AdasMode::PILOT) &&
                           (nnp_hmi_signals.has_dclc_sys_state() &&
                            nnp_hmi_signals.dclc_sys_state());
  static constexpr int KSolidCnt = 5;
  static constexpr double KQuality = 0.1;
  static double last_lane_high_timestamp = -1.0;
  static int is_solid_left_lane_cnt = 0;
  static int is_solid_right_lane_cnt = 0;
  bool is_lane_change_on_solid_lane = false;
  functionmanager::LaneChangeDir dir = functionmanager::LaneChangeDir::NONE_DIR;
  const auto& turn_signal = localview->GetChassis()->signal().turn_signal();
  const auto& turn_switch = localview->GetChassis()->signal().turn_switch();
  if (is_lat_lon_active && last_lane_high_timestamp < 0.0 &&
      turn_signal != common::VehicleSignal::TURN_NONE &&
      turn_switch != common::TurnLightSwitchStatus::NONE_REQUEST &&
      turn_switch != common::TurnLightSwitchStatus::ERROR) {
    last_lane_high_timestamp = Clock::NowInSeconds();
  } else if (last_lane_high_timestamp > 0.0 &&
             turn_signal == common::VehicleSignal::TURN_NONE) {
    last_lane_high_timestamp = -1.0;
  }
  const auto& lane_change_infor = ptr_trajectory_pb->function_manager_out()
                                      .nnp_fct_out()
                                      .lane_change_infor();
  bool is_lane_change_ongoing =
      lane_change_infor ==
      functionmanager::ChangeLaneInfor::LANE_CHANGE_ONGOING;
  if (!is_lane_change_ongoing && is_lat_lon_active &&
      turn_signal == common::VehicleSignal::TURN_RIGHT &&
      localview->GetLaneMarkers()->has_front_right_lane_marker()) {
    const auto& adc_right_lane_boundary_type =
        localview->GetLaneMarkers()->front_right_lane_marker().lane_type();
    const auto& quality =
        localview->GetLaneMarkers()->front_right_lane_marker().quality();
    if (quality > KQuality &&
        (adc_right_lane_boundary_type ==
             hdmap::LaneBoundaryType::SOLID_YELLOW ||
         adc_right_lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE ||
         adc_right_lane_boundary_type ==
             hdmap::LaneBoundaryType::DOUBLE_YELLOW ||
         adc_right_lane_boundary_type == hdmap::LaneBoundaryType::UNKNOWN)) {
      is_solid_right_lane_cnt++;
    } else {
      is_solid_right_lane_cnt = 0;
    }
    if (is_solid_right_lane_cnt > KSolidCnt) {
      is_lane_change_on_solid_lane = true;
      dir = functionmanager::LaneChangeDir::RIGHT_DIR;
    }
  }
  if (!is_lane_change_ongoing && is_lat_lon_active &&
      turn_signal == common::VehicleSignal::TURN_LEFT &&
      localview->GetLaneMarkers()->has_front_left_lane_marker()) {
    const auto& adc_left_lane_boundary_type =
        localview->GetLaneMarkers()->front_left_lane_marker().lane_type();
    const auto& quality =
        localview->GetLaneMarkers()->front_left_lane_marker().quality();
    if (quality > KQuality &&
        (adc_left_lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
         adc_left_lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE ||
         adc_left_lane_boundary_type ==
             hdmap::LaneBoundaryType::DOUBLE_YELLOW ||
         adc_left_lane_boundary_type == hdmap::LaneBoundaryType::UNKNOWN)) {
      is_solid_left_lane_cnt++;
    } else {
      is_solid_left_lane_cnt = 0;
    }
    if (is_solid_left_lane_cnt > KSolidCnt) {
      is_lane_change_on_solid_lane = true;
      dir = functionmanager::LaneChangeDir::LEFT_DIR;
    }
  }
  // 车道线高亮只判断打灯10s范围内
  static double last_cancel_timestamp = 0.0;
  static bool last_is_pending_alert = false;
  if (is_lat_lon_active && is_lane_change_on_solid_lane &&
      Clock::NowInSeconds() - last_lane_high_timestamp < 10.0) {
    auto* mutable_nnp_fct_out =
        ptr_trajectory_pb->mutable_function_manager_out()
            ->mutable_nnp_fct_out();
    if (ptr_trajectory_pb->function_manager_out()
            .nnp_fct_out()
            .lane_change_infor() != ChangeLaneInfor::LANE_CHANGE_PENDING) {
      mutable_nnp_fct_out->set_dclc_audio_play(
          functionmanager::DclcAudioPlay::DCLC_NONE);
    }
    mutable_nnp_fct_out->set_lane_change_infor(
        functionmanager::ChangeLaneInfor::LANE_CHANGE_PENDING_ALERT);
    mutable_nnp_fct_out->set_lane_change_direction(dir);
    mutable_nnp_fct_out->set_nnp_rino_status(
        functionmanager::NNPRinoStatus::RINO_NO_REQUEST);
    last_is_pending_alert = true;
  } else if (is_lat_lon_active && is_lane_change_on_solid_lane &&
             last_is_pending_alert && last_lane_high_timestamp > 0.0) {
    last_cancel_timestamp = Clock::NowInSeconds();
    last_is_pending_alert = false;
  }
  if (Clock::NowInSeconds() - last_cancel_timestamp < 0.3) {
    auto* mutable_nnp_fct_out =
        ptr_trajectory_pb->mutable_function_manager_out()
            ->mutable_nnp_fct_out();
    if (mutable_nnp_fct_out != nullptr) {
      mutable_nnp_fct_out->set_dclc_audio_play(
          functionmanager::DclcAudioPlay::CANCEL_BRO);
      mutable_nnp_fct_out->set_lane_change_infor(
          functionmanager::ChangeLaneInfor::LANE_CHANGE_CANCEL);
    }
    ptr_trajectory_pb->mutable_soc_to_fct_bus()
        ->mutable_soc_to_fct_bus_u8()
        ->set_soc_close_turn_light_req(0x01);
  }
}

void EthHmi::UpdateDriverMode(  //NOLINT
    const TL::functionmanager::FunctionManagerIn& fct_in,
    const TL::functionmanager::FunctionManagerOut& fct_out,
    TL::hmi::NNPHmiOutput* const nnp_hmi_out) {
  const auto& is_valid_routing =
      fct_out.nnp_fct_out().nnp_activation_conditions().valid_of_lane_routing();
  if (fct_in.has_ta_pilot_mode()) {
    if (fct_in.ta_pilot_mode() == functionmanager::NNP && is_valid_routing) {
      nnp_hmi_out->set_driver_mode(TL::hmi::DriveMode::NNP);
    } else if (fct_in.has_adas_mode()) {
      if (fct_in.ta_pilot_mode() == functionmanager::ADAS &&
          fct_in.adas_mode() == functionmanager::ACC) {
        nnp_hmi_out->set_driver_mode(TL::hmi::DriveMode::ACC);
      } else if (fct_in.ta_pilot_mode() == functionmanager::ADAS &&
                 fct_in.adas_mode() == functionmanager::PILOT) {
        nnp_hmi_out->set_driver_mode(TL::hmi::DriveMode::NPILOT);
      }
    }
  } else {
    nnp_hmi_out->set_driver_mode(TL::hmi::DriveMode::MANAUL);
  }
}

void EthHmi::UpdateTsrData(  //NOLINT
    const TL::functionmanager::FunctionManagerIn& fct_in,
    const TL::functionmanager::NnpToFctOutput& nnp_fct_out,
    TL::hmi::NNPHmiOutput* const nnp_hmi_out) {
  const auto& nnp_sysstate = fct_in.fct_nnp_in().nnp_sysstate();
  if (nnp_sysstate == TL::functionmanager::NNPSysState::NNPS_ACTIVE ||
      nnp_sysstate == TL::functionmanager::NNPSysState::NNPS_LON_OVERRIDE) {
    nnp_hmi_out->set_is_nnp_active(true);
    nnp_hmi_out->set_map_spd_km_for_tsr(nnp_fct_out.map_spd_km());

  } else {
    nnp_hmi_out->set_is_nnp_active(false);
    nnp_hmi_out->set_map_spd_km_for_tsr(0.0);
  }
}

void EthHmi::UpdateHighlightID(  //NOLINT
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (ptr_trajectory_pb == nullptr || local_view == nullptr ||
      !local_view->HasFunctionManagerIn() ||
      !local_view->HasPerceptionObstacles()) {
    return;
  }
  auto* mutable_hmi_out = ptr_trajectory_pb->mutable_nnp_hmi_output();
  const auto& fct_in = local_view->GetFunctionManagerIn();
  const auto& perception_obstacles = local_view->GetPerceptionObstacles();
  const auto& object_decisions =
      ptr_trajectory_pb->decision().object_decision();
  if (object_decisions.decision().empty()) {
    return;
  }
  for (const auto& obstacle : perception_obstacles->perception_obstacle()) {
    for (const auto& obj_decision : object_decisions.decision()) {
      // filter the non obstacle case
      int32_t obj_id = obj_decision.perception_id();
      if (obj_id < 0 || obstacle.id() != obj_id) {
        continue;
      }
      if (obstacle.type() ==
          TL::perception::PerceptionObstacle::PEDESTRIAN) {
        const auto high_light = obj_id > 0 ? (0x01 << 26) : 0;
        AddDynamicSRObject(mutable_hmi_out, obj_id, high_light);
        break;
      }
      if (obstacle.type() == TL::perception::PerceptionObstacle::VEHICLE) {
        bool has_high_set = false;
        for (const auto& cur_decision : obj_decision.object_decision()) {
          if (cur_decision.object_tag_case() ==
                  TL::planning::ObjectDecisionType::ObjectTagCase::kFollow ||
              cur_decision.object_tag_case() ==
                  TL::planning::ObjectDecisionType::ObjectTagCase::kStop ||
              cur_decision.object_tag_case() ==
                  TL::planning::ObjectDecisionType::ObjectTagCase::kYield ||
              cur_decision.object_tag_case() ==
                  TL::planning::ObjectDecisionType::ObjectTagCase::
                      kOvertake) {
            const auto high_light = obj_id > 0 ? (0x01 << 4) : 0;
            AddDynamicSRObject(mutable_hmi_out, obj_id, high_light);
            has_high_set = true;
            break;
          }
        }
        if (has_high_set) {
          break;
        }
      }
    }
  }
}

void EthHmi::UpdateHmiData(  //NOLINT
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (ptr_trajectory_pb == nullptr || local_view == nullptr ||
      !local_view->HasLocalization() || !local_view->HasFunctionManagerIn() ||
      !ptr_trajectory_pb->has_function_manager_out()) {
    AERROR << "ptr is nullptr";
    return;
  }
  const auto& fct_in = local_view->GetFunctionManagerIn();
  const auto& fct_out = ptr_trajectory_pb->function_manager_out();
  auto* mutable_hmi_out = ptr_trajectory_pb->mutable_nnp_hmi_output();
  if (fct_in->ta_pilot_mode() == functionmanager::TaPilotMode::AVP) {
    UpdateHighlightID(local_view, ptr_trajectory_pb);
    if (fct_out.has_avp_fct_out() && fct_out.avp_fct_out().has_stage_type() &&
        fct_out.avp_fct_out().stage_type() ==
            functionmanager::AvpFctOut::CRUISING) {
      UpdateCruisingTraj(local_view, ptr_trajectory_pb);
    }
    if (fct_out.has_avp_fct_out() && fct_out.avp_fct_out().has_stage_type() &&
        fct_out.avp_fct_out().stage_type() ==
            functionmanager::AvpFctOut::PARKING) {
      UpdateParkingTraj(local_view, ptr_trajectory_pb);
    }
    if (fct_in->fct_avp_in().sys_mode() == functionmanager::AvpFctIn::LAPA) {
      UpdateMapPoint(local_view, ptr_trajectory_pb);
    }
    return;
  }
  const auto& warning_output = ptr_trajectory_pb->debug().warning_output();
  const auto& warning_status = ptr_trajectory_pb->warning_status();
  UpdateHmiChangeLaneState(ptr_trajectory_pb);

  u_int32_t id = 0;
  static constexpr u_int32_t mask = 0x03;
  bool is_nnp_active = false;
  bool is_pilot_active = false;
  is_pilot_active = fct_in->has_adas_mode() &&
                    (fct_in->adas_mode() == functionmanager::AdasMode::PILOT ||
                     fct_in->adas_mode() == functionmanager::AdasMode::ACC);
  const auto& nnp_state = fct_in->fct_nnp_in().nnp_sysstate();
  is_nnp_active = (nnp_state == NNPSysState::NNPS_ACTIVE ||
                   nnp_state == NNPSysState::NNPS_LAT_OVERRIDE ||
                   nnp_state == NNPSysState::NNPS_LON_OVERRIDE ||
                   nnp_state == NNPSysState::NNPS_OVERRIDE);
  const auto& nnp_fct_in = fct_in->fct_nnp_in();
  const auto& nnp_fct_out = ptr_trajectory_pb->function_manager_out();
  UpdateDriverMode(*fct_in, nnp_fct_out, mutable_hmi_out);
  UpdateTsrData(*fct_in,
                ptr_trajectory_pb->function_manager_out().nnp_fct_out(),
                mutable_hmi_out);

  // sr data
  u_int32_t high_light = 0;
  if (local_view->HasChassis()) {
    const auto& chassis = local_view->GetChassis();
    const auto& nnp_hmi_out = ptr_trajectory_pb->nnp_hmi_output();
    uint32 target_id = 0;
    for (const auto& nnp_obs_hightlight : nnp_hmi_out.nnp_obs_hightlight()) {
      const auto& hightlight_reason = nnp_obs_hightlight.highlight_reason();
      if (hightlight_reason == TL::hmi::NNPHmiOutput::LON_FOLLOW) {
        target_id = nnp_obs_hightlight.obs_hightlight_id();
        break;
      }
    }
    // AEB和FCW高亮目标临时使用acc的跟车目标
    if (chassis->has_aeb_obj_id() && chassis->aeb_obj_id() > 0) {
      high_light = 0x04;
      AddDynamicSRObject(mutable_hmi_out, target_id, high_light);
    }
    if (chassis->has_fcw_obj_id() && chassis->fcw_obj_id() > 0) {
      high_light = 0x02;
      AddDynamicSRObject(mutable_hmi_out, target_id, high_light);
    }
  }
  if (nnp_fct_in.has_acc_target_id()) {
    high_light = nnp_fct_in.acc_target_id() > 0 ? (0x01 << 4) : 0;
    AddDynamicSRObject(mutable_hmi_out, nnp_fct_in.acc_target_id(), high_light);
  }
  if (warning_output.has_fcta_debug()) {
    id = warning_output.fcta_debug().obj_critical_id();
    u_int32_t high_light = 0;
    if (warning_status.has_fcta_left_warning()) {
      high_light =
          warning_status.fcta_left_warning() != WarningLevel::NO_WARNING
              ? (0x01 << 6)
              : 0;
    }
    if (warning_status.has_fcta_right_warning()) {
      high_light =
          warning_status.fcta_right_warning() != WarningLevel::NO_WARNING
              ? (0x01 << 8)
              : 0;
    }
    AddDynamicSRObject(mutable_hmi_out, id, high_light);
  }
  if (warning_output.has_dow_debug()) {
    id = warning_output.dow_debug().dow_global().critical_obj_id();
    u_int32_t high_light = 0;
    static constexpr size_t K18 = 18;
    static constexpr size_t K20 = 20;
    if (warning_status.has_dow_left_warning()) {
      high_light =
          (static_cast<uint32>(warning_status.dow_left_warning()) << K18) &
          (mask << K18);
    }
    if (warning_status.has_dow_right_warning()) {
      high_light =
          (static_cast<uint32>(warning_status.dow_right_warning()) << K20) &
          (mask << K20);
    }
    AddDynamicSRObject(mutable_hmi_out, id, high_light);
  }

  if (warning_output.has_lca_debug()) {
    u_int32_t high_light = 0;
    const bool bsd_active = warning_output.has_bsd_debug() &&
                            warning_output.bsd_debug().bsd_warning_active();
    const auto bsd_id =
        bsd_active ? warning_output.bsd_debug().bsd_warning_criticle_id() : 0;
    const auto& lca_debug = warning_output.lca_debug();
    const bool lca_active =
        lca_debug.has_lca_global() &&
        lca_debug.lca_global().has_lca_warn_info() &&
        lca_debug.lca_global().lca_warn_info().lca_warning_active();

    id = lca_active ? lca_debug.lca_global().lca_warn_info().lca_warning_id()
                    : bsd_id;
    static constexpr size_t K14 = 14;
    static constexpr size_t K16 = 16;
    if (warning_status.has_lca_left_warning()) {
      const auto type = static_cast<uint32>(warning_status.lca_left_warning());
      if (type == WarningLevel::LV2_WARNING) {
        high_light = (type << K14) & (mask << K14);
        AddDynamicSRObject(mutable_hmi_out, id, high_light);
      }
    }
    if (warning_status.has_lca_right_warning()) {
      const auto type = static_cast<uint32>(warning_status.lca_right_warning());
      if (type == WarningLevel::LV2_WARNING) {
        high_light = (type << K16) & (mask << K16);
        AddDynamicSRObject(mutable_hmi_out, id, high_light);
      }
    }
  }

  if (warning_output.has_rcta_debug()) {
    id = warning_output.rcta_debug().obj_critical_id();
    u_int32_t high_light = 0;
    static constexpr size_t K10 = 10;
    static constexpr size_t K12 = 12;
    if (warning_status.has_rcta_left_warning()) {
      high_light =
          (static_cast<uint32>(warning_status.rcta_left_warning()) << K10) &
          (mask << K10);
    }
    if (warning_status.has_rcta_right_warning()) {
      high_light =
          (static_cast<uint32>(warning_status.rcta_right_warning()) << K12) &
          (mask << K12);
    }
    AddDynamicSRObject(mutable_hmi_out, id, high_light);
  }
  if (warning_status.has_rcw_warning() &&
      warning_status.rcw_warning() != planning::WarningLevel::NO_WARNING) {
    id = warning_output.rcw_debug().hmi_warning_id();
    u_int32_t high_light = 0;
    high_light = (0x01 << 22);
    AddDynamicSRObject(mutable_hmi_out, id, high_light);
  }
  if (is_nnp_active || is_pilot_active) {
    UpdateNNPAlcObsHighLight(ptr_trajectory_pb, mutable_hmi_out);
    UpdateNNPObsHighLight(ptr_trajectory_pb, mutable_hmi_out);
  }
}

void EthHmi::UpdateMapPoint(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (ptr_trajectory_pb == nullptr || !ptr_trajectory_pb->has_debug() ||
      !ptr_trajectory_pb->debug().has_ref_line() ||
      ptr_trajectory_pb->debug().ref_line().ref_points().empty() ||
      local_view == nullptr || !local_view->HasLocalization()) {
    AERROR << "ptr or localization is nullptr";
    return;
  }
  const auto& pose = local_view->GetLocalization()->pose();
  // set traj_map_coordinate
  auto* avp_to_hmi = ptr_trajectory_pb->mutable_avp_to_hmi();
  avp_to_hmi->clear_hmi_traj_map_coordinate();
  for (const auto& p : ptr_trajectory_pb->debug().ref_line().ref_points()) {
    auto* map_point = avp_to_hmi->add_hmi_traj_map_coordinate();
    std::pair<double, double> flu_point = common::math::ENUToFLU(
        p.x(), p.y(), pose.position().x(), pose.position().y(), pose.heading());
    map_point->set_x(flu_point.first);
    map_point->set_y(flu_point.second);
  }
}

void EthHmi::UpdateCruisingTraj(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (ptr_trajectory_pb == nullptr || !ptr_trajectory_pb->has_debug() ||
      !ptr_trajectory_pb->debug().has_planning_data() ||
      ptr_trajectory_pb->debug().planning_data().path().empty() ||
      !ptr_trajectory_pb->debug().planning_data().has_init_point() ||
      local_view == nullptr || !local_view->HasFunctionManagerIn() ||
      !local_view->GetFunctionManagerIn()->has_fct_avp_in() ||
      ptr_trajectory_pb->trajectory_point().empty()) {
    AERROR << "ptr is nullptr or vector is empty";
    return;
  }
  const auto& path_data = ptr_trajectory_pb->debug().planning_data().path();
  TL::common::Path path_used;
  for (const auto& path : path_data) {
    if (path.name() == "Planning PathData") {
      path_used = path;
      break;
    }
  }
  if (path_used.path_point().empty()) {
    return;
  }
  const auto& fct_in = local_view->GetFunctionManagerIn();
  const auto is_forward_path =
      fct_in->fct_avp_in().sys_mode() != functionmanager::AvpFctIn::TBA;
  const auto& dis_edge_to_center =
      is_forward_path ? common::VehicleConfigHelper::GetConfig()
                            .vehicle_param()
                            .front_edge_to_center()
                      : common::VehicleConfigHelper::GetConfig()
                            .vehicle_param()
                            .back_edge_to_center();
  //  显示范围最大是：轨迹长度+后轴到车头or车尾的长度
  const auto show_s_range = std::min(
      path_used.path_point().rbegin()->s(),
      ptr_trajectory_pb->trajectory_point().rbegin()->path_point().s() +
          dis_edge_to_center);
  std::vector<TL::common::Point2D> hmi_traj_enu_coordinate;
  for (const auto& point : path_used.path_point()) {
    if (point.s() >= show_s_range) {
      break;
    }
    TL::common::Point2D temp_point;
    temp_point.set_x(point.x());
    temp_point.set_y(point.y());
    hmi_traj_enu_coordinate.push_back(temp_point);
  }

  //  将显示范围内的path的全局坐标转为自车后轴FLU坐标(planning car坐标)
  const auto& init_point =
      ptr_trajectory_pb->debug().planning_data().init_point();
  for (const auto& point : hmi_traj_enu_coordinate) {
    auto* traj_local_coordinate = ptr_trajectory_pb->mutable_avp_to_hmi()
                                      ->add_hmi_traj_local_coordinate();
    std::pair<double, double> flu_point = common::math::ENUToFLU(
        point.x(), point.y(), init_point.path_point().x(),
        init_point.path_point().y(), init_point.path_point().theta());
    traj_local_coordinate->set_x(flu_point.first);
    traj_local_coordinate->set_y(flu_point.second);
  }
}

void EthHmi::UpdateParkingTraj(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (ptr_trajectory_pb == nullptr ||
      ptr_trajectory_pb->trajectory_point().empty() || local_view == nullptr ||
      !ptr_trajectory_pb->has_gear() || !ptr_trajectory_pb->has_debug() ||
      !ptr_trajectory_pb->debug().has_planning_data() ||
      !ptr_trajectory_pb->debug().planning_data().has_init_point()) {
    AERROR << "ptr is nullptr";
    return;
  }
  if (ptr_trajectory_pb->gear() != soc::Chassis::GEAR_DRIVE &&
      ptr_trajectory_pb->gear() != soc::Chassis::GEAR_REVERSE) {
    AERROR << "not target gear";
    return;
  }
  constexpr double kEpsilon = 1e-6;
  const bool is_forward = ptr_trajectory_pb->gear() == soc::Chassis::GEAR_DRIVE;
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  const auto edge_to_center = is_forward ? vehicle_param.front_edge_to_center()
                                         : -vehicle_param.back_edge_to_center();
  std::vector<TL::common::Point2D> hmi_traj_enu_coordinate_pose;
  TL::common::PathPoint start_point =
      ptr_trajectory_pb->debug().planning_data().init_point().path_point();
  //  求前保or后保行驶路径的全局坐标
  for (int i = 0; i < ptr_trajectory_pb->trajectory_point_size(); i++) {
    if (ptr_trajectory_pb->trajectory_point().at(i).relative_time() <
        kEpsilon) {
      start_point = ptr_trajectory_pb->trajectory_point().at(i).path_point();
      continue;
    }
    double x_tans_dis =
        edge_to_center *
        std::cos(
            ptr_trajectory_pb->trajectory_point().at(i).path_point().theta());
    double y_tans_dis =
        edge_to_center *
        std::sin(
            ptr_trajectory_pb->trajectory_point().at(i).path_point().theta());
    TL::common::Point2D temp_point;
    temp_point.set_x(
        ptr_trajectory_pb->trajectory_point().at(i).path_point().x() +
        x_tans_dis);
    temp_point.set_y(
        ptr_trajectory_pb->trajectory_point().at(i).path_point().y() +
        y_tans_dis);
    hmi_traj_enu_coordinate_pose.push_back(temp_point);
  }
  //  将前保or后保行驶路径的全局坐标转为自车后轴规划起始点坐标(planning car坐标)
  std::vector<TL::common::Point2D> hmi_traj_flu_coordinate_pose(
      hmi_traj_enu_coordinate_pose.size());
  for (int i = 0; i < hmi_traj_enu_coordinate_pose.size(); i++) {
    const auto& enu_point = hmi_traj_enu_coordinate_pose.at(i);
    std::pair<double, double> flu_point =
        common::math::ENUToFLU(enu_point.x(), enu_point.y(), start_point.x(),
                               start_point.y(), start_point.theta());
    hmi_traj_flu_coordinate_pose.at(i).set_x(flu_point.first);
    hmi_traj_flu_coordinate_pose.at(i).set_y(flu_point.second);
  }
  FillParkingTrajGap(is_forward, vehicle_param, &hmi_traj_flu_coordinate_pose);
  for (const auto& point : hmi_traj_flu_coordinate_pose) {
    auto* traj_local_coordinate = ptr_trajectory_pb->mutable_avp_to_hmi()
                                      ->add_hmi_traj_local_coordinate();
    traj_local_coordinate->set_x(point.x());
    traj_local_coordinate->set_y(point.y());
  }
}

void EthHmi::FillParkingTrajGap(
    const bool is_forward, const common::VehicleParam& vehicle_param,
    std::vector<TL::common::Point2D>* const hmi_traj_flu_coordinates) {
  if (nullptr == hmi_traj_flu_coordinates) {
    return;
  }
  double k = 0.0;
  constexpr double kEpsilon = 1e-6;
  constexpr double step_s = 0.1;
  const auto d = is_forward ? vehicle_param.front_edge_to_center() -
                                  vehicle_param.wheel_base()
                            : vehicle_param.back_edge_to_center();
  std::vector<TL::common::Point2D> hmi_fill_traj_flu_coordinates;
  if (hmi_traj_flu_coordinates->size() > 1 &&
      std::fabs(hmi_traj_flu_coordinates->at(1).x() -
                hmi_traj_flu_coordinates->at(0).x()) > kEpsilon) {
    k = (hmi_traj_flu_coordinates->at(1).y() -
         hmi_traj_flu_coordinates->at(0).y()) /
        (hmi_traj_flu_coordinates->at(1).x() -
         hmi_traj_flu_coordinates->at(0).x());
    const double theta = std::fabs(std::atan(k));
    const double R = d / (2 * sin(theta));
    const double step_theta = step_s / R;
    double acc_theta = -theta;
    while (acc_theta < theta) {
      hmi_fill_traj_flu_coordinates.emplace_back();
      hmi_fill_traj_flu_coordinates.back().set_x(R * sin(acc_theta) + d / 2);
      hmi_fill_traj_flu_coordinates.back().set_y(R *
                                                 (cos(theta) - cos(acc_theta)));
      acc_theta += step_theta;
    }
  } else {
    double acc_s = 0.0;
    while (acc_s < d) {
      hmi_fill_traj_flu_coordinates.emplace_back();
      hmi_fill_traj_flu_coordinates.back().set_x(acc_s);
      hmi_fill_traj_flu_coordinates.back().set_y(0.0);
      acc_s += step_s;
    }
  }
  const double x_offset = is_forward ? vehicle_param.wheel_base()
                                     : -vehicle_param.back_edge_to_center();
  for (auto& fill_point : hmi_fill_traj_flu_coordinates) {
    fill_point.set_x(fill_point.x() + x_offset);
    if (is_forward ^ (k > kEpsilon)) {
      fill_point.set_y(-1.0 * fill_point.y());
    }
  }
  if (!is_forward) {
    std::reverse(hmi_fill_traj_flu_coordinates.begin(),
                 hmi_fill_traj_flu_coordinates.end());
  }
  hmi_traj_flu_coordinates->insert(hmi_traj_flu_coordinates->begin(),
                                   hmi_fill_traj_flu_coordinates.begin(),
                                   hmi_fill_traj_flu_coordinates.end());
}

void EthHmi::AddDynamicSRObject(hmi::NNPHmiOutput* mutable_hmi_out,  //NOLINT
                                u_int32_t id, u_int32_t high_light) {
  if (mutable_hmi_out == nullptr) {
    AERROR << "hmi out ptr is null";
    return;
  }
  if (high_light > 0 && id > 0) {
    bool status = false;
    for (int i = 0; i < mutable_hmi_out->dynamic_sr_objects_size(); i++) {
      auto* sr_obj = mutable_hmi_out->mutable_dynamic_sr_objects(i);
      if (sr_obj != nullptr && sr_obj->id() == id) {
        sr_obj->set_is_high_light(sr_obj->is_high_light() | high_light);
        status = true;
        break;
      }
    }
    if (!status) {
      auto* fcta = mutable_hmi_out->add_dynamic_sr_objects();
      fcta->set_id(id);
      fcta->set_is_high_light(high_light);
    }
  }
}

void EthHmi::UpdateNNPAlcObsHighLight(
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb,
    hmi::NNPHmiOutput* const mutable_hmi_out) {
  const auto& nnp_hmi_out = ptr_trajectory_pb->nnp_hmi_output();
  lc_obs_warning_.clear();
  for (const auto& nnp_alc_obs_hightlight :
       nnp_hmi_out.nnp_alc_obs_hightlight()) {
    const auto& target_id = nnp_alc_obs_hightlight.obs_hightlight_id();
    const auto& hightlight_reason = nnp_alc_obs_hightlight.highlight_reason();
    if (hightlight_reason == hmi::NNPHmiOutput::ALC_LEFT ||
        hightlight_reason == hmi::NNPHmiOutput::ALC_RIGHT) {
      lc_obs_warning_.insert(target_id);
      uint32_t type =
          hightlight_reason == hmi::NNPHmiOutput::ALC_LEFT ? 0x01 : 0x00;
      type = hightlight_reason == hmi::NNPHmiOutput::ALC_RIGHT ? 0x02 : type;
      const auto high_light = target_id > 0 ? (type << 24) : 0;
      AddDynamicSRObject(mutable_hmi_out, target_id, high_light);
    }
  }
}

void EthHmi::UpdateNNPObsHighLight(
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb,
    hmi::NNPHmiOutput* const mutable_hmi_out) {
  const auto& nnp_hmi_out = ptr_trajectory_pb->nnp_hmi_output();
  for (const auto& nnp_obs_hightlight : nnp_hmi_out.nnp_obs_hightlight()) {
    const auto& target_id = nnp_obs_hightlight.obs_hightlight_id();
    const auto& hightlight_reason = nnp_obs_hightlight.highlight_reason();
    if (hightlight_reason == TL::hmi::NNPHmiOutput::LAT_NUDGE ||
        hightlight_reason == TL::hmi::NNPHmiOutput::LON_NUDGE) {
      const auto high_light = target_id > 0 ? (0x01 << 26) : 0;
      AddDynamicSRObject(mutable_hmi_out, target_id, high_light);
    } else if (hightlight_reason == TL::hmi::NNPHmiOutput::LON_FOLLOW &&
               lc_obs_warning_.count(target_id) == 0) {
      const auto high_light = target_id > 0 ? (0x01 << 4) : 0;
      AddDynamicSRObject(mutable_hmi_out, target_id, high_light);
    } else {
      // tmp nothing
    }
  }
}

}  // namespace planning
}  // namespace TL
