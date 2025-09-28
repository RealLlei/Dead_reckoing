/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/hmi/lon_hmi/spd_adapt/spd_adapt.h"
#include <sys/types.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/math_utils.h"
#include "common/time/clock.h"
#include "google/protobuf/stubs/port.h"
#include "map/hdmap/path.h"
#include "planning/common/frame.h"
#include "planning/common/util/common.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/map/map_road.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

using TL::common::math::ConvertDisplaySpdToReal;  //NOLINT
using TL::functionmanager::NNPSysState;

void SpdAdaptScenario::Init(
    const TL::functionmanager::SpeedAdaptConfig& speed_adapt_config) {
  config_ = speed_adapt_config;
  SpeedConventor::InitTable(speed_adapt_config);
}

void SpdAdaptScenario::UpdateInput(
    functionmanager::FunctionManagerIn* const fct_in,
    const functionmanager::FunctionManagerOut* fct_out,
    const TL::common::VehicleState& vehicle_state, int tgtspdctgset,
    int tgtspddrftset) {
  if (fct_in == nullptr || fct_out == nullptr) {
    return;
  }
  tgtspdctgset_ = tgtspdctgset;
  tgtspddrftset_ = tgtspddrftset;
  tgtspd_valid_ = tgtspddrftset_ >= config_.min_offset() &&
                  tgtspddrftset_ <= config_.max_offset();
  double cruise_speed = FLAGS_default_cruise_speed;
  auto* nnp_fct_in = fct_in->mutable_fct_nnp_in();
  const auto& nnp_state = nnp_fct_in->nnp_sysstate();
  is_nnp_active_ = (nnp_state == NNPSysState::NNPS_ACTIVE ||
                    nnp_state == NNPSysState::NNPS_LAT_OVERRIDE ||
                    nnp_state == NNPSysState::NNPS_LON_OVERRIDE ||
                    nnp_state == NNPSysState::NNPS_OVERRIDE ||
                    nnp_state == NNPSysState::NNPS_TO);

  fct_ctrl_spd_valid_ = nnp_fct_in->has_longitud_ctrl_setspeed();
  if (!fct_ctrl_spd_valid_) {
    nnp_fct_in->set_longitud_ctrl_cruise_speedms(cruise_speed);
    return;
  }

  cdcs_speed_limit_ = fct_in->fct_nnp_in().cdcs_info().cdcs_speed_limit();
  tsr_ = fct_in->fct_nnp_in().tsr_info();
  usr_adjust_cruise_speed_ =
      is_nnp_active_ && nnp_fct_in->usr_has_changed_cruise_spd();
  if (nnp_fct_in->has_driving_mode()) {
    driving_mode_ = nnp_fct_in->driving_mode();
  }
  fct_cruise_speed_km_ = nnp_fct_in->longitud_ctrl_setspeed();
  if (is_nnp_active_) {
    fct_in->mutable_fct_nnp_in()->set_usr_has_changed_cruise_spd(
        usr_adjust_cruise_speed_ || (tgtspd_valid_ && tgtspddrftset_ != 0) ||
        road_data_.is_unkonwn_road);
  }
  camera_data_.stucked =
      CameraDistanceStuck(cdcs_speed_limit_.camera_distance());
  fct_in->mutable_fct_nnp_in()
      ->mutable_cdcs_info()
      ->mutable_cdcs_speed_limit()
      ->set_camera_distance_stuck(camera_data_.stucked ||
                                  fct_in->ta_pilot_mode() ==
                                      functionmanager::ADAS);
  fct_cruise_speed_km_ = static_cast<uint32_t>(fmin(
      fmax(fct_cruise_speed_km_, config_.min_speed()), config_.max_speed()));
  const auto is_hdmap_mode =
      fct_out->has_fsm_state() &&
      fct_out->fsm_state() == functionmanager::MachineStateType::HDMAP_TYPE;
  SpeedConventor::UpdateOnlineTable(vehicle_state, is_hdmap_mode);
  const auto& chassis = vehicle_state.chassis();
  auto cruise_spd_ms = SpeedConventor::ConvertDisplaySpdToReal(
      static_cast<int>(fct_cruise_speed_km_));
  cruise_spd_ms = ProcessBadWeather(chassis, cruise_spd_ms);
  if (nnp_fct_in->has_longitud_ctrl_cruise_speedms()) {
    return;
  }

  nnp_fct_in->set_longitud_ctrl_cruise_speedms(cruise_spd_ms);
}

void SpdAdaptScenario::UpdateOutput(
    functionmanager::FunctionManagerOut* const fct_out,
    const std::shared_ptr<hdmap::PncMap>& pnc_map,
    planning::Frame* const frame) {
  if (fct_out == nullptr || pnc_map == nullptr || frame == nullptr) {
    return;
  }
#ifdef FOR_BAIDU_SIMULATION
  ProcessBaiduSim(fct_out, pnc_map, frame);
  return;
#endif
  if (fct_out->hdmap_sub_state() == functionmanager::LOCAL_HDMAP_TYPE) {
    // ncp
    UpdateNCPOutput(fct_out, pnc_map, frame);
  } else {
    UpdateNNPOutput(fct_out, pnc_map, frame);
  }
}

double SpdAdaptScenario::ProcessDrivingMode() const {
  auto ratio = config_.speed_ratio_by_driver_mode().default_ratio();
  if (config_.speed_ratio_by_driver_mode().enable_driver_mode()) {
    switch (driving_mode_) {
      case functionmanager::DrivingMode::UNKNOWN_MODE:
        ratio = config_.speed_ratio_by_driver_mode().default_ratio();
        break;
      case functionmanager::DrivingMode::BASIC:
        ratio = config_.speed_ratio_by_driver_mode().basic_ratio();
        break;
      case functionmanager::DrivingMode::NORMAL:
        ratio = config_.speed_ratio_by_driver_mode().normal_ratio();
        break;
      case functionmanager::DrivingMode::RADICAL:
        ratio = config_.speed_ratio_by_driver_mode().radical_ratio();
        break;
      default:
        ratio = config_.speed_ratio_by_driver_mode().default_ratio();
        break;
    }
  }
  return ratio;
}

double SpdAdaptScenario::ProcessBadWeather(const soc::Chassis& chassis,
                                           double orin_speed_ms) {
  auto target_speed_ms = orin_speed_ms;
  if (usr_adjust_cruise_speed_ ||
      !config_.speed_ratio_by_bad_weather().enable() ||
      !chassis.has_front_wiper_status()) {
    return target_speed_ms;
  }
  switch (chassis.front_wiper_status().status()) {
    case soc::FrontWiperStatus::RESERVED:
    case soc::FrontWiperStatus::OFF:
      break;
    case soc::FrontWiperStatus::LOW:
      target_speed_ms =
          orin_speed_ms * config_.speed_ratio_by_bad_weather().low_ratio();
      break;
    case soc::FrontWiperStatus::HIGH:
      target_speed_ms =
          orin_speed_ms * config_.speed_ratio_by_bad_weather().high_ratio();
      break;
    default:
      target_speed_ms = orin_speed_ms;
  }
  return target_speed_ms;
}

void SpdAdaptScenario::UpdateNNPOutput(
    functionmanager::FunctionManagerOut* const fct_out,
    const std::shared_ptr<hdmap::PncMap>& pnc_map,
    planning::Frame* const frame) {
  if (pnc_map == nullptr || fct_out == nullptr || frame == nullptr) {
    return;
  }
  auto* nnp_fct_out = fct_out->mutable_nnp_fct_out();
  if (nnp_fct_out == nullptr) {
    return;
  }
  use_memery_ = false;
  CacheRoadElement(pnc_map, fct_out, frame);
  nnp_fct_out->set_map_spd_km(road_data_.map_speed_km);
  auto target_spd_km = road_data_.cal_road_speed_km;
  double ratio = ProcessDrivingMode();
  if (!fct_ctrl_spd_valid_) {
    // 没有fct输入情况下用地图限速乘以驾驶风格系数
    target_spd_km = static_cast<int>(target_spd_km * ratio);
    need_adapt_ = false;
  } else {
    target_spd_km = SelectAdapt(fct_out);
  }
  ADEBUG << " target_spd_km : " << target_spd_km
         << " need_adapt_ : " << need_adapt_;
  nnp_fct_out->set_need_speed_adapt(need_adapt_);
  nnp_fct_out->set_adapt_cruise_speed_km(target_spd_km);
  uint32_t soc_04_val = fct_out->has_soc_2_fct_tbd_u32_04()
                            ? fct_out->soc_2_fct_tbd_u32_04()
                            : 0x00;
  fct_out->set_soc_2_fct_tbd_u32_04(
      soc_04_val | (static_cast<uint32_t>(target_spd_km) << 24));
  camera_data_.last_valid = camera_data_.valid;
  camera_data_.last_distance = camera_data_.distance;
  camera_data_.last_speed_km = camera_data_.speed_km;
  camera_data_.last_stucked = camera_data_.stucked;
  road_data_.last_is_main_road = road_data_.is_main_road;
  road_data_.last_road_speed_km = road_data_.road_speed_km;
  road_data_.last_is_unkonwn_road = road_data_.is_unkonwn_road;
  road_data_.last_in_tunnel = road_data_.in_tunnel;
  road_data_.last_sd_speed_km = road_data_.sd_speed_km;
  road_data_.last_sd_speed_valid = road_data_.last_sd_speed_valid;
  tunnel_data_.last_speed_km = tunnel_data_.speed_km;
  tunnel_data_.last_distance = tunnel_data_.distance;
  last_is_nnp_active_ = is_nnp_active_;
}

void SpdAdaptScenario::UpdateNCPOutput(
    functionmanager::FunctionManagerOut* const fct_out,
    const std::shared_ptr<hdmap::PncMap>& pnc_map,
    planning::Frame* const frame) {
  if (pnc_map == nullptr || fct_out == nullptr || frame == nullptr) {
    return;
  }
  auto* nnp_fct_out = fct_out->mutable_nnp_fct_out();
  auto target_lane = pnc_map->GetADCWaypoint().lane;
  if (target_lane == nullptr) {
    return;
  }
  const auto is_outermost_lane =
      TL::planning::util::IsOutermostLane(target_lane, pnc_map);

  const auto is_city_road =
      target_lane->GetRoadType() == TL::hdmap::Road::CITY_ROAD;
  const auto lane_type = target_lane->GetSectionType();
  road_data_.is_main_road =
      (lane_type == TL::hdmap::RoadSection::MultipleCarriageWay ||
       lane_type == TL::hdmap::RoadSection::SingleCarriageWay) ||
      (lane_type == TL::hdmap::RoadSection::UNKNOWN && is_nnp_active_);
  road_data_.road_speed_km =
      static_cast<int>(std::round(target_lane->GetSectionMaxSpeed() * kKmh2Ms));
  if (is_city_road && is_outermost_lane &&
      road_data_.road_speed_km >=
          static_cast<int>(
              config_.city_outermost_lane_need_limit_min_speed())) {
    road_data_.road_speed_km =
        static_cast<int>(road_data_.road_speed_km *
                         config_.city_outermost_lane_speed_limit_ratio());
    road_data_.road_speed_km =
        static_cast<int>(std::round((road_data_.road_speed_km + 1) / 10) * 10);
  }
  nnp_fct_out->set_map_spd_km(road_data_.road_speed_km);
  nnp_fct_out->set_curr_lane_spd_km(static_cast<uint32>(
      std::round(target_lane->lane().speed_limit() * kKmh2Ms)));
  auto target_spd_km = road_data_.road_speed_km;
  double ratio = ProcessDrivingMode();
  if (!fct_ctrl_spd_valid_) {
    // 没有fct输入情况下用地图限速乘以驾驶风格系数
    target_spd_km = static_cast<int>(target_spd_km * ratio);
    need_adapt_ = false;
  } else {
    // 城区的车速匹配先做成地图限速更新就更新的,避免超速
    need_adapt_ = usr_adjust_cruise_speed_ ? false
                                           : road_data_.road_speed_km !=
                                                 road_data_.last_road_speed_km;
    // 激活的时候重新匹配一下
    need_adapt_ = need_adapt_ || (is_nnp_active_ && !last_is_nnp_active_);
    target_spd_km = static_cast<int>(target_spd_km * ratio);
  }
  target_spd_km =
      std::min(target_spd_km, static_cast<int>(config_.max_speed()));
  nnp_fct_out->set_need_speed_adapt(need_adapt_);
  nnp_fct_out->set_adapt_cruise_speed_km(target_spd_km);

  road_data_.last_is_main_road = road_data_.is_main_road;
  road_data_.last_road_speed_km = road_data_.road_speed_km;
  last_is_nnp_active_ = is_nnp_active_;
  road_data_.last_is_unkonwn_road = road_data_.is_unkonwn_road;
}

void SpdAdaptScenario::SelectCurrRoadSpeed(
    const std::shared_ptr<hdmap::PncMap>& pnc_map,
    planning::Frame* const frame) {
  UNUSED(frame);
  const auto& adc_lane_type = pnc_map->GetAdcLaneType();
  const auto& adc_lane = pnc_map->GetADCWaypoint().lane;
  if (adc_lane == nullptr) {
    return;
  }
  road_data_.in_tunnel = adc_lane != nullptr &&
                         adc_lane->lane().has_map_lane_type() &&
                         adc_lane->lane().map_lane_type().tunnel_lane() &&
                         !std::isinf(tunnel_data_.last_distance);
  road_data_.is_main_road =
      adc_lane_type == TL::hdmap::RoadSection::MultipleCarriageWay ||
      adc_lane_type == TL::hdmap::RoadSection::SingleCarriageWay ||
      (adc_lane_type == TL::hdmap::RoadSection::UNKNOWN && is_nnp_active_);

  road_data_.is_unkonwn_road =
      adc_lane_type == TL::hdmap::RoadSection::UNKNOWN && is_nnp_active_;

  road_data_.map_speed_km =
      static_cast<int>(std::round(adc_lane->GetSectionMaxSpeed() * kKmh2Ms));
  const auto has_cdcs_road_speed = cdcs_speed_limit_.has_road_speed_limit() &&
                                   cdcs_speed_limit_.road_speed_limit() >= 80;
  // 优先级 座舱 > tsr > 地图 先不考虑tsr，不好用
  road_data_.road_speed_km =
      has_cdcs_road_speed
          ? std::max(road_data_.map_speed_km,
                     static_cast<int>(cdcs_speed_limit_.road_speed_limit()))
          : road_data_.map_speed_km;
  road_data_.road_speed_km = common::math::Clamp(
      road_data_.road_speed_km, config_.min_speed(), config_.max_speed());
  road_data_.sd_speed_km = cdcs_speed_limit_.road_speed_limit();
  road_data_.sd_speed_valid =
      cdcs_speed_limit_.road_speed_limit() >= 80 &&
      cdcs_speed_limit_.road_speed_limit() <= config_.max_speed();
}

void SpdAdaptScenario::ProcessBaiduSim(
    functionmanager::FunctionManagerOut* const fct_out,
    const std::shared_ptr<hdmap::PncMap>& pnc_map,
    planning::Frame* const frame) {
  if (pnc_map == nullptr || fct_out == nullptr || frame == nullptr) {
    return;
  }
  SelectCurrRoadSpeed(pnc_map, frame);
  if (!fct_ctrl_spd_valid_) {
    fct_out->mutable_nnp_fct_out()->set_tsr_speedsign(road_data_.road_speed_km);
    return;
  }
  const auto upper_speed_km =
      usr_adjust_cruise_speed_ ? config_.max_speed() : road_data_.road_speed_km;
  fct_out->mutable_nnp_fct_out()->set_tsr_speedsign(
      fmin(fct_cruise_speed_km_, upper_speed_km));
}

int SpdAdaptScenario::GetCalSpdOffset(int origin_speed_km) const {
  return tgtspd_valid_ ? (tgtspdctgset_ == 0)
                             ? tgtspddrftset_
                             : static_cast<int>(std::ceil(
                                   origin_speed_km * tgtspddrftset_ * 0.01))
                       : 0;
}

void SpdAdaptScenario::CacheRoadElement(
    const std::shared_ptr<hdmap::PncMap>& pnc_map,
    functionmanager::FunctionManagerOut* fct_out, planning::Frame* frame) {
  if (pnc_map == nullptr || fct_out == nullptr || frame == nullptr) {
    return;
  }
  const auto* reference_line_info = frame->FindDriveReferenceLineInfo();
  if (reference_line_info == nullptr) {
    return;
  }
  SelectCurrRoadSpeed(pnc_map, frame);
  static int last_tgtspddrftset = 0;
  static int last_tgtspdctgset = 0;
  spd_offset_ = GetCalSpdOffset(road_data_.road_speed_km);

  tgtspd_changed_ = (tgtspd_valid_ &&
                     (tgtspddrftset_ != last_tgtspddrftset ||
                      ((tgtspdctgset_ == 0 || last_tgtspdctgset == 0) &&
                       tgtspdctgset_ != last_tgtspdctgset)) &&
                     is_nnp_active_);
  road_data_.cal_road_speed_km =
      common::math::Clamp(road_data_.road_speed_km + spd_offset_,
                          config_.min_speed(), config_.max_speed());
  // pre  process
  // camera data
  camera_data_.valid = cdcs_speed_limit_.has_camera_speed_limit_km() &&
                       cdcs_speed_limit_.has_camera_distance() &&
                       cdcs_speed_limit_.camera_speed_limit_km() >=
                           (road_data_.in_tunnel ? 60 : 80);
  camera_data_.distance = cdcs_speed_limit_.camera_distance();
  camera_data_.speed_km =
      static_cast<int>(cdcs_speed_limit_.camera_speed_limit_km());

  camera_data_.near =
      camera_data_.distance > 0 &&
      camera_data_.distance <= config_.camera_distance_adapt_distance() &&
      camera_data_.valid && !camera_data_.stucked;
  // ramp data
  const auto& ramp_lane = reference_line_info->GetFrontRamp();
  ramp_data_.near = ramp_lane.start_lane != nullptr &&
                    ramp_lane.start_s > kMinDis &&
                    ramp_lane.start_s < kMaxDis && road_data_.is_main_road;
  ramp_data_.distance = ramp_lane.start_lane != nullptr
                            ? ramp_lane.start_s
                            : std::numeric_limits<double>::infinity();
  ramp_data_.speed = ramp_lane.speed_limit;
  if (!ramp_data_.near) {
    ramp_data_.adapted = false;
  }
  ADEBUG << "cal_road_speed_km :  " << road_data_.cal_road_speed_km
         << " camera dis : " << camera_data_.distance
         << " camera speed : " << camera_data_.speed_km
         << "is unknown road : " << road_data_.is_unkonwn_road;
  // tunnel data
  const auto& tunnel_lane = reference_line_info->GetFrontTunnel();
  tunnel_data_.find = tunnel_lane.start_lane != nullptr;
  tunnel_data_.distance = tunnel_data_.find
                              ? tunnel_lane.start_s
                              : std::numeric_limits<double>::infinity();
  const auto tunnel_lane_speed_km =
      tunnel_data_.find
          ? static_cast<int>(std::round(
                tunnel_lane.start_lane->GetSectionMaxSpeed() * kKmh2Ms))
          : road_data_.road_speed_km;
  tunnel_data_.speed_km = common::math::Clamp(
      tunnel_lane_speed_km + GetCalSpdOffset(tunnel_lane_speed_km),
      config_.min_speed(), config_.max_speed());
  const auto adc_speed = frame->vehicle_state().linear_velocity();
  tunnel_data_.dec =
      tunnel_data_.find
          ? (pow((tunnel_data_.speed_km / 3.6), 2) - pow(adc_speed, 2)) /
                (fmax(0.01, 2 * (tunnel_lane.start_s - 200)))
          : 0.0;
  tunnel_data_.near =
      tunnel_data_.find && tunnel_lane.start_s < 200 &&
      (!std::isinf(tunnel_data_.last_distance) || tunnel_data_.distance != 0);
  ADEBUG << " tunnel dis  : " << tunnel_data_.distance
         << "dec : " << tunnel_data_.dec << " near : " << tunnel_data_.near
         << "speed km : " << tunnel_data_.speed_km
         << " spd_offset_ : " << spd_offset_
         << "curr road : " << road_data_.cal_road_speed_km
         << " in tunnel : " << road_data_.in_tunnel;
  tunnel_data_.faraway =
      !tunnel_data_.find ||
      common::math::double_type::DefinitelyGreater(tunnel_data_.distance, 500);
  if (tunnel_data_.faraway && !road_data_.in_tunnel &&
      !road_data_.last_in_tunnel) {
    tunnel_data_.ignore_into = false;
    tunnel_data_.ignore_out = false;
  }
  if (config_.enable_tunnel_debug()) {
    auto* tunnel_debug =
        fct_out->mutable_nnp_fct_out()->mutable_tunnel_spd_adapt_debug();
    tunnel_debug->set_find_tunnel(tunnel_data_.find);
    tunnel_debug->set_tunnel_dis(tunnel_data_.distance);
    if (road_data_.in_tunnel) {
      tunnel_debug->set_curr_state(
          functionmanager::TunnelSpdAdaptDebug::IN_TUNNEL);
    }
  }

  // tollhouse_data
  const auto has_tollhouse =
      fct_out->has_odd_info() &&
      fct_out->odd_info().type() == routing::LaneWaypointType::ODD_START &&
      fct_out->odd_info().odd_type() == routing::LaneWaypoint::SPECIAL_AREA;
  tollhouse_data_.near =
      has_tollhouse && fct_out->odd_info().to_end_len() > kMinDis &&
      fct_out->odd_info().to_end_len() < config_.tollhouse_adapt_distance();
  tollhouse_data_.distance = has_tollhouse
                                 ? fct_out->odd_info().to_end_len()
                                 : std::numeric_limits<double>::infinity();
  if (!tollhouse_data_.near) {
    tollhouse_data_.adapted = false;
  }
  last_tgtspddrftset = tgtspddrftset_;
  last_tgtspdctgset = tgtspdctgset_;
}

int SpdAdaptScenario::SelectAdapt(
    functionmanager::FunctionManagerOut* const fct_out) {
  need_adapt_ = false;
  use_memery_ = false;
  int target_speed_km = road_data_.cal_road_speed_km;
  // 没有激活
  if (!is_nnp_active_) {
    return AdaptNotActive(target_speed_km);
  }
  // 激活
  if (is_nnp_active_ && !last_is_nnp_active_) {
    target_speed_km = AdaptActive(target_speed_km);
    ADEBUG << " AdaptActive target_spd_km : " << target_speed_km
           << " need_adapt_ : " << need_adapt_;
  }
  // 道路类型变了
  if ((road_data_.is_main_road != road_data_.last_is_main_road &&
       !road_data_.last_is_unkonwn_road && !road_data_.is_unkonwn_road)) {
    target_speed_km = AdaptRoadTypeChanged(road_data_.cal_road_speed_km);
    ADEBUG << "AdaptRoadTypeChanged  target_spd_km : " << target_speed_km
           << " need_adapt_ : " << need_adapt_;
  }
  // 道路限速变了
  if (road_data_.road_speed_km != road_data_.last_road_speed_km &&
      (!road_data_.last_is_unkonwn_road && !road_data_.is_unkonwn_road ||
       road_data_.active_on_unkonwn_road ||
       (road_data_.is_unkonwn_road && road_data_.sd_speed_valid &&
        road_data_.last_sd_speed_valid &&
        road_data_.sd_speed_km != road_data_.last_sd_speed_km)) &&
      !usr_adjust_cruise_speed_ && !tunnel_data_.adapted &&
      (!road_data_.in_tunnel || tunnel_data_.ignore_into)) {
    target_speed_km = AdaptRoadSpeedChanged(road_data_.cal_road_speed_km);
    ADEBUG << "AdaptRoadSpeedChanged target_spd_km : " << target_speed_km
           << " need_adapt_ : " << need_adapt_;
  }
  // 车速偏移设置变了
  if (tgtspd_changed_ && is_nnp_active_ && last_is_nnp_active_) {
    target_speed_km = AdaptTgtspdChanged(road_data_.cal_road_speed_km);
    ADEBUG << " AdaptTgtspdChanged target_spd_km : " << target_speed_km
           << " need_adapt_ : " << need_adapt_;
  }
  // 进隧道
  if (tunnel_data_.find && !tunnel_data_.ignore_into &&
      (tunnel_data_.dec < -0.6 || tunnel_data_.near) && !tunnel_data_.adapted) {
    target_speed_km = AdaptIntoTunnel(target_speed_km, fct_out);
    ADEBUG << "  AdaptIntoTunnel target_spd_km : " << target_speed_km
           << " need_adapt_ : " << need_adapt_;
  }
  // 出隧道
  if (road_data_.last_in_tunnel && !road_data_.in_tunnel &&
      tunnel_data_.adapted) {
    target_speed_km = AdaptOutTunnel(target_speed_km, fct_out);
    ADEBUG << "AdaptOutTunnel target_spd_km : " << target_speed_km
           << " need_adapt_ : " << need_adapt_;
  }
  // 下匝道的过程中的80
  if (ramp_data_.near && !ramp_data_.adapted) {
    target_speed_km = AdaptIntoRamp(target_speed_km);
    ADEBUG << " AdaptIntoRamp target_spd_km : " << target_speed_km
           << " need_adapt_ : " << need_adapt_;
  }
  // 进摄像头
  if (camera_data_.near && !camera_data_.adapted) {
    target_speed_km = AdaptIntoCamera(target_speed_km);
    ADEBUG << "AdaptIntoCamera target_spd_km : " << target_speed_km
           << " need_adapt_ : " << need_adapt_;
  }
  // 连续摄像头
  if (camera_data_.near && camera_data_.adapted && !usr_adjust_cruise_speed_ &&
      (!camera_data_.last_stucked && camera_data_.last_valid &&
       camera_data_.last_speed_km != camera_data_.speed_km)) {
    target_speed_km = AdaptCameraSpeedChanged(target_speed_km);
    ADEBUG << "AdaptCameraSpeedChanged  target_spd_km : " << target_speed_km
           << " need_adapt_ : " << need_adapt_;
  }
  // 出摄像头
  if (camera_data_.last_distance > 0 &&
      camera_data_.last_distance <= config_.camera_distance_adapt_distance() &&
      !camera_data_.last_stucked &&
      (camera_data_.distance == 0 ||
       camera_data_.distance > config_.camera_distance_adapt_distance() ||
       (camera_data_.stucked && !camera_data_.last_stucked)) &&
      camera_data_.adapted) {
    target_speed_km = AdaptOutCamera(target_speed_km);
    ADEBUG << "AdaptOutCamera target_spd_km : " << target_speed_km
           << " need_adapt_ : " << need_adapt_;
  }
  // 进收费站
  if (tollhouse_data_.near && !tollhouse_data_.adapted &&
      fct_cruise_speed_km_ > config_.tollhouse_adapt_speedkm()) {
    target_speed_km = AdaptIntoTollhouse(target_speed_km);
    ADEBUG << "AdaptIntoTollhouse target_spd_km : " << target_speed_km
           << " need_adapt_ : " << need_adapt_;
  }
  return target_speed_km;
}

int SpdAdaptScenario::AdaptNotActive(const int origin_speed_km) {
  need_adapt_ = false;
  use_memery_ = false;
  ramp_data_.adapted = false;
  tollhouse_data_.adapted = false;
  camera_data_.adapt_time = 0.0;
  camera_data_.adapted = false;
  camera_data_.state_before_camera = false;
  camera_data_.ignore_out = false;
  camera_data_.road_changed_during_camera = false;
  tunnel_data_.adapt_time = 0, 0;
  tunnel_data_.adapted = false;
  tunnel_data_.state_before_tunnel = false;
  tunnel_data_.ignore_into = false;
  tunnel_data_.ignore_out = false;
  road_data_.active_on_unkonwn_road = false;
  if (camera_data_.near &&
      road_data_.cal_road_speed_km > camera_data_.speed_km) {
    camera_data_.adapted = true;
    camera_data_.mem_speed_km = origin_speed_km;
    camera_data_.state_before_camera = false;
    camera_data_.road_speed_before_camera = road_data_.cal_road_speed_km;
    camera_data_.ignore_out = origin_speed_km <= camera_data_.speed_km;
    return std::min(origin_speed_km, camera_data_.speed_km);
  }
  if (tunnel_data_.near &&
      road_data_.cal_road_speed_km > tunnel_data_.speed_km) {
    tunnel_data_.adapted = true;
    tunnel_data_.mem_speed_km = origin_speed_km;
    tunnel_data_.state_before_tunnel = false;
    tunnel_data_.mem_speed_km = road_data_.cal_road_speed_km;
    tunnel_data_.ignore_out = origin_speed_km <= tunnel_data_.speed_km;
    return std::min(origin_speed_km, camera_data_.speed_km);
  }
  if (ramp_data_.near) {
    return std::min(config_.speed_display_for_ramp_dec(), origin_speed_km);
  }
  if (tollhouse_data_.near) {
    return std::min(config_.tollhouse_adapt_speedkm(), origin_speed_km);
  }
  return origin_speed_km;
}

int SpdAdaptScenario::AdaptActive(const int origin_speed_km) {
  need_adapt_ = true;
  use_memery_ = false;
  road_data_.active_on_unkonwn_road = road_data_.is_unkonwn_road;
  if (camera_data_.near &&
      road_data_.cal_road_speed_km > camera_data_.speed_km) {
    // 激活的时候如果是摄像头范围内，地图限速大于摄像头限速，应该匹配地图限速
    camera_data_.adapted = true;
    camera_data_.mem_speed_km = origin_speed_km;
    camera_data_.state_before_camera = false;
    camera_data_.road_speed_before_camera = road_data_.cal_road_speed_km;
    camera_data_.ignore_out = origin_speed_km <= camera_data_.speed_km;
    return std::min(origin_speed_km, camera_data_.speed_km);
  }
  if (tunnel_data_.near) {
    tunnel_data_.adapted = true;
    tunnel_data_.state_before_tunnel = false;
    tunnel_data_.ignore_out = false;
    tunnel_data_.ignore_into = false;
    tunnel_data_.mem_speed_km = road_data_.cal_road_speed_km;
    return std::min(tunnel_data_.speed_km, road_data_.cal_road_speed_km);
  }
  if (ramp_data_.near) {
    return std::min(config_.speed_display_for_ramp_dec(), origin_speed_km);
  }
  if (tollhouse_data_.near) {
    return std::min(config_.tollhouse_adapt_speedkm(), origin_speed_km);
  }
  return origin_speed_km;
}

int SpdAdaptScenario::AdaptRoadTypeChanged(const int origin_speed_km) {
  // 道路类型变的时候，
  need_adapt_ = true;
  use_memery_ = false;
  if (camera_data_.near &&
      road_data_.cal_road_speed_km > camera_data_.speed_km &&
      !usr_adjust_cruise_speed_) {
    // 看一下摄像头，如果没有超过摄像头限速，匹配，如果超了摄像头限速，先按摄像头的走，出了摄像头再匹配
    need_adapt_ = true;
    camera_data_.road_changed_during_camera = true;
    camera_data_.road_speed_during_camera = road_data_.cal_road_speed_km;
    camera_data_.ignore_out = false;
    return camera_data_.speed_km;
  }

  return road_data_.is_main_road ? origin_speed_km
                                 : std::min(origin_speed_km, 80);
}

int SpdAdaptScenario::AdaptRoadSpeedChanged(const int origin_speed_km) {
  // 道路限速变的时候，
  need_adapt_ = true;
  use_memery_ = false;
  road_data_.active_on_unkonwn_road = false;
  if (camera_data_.near &&
      road_data_.cal_road_speed_km > camera_data_.speed_km &&
      !usr_adjust_cruise_speed_) {
    // 看一下摄像头，如果没有超过摄像头限速，匹配，如果超了摄像头限速，先按摄像头的走，出了摄像头再匹配
    need_adapt_ = true;
    camera_data_.road_changed_during_camera = true;
    camera_data_.road_speed_during_camera = road_data_.cal_road_speed_km;
    camera_data_.ignore_out = false;
    return camera_data_.speed_km;
  }
  if (camera_data_.near &&
      road_data_.cal_road_speed_km <= camera_data_.speed_km &&
      !usr_adjust_cruise_speed_) {
    // 看一下摄像头，如果没有超过摄像头限速，匹配，出摄像头不匹配
    need_adapt_ = true;
    camera_data_.road_changed_during_camera = true;
    camera_data_.road_speed_during_camera = road_data_.cal_road_speed_km;
    camera_data_.ignore_out = true;
    return origin_speed_km;
  }

  if (!camera_data_.near && camera_data_.adapted) {
    need_adapt_ = true;
    camera_data_.ignore_out = true;
  }
  if (ramp_data_.near) {
    need_adapt_ = origin_speed_km < config_.speed_display_for_ramp_dec();
    return std::min(config_.speed_display_for_ramp_dec(), origin_speed_km);
  }
  return road_data_.is_main_road ? origin_speed_km
                                 : std::min(origin_speed_km, 80);
}

int SpdAdaptScenario::AdaptTgtspdChanged(const int origin_speed_km) {
  need_adapt_ = true;
  use_memery_ = false;
  // 只要用户调了，立即匹配，出摄像头的时候不恢复
  if (camera_data_.near) {
    camera_data_.ignore_out = true;
  }
  // 隧道内调了，立即匹配，出隧道的时候也不恢复
  if (tunnel_data_.adapted) {
    tunnel_data_.ignore_out = true;
  }
  return origin_speed_km;
}

int SpdAdaptScenario::AdaptIntoRamp(const int origin_speed_km) {
  need_adapt_ = true;
  use_memery_ = false;
  ramp_data_.adapted = true;
  if (fct_cruise_speed_km_ <= config_.speed_display_for_ramp_dec()) {
    need_adapt_ = false;
    return origin_speed_km;
  }
  return config_.speed_display_for_ramp_dec();
}

int SpdAdaptScenario::AdaptIntoTunnel(
    const int origin_speed_km,
    functionmanager::FunctionManagerOut* const fct_out) {
  UNUSED(origin_speed_km);
  tunnel_data_.adapted = true;
  need_adapt_ = fct_cruise_speed_km_ > tunnel_data_.speed_km;
  use_memery_ = false;
  tunnel_data_.state_before_tunnel = usr_adjust_cruise_speed_;
  tunnel_data_.mem_speed_km = static_cast<int>(fct_cruise_speed_km_);
  tunnel_data_.adapt_time = common::Clock::NowInMicroseconds();
  if (config_.enable_tunnel_debug()) {
    auto* tunnel_debug =
        fct_out->mutable_nnp_fct_out()->mutable_tunnel_spd_adapt_debug();
    tunnel_debug->set_tunnel_adapt(need_adapt_);
    tunnel_debug->set_curr_state(
        functionmanager::TunnelSpdAdaptDebug::INTO_TUNNEL);
  }
  ADEBUG << " need adapt : " << need_adapt_;
  return tunnel_data_.speed_km;
}

int SpdAdaptScenario::AdaptOutTunnel(
    const int origin_speed_km,
    functionmanager::FunctionManagerOut* const fct_out) {
  auto target_speed_km = origin_speed_km;
  // 如果用户隧道内调整了
  if (tunnel_data_.ignore_out || usr_adjust_cruise_speed_) {
    // 忽略说明之前用户没有调滚轮，但是调了偏移设置项
    tunnel_data_.adapted = false;
    tunnel_data_.ignore_out = false;
    need_adapt_ = false;
    // 连续隧道忽略
    tunnel_data_.ignore_into = !tunnel_data_.faraway;
    if (config_.enable_tunnel_debug()) {
      auto* tunnel_debug =
          fct_out->mutable_nnp_fct_out()->mutable_tunnel_spd_adapt_debug();
      tunnel_debug->set_tunnel_adapt(need_adapt_);
      tunnel_debug->set_curr_state(
          functionmanager::TunnelSpdAdaptDebug::OUT_TUNNEL);
    }
    return target_speed_km;
  }
  if (tunnel_data_.faraway) {
    // 隧道在500m以外，要么恢复之前要么匹配地图限速
    // 看进隧道前的巡航速度，如果当时调速了，
    target_speed_km = tunnel_data_.state_before_tunnel
                          ? std::max(tunnel_data_.mem_speed_km, origin_speed_km)
                          : origin_speed_km;
    tunnel_data_.adapted = false;
    need_adapt_ = true;
    use_memery_ = tunnel_data_.state_before_tunnel &&
                  tunnel_data_.mem_speed_km >= origin_speed_km;
    mem_state_ = tunnel_data_.state_before_tunnel;
  } else {
    // 出隧道的时候如果是连续隧道，用户没有调速，匹配下一个隧道的信息
    if (tunnel_data_.last_speed_km != tunnel_data_.speed_km) {
      tunnel_data_.adapted = true;
      target_speed_km = tunnel_data_.speed_km;
      need_adapt_ = fct_cruise_speed_km_ != tunnel_data_.speed_km;
    } else {
      tunnel_data_.ignore_into = true;
      need_adapt_ = false;
      target_speed_km = origin_speed_km;
    }
  }
  if (config_.enable_tunnel_debug()) {
    auto* tunnel_debug =
        fct_out->mutable_nnp_fct_out()->mutable_tunnel_spd_adapt_debug();
    tunnel_debug->set_tunnel_adapt(need_adapt_);
    tunnel_debug->set_curr_state(
        functionmanager::TunnelSpdAdaptDebug::OUT_TUNNEL);
  }
  return target_speed_km;
}

int SpdAdaptScenario::AdaptIntoTollhouse(const int origin_speed_km) {
  UNUSED(origin_speed_km);
  need_adapt_ = true;
  use_memery_ = false;
  return static_cast<int>(config_.tollhouse_adapt_speedkm());
}

int SpdAdaptScenario::AdaptIntoCamera(const int origin_speed_km) {
  UNUSED(origin_speed_km);
  auto adapt = false;
  if (fct_cruise_speed_km_ > camera_data_.speed_km) {
    adapt = true;
    use_memery_ = false;
  }
  camera_data_.adapted = true;
  camera_data_.mem_speed_km = static_cast<int>(fct_cruise_speed_km_);
  camera_data_.state_before_camera = usr_adjust_cruise_speed_;
  camera_data_.road_speed_before_camera = road_data_.cal_road_speed_km;
  camera_data_.ignore_out = !adapt;
  camera_data_.adapt_time = common::Clock::NowInMicroseconds();
  need_adapt_ = need_adapt_ || adapt;
  if (!adapt) {
    return origin_speed_km;
  }
  return camera_data_.speed_km;
}

int SpdAdaptScenario::AdaptCameraSpeedChanged(const int origin_speed_km) {
  auto adapt = false;
  if (fct_cruise_speed_km_ > camera_data_.speed_km) {
    adapt = true;
    use_memery_ = false;
  }
  camera_data_.adapted = true;
  camera_data_.road_speed_before_camera = road_data_.cal_road_speed_km;
  need_adapt_ = need_adapt_ || adapt;
  if (!adapt) {
    return origin_speed_km;
  }
  return camera_data_.speed_km;
}

int SpdAdaptScenario::AdaptOutCamera(const int origin_speed_km) {
  UNUSED(origin_speed_km);
  camera_data_.adapted = false;
  // 如果用户滚轮调速、调整设置偏移或者进摄像头的时候不需要匹配
  if (camera_data_.ignore_out || usr_adjust_cruise_speed_) {
    camera_data_.ignore_out = false;
    use_memery_ = false;
    camera_data_.adapt_time = 0.0;
    return static_cast<int>(origin_speed_km);
  }
  if (ramp_data_.near) {
    use_memery_ = false;
    need_adapt_ = fct_cruise_speed_km_ > config_.speed_display_for_ramp_dec();
    camera_data_.adapt_time = 0.0;
    return std::min(config_.speed_display_for_ramp_dec(),
                    static_cast<int>(fct_cruise_speed_km_));
  }
  // 出摄像头的时候如果在隧道里,记忆值大于当前的隧道值,说明摄像头先匹配，这个时候先匹配地图限速
  if ((tunnel_data_.adapted || road_data_.in_tunnel) &&
      camera_data_.mem_speed_km > tunnel_data_.speed_km &&
      camera_data_.adapt_time > tunnel_data_.adapt_time) {
    tunnel_data_.state_before_tunnel = camera_data_.state_before_camera;
    tunnel_data_.mem_speed_km = camera_data_.mem_speed_km;
    need_adapt_ = true;
    use_memery_ = false;
    camera_data_.adapt_time = 0.0;
    return tunnel_data_.speed_km;
  }
  // 出摄像头的时候看下是不是道路限速变了,看下匝道信息
  if (camera_data_.road_changed_during_camera) {
    camera_data_.road_changed_during_camera = false;
    use_memery_ = false;
    need_adapt_ = true;
    camera_data_.adapt_time = 0.0;
    return camera_data_.road_speed_during_camera;
  }
  need_adapt_ = true;
  use_memery_ = true;
  mem_state_ = camera_data_.state_before_camera;
  camera_data_.adapt_time = 0.0;
  if (camera_data_.mem_speed_km < config_.min_speed() ||
      camera_data_.mem_speed_km > config_.max_speed()) {
    need_adapt_ = false;
    use_memery_ = false;
    return origin_speed_km;
  }
  return camera_data_.mem_speed_km;
}
}  // namespace planning
}  // namespace TL
