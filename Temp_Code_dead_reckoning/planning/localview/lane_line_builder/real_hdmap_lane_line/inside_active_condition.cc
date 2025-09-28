/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "absl/strings/match.h"
#include "common/file/file.h"
#include "common/math/double_type.h"
#include "common/time/clock.h"

#include "planning/localview/lane_line_builder/real_hdmap_lane_line/inside_active_condition.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {
namespace {
const double kSegmentationEpsilon = 0.2;
const std::size_t kLanePointMinSize = 2U;
constexpr double kMinDownTime = 3.0;

void RemoveDuplicates(std::vector<Vec2d>* points) {
  RETURN_IF_NULL(points);

  size_t points_size = points->size();

  if (points_size <= kLanePointMinSize) {
    return;
  }

  int count = 0;
  // 3个条件：
  // 1.第1个点和最后一个点必要
  // 2.中间点距离>limit
  // 3.最后两个点如果距离太近只要最后一个点
  for (size_t i = 0; i < points_size; ++i) {
    if (count == 0 || i == points_size - 1) {
      (*points)[count++] = points->at(i);
    } else {
      if (points->at(i).DistanceTo(points->at(count - 1)) >
          kSegmentationEpsilon) {
        (*points)[count++] = points->at(i);
      }
    }
  }
  if (count > 2) {
    if (points->at(count - 2).DistanceTo(points->at(count - 1)) <=
        kSegmentationEpsilon) {
      points->at(count - 2) = points->at(count - 1);
      --count;
    }
  }
  points->resize(count);
}
}  // namespace

using TL::functionmanager::NNPSysState;

constexpr double kMaxHeadingErr = 0.18;

InsideActiveCondition::InsideActiveCondition(
    const std::shared_ptr<LocalViewData>& local_view_data)
    : local_view_data_(local_view_data) {
  if (!common::GetProtoFromFile(FLAGS_hmi_config_file, &hmi_config_)) {
    AERROR << "Failed to load obs follow time config file "
           << FLAGS_hmi_config_file;
  }
  const auto& curvature_velocity = hmi_config_.curvature_velocity_conf();
  const auto& curvature_r = curvature_velocity.curvature_r();
  const auto& centripetal_acceleration =
      curvature_velocity.centripetal_acceleration();
  if (centripetal_acceleration.size() != curvature_r.size()) {
    AERROR << "curvature velocity conf is error";
    return;
  }
  for (int i = 0; i < curvature_r.size(); i++) {
    hmi_curvature_velocity_.emplace_back(
        std::make_tuple(curvature_r.at(i), centripetal_acceleration.at(i)));
  }
}

void InsideActiveCondition::Process(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<hdmap::PncMap>& pnc_map,
    functionmanager::FunctionManagerOut* const to_fct, bool is_hd_map) {
  OddDeal(local_view, pnc_map, to_fct);
  auto* mutable_active_status =
      to_fct->mutable_nnp_fct_out()->mutable_local_map_active_status();
  if (is_hd_map) {
    mutable_active_status =
        to_fct->mutable_nnp_fct_out()->mutable_hd_map_active_status();
  }
  if (local_view == nullptr) {
    return;
  }
  const auto& adc_waypoint = pnc_map->GetADCWaypoint();
  to_fct->set_adc_is_in_tunnel(
      adc_waypoint.lane != nullptr &&
      adc_waypoint.lane->lane().map_lane_type().tunnel_lane());
  const auto& odd_info = to_fct->odd_info();
  // 1,高清地图和odd区域
  double to_ramp_len = to_fct->nnp_fct_out().nnp_d_distance2_downramp_sg();
  bool is_ok_odd_type =
      (odd_info.odd_type() == routing::LaneWaypoint::LAYER_ODD &&
       (odd_info.info() == "DownGrade" ||
        odd_info.info() == "lane is error")) ||
      odd_info.odd_type() != routing::LaneWaypoint::LAYER_ODD;

  bool is_odd_start =
      is_ok_odd_type &&
      (odd_info.type() == routing::LaneWaypointType::ODD_START ||
       odd_info.type() == routing::LaneWaypointType::ROUTE_BREAK);
  static constexpr double kToRampFailLen = 250.0;
  bool is_fail_to_ramp = is_odd_start && odd_info.to_end_len() > 1.0 &&
                         to_ramp_len > odd_info.to_end_len() &&
                         to_ramp_len < kToRampFailLen;
  bool is_check_odd =
      (odd_info.odd_type() != routing::LaneWaypoint::SPECIAL_AREA);
  bool is_odd_region =
      ((odd_info.type() == routing::LaneWaypointType::ODD_END &&
        is_ok_odd_type) ||
       (is_odd_start && odd_info.to_end_len() > 1.0 &&
        odd_info.to_end_len() < 10.0 && is_check_odd));
  is_odd_region = is_odd_region || is_fail_to_ramp;

  const auto& adc_remain_len = to_fct->adc_passage_remain_len();
  bool is_ok_ref_len = !is_check_odd || (is_check_odd && adc_remain_len > 90.0);
  mutable_active_status->set_vehicle_in_hdmap(!is_odd_region && is_ok_ref_len);
  // 2,定位检测
  const auto& location_err_state =
      to_fct->nnp_fct_out().nnp_statechange_conditions().location_err_state();
  bool location_state = is_hd_map ? (location_err_state == 0) : true;

  mutable_active_status->set_valid_of_lane_localization(location_state);

  // 3,导航routing
  bool is_out_of_routing = pnc_map->IsOutOfRouting();
  mutable_active_status->set_valid_of_lane_routing(!is_out_of_routing);

  // 4,检测车辆是否处于禁行区域或其他不可行驶区域
  auto adc_lane_type = pnc_map->GetAdcLaneType();
  to_fct->set_road_type(adc_lane_type);
  bool is_ok_location_state = true;
  if (local_view->HasLocalization() && is_hd_map) {
    const auto& loc_state = local_view->GetLocalization()->location_state();
    // 定位MM状态目前只用了2
    is_ok_location_state = (loc_state == 2);
  }

  bool is_ncp_other = false;
  if (to_fct->has_hdmap_sub_state()) {
    is_ncp_other = adc_lane_type == hdmap::RoadSection::Other;
  }
  auto active_condition =
      (pnc_map->AdcInMainRoad() || pnc_map->ADCInRamp() ||
       adc_lane_type == hdmap::RoadSection::SideRoad || is_ncp_other) &&
      is_ok_location_state && !to_fct->adc_is_in_tunnel();

  mutable_active_status->set_vehicle_not_in_forbidlane(
      not_in_forbidlane_debounce_.DealDebounce(active_condition));

  // 5,检测自车前方5s位置车道宽度
  static constexpr double kMinLaneWidth = 2.5;
  static constexpr double kMaxLaneWidth = 5.0;
  static constexpr double kMinSecond = 5;  // s
  double lane_width = 0.0;
  if (adc_waypoint.lane != nullptr && local_view->HasVehicleState()) {
    const auto& vehicle_state = local_view->GetVehicleState();
    double front_check_len = std::min(
        adc_waypoint.lane->total_length(),
        (adc_waypoint.s + vehicle_state->linear_velocity() * kMinSecond));
    lane_width = adc_waypoint.lane->GetWidth(front_check_len);
  }
  bool is_ok_lane_width =
      (lane_width > kMinLaneWidth && lane_width < kMaxLaneWidth);
  mutable_active_status->set_appropriate_current_lane_width(is_ok_lane_width);

  // 6,检测车辆航向角是否满足激活条件和检测车辆是否处于逆行车道
  auto vehicle_heading_and_reverse_lane =
      CheckVehicleHeadingAndReverseLane(local_view, adc_waypoint);
  mutable_active_status->set_appropriate_current_lane_headingerr(
      current_lane_headingerr_debounce_.DealDebounce(
          vehicle_heading_and_reverse_lane.first));
  mutable_active_status->set_vehicle_not_in_reverselane(
      vehicle_heading_and_reverse_lane.second);

  // 7,判断自车当前速度*5s的位置的曲率
  const auto curve_status = true;
  // CheckCurvature(local_view, adc_waypoint, to_fct);
  bool is_ok_speed_limit = true;
  if (!local_view_data_->update_data()->is_nnp_drive_auto() &&
      pnc_map->AdcInMainRoad()) {
    static constexpr double kMaxSpeedLimit = 36.1;
    const auto& vehicle_state = local_view->GetVehicleState();
    if (vehicle_state->linear_velocity() > kMaxSpeedLimit) {
      is_ok_speed_limit = false;
    }
  }
  mutable_active_status->set_appropriate_current_lane_curve(
      current_lane_curve_debounce_.DealDebounce(curve_status &&
                                                is_ok_speed_limit));

  // 8,检测轨迹是否压线上游给true，具体放在下游检测
  mutable_active_status->set_vehicle_not_in_otherforbidarea(true);
}

void InsideActiveCondition::MapFusionProcess(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<hdmap::PncMap>& pnc_map,
    const std::shared_ptr<hdmap::HDMap>& hd_map,
    functionmanager::FunctionManagerOut* to_fct, bool is_percep_map) {
  auto* mutable_active_status =
      to_fct->mutable_nnp_fct_out()->mutable_hd_map_active_status();
  OddDeal(local_view, pnc_map, to_fct);
  if (local_view == nullptr || pnc_map->GetADCWaypoint().lane == nullptr) {
    return;
  }
  const auto& adc_waypoint = pnc_map->GetADCWaypoint();
  to_fct->set_adc_is_in_tunnel(
      adc_waypoint.lane->lane().map_lane_type().tunnel_lane());
  double distance_outof_odd_sg =
      to_fct->nnp_fct_out().nnp_d_distance_outof_odd_sg();
  // 只在percep_map下检查
  local_view_data_->update_data()->set_is_cruise_check(
      !is_percep_map || CheckCruiseCurvature(local_view, hd_map, adc_waypoint));
  // 检查未激活前车辆和地图的夹角是否符合 < 0.18（10度）
  double vehicle_heading = local_view->GetVehicleState()->heading();
  double lane_heading = adc_waypoint.lane->Heading(adc_waypoint.s);
  bool position_attitude = true;
  double vehicle_heading_err =
      common::math::AngleDiff(vehicle_heading, lane_heading);
  if ((adc_waypoint.l > 0 && vehicle_heading_err < -0.05) ||
      (adc_waypoint.l < 0 && vehicle_heading_err > 0.05)) {
    position_attitude = false;
  }
  bool is_heading_err_fit = headingerr_fit_debounce_.DealDebounce(
      local_view->GetFunctionManagerIn()->ta_pilot_mode() ==
              functionmanager::TaPilotMode::NO_CONTROL
          ? ((std::fabs(vehicle_heading_err) < kMaxHeadingErr) &&
             position_attitude)
          : true);
  ADEBUG << " vehicle_heading: " << vehicle_heading
         << " , lane_heading: " << lane_heading
         << " ,is_heading_err_fit: " << is_heading_err_fit;
  // 1,高清地图和odd区域
  const auto& adc_remain_len = to_fct->adc_passage_remain_len();
  // bool is_ok_ref_len = !is_check_odd ||
  // (is_check_odd && adc_remain_len > 90.0);
  const double min_distance = std::fmax(
      local_view->GetVehicleState()->linear_velocity() * kMinDownTime, 5.0);
  double adc_to_dest_len = pnc_map->GetDistanceFromADCToDestination(true);
  mutable_active_status->set_vehicle_in_hdmap(
      adc_remain_len > 5.0 && adc_to_dest_len > min_distance &&
      local_view_data_->update_data()->is_cruise_check() && is_heading_err_fit);
  // 2,定位检测
  mutable_active_status->set_valid_of_lane_localization(true);

  // 3,导航routing
  bool is_out_of_routing = pnc_map->IsOutOfRouting();
  mutable_active_status->set_valid_of_lane_routing(!is_out_of_routing);

  // 4,检测车辆是否处于禁行区域或其他不可行驶区域
  auto adc_lane_type = pnc_map->GetAdcLaneType();
  to_fct->set_road_type(adc_lane_type);

  bool is_ncp_other = adc_lane_type == hdmap::RoadSection::Other;
  auto active_condition =
      (pnc_map->AdcInMainRoad() || pnc_map->ADCInRamp() ||
       adc_lane_type == hdmap::RoadSection::SideRoad || is_ncp_other);
  bool is_distance_outof_odd =
      (distance_outof_odd_sg < 0.1 || distance_outof_odd_sg > 500);
  mutable_active_status->set_vehicle_not_in_forbidlane(
      is_percep_map
          ? true
          : (not_in_forbidlane_debounce_.DealDebounce(active_condition) &&
             is_distance_outof_odd));
  // is_in_hdmap在地图中判断，下游mcu会使用作为“已进入NNP可开启区域，是否激活NNP？”提醒的条件
  if (!is_percep_map) {
    to_fct->mutable_nnp_fct_out()->set_is_in_hdmap(
        to_fct->nnp_fct_out().is_in_hdmap() && is_distance_outof_odd);
  }

  // 5,检测自车前方5s位置车道宽度
  static constexpr double kMinLaneWidth = 2.5;
  static constexpr double kMaxLaneWidth = 5.0;
  static constexpr double kMinSecond = 5;  // s
  double lane_width = 0.0;
  if (adc_waypoint.lane != nullptr && local_view->HasVehicleState()) {
    const auto& vehicle_state = local_view->GetVehicleState();
    double front_check_len = std::min(
        adc_waypoint.lane->total_length(),
        (adc_waypoint.s + vehicle_state->linear_velocity() * kMinSecond));
    lane_width = adc_waypoint.lane->GetWidth(front_check_len);
  }
  bool is_ok_lane_width =
      (lane_width > kMinLaneWidth && lane_width < kMaxLaneWidth);
  mutable_active_status->set_appropriate_current_lane_width(is_ok_lane_width);

  // 6,检测车辆航向角是否满足激活条件和检测车辆是否处于逆行车道
  mutable_active_status->set_appropriate_current_lane_headingerr(true);
  mutable_active_status->set_vehicle_not_in_reverselane(true);

  // 7,判断自车当前速度*5s的位置的曲率
  const auto curve_status = true;
  // CheckCurvature(local_view, adc_waypoint, to_fct);
  bool is_ok_speed_limit = true;
  if (pnc_map->AdcInMainRoad()) {
    static constexpr double kMaxSpeedLimit = 36.1;
    const auto& vehicle_state = local_view->GetVehicleState();
    if (vehicle_state->linear_velocity() > kMaxSpeedLimit) {
      is_ok_speed_limit = false;
    }
  }
  mutable_active_status->set_appropriate_current_lane_curve(
      current_lane_curve_debounce_.DealDebounce(curve_status &&
                                                is_ok_speed_limit));

  // 8,检测车辆是否压线
  // 定位点在车道中心线左边l为正，右边l为负
  double left_c0{0.0};
  double right_c0{0.0};
  double left_length{0.0};
  double right_length{0.0};
  if (local_view->HasLaneMarkers() &&
      local_view->GetLaneMarkers()->has_front_left_lane_marker() &&
      local_view->GetLaneMarkers()->has_front_right_lane_marker()) {
    left_c0 =
        local_view->GetLaneMarkers()->front_left_lane_marker().c0_position();
    right_c0 =
        local_view->GetLaneMarkers()->front_right_lane_marker().c0_position();
    left_length =
        local_view->GetLaneMarkers()->front_left_lane_marker().view_range();
    right_length =
        local_view->GetLaneMarkers()->front_right_lane_marker().view_range();
    ADEBUG << " , left_c0: " << left_c0 << " , right_c0: " << right_c0;
  }
  bool is_lane_virtual =
      (is_percep_map && local_view_data_ != nullptr)
          ? local_view_data_->update_data()->is_adc_lane_virtual()
          : false;
  double lane_line_width = left_c0 - right_c0;
  const bool is_in_lane =
      current_lane_c0_debounce_.DealDebounce(
          left_c0 > 1.0 && right_c0 < -1.0 && lane_line_width > 2.5 &&
          lane_line_width < 5 && left_length > 15 && right_length > 15) &&
      !is_lane_virtual;
  // is_pilot_lat_suspend_bl (bit5)
  auto soc_04_val = to_fct->soc_2_fct_tbd_u32_04();
  uint32_t is_pilot_lat_suspend = is_in_lane ? 0 : 0x20;
  to_fct->set_soc_2_fct_tbd_u32_04(soc_04_val | is_pilot_lat_suspend);
  mutable_active_status->set_vehicle_not_in_otherforbidarea(is_in_lane);
}

void InsideActiveCondition::OddDeal(  // NOLINT
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<hdmap::PncMap>& pnc_map,
    functionmanager::FunctionManagerOut* to_fct) const {
  if (pnc_map == nullptr || !local_view->HasVehicleState()) {
    AERROR << "pnc_map or VehicleState is nullptr";
    return;
  }
  auto is_need_odd = [&](const routing::LaneWaypoint::ODDType& odd_type,
                         const std::string& info) {
    return FLAGS_enable_odd_area_internal_to_perception &&
           (absl::StrContains(info, "DownVisual") ||
            (FLAGS_enable_layer_odd_area_reduce
                 ? odd_type == routing::LaneWaypoint::LAYER_ODD
                 : false));
  };
  const auto& odd_info = pnc_map->GetOddInfo();
  auto* nnp_fct_out = to_fct->mutable_nnp_fct_out();
  auto* nnp_active_condition =
      nnp_fct_out->mutable_nnp_statechange_conditions();
  static constexpr double kMinLenToOddStart = 100.0;
  static constexpr double kMinTimeStamp = 2.0;
  static double last_timestamp = 0.0;
  routing::LaneWaypointType type = odd_info.type;
  if (type == routing::LaneWaypointType::ODD_END) {
    last_timestamp = common::Clock::NowInSeconds();
  }
  if (common::Clock::NowInSeconds() - last_timestamp < kMinTimeStamp) {
    type = routing::LaneWaypointType::ODD_END;
  }
  auto* odd_info_debug = to_fct->mutable_odd_info();
  double to_end_len = odd_info.to_end_len;
  if (odd_info.next_type == routing::LaneWaypointType::ODD_START &&
      (odd_info.next_odd_type == routing::LaneWaypoint::SPECIAL_AREA ||
       odd_info.next_odd_type == routing::LaneWaypoint::ROAD_END)) {
    type = routing::LaneWaypointType::ODD_START;
    to_end_len = std::min(odd_info.to_next_len, to_end_len);
  }
  const auto& remain_len = pnc_map->GetAdcPassageRemainLen();
  to_fct->set_adc_passage_remain_len(remain_len);
  static constexpr double kBuff = 5.0;
  if (odd_info.next_odd_type == routing::LaneWaypoint::ROAD_END &&
      remain_len > (odd_info.to_end_len + kBuff)) {
    type = routing::LaneWaypointType::NORMAL;
  }
  odd_info_debug->set_type(type);
  odd_info_debug->set_next_type(odd_info.next_type);
  odd_info_debug->set_odd_type(odd_info.odd_type);
  odd_info_debug->set_next_odd_type(odd_info.next_odd_type);
  odd_info_debug->set_to_end_len(to_end_len);
  odd_info_debug->set_to_next_len(odd_info.to_next_len);
  odd_info_debug->set_info(odd_info.info);
  odd_info_debug->set_next_info(odd_info.next_info);

  // 高清地图和odd区域
  bool is_odd_region =
      ((odd_info.next_type == routing::LaneWaypointType::ODD_END ||
        (odd_info.next_type == routing::LaneWaypointType::ODD_START &&
         odd_info.to_next_len < kMinLenToOddStart)) &&
       absl::StrContains(odd_info.next_info, "DownVisual"));
  ADEBUG << "is_odd_region: " << is_odd_region;
  // 当前是否为ODD区域
  nnp_active_condition->set_is_odd_region(!is_odd_region);

  // 当前ODD类型是否需要内部降级
  const auto& adc_lane_type = pnc_map->GetAdcLaneType();
  bool by_odd = (odd_info.next_type == routing::LaneWaypointType::ODD_END ||
                 (odd_info.next_type == routing::LaneWaypointType::ODD_START &&
                  odd_info.to_next_len < kMinLenToOddStart &&
                  odd_info.to_next_len > 0.1)) &&
                is_need_odd(odd_info.next_odd_type, odd_info.next_info);
  bool by_lane_type =
      (adc_lane_type == hdmap::RoadSection::MultipleCarriageWay ||
       adc_lane_type == hdmap::RoadSection::SingleCarriageWay);
  nnp_active_condition->set_is_change_mode_by_odd_type(by_odd && by_lane_type);

  // 导航routing
  bool is_out_of_routing = pnc_map->IsOutOfRouting();
  nnp_active_condition->set_valid_of_lane_routing(!is_out_of_routing);

  // update adc to odd distance
  double adc_to_dest_len = pnc_map->GetDistanceFromADCToDestination(true);
  static constexpr double kEpsilon = 1e-6;
  static constexpr double kMinRoutingLen = 800;
  const auto distance = static_cast<float>(
      std::isinf(odd_info.to_end_len) ? kEpsilon : odd_info.to_end_len);
  if (type == routing::LaneWaypointType::NORMAL) {
    nnp_fct_out->set_nnp_d_distance_outof_odd_sg(kEpsilon);
    nnp_fct_out->set_nnp_d_distance_into_odd_sg(kEpsilon);
#ifndef FOR_BAIDU_SIMULATION
    const auto& fsm_state = to_fct->fsm_state();
    const auto& local_map_type = to_fct->localization_maptype();
    bool is_nnp_or_ncp_map =
        (fsm_state == functionmanager::MachineStateType::HDMAP_TYPE &&
         (local_map_type == navigation_hdmap::MapMsg::FUSION_NNP_MAP ||
          local_map_type == navigation_hdmap::MapMsg::FUSION_NCP_MAP));
    if (is_nnp_or_ncp_map && adc_to_dest_len < kMinRoutingLen &&
        adc_to_dest_len > 1.0) {
      odd_info_debug->set_odd_type(routing::LaneWaypoint::ROAD_END);
      odd_info_debug->set_type(routing::LaneWaypointType::ODD_START);
      odd_info_debug->set_to_next_len(adc_to_dest_len);
      odd_info_debug->set_to_end_len(adc_to_dest_len);
      nnp_fct_out->set_nnp_d_distance_outof_odd_sg(
          static_cast<float>(adc_to_dest_len));
    }
#endif
  } else if (type == routing::LaneWaypointType::ODD_END) {
    nnp_fct_out->set_nnp_d_distance_outof_odd_sg(kEpsilon);
    nnp_fct_out->set_nnp_d_distance_into_odd_sg(kEpsilon);
  } else if (type == routing::LaneWaypointType::ODD_START) {
    const double min_distance = std::fmax(
        local_view->GetVehicleState()->linear_velocity() * kMinDownTime, 5.0);
    const auto dis = ((distance - min_distance) < kEpsilon ||
                      is_need_odd(odd_info.next_odd_type, odd_info.info))
                         ? kEpsilon
                         : distance - min_distance;
    nnp_fct_out->set_nnp_d_distance_outof_odd_sg(static_cast<float>(dis));
    nnp_fct_out->set_nnp_d_distance_into_odd_sg(kEpsilon);
  } else if (type == routing::LaneWaypointType::ROUTE_BREAK) {
    nnp_fct_out->set_nnp_d_distance_outof_odd_sg(distance);
    nnp_fct_out->set_nnp_d_distance_into_odd_sg(kEpsilon);
  }
}

std::pair<bool, bool>
InsideActiveCondition::CheckVehicleHeadingAndReverseLane(  // NOLINT
    const std::shared_ptr<LocalView>& local_view,
    const hdmap::LaneWaypoint& adc_waypoint) {
  auto active_condition = std::make_pair(false, false);
  if (local_view == nullptr || adc_waypoint.lane == nullptr ||
      !local_view->HasVehicleState()) {
    return active_condition;
  }
  const auto& vehicle_state = local_view->GetVehicleState();
  // 车辆与车道的角度差 和 车辆是否处于逆向车道
  const auto angle_diff = common::math::AngleDiff(
      vehicle_state->heading(), adc_waypoint.lane->Heading(adc_waypoint.s));
  static constexpr double kMaxAngleLaneVehicle = M_PI / 18;
  static constexpr double kMaxAngleReverseLane = M_PI / 2;
  active_condition.first = (std::fabs(angle_diff) < kMaxAngleLaneVehicle);
  active_condition.second = (std::fabs(angle_diff) < kMaxAngleReverseLane);
  return active_condition;
}

bool InsideActiveCondition::CheckCurvatureAndVelocity(
    const std::shared_ptr<LocalView>& local_view,
    const std::pair<double, double>& curvature_velocity,
    functionmanager::FunctionManagerOut* to_fct) {
  double curvature = std::fabs(curvature_velocity.first);
  double vehicle_speed = curvature_velocity.second;
  const auto& curvature_v = hmi_config_.curvature_velocity_conf();
  bool is_active = false;
  if (local_view->HasFunctionManagerIn()) {
    const auto& nnp_sys_state =
        local_view->GetFunctionManagerIn()->fct_nnp_in().nnp_sysstate();
    is_active = (nnp_sys_state == NNPSysState::NNPS_ACTIVE ||
                 nnp_sys_state == NNPSysState::NNPS_LAT_OVERRIDE);
  }
  double over_speed_ratio = is_active ? curvature_v.passive_over_speed_ratio()
                                      : curvature_v.active_over_speed_ratio();
  static constexpr double kEpsilon = 1e-6;
  if (common::math::double_type::DefinitelyGreater(curvature, kEpsilon)) {
    double curvature_R = 1.0 / curvature;
    to_fct->mutable_cur_limit_speed_info()->set_curvature_r(curvature_R);
    const auto size = hmi_curvature_velocity_.size();
    for (size_t i = 0; i < size; i++) {
      double max_curv = 0.0;
      max_curv = std::numeric_limits<double>::infinity();
      if (i < size - 1) {
        max_curv = std::get<0>(hmi_curvature_velocity_.at(i + 1));
      }
      double min_curv = 0.0;
      min_curv = std::get<0>(hmi_curvature_velocity_.at(i));
      double speed_limit =
          std::sqrt(std::get<1>(hmi_curvature_velocity_.at(i)) * curvature_R) *
          over_speed_ratio;
      to_fct->mutable_cur_limit_speed_info()->set_limit_speed(speed_limit);
      if (curvature_R > min_curv && curvature_R <= max_curv &&
          vehicle_speed <= speed_limit) {
        return true;
      }
    }
  } else {
    return true;
  }
  return false;
}

bool InsideActiveCondition::CheckCurvature(
    const std::shared_ptr<LocalView>& local_view,
    const hdmap::LaneWaypoint& adc_waypoint,
    functionmanager::FunctionManagerOut* to_fct) {
  double curvature = std::numeric_limits<double>::max();
  if (local_view == nullptr || adc_waypoint.lane == nullptr ||
      !local_view->HasVehicleState()) {
    return false;
  }
  const auto& vehicle_state = local_view->GetVehicleState();
  static constexpr double kEpsilon = 1e-6;
  static constexpr double kMinSecond = 5;  // s
  double max_cur = kEpsilon;
  double front_check_len = std::min(
      adc_waypoint.lane->total_length(),
      (adc_waypoint.s + vehicle_state->linear_velocity() * kMinSecond));
  double adc_s = adc_waypoint.s;
  max_cur = adc_waypoint.lane->Curvature(adc_s);
  while (adc_s < front_check_len) {
    curvature = adc_waypoint.lane->Curvature(adc_s);
    if (std::fabs(curvature) > std::fabs(max_cur)) {
      max_cur = curvature;
    }
    adc_s += 5.0;
  }
  auto curvature_velocity =
      std::make_pair(std::fabs(max_cur), vehicle_state->linear_velocity());
  return CheckCurvatureAndVelocity(local_view, curvature_velocity, to_fct);
}

bool InsideActiveCondition::CheckCruiseCurvature(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<hdmap::HDMap>& hd_map,
    const hdmap::LaneWaypoint& adc_waypoint) {
  // double average_curvature = std::numeric_limits<double>::max();
  if (local_view == nullptr || adc_waypoint.lane == nullptr ||
      !local_view->HasVehicleState() || hd_map == nullptr) {
    return true;
  }
  const auto driving_mode = local_view->GetChassis()->driving_mode();
  const bool is_suspend = driving_mode == soc::Chassis::AUTO_SPEED_ONLY;
  TL::common::Pose pose;
  if (local_view->HasLocalization() &&
      local_view->GetLocalization()->has_pose()) {
    pose = local_view->GetLocalization()->pose();
  } else {
    pose.set_heading(0.0);
    pose.mutable_position()->set_x(0.0);
    pose.mutable_position()->set_y(0.0);
  }
  double start_s = adc_waypoint.s;
  auto adc_lane = adc_waypoint.lane;
  double lane_length = adc_lane->total_length() - start_s;
  double view_range = 40.0;
  std::vector<Vec2d> map_central_points;
  AddPoints(&map_central_points, adc_lane, pose);
  while (lane_length < view_range) {
    if (adc_lane->lane().successor_id().empty() ||
        adc_lane->lane().successor_id().size() > 1) {
      break;
    }
    auto lane_id = adc_lane->lane().successor_id().at(0);
    ADEBUG << " success lane id: " << lane_id.id();
    adc_lane = hd_map->GetLaneById(lane_id);
    if (adc_lane == nullptr) {
      break;
    }
    AddPoints(&map_central_points, adc_lane, pose);
    lane_length += adc_lane->total_length();
    ADEBUG << "add lane length:" << adc_lane->total_length()
           << " , total lane length: " << lane_length;
  }
  if (map_central_points.size() < 3) {
    return true;
  }
  constexpr int N = 3;
  std::vector<double> coff = common::math::FitPolynomial<N>(map_central_points);
  auto fusion_kappa = std::fabs(local_view->GetSubjectKappa());
  double original_steering_percentage =
      local_view->GetChassis()->has_steering_percentage()
          ? local_view->GetChassis()->steering_percentage()
          : 0.0;
  double steer_angle =
      std::fabs(original_steering_percentage / 100 * (8.0345 / M_PI * 180));
  constexpr double kMaxCure = 1.0 / 300.0;
  constexpr double kMinCure = 1.0 / 1000.0;
  constexpr double kSteeringAngel = 20.0;
  const double map_curvature = std::fabs(coff[2]) * 2;
  // ACC下，地图直行，但是驾驶员打方向盘转弯需要切换到巡航地图
  const bool is_driver_diff = map_curvature < kMinCure &&
                              fusion_kappa > kMaxCure &&
                              steer_angle > kSteeringAngel;
  double heading_err = std::fabs(
      common::math::AngleDiff(local_view->GetVehicleState()->heading(),
                              adc_waypoint.lane->Heading(adc_waypoint.s)));
  // ACC下，地图转弯，但是驾驶员打方向盘保持直行也需要切换到巡航地图
  const bool is_map_diff = fusion_kappa < kMinCure &&
                           map_curvature > kMaxCure &&
                           steer_angle < kSteeringAngel / 2;
  // ACC下，车速小于40km/h,车辆压线超过0.6m，持续300ms以上，需要切到巡航地图
  constexpr double kMaxLimitSpeed = 40.0 / 3.6;
  constexpr double kMinLimitC0 = 0.4;
  double left_c0{1.8};
  double right_c0{-1.8};
  if (local_view->HasLaneMarkers() &&
      local_view->GetLaneMarkers()->has_front_left_lane_marker()) {
    left_c0 =
        local_view->GetLaneMarkers()->front_left_lane_marker().c0_position();
  }
  if (local_view->HasLaneMarkers() &&
      local_view->GetLaneMarkers()->has_front_right_lane_marker()) {
    right_c0 =
        local_view->GetLaneMarkers()->front_right_lane_marker().c0_position();
  }
  ADEBUG << " , left_c0: " << left_c0 << " , right_c0: " << right_c0;
  const bool is_not_in_lane = cruise_lane_c0_debounce_.DealDebounce(
      (std::fabs(left_c0) < kMinLimitC0 || std::fabs(right_c0) < kMinLimitC0) &&
      local_view->GetVehicleState()->linear_velocity() < kMaxLimitSpeed);
  // ACC下，左右车道有一个是虚拟的，需要切到巡航地图
  auto now_lane = adc_waypoint.lane->lane();
  const bool is_adc_has_virtual = now_lane.left_boundary().virtual_() ||
                                  now_lane.right_boundary().virtual_();
  const bool is_check_bl =
      (is_driver_diff || is_map_diff || is_not_in_lane || is_adc_has_virtual) &&
      is_suspend;
  ADEBUG << " map_kappa: " << coff[2] * 2 << " , fusion_kappa: " << fusion_kappa
         << " ,steer_angle: " << steer_angle
         << " , heading_err: " << heading_err
         << " ,is_map_diff: " << is_map_diff
         << " , is_driver_diff: " << is_driver_diff
         << " , is_check_bl: " << is_check_bl
         << " , is_not_in_lane: " << is_not_in_lane;
  return !is_check_bl;
}

void InsideActiveCondition::AddPoints(
    std::vector<Vec2d>* map_points, const TL::hdmap::LaneInfoConstPtr& lane,
    const common::Pose& pose) {
  RETURN_IF_NULL(lane);
  TL::common::PointENU prev_p;
  prev_p.set_x(std::numeric_limits<double>::infinity());
  prev_p.set_y(std::numeric_limits<double>::infinity());
  for (const auto& seg : lane->lane().central_curve().segment()) {
    if (!seg.has_line_segment()) {
      continue;
    }
    for (const auto& p : seg.line_segment().point()) {
      if (!std::isinf(prev_p.x()) && !std::isinf(prev_p.y()) &&
          common::math::double_type::SeemsEqual(prev_p.x(), p.x()) &&
          common::math::double_type::SeemsEqual(prev_p.y(), p.y())) {
        continue;
      }
      map_points->emplace_back(PointEarth2Bus({p.x(), p.y()}, pose));
      prev_p.set_x(p.x());
      prev_p.set_y(p.y());
    }
  }
  RemoveDuplicates(map_points);
}

Vec2d InsideActiveCondition::PointEarth2Bus(const Vec2d& point,  // NOLINT
                                            const common::Pose& pose) {
  Vec2d bus_point;
  double x = point.x() - pose.position().x();
  double y = point.y() - pose.position().y();
  bus_point.set_x(x * std::cos(pose.heading()) + y * std::sin(pose.heading()));
  bus_point.set_y(-x * std::sin(pose.heading()) + y * std::cos(pose.heading()));
  return bus_point;
}
}  // namespace planning
}  // namespace TL
