/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "common/file/file.h"
#include "planning/tasks/deciders/speed_bounds_decider/over_speed_decider.h"

namespace TL {
namespace planning {

void OverSpeedDecider::Init(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  CaheADCLaneInfo(frame, reference_line_info);
}

void OverSpeedDecider::CaheADCLaneInfo(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  adc_is_tunnel_lane_ = false;
  usr_adjust_cruise_speed_ = false;
  adc_is_main_road_ = true;
  if (frame == nullptr || !frame->local_view().HasFunctionManagerIn() ||
      !frame->local_view().GetFunctionManagerIn()->has_fct_nnp_in() ||
      !frame->local_view()
           .GetFunctionManagerIn()
           ->fct_nnp_in()
           .has_longitud_ctrl_setspeed() ||
      reference_line_info == nullptr ||
      !reference_line_info->path_data().frenet_frame_path().is_forward_path()) {
    return;
  }
  const auto& nnp_fct_in =
      frame->local_view().GetFunctionManagerIn()->fct_nnp_in();
  const auto* reference_line_provider = frame->GetReferenceLineProvider();
  if (reference_line_provider == nullptr) {
    return;
  }
  const auto& pnc_map = reference_line_provider->GetPncMap();
  if (pnc_map == nullptr) {
    return;
  }
  const auto adc_lane_type = pnc_map->GetAdcLaneType();
  const auto adc_waypoint = pnc_map->GetADCWaypoint();
  if (adc_waypoint.lane == nullptr) {
    return;
  }
  adc_is_tunnel_lane_ =
      adc_waypoint.lane->lane().has_map_lane_type() &&
      adc_waypoint.lane->lane().map_lane_type().has_tunnel_lane();

  usr_adjust_cruise_speed_ = nnp_fct_in.has_usr_has_changed_cruise_spd() &&
                             nnp_fct_in.usr_has_changed_cruise_spd();
  adc_is_main_road_ =
      adc_lane_type == TL::hdmap::RoadSection::MultipleCarriageWay ||
      adc_lane_type == TL::hdmap::RoadSection::SingleCarriageWay ||
      adc_lane_type == TL::hdmap::RoadSection::UNKNOWN;
}

bool OverSpeedDecider::CheckAllowOverSpeed(bool const target_is_on_main_road,
                                           bool const target_is_tunnel_lane) {
#ifdef FOR_BAIDU_SIMULATION
  return false;
#endif
  const auto allow_over_speed =
      ((last_is_on_main_road_ == adc_is_main_road_) &&
       usr_adjust_cruise_speed_ &&
       (adc_is_main_road_ == target_is_on_main_road) &&
       (adc_is_tunnel_lane_ == target_is_tunnel_lane));
  last_is_on_main_road_ = adc_is_main_road_;
  return allow_over_speed;
}
}  // namespace planning
}  // namespace TL
