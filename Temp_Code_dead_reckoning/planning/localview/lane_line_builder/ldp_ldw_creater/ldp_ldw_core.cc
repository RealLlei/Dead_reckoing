/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/ldp_ldw_creater/ldp_ldw_core.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "common/configs/config_gflags.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/util/util.h"
#include "map/hdmap/path.h"

namespace TL {
namespace planning {
using TL::common::math::Vec2d;      // NOLINT
using TL::common::util::operator+;  // NOLINT

LdpLdwCore::LdpLdwCore() {}  // NOLINT

Status LdpLdwCore::Init() {  // NOLINT
  return Status::OK();
}

Status LdpLdwCore::Init(const PerceptionMapConfig& config) {
  navi_hdmap_config_ = config;
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  ldw_creater_ = std::make_unique<LdwCreater>(config);
  ldp_creater_ = std::make_unique<LdpCreater>(config);
  lane_line_marker_detection_ =
      std::make_unique<LaneLineMarkerDetection>(config);
  ldw_creater_->Init();
  ldp_creater_->Init();
  lane_line_marker_detection_->Init();
  return Status::OK();
}

Status LdpLdwCore::Start() {
  ldw_creater_->Start();
  ldp_creater_->Start();
  lane_line_marker_detection_->Start();
  return Status::OK();
}

void LdpLdwCore::Stop() {
  ldw_creater_->Stop();
  ldp_creater_->Stop();
  lane_line_marker_detection_->Stop();
}

void LdpLdwCore::PreLanemarkers(
    const ::TL::perception::LaneMarkers& lane_markers, const double v_spd,
    const double yaw_rate) {
  const double pre_time = navi_hdmap_config_.lanemarkers_pre_time();
  ldp_ldw_data_.left_line_markers_quality =
      lane_markers.front_left_lane_marker().quality();
  ldp_ldw_data_.right_line_markers_quality =
      lane_markers.front_right_lane_marker().quality();
  if (lane_markers.has_front_left_lane_marker() &&
      lane_markers.front_left_lane_marker().view_range() >
          lanelineprocess::kMinLaneDis) {
    ADEBUG << "pre left lanemarker!";
    LeftPreLanemarker(lane_markers.front_left_lane_marker(), v_spd, yaw_rate,
                      pre_time);
  } else {
    ldp_ldw_data_.left_line_markers_c0 =
        lane_markers.front_right_lane_marker().c0_position();
    ldp_ldw_data_.left_line_markers_c1 =
        lane_markers.front_right_lane_marker().c1_heading_angle();
    ldp_ldw_data_.left_line_markers_c2 =
        lane_markers.front_right_lane_marker().c2_curvature();
    ldp_ldw_data_.left_line_markers_c3 =
        lane_markers.front_right_lane_marker().c3_curvature_derivative();
  }
  if (lane_markers.has_front_right_lane_marker() &&
      lane_markers.front_right_lane_marker().view_range() >
          lanelineprocess::kMinLaneDis) {
    ADEBUG << "pre right lanemarker!";
    RightPreLanemarker(lane_markers.front_right_lane_marker(), v_spd, yaw_rate,
                       pre_time);
  } else {
    ldp_ldw_data_.right_line_markers_c0 =
        lane_markers.front_left_lane_marker().c0_position();
    ldp_ldw_data_.right_line_markers_c1 =
        lane_markers.front_left_lane_marker().c1_heading_angle();
    ldp_ldw_data_.right_line_markers_c2 =
        lane_markers.front_left_lane_marker().c2_curvature();
    ldp_ldw_data_.right_line_markers_c3 =
        lane_markers.front_left_lane_marker().c3_curvature_derivative();
  }
  ldp_ldw_data_.left_line_markers_offset_last = LineMarkerEUation(
      vehicle_param_.wheel_base(), ldp_ldw_data_.left_line_markers_c0,
      ldp_ldw_data_.left_line_markers_c1, ldp_ldw_data_.left_line_markers_c2,
      ldp_ldw_data_.left_line_markers_c3);
  ldp_ldw_data_.right_line_markers_offset_last = LineMarkerEUation(
      vehicle_param_.wheel_base(), ldp_ldw_data_.right_line_markers_c0,
      ldp_ldw_data_.right_line_markers_c1, ldp_ldw_data_.right_line_markers_c2,
      ldp_ldw_data_.right_line_markers_c3);
  ldp_ldw_data_.left_tire_distance_2_line =
      GetLeftTireDistance2Line(ldp_ldw_data_.left_line_markers_offset_last);
  ldp_ldw_data_.right_tire_distance_2_line =
      GetRightTireDistance2Line(ldp_ldw_data_.right_line_markers_offset_last);
}

void LdpLdwCore::LeftPreLanemarker(
    const ::TL::perception::LaneMarker& ori_lane_marker, const double speed,
    const double yaw_rate, const double pre_time) {
  const double ori_c0 = ori_lane_marker.c0_position();
  const double ori_c1 = std::atan(ori_lane_marker.c1_heading_angle());
  const double ori_c2 = ori_lane_marker.c2_curvature() * 2;
  const double ori_c3 = ori_lane_marker.c3_curvature_derivative() * 6;
  const double dx = speed * pre_time;
  double c0 = ori_c0 + ori_c1 * dx +
              lanelineprocess::kHalfNum * ori_c2 * dx * dx +
              ori_c3 * dx * dx * dx / lanelineprocess::kMagicNumber6 -
              lanelineprocess::kHalfNum * pre_time * dx * yaw_rate;
  double c1 = std::tan(std::tan(ori_c1) + ori_c2 * dx +
                       lanelineprocess::kHalfNum * ori_c3 * dx * dx -
                       lanelineprocess::kHalfNum * pre_time * yaw_rate);
  ldp_ldw_data_.left_line_markers_c0 = c0;
  ldp_ldw_data_.left_line_markers_c1 = c1;
  ldp_ldw_data_.left_line_markers_c2 =
      lanelineprocess::kHalfNum * (ori_c2 + dx * ori_c3);
  ldp_ldw_data_.left_line_markers_c3 =
      ori_lane_marker.c3_curvature_derivative();
}

void LdpLdwCore::RightPreLanemarker(
    const ::TL::perception::LaneMarker& ori_lane_marker, const double speed,
    const double yaw_rate, const double pre_time) {
  const double ori_c0 = ori_lane_marker.c0_position();
  const double ori_c1 = std::atan(ori_lane_marker.c1_heading_angle());
  const double ori_c2 = ori_lane_marker.c2_curvature() * 2;
  const double ori_c3 = ori_lane_marker.c3_curvature_derivative() * 6;
  const double dx = speed * pre_time;
  double c0 = ori_c0 + ori_c1 * dx +
              lanelineprocess::kHalfNum * ori_c2 * dx * dx +
              ori_c3 * dx * dx * dx / lanelineprocess::kMagicNumber6 -
              lanelineprocess::kHalfNum * pre_time * dx * yaw_rate;
  double c1 = std::tan(std::tan(ori_c1) + ori_c2 * dx +
                       lanelineprocess::kHalfNum * ori_c3 * dx * dx -
                       lanelineprocess::kHalfNum * pre_time * yaw_rate);
  ldp_ldw_data_.right_line_markers_c0 = c0;
  ldp_ldw_data_.right_line_markers_c1 = c1;
  ldp_ldw_data_.right_line_markers_c2 =
      lanelineprocess::kHalfNum * (ori_c2 + dx * ori_c3);
  ldp_ldw_data_.right_line_markers_c3 =
      ori_lane_marker.c3_curvature_derivative();
}

bool LdpLdwCore::Process(const std::shared_ptr<LocalView>& local_view,
                         functionmanager::FunctionManagerOut* const to_fct) {
  // ProcessLaneMarkers(local_view);
  PreLanemarkers(*local_view->GetLaneMarkers(),
                 local_view->GetVehicleState()->linear_velocity(),
                 local_view->GetVehicleState()->angular_velocity());
  SetMg(local_view);
  uint16_t odd_indx_lanemarker_cond =
      lane_line_marker_detection_->ODDLaneLineProcess(local_view);
  ldp_ldw_data_.odd_indx_line_cond = odd_indx_lanemarker_cond;
  ldw_creater_->LdwDeal(to_fct, &ldp_ldw_data_, local_view);
  ldp_creater_->LdpDeal(to_fct, &ldp_ldw_data_, local_view, lanemarkerdebug_);
  LdwLdpDebugInfo(local_view);
  ADEBUG << "LdpLdwCore End!!!";
  return true;
}

void LdpLdwCore::LdwLdpDebugInfo(const std::shared_ptr<LocalView>& local_view) {
  ldw_creater_->LdwMessageInfo(local_view, lanemarkerdebug_);
  // ldp_creater_->LdpMessageInfo(local_view);
}

// void LdpLdwCore::ProcessLaneMarkers(
//     const std::shared_ptr<LocalView>& local_view) {
//   lane_line_marker_detection_->SetLaneMg(local_view);
// }

void LdpLdwCore::SetMg(const std::shared_ptr<LocalView>& local_view) {
  ldw_creater_->SetDealMg(local_view);
  ldp_creater_->SetDealMg(local_view);
}

double LdpLdwCore::GetLeftTireDistance2Line(double line_markers_offset_last) {
  return line_markers_offset_last - 0.5 * vehicle_param_.width();
}

double LdpLdwCore::GetRightTireDistance2Line(double line_markers_offset_last) {
  return line_markers_offset_last + 0.5 * vehicle_param_.width();
}

double LdpLdwCore::LineMarkerEUation(double x, double c0, double c1,  // NOLINT
                                     double c2,                       // NOLINT
                                     double c3) {
  return (c0 + c1 * x + c2 * x * x + c3 * x * x * x) * std::cos(std::atan(c1));
}

}  // namespace planning
}  // namespace TL
