/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/ldp_ldw_creater/ldp_creater/ldp_creater.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "common/interpolation/interpolation_1d.h"
#include "common/status/status.h"
#include "planning/localview/local_view.h"

namespace TL {
namespace planning {
namespace {
constexpr double kVxThresholdVaule = 0.1;  // 配置文件
constexpr double kInWarningZoneHysteresisLow = 0.1;
constexpr double kInWarningZoneHysteresisStandard = 0.1;
constexpr double kInWarningZoneHysteresisHigh = 0.1;
constexpr double kOutWarningZoneHysteresisLow = -0.1;
constexpr double kOutWarningZoneHysteresisStandard = -0.1;
constexpr double kOutWarningZoneHysteresisHigh = -0.1;
constexpr double kWarningVyTtlcLow = 0.25;
constexpr double kWarningVyTtlcStandard = 0.3;
constexpr double kWarningVyTtlcHigh = 0.35;
constexpr double kWarningAyTtlcLow = 0.8;
constexpr double kWarningAyTtlcStandard = 0.6;
constexpr double kWarningAyTtlcHigh = 0.4;
constexpr double kLookaheadCurvatureTimeLow = 0.25;
constexpr double kLookaheadCurvatureTimeStandard = 0.3;
constexpr double kLookaheadCurvatureTimeHigh = 0.35;
constexpr double kLdpVehiclePosZoneTime = 1.5;  // ldp居中位置时间阈值
// constexpr double KLdpVehiclePosHeaderZoneTime =
//     1;  // ldp居中位置&&header时间阈值
// constexpr double KLdpVehicleHeaderZoneTime = 1.5;  // ldp居中header时间阈值
constexpr double kLdpPrevDonePosThd = 0.3;
}  // namespace

using TL::functionmanager::NNPSysState;
using TL::functionmanager::FctToNnpInput;
LdpCreater::LdpCreater(const PerceptionMapConfig& config)
    : navi_hdmap_config_(config) {}

Status LdpCreater::Init() {
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  if (fLB::FLAGS_using_ldp_stvl_high) {
    ldwsensitivity_ = StvlHigh;
  }
  InnerZoneTableCreater();
  ldp_lowpass_filter_.set_fs_and_fc(10, 0.5);
  in_zone_position_table_low_.Init(xy1_);
  in_zone_position_table_standard_.Init(xy2_);
  in_zone_position_table_high_.Init(xy3_);
  ldp_curve_ref_point_dist_.Init(ldp_curve_ref_point_dist_xy_);
  ldp_heading_threshold_.Init(ldp_heading_threshold_xy_);
  ldp_latacc_threshold_.Init(ldp_latacc_threshold_xy_);
  ldp_road_classification_debounce_.ResetTime(0.5, 0.5, 0.1);
  return Status::OK();
}

Status LdpCreater::Start() {
  return Status::OK();
}

void LdpCreater::Stop() {}

void LdpCreater::SetDealMg(const std::shared_ptr<LocalView>& local_view) {
  vehicle_state_ = local_view->GetVehicleState();
  perception_obstacles_ = local_view->GetPerceptionObstacles();
}

//  插值表创建
void LdpCreater::InnerZoneTableCreater() {
  for (int i = 0; i < 6; i++) {
    xy1_.emplace_back(std::make_pair(inner_zone_table_arr_x_.at(i),
                                    inner_zone_table_arr_y1_.at(i)));
    xy2_.emplace_back(std::make_pair(inner_zone_table_arr_x_.at(i),
                                    inner_zone_table_arr_y2_.at(i)));
    xy3_.emplace_back(std::make_pair(inner_zone_table_arr_x_.at(i),
                                    inner_zone_table_arr_y3_.at(i)));
  }
  for (int i = 0; i < 8; i++) {
    ldp_heading_threshold_xy_.emplace_back(
        std::make_pair(vehicle_speed_x.at(i), ldp_heading_threshold_y_.at(i)));
    ldp_curve_ref_point_dist_xy_.emplace_back(
        std::make_pair(vehicle_speed_x.at(i), ldp_curve_ref_point_y_.at(i)));
    ldp_latacc_threshold_xy_.emplace_back(
        std::make_pair(vehicle_speed_x.at(i), ldp_latacc_threshold_y_.at(i)));
  }
}

bool LdpCreater::IsGoodPath(double line_quality, double quality_vaule) {
  return line_quality > quality_vaule;
}

// 限幅函数
double LdpCreater::SaturationDynamicLimit(double x, double low, double high) {
  if (x > high) {
    return high;
  }
  if (x < low) {
    return low;
  }
  return x;
}

// 触发区域限幅
double LdpCreater::WarningZoneLimit(double zone, double slow_low,
                                    double slow_high, double sstandard_low,
                                    double sstandard_high, double shigh_low,
                                    double shigh_high) {
  double zoneresult = 0;
  switch (ldwsensitivity_) {
    case StvlLow:
      zoneresult = SaturationDynamicLimit(zone, slow_low, slow_high);
      break;
    case StvlStandard:
      zoneresult = SaturationDynamicLimit(zone, sstandard_low, sstandard_high);
      break;
    case StvlHigh:
      zoneresult = SaturationDynamicLimit(zone, shigh_low, shigh_high);
      break;
    default:
      zoneresult = 0.0;
      break;
  }
  return zoneresult;
}

// 左侧触发迟滞范围
std::pair<double, double> LdpCreater::GetZoneHysteresis(bool trigger_last) {
  std::pair<double, double> warning_in_out_zone(0, 0);
  if (!trigger_last) {
    warning_in_out_zone.first = 0.0;
    warning_in_out_zone.second = 0.0;
  } else {
    switch (ldwsensitivity_) {
      case StvlLow:
        warning_in_out_zone.first = kInWarningZoneHysteresisLow;
        warning_in_out_zone.second = kOutWarningZoneHysteresisLow;
        break;
      case StvlStandard:
        warning_in_out_zone.first = kInWarningZoneHysteresisStandard;
        warning_in_out_zone.second = kOutWarningZoneHysteresisStandard;
        break;
      case StvlHigh:
        warning_in_out_zone.first = kInWarningZoneHysteresisHigh;
        warning_in_out_zone.second = kOutWarningZoneHysteresisHigh;
        break;
      default:
        warning_in_out_zone.first = 0.0;
        warning_in_out_zone.second = 0.0;
        break;
    }
  }
  return warning_in_out_zone;
}

// 车道宽度计算
double LdpCreater::GetEgoLaneWidth(bool left_goodpath, bool right_goodpath,
                                   LdpLdwData* const ldp_ldw_data) {
  if (!left_goodpath || !right_goodpath) {
    return 3.5;
  }
  double ego_lane_width =
      std::abs(ldp_ldw_data->left_line_markers_c0 *
                   (std::cos(std::atan(ldp_ldw_data->left_line_markers_c1))) -
               ldp_ldw_data->right_line_markers_c0 *
                   (std::cos(std::atan(ldp_ldw_data->right_line_markers_c1))));
  ADEBUG << "lane width: " << SaturationDynamicLimit(ego_lane_width, 2.6, 4.5);
  return SaturationDynamicLimit(ego_lane_width, 2.6, 4.5);
}

// Ay插值--inner
double LdpCreater::InnerZoneCalculatorAyShift(double aheadcurvature,
                                              double speed, int symbol) {
  double ego_ay = aheadcurvature * speed * speed * symbol;
  switch (ldwsensitivity_) {
    case StvlLow:
      return SaturationDynamicLimit(
          0.5 * ego_ay * kWarningAyTtlcLow * kWarningAyTtlcLow, -0.15, 0.2);
      break;
    case StvlStandard:
      return SaturationDynamicLimit(
          0.5 * ego_ay * kWarningAyTtlcStandard * kWarningAyTtlcStandard, -0.15,
          0.2);
      break;
    case StvlHigh:
      return SaturationDynamicLimit(
          0.5 * ego_ay * kWarningAyTtlcHigh * kWarningAyTtlcHigh, -0.15, 0.2);
      break;
    default:
      return SaturationDynamicLimit(
          0.5 * ego_ay * kWarningAyTtlcLow * kWarningAyTtlcLow, -0.15, 0.2);
      break;
  }
}

// Vy插值inner&&outer
double LdpCreater::InnerZoneCalculatorVyShift(double angletheta, double speed,
                                              int symbol) {
  double ego_vy = speed * std::sin(angletheta) * symbol;
  switch (ldwsensitivity_) {
    case StvlLow:
      return SaturationDynamicLimit(ego_vy * kWarningVyTtlcLow, 0.0, 0.1);
      break;
    case StvlStandard:
      return SaturationDynamicLimit(ego_vy * kWarningVyTtlcStandard, 0.0, 0.2);
      break;
    case StvlHigh:
      return SaturationDynamicLimit(ego_vy * kWarningVyTtlcHigh, 0.0, 0.3);
      break;
    default:
      return SaturationDynamicLimit(ego_vy * kWarningVyTtlcLow, 0.0, 0.1);
      break;
  }
}

// position插值--inner
double LdpCreater::InnerZoneCalculatorPositionn(double ego_lane_width) {
  switch (ldwsensitivity_) {
    case StvlLow:
      return in_zone_position_table_low_.Interpolate(ego_lane_width);
      // return 0.2001;
      break;
    case StvlStandard:
      return in_zone_position_table_standard_.Interpolate(ego_lane_width);
      // return 0.2002;
      break;
    case StvlHigh:
      return in_zone_position_table_high_.Interpolate(ego_lane_width);
      // return 0.2003;
      break;
    default:
      return in_zone_position_table_low_.Interpolate(ego_lane_width);
      // return 0.2;
      break;
  }
}

// 前馈曲率计算
double LdpCreater::LookAheadCurvature(double speed, double a3, double a2) {
  double lookahead_curvature_time = 0.0;
  switch (ldwsensitivity_) {
    case StvlLow:
      lookahead_curvature_time = kLookaheadCurvatureTimeLow;
      break;
    case StvlStandard:
      lookahead_curvature_time = kLookaheadCurvatureTimeStandard;
      break;
    case StvlHigh:
      lookahead_curvature_time = kLookaheadCurvatureTimeHigh;
      break;
    default:
      lookahead_curvature_time = kLookaheadCurvatureTimeLow;
      break;
  }
  return ldp_lowpass_filter_.Filter(
      SaturationDynamicLimit((lookahead_curvature_time * speed * a3 + a2) * 2,
                             -0.02, 0.02));  // 滤波结果待确认
}

// LDW内外触发区域计算//LDP可复用，但限幅要改
std::tuple<double, double, double, double> LdpCreater::WarningZoneCalculator(
    double ego_lane_width, double speed, bool left_trigger_last,
    bool right_trigger_last, LdpLdwData* const ldp_ldw_data) {
  // leftin,leftouter,rightin,rightouter
  std::tuple<double, double, double, double> warning_inout_zone{0, 0, 0, 0};

  double inner_zone_position_result =
      InnerZoneCalculatorPositionn(ego_lane_width);
  double inner_zone_calculator_ayshift_left =
      InnerZoneCalculatorAyShift(
          LookAheadCurvature(speed, ldp_ldw_data->left_line_markers_c3,
                             ldp_ldw_data->left_line_markers_c2),
          speed, 1) *
      -1;
  double inner_zone_calculator_ayshift_right =
      InnerZoneCalculatorAyShift(
          LookAheadCurvature(speed, ldp_ldw_data->right_line_markers_c3,
                             ldp_ldw_data->right_line_markers_c2),
          speed, -1) *
      -1;
  std::get<0>(warning_inout_zone) =
      WarningZoneLimit(
          inner_zone_position_result +
              InnerZoneCalculatorVyShift(
                  std::atan(ldp_ldw_data->left_line_markers_c1), speed, -1) +
              inner_zone_calculator_ayshift_left,
          -0.2, 0.3, -0.4, 0.55, -0.4, 0.5) +
      GetZoneHysteresis(left_trigger_last).first;
  std::get<1>(warning_inout_zone) =
      WarningZoneLimit(-0.35 + inner_zone_calculator_ayshift_left, -0.8, 1,
                       -0.8, 1, -0.8, 1) +
      GetZoneHysteresis(left_trigger_last).second;
  std::get<2>(warning_inout_zone) =
      -1 * WarningZoneLimit(
               inner_zone_position_result +
                   InnerZoneCalculatorVyShift(
                       std::atan(ldp_ldw_data->right_line_markers_c1), speed,
                       1) +
                   inner_zone_calculator_ayshift_right,
               -0.3, 0.2, -0.4, 0.55, -0.5, 0.4) +
      (-1) * GetZoneHysteresis(right_trigger_last).first;
  std::get<3>(warning_inout_zone) =
      -1 * WarningZoneLimit(-0.35 + inner_zone_calculator_ayshift_right, -0.8,
                            1, -0.8, 1, -0.8, 1) +
      (-1) * GetZoneHysteresis(right_trigger_last).second;
  // AERROR << "左内" << std::get<0>(warning_inout_zone);
  // AERROR << "左外" << std::get<1>(warning_inout_zone);
  // AERROR << "右内" << std::get<2>(warning_inout_zone);
  // AERROR << "右外" << std::get<3>(warning_inout_zone);
  // AERROR << "右内未limit:"
  //        << inner_zone_position_result +
  //               InnerZoneCalculatorVyShift(std::atan(right_line_markers_c1_),
  //                                          speed, 1) +
  //               inner_zone_calculator_ayshift_right;
  // AERROR << "右侧Vy偏移量:"
  //  << InnerZoneCalculatorVyShift(std::atan(right_line_markers_c1_), speed,
  //                                      1);
  // AERROR << "右侧Ay偏移量:" << inner_zone_calculator_ayshift_right;
  // AERROR << "左内未limit:"
  //        << inner_zone_position_result +
  //               InnerZoneCalculatorVyShift(std::atan(left_line_markers_c1_),
  //                                          speed, -1) +
  //               inner_zone_calculator_ayshift_left;
  // AERROR << "左侧Vy偏移量:"
  //        << inner_zone_position_result +
  //               InnerZoneCalculatorVyShift(std::atan(left_line_markers_c1_),
  //                                          speed, -1);
  // AERROR << "左侧Ay偏移量:" << inner_zone_calculator_ayshift_left;
  // AERROR << "基础区域:" << inner_zone_position_result;
  return warning_inout_zone;
}

bool LdpCreater::LeftWarningLogic(const double a1,      // NOLINT
                                  const double vehspd,  // NOLINT
                                  const double tire_distance_2_line,
                                  const bool good_path,
                                  const double warning_inzone,
                                  const double warning_outzone) const {
  // AERROR << "左前轮距离左侧车道线距离:" << tire_distance_2_line;
  // AERROR << "车辆横向速度：" << vehspd * std::sin(std::atan(a1));
  return (vehspd * std::sin(std::atan(a1)) < kVxThresholdVaule) &&
         (warning_inzone > tire_distance_2_line) &&
         (warning_outzone < tire_distance_2_line) && good_path;
}

bool LdpCreater::RightWarningLogic(const double a1,      // NOLINT
                                   const double vehspd,  // NOLINT
                                   const double tire_distance_2_line,
                                   const bool good_path,
                                   const double warning_inzone,
                                   const double warning_outzone) const {
  // AERROR << "右前轮距离右侧车道线距离:" << tire_distance_2_line;
  return (vehspd * std::sin(std::atan(a1)) > -kVxThresholdVaule) &&
         (warning_inzone < tire_distance_2_line) &&
         (warning_outzone > tire_distance_2_line) && good_path;
}

bool LdpCreater::ODDindxCondDeal(uint16_t indx_cond,
                                 uint16_t indx_select_cond) {
  uint16_t result_and = (indx_cond & indx_select_cond);
  uint16_t result_not = (~indx_select_cond);
  return (result_and ^ result_not) == 65535;
}

int LdpCreater::LdpLineBoundaryCase(bool left_quality, bool right_quality) {
  if (!left_quality && !right_quality) {
    return 0;
  }
  if (!left_quality && right_quality) {
    return 1;
  }
  if (!right_quality && left_quality) {
    return 2;
  }
  return 3;
}

bool LdpCreater::LdpPositionPrevDone(int boundary_case,  // NOLINT
                                     double left_a0,     // NOLINT
                                     double right_a0) const {
  double ldp_pos_center = 0;
  switch (boundary_case) {
    case 0:
      return false;
      break;
    case 1:
      ldp_pos_center = 3.5 + right_a0;
      break;
    case 2:
      ldp_pos_center = 3.5 - left_a0;
      break;
    case 3:
      ldp_pos_center = (left_a0 + right_a0) * 0.5;
      break;
    default:
      break;
  }
  return std::abs(ldp_pos_center) < kLdpPrevDonePosThd;
}

bool LdpCreater::LdpHeaderPrevDone(int boundary_case, double left_a1,
                                   double right_a1, double speed,
                                   bool trigger_condition) {
  double ldp_ref_heading = 0;
  switch (boundary_case) {
    case 0:
      return false;
      break;
    case 1:
      ldp_ref_heading = left_a1;
      break;
    case 2:
      ldp_ref_heading = right_a1;
      break;
    case 3:
      ldp_ref_heading = (left_a1 + right_a1) * 0.5;
      break;
    default:
      break;
  }
  return (std::abs(ldp_ref_heading) <
          ldp_heading_threshold_.Interpolate(speed * 3.6)) &&
         !trigger_condition;
}

void LdpCreater::LdpDoneState(int* last_state, bool ldp_active,  // NOLINT
                              double* counter,                   // NOLINT
                              bool ldp_done_trigger) const {
  switch (*last_state) {
    case 0:
      if (ldp_active && ldp_done_trigger) {
        *counter = 0;
        *last_state = 1;
      }
      break;
    case 1:
      *counter += 0.1;
      if (!ldp_active || !ldp_done_trigger) {
        *last_state = 0;
        *counter = 0;
      } else if (*counter >= kLdpVehiclePosZoneTime) {
        *last_state = 2;
      }
      break;
    case 2:
      if (!ldp_active || !ldp_done_trigger) {
        *last_state = 0;
        *counter = 0;
      }
      break;
    default:
      break;
  }
}

bool LdpCreater::LdpDynamicRelay(double x, double relay_on, double relay_off,
                                 bool* result_last) {
  if (x >= relay_on) {
    *result_last = true;
  }
  if (x <= relay_off) {
    *result_last = false;
  }
  return *result_last;
}

int LdpCreater::LdpRoadCurveJudgement(double left_a2, double left_a3,
                                      double right_a2, double right_a3,
                                      int ldp_boundary_case, double speed) {
  double a2 = 0;
  double a3 = 0;
  switch (ldp_boundary_case) {
    case 0:
      a2 = 0;
      a3 = 0;
      break;
    case 1:
      a2 = left_a2;
      a3 = left_a3;
      break;
    case 2:
      a2 = right_a2;
      a3 = right_a3;
      break;
    case 3:
      a2 = (left_a2 + right_a2) * 0.5;
      a3 = (left_a3 + right_a3) * 0.5;
      break;
    default:
      break;
  }
  double road_curve_lat_acc =
      speed * speed *
      SaturationDynamicLimit(
          2 * (a2 + a3 * ldp_curve_ref_point_dist_.Interpolate((speed * 3.6))),
          -0.02, 0.02);
  bool left_curve =
      ldp_road_classification_debounce_.DealDebounce(LdpDynamicRelay(
          road_curve_lat_acc, ldp_latacc_threshold_.Interpolate(speed * 3.6),
          ldp_latacc_threshold_.Interpolate(speed * 3.6) - 0.15,
          &ldp_latacc_to_left_last_));
  bool right_curve =
      ldp_road_classification_debounce_.DealDebounce(LdpDynamicRelay(
          -road_curve_lat_acc, ldp_latacc_threshold_.Interpolate(speed * 3.6),
          ldp_latacc_threshold_.Interpolate(speed * 3.6) - 0.15,
          &ldp_latacc_to_right_last_));
  return static_cast<int>(left_curve) * 1 + static_cast<int>(right_curve) * 2;
}

void LdpCreater::LdpMessageInfo(
    const std::shared_ptr<LocalView>& local_view,
    std::tuple<double, double, double, double> warning_zone,
    LdpLdwData* const ldp_ldw_data,
    const std::shared_ptr<LanemarkersLaneLine>& lanemarker_debug) const {
  UNUSED(local_view);
  auto* ldp_ldw_info = lanemarker_debug->mutable_ldp_ldw_warning_info();
  // ldp_ldw_info->set_name("LDW_LDP_DEBUG:");
  ldp_ldw_info->set_ldp_left_flag(ldp_left_warning_trigger_);
  ldp_ldw_info->set_ldp_right_flag(ldp_right_warning_trigger_);
  ldp_ldw_info->set_ldp_left_done(ldp_left_warning_done_trigger_);
  ldp_ldw_info->set_ldp_right_done(ldp_right_warning_done_trigger_);
  auto* warning_z = ldp_ldw_info->mutable_ldw_warning_zone();
  warning_z->set_name("LDP_warning_zone:");
  warning_z->set_left_in(std::get<0>(warning_zone));
  warning_z->set_left_out(std::get<1>(warning_zone));
  warning_z->set_right_in(std::get<2>(warning_zone));
  warning_z->set_right_out(std::get<3>(warning_zone));
  auto* tire_2_line = ldp_ldw_info->mutable_ldp_ldw_tire_2_line();
  tire_2_line->set_name("Tire_2_Line::");
  tire_2_line->set_dis_2_left(ldp_ldw_data->left_tire_distance_2_line);
  tire_2_line->set_dis_2_right(ldp_ldw_data->right_tire_distance_2_line);
  // tire_2_line->set_name("tire_2_line:");
  ADEBUG << "left_tire_distance_2_line:"
         << ldp_ldw_data->left_tire_distance_2_line;
  ADEBUG << "left_tire_distance_2_line:"
         << ldp_ldw_data->right_tire_distance_2_line;
  // tire_2_line->set_dis_2_left(ldp_ldw_data->left_tire_distance_2_line);
  // tire_2_line->set_dis_2_right(ldp_ldw_data->right_tire_distance_2_line);
}

bool LdpCreater::LdpPreventionDone(LdpLdwData* const ldp_ldw_data,
                                   int ldp_boundary_case,
                                   bool trigger_condition, bool ldp_active,
                                   double speed, bool is_hand_off) {
  bool ldp_pos_prevention_done =
      LdpPositionPrevDone(ldp_boundary_case, ldp_ldw_data->left_line_markers_c0,
                          ldp_ldw_data->right_line_markers_c0);
  bool ldp_header_prevention_done = LdpHeaderPrevDone(
      ldp_boundary_case, ldp_ldw_data->left_line_markers_c1,
      ldp_ldw_data->right_line_markers_c1, speed, trigger_condition);
  // 未区分左右侧，左右公用一个信号，后面要改
  LdpDoneState(&ldp_isin_pos_zone_last_, ldp_active, &pos_counter_,
               ldp_pos_prevention_done);
  LdpDoneState(&ldp_isin_pos_header_zone_last_, ldp_active,
               &pos_header_counter_,
               (ldp_pos_prevention_done && ldp_header_prevention_done));
  LdpDoneState(&ldp_isin_header_zone_last_, ldp_active, &header_counter_,
               ldp_header_prevention_done);
  int ldp_road_classification = LdpRoadCurveJudgement(
      ldp_ldw_data->left_line_markers_c2, ldp_ldw_data->left_line_markers_c3,
      ldp_ldw_data->right_line_markers_c2, ldp_ldw_data->right_line_markers_c3,
      ldp_boundary_case, speed);
  bool ldp_precondition = false;
  if (ldp_road_classification == 0 || !is_hand_off) {  //  脱手报警信号取or
    ldp_precondition = true;
  }
  return (ldp_isin_pos_zone_last_ == 2 || ldp_isin_pos_header_zone_last_ == 2 ||
          ldp_isin_header_zone_last_ == 2) &&
         ldp_precondition;
}

bool LdpCreater::LdpDeal(
    functionmanager::FunctionManagerOut* const to_fct,
    LdpLdwData* const ldp_ldw_data,
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<LanemarkersLaneLine>& lanemarker_debug) {
  ADEBUG << "Start ldpCreater!!!";  //  4671 7986
  ldp_left_lane_line_condition_ =
      ODDindxCondDeal(ldp_ldw_data->odd_indx_line_cond, 4745);
  ldp_right_lane_line_condition_ =
      ODDindxCondDeal(ldp_ldw_data->odd_indx_line_cond, 40754);
  std::tuple<double, double, double, double> warning_inout_zone_result =
      WarningZoneCalculator(
          GetEgoLaneWidth(ldp_left_lane_line_condition_,
                          ldp_right_lane_line_condition_, ldp_ldw_data),
          vehicle_state_->linear_velocity(), ldp_left_warning_trigger_last_,
          ldp_right_warning_trigger_last_, ldp_ldw_data);
  ldp_left_warning_trigger_ = LeftWarningLogic(
      ldp_ldw_data->left_line_markers_c1, vehicle_state_->linear_velocity(),
      ldp_ldw_data->left_tire_distance_2_line, ldp_left_lane_line_condition_,
      std::get<0>(warning_inout_zone_result),
      std::get<1>(warning_inout_zone_result));
  ldp_left_warning_trigger_last_ = ldp_left_warning_trigger_;
  ldp_right_warning_trigger_ = RightWarningLogic(
      ldp_ldw_data->right_line_markers_c1, vehicle_state_->linear_velocity(),
      ldp_ldw_data->right_tire_distance_2_line, ldp_right_lane_line_condition_,
      std::get<2>(warning_inout_zone_result),
      std::get<3>(warning_inout_zone_result));
  ldp_right_warning_trigger_last_ = ldp_right_warning_trigger_;
  ADEBUG << "LDPLDP_left_: " << ldp_left_warning_trigger_;
  ADEBUG << "LDPLDP__right_: " << ldp_right_warning_trigger_;
  ADEBUG << "-----ldp_left_line: " << ldp_left_lane_line_condition_;
  ADEBUG << "-----ldp_right_line: " << ldp_right_lane_line_condition_;
  uint32_t k_active_left = 0x8000;
  uint32_t k_active_right = 0x10000;
  bool left_ldp_active =
      (local_view->GetFunctionManagerIn()->fct_2_soc_tbd_u32_05() &
       k_active_left) != 0U;
  bool right_ldp_active =
      (local_view->GetFunctionManagerIn()->fct_2_soc_tbd_u32_05() &
       k_active_right) != 0U;
  ADEBUG << "fct_in_05: "
         << local_view->GetFunctionManagerIn()->fct_2_soc_tbd_u32_05();
  ADEBUG << "fct_in_02: "
         << local_view->GetFunctionManagerIn()->fct_2_soc_tbd_u32_02();
  int ldp_boundary_case = LdpLineBoundaryCase(ldp_left_lane_line_condition_,
                                              ldp_right_lane_line_condition_);
  uint32_t k_hand_off = 0x100;
  bool is_hand_off =
      (local_view->GetFunctionManagerIn()->fct_2_soc_tbd_u32_02() &
       k_hand_off) != 0U;
  ldp_left_warning_done_trigger_ = LdpPreventionDone(
      ldp_ldw_data, ldp_boundary_case, ldp_left_warning_trigger_,
      left_ldp_active, vehicle_state_->linear_velocity(), is_hand_off);
  ldp_right_warning_done_trigger_ = LdpPreventionDone(
      ldp_ldw_data, ldp_boundary_case, ldp_right_warning_trigger_,
      right_ldp_active, vehicle_state_->linear_velocity(), is_hand_off);
  // AERROR << "ldp_left_done: " << ldp_left_warning_done_trigger_;
  // AERROR << "ldp_rightt_done: " << ldp_right_warning_done_trigger_;
  auto soc_04_val = to_fct->soc_2_fct_tbd_u32_04();
  uint32_t ldp_warning_trigger_left = ldp_left_warning_trigger_ ? 0x1000 : 0x0;
  uint32_t ldp_warning_trigger_right =
      ldp_right_warning_trigger_ ? 0x2000 : 0x0;
  uint32_t ldw_lane_line_left = ldp_left_lane_line_condition_ ? 0x10000 : 0x0;
  uint32_t ldw_lane_line_right = ldp_right_lane_line_condition_ ? 0x20000 : 0x0;
  uint32_t ldp_warning_done_trigger_left =
      ldp_left_warning_done_trigger_ ? 0x4000 : 0x0;
  uint32_t ldp_warning_done_trigger_right =
      ldp_right_warning_done_trigger_ ? 0x8000 : 0x0;
  to_fct->set_soc_2_fct_tbd_u32_04(
      soc_04_val | ldp_warning_trigger_left | ldp_warning_trigger_right |
      ldw_lane_line_left | ldw_lane_line_right | ldp_warning_done_trigger_left |
      ldp_warning_done_trigger_right);
  ADEBUG << "ldp_u32_o4 : " << to_fct->soc_2_fct_tbd_u32_04();
  LdpMessageInfo(local_view, warning_inout_zone_result, ldp_ldw_data,
                 lanemarker_debug);
  bool is_drive_auto{false};
  if (local_view->HasFunctionManagerIn() &&
      local_view->GetFunctionManagerIn()->has_fct_nnp_in()) {
    const auto nnp_sys_state =
        local_view->GetFunctionManagerIn()->fct_nnp_in().nnp_sysstate();
    const auto pilot_sys_state =
        local_view->GetFunctionManagerIn()->fct_nnp_in().npilot_state();
    is_drive_auto = (nnp_sys_state == NNPSysState::NNPS_ACTIVE ||
                     nnp_sys_state == NNPSysState::NNPS_OVERRIDE ||
                     nnp_sys_state == NNPSysState::NNPS_LAT_OVERRIDE ||
                     nnp_sys_state == NNPSysState::NNPS_LON_OVERRIDE ||
                     pilot_sys_state == FctToNnpInput::PILOT_ACTIVE);
  }
#ifdef FOR_BAIDU_SIMULATION
  is_drive_auto = true;
#endif
  to_fct->mutable_ldp_ldw_warning_info()->set_ldp_left_flag(
      !is_drive_auto && ldp_left_warning_trigger_);
  to_fct->mutable_ldp_ldw_warning_info()->set_ldp_right_flag(
      !is_drive_auto && ldp_right_warning_trigger_);
  to_fct->mutable_ldp_ldw_warning_info()->set_ldp_left_done(
      ldp_left_warning_done_trigger_);
  to_fct->mutable_ldp_ldw_warning_info()->set_ldp_right_done(
      ldp_right_warning_done_trigger_);
  return true;
}

}  // namespace planning
}  // namespace TL
