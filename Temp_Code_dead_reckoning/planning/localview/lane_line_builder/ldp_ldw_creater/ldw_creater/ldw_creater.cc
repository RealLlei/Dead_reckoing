/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/ldp_ldw_creater/ldw_creater/ldw_creater.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>
#include "common/interpolation/interpolation_1d.h"
#include "common/status/status.h"
#include "planning/localview/lane_line_builder/ldp_ldw_creater/ldp_ldw_base.h"

namespace TL {
namespace planning {
namespace {
constexpr double kVxThresholdVaule = 0.1;  // 配置参数
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
}  // namespace

LdwCreater::LdwCreater(const PerceptionMapConfig& config)
    : navi_hdmap_config_(config) {}

Status LdwCreater::Init() {
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  InnerZoneTableCreater();
  ldw_lowpass_filter_.set_fs_and_fc(10, 0.5);
  in_zone_position_table_low_.Init(xy1_);
  in_zone_position_table_standard_.Init(xy2_);
  in_zone_position_table_high_.Init(xy3_);
  return Status::OK();
}

Status LdwCreater::Start() {
  return Status::OK();
}

void LdwCreater::Stop() {}

void LdwCreater::SetDealMg(const std::shared_ptr<LocalView>& local_view) {
  vehicle_state_ = local_view->GetVehicleState();
  perception_obstacles_ = local_view->GetPerceptionObstacles();
}

//  插值表创建
void LdwCreater::InnerZoneTableCreater() {
  for (int i = 0; i < 6; i++) {
    xy1_.emplace_back(std::make_pair(inner_zone_table_arr_x_.at(i),
                                    inner_zone_table_arr_y1_.at(i)));
    xy2_.emplace_back(std::make_pair(inner_zone_table_arr_x_.at(i),
                                    inner_zone_table_arr_y2_.at(i)));
    xy3_.emplace_back(std::make_pair(inner_zone_table_arr_x_.at(i),
                                    inner_zone_table_arr_y3_.at(i)));
  }
}

bool LdwCreater::IsGoodPath(double line_quality, double quality_vaule) {
  return line_quality > quality_vaule;
}

// 限幅函数
double LdwCreater::SaturationDynamicLimit(double x, double low, double high) {
  if (x > high) {
    return high;
  }
  if (x < low) {
    return low;
  }
  return x;
}

// 触发区域限幅
double LdwCreater::WarningZoneLimit(double zone, double slow_low,
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

// //  右侧触发迟滞范围
// std::pair<double, double> LdwCreater::GetRightZoneHysteresis() {
//   std::pair<double, double> warning_in_out_zone;
//   if (!rightwarningtriggerlast_) {
//     warning_in_out_zone.first = 0.0;
//     warning_in_out_zone.second = 0.0;
//   } else {
//     switch (ldwsensitivity_) {
//       case StvlLow:
//         warning_in_out_zone.first = kInWarningZoneHysteresisLow;
//         warning_in_out_zone.second = kOutWarningZoneHysteresisLow;
//         break;
//       case StvlStandard:
//         warning_in_out_zone.first = kInWarningZoneHysteresisStandard;
//         warning_in_out_zone.second = kOutWarningZoneHysteresisStandard;
//         break;
//       case StvlHigh:
//         warning_in_out_zone.first = kInWarningZoneHysteresisHigh;
//         warning_in_out_zone.second = kOutWarningZoneHysteresisHigh;
//         break;
//       default:
//         warning_in_out_zone.first = 0.0;
//         warning_in_out_zone.second = 0.0;
//         break;
//     }
//   }
//   return warning_in_out_zone;
// }

// 左侧触发迟滞范围
std::pair<double, double> LdwCreater::GetZoneHysteresis(bool trigger_last) {
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
double LdwCreater::GetEgoLaneWidth(bool left_goodpath, bool right_goodpath,
                                   LdpLdwData* const ldp_ldw_data) {
  if (!left_goodpath || !right_goodpath) {
    return 3.5;
  }
  double ego_lane_width =
      std::abs(ldp_ldw_data->left_line_markers_c0 *
                   (std::cos(std::atan(ldp_ldw_data->left_line_markers_c1))) -
               ldp_ldw_data->right_line_markers_c0 *
                   (std::cos(std::atan(ldp_ldw_data->right_line_markers_c1))));
  ADEBUG << "lane width:" << SaturationDynamicLimit(ego_lane_width, 2.6, 4.5);
  return SaturationDynamicLimit(ego_lane_width, 2.6, 4.5);
}

// Ay插值--inner
double LdwCreater::InnerZoneCalculatorAyShift(double aheadcurvature,
                                              double speed, int symbol) {
  double ego_ay = aheadcurvature * speed * speed * symbol;
  switch (ldwsensitivity_) {
    case StvlLow:
      return SaturationDynamicLimit(
          0.5 * ego_ay * kWarningAyTtlcLow * kWarningAyTtlcLow, 0.0, 0.35);
      break;
    case StvlStandard:
      return SaturationDynamicLimit(
          0.5 * ego_ay * kWarningAyTtlcStandard * kWarningAyTtlcStandard, 0.0,
          0.25);
      break;
    case StvlHigh:
      return SaturationDynamicLimit(
          0.5 * ego_ay * kWarningAyTtlcHigh * kWarningAyTtlcHigh, 0.0, 0.15);
      break;
    default:
      return SaturationDynamicLimit(
          0.5 * ego_ay * kWarningAyTtlcLow * kWarningAyTtlcLow, 0.0, 0.25);
      break;
  }
}

// Vy插值inner&&outer
double LdwCreater::InnerZoneCalculatorVyShift(double angletheta, double speed,
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
double LdwCreater::InnerZoneCalculatorPositionn(double ego_lane_width) {
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
double LdwCreater::LookAheadCurvature(double speed, double a3, double a2) {
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
  return ldw_lowpass_filter_.Filter(
      SaturationDynamicLimit((lookahead_curvature_time * speed * a3 + a2) * 2,
                             -0.02, 0.02));  // 滤波结果待确认
}

// LDW内外触发区域计算//LDP可复用，但限幅要改
std::tuple<double, double, double, double> LdwCreater::WarningZoneCalculator(
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
          -0.2, 0.3, -0.3, 0.4, -0.4, 0.5) +
      GetZoneHysteresis(left_trigger_last).first;
  std::get<1>(warning_inout_zone) =
      WarningZoneLimit(-0.35 + inner_zone_calculator_ayshift_left, -0.8, 999,
                       -0.8, 999, -0.8, 999) +
      GetZoneHysteresis(left_trigger_last).second;
  std::get<2>(warning_inout_zone) =
      -1 * WarningZoneLimit(
               inner_zone_position_result +
                   InnerZoneCalculatorVyShift(
                       std::atan(ldp_ldw_data->right_line_markers_c1), speed,
                       1) +
                   inner_zone_calculator_ayshift_right,
               -0.3, 0.2, -0.4, 0.3, -0.5, 0.4) +
      (-1) * GetZoneHysteresis(right_trigger_last).first;
  std::get<3>(warning_inout_zone) =
      -1 * WarningZoneLimit(-0.35 + inner_zone_calculator_ayshift_right, -999,
                            0.8, -999, 0.8, -999, 0.8) +
      (-1) * GetZoneHysteresis(right_trigger_last).second;
  ADEBUG << "left in: " << std::get<0>(warning_inout_zone);
  ADEBUG << "left out: " << std::get<1>(warning_inout_zone);
  ADEBUG << "right in: " << std::get<2>(warning_inout_zone);
  ADEBUG << "right out: " << std::get<3>(warning_inout_zone);
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

// // 计算左前轮到左车道线的距离
// double LdwCreater::GetLeftTireDistance2Line() {
//   return left_line_markers_offset_last_ - 0.5 * vehicle_param_.width();
// }

// // 计算右前轮到右车道线的距离
// double LdwCreater::GetRightTireDistance2Line() {
//   return right_line_markers_offset_last_ + 0.5 * vehicle_param_.width();
// }

bool LdwCreater::LeftWarningLogic(const double a1,      // NOLINT
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

bool LdwCreater::RightWarningLogic(const double a1,      // NOLINT
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

bool LdwCreater::ODDindxCondDeal(uint16_t indx_cond,
                                 uint16_t indx_select_cond) {
  uint16_t result_and = (indx_cond & indx_select_cond);
  uint16_t result_not = (~indx_select_cond);
  return (result_and ^ result_not) == 65535;
}

void LdwCreater::LdwMessageInfo(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<LanemarkersLaneLine>& lanemarker_debug) const {
  UNUSED(local_view);
  auto* ldp_ldw_info = lanemarker_debug->mutable_ldp_ldw_warning_info();
  ldp_ldw_info->set_name("LDW_LDP_DEBUG:");
  ldp_ldw_info->set_ldw_left_flag(ldw_left_warning_trigger_);
  ldp_ldw_info->set_ldw_right_flag(ldw_right_warning_trigger_);
  // auto* ldw_warning_info = ldp_ldw_in
  // ldp_ldw_info->set_ldp_left_flag(ldp_left_warning_trigger_);
  // ldp_ldw_info->set_ldp_right_flag(ldp_right_warning_trigger_);
  // ldp_ldw_info->set_ldp_left_done(ldp_left_warning_done_trigger_);
  // ldp_ldw_info->set_ldp_right_done(ldp_right_warning_done_trigger_);
}

bool LdwCreater::LdwDeal(functionmanager::FunctionManagerOut* const to_fct,
                         LdpLdwData* const ldp_ldw_data,
                         const std::shared_ptr<LocalView>& local_view) {
  ADEBUG << "Start LdwCreater!!!";
  if (local_view->HasFunctionManagerIn() &&
      local_view->GetFunctionManagerIn()->has_fct_nnp_in()) {
    const auto nnp_sys_state =
        local_view->GetFunctionManagerIn()->fct_nnp_in().npilot_state();
    if (nnp_sys_state == functionmanager::FctToNnpInput::PILOT_ACTIVE) {
      ldwsensitivity_ = StvlLow;
    } else {
      ldwsensitivity_ = StvlStandard;
    }
  }
  // const auto lanemarkers = perception_obstacles_->lane_marker();
  // LineUpdate(lanemarkers);
  ldw_left_lane_line_condition_ =
      ODDindxCondDeal(ldp_ldw_data->odd_indx_line_cond, 32093);
  ldw_right_lane_line_condition_ =
      ODDindxCondDeal(ldp_ldw_data->odd_indx_line_cond, 32093);
  std::tuple<double, double, double, double> warning_inout_zone_result =
      WarningZoneCalculator(
          GetEgoLaneWidth(ldw_left_lane_line_condition_,
                          ldw_right_lane_line_condition_, ldp_ldw_data),
          vehicle_state_->linear_velocity(), ldw_left_warning_trigger_last_,
          ldw_right_warning_trigger_last_, ldp_ldw_data);
  ldw_left_warning_trigger_ = LeftWarningLogic(
      ldp_ldw_data->left_line_markers_c1, vehicle_state_->linear_velocity(),
      ldp_ldw_data->left_tire_distance_2_line, ldw_left_lane_line_condition_,
      std::get<0>(warning_inout_zone_result),
      std::get<1>(warning_inout_zone_result));
  ADEBUG << "ldw_left_warning_trigger: :" << ldw_left_warning_trigger_;
  ldw_left_warning_trigger_last_ = ldw_left_warning_trigger_;
  ldw_right_warning_trigger_ = RightWarningLogic(
      ldp_ldw_data->right_line_markers_c1, vehicle_state_->linear_velocity(),
      ldp_ldw_data->right_tire_distance_2_line, ldw_right_lane_line_condition_,
      std::get<2>(warning_inout_zone_result),
      std::get<3>(warning_inout_zone_result));
  ADEBUG << "ldw_right_warning_trigger: :" << ldw_right_warning_trigger_;
  ldw_right_warning_trigger_last_ = ldw_right_warning_trigger_;
  ADEBUG << "++++++ldw_left_line: " << ldw_left_lane_line_condition_;
  ADEBUG << "++++++ldw_right_line: " << ldw_right_lane_line_condition_;
  auto soc_04_val = to_fct->soc_2_fct_tbd_u32_04();
  uint32_t ldw_warning_trigger_left = ldw_left_warning_trigger_ ? 0x100 : 0x0;
  uint32_t ldw_warning_trigger_right = ldw_right_warning_trigger_ ? 0x200 : 0x0;
  uint32_t ldw_lane_line_left = ldw_left_lane_line_condition_ ? 0x400 : 0x0;
  uint32_t ldw_lane_line_right = ldw_right_lane_line_condition_ ? 0x800 : 0x0;
  to_fct->set_soc_2_fct_tbd_u32_04(soc_04_val | ldw_warning_trigger_left |
                                   ldw_warning_trigger_right |
                                   ldw_lane_line_left | ldw_lane_line_right);
  ADEBUG << "ldw_u32_o4 : " << to_fct->soc_2_fct_tbd_u32_04();
  to_fct->mutable_ldp_ldw_warning_info()->set_ldw_left_flag(
      ldw_left_warning_trigger_);
  to_fct->mutable_ldp_ldw_warning_info()->set_ldw_right_flag(
      ldw_right_warning_trigger_);
  return true;
}

}  // namespace planning
}  // namespace TL
