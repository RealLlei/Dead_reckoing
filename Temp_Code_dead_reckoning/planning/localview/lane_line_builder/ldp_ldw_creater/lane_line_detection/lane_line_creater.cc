/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/ldp_ldw_creater/lane_line_detection/lane_line_creater.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>
#include "common/status/status.h"
#include "planning/localview/lane_line_builder/ldp_ldw_creater/ldw_creater/ldw_creater.h"

namespace TL {
namespace planning {
namespace {
constexpr double kLdwMainLoopTime = 0.1;  // 配置文件
constexpr double kOddCorridorLengthRiseTime = 0.1;
constexpr double kOddCorridorLengthFallTime = 1;
constexpr double kOddMaxLaneRangeStart = 2;            // m
constexpr double kOddMinCorridorLengthThreshold = 10;  // m
constexpr double kOddLineCurvatureEnbleThreshold = 0.007;
constexpr double kOddLineCurvatureDisenableThreshold = 0.0084;
// constexpr double KOddRideLineRiseTime = 0.1;
// constexpr double KOddRideLineFallTime = 1;
constexpr double kLineA2RatMaxThreshold_ = 0.01;
constexpr double kLineA1RatMaxThreshold_ = 0.2;
constexpr double kLineA0RatMaxThreshold_ = 2;
constexpr double kOddLineCrvRefPntDist = 40;               // m
constexpr double kOddEnbleMaxCoridorWidthThreshold = 4.6;  // m
constexpr double kOddDisenableMaxCoridorWidthThreshold = 4.8;
constexpr double kOddEnableMinCoridorWidthThreshold = 3.2;
constexpr double kOddDisenableMinCoridorWidthThreshold = 3;
constexpr double kLdwCorridorWidthHighThd = 4.5;
constexpr double kLdwCorridorWidthLowThd = 2.9;
constexpr double kLdwLdpActiveLineStartMax = 3.0;
}  // namespace

LaneLineMarkerDetection::LaneLineMarkerDetection(
    const PerceptionMapConfig& config)
    : navi_hdmap_config_(config) {}

Status LaneLineMarkerDetection::Start() {  // NOLINT
  return Status::OK();
}

void LaneLineMarkerDetection::Stop() {}

// void LaneLineMarkerDetection::SetLaneMg(
//     const std::shared_ptr<LocalView>& local_view) {
//   // perception_obstaclesLine_ = local_view->GetPerceptionObstacles();
//   lane_marker = local_view->GetLaneMarkers();
// }

Status LaneLineMarkerDetection::Init() {
  //  车道线初始化
  // lane_markers_last_ = perception_obstaclesLine_->lane_marker();
  vehicle_paramline_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  line_length_debounce_.ResetTime(kOddCorridorLengthRiseTime,
                                  kOddCorridorLengthFallTime, kLdwMainLoopTime);
  line_range_start_low_pass_filter_.set_fs_and_fc(1 / kLdwMainLoopTime, 1.5);
  // line_quality_debounce_.ResetTime(0.5, 0, kLdwMainLoopTime);
  line_curvature_debounce_.ResetTime(0.5, 1, kLdwMainLoopTime);
  left_line_ride_line_debounce_.ResetTime(0.1, 1, kLdwMainLoopTime);
  right_line_ride_line_debounce_.ResetTime(0.1, 1, kLdwMainLoopTime);
  line_jump_pass_filter_.set_fs_and_fc(1 / kLdwMainLoopTime, 5);
  line_terrible_debounce_.ResetTime(1, 0.1, kLdwMainLoopTime);
  line_width_high_debounce_.ResetTime(1, 0.1, kLdwMainLoopTime);
  line_width_low_debounce_.ResetTime(0.5, 1, kLdwMainLoopTime);

  return Status::OK();
}

// 车道线长度是否满足
bool LaneLineMarkerDetection::ODDCorridorLengthDetection(double start_range,
                                                         double end_range) {
  return ((line_length_debounce_.DealDebounce(kOddMinCorridorLengthThreshold <
                                              (end_range - start_range))) &&
          (line_range_start_low_pass_filter_.Filter(start_range) <=
           kOddMaxLaneRangeStart) &&
          (start_range <= kLdwLdpActiveLineStartMax) &&
          (end_range >= kLdwLdpActiveLineStartMax));
}

// 曲率是否满足
bool LaneLineMarkerDetection::ODDLaneLineCurvatureDetection(double line_a2) {
  bool line_curvature_is_enable = false;
  if (((2 * std::abs(line_a2)) >= kOddLineCurvatureDisenableThreshold)) {
    line_curvature_is_enable = false;
  } else if (((2 * std::abs(line_a2)) <= kOddLineCurvatureEnbleThreshold)) {
    line_curvature_is_enable = true;
  } else {
    line_curvature_is_enable = odd_line_curvature_is_enabel_last_;
  }
  odd_line_curvature_is_enabel_last_ =
      line_curvature_debounce_.DealDebounce(line_curvature_is_enable);
  return odd_line_curvature_is_enabel_last_;
}

// 待验证逻辑正确性//骑线判断
// bool LaneLineMarkerDetection::ODDRideLineDetection(
//     double width, double a0, bool line_quality, bool change_line,
//     bool* odd_ride_line_condition) {
//   bool line0_act = std::abs((a0 - 0.5 * width)) >= 0.1;
//   bool line0_sup = (std::abs(a0) <= (0.8 * 0.5 * width)) && line_quality;
//   AERROR << "line0_act: " << line0_act;
//   AERROR << "line0_sup: " << line0_sup;
//   if (change_line) {
//     *odd_ride_line_condition = true;
//     return *odd_ride_line_condition;
//   }
//   if (!line_quality) {
//     *odd_ride_line_condition = false;
//     return *odd_ride_line_condition;
//   }
//   if (!*odd_ride_line_condition && line0_sup) {
//     *odd_ride_line_condition = true;
//   } else if (*odd_ride_line_condition && !line0_sup && line0_act) {
//     *odd_ride_line_condition = false;
//   }
//   // return line_ride_line_debounce_.DealDebounce(*odd_ride_line_condition);
//   return *odd_ride_line_condition;
// }

//  左侧骑线判断
bool LaneLineMarkerDetection::LeftODDRideLineDetection(double width, double a0,
                                                       bool line_quality,
                                                       bool change_line) {
  bool line0_act = std::abs((a0 - 0.5 * width)) >= 0.1;
  bool line0_sup = (std::abs(a0) <= (0.3 * 0.5 * width)) && line_quality;
  // AERROR << "left_line0_act: " << line0_act;
  // AERROR << "left_line0_sup: " << line0_sup;
  if (change_line) {
    left_ride_line_condition_last_ = true;
    return left_ride_line_condition_last_;
  }
  if (!line_quality) {
    left_ride_line_condition_last_ = false;
    return left_ride_line_condition_last_;
  }
  if (!left_ride_line_condition_last_ && line0_sup) {
    left_ride_line_condition_last_ = true;
  } else if (left_ride_line_condition_last_ && !line0_sup && line0_act) {
    left_ride_line_condition_last_ = false;
  }
  return left_line_ride_line_debounce_.DealDebounce(
      left_ride_line_condition_last_);
}

//  右侧骑线判断
bool LaneLineMarkerDetection::RightODDRideLineDetection(double width, double a0,
                                                        bool line_quality,
                                                        bool change_line) {
  bool line0_act = std::abs((a0 - 0.5 * width)) >= 0.1;
  bool line0_sup = (std::abs(a0) <= (0.3 * 0.5 * width)) && line_quality;
  // AERROR << "right_line0_act: " << line0_act;
  // AERROR << "right_line0_sup: " << line0_sup;
  if (change_line) {
    right_ride_line_condition_last_ = true;
    return right_ride_line_condition_last_;
  }
  if (!line_quality) {
    right_ride_line_condition_last_ = false;
    return right_ride_line_condition_last_;
  }
  if (!right_ride_line_condition_last_ && line0_sup) {
    right_ride_line_condition_last_ = true;
  } else if (right_ride_line_condition_last_ && !line0_sup && line0_act) {
    right_ride_line_condition_last_ = false;
  }
  return right_line_ride_line_debounce_.DealDebounce(
      right_ride_line_condition_last_);
}

// 车道线参数求导
double LaneLineMarkerDetection::ODDLaneLineParamDerviation(double current,
                                                           double last) {
  return line_jump_pass_filter_.Filter((current - last) / kLdwMainLoopTime);
}

double LaneLineMarkerDetection::LookAheadCurvature(double a2,          // NOLINT
                                                   double a3) const {  // NOLINT
  return LdwCreater::SaturationDynamicLimit(
      (a2 + a3 * kOddLineCrvRefPntDist) * 2, -0.02, 0.02);
}

// 车道线跳变判断//xuetianshuaiyaoqiugaiweijump
bool LaneLineMarkerDetection::ODDLaneLineParamJump(
    double lookahead_curvature, double lookahead_curvature_last, double a1,
    double a1_last, double a0, double a0_last, bool quality,
    bool quality_last) {
  double dcurvature =
      ODDLaneLineParamDerviation(lookahead_curvature, lookahead_curvature_last);
  double da1 = ODDLaneLineParamDerviation(a1, a1_last);
  double da0 = ODDLaneLineParamDerviation(a0, a0_last);

  bool lineterriblebl = (std::abs(dcurvature) > kLineA2RatMaxThreshold_ ||
                         std::abs(da1) > kLineA1RatMaxThreshold_ ||
                         std::abs(da0) > kLineA0RatMaxThreshold_) &&
                        quality && quality_last;
  return line_terrible_debounce_.DealDebounce(!lineterriblebl);
}

// 车道线状态判断
void LaneLineMarkerDetection::ODDLaneLineCondition(bool left_q, bool right_q) {
  if (left_q && right_q) {
    lanelinecase_ = BothGood;
  } else if (!left_q && right_q) {
    lanelinecase_ = RightGood;
  } else if (left_q && !right_q) {
    lanelinecase_ = LeftGood;
  } else {
    lanelinecase_ = NoneLine;
  }
}

// 车道宽度计算
double LaneLineMarkerDetection::ODDLaneWidthCalc(
    double vehicle_width, double left_a0, double left_a1, bool left_quality,
    double right_ao, double right_a1, bool right_quality) {
  switch (lanelinecase_) {
    case NoneLine:
      return 3.5;
      break;
    case LeftGood:
      return LdwCreater::SaturationDynamicLimit(
          std::abs(left_a0) + 0.5 * vehicle_width, kLdwCorridorWidthLowThd,
          kLdwCorridorWidthHighThd);
      break;
    case RightGood:
      return LdwCreater::SaturationDynamicLimit(
          std::abs(right_ao) + 0.5 * vehicle_width, kLdwCorridorWidthLowThd,
          kLdwCorridorWidthHighThd);
      break;
    case BothGood:
      if (left_quality && right_quality) {
        return std::abs(std::cos(std::atan(left_a1)) * left_a0) +
               std::abs(std::cos(std::atan(right_a1)) * right_ao);
      } else {
        return 3.5;
      }
      break;
    default:
      return 3.5;
      break;
  }
}

// 车道宽度判断
bool LaneLineMarkerDetection::ODDCorridorWidthDection(double ego_lane_width) {
  bool odd_max_corridor_width_condition = false;
  bool odd_min_corridor_width_condition = false;
  if (ego_lane_width > kOddDisenableMaxCoridorWidthThreshold) {
    odd_max_corridor_width_condition = false;
  } else if (ego_lane_width < kOddEnbleMaxCoridorWidthThreshold) {
    odd_max_corridor_width_condition = true;
  } else {
    odd_max_corridor_width_condition = odd_max_corridor_width_condition_last_;
  }
  if (ego_lane_width > kOddEnableMinCoridorWidthThreshold) {
    odd_min_corridor_width_condition = true;
  } else if (ego_lane_width < kOddDisenableMinCoridorWidthThreshold) {
    odd_min_corridor_width_condition = false;
  } else {
    odd_min_corridor_width_condition = odd_min_corridor_width_condition_last_;
  }
  return line_width_high_debounce_.DealDebounce(
             odd_max_corridor_width_condition) &&
         line_width_low_debounce_.DealDebounce(
             odd_min_corridor_width_condition);
}

// 车辆heading判断
bool LaneLineMarkerDetection::VehicleHeadingDection(double a1, bool quality) {
  return quality &&
         (std::abs(a1) <=
          navi_hdmap_config_.lanemarker_decider_config().max_heading_err_rad());
}

// 车道线延时
void LaneLineMarkerDetection::ODDLaneLineDelay(
    const std::shared_ptr<const perception::LaneMarkers>& lane_marker) {
  lane_markers_last_ = *lane_marker;
  left_look_ahead_curvature_last_ = LookAheadCurvature(
      lane_marker->front_left_lane_marker().c2_curvature(),
      lane_marker->front_left_lane_marker().c3_curvature_derivative());
  right_look_ahead_curvature_last_ = LookAheadCurvature(
      lane_marker->front_right_lane_marker().c2_curvature(),
      lane_marker->front_right_lane_marker().c3_curvature_derivative());
}

//  车道线判断main function
uint16_t LaneLineMarkerDetection::ODDLaneLineProcess(
    const std::shared_ptr<LocalView>& local_view) {
  // const auto laneline = perception_obstaclesLine_->lane_marker();
  // const auto laneline = *lane_marker;
  if (local_view == nullptr || !local_view->HasLaneMarkers()) {
    return 0;
  }
  const auto& lane_marker = local_view->GetLaneMarkers();
  uint16_t odd_indx_lanemarker_detection_cond_u16 = 0;

  bool ldp_left_quality_cond = LdwCreater::IsGoodPath(
      lane_marker->front_left_lane_marker().quality(), 0.5);
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(ldp_left_quality_cond) << 1;

  bool ldp_right_quality_cond = LdwCreater::IsGoodPath(
      lane_marker->front_right_lane_marker().quality(), 0.5);
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(ldp_right_quality_cond) << 9;

  ODDLaneLineCondition(ldp_left_quality_cond, ldp_right_quality_cond);

  bool left_crd_length_cond = ODDCorridorLengthDetection(
      lane_marker->front_left_lane_marker().longitude_start(),
      lane_marker->front_left_lane_marker().longitude_end());
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(left_crd_length_cond) << 0;

  bool right_crd_length_cond = ODDCorridorLengthDetection(
      lane_marker->front_right_lane_marker().longitude_start(),
      lane_marker->front_right_lane_marker().longitude_end());
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(right_crd_length_cond) << 8;

  bool left_quality_cond = LdwCreater::IsGoodPath(
      lane_marker->front_left_lane_marker().quality(), 0.3);
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(left_quality_cond) << 6;

  bool right_quality_cond = LdwCreater::IsGoodPath(
      lane_marker->front_right_lane_marker().quality(), 0.3);
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(right_quality_cond) << 13;

  bool left_quality_last = LdwCreater::IsGoodPath(
      lane_markers_last_.front_left_lane_marker().quality(), 0.3);
  bool right_quality_last = LdwCreater::IsGoodPath(
      lane_markers_last_.front_right_lane_marker().quality(), 0.3);
  bool left_curvature_cond = ODDLaneLineCurvatureDetection(
      lane_marker->front_left_lane_marker().c2_curvature());
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(left_curvature_cond) << 2;

  bool right_curvature_cond = ODDLaneLineCurvatureDetection(
      lane_marker->front_right_lane_marker().c2_curvature());
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(right_curvature_cond) << 10;

  bool left_ride_line_cond = LeftODDRideLineDetection(
      vehicle_paramline_.width(),
      lane_marker->front_left_lane_marker().c0_position(), left_quality_cond,
      lane_marker->is_lanechange_to_left());
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(!left_ride_line_cond) << 4;

  bool right_ride_line_cond = RightODDRideLineDetection(
      vehicle_paramline_.width(),
      lane_marker->front_right_lane_marker().c0_position(), right_quality_cond,
      lane_marker->is_lanechange_to_right());
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(!right_ride_line_cond) << 12;

  double left_lookahead_curvature_curr = LookAheadCurvature(
      lane_marker->front_left_lane_marker().c2_curvature(),
      lane_marker->front_left_lane_marker().c3_curvature_derivative());
  double right_lookahead_curvature_curr = LookAheadCurvature(
      lane_marker->front_right_lane_marker().c2_curvature(),
      lane_marker->front_right_lane_marker().c3_curvature_derivative());
  bool left_linepos_a0_big_change = ODDLaneLineParamJump(
      left_lookahead_curvature_curr, left_look_ahead_curvature_last_,
      lane_marker->front_left_lane_marker().c1_heading_angle(),
      lane_markers_last_.front_left_lane_marker().c1_heading_angle(),
      lane_marker->front_left_lane_marker().c0_position(),
      lane_markers_last_.front_left_lane_marker().c0_position(),
      left_quality_cond, left_quality_last);
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(!left_linepos_a0_big_change) << 3;

  bool right_linepos_a0_big_change = ODDLaneLineParamJump(
      right_lookahead_curvature_curr, right_look_ahead_curvature_last_,
      lane_marker->front_right_lane_marker().c1_heading_angle(),
      lane_markers_last_.front_right_lane_marker().c1_heading_angle(),
      lane_marker->front_right_lane_marker().c0_position(),
      lane_markers_last_.front_right_lane_marker().c0_position(),
      right_quality_cond, right_quality_last);
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(!right_linepos_a0_big_change) << 11;

  bool ldp_corridor_width_cond = ODDCorridorWidthDection(ODDLaneWidthCalc(
      vehicle_paramline_.width(),
      lane_marker->front_left_lane_marker().c0_position(),
      lane_marker->front_left_lane_marker().c1_heading_angle(),
      left_quality_cond, lane_marker->front_right_lane_marker().c0_position(),
      lane_marker->front_right_lane_marker().c1_heading_angle(),
      right_quality_cond));
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(ldp_corridor_width_cond) << 5;

  bool ldw_corridor_width_cond = ldp_corridor_width_cond;
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(ldw_corridor_width_cond) << 14;

  bool left_line_heading_cond = VehicleHeadingDection(
      lane_marker->front_left_lane_marker().c1_heading_angle(),
      ldp_left_quality_cond);
  bool right_line_heading_cond = VehicleHeadingDection(
      lane_marker->front_right_lane_marker().c1_heading_angle(),
      ldp_right_quality_cond);

  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(left_line_heading_cond) << 7;
  odd_indx_lanemarker_detection_cond_u16 |=
      static_cast<unsigned int>(right_line_heading_cond) << 15;

  ODDLaneLineDelay(lane_marker);
  ADEBUG << "====================lanemarker:"
         << odd_indx_lanemarker_detection_cond_u16;
  return odd_indx_lanemarker_detection_cond_u16;
}
}  // namespace planning
}  // namespace TL
