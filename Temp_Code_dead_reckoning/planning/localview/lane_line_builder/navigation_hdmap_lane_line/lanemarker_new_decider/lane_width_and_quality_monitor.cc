/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_width_and_quality_monitor.h"

namespace TL {
namespace planning {
namespace lanelineprocess {

LaneWidthQualityMonitor::LaneWidthQualityMonitor(
    const planning::PerceptionMapConfig& config)
    : config_(config) {}

void LaneWidthQualityMonitor::SetConf(const double width_var_thd) {
  lane_width_var_thd_ = width_var_thd;
  lane_a0_coff_var_thd_ = config_.reset_quality_config().lane_a0_coff_varthd();
  lane_a1_coff_var_thd_ = config_.reset_quality_config().lane_a1_coff_varthd();
  lane_a2_coff_var_thd_ = config_.reset_quality_config().lane_a2_coff_varthd();
  main_loop_time_ = config_.main_loop_time();
  lane_width_rise_time_ = config_.lanewidth_predictor_config().rise_time();
  lane_width_fall_time_ = config_.lanewidth_predictor_config().fall_time();
  lane_width_delta_distance_ =
      config_.lanewidth_predictor_config().lane_width_delta_distance();
  lane_width_max_thd_ = 2 * config_.max_lane_half_width();
  lane_width_min_thd_ = 2 * config_.min_lane_half_width();
  debounce_module_.ResetTime(lane_width_fall_time_, lane_width_rise_time_,
                             main_loop_time_);
  good_quality_threshold_ = config_.lanemarker_quality_not_reliable_value();
  batter_quality_threshold_ =
      config_.lanemarker_quality_reliableforwarning_value();
}

void LaneWidthQualityMonitor::DealCommonData(
    const double lane_coast_time, const double max_predict_width_distance) {
  lane_coast_time_ = lane_coast_time;
  max_predict_width_distance_ = max_predict_width_distance;
}

bool LaneWidthQualityMonitor::Update(const LaneMarker& origin_lanemarker,
                                     const LaneMarker& side_lanemarker,
                                     const int lane_width_predict_valid,
                                     const double lane_width_predict_sg,
                                     const bool lane_change,
                                     LaneSplit* const lane_split_debug,
                                     const LaneMarkerState lanemarker_state) {
  UNUSED(lane_width_predict_valid);
  const double a0 = origin_lanemarker.c0_position();
  const double a1 = origin_lanemarker.c1_heading_angle();
  const double a2 = origin_lanemarker.c2_curvature();
  const double ave_a0 = origin_lanemarker_a0_delay_.GetAverageValue();
  const double ave_a1 = origin_lanemarker_a1_delay_.GetAverageValue();
  const double ave_a2 = origin_lanemarker_a2_delay_.GetAverageValue();
  // 质量好，又不在换道状态，如果系数的方差大于阈值说明系数不可靠
  origin_lanemarker_quality_.Deal(origin_lanemarker.quality());
  const bool is_width_coff_terrible =
      (std::pow((ave_a0 - a0), 2) > lane_a0_coff_var_thd_ ||
       std::pow((ave_a1 - a1), 2) > lane_a1_coff_var_thd_ ||
       std::pow((ave_a2 - a2), 2) > lane_a2_coff_var_thd_) &&
      origin_lanemarker.quality() > good_quality_threshold_ &&
      origin_lanemarker_quality_.GetDelay(4) > good_quality_threshold_ &&
      !lane_change;
  // AERROR << "a0: " << a0 << ", ave_a0: " << ave_a0 << ", a1: " << a1 << ",
  // ave_a1: " << ave_a1 << ", a2: "
  //         << a2 << ", ave_a2: " << ave_a2;
  // AERROR << "a0_delay,0: " << origin_lanemarker_a0_delay_.GetDelay(0) << ",
  // 1: " << origin_lanemarker_a0_delay_.GetDelay(1) << ", 2: "
  //     << origin_lanemarker_a0_delay_.GetDelay(2) << ", 3: " <<
  //     origin_lanemarker_a0_delay_.GetDelay(3) << ", 4: "<<
  //     origin_lanemarker_a0_delay_.GetDelay(4);
  // AERROR << "a1_delay,0: " << origin_lanemarker_a1_delay_.GetDelay(0) << ",
  // 1: " << origin_lanemarker_a1_delay_.GetDelay(1) << ", 2: "
  //     << origin_lanemarker_a1_delay_.GetDelay(2) << ", 3: " <<
  //     origin_lanemarker_a1_delay_.GetDelay(3) << ", 4: "<<
  //     origin_lanemarker_a1_delay_.GetDelay(4);
  origin_lanemarker_a0_delay_.Deal(a0);
  origin_lanemarker_a1_delay_.Deal(a1);
  origin_lanemarker_a2_delay_.Deal(a2);
  side_lanemarker_quality_.Deal(side_lanemarker.quality());
  const bool is_lane_width_terrible = LaneWidthMonitor(
      origin_lanemarker, side_lanemarker, lane_width_predict_sg);
  lane_split_debug->set_is_lane_width_terrible(is_lane_width_terrible);
  const bool is_quality_coff_terrible =
      origin_lanemarker_quality_.GetDelay(1) > batter_quality_threshold_ &&
      (origin_lanemarker_quality_.GetDelay(0) > good_quality_threshold_ &&
       origin_lanemarker_quality_.GetDelay(0) < batter_quality_threshold_);
  const bool width_split = LaneWidthCoffMonitor(
      &lane_wde_coff_monitor_state_, &coff_monitor_time_,
      is_width_coff_terrible, lane_coast_time_, is_lane_width_terrible);
  lane_split_debug->set_width_split(width_split);
  auto* width_state = lane_split_debug->mutable_width_state();
  width_state->set_state_name("width_state");
  width_state->set_now_state(lane_wde_coff_monitor_state_ ? 1 : 0);
  width_state->set_time(coff_monitor_time_);
  width_state->add_threshold(lane_coast_time_);
  width_state->mutable_input_one()->set_name("is_width_coff_terrible");
  width_state->mutable_input_one()->set_value_b(is_width_coff_terrible);

  bool lanemarker_state_split =
      (origin_lanemarker.quality() < good_quality_threshold_) &&
      (lanemarker_state != LaneMarkerState::BAD_LANEMARKER);
  const bool quality_split =
      LaneWidthqualityMonitor(&lane_wde_quality_monitor_state_,
                              &quality_monitor_time_, is_quality_coff_terrible,
                              lane_coast_time_, origin_lanemarker.quality(),
                              lane_change) ||
      lanemarker_state_split;

  lane_split_debug->set_quality_split(quality_split);
  auto* quality_state = lane_split_debug->mutable_quality_state();
  quality_state->set_state_name("quality_state");
  quality_state->set_now_state(lane_wde_quality_monitor_state_ ? 1 : 0);
  quality_state->set_time(quality_monitor_time_);
  quality_state->mutable_input_one()->set_name("is_quality_coff_terrible");
  quality_state->mutable_input_one()->set_value_b(is_quality_coff_terrible);
  ADEBUG << "is_width_coff_terrible: " << is_width_coff_terrible
         << ", is_lane_width_terrible: " << is_lane_width_terrible
         << ", is_quality_coff_terrible: " << is_quality_coff_terrible
         << ", width_split: " << width_split
         << ", quality_split: " << quality_split;
  return width_split || quality_split;
}

// 再次计算车道宽度是否有效
bool LaneWidthQualityMonitor::LaneWidthMonitor(
    const LaneMarker& origin_lanemarker, const LaneMarker& side_lanemarker,
    const double lane_width_predict_sg) {
  const bool is_lane_reliable =
      origin_lanemarker_quality_.GetDelay(0) > good_quality_threshold_ &&
      origin_lanemarker_quality_.GetDelay(4) > good_quality_threshold_ &&
      side_lanemarker_quality_.GetDelay(0) > good_quality_threshold_ &&
      side_lanemarker_quality_.GetDelay(4) > good_quality_threshold_;
  const double lane_width_delta =
      CalculateLanemarkerY(max_predict_width_distance_, origin_lanemarker) -
      CalculateLanemarkerY(max_predict_width_distance_, side_lanemarker);
  const double predict_lane_width = std::fmax(lane_width_delta, 0.0);
  const bool is_predict_width_change_terrible =
      is_lane_reliable &&
      (std::pow((lane_wide_delta_delay_.GetAverageValue() - predict_lane_width),
                2) > lane_width_var_thd_);
  const bool is_pre_lane_variance_terrible =
      debounce_module_.DealDebounce(is_predict_width_change_terrible);
  const double now_lane_width = std::fabs(origin_lanemarker.c0_position() -
                                          side_lanemarker.c0_position());
  const bool is_pre_lane_normal =
      std::abs(lane_width_predict_sg - now_lane_width) <
      lane_width_delta_distance_;
  ADEBUG << "is_pre_lane_normal: " << is_pre_lane_normal
         << "lane_width_predict_sg: " << lane_width_predict_sg
         << ", now_lane_width:" << now_lane_width;
  const bool is_lane_wide_terrible = now_lane_width > lane_width_max_thd_ ||
                                     now_lane_width < lane_width_min_thd_;
  return LaneWdeTooMonitorSM(&lane_wde_too_monitor_sm_state_,
                             is_pre_lane_variance_terrible,
                             is_lane_wide_terrible) ||
         LaneWdeTooNarrowMonitorSM(&lane_wde_too_narrow_monitor_sm_state_,
                                   is_pre_lane_variance_terrible,
                                   is_lane_reliable, is_pre_lane_normal);
}

double LaneWidthQualityMonitor::CalculateLanemarkerY(
    const double distance, const LaneMarker& lane_marker) {
  return lane_marker.c0_position() + lane_marker.c1_heading_angle() * distance +
         lane_marker.c2_curvature() * std::pow(distance, 2) +
         lane_marker.c3_curvature_derivative() * std::pow(distance, 3);
}

bool LaneWidthQualityMonitor::LaneWdeTooMonitorSM(
    bool* state, const bool lane_wide_variance_terrible,
    const bool lane_wide_terrible) {
  bool predict_lane_wide_terrible_bl = false;
  if (*state) {
    if (lane_wide_variance_terrible && lane_wide_terrible) {
      *state = false;
      predict_lane_wide_terrible_bl = true;
    } else {
      predict_lane_wide_terrible_bl = false;
    }
  } else {
    if (!lane_wide_terrible) {
      *state = true;
      predict_lane_wide_terrible_bl = false;
    } else {
      predict_lane_wide_terrible_bl = true;
    }
  }
  return predict_lane_wide_terrible_bl;
}

bool LaneWidthQualityMonitor::LaneWdeTooNarrowMonitorSM(
    bool* state, const bool is_prelne_wde_variance_terrible,
    const bool is_lane_reliable, const bool is_prelne_wde_normal) {
  bool predict_lane_wide_terrible_bl = false;
  if (*state) {
    if (is_prelne_wde_variance_terrible && !is_prelne_wde_normal) {
      *state = false;
      predict_lane_wide_terrible_bl = true;
    } else {
      predict_lane_wide_terrible_bl = false;
    }
  } else {
    if (!is_lane_reliable || is_prelne_wde_normal) {
      *state = true;
      predict_lane_wide_terrible_bl = false;
    } else {
      predict_lane_wide_terrible_bl = true;
    }
  }
  return predict_lane_wide_terrible_bl;
}

}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
