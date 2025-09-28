/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lanemarker_change_decider.h"

#include <utility>

#include "common/math/math_utils.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
using TL::common::math::Clamp;

LaneChangeDecider::LaneChangeDecider(
    const planning::PerceptionMapConfig& config)
    : config_(config) {}

Status LaneChangeDecider::Init() {
  left_a0_delay_ = Delay<double>(kMagicNumber5);
  right_a0_delay_ = Delay<double>(kMagicNumber5);
  left_coff_monitor_inside_ = std::make_tuple(true, 0.0, 0);
  right_coff_monitor_inside_ = std::make_tuple(true, 0.0, 0);
  left_lanewde_inside_ = std::make_pair(false, 0.0);
  right_lanewde_inside_ = std::make_pair(false, 0.0);
  next_left_lane_width_predict_sg_delay_ = Delay<double>(3);
  next_left_lane_width_predict_valid_delay_ = Delay<int>(3);
  next_right_lane_width_predict_sg_delay_ = Delay<double>(3);
  next_right_lane_width_predict_valid_delay_ = Delay<int>(3);
  LoadLaneChangeGainScheduler(config_.lanechange_decider_config());
  main_loop_time_ = config_.main_loop_time();
  next_lane_width_hldtm_ =
      config_.lanechange_decider_config().next_lane_width_hldtm();
  default_lane_width_ =
      config_.default_left_width() + config_.default_right_width();
  good_quality_threshold_ = config_.lanemarker_quality_not_reliable_value();
  return Status::OK();
}

void LaneChangeDecider::LoadLaneChangeGainScheduler(
    const planning::PerceptionMapLaneChangeDeciderConfig& lane_change_conf) {
  const auto& lane_change_gain_scheduler =
      lane_change_conf.lane_change_gain_scheduler();
  AINFO << "distance gain scheduler loaded";
  Interpolation1D::DataType xy1{};
  Interpolation1D::DataType xy2{};
  for (const auto& scheduler : lane_change_gain_scheduler.scheduler()) {
    xy1.emplace_back(std::make_pair(scheduler.length(), scheduler.ratio()));
  }
  // lane_change_interpolation_.reset(new Interpolation1D);
  lane_change_interpolation_ = std::make_unique<Interpolation1D>();
  ACHECK(lane_change_interpolation_->Init(xy1))
      << "Fail to load lane change gain scheduler";
}

void LaneChangeDecider::ComputerCoastTime(DeciderData* decider_data) {
  const double v_spd = decider_data->filter_vehicle_state.vehicle_speed_average;
  decider_data->lane_speed_cost_time =
      lane_change_interpolation_->Interpolate(v_spd);
}

void LaneChangeDecider::GetLeftChangeFlag(DeciderData* decider_data) {
  ADEBUG << "GetLeftChangeCoff Process!"
         << decider_data->original_lanemarkers.header().seq();
  const double lanechange_a0_varthd =
      config_.lanechange_decider_config().lanechange_a0_varthd();
  const bool lanechange_flag = decider_data->is_lanechange_to_left;
  int next_left_lane_width_predict_valid =
      decider_data->lane_width_valid_out.next_leftlane_widthpredict_valid;
  double next_left_lane_width_predict_sg =
      decider_data->lane_width_valid_out.next_leftlane_widthpredict_sg;
  const auto& host_right_lanemarker =
      decider_data->trans_lanemarkers.front_right_lane_marker();
  const auto& host_left_lanemarker =
      decider_data->copy_lanemarkers.front_left_lane_marker();
  auto* left_change_debug =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lane_change_info()
          ->add_lane_change();
  left_change_debug->set_name("left_change_info");
  left_lanemarker_delay_.Deal(host_left_lanemarker);
  right_lanemarker_delay_.Deal(host_right_lanemarker);

  double pos_a0 = host_left_lanemarker.c0_position();
  double lane_wide_measure = pos_a0 - host_right_lanemarker.c0_position();
  const double lane_cnfd = host_left_lanemarker.quality();
  const double lane_coast_time = decider_data->lane_speed_cost_time;
  next_left_lane_width_predict_sg_delay_.Deal(next_left_lane_width_predict_sg);
  next_left_lane_width_predict_valid_delay_.Deal(
      next_left_lane_width_predict_valid);
  if (!host_lane_change_to_left_delay_.Deal(lanechange_flag) &&
      lanechange_flag) {
    last_next_left_lanewdepre_sg_ =
        next_left_lane_width_predict_sg_delay_.GetDelay(2);
    last_next_left_lanewdepre_valid_ =
        next_left_lane_width_predict_valid_delay_.GetDelay(2);
  }
  // 换道的时候，在cost time时间内使用历史的valid和sg
  LaneWidePredictSM(
      &left_lanewde_inside_, lanechange_flag,
      &next_left_lane_width_predict_valid, last_next_left_lanewdepre_valid_,
      &next_left_lane_width_predict_sg, last_next_left_lanewdepre_sg_);
  auto* wide_state = left_change_debug->mutable_width_state_info();
  wide_state->set_now_state(std::get<0>(left_lanewde_inside_) ? 1 : 0);
  wide_state->set_time(std::get<1>(left_lanewde_inside_));
  wide_state->mutable_input_one()->set_name("last_next_left_lanewdepre_sg");
  wide_state->mutable_input_one()->set_value_d(last_next_left_lanewdepre_sg_);

  const bool is_lane_wide_terrible =
      (next_left_lane_width_predict_valid == 1 ||
       next_left_lane_width_predict_valid == 2) &&
      (std::fabs(lane_wide_measure - next_left_lane_width_predict_sg) >
       good_quality_threshold_);
  const bool is_coff_terrible =
      std::pow((left_a0_delay_.GetAverageValue() - pos_a0), 2) >
      lanechange_a0_varthd;
  left_a0_delay_.Deal(pos_a0);
  // 根据车道宽度变化和系数c0的变化以及换道标志判断该标志位，如果系数c0变化较大，且
  // 宽度有效，但是宽度变化较大，且处于换道中，则该标志位true；只要大于一定时间或者
  // 车辆宽度可靠车道线质量较好，则标志为false
  decider_data->lane_change_out.left_lanechange_mnt_flag =
      LaneChangeCoffMonitorSM(&left_coff_monitor_inside_, is_coff_terrible,
                              is_lane_wide_terrible, lanechange_flag, lane_cnfd,
                              lane_coast_time);
  auto* change_state = left_change_debug->mutable_change_state_info();
  change_state->set_now_state(std::get<0>(left_coff_monitor_inside_) ? 1 : 0);
  change_state->set_time(std::get<1>(left_coff_monitor_inside_));
  change_state->set_count(std::get<2>(left_coff_monitor_inside_));
  change_state->add_threshold(lane_coast_time);
  left_change_debug->set_is_lane_wide_terrible(is_lane_wide_terrible);
  left_change_debug->set_is_coff_terrible(is_coff_terrible);

  if ((next_left_lane_width_predict_valid == 1 ||
       next_left_lane_width_predict_valid == 2) &&
      std::get<2>(left_coff_monitor_inside_) == 1) {
    predict_host_left_pos_a0_ =
        next_left_lane_width_predict_sg + host_right_lanemarker.c0_position();
  } else {
    predict_host_left_pos_a0_ =
        default_lane_width_ + host_right_lanemarker.c0_position();
  }
  left_change_debug->set_predict_pos_a0(predict_host_left_pos_a0_);
  ADEBUG << "left_lanechange_mnt_flag: "
         << decider_data->lane_change_out.left_lanechange_mnt_flag;
}

void LaneChangeDecider::GetLeftChangeCoff(DeciderData* decider_data) {
  ADEBUG << "GetLeftChangeCoff Process!"
         << decider_data->original_lanemarkers.header().seq();
  std::vector<Vec2d> coff_fix;
  bool using_filter_points = false;
  if (std::get<2>(left_coff_monitor_inside_) == 0) {
    coff_fix.emplace_back(0.0, 0.0);
  } else if (std::get<2>(left_coff_monitor_inside_) == 1) {
    CreatInitLaneMarkerPoints(left_lanemarker_delay_.GetDelay(2),
                              predict_host_left_pos_a0_, &coff_fix);
  } else {
    using_filter_points = true;
  }
  if (decider_data->lane_change_out.left_lanechange_mnt_flag &&
      !using_filter_points) {
    decider_data->lane_marker_points["left"].swap(coff_fix);
  }
  auto* lane_changes =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lane_change_info()
          ->mutable_lane_change();
  if (!lane_changes->empty()) {
    lane_changes->begin()->set_using_filter_points(using_filter_points);
  }
  ADEBUG << "left using_filter_points: " << using_filter_points;
}

void LaneChangeDecider::GetRightChangeFlag(DeciderData* decider_data) {
  ADEBUG << "GetRightChangeFlag Process!"
         << decider_data->original_lanemarkers.header().seq();
  const double lanechange_a0_varthd =
      config_.lanechange_decider_config().lanechange_a0_varthd();
  const bool lanechange_flag = decider_data->is_lanechange_to_right;
  int next_right_lane_width_predict_valid =
      decider_data->lane_width_valid_out.next_rightlane_widthpredict_valid;
  double next_right_lane_width_predict_sg =
      decider_data->lane_width_valid_out.next_rightlane_widthpredict_sg;
  const LaneMarker host_right_lanemarker =
      decider_data->copy_lanemarkers.front_right_lane_marker();
  const LaneMarker host_left_lanemarker =
      decider_data->trans_lanemarkers.front_left_lane_marker();
  auto* right_change_debug =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lane_change_info()
          ->add_lane_change();
  right_change_debug->set_name("right_change_info");
  double pos_a0 = host_left_lanemarker.c0_position();
  double lane_wide_measure = pos_a0 - host_right_lanemarker.c0_position();
  const double lane_cnfd = host_left_lanemarker.quality();
  const double lane_coast_time = decider_data->lane_speed_cost_time;
  next_right_lane_width_predict_sg_delay_.Deal(
      next_right_lane_width_predict_sg);
  next_right_lane_width_predict_valid_delay_.Deal(
      next_right_lane_width_predict_valid);
  host_lane_change_to_right_delay_.Deal(lanechange_flag);
  ADEBUG << "lanechange_flag: " << lanechange_flag << ", last_lanechange_flag: "
         << host_lane_change_to_right_delay_.GetDelay(1);
  if (!host_lane_change_to_right_delay_.GetDelay(1) && lanechange_flag) {
    last_next_right_lanewdepre_sg_ =
        next_right_lane_width_predict_sg_delay_.GetDelay(2);
    last_next_right_lanewdepre_valid_ =
        next_right_lane_width_predict_valid_delay_.GetDelay(2);
  }
  LaneWidePredictSM(
      &right_lanewde_inside_, lanechange_flag,
      &next_right_lane_width_predict_valid, last_next_right_lanewdepre_valid_,
      &next_right_lane_width_predict_sg, last_next_right_lanewdepre_sg_);
  auto* wide_state = right_change_debug->mutable_width_state_info();
  wide_state->set_now_state(std::get<0>(right_lanewde_inside_) ? 1 : 0);
  wide_state->set_time(std::get<1>(right_lanewde_inside_));
  wide_state->mutable_input_one()->set_name("last_next_right_lanewdepre_sg");
  wide_state->mutable_input_one()->set_value_d(last_next_right_lanewdepre_sg_);
  const bool is_lane_wide_terrible =
      (next_right_lane_width_predict_valid == 1 ||
       next_right_lane_width_predict_valid == 2) &&
      (std::fabs(lane_wide_measure - next_right_lane_width_predict_sg) >
       good_quality_threshold_);
  const bool is_coff_terrible =
      std::pow((right_a0_delay_.GetAverageValue() - pos_a0), 2) >
      lanechange_a0_varthd;
  ADEBUG << "a0_average: " << right_a0_delay_.GetAverageValue()
         << ", pos_a0: " << pos_a0;
  right_a0_delay_.Deal(pos_a0);
  ADEBUG << "out_next_right_lane_width_predict_valid: "
         << next_right_lane_width_predict_valid
         << ", out_next_right_lane_width_predict_sg: "
         << next_right_lane_width_predict_sg
         << ", is_lane_wide_terrible: " << is_lane_wide_terrible
         << ", is_coff_terrible: " << is_coff_terrible
         << ", lanechange_a0_varthd: " << lanechange_a0_varthd;
  decider_data->lane_change_out.right_lanechange_mnt_flag =
      LaneChangeCoffMonitorSM(&right_coff_monitor_inside_, is_coff_terrible,
                              is_lane_wide_terrible, lanechange_flag, lane_cnfd,
                              lane_coast_time);
  auto* change_state = right_change_debug->mutable_change_state_info();
  change_state->set_now_state(std::get<0>(right_coff_monitor_inside_) ? 1 : 0);
  change_state->set_time(std::get<1>(right_coff_monitor_inside_));
  change_state->set_count(std::get<2>(right_coff_monitor_inside_));
  change_state->add_threshold(lane_coast_time);
  right_change_debug->set_is_lane_wide_terrible(is_lane_wide_terrible);
  right_change_debug->set_is_coff_terrible(is_coff_terrible);
  if ((next_right_lane_width_predict_valid == 1 ||
       next_right_lane_width_predict_valid == 2) &&
      std::get<2>(left_coff_monitor_inside_) == 1) {
    predict_host_right_pos_a0_ =
        host_left_lanemarker.c0_position() - next_right_lane_width_predict_sg;
  } else {
    predict_host_right_pos_a0_ =
        host_left_lanemarker.c0_position() - default_lane_width_;
  }
  right_change_debug->set_predict_pos_a0(predict_host_right_pos_a0_);
  ADEBUG << "right_lanechange_mnt_flag: "
         << decider_data->lane_change_out.right_lanechange_mnt_flag
         << ", right_monitor_state: "
         << std::get<0>(right_coff_monitor_inside_);
}

void LaneChangeDecider::GetRightChangeCoff(DeciderData* decider_data) {
  ADEBUG << "GetRightChangeCoff Process!"
         << decider_data->original_lanemarkers.header().seq();
  std::vector<Vec2d> coff_fix;
  bool using_filter_points = false;
  if (std::get<2>(right_coff_monitor_inside_) == 0) {
    coff_fix.emplace_back(0.0, 0.0);
  } else if (std::get<2>(right_coff_monitor_inside_) == 1) {
    CreatInitLaneMarkerPoints(right_lanemarker_delay_.GetDelay(2),
                              predict_host_right_pos_a0_, &coff_fix);
  } else {
    using_filter_points = true;
  }
  if (decider_data->lane_change_out.right_lanechange_mnt_flag &&
      !using_filter_points) {
    decider_data->lane_marker_points["right"].swap(coff_fix);
  }
  auto* lane_changes =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lane_change_info()
          ->mutable_lane_change();
  if (lane_changes->size() > 1) {
    lane_changes->at(1).set_using_filter_points(using_filter_points);
  }
  ADEBUG << "right using_filter_points: " << using_filter_points;
}

void LaneChangeDecider::LaneWidePredictSM(
    std::pair<bool, double>* inside_value, const bool lane_change_flag,
    int* const next_lane_width_predict_flag,
    const int next_lane_width_predict_last_flag,
    double* const next_lane_predict_width,
    const double next_lane_predict_width_last) const {
  // inside_value   0:state 1:time
  if (std::get<0>(*inside_value)) {
    if (std::get<1>(*inside_value) > next_lane_width_hldtm_) {
      std::get<0>(*inside_value) = false;
      std::get<1>(*inside_value) = 0.0;
    } else {
      std::get<1>(*inside_value) += main_loop_time_;
      *next_lane_width_predict_flag = next_lane_width_predict_last_flag;
      *next_lane_predict_width = next_lane_predict_width_last;
    }
  } else {
    if (lane_change_flag) {
      std::get<1>(*inside_value) = 0.0;
      std::get<0>(*inside_value) = true;
      *next_lane_width_predict_flag = next_lane_width_predict_last_flag;
      *next_lane_predict_width = next_lane_predict_width_last;
    } else {
      std::get<1>(*inside_value) = 0.0;
    }
  }
}

bool LaneChangeDecider::LaneChangeCoffMonitorSM(
    std::tuple<bool, double, int>* const inside_valus,
    const bool is_coff_terrible, const bool lane_wide_terrible,
    const bool lanechange_flag, const double lane_quality,
    const double lane_coast_time) {
  // inside_valus:0.state, 1:time, 2:count
  bool flag = false;
  if (std::get<0>(*inside_valus)) {
    if (is_coff_terrible && lane_wide_terrible && lanechange_flag) {
      std::get<0>(*inside_valus) = false;
      std::get<1>(*inside_valus) = 0.0;
      flag = true;
      std::get<2>(*inside_valus) = 1;
    } else {
      std::get<1>(*inside_valus) = 0.0;
      flag = false;
      std::get<2>(*inside_valus) = 0;
    }
  } else {
    if (std::get<1>(*inside_valus) > lane_coast_time ||
        (!lane_wide_terrible &&
         lane_quality >
             config_.lanemarker_quality_reliableforwarning_value())) {
      std::get<0>(*inside_valus) = true;
      std::get<1>(*inside_valus) = 0.0;
      flag = false;
      std::get<2>(*inside_valus) = 0;
    } else {
      std::get<1>(*inside_valus) += main_loop_time_;
      std::get<2>(*inside_valus)++;
      flag = true;
    }
  }
  return flag;
}

void LaneChangeDecider::CreatInitLaneMarkerPoints(
    const LaneMarker& input_lanemarker, const double pos_a0,
    std::vector<Vec2d>* const init_points) {
  init_points->clear();
  const double lanemarker_size =
      Clamp(input_lanemarker.view_range(),
            config_.lanemarker_filter_config().low_viewrange(),
            config_.lanemarker_filter_config().upper_viewrange());
  const double back_start = config_.lanemarker_back_length();
  const double step = config_.lanemarker_filter_config().step_distance();
  int max_index = floor((back_start + lanemarker_size) / step + 1);
  for (int i = 0; i < max_index; i++) {
    double x = i * step - back_start;
    double y = pos_a0 + input_lanemarker.c1_heading_angle() * x +
               input_lanemarker.c2_curvature() * x * x +
               input_lanemarker.c3_curvature_derivative() * x * x * x;
    init_points->push_back(Vec2d(x, y));
  }
  ADEBUG << "Init lanemarker points size: " << init_points->size()
         << ",start_point_x: " << init_points->front().x()
         << ", start_point_y: " << init_points->front().y()
         << "; end_point_x: " << init_points->back().x()
         << ", end_point_y: " << init_points->back().y();
}
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
