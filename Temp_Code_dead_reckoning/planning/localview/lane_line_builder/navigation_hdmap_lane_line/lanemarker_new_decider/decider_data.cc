/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "common/math/double_type.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
using TL::common::math::double_type::Compare;

double LowPassFilter::Filter(const double x_insert) {
  if (is_first_filter_) {
    last_x_insert_ = x_insert;
    last_y_output_ = x_insert;
    is_first_filter_ = false;
  }
  const double a0 = (pi * fc_) / (pi * fc_ + fs_);
  const double a1 = (pi * fc_) / (pi * fc_ + fs_);
  const double b1 = (pi * fc_ - fs_) / (pi * fc_ + fs_);
  const double y_out =
      (a0 * x_insert + a1 * last_x_insert_) - b1 * last_y_output_;
  last_x_insert_ = x_insert;
  last_y_output_ = y_out;
  return y_out;
}

void LowPassFilter::set_fs_and_fc(const double fs, const double fc) {
  fs_ = fs;
  fc_ = fc;
}

void RunningTimeMeter::Start() {
  if (has_started_) {
    return;
  }
  start_time_ = Clock::NowInSeconds();
  has_started_ = true;
}

double RunningTimeMeter::DiffTime() const {
  return has_started_ ? (Clock::NowInSeconds() - start_time_) : 0.0;
}

void RunningTimeMeter::Stop() {
  has_started_ = false;
  start_time_ = 0.0;
}

DebounceModule::DebounceModule(const double rise_time, const double fall_time,
                               const double main_loop_time)
    : rise_time_limit_(rise_time),
      fall_time_limit_(fall_time),
      main_loop_time_(main_loop_time) {}

void DebounceModule::ResetTime(const double rise_time, const double fall_time,
                               const double main_loop_time) {
  rise_time_limit_ = rise_time;
  fall_time_limit_ = fall_time;
  main_loop_time_ = main_loop_time;
}

void DebounceModule::Reset() {
  in_pre_ = false;
  rise_time_val_ = 0.0;
  fall_time_val_ = 0.0;
}

bool DebounceModule::DealDebounce(bool input) {
  if (input && !in_pre_) {
    fall_time_val_ = 0.0;
    if (Compare(rise_time_val_, rise_time_limit_) == -1) {
      rise_time_val_ += main_loop_time_;
    } else {
      rise_time_val_ = 0.0;
      in_pre_ = input;
    }
  } else if (!input && in_pre_) {
    rise_time_val_ = 0.0;
    if (Compare(fall_time_val_, fall_time_limit_) == -1) {
      fall_time_val_ += main_loop_time_;
    } else {
      fall_time_val_ = 0.0;
      in_pre_ = input;
    }
  } else {
    fall_time_val_ = 0.0;
    rise_time_val_ = 0.0;
    in_pre_ = input;
  }
  return in_pre_;
}

double FirstOrderLowerPassFilter::Filter(const double x_insert) {
  double output = 0.0;
  if (is_first_step_) {
    last_output_ = x_insert;
    is_first_step_ = false;
  }
  if (filter_reset_) {
    output = x_insert;
  } else {
    output = filter_coefficient_ * x_insert +
             (1 - filter_coefficient_) * last_output_;
  }
  last_output_ = output;
  return output;
}

void FirstOrderLowerPassFilter::SetCoefficientAndFlag(
    const double filter_coefficient, const bool filter_reset) {
  filter_coefficient_ = filter_coefficient;
  filter_reset_ = filter_reset;
}

}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
