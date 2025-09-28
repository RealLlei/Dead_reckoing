/*
 * Copyright (c) TL auto Co., Ltd. 2022-2023. All rights reserved.
 */

#include "planning/localview/lane_line_builder/real_hdmap_lane_line/double_rise_decider.h"

namespace TL {
namespace planning {

void DoubleRiseDecider::Reset() {
  state_ = DoubleRiseState::None;
  wait_time_ = 0.0;
}

bool DoubleRiseDecider::Decider(const bool first, const bool second) {
  const double main_loop_time = 0.1;
  bool is_rise = false;
  switch (state_) {
    case DoubleRiseState::None:
      if (first && second) {
        is_rise = true;
        state_ = DoubleRiseState::ExpectEnd;
      } else if (first) {
        is_rise = true;
        state_ = DoubleRiseState::ExpextSecond;
      } else if (second) {
        is_rise = true;
        state_ = DoubleRiseState::ExpectFirst;
      }
      break;
    case DoubleRiseState::ExpextSecond:
      is_rise = true;
      wait_time_ += main_loop_time;
      if (wait_time_ > max_wait_time_ || second) {
        wait_time_ = 0.0;
        state_ = DoubleRiseState::ExpectEnd;
      }
      break;
    case DoubleRiseState::ExpectFirst:
      is_rise = true;
      wait_time_ += main_loop_time;
      if (wait_time_ > max_wait_time_ || first) {
        wait_time_ = 0.0;
        state_ = DoubleRiseState::ExpectEnd;
      }
      break;
    case DoubleRiseState::ExpectEnd:
      if (first || second) {
        is_rise = true;
      } else {
        state_ = DoubleRiseState::None;
      }
      break;
    default:
      break;
  }
  return is_rise;
}

}  // namespace planning
}  // namespace TL
