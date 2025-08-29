// Copyright (c) TL Technologies Co., Ltd. 2019-2020. All rights reserved.
// Created by pengsuhang on 2023/9/18.

#include "common/math/pure_pursuit_control.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

/**
 * @namespace TL::common::math
 * @brief TL::common::math
 */
namespace TL {
namespace common {
namespace math {

// PurePursuit func
void PurePursuit::StateUpdate(const double delta) {
  cur_point_.set_x(cur_point_.x() + vehicle_v_ * cos(heading_) * dt_);
  cur_point_.set_y(cur_point_.y() + vehicle_v_ * sin(heading_) * dt_);
  heading_ += vehicle_v_ / vehicle_l_ * tan(delta) * dt_;
}

size_t PurePursuit::CalTargetIndex(
    const std::vector<TL::common::math::Vec2d>& path) {
  size_t min_index = 0;
  double min_dis = std::numeric_limits<double>::infinity();
  const int start_idx = std::max(0, cur_index_ - 10);
  const int end_idx =
      std::min(static_cast<int>(path.size() - 1), cur_index_ + 10);
  for (int i = start_idx; i <= end_idx; i++) {
    double cur_dis = path[i].DistanceTo(cur_point_);
    if (cur_dis < min_dis) {
      min_dis = cur_dis;
      min_index = i;
    }
  }
  cur_index_ = static_cast<int>(min_index);

  while (min_dis < ld_ && min_index < path.size() - 1) {
    min_dis = path[min_index + 1].DistanceTo(cur_point_);
    min_index++;
  }
  return min_index;
}

double PurePursuit::PurePursuitControl(
    const std::vector<TL::common::math::Vec2d>& path,
    const size_t min_index) {
  TL::common::math::Vec2d vec = path[min_index] - cur_point_;
  double alpha = atan2(vec.y(), vec.x()) - heading_;
  double delta = atan2(2 * vehicle_l_ * sin(alpha), ld_);
  return delta;
}

}  // namespace math
}  // namespace common
}  // namespace TL
