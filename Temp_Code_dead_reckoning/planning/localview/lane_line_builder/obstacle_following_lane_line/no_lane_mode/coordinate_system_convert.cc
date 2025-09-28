/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/04/21
 *****************************************************************************/

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/coordinate_system_convert.h"

#include <tuple>
#include <utility>

namespace TL {
namespace planning {
namespace nolane {

void CoordinateSystemConvert::Process(
    const TL::common::VehicleState& vehicle_state) {
  // lp: TODO coordinate system convert with vehicle dynamic formula.
  // lp: 1 simple
  // lp: 2 complex
  std::get<0>(ego_state_) = vehicle_state.x();
  std::get<1>(ego_state_) = vehicle_state.y();
  std::get<2>(ego_state_) = vehicle_state.heading();
  return;
}

bool CoordinateSystemConvert::SystemEgo2Earth(
    double x2, double y2, std::pair<double, double>* const new_point_ptr,
    double x1, double y1, double theta) const {
  if (SeemsEqual(x1, -10.0) && SeemsEqual(y1, -10.0) &&
      SeemsEqual(theta, -10.0)) {
    x1 = std::get<0>(ego_state_);
    y1 = std::get<1>(ego_state_);
    theta = std::get<2>(ego_state_);
  }
  if (new_point_ptr) {
    new_point_ptr->first = x1 + x2 * cos(-theta) + y2 * sin(-theta);
    new_point_ptr->second = y1 + y2 * cos(-theta) - x2 * sin(-theta);
    return true;
  } else {
    return false;
  }
}
}  // namespace nolane
}  // namespace planning
}  // namespace TL
