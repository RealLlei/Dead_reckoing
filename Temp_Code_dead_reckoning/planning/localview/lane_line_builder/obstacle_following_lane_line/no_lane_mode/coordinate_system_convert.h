/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/04/21
 *****************************************************************************/

#pragma once

#include <tuple>
#include <utility>

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/ego_vehicle_state.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/obstacle_perception_no_lane.h"

namespace TL {
namespace planning {
namespace nolane {
class CoordinateSystemConvert {
 public:
  CoordinateSystemConvert() = default;

  ~CoordinateSystemConvert() = default;

  void Process(const TL::common::VehicleState& vehicle_state);

  /**
   * @brief calculate the new_point_ptr value of (x2,y2) in system of (x1 ,y1
   * ,theta). convert ego system to earth system.
   * @param x2 raw x value in system (x1,y1,theta).
   * @param y2 raw y value in system (x1,y1,theta).
   * @param new_point_ptr  return <x,y>.
   * @param x1 relative system x.
   * @param y1 relative system y.
   * @param theta relative system x,y in theta direction.
   * @return true, new_point_ptr is not nullptr. otherwise, return false.
   */
  bool SystemEgo2Earth(double x2, double y2,
                       std::pair<double, double>* const new_point_ptr,
                       double x1 = -10.0, double y1 = -10.0,
                       double theta = -10.0) const;

 private:
  std::tuple<double, double, double> ego_state_;  // lp: x,y,theta in earth.
};
}  // namespace nolane
}  // namespace planning
}  // namespace TL
