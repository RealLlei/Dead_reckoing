/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning osqp finite differfence interface smoother
 * Author: ROC
 */

#pragma once

#include <tuple>
#include <utility>
#include <vector>

#include "planning/math/discretized_points_smoothing/osqp_finite_difference_interface.h"
#include "planning/reference_line/util/reference_line_debug.h"

#include "planning/proto/osqp_finite_difference_smoother_config.pb.h"

namespace TL {
namespace planning {

/*
 * @brief:
 * This class solve an optimization problem:
 * Y
 * |
 * |                       P(x1, y1)  P(x2, y2)
 * |            P(x0, y0)                       ... P(x(k-1), y(k-1))
 * |P(start)
 * |
 * |________________________________________________________ X
 *
 *
 * Given an initial set of points from 0 to k-1,  The goal is to find a set of
 * points which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */

class OsqpFiniteDifferenceSmoother {
 public:
  explicit OsqpFiniteDifferenceSmoother(OsqpConfig config);

  bool Solve(const std::vector<std::pair<double, double>>& raw_point2d,
             const AnchorpointsInformation& anchorpoints_information,
             std::vector<double>* opt_x, std::vector<double>* opt_y,
             common::ReferenceLine* reference_line_debug,
             bool openspace_status);

  bool QpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d,
                  const AnchorpointsInformation& anchorpoints_information,
                  std::vector<double>* opt_x, std::vector<double>* opt_y,
                  common::ReferenceLine* reference_line_debug,
                  bool openspace_status);

 private:
  const OsqpConfig config_;
};
}  // namespace planning
}  // namespace TL
