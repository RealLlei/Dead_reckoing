/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: Lingpeng
 * Created time: 2022/04/21
 *****************************************************************************/
//
// Created by lingpeng on 2022/4/21.
//

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_osqp_curve.h"

#include <utility>
#include <vector>

namespace TL {
namespace planning {
namespace nolane {
FitOsqpCurve::FitOsqpCurve() {}

FitOsqpCurve::~FitOsqpCurve() {}

void FitOsqpCurve::Fit(const std::vector<Vec2d>& points) {}

void FitOsqpCurve::CalibratedByTrueValue(
    const std::vector<std::pair<double, double>>& lane_line_hdmap) {}

double FitOsqpCurve::GenerateSinglePoint(double x) {
  return 0;
}
}  // namespace nolane
}  // namespace planning
}  // namespace TL
