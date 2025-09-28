/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: Lingpeng
 * Created time: 2022/04/21
 *****************************************************************************/
//
// Created by lingpeng on 2022/4/21.
//

#pragma once

#include <utility>
#include <vector>

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_curve.h"

namespace TL {
namespace planning {
namespace nolane {
class FitOsqpCurve final : public FitCurve {
 public:
  FitOsqpCurve();
  ~FitOsqpCurve();
  void Fit(const std::vector<Vec2d>& points) override;
  void CalibratedByTrueValue(
      const std::vector<std::pair<double, double>>& lane_line_hdmap) override;
  double GenerateSinglePoint(double x) override;
};
}  // namespace nolane
}  // namespace planning
}  // namespace TL
