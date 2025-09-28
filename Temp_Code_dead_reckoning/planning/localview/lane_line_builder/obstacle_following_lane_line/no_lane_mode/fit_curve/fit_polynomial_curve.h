/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: Lingpeng
 * Created time: 2022/04/21
 *****************************************************************************/

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_curve.h"

namespace TL {
namespace planning {
namespace nolane {

class FitPolynomialCurve final : public FitCurve {
 public:
  explicit FitPolynomialCurve(size_t n);

  ~FitPolynomialCurve();

  void Fit(const std::vector<Vec2d>& points) override;

  void CalibratedByTrueValue(
      const std::vector<std::pair<double, double>>& lane_line_hdmap) override;

  double GenerateSinglePoint(double x) override;

  /**
   * @brief  use 1 order polynomial to fit points,
   * if slope is too big, return false.
   * @param points
   * @return true: slope less than 60
   * false: slope big than 60
   */
  bool TestFit(const std::vector<Vec2d>& points) override;

 private:
  int order_;
  int point_num_;
};
}  // namespace nolane
}  // namespace planning
}  // namespace TL
