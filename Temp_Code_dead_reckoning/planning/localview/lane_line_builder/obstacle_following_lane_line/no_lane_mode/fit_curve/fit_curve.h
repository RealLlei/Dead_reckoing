/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: Lingpeng
 * Created time: 2022/04/21
 *****************************************************************************/

#pragma once

#include <Eigen/Dense>

#include <chrono>
#include <memory>
#include <utility>
#include <vector>

#include "common/math/curve_fitting.h"
#include "common/math/double_type.h"
#include "common/math/vec2d.h"
#include "common/time/clock.h"
#include "common/util/string_util.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/common_util/coordinate_transform.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/common_util/util.h"

namespace TL {
namespace planning {
namespace nolane {

using TL::common::math::Vec2d;

// lp: TODO: question:
//  How to handle cross road? fitting to curve? sample points?
// lp: input some points, output some fitted points.
class FitCurve {
 public:
  explicit FitCurve(double s_interval = 1.0, int minimal_size = 10);

  virtual ~FitCurve() = default;

  FitCurve(const FitCurve& rhs) = delete;
  FitCurve& operator=(const FitCurve& rhs) = delete;

  /**
   * @brief fit specific curve base on algorithm.
   * @param points
   */
  virtual void Fit(const std::vector<Vec2d>& points) = 0;

  bool FitPiecewise(
      std::vector<std::pair<common::SLPoint, Vec2d>>* const points,
      std::vector<Vec2d>* const fit_points,
      const std::shared_ptr<ReferenceLine>& reference_line_ptr);

  /**
   * @brief Improve performance when having true value of lane-line.
   * @param lane_line_hdmap
   */
  virtual void CalibratedByTrueValue(
      const std::vector<std::pair<double, double>>& lane_line_hdmap) = 0;

  const std::vector<double>& GetParameter() const;

  bool GeneratePointsByFormula(std::vector<Vec2d>* const points_fit);

  virtual double GenerateSinglePoint(double x) = 0;

  double GetSInterval() const;

  double GetError() const;

  virtual bool TestFit(const std::vector<Vec2d>& points);

 protected:
  std::vector<double> parameter_;
  double start_s_;
  double end_s_;
  double extend_length_before_start_s_;
  double extend_length_after_end_s_;
  double s_interval_;
  int minimal_size_;
  double error_;
};
}  // namespace nolane
}  // namespace planning
}  // namespace TL
