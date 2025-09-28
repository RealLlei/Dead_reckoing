/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: Lingpeng
 * Created time: 2022/04/21
 *****************************************************************************/
//
// Created by lingpeng on 2022/4/21.
//

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_polynomial_curve.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace TL {
namespace planning {
namespace nolane {
FitPolynomialCurve::FitPolynomialCurve(size_t n)
    : FitCurve(1.0), order_(n), point_num_(100) {}

FitPolynomialCurve::~FitPolynomialCurve() {}

void FitPolynomialCurve::Fit(const std::vector<Vec2d>& points) {
  auto min_max_pair = std::minmax_element(
      points.cbegin(), points.cend(), [](auto& lhs, auto& rhs) {
        return TL::common::math::double_type::DefinitelyLess(lhs.x(),
                                                                rhs.x());
      });
  FitCurve::Fit(points);
  start_s_ = min_max_pair.first->x() - extend_length_before_start_s_;
  end_s_ = min_max_pair.second->x() + extend_length_after_end_s_;
  if (points.size() >= point_num_) {
    // lp: TODO too many points to calc matrix inverse. interpolate value
    // between start_s_ and end_s_???
  }
  double start_time = 0.0;
  if (debug_flag[2]) {
    start_time = std::chrono::steady_clock::now().time_since_epoch().count();
  }
  std::vector<double> coef;
  if (order_ == 1) {
    coef = TL::common::math::FitPolynomial<1>(points, &error_);
  } else if (order_ == 2) {
    coef = TL::common::math::FitPolynomial<2>(points, &error_);
  } else if (order_ == 3) {
    coef = TL::common::math::FitPolynomial<3>(points, &error_);
  } else if (order_ == 4) {
    coef = TL::common::math::FitPolynomial<4>(points, &error_);
  } else if (order_ == 5) {
    coef = TL::common::math::FitPolynomial<5>(points, &error_);
  }
  parameter_.assign(coef.begin(), coef.end());
  if (debug_flag[2]) {
    double end_time =
        std::chrono::steady_clock::now().time_since_epoch().count();
    std::string coef_str = "";
    TL::common::util::vec2str(coef, &coef_str);
    coef_str += "  error:" + std::to_string(error_);
    AERROR << "order:" << order_ << "  point size:" << points.size()
           << "  start_s:" << start_s_ << "   end_s:" << end_s_
           << "  coef_size:" << coef.size() << "  point_size:" << points.size()
           << "  fit_time:"
           << static_cast<double>((end_time - start_time) / 1e6) << "(ms)."
           << "  coef:" << coef_str;
  }
}

void FitPolynomialCurve::CalibratedByTrueValue(
    const std::vector<std::pair<double, double>>&
        lane_line_hdmap) {  // lp: TODO algorithm can be test by true value.
}

double FitPolynomialCurve::GenerateSinglePoint(double x) {
  double l = parameter_.at(0);
  for (int j = 1; j <= order_; ++j) {
    l += parameter_.at(j) * pow(x, static_cast<double>(j));
  }
  return l;
}

bool FitPolynomialCurve::TestFit(const std::vector<Vec2d>& points) {
  int old_order = order_;
  order_ = 1;
  Fit(points);
  order_ = old_order;
  return fabs(atan(parameter_.back())) > M_PI / 3 ? false : true;
}

}  // namespace nolane
}  // namespace planning
}  // namespace TL
