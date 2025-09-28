/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: Lingpeng
 * Created time: 2022/04/21
 *****************************************************************************/

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_curve.h"

#include <algorithm>
#include <string>

namespace TL {
namespace planning {
namespace nolane {

FitCurve::FitCurve(double s_interval, int minimal_size)
    : start_s_(-1.0),
      end_s_(-2.0),
      extend_length_before_start_s_(5.0),
      extend_length_after_end_s_(0.0),
      s_interval_(s_interval),
      minimal_size_(minimal_size),
      error_(0.0) {}

void FitCurve::Fit(const std::vector<Vec2d>& points) {
  if (debug_flag[23]) {
    std::string x_l = "[";
    std::string y_l = "[";
    for (const auto& point : points) {
      x_l += std::to_string(point.x()) + ",";
      y_l += std::to_string(point.y()) + ",";
      // AERROR << PRECISION(3) << "counter:" << i
      //        << "   p_raw_x:" << points.at(i).x()
      //        << "   p_raw_y:" << points.at(i).y();
    }
    x_l.back() = ']';
    y_l.back() = ']';
    // x_l += "]";
    // y_l += "]";
    AERROR << " raw_x = " << x_l;
    AERROR << " raw_y = " << y_l;
  }
}

bool FitCurve::FitPiecewise(
    std::vector<std::pair<common::SLPoint, Vec2d>>* const points,
    std::vector<Vec2d>* const fit_points,
    const std::shared_ptr<ReferenceLine>& reference_line_ptr) {
  if (points->size() < minimal_size_) {
    AERROR << "point size is to small, point size:" << points->size()
           << "  minimal_size:" << minimal_size_;
    return false;
  }
  std::sort(points->begin(), points->end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.first.s() < rhs.first.s();
            });

  if (debug_flag[8]) {
    int counter = 0;
    ADEBUG << PRECISION(3) << "   start_s:" << points->front().first.s()
           << "   end_s:" << points->back().first.s();
    for (const auto& point : *points) {
      AERROR << PRECISION(3) << "counter:" << counter++
             << "  fit_input_s:" << point.first.s()
             << "   fit_input_l:" << point.first.l()
             << "   obs_raw_x:" << point.second.x()
             << "   obs_raw_y:" << point.second.y();
    }
  }

  return true;
}

const std::vector<double>& FitCurve::GetParameter() const {
  return parameter_;
}

bool FitCurve::GeneratePointsByFormula(std::vector<Vec2d>* const points_fit) {
  if (!points_fit) {
    AERROR << "points_fit is nullptr.";
    return false;
  }
  if (start_s_ > end_s_ && start_s_ < 0) {
    AERROR << "fit parameter error!!!"
           << "  start_s:" << start_s_ << "   end_s:" << end_s_;
    return false;
  }
  size_t num = static_cast<size_t>((end_s_ - start_s_) / s_interval_) + 1;
  points_fit->clear();
  points_fit->reserve(num);
  double s = start_s_;
  double l = 0;
  for (int i = 0; i < num && DefinitelyLessEqual(s, end_s_); ++i) {
    l = GenerateSinglePoint(s);
    s += s_interval_;
    points_fit->emplace_back(s, l);
  }
  return true;
}

double FitCurve::GetSInterval() const {
  return s_interval_;
}

double FitCurve::GetError() const {
  return error_;
}

bool FitCurve::TestFit(const std::vector<Vec2d>& points) {
  return false;
}
}  // namespace nolane
}  // namespace planning
}  // namespace TL
