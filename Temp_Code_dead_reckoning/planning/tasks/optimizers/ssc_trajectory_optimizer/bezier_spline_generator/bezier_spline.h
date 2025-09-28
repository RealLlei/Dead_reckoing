/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file bezier_spline.h
 **/

#pragma once

#include <algorithm>
#include <array>
#include <utility>
#include <vector>

#include "planning/tasks/optimizers/ssc_trajectory_optimizer/bezier_spline_generator/bezier_utils.h"

namespace TL::planning {

template <int N_DEG>
class BezierUtils;

/**
 * @brief Bezier spline class
 * The the j-th segment evaluation is given by
 * B_j(t) = s_j*\sum_{i=0}^{N_DEG} c_j^i * b_{N_DEG}^i(t-Tj/s_j)
 * the s_j is the time scaling factor of the j-the segment
 */
template <int N_DEG, int N_DIM>
class BezierSpline {
 public:
  using BezierPoint = std::array<double, N_DIM>;
  using ControlPointSegment = std::array<BezierPoint, N_DEG + 1>;

  BezierSpline() = default;

  void SetAccumulateTime(std::vector<double>&& accumulate_time) {
    accumulate_time_ = std::move(accumulate_time);
  }

  void SetControlPointSegments(
      std::vector<ControlPointSegment>&& control_point_segments) {
    control_point_segments_ = std::move(control_point_segments);
  }

  int GetSegmentCount() const {
    return static_cast<int>(control_point_segments_.size());
  }

  const BezierPoint& GetControlPoint(const int segment_index,
                                     const int control_point_index) {
    return control_point_segments_.at(segment_index).at(control_point_index);
  }

  const std::vector<ControlPointSegment>& GetControlPointSegments() const {
    return control_point_segments_;
  }

  const std::vector<double>& GetAccumulateTime() const {
    return accumulate_time_;
  }

  /**
   * @brief Get the begin of the parameterization
   */
  double Begin() const {
    return accumulate_time_.empty() ? 0.0 : accumulate_time_.front();
  }

  /**
   * @brief Get the end of the parameterization
   */
  double End() const {
    return accumulate_time_.empty() ? 0.0 : accumulate_time_.back();
  }

  /**
   * @brief Return evaluation of spline at given parameterization
   * @param s parameterization, s should be in the vector domain
   * @param d derivate to take
   * @note  out of domain s will be rounded to the end()
   */
  bool Evaluate(const double s, const int d, BezierPoint* point) const {
    if (point == nullptr) {
      return false;
    }

    int num_pts = accumulate_time_.size();
    if (num_pts < 1) {
      return false;
    }

    auto it =
        std::lower_bound(accumulate_time_.begin(), accumulate_time_.end(), s);
    int idx = std::min(
        std::max(static_cast<int>(it - accumulate_time_.begin()) - 1, 0),
        num_pts - 2);
    auto h = s - accumulate_time_.at(idx);
    if (s < accumulate_time_.at(0)) {
      return false;
    }

    auto duration = accumulate_time_.at(idx + 1) - accumulate_time_.at(idx);
    auto normalized_time = h / duration;
    const auto basis = BezierUtils<N_DEG>::GetBezierBasis(d, normalized_time);

    const auto& control_point_segment = control_point_segments_[idx];
    for (int i = 0; i < N_DIM; ++i) {
      point->at(i) = 0.0;
      for (int j = 0; i < basis.size(); ++j) {
        point->at(i) += control_point_segment.at(j).at(i) * basis.at(j);
      }
    }

    return true;
  }

 private:
  std::vector<ControlPointSegment> control_point_segments_;
  std::vector<double> accumulate_time_;
};

}  // namespace TL::planning
