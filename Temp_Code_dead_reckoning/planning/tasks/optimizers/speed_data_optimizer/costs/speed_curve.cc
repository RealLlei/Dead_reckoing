/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_curve.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_curve.h"

#include <algorithm>
#include <cstddef>
#include <limits>

#include "common/math/math_utils.h"
#include "common/util/point_factory.h"

namespace TL {
namespace planning {

SpeedCurve::SpeedCurve(const SpeedCurveConfig& config)
    : min_sparse_point_count_(
          static_cast<int>(
              round(config.min_total_time() / config.sparse_point_interval())) +
          1),
      max_sparse_point_count_(
          static_cast<int>(
              round(config.max_total_time() / config.sparse_point_interval())) +
          1),
      min_dense_point_count_(
          static_cast<int>(
              round(config.min_total_time() / config.dense_point_interval())) +
          1),
      max_dense_point_count_(
          static_cast<int>(
              round(config.max_total_time() / config.dense_point_interval())) +
          1),
      sparse_point_interval_(config.sparse_point_interval()),
      dense_point_interval_(config.dense_point_interval()) {
  sparse_points_.resize(max_sparse_point_count_);
  dense_points_.resize(max_dense_point_count_);
}

void SpeedCurve::Discretize(const double total_t, const double unit_t,
                            SpeedData* const speed_data) const {
  const auto point_count = static_cast<int>(round(total_t / unit_t)) + 1;
  std::vector<SpeedCurvePoint> speed_curve_points(point_count,
                                                  SpeedCurvePoint());
  speed_data->reserve(point_count);
  Discretize(total_t, unit_t, false, &speed_curve_points);
  std::transform(
      speed_curve_points.begin(), speed_curve_points.end(),
      std::back_inserter(*speed_data), [](const auto& vt_graph_point) {
        return common::util::PointFactory::ToSpeedPoint(
            vt_graph_point.s(), vt_graph_point.t(), vt_graph_point.v(),
            vt_graph_point.a(), vt_graph_point.j());
      });
}

int SpeedCurve::DiscretizeSparsePoint() {
  // sample sparse point
  curve_sparse_point_count_ =
      static_cast<int>(
          round((GetEndTime() - GetStartTime()) / sparse_point_interval_)) +
      1;
  curve_sparse_point_count_ = common::math::Clamp(curve_sparse_point_count_, 1,
                                                  max_sparse_point_count_);
  return Discretize(sparse_point_interval_ * (max_sparse_point_count_ - 1),
                    sparse_point_interval_, true, &sparse_points_);
}

int SpeedCurve::DiscretizeDensePoint() {
  // sample dense point
  curve_dense_point_count_ =
      static_cast<int>(
          round((GetEndTime() - GetStartTime()) / dense_point_interval_)) +
      1;
  curve_dense_point_count_ =
      common::math::Clamp(curve_dense_point_count_, 1, max_dense_point_count_);
  return Discretize(dense_point_interval_ * (max_dense_point_count_ - 1),
                    dense_point_interval_, true, &dense_points_);
}

}  // namespace planning
}  // namespace TL
