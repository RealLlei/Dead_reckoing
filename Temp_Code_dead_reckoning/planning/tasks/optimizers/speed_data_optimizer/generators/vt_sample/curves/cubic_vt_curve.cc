/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_curve.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/curves/cubic_vt_curve.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>

#include "common/file/log.h"

namespace TL {
namespace planning {

CubicVTCurve::CubicVTCurve(const SpeedCurveConfig& config)
    : SpeedCurve(config) {}

bool CubicVTCurve::Init(const common::TrajectoryPoint& init_point,
                        const double end_t, const double end_v,
                        const double end_a, const SpeedCurveTarget& target) {
  target_ = target;
  start_time_ = 0.0;
  end_time_ = end_t;
  start_s_ = 0.0;
  start_v_ = init_point.v();
  end_v_ = end_v;
  start_accel_ = init_point.a();
  end_accel_ = end_a;
  s_piece_.ComputeCoefficients(start_s_, start_v_, start_accel_, end_v_,
                               end_accel_, end_time_);
  v_piece_.DerivedFromQuarticCurve(s_piece_);
  a_piece_.DerivedFromCubicCurve(v_piece_);
  j_piece_.DerivedFromQuadraticCurve(a_piece_);

  end_s_ = s_piece_.Evaluate(0, end_time_);

  start_jerk_ = j_piece_.Evaluate(0, start_time_);
  end_jerk_ = j_piece_.Evaluate(0, end_time_);

  peek_v_ =
      v_piece_.GetMinAndMaxValue(start_time_, end_time_, &min_v_, &max_v_);
  a_piece_.GetMinAndMaxValue(start_time_, end_time_, &min_accel_, &max_accel_);
  j_piece_.GetMinAndMaxValue(start_time_, end_time_, &min_jerk_, &max_jerk_);
  return true;
}

int CubicVTCurve::Discretize(
    const double total_t, const double unit_t, const bool process_peak_value,
    std::vector<SpeedCurvePoint>* const vt_graph_points) const {
  UNUSED(process_peak_value);
  const auto point_count = static_cast<int>(round(total_t / unit_t)) + 1;
  if (vt_graph_points == nullptr ||
      point_count > static_cast<int>(vt_graph_points->size())) {
    return 0;
  }

  for (int i = 0; i < point_count; ++i) {
    auto& point = vt_graph_points->at(i);
    const auto t = start_time_ + i * unit_t;
    point.set_t(t - start_time_);

    if (t < end_time_) {
      point.set_s(s_piece_.Evaluate(0, t) - start_s_);
      point.set_v(v_piece_.Evaluate(0, t));
      point.set_a(a_piece_.Evaluate(0, t));
      point.set_j(j_piece_.Evaluate(0, t));
    } else {
      point.set_s(end_s_ + end_v_ * (t - end_time_) - start_s_);
      point.set_v(end_v_);
      point.set_a(end_accel_);
      point.set_j(0.0);
    }

    if (point.s() < 0 || point.v() < -1e-3) {
      return 0;
    }
  }
  return point_count;
}

SpeedCurvePoint CubicVTCurve::GetPoint(const double relative_t) const {
  SpeedCurvePoint point;
  point.set_t(relative_t);

  double t = relative_t + start_time_;

  if (t < end_time_) {
    point.set_s(s_piece_.Evaluate(0, t) - start_s_);
    point.set_v(v_piece_.Evaluate(0, t));
    point.set_a(a_piece_.Evaluate(0, t));
    point.set_j(j_piece_.Evaluate(0, t));
  } else {
    point.set_s(end_s_ + end_v_ * (t - end_time_) - start_s_);
    point.set_v(end_v_);
    point.set_a(end_accel_);
    point.set_j(0.0);
  }
  return point;
}

bool CubicVTCurve::SetStartPoint(const double estimated_start_time) {
  start_time_ = estimated_start_time;
  ADEBUG << "start_time:" << start_time_;

  if (start_time_ < end_time_) {
    start_s_ = s_piece_.Evaluate(0, start_time_);
  } else {
    start_s_ = end_s_ + end_v_ * (start_time_ - end_time_);
  }
  return true;
}

std::shared_ptr<SpeedCurve> CubicVTCurve::Clone() const {
  return std::make_shared<CubicVTCurve>(*this);
}

std::string CubicVTCurve::DebugString() const {
  std::stringstream ss;
  ss << "accumulated_time:" << end_time_ << std::endl;
  ss << "coef:" << s_piece_.ToString();
  return ss.str();
}

}  // namespace planning
}  // namespace TL
