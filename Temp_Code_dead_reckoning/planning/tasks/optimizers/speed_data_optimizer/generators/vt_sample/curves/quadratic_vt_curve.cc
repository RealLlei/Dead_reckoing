/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_curve.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/curves/quadratic_vt_curve.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <sstream>

#include "absl/strings/str_join.h"
#include "common/file/log.h"
#include "common/math/double_type.h"

namespace TL {
namespace planning {

using common::math::double_type::SeemsEqual;

QuadraticVTCurve::QuadraticVTCurve(const SpeedCurveConfig& config)
    : SpeedCurve(config) {}

bool QuadraticVTCurve::InitPiecewiseJerkCurve(
    const common::TrajectoryPoint& init_point,
    const std::vector<std::pair<double, double>>& params,
    const SpeedCurveTarget& target, const bool allow_negative_speed) {
  if (params.empty() || params.size() > v_pieces_.size()) {
    return false;
  }

  target_ = target;
  start_time_ = 0.0;
  start_s_ = 0.0;
  end_s_ = 0.0;
  start_v_ = init_point.v();
  end_v_ = init_point.v();
  max_v_ = std::numeric_limits<double>::lowest();
  min_v_ = std::numeric_limits<double>::max();
  start_accel_ = init_point.a();
  end_accel_ = init_point.a();
  max_accel_ = std::numeric_limits<double>::lowest();
  min_accel_ = std::numeric_limits<double>::max();
  start_jerk_ = init_point.da();

  start_jerk_ = params.at(0).second;
  max_jerk_ = std::numeric_limits<double>::lowest();
  min_jerk_ = std::numeric_limits<double>::max();

  piece_count_ = params.size();
  accumulated_time_.at(0) = 0.0;

  for (size_t i = 0; i < piece_count_; ++i) {
    const auto t = params.at(i).first;
    const auto j = params.at(i).second;
    auto& v_piece = v_pieces_.at(i);
    accumulated_time_.at(i + 1) = accumulated_time_.at(i) + t;
    v_piece.Init(end_v_, end_accel_, j / 2);
    end_v_ = v_piece.Evaluate(0, t);
    end_accel_ = v_piece.Evaluate(1, t);
    max_v_ = fmax(end_v_, max_v_);
    min_v_ = fmin(end_v_, min_v_);
    const auto peak_v = v_piece.GetPeakValue(0.0, t);
    if (std::isfinite(peak_v)) {
      max_v_ = fmax(peak_v, max_v_);
      min_v_ = fmin(peak_v, min_v_);
    }
    max_accel_ = fmax(end_accel_, max_accel_);
    min_accel_ = fmin(end_accel_, min_accel_);
    max_jerk_ = fmax(j, max_jerk_);
    min_jerk_ = fmin(j, min_jerk_);
    end_s_ += v_piece.Integral(t);
  }
  end_time_ = accumulated_time_.at(piece_count_);

  allow_negative_speed_ = allow_negative_speed;
  return true;
}

bool QuadraticVTCurve::InitPiecewiseAccelCurve(
    const common::TrajectoryPoint& init_point,
    const std::vector<std::pair<double, double>>& params,
    const double end_accel, const SpeedCurveTarget& target) {
  if (params.empty() || params.size() > v_pieces_.size()) {
    return false;
  }

  target_ = target;
  start_time_ = 0.0;
  start_s_ = 0.0;
  end_s_ = 0.0;
  start_v_ = init_point.v();
  end_v_ = init_point.v();
  max_v_ = std::numeric_limits<double>::lowest();
  min_v_ = std::numeric_limits<double>::max();
  start_accel_ = params.at(0).second;
  end_accel_ = end_accel;
  max_accel_ = std::numeric_limits<double>::lowest();
  min_accel_ = std::numeric_limits<double>::max();
  start_jerk_ = 0.0;
  max_jerk_ = 0.0;
  min_jerk_ = 0.0;

  piece_count_ = 0;
  accumulated_time_.at(0) = 0.0;

  for (const auto& param : params) {
    const auto t = param.first;
    const auto a = param.second;
    if (SeemsEqual(t, 0.0)) {
      continue;
    }

    accumulated_time_.at(piece_count_ + 1) =
        accumulated_time_.at(piece_count_) + t;
    v_pieces_.at(piece_count_).Init(end_v_, a, 0.0);
    end_v_ = v_pieces_.at(piece_count_).Evaluate(0, t);
    max_v_ = fmax(end_v_, max_v_);
    min_v_ = fmin(end_v_, min_v_);
    max_accel_ = fmax(a, max_accel_);
    min_accel_ = fmin(a, min_accel_);
    end_s_ += v_pieces_.at(piece_count_).Integral(t);
    ++piece_count_;
  }

  max_v_ = fmax(start_v_, max_v_);
  min_v_ = fmin(start_v_, min_v_);
  max_accel_ = fmax(start_accel_, max_accel_);
  min_accel_ = fmin(start_accel_, min_accel_);
  end_time_ = accumulated_time_.at(piece_count_);
  return true;
}

int QuadraticVTCurve::Discretize(
    const double total_t, const double unit_t, const bool process_peak_value,
    std::vector<SpeedCurvePoint>* const vt_graph_points) const {
  const auto point_count = static_cast<int>(round(total_t / unit_t)) + 1;
  if (vt_graph_points == nullptr ||
      point_count > static_cast<int>(vt_graph_points->size())) {
    return 0;
  }

  // piece_index is the index of the current piece ,
  // accumulated_time_.at(piece_index) is the start time of the current piece,
  // accumulated_time_.at(piece_index + 1) is the end time of the current piece
  std::size_t piece_index = 0;
  double accumulated_s = 0.0;
  for (int i = 0; i < point_count; ++i) {
    auto& vt_graph_point = vt_graph_points->at(i);
    const auto t = start_time_ + i * unit_t;
    for (; t > accumulated_time_.at(piece_index + 1) &&
           piece_index < piece_count_;
         ++piece_index) {
      const auto dt = accumulated_time_.at(piece_index + 1) -
                      accumulated_time_.at(piece_index);
      accumulated_s += v_pieces_.at(piece_index).Integral(dt);
    }

    const auto dt = t - accumulated_time_.at(piece_index);
    vt_graph_point.set_t(t - start_time_);
    if (piece_index < piece_count_) {
      vt_graph_point.set_s(accumulated_s +
                           v_pieces_.at(piece_index).Integral(dt) - start_s_);
      vt_graph_point.set_v(v_pieces_.at(piece_index).Evaluate(0, dt));
      vt_graph_point.set_a(v_pieces_.at(piece_index).Evaluate(1, dt));
      vt_graph_point.set_j(v_pieces_.at(piece_index).Evaluate(2, dt));
    } else {
      vt_graph_point.set_s(accumulated_s + end_v_ * (t - end_time_) - start_s_);
      vt_graph_point.set_v(end_v_);
      vt_graph_point.set_a(end_accel_);
      vt_graph_point.set_j(0.0);
    }

    if (vt_graph_point.s() < 0 || vt_graph_point.v() < -1e-3) {
      if (!allow_negative_speed_ || i == 0) {
        return 0;
      }

      const auto last_point = vt_graph_points->at(i - 1);
      for (; i < point_count; ++i) {
        vt_graph_point.set_t(i * unit_t);
        vt_graph_point.set_s(last_point.s());
        vt_graph_point.set_v(0.0);
        vt_graph_point.set_a(last_point.a());
        vt_graph_point.set_j(0.0);
      }
      return point_count;
    }
  }

  // process peak value
  if (!process_peak_value) {
    return point_count;
  }

  for (piece_index = 0; piece_index < piece_count_; ++piece_index) {
    const auto point_index = static_cast<int>(
        round((accumulated_time_.at(piece_index + 1) - start_time_) / unit_t));
    if (point_index >= point_count || point_index < 0) {
      break;
    }

    const auto dt = accumulated_time_.at(piece_index + 1) -
                    accumulated_time_.at(piece_index);
    const auto a = v_pieces_.at(piece_index).Evaluate(1, dt);
    const auto j = v_pieces_.at(piece_index).Evaluate(2, dt);

    auto& vt_graph_point = vt_graph_points->at(point_index);
    if (fabs(vt_graph_point.a()) < fabs(a)) {
      vt_graph_point.set_a(a);
    }

    if (fabs(vt_graph_point.j()) < fabs(j)) {
      vt_graph_point.set_j(j);
    }
  }

  return point_count;
}

SpeedCurvePoint QuadraticVTCurve::GetPoint(const double relative_t) const {
  SpeedCurvePoint point;
  // piece_index is the index of the current piece ,
  // accumulated_time_.at(piece_index) is the start time of the current piece,
  // accumulated_time_.at(piece_index + 1) is the end time of the current piece
  size_t piece_index = 0;
  double accumulated_s = 0.0;
  double t = relative_t + start_time_;
  for (;
       t > accumulated_time_.at(piece_index + 1) && piece_index < piece_count_;
       ++piece_index) {
    const auto dt = accumulated_time_.at(piece_index + 1) -
                    accumulated_time_.at(piece_index);
    accumulated_s += v_pieces_.at(piece_index).Integral(dt);
  }

  point.set_t(t - start_time_);
  if (piece_index < piece_count_) {
    const auto dt = t - accumulated_time_.at(piece_index);
    point.set_s(accumulated_s + v_pieces_.at(piece_index).Integral(dt) -
                start_s_);
    point.set_v(v_pieces_.at(piece_index).Evaluate(0, dt));
    point.set_a(v_pieces_.at(piece_index).Evaluate(1, dt));
    point.set_j(v_pieces_.at(piece_index).Evaluate(2, dt));
  } else {
    point.set_s(accumulated_s + end_v_ * (t - end_time_) - start_s_);
    point.set_v(end_v_);
    point.set_a(end_accel_);
    point.set_j(0.0);
  }

  return point;
}

bool QuadraticVTCurve::SetStartPoint(const double estimated_start_time) {
  start_time_ = estimated_start_time;
  ADEBUG << "start_time:" << start_time_;

  //
  if (target_.mode == SpeedCurveTarget::Mode::STOP && piece_count_ >= 3 &&
      accumulated_time_.size() >= 4 && start_time_ > accumulated_time_.at(3)) {
    target_.mode = SpeedCurveTarget::Mode::STOP_TO_STANDSTILL;
  }

  // piece_index is the index of the current piece ,
  // accumulated_time_.at(piece_index) is the start time of the current piece,
  // accumulated_time_.at(piece_index + 1) is the end time of the current piece
  std::size_t piece_index = 0;
  double accumulated_s = 0.0;
  for (; start_time_ > accumulated_time_.at(piece_index + 1) &&
         piece_index < piece_count_;
       ++piece_index) {
    accumulated_s += v_pieces_.at(piece_index)
                         .Integral(accumulated_time_.at(piece_index + 1) -
                                   accumulated_time_.at(piece_index));
  }

  if (piece_index < piece_count_) {
    start_s_ = accumulated_s +
               v_pieces_.at(piece_index)
                   .Integral(start_time_ - accumulated_time_.at(piece_index));
  } else {
    start_s_ = accumulated_s + end_v_ * (start_time_ - end_time_);
  }
  return true;
}

std::shared_ptr<SpeedCurve> QuadraticVTCurve::Clone() const {
  return std::make_shared<QuadraticVTCurve>(*this);
}

std::string QuadraticVTCurve::DebugString() const {
  std::stringstream ss;
  ss << "piece_count:" << piece_count_ << std::endl;
  ss << "accumulated_time:" << absl::StrJoin(accumulated_time_, ", ")
     << std::endl;
  for (const auto& v_piece : v_pieces_) {
    ss << "piece_coef:" << v_piece.ToString();
  }
  return ss.str();
}

}  // namespace planning
}  // namespace TL
