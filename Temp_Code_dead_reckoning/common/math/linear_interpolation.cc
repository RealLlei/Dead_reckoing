/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  common math interpolation
 * Author: ROC
 */

#include "common/math/linear_interpolation.h"

#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/math_utils.h"

namespace TL {
namespace common {
namespace math {

using common::PointENU;
using common::math::double_type::Compare;

double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    ADEBUG << "input time difference is too small";
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}

SLPoint InterpolateUsingLinearApproximation(const SLPoint& p0,
                                            const SLPoint& p1, const double w) {
  CHECK_GE(w, 0.0);

  SLPoint p;
  p.set_s((1 - w) * p0.s() + w * p1.s());
  p.set_l((1 - w) * p0.l() + w * p1.l());
  return p;
}

PathPoint InterpolateUsingLinearApproximation(const PathPoint& p0,
                                              const PathPoint& p1,
                                              const double s) {
  double s0 = p0.s();
  double s1 = p1.s();

  PathPoint path_point;
  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * p0.x() + weight * p1.x();
  double y = (1 - weight) * p0.y() + weight * p1.y();
  double z = (1 - weight) * p0.z() + weight * p1.z();
  double theta = slerp(p0.theta(), p0.s(), p1.theta(), p1.s(), s);
  double kappa = (1 - weight) * p0.kappa() + weight * p1.kappa();
  double dkappa = (1 - weight) * p0.dkappa() + weight * p1.dkappa();
  double ddkappa = (1 - weight) * p0.ddkappa() + weight * p1.ddkappa();
  path_point.set_x(x);
  path_point.set_y(y);
  path_point.set_z(z);
  path_point.set_theta(theta);
  path_point.set_kappa(kappa);
  path_point.set_dkappa(dkappa);
  path_point.set_ddkappa(ddkappa);
  path_point.set_s(s);
  return path_point;
}

TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint& tp0,
                                                    const TrajectoryPoint& tp1,
                                                    const double t) {
  if (!tp0.has_path_point() || !tp1.has_path_point()) {
    TrajectoryPoint p;
    p.mutable_path_point()->CopyFrom(PathPoint());
    return p;
  }
  const auto& pp0 = tp0.path_point();
  const auto& pp1 = tp1.path_point();
  double t0 = tp0.relative_time();
  double t1 = tp1.relative_time();

  TrajectoryPoint tp;
  tp.set_v(lerp(tp0.v(), t0, tp1.v(), t1, t));
  tp.set_a(lerp(tp0.a(), t0, tp1.a(), t1, t));
  tp.set_da(lerp(tp0.da(), t0, tp1.da(), t1, t));
  tp.set_relative_time(t);
  tp.set_steer(slerp(tp0.steer(), t0, tp1.steer(), t1, t));

  PathPoint* path_point = tp.mutable_path_point();
  path_point->set_x(lerp(pp0.x(), t0, pp1.x(), t1, t));
  path_point->set_y(lerp(pp0.y(), t0, pp1.y(), t1, t));
  path_point->set_z(lerp(pp0.z(), t0, pp1.z(), t1, t));
  path_point->set_theta(slerp(pp0.theta(), t0, pp1.theta(), t1, t));
  path_point->set_kappa(lerp(pp0.kappa(), t0, pp1.kappa(), t1, t));
  path_point->set_dkappa(lerp(pp0.dkappa(), t0, pp1.dkappa(), t1, t));
  path_point->set_ddkappa(lerp(pp0.ddkappa(), t0, pp1.ddkappa(), t1, t));
  path_point->set_s(lerp(pp0.s(), t0, pp1.s(), t1, t));

  return tp;
}

double InterpolationOne(const double& input_x,
                        const std::vector<double>& input_v,
                        const std::vector<double>& output_v) {
  uint v_size = input_v.size();
  if (input_x >= input_v[v_size - 1]) {
    return output_v[v_size - 1];
  }
  if (input_x <= input_v[0]) {
    return output_v[0];
  }
  for (int a = 1; a < input_v.size(); ++a) {
    if (input_x >= input_v[a - 1] && input_x < input_v[a]) {
      return output_v[a - 1] + (output_v[a] - output_v[a - 1]) *
                                   (input_x - input_v[a - 1]) /
                                   (input_v[a] - input_v[a - 1]);
    }
  }
  ADEBUG << "InterpolationOne error!";
  return 1.0;
}

double InterpolationOne(
    const double& input_x,
    const google::protobuf::RepeatedField<double>& input_v,
    const google::protobuf::RepeatedField<double>& output_v) {
  int input_v_size = input_v.size();
  int output_v_size = output_v.size();
  if (input_v_size < 1 || output_v_size < 1) {
    ADEBUG << "InterpolationOne: vector error 0";
    return 1.0;
  }
  if (input_v_size != output_v_size) {
    ADEBUG << "InterpolationOne: vector error"
           << ", input_v_size: " << input_v_size
           << ", output_v_size: " << output_v_size;
    return 1.0;
  }
  if (input_x >= input_v.Get(input_v_size - 1)) {
    return output_v.Get(input_v_size - 1);
  }
  if (input_x <= input_v.Get(0)) {
    return output_v.Get(0);
  }
  for (int a = 1; a < input_v_size; ++a) {
    if (input_x >= input_v.Get(a - 1) && input_x < input_v.Get(a)) {
      return output_v.Get(a - 1) + (output_v.Get(a) - output_v.Get(a - 1)) *
                                       (input_x - input_v.Get(a - 1)) /
                                       (input_v.Get(a) - input_v.Get(a - 1));
    }
  }
  ADEBUG << "InterpolationOne protobuf error!";
  return 1.0;
}

PointENU InterpolatePoint(const PointENU& start, const PointENU& end,
                          double length, double s) {
  PointENU new_point;
  double weight = s / length;
  double x = (1 - weight) * start.x() + weight * end.x();
  double y = (1 - weight) * start.y() + weight * end.y();
  double z = (1 - weight) * start.z() + weight * end.z();
  new_point.set_x(x);
  new_point.set_y(y);
  new_point.set_z(z);
  return new_point;
}

Vec2d InterpolatePoint(const Vec2d& start, const Vec2d& end, double length,
                       double s) {
  Vec2d new_point;
  double weight = s / length;
  double x = (1 - weight) * start.x() + weight * end.x();
  double y = (1 - weight) * start.y() + weight * end.y();
  new_point.set_x(x);
  new_point.set_y(y);
  return new_point;
}

hdmap::LineSegment InterpolatePoints(const hdmap::LineSegment& raw_points,
                                     double delta_s) {
  if (raw_points.point_size() <= 1) {
    return raw_points;
  }
  hdmap::LineSegment new_points;
  PointENU tmp_point = raw_points.point(0);
  new_points.add_point()->CopyFrom(tmp_point);
  for (int i = 0; i < raw_points.point_size() - 1; i++) {
    double s = delta_s;
    const auto& start = tmp_point;
    const auto& end = raw_points.point(i + 1);
    const double length = sqrt((start.x() - end.x()) * (start.x() - end.x()) +
                               (start.y() - end.y()) * (start.y() - end.y()));
    if (length < delta_s) {
      continue;
    }
    PointENU interpolate_point;
    while (s < length) {
      interpolate_point = InterpolatePoint(start, end, length, s);
      new_points.add_point()->CopyFrom(interpolate_point);
      s += delta_s;
    }
    tmp_point.CopyFrom(interpolate_point);
  }
  if (new_points.point_size() % 2 != 0) {
    const auto end_point = new_points.point(new_points.point_size() - 1);
    new_points.add_point()->CopyFrom(end_point);
  }
  return new_points;
}

std::vector<Vec2d> InterpolateVec2dPoints(const std::vector<Vec2d>& raw_points,
                                          double delta_s) {
  if (raw_points.size() <= 2) {
    return raw_points;
  }
  std::vector<Vec2d> new_points;
  new_points.push_back(raw_points.front());
  double sum_length = 0;
  for (int i = 0; i < raw_points.size() - 1; i++) {
    sum_length += raw_points.at(i).DistanceTo(raw_points.at(i + 1));
  }
  if (sum_length <= delta_s) {
    new_points.push_back(raw_points.back());
    return new_points;
  }
  Vec2d now_point;
  Vec2d end_point;
  double total_length = 0;
  for (int i = 0; i < raw_points.size() - 1; i++) {
    now_point = raw_points.at(i);
    end_point = raw_points.at(i + 1);
    const double point_distance = now_point.DistanceTo(end_point);
    total_length += point_distance;
    if (Compare(total_length, delta_s) <= 0) {
      continue;
    }
    Vec2d interpolate_point;
    double s = delta_s;
    /*
    **     |<-------total_length-------->|
    **     |<----------s-------->|
    **                |<-point_distance->|
    **-----*----------*----------.-------*---------
    **     ^       now_point     ^   end_point
    **     |<------delta_s------>|
    **                    interpolate_point
    */
    while (s < total_length) {
      interpolate_point = InterpolatePoint(now_point, end_point, point_distance,
                                           point_distance - (total_length - s));
      new_points.push_back(interpolate_point);
      s += delta_s;
    }
    total_length = interpolate_point.DistanceTo(end_point);
  }
  return new_points;
}
}  // namespace math
}  // namespace common
}  // namespace TL
