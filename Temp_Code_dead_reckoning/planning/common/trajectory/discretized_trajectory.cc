/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file discretized_trajectory.cc
 **/

#include "planning/common/trajectory/discretized_trajectory.h"

#include <cmath>
#include <limits>

#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "planning/common/planning_gflags.h"

namespace TL {
namespace planning {

using ::google::protobuf::RepeatedPtrField;
using TL::common::TrajectoryPoint;

DiscretizedTrajectory::DiscretizedTrajectory(
    const RepeatedPtrField<TrajectoryPoint>& trajectory_points)
    : trajectory_(trajectory_points) {
  ACHECK(!trajectory_points.empty())
      << "trajectory_points should NOT be empty()";
}

DiscretizedTrajectory::DiscretizedTrajectory(const ADCTrajectory& trajectory) {
  trajectory_ =
      RepeatedPtrField<TrajectoryPoint>(trajectory.trajectory_point().begin(),
                                        trajectory.trajectory_point().end());
}

void DiscretizedTrajectory::SetStopTrajectory(const double x, const double y,
                                              const double theta,
                                              const double kappa,
                                              const double start_time,
                                              const double acc) {
  trajectory_.Clear();
  trajectory_.Reserve(FLAGS_publish_trajectory_points_number);
  TrajectoryPoint tp;
  auto* path_point = tp.mutable_path_point();
  path_point->set_x(x);
  path_point->set_y(y);
  path_point->set_theta(theta);
  path_point->set_kappa(kappa);
  path_point->set_s(0.0);
  tp.set_v(0.0);
  tp.set_a(acc);
  double t = start_time;
  for (int i = 0; i < FLAGS_publish_trajectory_points_number; ++i) {
    tp.set_relative_time(t);
    trajectory_.Add()->CopyFrom(tp);
    t += FLAGS_fallback_time_unit;
  }
}

TrajectoryPoint DiscretizedTrajectory::Evaluate(
    const double relative_time) const {
  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  auto it_lower = std::lower_bound(trajectory_.begin(), trajectory_.end(),
                                   relative_time, comp);

  if (it_lower == trajectory_.begin()) {
    return *trajectory_.begin();
  }
  if (it_lower == trajectory_.end()) {
    AWARN << "When evaluate trajectory, relative_time(" << relative_time
          << ") is too large";
    return *trajectory_.rbegin();
  }
  return common::math::InterpolateUsingLinearApproximation(
      *(it_lower - 1), *it_lower, relative_time);
}

size_t DiscretizedTrajectory::QueryLowerBoundPoint(const double relative_time,
                                                   const double epsilon) const {
  ACHECK(!trajectory_.empty());

  if (relative_time >= trajectory_.rbegin()->relative_time()) {
    return trajectory_.size() - 1;
  }
  auto func = [&epsilon](const TrajectoryPoint& tp,
                         const double relative_time) {
    return tp.relative_time() + epsilon < relative_time;
  };
  auto it_lower = std::lower_bound(trajectory_.begin(), trajectory_.end(),
                                   relative_time, func);
  return std::distance(trajectory_.begin(), it_lower);
}

size_t DiscretizedTrajectory::QueryNearestPoint(
    const common::math::Vec2d& position) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (int i = 0; i < trajectory_.size(); ++i) {
    const common::math::Vec2d curr_point(trajectory_[i].path_point().x(),
                                         trajectory_[i].path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

size_t DiscretizedTrajectory::QueryNearestPointWithBuffer(
    const common::math::Vec2d& position, const double buffer) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (int i = 0; i <= trajectory_.size() - 1; ++i) {
    const common::math::Vec2d curr_point(trajectory_[i].path_point().x(),
                                         trajectory_[i].path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min - buffer) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

double DiscretizedTrajectory::QueryMatchedRelativeTime(
    const common::math::Vec2d& position) const {
  int nearest_index =
      static_cast<int>(QueryNearestPointWithBuffer(position, 1e-6));
  const auto& nearest_point = at(nearest_index);
  const auto& theta = nearest_point.path_point().theta();

  common::math::Vec2d v = {nearest_point.path_point().x(),
                           nearest_point.path_point().y()};
  common::math::Vec2d n(std::cos(theta), std::sin(theta));

  const double delta_s = n.InnerProd(position - v);
  double matched_time = nearest_point.relative_time();
  if (!common::math::double_type::IsZero(nearest_point.v())) {
    matched_time += delta_s / nearest_point.v();
  }
  const auto next_time = (nearest_index + 1 < size())
                             ? at(nearest_index + 1).relative_time()
                             : back().relative_time();
  return common::math::Clamp(matched_time, front().relative_time(), next_time);
}

void DiscretizedTrajectory::AppendTrajectoryPoint(
    const TrajectoryPoint& trajectory_point) {
  if (!trajectory_.empty()) {
    CHECK_GT(trajectory_point.relative_time(),
             trajectory_.rbegin()->relative_time());
  }
  trajectory_.Add()->CopyFrom(trajectory_point);
}

TrajectoryPoint* DiscretizedTrajectory::AppendTrajectoryPoint(
    const double relative_time) {
  if (!trajectory_.empty()) {
    CHECK_GT(relative_time, trajectory_.rbegin()->relative_time());
  }
  return trajectory_.Add();
}

const TrajectoryPoint& DiscretizedTrajectory::TrajectoryPointAt(
    const size_t index) const {
  CHECK_LT(index, NumOfPoints());
  return trajectory_[static_cast<int>(index)];
}

TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
  ACHECK(!trajectory_.empty());
  return *trajectory_.begin();
}

double DiscretizedTrajectory::GetTemporalLength() const {
  if (trajectory_.empty()) {
    return 0.0;
  }
  return trajectory_.begin()->relative_time() -
         trajectory_.rbegin()->relative_time();
}

double DiscretizedTrajectory::GetSpatialLength() const {
  if (trajectory_.empty()) {
    return 0.0;
  }
  return trajectory_.rbegin()->path_point().s() -
         trajectory_.begin()->path_point().s();
}

size_t DiscretizedTrajectory::GetTrajectorySegmentSize(
    const TL::soc::Chassis_GearPosition& init_gear) const {
  size_t trajectorie_segment_size = 0;
  auto gear = [](const common::TrajectoryPoint& from_point,
                 const common::TrajectoryPoint& to_point) {
    double diff_angle = std::fabs(common::math::NormalizeAngle(
        std::atan2(to_point.path_point().y() - from_point.path_point().y(),
                   to_point.path_point().x() - from_point.path_point().x()) -
        from_point.path_point().theta()));
    if (diff_angle < M_PI_2) {
      return soc::Chassis::GEAR_DRIVE;
    }
    return soc::Chassis::GEAR_REVERSE;
  };
  if (NumOfPoints() > 2) {
    auto current_gear_status = init_gear;
    for (int i = 1; i < NumOfPoints(); ++i) {
      const auto gear_position = gear(trajectory_[i - 1], trajectory_[i]);
      if (current_gear_status != gear_position) {
        if (current_gear_status == soc::Chassis::GEAR_DRIVE ||
            current_gear_status == soc::Chassis::GEAR_REVERSE) {
          ++trajectorie_segment_size;
        }
        current_gear_status = gear_position;
      }
    }
  }

  return trajectorie_segment_size;
}

}  // namespace planning
}  // namespace TL
