#ifndef PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H
#define PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H

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
 * @file
 **/

#pragma once

#include <vector>

#include "common/file/log.h"
#include "common/math/vec2d.h"

#include "proto/planning/planning.pb.h"

namespace TL {
namespace planning {
using ::google::protobuf::RepeatedPtrField;
using TL::common::TrajectoryPoint;

class DiscretizedTrajectory {
 public:
  DiscretizedTrajectory() = default;

  /**
   * Create a DiscretizedTrajectory based on protobuf message
   */
  explicit DiscretizedTrajectory(const ADCTrajectory& trajectory);

  explicit DiscretizedTrajectory(
      const RepeatedPtrField<TrajectoryPoint>& trajectory_points);

  void SetTrajectoryPoints(
      const RepeatedPtrField<TrajectoryPoint>& trajectory_points);

  void SetStopTrajectory(double x, double y, double theta, double kappa,
                         double start_time = 0.0, double acc = 0.0);

  virtual ~DiscretizedTrajectory() = default;

  virtual TrajectoryPoint StartPoint() const;

  virtual double GetTemporalLength() const;

  virtual double GetSpatialLength() const;

  virtual TrajectoryPoint Evaluate(double relative_time) const;

  size_t QueryLowerBoundPoint(double relative_time,
                              double epsilon = 1.0e-5) const;

  virtual size_t QueryNearestPoint(const common::math::Vec2d& position) const;

  double QueryMatchedRelativeTime(const common::math::Vec2d& position) const;

  size_t QueryNearestPointWithBuffer(const common::math::Vec2d& position,
                                     double buffer) const;

  virtual void AppendTrajectoryPoint(const TrajectoryPoint& trajectory_point);

  virtual TrajectoryPoint* AppendTrajectoryPoint(double relative_time);

  void PrependTrajectoryPoints(
      const RepeatedPtrField<TrajectoryPoint>& trajectory_points) {
    if (!trajectory_.empty() && trajectory_points.size() > 1) {
      ACHECK(trajectory_points.rbegin()->relative_time() <
             trajectory_.begin()->relative_time());
    }
    RepeatedPtrField<TrajectoryPoint> merge_trajectory;
    merge_trajectory.CopyFrom(trajectory_points);
    merge_trajectory.MergeFrom(trajectory_);
    trajectory_.Swap(&merge_trajectory);
  }

  const TrajectoryPoint& TrajectoryPointAt(size_t index) const;

  /**
   * @brief Get the Trajectory Segment Size object
   *
   * @param init_gear init gear of trajectory
   * @return int trajectory segment size
   */
  size_t GetTrajectorySegmentSize(
      const TL::soc::Chassis_GearPosition& init_gear =
          soc::Chassis::GEAR_PARKING) const;

  inline size_t NumOfPoints() const { return trajectory_.size(); }

  inline void clear() { trajectory_.Clear(); }

  inline bool empty() const { return trajectory_.empty(); }

  inline const auto& front() const { return *trajectory_.begin(); }

  inline const auto& back() const { return *(trajectory_.rbegin()); }

  inline auto begin() const { return trajectory_.begin(); }

  inline auto end() const { return trajectory_.end(); }

  inline int size() const { return trajectory_.size(); }

  inline const auto& at(int index) const { return trajectory_[index]; }

  inline const auto& operator[](int index) const { return trajectory_[index]; }

  inline auto* add() { return trajectory_.Add(); }

  inline auto* Mutable(int index) { return trajectory_.Mutable(index); }

 protected:
  // NOLINTBEGIN
  ::google::protobuf::RepeatedPtrField<TrajectoryPoint> trajectory_;
  // NOLINTEND
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H
