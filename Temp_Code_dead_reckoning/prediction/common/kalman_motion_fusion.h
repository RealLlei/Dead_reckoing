/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <deque>
#include <string>
#include <vector>

#include "planning/prediction/common/kalman_filter.h"

#include "proto/perception/perception_obstacle.pb.h"
#include "proto/prediction/feature.pb.h"

namespace TL {
namespace prediction {

class KalmanMotionFusion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  KalmanMotionFusion() = default;

  ~KalmanMotionFusion() = default;

  KalmanMotionFusion(const KalmanMotionFusion&) = delete;
  KalmanMotionFusion& operator=(const KalmanMotionFusion&) = delete;

  // @brief init kalman filter and some magic number
  bool InitFilter(const perception::PerceptionObstacle& measurement,
                  double timestamp);

  // @brief update the tracker with current measurement
  // @params[IN] measurement: sensor results
  // @params[IN] target_timestamp: tracker timestamp
  void UpdateWithMeasurement(const perception::PerceptionObstacle& measurement,
                             double target_timestamp,
                             prediction::Feature* feature);

  void GetStates(Eigen::Vector3d* anchor_point, Eigen::Vector3d* velocity);

  void DeInitFilter();

 private:
  void MotionFusionWithMeasurement(
      const perception::PerceptionObstacle& measurement, double time_diff);

  void UpdateSensorHistory(const double& timestamp);

 private:
  bool filter_init_ = false;
  std::deque<double> history_timestamp_;
  perception::PerceptionObstacle last_measurement_;
  KalmanFilter kalman_filter_;
  Eigen::Matrix3f center_uncertainty_ = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f velo_uncertainty_ = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f acc_uncertainty_ = Eigen::Matrix3f::Zero();

  static int s_eval_window_;
  static size_t s_history_size_maximum_;
};

}  // namespace prediction
}  // namespace TL
