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

#include <algorithm>
#include <iomanip>
#include <limits>

#include "common/file/log.h"
#include "planning/prediction/common/kalman_motion_fusion.h"
#include "planning/prediction/common/prediction_gflags.h"
#include "proto/prediction/feature.pb.h"

namespace TL {
namespace prediction {

int KalmanMotionFusion::s_eval_window_ = 3;
size_t KalmanMotionFusion::s_history_size_maximum_ = 20;

bool KalmanMotionFusion::InitFilter(
    const perception::PerceptionObstacle& measurement, double timestamp) {
  UNUSED(timestamp);
  const std::vector<bool> gain_break_down = {false, false, false,
                                             false, true,  true};
  const std::vector<bool> value_break_down = {false, false, true,
                                              true,  false, false};
  const float gain_break_down_threshold = 2.0F;
  const float value_break_down_threshold = 0.05F;
  Eigen::MatrixXd global_uncertainty;
  Eigen::VectorXd global_states;
  // global_states: center(2), velocity(2), acceleration(2)
  global_uncertainty.setIdentity(6, 6);
  global_states.setZero(6, 1);

  global_states(0) = measurement.position().x();
  global_states(1) = measurement.position().y();
  global_states(2) = measurement.velocity().x();
  global_states(3) = measurement.velocity().y();
  if (measurement.has_acceleration()) {
    global_states(4) = measurement.acceleration().x();
    global_states(5) = measurement.acceleration().y();
  } else {
    global_states(4) = 0.0;
    global_states(5) = 0.0;
  }

  ADEBUG << "global_states@(" << SETPRECISION(10) << global_states(0) << ","
         << global_states(1) << "," << global_states(2) << ","
         << global_states(3) << "," << global_states(4) << ","
         << global_states(5) << ")";

  if (!kalman_filter_.Init(global_states, global_uncertainty)) {
    return false;
  }
  if (!kalman_filter_.SetGainBreakdownThresh(gain_break_down,
                                             gain_break_down_threshold) ||
      !kalman_filter_.SetValueBreakdownThresh(value_break_down,
                                              value_break_down_threshold)) {
    return false;
  }

  return true;
}

void KalmanMotionFusion::DeInitFilter() {
  filter_init_ = false;
  history_timestamp_.clear();
}

void KalmanMotionFusion::UpdateWithMeasurement(
    const perception::PerceptionObstacle& measurement, double target_timestamp,
    prediction::Feature* feature) {
  // Motion fusion
  if (filter_init_) {
    double time_diff = target_timestamp - history_timestamp_.back();
    MotionFusionWithMeasurement(measurement, time_diff);
  } else {
    filter_init_ = InitFilter(measurement, measurement.timestamp());
  }
  last_measurement_.CopyFrom(measurement);
  UpdateSensorHistory(target_timestamp);

  if (!filter_init_ || feature == nullptr) {
    // No kalman result, no matter which sensortype
    // of measurement, use measurement's
    // anchor point and velocity
    return;
  }
  feature->mutable_position()->set_x(kalman_filter_.GetStates()(0));
  feature->mutable_position()->set_y(kalman_filter_.GetStates()(1));
  feature->mutable_velocity()->set_x(kalman_filter_.GetStates()(2));
  feature->mutable_velocity()->set_y(kalman_filter_.GetStates()(3));

  feature->mutable_raw_position()->set_x(measurement.position().x());
  feature->mutable_raw_position()->set_y(measurement.position().y());
  feature->mutable_raw_velocity()->set_x(measurement.velocity().x());
  feature->mutable_raw_velocity()->set_y(measurement.velocity().y());
  if (measurement.has_acceleration()) {
    feature->mutable_t_acceleration()->set_x(measurement.acceleration().x());
    feature->mutable_t_acceleration()->set_y(measurement.acceleration().y());
  } else {
    feature->mutable_t_acceleration()->set_x(0.0);
    feature->mutable_t_acceleration()->set_y(0.0);
  }
  if (measurement.has_position_flu()) {
    feature->mutable_position_flu()->CopyFrom(measurement.position_flu());
  }
  if (measurement.has_velocity_flu()) {
    feature->mutable_velocity_flu()->CopyFrom(measurement.velocity_flu());
  }

  // Prevent acc
  double velocity_heading =
      std::atan2(feature->velocity().y(), feature->velocity().x());
  double acc_x = kalman_filter_.GetStates()(4);
  double acc_y = kalman_filter_.GetStates()(5);
  double acc = acc_x * cos(velocity_heading) + acc_y * sin(velocity_heading);
  if (acc > FLAGS_vehicle_max_linear_acc) {
    double coefficient = FLAGS_vehicle_max_linear_acc / acc;
    acc_x = acc_x * coefficient;
    acc_y = acc_y * coefficient;
  } else if (acc < FLAGS_vehicle_min_linear_acc) {
    double coefficient = FLAGS_vehicle_min_linear_acc / acc;
    acc_x = acc_x * coefficient;
    acc_y = acc_y * coefficient;
  }

  // acc :  projected to the direction of velocity heading
  acc = acc_x * std::cos(velocity_heading) + acc_y * std::sin(velocity_heading);

  feature->set_speed(
      std::hypot(feature->velocity().x(), feature->velocity().y()));
  feature->set_velocity_heading(velocity_heading);
  feature->set_acc(acc);
  feature->mutable_acceleration()->set_x(acc_x);
  feature->mutable_acceleration()->set_y(acc_y);

  ADEBUG << "Obstacle [" << measurement.id()
         << "] has filter position:" << feature->position().DebugString()
         << ", raw position:" << feature->raw_position().DebugString() << ".";
  ADEBUG << "Obstacle [" << measurement.id() << "] has filter velocity ["
         << FIXED << SETPRECISION(6) << feature->velocity().x() << ", "
         << feature->velocity().y() << "]"
         << ", raw velocity [" << feature->raw_velocity().x() << ", "
         << feature->raw_velocity().y() << "] has velocity heading ["
         << feature->velocity_heading() << "] "
         << "] has speed [" << feature->speed() << "].";
  ADEBUG << "has acceleration [" << FIXED << SETPRECISION(6)
         << feature->acceleration().x() << ", " << feature->acceleration().y()
         << "] ,has acceleration value [" << feature->acc() << "].";
}

void KalmanMotionFusion::MotionFusionWithMeasurement(
    const perception::PerceptionObstacle& measurement, double time_diff) {
  // we use kalman filter to update our tracker.
  // The pipeline is detailed as follows:
  // 1) compute the time diff to predict the tracker
  //    (although we introduce the acceleration, we
  //    doesn't use it to update the position)
  // 2) DeCorrelation the uncertainty matrix (we belief
  //    that the velocity won`t be affected by position)
  // 3) compute the acceleration of the measurement
  // 4) use the history radar or lidar(depend on which sensor
  //    type in current) to correct the observation
  // 5) set r_matrix according to converged or not
  // 6) use kalman to correct the predict before
  // 7) use correction breakdown to eliminate the unreasonable
  //    acceleration gain or velocity noise
  Eigen::MatrixXd transform_matrix;
  Eigen::MatrixXd env_uncertainty;

  transform_matrix.setIdentity(6, 6);
  transform_matrix(0, 2) = time_diff;
  transform_matrix(1, 3) = time_diff;
  transform_matrix(2, 4) = time_diff;
  transform_matrix(3, 5) = time_diff;
  transform_matrix(0, 4) = 0.5 * time_diff * time_diff;
  transform_matrix(1, 5) = 0.5 * time_diff * time_diff;

  env_uncertainty.setIdentity(6, 6);
  env_uncertainty *= 0.5;

  kalman_filter_.Predict(transform_matrix, env_uncertainty);

  Eigen::VectorXd observation;
  observation.setZero(6, 1);
  observation(0) = measurement.position().x();
  observation(1) = measurement.position().y();
  observation(2) = measurement.velocity().x();
  observation(3) = measurement.velocity().y();
  if (measurement.has_acceleration()) {
    observation(4) = measurement.acceleration().x();
    observation(5) = measurement.acceleration().y();
  } else {
    observation(4) = kalman_filter_.GetStates()(4);
    observation(5) = kalman_filter_.GetStates()(5);
  }

  Eigen::MatrixXd r_matrix;
  r_matrix.setIdentity(6, 6);

  ADEBUG << "obstacle [" << measurement.id()
         << "],fusion_original_measurement@(" << SETPRECISION(10)
         << observation(0) << "," << observation(1) << "," << observation(2)
         << "," << observation(3) << "," << observation(4) << ","
         << observation(5) << "),time_diff : " << time_diff;
  ADEBUG << "fusion_original_measurement_covariance@(" << r_matrix(0, 0) << ","
         << r_matrix(0, 1) << "," << r_matrix(1, 0) << "," << r_matrix(1, 1)
         << "," << r_matrix(2, 2) << "," << r_matrix(2, 3) << ","
         << r_matrix(3, 2) << "," << r_matrix(3, 3) << ")";

  // Adapt noise level to rewarding status
  const auto converged_scale = static_cast<float>(FLAGS_converged_scale);
  const auto unconverged_scale = static_cast<float>(FLAGS_unconverged_scale);
  r_matrix.setIdentity();
  r_matrix.block<2, 2>(0, 0) *= converged_scale;
  r_matrix.block<2, 2>(2, 2) *= unconverged_scale;
  r_matrix.block<2, 2>(4, 4) *= unconverged_scale;

  ADEBUG << "fusion_pseudo_measurement@(" << SETPRECISION(10) << observation(0)
         << "," << observation(1) << "," << observation(2) << ","
         << observation(3) << ")";
  ADEBUG << "fusion_pseudo_measurement_covariance@(" << r_matrix(0, 0) << ","
         << r_matrix(0, 1) << "," << r_matrix(1, 0) << "," << r_matrix(1, 1)
         << "," << r_matrix(2, 2) << "," << r_matrix(2, 3) << ","
         << r_matrix(3, 2) << "," << r_matrix(3, 3) << ")";

  kalman_filter_.DeCorrelation(2, 0, 2, 2);
  kalman_filter_.Correct(observation, r_matrix);
  kalman_filter_.CorrectionBreakdown();

  ADEBUG << "fusion_filter_belief@(" << SETPRECISION(10)
         << kalman_filter_.GetStates()(0) << ","
         << kalman_filter_.GetStates()(1) << ","
         << kalman_filter_.GetStates()(2) << ","
         << kalman_filter_.GetStates()(3) << ","
         << kalman_filter_.GetStates()(4) << ","
         << kalman_filter_.GetStates()(5) << ")";

  ADEBUG << "fusion_filter_belief_covariance@("
         << kalman_filter_.GetUncertainty()(0, 0) << ","
         << kalman_filter_.GetUncertainty()(0, 1) << ","
         << kalman_filter_.GetUncertainty()(1, 0) << ","
         << kalman_filter_.GetUncertainty()(1, 1) << ","
         << kalman_filter_.GetUncertainty()(2, 2) << ","
         << kalman_filter_.GetUncertainty()(2, 3) << ","
         << kalman_filter_.GetUncertainty()(3, 2) << ","
         << kalman_filter_.GetUncertainty()(3, 3) << ")"
         << "\n---------------";
}

void KalmanMotionFusion::UpdateSensorHistory(const double& timestamp) {
  if (history_timestamp_.size() > s_history_size_maximum_) {
    history_timestamp_.pop_front();
  }
  history_timestamp_.push_back(timestamp);
}

}  // namespace prediction
}  // namespace TL
