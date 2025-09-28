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

#include "planning/prediction/predictor/predictor.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "common/math/angle.h"
#include "planning/prediction/common/prediction_gflags.h"
#include "proto/prediction/feature.pb.h"

namespace TL {
namespace prediction {

using TL::common::PathPoint;
using TL::common::TrajectoryPoint;
using TL::hdmap::LaneInfo;

using TL::prediction::prediction_util::EvaluateCubicPolynomial;
using TL::prediction::prediction_util::EvaluateQuarticPolynomial;
using TL::prediction::prediction_util::GetCubicPolynomial;
using TL::prediction::prediction_util::GetQuarticPolynomial;

int Predictor::NumOfTrajectories(const Obstacle& obstacle) {
  CHECK_GT(obstacle.history_size(), 0);
  return obstacle.latest_feature().predicted_trajectory_size();
}

void Predictor::SetEqualProbability(const double total_probability,
                                    const int start_index,
                                    Obstacle* obstacle_ptr) {
  int num = NumOfTrajectories(*obstacle_ptr);
  if (num <= start_index) {
    AERROR << "trajectory size < start_idx num: " << num
           << " start_index: " << start_index;
    num = start_index + 1;
  }

  const auto prob = total_probability / static_cast<double>(num - start_index);
  for (int i = start_index; i < num; ++i) {
    obstacle_ptr->mutable_latest_feature()
        ->mutable_predicted_trajectory(i)
        ->set_probability(prob);
  }
}

void Predictor::Clear() {}

void Predictor::TrimTrajectories(
    const ADCTrajectoryContainer& adc_trajectory_container,
    Obstacle* obstacle) {
  for (auto& predicted_trajectory :
       *obstacle->mutable_latest_feature()->mutable_predicted_trajectory()) {
    TrimTrajectory(adc_trajectory_container, obstacle, &predicted_trajectory);
  }
}

bool Predictor::TrimTrajectory(
    const ADCTrajectoryContainer& adc_trajectory_container, Obstacle* obstacle,
    Trajectory* trajectory) {
  if (!adc_trajectory_container.IsProtected()) {
    ADEBUG << "Not in protection mode.";
    return false;
  }
  if (obstacle == nullptr || obstacle->history_size() == 0) {
    AERROR << "Invalid obstacle.";
    return false;
  }
  int num_of_point = trajectory->trajectory_point_size();
  if (num_of_point == 0) {
    return false;
  }
  const Feature& feature = obstacle->latest_feature();
  double vehicle_length = feature.length();
  double vehicle_heading = feature.velocity_heading();
  double forward_length =
      std::fmax(vehicle_length / 2.0 - FLAGS_distance_beyond_junction, 0.0);

  double front_x = trajectory->trajectory_point(0).path_point().x() +
                   forward_length * std::cos(vehicle_heading);
  double front_y = trajectory->trajectory_point(0).path_point().y() +
                   forward_length * std::sin(vehicle_heading);
  PathPoint front_point;
  front_point.set_x(front_x);
  front_point.set_y(front_y);
  bool front_in_junction =
      adc_trajectory_container.IsPointInJunction(front_point);

  const PathPoint& start_point = trajectory->trajectory_point(0).path_point();
  bool start_in_junction =
      adc_trajectory_container.IsPointInJunction(start_point);

  if (front_in_junction || start_in_junction) {
    return false;
  }

  int index = 0;
  while (index < num_of_point) {
    const PathPoint& point = trajectory->trajectory_point(index).path_point();
    if (adc_trajectory_container.IsPointInJunction(point)) {
      break;
    }
    ++index;
  }

  // if no intersect
  if (index == num_of_point) {
    return false;
  }

  for (int i = index; i < num_of_point; ++i) {
    trajectory->mutable_trajectory_point()->RemoveLast();
  }
  return true;
}

bool Predictor::SupposedToStop(const Feature& feature,
                               const double stop_distance,
                               double* acceleration) {
  if (stop_distance < std::max(feature.length() * 0.5, 1.0)) {
    return false;
  }
  if (stop_distance > FLAGS_distance_to_slow_down_at_stop_sign) {
    return false;
  }
  double speed = feature.speed();
  *acceleration = -speed * speed / (2.0 * stop_distance);
  return *acceleration <= -FLAGS_double_precision &&
         *acceleration >= FLAGS_vehicle_min_linear_acc;
}

const ObstacleConf::PredictorType& Predictor::predictor_type() {
  return predictor_type_;
}

void Predictor::DrawFreeMoveTrajectoryPoints(
    const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
    const Eigen::Vector2d& acc, const double theta, const double start_time,
    const double total_time, const double period, const double still_speed_th,
    prediction::Trajectory* trajectory) {
  Eigen::Matrix<double, 6, 1> state;
  state.setZero();
  state(0, 0) = 0.0;
  state(1, 0) = 0.0;
  state(2, 0) = velocity(0);
  state(3, 0) = velocity(1);

  state(4, 0) =
      common::math::Clamp(acc(0), FLAGS_vehicle_min_linear_acc_freemove,
                          FLAGS_vehicle_max_linear_acc_freemove);
  state(5, 0) =
      common::math::Clamp(acc(1), FLAGS_vehicle_min_linear_acc_freemove,
                          FLAGS_vehicle_max_linear_acc_freemove);

  Eigen::Matrix<double, 6, 6> transition;
  static constexpr double acc_damping_ratio = 0.9;
  transition.setIdentity();
  transition(0, 2) = period;
  transition(0, 4) = 0.5 * period * period;
  transition(1, 3) = period;
  transition(1, 5) = 0.5 * period * period;
  transition(2, 4) = period;
  transition(3, 5) = period;
  transition(4, 4) = acc_damping_ratio;
  transition(5, 5) = acc_damping_ratio;

  auto num = static_cast<size_t>(total_time / period);
  ::TL::prediction::prediction_util::GenerateFreeMoveTrajectoryPoints(
      &state, transition, theta, start_time, num, period, still_speed_th,
      trajectory);

  for (int i = 0; i < trajectory->trajectory_point_size(); ++i) {
    ::TL::prediction::prediction_util::TranslatePoint(
        position[0], position[1], trajectory->mutable_trajectory_point(i));
  }
}

bool Predictor::DrawStitchFreemoveTrajectory(const double period,
                                             prediction::Trajectory* trajectory,
                                             const double total_time) {
  if (trajectory->trajectory_point_size() > 0 &&
      trajectory->trajectory_point().rbegin()->relative_time() + period <
          total_time) {
    const auto& joint_point = trajectory->trajectory_point().rbegin();
    Trajectory free_move_trajectory;
    double start_time = joint_point->relative_time() + period;
    double stitch_total_time = total_time - start_time;

    Eigen::Vector2d position(joint_point->path_point().x(),
                             joint_point->path_point().y());
    double theta = joint_point->path_point().theta();
    Eigen::Vector2d velocity(joint_point->v() * cos(theta),
                             joint_point->v() * sin(theta));
    Eigen::Vector2d acc(joint_point->a() * cos(theta),
                        joint_point->a() * sin(theta));

    DrawFreeMoveTrajectoryPoints(
        position, velocity, acc, theta, start_time, stitch_total_time, period,
        FLAGS_still_obstacle_speed_threshold, &free_move_trajectory);

    trajectory->mutable_trajectory_point()->MergeFrom(
        free_move_trajectory.trajectory_point());
    return true;
  }
  return false;
}

bool Predictor::DrawTrajFollowEgoLane(const Obstacle& ego_obs,
                                      double total_time, double period,
                                      bool is_extend, Trajectory* trajectory,
                                      Obstacle* obstacle) {
  if (nullptr == obstacle || nullptr == trajectory) {
    AERROR << "obstacle_ptr or trajectory ptr is null!";
    return false;
  }

  const Feature& feature = obstacle->latest_feature();
  const Feature& ego_feature = ego_obs.latest_feature();
  if (!feature.has_position() || !feature.has_velocity() ||
      !feature.position().has_x() || !feature.position().has_y()) {
    AERROR << "Obstacle [" << obstacle->id()
           << " is missing position or velocity";
    return false;
  }

  const LaneSequence* ego_lane_sequence_ptr = nullptr;

  for (const auto& sequence : ego_feature.lane().lane_graph().lane_sequence()) {
    if (sequence.vehicle_on_lane()) {
      ego_lane_sequence_ptr = &sequence;
      break;
    }
  }

  if (ego_lane_sequence_ptr == nullptr ||
      ego_lane_sequence_ptr->lane_segment().empty() ||
      (ego_lane_sequence_ptr->has_adc_lane_segment_idx() &&
       ego_lane_sequence_ptr->adc_lane_segment_idx() + 1 >
           ego_lane_sequence_ptr->lane_segment_size())) {
    if (ego_lane_sequence_ptr == nullptr) {
      AERROR
          << "Obstacle " << obstacle->id()
          << " Failed in getting ego lane : ego_lane_sequence_ptr is nullptr";
    } else {
      AERROR << "Obstacle " << obstacle->id()
             << " Failed in getting ego lane : ego_lane_sequence_ptr: "
             << ego_lane_sequence_ptr->ShortDebugString();
    }
    return false;
  }

  int lane_segment_index = ego_lane_sequence_ptr->has_adc_lane_segment_idx()
                               ? ego_lane_sequence_ptr->adc_lane_segment_idx()
                               : 0;

  auto lane_id =
      ego_lane_sequence_ptr->lane_segment(lane_segment_index).lane_id();
  std::shared_ptr<const LaneInfo> ego_lane_info =
      PredictionMap::LaneById(lane_id);
  if (ego_lane_info == nullptr) {
    AERROR << "Failed in getting lane : " << lane_id;
    return false;
  }

  double start_x = feature.position().x();
  double start_y = feature.position().y();
  double v = feature.speed();
  double a = feature.acc();

  // if not extend , clear the trajectory,add current point as start point
  // else, use the last point of the trajectory as the start point
  if (!is_extend) {
    trajectory->clear_trajectory_point();
  }
  if (is_extend && !trajectory->trajectory_point().empty()) {
    start_x = trajectory->trajectory_point().rbegin()->path_point().x();
    start_y = trajectory->trajectory_point().rbegin()->path_point().y();
    v = trajectory->trajectory_point().rbegin()->v();
    a = trajectory->trajectory_point().rbegin()->a();
  }

  double lane_s = 0.0;
  double lane_l = 0.0;
  Eigen::Vector2d position(start_x, start_y);
  if (!PredictionMap::GetProjection(position, ego_lane_info, &lane_s,
                                    &lane_l)) {
    AERROR << "Failed in getting lane s and lane l";
    return false;
  }

  while (lane_s > ego_lane_info->total_length() &&
         lane_segment_index + 1 < ego_lane_sequence_ptr->lane_segment_size()) {
    lane_segment_index += 1;
    lane_id = ego_lane_sequence_ptr->lane_segment(lane_segment_index).lane_id();
    ego_lane_info = PredictionMap::LaneById(lane_id);
    if (ego_lane_info == nullptr) {
      AERROR << "Failed in getting lane : " << lane_id;
      return false;
    }
    if (!PredictionMap::GetProjection(position, ego_lane_info, &lane_s,
                                      &lane_l)) {
      AERROR << "Failed in getting lane s and lane l";
      return false;
    }
  }

  double ego_lane_heading = PredictionMap::HeadingOnLane(ego_lane_info, lane_s);
  double obs_heading = feature.velocity_heading();

  double angle_diff =
      std::abs(common::math::AngleDiff(obs_heading, ego_lane_heading));

  if (!is_extend || trajectory->trajectory_point_size() == 0) {
    TrajectoryPoint* start_trajectory_point =
        trajectory->add_trajectory_point();
    PathPoint* start_path_point = start_trajectory_point->mutable_path_point();
    start_path_point->set_x(feature.position().x());
    start_path_point->set_y(feature.position().y());
    start_path_point->set_theta(obs_heading);
    start_path_point->set_lane_id(lane_id);
    start_trajectory_point->set_v(v);
    start_trajectory_point->set_a(a);
    start_trajectory_point->set_relative_time(0.0);
    // forward 45°，reverse 11°
    if (angle_diff > FLAGS_max_lane_angle_diff &&
        (angle_diff < M_PI - FLAGS_max_lane_angle_diff / 4)) {
      return false;
    }
  }

  // calculate the lateral and longitudinal trajectory
  double dl0 = v * std::sin(obs_heading - ego_lane_heading);
  // determine end state
  double left = 0.0;
  double right = 0.0;
  ego_lane_info->GetWidth(lane_s, &left, &right);
  double lane_width = left + right;

  double end_l = 0.0;
  double time_to_lat_end_state = FLAGS_default_time_to_lat_end_state;
  std::array<double, 4> lateral_coeffs{};
  std::array<double, 5> longitudinal_coeffs{};
  double virtual_lane_idx = std::floor(lane_l / lane_width + 0.5);
  static constexpr double dl_limit = 1.0;  // need revise by data analysis
  double lane_change_bias =
      common::math::Clamp(std::floor(dl0 / dl_limit + 0.5), -1.0, 1.0);
  if (feature.has_lane() && feature.lane().has_lane_graph() &&
      feature.lane().lane_graph().lane_sequence_size() == 1 &&
      !ego_lane_sequence_ptr->has_merge_lane_idx()) {
    const auto& seq = feature.lane().lane_graph().lane_sequence(0);
    if (PredictionMap::IsSequenceContainsAnother(seq, *ego_lane_sequence_ptr) &&
        seq.has_probability() && seq.probability() < 0.4) {
      lane_change_bias = 0.0;
    }
  }
  end_l = (virtual_lane_idx + lane_change_bias) * lane_width;
  if (feature.speed() < FLAGS_slow_obstacle_speed_threshold) {
    end_l = lane_l;
  }
  double avg_lv =
      std::max(FLAGS_default_lateral_approach_speed, std::fabs(0.5 * dl0));
  time_to_lat_end_state =
      std::min(time_to_lat_end_state,
               std::max(std::fabs((end_l - lane_l) / avg_lv), 1.0));
  GetCubicPolynomial(lane_l, dl0, end_l, 0.0, time_to_lat_end_state,
                     &lateral_coeffs);

  auto* output_feature = obstacle->mutable_latest_feature();
  auto* behavior_info = output_feature->add_behavior_infos();
  if (std::isless(std::fabs(end_l), left) &&
      std::isless(std::fabs(end_l), right)) {
    behavior_info->set_targetlane_id(ego_lane_info->id().id());
    if (std::isless(std::fabs(lane_l), left) &&
        std::isless(std::fabs(lane_l), right)) {
      behavior_info->mutable_intent()->set_type(ObstacleIntent::NO_CHANGE);
    } else if (std::isgreater(std::fabs(lane_l), 0.0)) {
      behavior_info->mutable_intent()->set_type(ObstacleIntent::RIGHT_CHANGE);
    } else {
      behavior_info->mutable_intent()->set_type(ObstacleIntent::LEFT_CHANGE);
    }
  }

  double s0 = 0.0;
  double ds0 = v * std::cos(obs_heading - ego_lane_heading);
  double dds0 = a * std::cos(obs_heading - ego_lane_heading);
  auto lon_end_vt = GetQuarticPolynomial(s0, ds0, dds0, &longitudinal_coeffs);
  double ds1 = lon_end_vt.first;
  double time_to_lon_end_state = lon_end_vt.second;
  double prev_s = 0.0;
  // Draw each trajectory point within the total time of prediction
  auto total_num = static_cast<size_t>(total_time / period);

  // double trajectory_point_theta = obs_heading;
  size_t traj_index = trajectory->trajectory_point_size();
  for (; traj_index < total_num; ++traj_index) {
    double relative_time = static_cast<double>(traj_index) * period;
    Eigen::Vector2d point;
    double theta = M_PI;
    lane_l = EvaluateCubicPolynomial(lateral_coeffs, relative_time, 0,
                                     time_to_lat_end_state, 0.0);
    double curr_s = EvaluateQuarticPolynomial(
        longitudinal_coeffs, relative_time, 0, time_to_lon_end_state, ds1);
    double lane_speed = EvaluateQuarticPolynomial(
        longitudinal_coeffs, relative_time, 1, time_to_lon_end_state, ds1);

    lane_s += curr_s - prev_s;
    prev_s = curr_s;
    while (lane_s > ego_lane_info->total_length() &&
           lane_segment_index + 1 <
               ego_lane_sequence_ptr->lane_segment_size()) {
      lane_segment_index += 1;
      ego_lane_info = PredictionMap::LaneById(lane_id);
      if (ego_lane_info == nullptr) {
        return false;
      }
      lane_s = lane_s - ego_lane_info->total_length();
      lane_id =
          ego_lane_sequence_ptr->lane_segment(lane_segment_index).lane_id();
      ego_lane_info = PredictionMap::LaneById(lane_id);
      if (ego_lane_info == nullptr) {
        return false;
      }
    }

    if (lane_segment_index + 1 >= ego_lane_sequence_ptr->lane_segment_size() &&
        lane_s > ego_lane_info->total_length()) {
      return false;
    }
    if (lane_speed * ds0 < FLAGS_double_precision) {
      return false;
    }
    if (!PredictionMap::SmoothPointFromLane(lane_id, lane_s, lane_l, &point,
                                            &theta)) {
      AERROR << "Unable to get smooth point from lane [" << lane_id
             << "] with s [" << lane_s << "] and l [" << lane_l << "]";
      return false;
    }

    if (lane_s < 0) {
      point.x() += lane_s * std::cos(theta);
      point.y() += lane_s * std::sin(theta);
    }

    if (std::isnan(point.x()) || std::isnan(point.y())) {
      continue;
    }

    const auto& prev_point = trajectory->trajectory_point().rbegin();
    auto dy = static_cast<double>(point.y() - prev_point->path_point().y());
    auto dx = static_cast<double>(point.x() - prev_point->path_point().x());
    auto trajectory_point_theta = static_cast<double>(
        common::math::atan2(static_cast<float>(dy), static_cast<float>(dx)));
    v = std::hypot(dy, dx) / period;
    a = (v - prev_point->v()) / period;
    TrajectoryPoint* trajectory_point = trajectory->add_trajectory_point();
    PathPoint* path_point = trajectory_point->mutable_path_point();
    path_point->set_x(point.x());
    path_point->set_y(point.y());
    path_point->set_theta(trajectory_point_theta);
    path_point->set_lane_id(lane_id);
    trajectory_point->set_v(v);
    trajectory_point->set_a(a);
    trajectory_point->set_relative_time(relative_time);
  }

  auto* target_point = behavior_info->mutable_target_point();
  if (!trajectory->trajectory_point().empty()) {
    int last_point_idx = trajectory->trajectory_point_size() - 1;
    target_point->set_x(
        trajectory->mutable_trajectory_point(last_point_idx)->path_point().x());
    target_point->set_y(
        trajectory->mutable_trajectory_point(last_point_idx)->path_point().y());
    target_point->set_z(
        trajectory->mutable_trajectory_point(last_point_idx)->path_point().z());
  }

  return (traj_index > total_num - 1);
}

void Predictor::ExtrapolateByLane(double obs_speed,
                                  const std::string& start_lane_id,
                                  Trajectory* trajectory_ptr) {
  if (nullptr == trajectory_ptr) {
    return;
  }

  double need_length = obs_speed * FLAGS_prediction_trajectory_time_length;

  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lane_id_que;
  const auto& lane_info = PredictionMap::LaneById(start_lane_id);
  if (nullptr == lane_info) {
    return;
  }

  int size = trajectory_ptr->trajectory_point_size();
  double last_relative_time = 0.0;
  if (size > 0) {
    const auto& last_point = trajectory_ptr->trajectory_point(size - 1);
    last_relative_time = last_point.relative_time();
  }

  double time_resolution = FLAGS_prediction_trajectory_time_resolution;
  double time_range =
      FLAGS_prediction_trajectory_time_length - last_relative_time;

  int num_point_remained = static_cast<int>(time_range / time_resolution);
  if (num_point_remained <= 0) {
    return;
  }

  lane_id_que.push_back(lane_info);
  double que_length = lane_info->total_length();
  int loop_time = 0;
  while (que_length < need_length && loop_time < 20) {
    const auto& lane_info_ptr = lane_id_que.back();
    if (lane_info_ptr->lane().successor_id_size() <= 0) {
      break;
    }
    const auto& successor_id = lane_info_ptr->lane().successor_id(0).id();
    const auto& successor_info_ptr = PredictionMap::LaneById(successor_id);
    if (nullptr == successor_info_ptr) {
      break;
    }

    lane_id_que.push_back(successor_info_ptr);
    que_length += successor_info_ptr->total_length();

    ++loop_time;
  }
  int lane_segment_index = 0;
  std::string lane_id = lane_id_que.at(lane_segment_index)->id().id();

  double lane_s = 0.0;
  double lane_l = 0.0;
  auto& lane_info_ptr = lane_id_que.at(lane_segment_index);
  for (int i = 1; i <= num_point_remained; ++i) {
    double relative_time =
        last_relative_time + static_cast<double>(i) * time_resolution;
    Eigen::Vector2d point;
    double theta = M_PI;
    if (!PredictionMap::SmoothPointFromLane(lane_id, lane_s, lane_l, &point,
                                            &theta)) {
      AERROR << "Unable to get smooth point from lane [" << lane_id
             << "] with s [" << lane_s << "] and l [" << lane_l << "]";
      break;
    }
    TrajectoryPoint* trajectory_point = trajectory_ptr->add_trajectory_point();
    PathPoint* path_point = trajectory_point->mutable_path_point();
    path_point->set_x(point.x());
    path_point->set_y(point.y());
    path_point->set_z(0.0);
    path_point->set_theta(theta);
    path_point->set_lane_id(lane_id);
    trajectory_point->set_v(obs_speed);
    trajectory_point->set_a(0.0);
    trajectory_point->set_relative_time(relative_time);

    lane_s += obs_speed * time_resolution;

    while (lane_s > lane_info_ptr->total_length() &&
           lane_segment_index + 1 < lane_id_que.size()) {
      lane_segment_index += 1;
      lane_s = lane_s - lane_info_ptr->total_length();
      lane_id = lane_id_que.at(lane_segment_index)->id().id();
      lane_info_ptr = lane_id_que.at(lane_segment_index);
    }

    lane_l *= FLAGS_go_approach_rate;
  }
}

bool Predictor::DrawCubicBezierTrajectory(const Feature& feature,
                                          const Eigen::Vector2d& end_p,
                                          double end_heading, double total_time,
                                          double period,
                                          Trajectory* trajectory) {
  if (nullptr == trajectory) {
    return false;
  }
  trajectory->clear_trajectory_point();

  double speed = feature.speed();
  Eigen::Vector2d p3_v(speed * std::cos(end_heading),
                       speed * std::sin(end_heading));
  Eigen::Vector2d p0_v(feature.velocity().x(), feature.velocity().y());

  Eigen::Vector2d p0(feature.position().x(), feature.position().y());
  const Eigen::Vector2d& p3 = end_p;

  double ctrl_dist = std::hypot(p0.x() - p3.x(), p0.y() - p3.y());
  if (ctrl_dist < 6.0) {
    return false;
  }
  ctrl_dist = ctrl_dist / 2.0;

  Eigen::Vector2d p1(p0.x() + ctrl_dist * std::cos(feature.velocity_heading()),
                     p0.y() + ctrl_dist * std::sin(feature.velocity_heading()));
  Eigen::Vector2d p2(p3.x() - ctrl_dist * std::cos(end_heading),
                     p3.y() - ctrl_dist * std::sin(end_heading));

  std::vector<Eigen::Vector2d> control_points{p0, p1, p2, p3};

  double bezier_length = 0.0;
  Eigen::Vector2d prev_point = p0;
  for (int i = 0; i < 101; ++i) {
    double t = i * 0.01;
    const auto& curr_point =
        prediction_util::EvaluateCubicBezierPoint(control_points, t);

    double delta_dist = std::hypot(curr_point.x() - prev_point.x(),
                                   curr_point.y() - prev_point.y());

    bezier_length += delta_dist;

    prev_point = curr_point;
  }

  double estimate_t = bezier_length / std::max(0.2, speed);
  double traj_time = total_time > estimate_t ? estimate_t : total_time;

  int num = static_cast<int>(traj_time / period);
  for (int i = 0; i < num; ++i) {
    double t = i * period / estimate_t;
    const auto& bezier_p =
        prediction_util::EvaluateCubicBezierPoint(control_points, t);

    auto* trajectory_point = trajectory->add_trajectory_point();
    auto* path_point = trajectory_point->mutable_path_point();
    path_point->set_x(bezier_p.x());
    path_point->set_y(bezier_p.y());

    path_point->set_theta(feature.velocity_heading());
    trajectory_point->set_v(feature.speed());
    trajectory_point->set_a(feature.acc());
    trajectory_point->set_relative_time(i * period);
  }
  return true;
}

}  // namespace prediction
}  // namespace TL
