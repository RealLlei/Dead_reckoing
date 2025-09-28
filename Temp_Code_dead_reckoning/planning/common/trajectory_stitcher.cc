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

#include "planning/common/trajectory_stitcher.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>

#include "absl/strings/str_cat.h"
#include "common/configs/config_gflags.h"
#include "common/file/log.h"
#include "common/math/angle.h"
#include "common/math/double_type.h"
#include "common/math/math_utils.h"
#include "common/math/quaternion.h"
#include "common/util/util.h"
#include "common/vehicle_model/vehicle_model.h"
#include "planning/common/frame.h"
#include "planning/common/planning_gflags.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {

using ::google::protobuf::RepeatedPtrField;
using TL::common::TrajectoryPoint;
using TL::common::VehicleState;
using TL::common::math::Vec2d;

TrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
    const double vehicle_state_time, const VehicleState& vehicle_state) {
  TrajectoryPoint point;
  point.mutable_path_point()->set_s(0.0);
  point.mutable_path_point()->set_x(vehicle_state.x());
  point.mutable_path_point()->set_y(vehicle_state.y());
  point.mutable_path_point()->set_theta(vehicle_state.heading());
  point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  point.mutable_path_point()->set_dkappa(0.0);
  point.mutable_path_point()->set_ddkappa(0.0);
  point.set_v(vehicle_state.linear_velocity());
  point.set_a(fabs(vehicle_state.linear_velocity()) < 0.05
                  ? 0.0
                  : vehicle_state.linear_acceleration());
  point.set_relative_time(vehicle_state_time);
  return point;
}

RepeatedPtrField<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double vehicle_state_time, const VehicleState& vehicle_state,
    const common::VehicleModelConfig& vehicle_model_config) {
  TrajectoryPoint reinit_point;
  UNUSED(vehicle_model_config);

  static constexpr double kEpsilon_v = 0.1;
  static constexpr double kEpsilon_a = 0.4;
  // TODO(Jinyun/Yu): adjust kEpsilon if corrected IMU acceleration provided
  if (std::fabs(vehicle_state.linear_velocity()) < kEpsilon_v) {
    reinit_point = ComputeTrajectoryPointFromVehicleState(0.0, vehicle_state);
  } else {
    reinit_point = ComputeTrajectoryPointFromVehicleState(vehicle_state_time,
                                                          vehicle_state);
  }
  RepeatedPtrField<TrajectoryPoint> reinit_stitching_trajectory;
  reinit_stitching_trajectory.Add()->CopyFrom(reinit_point);
  return reinit_stitching_trajectory;
}

void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const common::VehicleState& vehicle_state,
    PublishableTrajectory* const prev_trajectory) {
  if (prev_trajectory == nullptr) {
    return;
  }
  static constexpr double kMagicCompensationCoefficient = 1.12;
  double time_diff = vehicle_state.timestamp() - prev_trajectory->header_time();
  double theta_diff = vehicle_state.angular_velocity() * time_diff *
                      kMagicCompensationCoefficient;
  double x_diff = vehicle_state.linear_velocity() * time_diff;
  double y_diff =
      x_diff * x_diff *
      (theta_diff / TL::common::math::Clamp(vehicle_state.linear_velocity(),
                                               0.001, 1000.0)) *
      0.5;

  ADEBUG << "yaw_rate "
         << " x_diff " << x_diff << " y_diff " << y_diff << " theta_diff "
         << theta_diff << " speed " << vehicle_state.linear_velocity();

  // R^-1
  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  for (int i = 0; i < prev_trajectory->size(); i++) {
    auto* p = prev_trajectory->Mutable(i);
    auto x = p->path_point().x();
    auto y = p->path_point().y();
    auto theta = p->path_point().theta();

    auto x_new = cos_theta * x - sin_theta * y + tx;
    auto y_new = sin_theta * x + cos_theta * y + ty;
    auto theta_new = common::math::NormalizeAngle(theta - theta_diff);

    p->mutable_path_point()->set_x(x_new);
    p->mutable_path_point()->set_y(y_new);
    p->mutable_path_point()->set_theta(theta_new);
  }
}

/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
RepeatedPtrField<TrajectoryPoint>
TrajectoryStitcher::ComputeStitchingTrajectory(  // NOLINT
    const VehicleState& vehicle_state,
    const std::shared_ptr<VehicleState>& prev_vehicle_state,
    const double current_timestamp, const double planning_cycle_time,
    const size_t preserved_points_num, const bool replan_by_offset,
    const bool is_forward, const functionmanager::FunctionManagerIn& fct_input,
    const common::VehicleModelConfig& vehicle_model_config,
    const functionmanager::AvpFctOut::FsmStageType& fsm_stage_type,
    PublishableTrajectory* const prev_trajectory,
    std::pair<int, std::string>* const replan_reason,
    const ForceRplanType& force_replan_type) {
  replan_reason->first = 0;
  double offset_time =
      std::fmax(std::fmin(vehicle_state.timestamp() - current_timestamp,
                          2.0 * planning_cycle_time),
                -2.0 * planning_cycle_time);

  if (offset_time > 0) {
    AWARN << "receive future vehicle localization state. current timestamp "
          << FIXED << SETPRECISION(3) << current_timestamp
          << " vehicle state time " << vehicle_state.timestamp();
  }
  if (!FLAGS_enable_trajectory_stitcher) {
    replan_reason->second = "stitch is disabled by gflag.";
    return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                            vehicle_model_config);
  }

  if (functionmanager::AvpFctOut::PARKING == fsm_stage_type) {
    return ComputeStitchingTrajectoryOnlyByPose(
        preserved_points_num, offset_time, current_timestamp, is_forward,
        vehicle_state, vehicle_model_config, prev_trajectory);
  }

  if (prev_trajectory == nullptr) {
    replan_reason->second = "replan for no previous trajectory.";
    return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                            vehicle_model_config);
  }

  if (vehicle_state.driving_mode() == soc::Chassis::COMPLETE_MANUAL) {
    replan_reason->second = "replan for manual mode.";
    return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                            vehicle_model_config);
  }
  // auto nnp_sys_state = fct_input.fct_nnp_in().nnp_sysstate();
  // if (nnp_sys_state == functionmanager::NNPSysState::NNPS_OVERRIDE ) {
  //   ADEBUG << "replan for driver both override.";
  //   replan_reason->second = "replan for driver both override.";
  //   return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
  //                                           vehicle_model_config);
  // }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    replan_reason->second = "replan for empty previous trajectory.";
    return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                            vehicle_model_config);
  }
  // hdmap-->perception(earth2bus)
  if (prev_vehicle_state != nullptr &&
      !prev_vehicle_state->is_vehicle_reference_frame() &&
      vehicle_state.is_vehicle_reference_frame()) {
    AINFO << "-------TrajectoryEarth2Bus------------";
    TrajectoryEarth2Bus(*prev_vehicle_state, prev_trajectory);
  }
  // perception-->hdmap(bus2earth)
  if (prev_vehicle_state != nullptr &&
      prev_vehicle_state->is_vehicle_reference_frame() &&
      !vehicle_state.is_vehicle_reference_frame()) {
    AINFO << "-------TrajectoryBus2Earth------------";
    TrajectoryBus2Earth(vehicle_state, prev_trajectory);
  }

  if (prev_vehicle_state != nullptr &&
      prev_vehicle_state->pose().using_utm_zone() !=
          vehicle_state.pose().using_utm_zone()) {
    TrajectoryEarth2Bus(*prev_vehicle_state, prev_trajectory);
    TrajectoryBus2Earth(vehicle_state, prev_trajectory);
  }

  if (vehicle_state.is_vehicle_reference_frame()) {
    TransformLastPublishedTrajectory(vehicle_state, prev_trajectory);
  }
  const double veh_rel_time =
      vehicle_state.timestamp() - prev_trajectory->header_time();

  size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  if (time_matched_index == 0 &&
      vehicle_state.driving_mode() != soc::Chassis::AUTO_SPEED_ONLY &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    replan_reason->second =
        "replan for current time smaller than the previous trajectory's "
        "first "
        "time.";
    return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                            vehicle_model_config);
  }
  if (time_matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time";
    replan_reason->second =
        "replan for current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                            vehicle_model_config);
  }

  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(time_matched_index));

  if (!time_matched_point.has_path_point()) {
    replan_reason->second = "replan for previous trajectory missed path point";
    return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                            vehicle_model_config);
  }

  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
      {vehicle_state.x(), vehicle_state.y()}, 1.0e-6);
  static constexpr double kMinRelativeTimeWhenMatchingPosition = -2.0;
  static constexpr double kMaxRelativeTimeWhenMatchingPosition = 2.0;
  size_t min_matched_position_index = prev_trajectory->QueryLowerBoundPoint(
      kMinRelativeTimeWhenMatchingPosition);
  position_matched_index = min_matched_position_index > position_matched_index
                               ? min_matched_position_index
                               : position_matched_index;
  auto position_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(position_matched_index));
  auto frenet_sd = ComputePositionProjection(
      is_forward, vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index)));

  if (replan_by_offset) {
    const auto& vehicle_time_point = prev_trajectory->TrajectoryPointAt(
        static_cast<uint32_t>(prev_trajectory->QueryLowerBoundPoint(
            vehicle_state.timestamp() - prev_trajectory->header_time())));
    if (FLAGS_replan_every_period) {
      auto vec_init_points = ComputeReinitStitchingTrajectory(
          offset_time, vehicle_state, vehicle_model_config);
      if (std::fabs(vehicle_state.linear_velocity() - vehicle_time_point.v()) >
              FLAGS_longitudinal_replan_speed_threshold ||
          std::fabs(vehicle_time_point.v() / vehicle_state.linear_velocity()) >
              FLAGS_longitudinal_replan_speed_threshold) {
        replan_reason->first = 3;
        return vec_init_points;
      }
      vec_init_points.at(0).set_v(vehicle_time_point.v());
      vec_init_points.at(0).set_a(vehicle_time_point.a());
      return vec_init_points;
    }
    auto lon_diff = vehicle_time_point.path_point().s() - frenet_sd.first;
    auto lat_diff = frenet_sd.second;
    auto heading_err = common::math::NormalizeAngle(
        position_matched_point.path_point().theta() - vehicle_state.heading());

    ADEBUG << "Control_lateral_diff:" << lat_diff
           << "  longitudinal_diff:" << lon_diff
           << "   veh_x:" << vehicle_state.x()
           << "   veh_y:" << vehicle_state.y() << "   time_diff:"
           << vehicle_state.timestamp() - prev_trajectory->header_time()
           << "   time_point_s:" << vehicle_time_point.path_point().s()
           << "   pos_point_s:" << frenet_sd.first;
    if ((std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) ||
        (std::fabs(heading_err) > FLAGS_replan_heading_threshold)) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lat_diff = ",
          lat_diff, "heading_err: ", heading_err);
      AERROR << msg;
      replan_reason->first = 1;
      replan_reason->second = msg;
      return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                              vehicle_model_config);
    }

    double replan_longitudinal_distance_threshold =
        FLAGS_replan_longitudinal_distance_threshold;
    if (fct_input.ta_pilot_mode() == functionmanager::AVP) {
      replan_longitudinal_distance_threshold =
          FLAGS_valet_parking_replan_longitudinal_distance_threshold;
    }

    if (std::fabs(lon_diff) > replan_longitudinal_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lon_diff = ",
          lon_diff);
      AERROR << msg;
      replan_reason->first = 2;
      replan_reason->second = msg;
      return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                              vehicle_model_config);
    }
  } else {
    ADEBUG << "replan according to certain amount of lat and lon offset is "
              "disabled";
  }
  // lon override
  if (vehicle_state.driving_mode() == soc::Chassis::AUTO_STEER_ONLY) {
    ADEBUG << "replan for driver lon override.";
    auto vehicle_position_point =
        prev_trajectory->Evaluate(prev_trajectory->QueryMatchedRelativeTime(
            {vehicle_state.x(), vehicle_state.y()}));
    replan_reason->second = "replan for driver lon override.";
    vehicle_position_point.mutable_path_point()->set_s(0.0);
    vehicle_position_point.set_v(vehicle_state.linear_velocity());
    vehicle_position_point.set_a(vehicle_state.linear_acceleration());
    vehicle_position_point.set_relative_time(0.0);
    if (fct_input.ta_pilot_mode() != functionmanager::AVP &&
        (fct_input.driver_mode() ==
             functionmanager::ADAS_LAT_ACTIVE_LGT_OVERRIDE ||
         fct_input.driver_mode() ==
             functionmanager::NNP_LAT_ACTIVE_LGT_OVERRIDE ||
         fct_input.driver_mode() ==
             functionmanager::NCP_LAT_ACTIVE_LGT_OVERRIDE ||
         fct_input.fct_nnp_in().acc_state() ==
             functionmanager::FctToNnpInput::ACC_OVERRIDE)) {
      vehicle_position_point.set_a(0.0);
    }
    RepeatedPtrField<TrajectoryPoint> lon_override_stitching_trajectory;
    lon_override_stitching_trajectory.Add()->CopyFrom(vehicle_position_point);
    return lon_override_stitching_trajectory;
  }

  // lat override
  if (vehicle_state.driving_mode() == soc::Chassis::AUTO_SPEED_ONLY) {
    ADEBUG << "replan for driver lat override.";
    replan_reason->second = "replan for driver lat override.";
    auto vehicle_position_point =
        prev_trajectory->Evaluate(prev_trajectory->QueryMatchedRelativeTime(
            {vehicle_state.x(), vehicle_state.y()}));
    const auto lat_offset_time = vehicle_position_point.relative_time() +
                                 prev_trajectory->header_time() -
                                 current_timestamp;
    if (fabs(lat_offset_time) > kMaxRelativeTimeWhenMatchingPosition) {
      replan_reason->first = 2;
      replan_reason->second = "replan for fabs (offset_time) > 2 ";
      return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                              vehicle_model_config);
    }
    return ComputeStitchingTrajectoryByTransfromLast(
        vehicle_position_point, vehicle_state, prev_trajectory,
        vehicle_model_config, offset_time, current_timestamp);
  }
  if (force_replan_type != NO_FORCE_REPLAN &&
      fct_input.ta_pilot_mode() != functionmanager::AVP &&
      fabs(vehicle_state.linear_velocity()) < 0.05) {
    if (force_replan_type == LAT_FORCE_REPLAN) {
      replan_reason->first = 1;
      replan_reason->second = "replan for LAT_FORCE_REPLAN ,clear error ";
      const auto vehicle_position_point =
          prev_trajectory->Evaluate(prev_trajectory->QueryMatchedRelativeTime(
              {vehicle_state.x(), vehicle_state.y()}));
      auto lat_replan_points = ComputeReinitStitchingTrajectory(
          offset_time, vehicle_state, vehicle_model_config);
      lat_replan_points.at(0).set_v(vehicle_state.linear_velocity());
      lat_replan_points.at(0).set_a(vehicle_position_point.a());
      return lat_replan_points;
    }
    if (force_replan_type == LON_FORCE_REPLAN) {
      replan_reason->first = 2;
      replan_reason->second = "replan for LON_FORCE_REPLAN ,clear error ";
      auto vehicle_position_point =
          prev_trajectory->Evaluate(prev_trajectory->QueryMatchedRelativeTime(
              {vehicle_state.x(), vehicle_state.y()}));
      vehicle_position_point.mutable_path_point()->set_s(0.0);
      vehicle_position_point.set_v(vehicle_state.linear_velocity());
      vehicle_position_point.set_a(fabs(vehicle_state.linear_velocity()) < 0.05
                                       ? 0.0
                                       : vehicle_state.linear_acceleration());
      vehicle_position_point.set_relative_time(0.0);
      RepeatedPtrField<TrajectoryPoint> lon_replan_trajectory;
      lon_replan_trajectory.Add()->CopyFrom(vehicle_position_point);
      return lon_replan_trajectory;
    }
    if (force_replan_type == LAT_LON_FORCE_REPLAN) {
      replan_reason->first = 3;
      replan_reason->second = "replan for LAT_LON_FORCE_REPLAN ,clear error ";
      return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                              vehicle_model_config);
    }
  }

  double forward_rel_time = veh_rel_time + FLAGS_trajectory_stitching_time;

  // size_t forward_time_index =
  //     prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  ADEBUG << "Position matched index:\t" << position_matched_index;
  ADEBUG << "Time matched index:\t" << time_matched_index;

  auto matched_index = std::min(time_matched_index, position_matched_index);

  // RepeatedPtrField<TrajectoryPoint> stitching_trajectory(
  //     prev_trajectory->begin() +
  //         std::max(0, static_cast<int>(matched_index - preserved_points_num)),
  //     prev_trajectory->begin() + forward_time_index + 2);
  // ADEBUG << "stitching_trajectory size: " << stitching_trajectory.size();

  double stitching_start_time =
      (prev_trajectory->begin() +
       std::min(
           prev_trajectory->size() - 1,
           std::max(0, static_cast<int>(matched_index - preserved_points_num))))
          ->relative_time();
  double stitching_end_time = forward_rel_time;

  RepeatedPtrField<TrajectoryPoint> stitching_trajectory;
  stitching_trajectory.Reserve(static_cast<int>(preserved_points_num));
  static constexpr double kEpsilon = 0.01;
  for (size_t i = preserved_points_num; i > 0; i--) {
    double evaluate_time =
        stitching_end_time -
        FLAGS_trajectory_time_max_interval * (static_cast<double>(i) - 1);
    if (evaluate_time < stitching_start_time - kEpsilon) {
      continue;
    }
    stitching_trajectory.Add()->CopyFrom(
        prev_trajectory->Evaluate(evaluate_time));
  }

  if (stitching_trajectory.empty()) {
    stitching_trajectory.Add()->CopyFrom(prev_trajectory->back());
  }

  // RepeatedPtrField<TrajectoryPoint> stitching_trajectory;

  const double zero_s = stitching_trajectory.rbegin()->path_point().s();
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      replan_reason->second =
          "replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                              vehicle_model_config);
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }
  return stitching_trajectory;
}

std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const bool is_forward, const double x, const double y,
    const TrajectoryPoint& matched_trajectory_point) {
  Vec2d v(x - matched_trajectory_point.path_point().x(),
          y - matched_trajectory_point.path_point().y());
  double theta = common::math::NormalizeAngle(
      is_forward ? matched_trajectory_point.path_point().theta()
                 : matched_trajectory_point.path_point().theta() + M_PI);
  Vec2d n(std::cos(theta), std::sin(theta));

  std::pair<double, double> frenet_sd;
  frenet_sd.first = n.InnerProd(v) + matched_trajectory_point.path_point().s();
  frenet_sd.second = n.CrossProd(v);
  return frenet_sd;
}

bool TrajectoryStitcher::CalculateStitchingIndex(
    const double x, const double y, const PublishableTrajectory& pre_trajectory,
    size_t* const index) {
  size_t pre_trajectory_size = pre_trajectory.NumOfPoints();
  if (pre_trajectory_size == 0) {
    return false;
  }

  if (pre_trajectory.front().path_point().s() < 1.0e-6 &&
      pre_trajectory.back().path_point().s() < 1.0e-6) {
    return false;
  }

  size_t position_matched_index = pre_trajectory.QueryNearestPoint(Vec2d(x, y));
  if (position_matched_index < pre_trajectory_size) {
    position_matched_index =
        std::min(position_matched_index + 1, pre_trajectory_size - 1);
    *index = position_matched_index;
    return true;
  }

  return false;
}

RepeatedPtrField<TrajectoryPoint>
TrajectoryStitcher::ComputeStitchingTrajectoryOnlyByPose(
    const size_t preserved_points_num, double offset_time,
    const double current_timestamp, const bool is_forward,
    const VehicleState& vehicle_state,
    const common::VehicleModelConfig& vehicle_model_config,
    PublishableTrajectory* const prev_trajectory) {
  static constexpr double kEpisonS = 1.0e-6;
  offset_time = fmax(offset_time, 0.0);

  if (prev_trajectory == nullptr ||
      vehicle_state.driving_mode() != soc::Chassis::COMPLETE_AUTO_DRIVE) {
    return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                            vehicle_model_config);
  }

  if (prev_trajectory->empty() ||
      (prev_trajectory->back().path_point().s() < kEpisonS &&
       prev_trajectory->front().path_point().s() < kEpisonS)) {
    return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                            vehicle_model_config);
  }

  if (fabs(vehicle_state.linear_velocity()) <
      FLAGS_open_space_replan_velocity_threshold) {
    return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                            vehicle_model_config);
  }

  size_t position_matched_index = prev_trajectory->QueryNearestPoint(
      Vec2d(vehicle_state.x(), vehicle_state.y()));
  auto vehicle_time_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(prev_trajectory->QueryLowerBoundPoint(
          vehicle_state.timestamp() - prev_trajectory->header_time())));
  auto frenet_sd = ComputePositionProjection(
      is_forward, vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index)));
  auto lon_diff = vehicle_time_point.path_point().s() - frenet_sd.first;
  if (fabs(lon_diff) >
      FLAGS_open_space_replan_longitudinal_distance_threshold) {
    return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                            vehicle_model_config);
  }

  position_matched_index =
      std::min(position_matched_index + 1, prev_trajectory->NumOfPoints() - 1);

  RepeatedPtrField<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->begin() +
          std::max(0, static_cast<int>(position_matched_index -
                                       preserved_points_num)),
      prev_trajectory->begin() + static_cast<int>(position_matched_index) + 1);

  const double zero_s = stitching_trajectory.rbegin()->path_point().s();
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                              vehicle_model_config);
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }

  double last_point_relative_t =
      stitching_trajectory.rbegin()->relative_time() +
      static_cast<double>(FLAGS_publish_trajectory_points_number -
                          stitching_trajectory.size()) *
          FLAGS_trajectory_time_resolution;
  if (last_point_relative_t < FLAGS_trajectory_time_resolution) {
    return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                            vehicle_model_config);
  }

  return stitching_trajectory;
}

void TrajectoryStitcher::TrajectoryBus2Earth(
    const common::VehicleState& vehicle_state,
    PublishableTrajectory* prev_trajectory) {
  prev_trajectory->set_header_time(vehicle_state.timestamp());
  for (int i = 0; i < prev_trajectory->size(); i++) {
    auto* p = prev_trajectory->Mutable(i);
    Eigen::Vector2d enu_coordinate = common::math::RotateVector2d(
        {p->path_point().x(), p->path_point().y()}, vehicle_state.heading());

    auto x_new = enu_coordinate.x() + vehicle_state.x();
    auto y_new = enu_coordinate.y() + vehicle_state.y();
    auto theta_new = common::math::NormalizeAngle(
        common::math::NormalizeAngle(p->path_point().theta()) +
        vehicle_state.heading());

    p->mutable_path_point()->set_x(x_new);
    p->mutable_path_point()->set_y(y_new);
    p->mutable_path_point()->set_theta(theta_new);
  }
}

void TrajectoryStitcher::TrajectoryEarth2Bus(
    const common::VehicleState& vehicle_state,
    PublishableTrajectory* prev_trajectory) {
  prev_trajectory->set_header_time(vehicle_state.timestamp());
  for (int i = 0; i < prev_trajectory->size(); i++) {
    auto* p = prev_trajectory->Mutable(i);
    auto x = p->path_point().x() - vehicle_state.x();
    auto y = p->path_point().y() - vehicle_state.y();
    auto theta = vehicle_state.heading();
    Eigen::Vector2d enu_coordinate =
        common::math::RotateVector2d({y, x}, theta);
    // auto x_new = x * std::cos(theta) + y * std::sin(theta);
    // auto y_new = -x * std::sin(theta) + y * std::cos(theta);
    auto theta_new =
        common::math::NormalizeAngle(p->path_point().theta() - theta);
    p->mutable_path_point()->set_x(enu_coordinate.y());
    p->mutable_path_point()->set_y(enu_coordinate.x());
    p->mutable_path_point()->set_theta(theta_new);
  }
}

::google::protobuf::RepeatedPtrField<common::TrajectoryPoint>
TrajectoryStitcher::ComputeStitchingTrajectoryByTransfromLast(
    const TrajectoryPoint& base_point,
    const common::VehicleState& vehicle_state,
    PublishableTrajectory* const prev_trajectory,
    const common::VehicleModelConfig& vehicle_model_config,
    const double offset_time, const double current_timestamp) {
  if (prev_trajectory == nullptr || prev_trajectory->empty()) {
    return ComputeReinitStitchingTrajectory(offset_time, vehicle_state,
                                            vehicle_model_config);
  }
  double theta_diff = vehicle_state.heading() - base_point.path_point().theta();
  const auto x_offset = vehicle_state.x() - base_point.path_point().x();
  const auto y_offset = vehicle_state.y() - base_point.path_point().y();
  RepeatedPtrField<TrajectoryPoint> stitching_trajectory;
  DiscretizedTrajectory transformed_prev_trajectory;
  const auto time_diff = prev_trajectory->header_time() - current_timestamp;
  const auto max_preserved_time =
      FLAGS_trajectory_time_resolution *
      static_cast<double>(FLAGS_trajectory_stitching_preserved_length - 1);
  for (int i = 0; i < prev_trajectory->size(); i++) {
    const auto t = prev_trajectory->at(i).relative_time();
    // 往前最多20个点
    if (common::math::double_type::DefinitelyLess(
            t, base_point.relative_time() - max_preserved_time)) {
      continue;
    }
    if (common::math::double_type::DefinitelyGreaterEqual(
            t, base_point.relative_time())) {
      break;
    }
    auto* p = prev_trajectory->Mutable(i);
    auto x = p->path_point().x();
    auto y = p->path_point().y();
    auto theta = p->path_point().theta();
    auto delta_x = x - base_point.path_point().x();
    auto delta_y = y - base_point.path_point().y();
    Eigen::Vector2d rotated_point = common::math::RotateVector2d(
        {delta_x, delta_y}, common::math::NormalizeAngle(theta_diff));
    auto x_new = base_point.path_point().x() + rotated_point.x() + x_offset;
    auto y_new = base_point.path_point().y() + rotated_point.y() + y_offset;
    auto theta_new = common::math::NormalizeAngle(theta + theta_diff);
    p->mutable_path_point()->set_x(x_new);
    p->mutable_path_point()->set_y(y_new);
    p->mutable_path_point()->set_theta(theta_new);
    p->set_relative_time(t + time_diff);
    transformed_prev_trajectory.add()->CopyFrom(*p);
  }
  const auto lat_offset_time = base_point.relative_time() +
                               prev_trajectory->header_time() -
                               current_timestamp;
  auto vec_point =
      ComputeTrajectoryPointFromVehicleState(lat_offset_time, vehicle_state);
  vec_point.set_v(base_point.v());
  vec_point.set_a(base_point.a());
  vec_point.set_da(base_point.da());
  transformed_prev_trajectory.add()->CopyFrom(vec_point);
  const auto start_time = transformed_prev_trajectory.at(0).relative_time();
  const auto trajectory_size = transformed_prev_trajectory.size();
  for (int i = 0; i < trajectory_size; i++) {
    if (i == trajectory_size - 1) {
      stitching_trajectory.Add()->CopyFrom(vec_point);
      break;
    }
    const auto time = vec_point.relative_time() -
                      static_cast<double>(FLAGS_trajectory_time_resolution *
                                          (trajectory_size - 1 - i));
    if (common::math::double_type::DefinitelyLess(time, start_time)) {
      continue;
    }
    const auto& point = transformed_prev_trajectory.Evaluate(time);
    stitching_trajectory.Add()->CopyFrom(point);
  }
  return stitching_trajectory;
}
}  // namespace planning
}  // namespace TL
