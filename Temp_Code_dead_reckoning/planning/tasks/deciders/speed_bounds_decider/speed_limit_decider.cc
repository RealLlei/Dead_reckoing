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

#include "planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"
#include <cmath>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <set>

#include "common/configs/vehicle_config_helper.h"
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"
#include "common/status/status.h"
#include "planning/common/open_space_info.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/speed_limit.h"
#include "planning/common/util/common.h"
#include "planning/common/util/util.h"
#include "planning/hmi/lon_hmi/speed_convertor/speed_convertor.h"
#include "planning/localview/local_view.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/common/types.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/map/map_road.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/planning/decision.pb.h"

namespace TL {
namespace planning {

using TL::common::Status;

Status SpeedLimitDecider::Init(const SpeedBoundsDeciderConfig& config,
                               const ReferenceLine& reference_line,
                               const PathData& path_data) {
  speed_bounds_config_ = config;
  reference_line_ = reference_line;
  path_data_ = path_data;
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  if (!LoadCentricAccelCalibrationTableForward()) {
    centric_accel_calibration_table_avp_forward_.first = {
        0.0, std::numeric_limits<double>::max()};
    centric_accel_calibration_table_avp_forward_.second = {
        speed_bounds_config_.max_centric_acceleration_limit(),
        speed_bounds_config_.max_centric_acceleration_limit()};
  }
  if (!LoadCentricAccelCalibrationTableReverse()) {
    centric_accel_calibration_table_avp_reverse_.first = {
        0.0, std::numeric_limits<double>::max()};
    centric_accel_calibration_table_avp_reverse_.second = {
        speed_bounds_config_.max_centric_acceleration_limit(),
        speed_bounds_config_.max_centric_acceleration_limit()};
  }
  if (cone_v_conf_.empty() && cone_l_conf_.empty()) {
    if (config.has_cone_nudge_speed_limit_coef_calibration_2d_table() &&
        config.cone_nudge_speed_limit_coef_calibration_2d_table()
                .calibration_info()
                .size() > 1) {
      for (const auto& calibration_info :
           config.cone_nudge_speed_limit_coef_calibration_2d_table()
               .calibration_info()) {
        cone_v_conf_.emplace_back(calibration_info.speed());
        cone_l_conf_.emplace_back(calibration_info.l_distance());
      }
    } else {
      cone_v_conf_ = {20, 40};
      cone_l_conf_ = {0.4, 0.8};
    }
  }
  return Status::OK();
}

Status SpeedLimitDecider::GetSpeedLimits(
    const IndexedList<std::string, Obstacle>& obstacles,
    const ReferenceLineInfo& reference_line_info,
    const TrajectoryPoint& init_point, const LocalView& local_view,
    const std::pair<std::vector<double>, std::vector<double>>&
        centric_accel_calibration_table,
    Frame* const frame,
    SpeedLimit* const speed_limit_data) {  // NOLINT
  CHECK_NOTNULL(speed_limit_data);
  if (!local_view.HasChassis() &&
      !local_view.GetChassis()->has_steering_angle()) {
    const std::string msg = "no chassis input!";
    AERROR << msg;
    return Status(common::ErrorCode::PLANNER_CRUISING_SPEEDBOUNDSPRIORI_ERROR,
                  msg);
  }
  SpeedLimit curvature_speed_limit_data;
  std::vector<const Obstacle*> pedestrians;
  std::set<int> pedestrian_ids;
  GetConeSpeedLimitSegments(reference_line_info);
  for (const auto* ptr_obstacle : obstacles.Items()) {
    if (ptr_obstacle == nullptr || ptr_obstacle->IsVirtual() ||
        ptr_obstacle->Perception().type() !=
            perception::PerceptionObstacle::PEDESTRIAN ||
        (reference_line_info.path_data()
             .frenet_frame_path()
             .is_forward_path() &&
         ptr_obstacle->PerceptionSLBoundary().end_s() <
             reference_line_info.AdcSlBoundary().start_s()) ||
        !ptr_obstacle->HasTrajectory() ||
        ptr_obstacle->Trajectory().trajectory_point().empty() ||
        !ptr_obstacle->Trajectory()
             .trajectory_point(
                 ptr_obstacle->Trajectory().trajectory_point_size() - 1)
             .has_gaussian_info() ||
        pedestrian_ids.count(ptr_obstacle->PerceptionId()) > 0) {
      continue;
    }
    pedestrians.emplace_back(ptr_obstacle);
    pedestrian_ids.insert(ptr_obstacle->PerceptionId());
  }

  const auto is_acc_mode =
      frame->vehicle_state().driving_mode() == soc::Chassis::AUTO_SPEED_ONLY;
  const auto& current_state =
      local_view.GetFunctionManagerOut()->perception_sub_state();
  SpeedLimitInfo speed_limit_info;
  double min_pedestrian_speed_limit = FLAGS_planning_upper_speed_limit;
  const auto& discretized_path = path_data_.discretized_path();
  const auto& frenet_path = path_data_.frenet_frame_path();
  TL::planning::util::GetStateAtMinJerk(
      init_point.v(), init_point.a(), 0, reference_line_info.GetMaxSpeed(),
      reference_line_info.GetMaxDeceleration(),
      FLAGS_longitudinal_jerk_lower_bound *
          FLAGS_longitudinal_jerk_lower_bound_slack,
      FLAGS_trajectory_time_length, FLAGS_delta_time_speed_limit, 0,
      &state_all_);
  auto has_cdcs_road_speed = false;
  auto cdcs_road_speed = speed_bounds_config_.user_max_speed();
  if (local_view.HasFunctionManagerIn() &&
      local_view.GetFunctionManagerIn()->fct_nnp_in().has_cdcs_info() &&
      local_view.GetFunctionManagerIn()
          ->fct_nnp_in()
          .cdcs_info()
          .has_cdcs_speed_limit()) {
    const auto& cdcs_speed_limit = local_view.GetFunctionManagerIn()
                                       ->fct_nnp_in()
                                       .cdcs_info()
                                       .cdcs_speed_limit();
    has_cdcs_road_speed = cdcs_speed_limit.has_road_speed_limit() &&
                          cdcs_speed_limit.road_speed_limit() >= 80;
    cdcs_road_speed = SpeedConventor::ConvertDisplaySpdToReal(
        static_cast<int>(cdcs_speed_limit.road_speed_limit()));
  }
  const auto usr_changed_speed =
      local_view.HasFunctionManagerIn() && local_view.GetFunctionManagerIn()
                                               ->fct_nnp_in()
                                               .usr_has_changed_cruise_spd();
  auto max_speed = FLAGS_planning_upper_speed_limit;
  if (usr_changed_speed) {
    max_speed = SpeedConventor::ConvertDisplaySpdToReal(static_cast<int>(
        std::round(speed_bounds_config_.user_max_speed() * 3.6)));
  }

  // ignore first part and last part kappa, because they are invalid
  // TODO(chenwei): in the furture if path kappa is all valid, remove this code
  auto valid_kappa_start_s = std::numeric_limits<double>::lowest();
  auto valid_kappa_end_s = std::numeric_limits<double>::max();
  auto valid_kappa_start_index = std::numeric_limits<int>::infinity();
  const auto& candidate_path_boundaries =
      reference_line_info.GetCandidatePathBoundaries();
  const auto is_avp_mode = local_view.HasFunctionManagerIn() &&
                           local_view.GetFunctionManagerIn()->ta_pilot_mode() ==
                               TL::functionmanager::TaPilotMode::AVP;
  const auto real_steer_angle = local_view.GetChassis()->steering_angle();
  const auto ntp_preview_path_index =
      !discretized_path.empty()
          ? std::min(speed_bounds_config_.ntp_preview_path_index(),
                     static_cast<uint>(discretized_path.size() - 1))
          : 0;
  const double kappa = !discretized_path.empty()
                           ? discretized_path.at(ntp_preview_path_index).kappa()
                           : 0.0;
  const double steer_angle_diff_speed_limit = GetAvpSteerDiffSpeedLimit(
      kappa, real_steer_angle, reference_line_info.GetCruiseSpeed());
  if (!candidate_path_boundaries.empty()) {
    const auto& candidate_path_boundary = candidate_path_boundaries.front();
    valid_kappa_start_s =
        candidate_path_boundary.start_s() + init_point.v() * 0.5;
    valid_kappa_end_s =
        candidate_path_boundary.start_s() +
        candidate_path_boundary.delta_s() *
            (static_cast<int>(candidate_path_boundary.boundary().size()) - 1) -
        init_point.v() * 0.5;
  }
  std::vector<PedestrianPoint> pedestrians_point;
  pedestrians_point.reserve(pedestrians.size());
  if (!discretized_path_last_.empty()) {
    discretized_path_last_valid_ = true;
  }
  if (current_state != last_state_ && discretized_path_last_valid_) {
    mode_change_valid_ = true;
  }
  if (reference_line_info.IsChangeLanePath()) {
    mode_change_valid_ = false;
  }
  if (mode_change_valid_) {
    mode_change_count_++;
  } else {
    mode_change_count_ = 0;
  }
  if (mode_change_count_ > 25) {
    mode_change_count_ = 25;
    mode_change_valid_ = false;
  }
  for (const auto* ptr_obstacle : pedestrians) {
    if (ptr_obstacle == nullptr ||
        ptr_obstacle->Trajectory().trajectory_point().empty() ||
        !ptr_obstacle->Trajectory()
             .trajectory_point(
                 ptr_obstacle->Trajectory().trajectory_point_size() - 1)
             .has_gaussian_info()) {
      continue;
    }
    PedestrianPoint pedestrian_point;

    const auto& trajectory_point = ptr_obstacle->Trajectory().trajectory_point(
        ptr_obstacle->Trajectory().trajectory_point_size() - 1);
    pedestrian_point.mean_x = trajectory_point.path_point().x();
    pedestrian_point.mean_y = trajectory_point.path_point().y();
    pedestrian_point.sigma_x = 1 / trajectory_point.gaussian_info().sigma_x();
    pedestrian_point.sigma_y = 1 / trajectory_point.gaussian_info().sigma_y();
    pedestrian_point.product_sigma_xy =
        pedestrian_point.sigma_x * pedestrian_point.sigma_y;
    pedestrian_point.double_correlation =
        2 * trajectory_point.gaussian_info().correlation();
    pedestrian_point.temp2 =
        1 / (2 * (1 - pow(trajectory_point.gaussian_info().correlation(), 2)));
    pedestrians_point.emplace_back(pedestrian_point);
  }
  const double delta_s = EgoMovingStep(local_view);
  const auto current_function_status = frame->vehicle_state().driving_mode();
  ADEBUG << "current_function_status  "
         << static_cast<int>(current_function_status)
         << " last_function_status_ " << static_cast<int>(last_function_status_)
         << " status "
         << static_cast<int>(
                local_view.GetFunctionManagerIn()->ta_pilot_mode());
  if (current_function_status == soc::Chassis::COMPLETE_MANUAL) {
    active_time_ = -0.1;
  } else if (current_function_status != last_function_status_ &&
             (current_function_status == soc::Chassis::AUTO_SPEED_ONLY ||
              current_function_status == soc::Chassis::COMPLETE_AUTO_DRIVE)) {
    active_time_ = 0.0;
  } else {
    active_time_ = std::max(0.0, std::min(active_time_ + 0.1, 10.0));
  }
  if (current_function_status != last_function_status_ &&
      (current_function_status == soc::Chassis::AUTO_SPEED_ONLY ||
       current_function_status == soc::Chassis::COMPLETE_AUTO_DRIVE)) {
    current_centric_accel_ = std::abs(discretized_path.front().kappa()) *
                             std::pow(frame->PlanningStartPoint().v(), 2);
  }
  std::unordered_map<uint32_t, perception::PerceptionObstacle_Type> box_id_type;
  for (const auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (obstacle != nullptr && !obstacle->IsStatic()) {
      if (obstacle->Perception().type() ==
          TL::perception::PerceptionObstacle::VEHICLE) {
        const auto& reference_point = reference_line_.GetReferencePoint(
            obstacle->PerceptionSLBoundary().start_s());
        const double angle_diff = fabs(common::math::NormalizeAngle(
            obstacle->Perception().theta() - reference_point.heading()));
        if (angle_diff >
            speed_bounds_config_.ntp_reverse_vehicle_heading_diff_threshold() *
                M_PI) {
          continue;
        }
      }

      box_id_type.insert(std::make_pair(obstacle->Perception().id(),
                                        obstacle->Perception().type()));
    }
  }

  for (uint32_t i = 0; i < discretized_path.size(); ++i) {
    const double path_s = discretized_path.at(i).s();
    const double reference_line_s = frenet_path.at(i).s();
    if (frenet_path.is_forward_path() &&
        reference_line_s > reference_line_.Length()) {
      AWARN << "path w.r.t. reference line at [" << reference_line_s
            << "] is LARGER than reference_line_ length ["
            << reference_line_.Length() << "]. Please debug before proceeding.";
      break;
    }
    if (!frenet_path.is_forward_path() && reference_line_s < 0.0) {
      AWARN << "path w.r.t. reference line at [" << reference_line_s
            << "] is LESS than reference_line_ start s [0.0 ]. Please debug "
               "before proceeding.";
      break;
    }

#ifdef ISORIN
    // (1) speed limit from map
    // 如果有摄像头限速，只要用户调了，就是可以超速的
    speed_limit_info.map_speed_limit = reference_line_.GetRoadSpeedLimitFromS(
        reference_line_s, reference_line_info.GetMaxSpeed());
#else
    if (!is_avp_mode) {
      speed_limit_info.map_speed_limit = reference_line_.GetRoadSpeedLimitFromS(
          reference_line_s, reference_line_info.GetMaxSpeed());
    } else {
      speed_limit_info.map_speed_limit = reference_line_info.GetCruiseSpeed();
    }
#endif
    speed_limit_info.map_speed_limit = fmax(speed_bounds_config_.lowest_speed(),
                                            speed_limit_info.map_speed_limit);
    speed_limit_info.origin_map_speed_limit = speed_limit_info.map_speed_limit;
    // speed limit from usr over speed
    if (frenet_path.is_forward_path()) {
      speed_limit_info.allow_over_speed = usr_changed_speed;
      const double map_speed_ms =
          SpeedConventor::ConvertDisplaySpdToReal(static_cast<int>(
              std::round(speed_limit_info.origin_map_speed_limit * 3.6)));
      speed_limit_info.map_speed_limit =
          usr_changed_speed     ? max_speed
          : has_cdcs_road_speed ? fmax(cdcs_road_speed, map_speed_ms)
                                : map_speed_ms;
    }

    // (2) speed limit from decision
    speed_limit_info.decision_speed_limit =
        reference_line_.GetDecisionSpeedLimitFromS(
            reference_line_s, reference_line_info.GetMaxSpeed());
    // (2.5) speed limit from speed bump
    if (is_avp_mode && frenet_path.is_forward_path()) {
      TL::common::PointENU path_point_enu;
      path_point_enu.set_x(discretized_path.at(i).x());
      path_point_enu.set_y(discretized_path.at(i).y());
      static constexpr double search_distance = 5.0;
      const std::vector<double> distance_to_speed_bump = {0.0, 1.5};
      const std::vector<double> speed_limit_speed_bump = {
          speed_bounds_config_.ntp_speed_bump_speed_limit(),
          reference_line_info.GetCruiseSpeed()};
      std::vector<hdmap::SpeedBumpInfoConstPtr> speed_bumps;
      const auto center_path_point = path_data_.GetPathPointWithPathS(
          path_s + vehicle_param_.length() / 2 -
          vehicle_param_.back_edge_to_center());
      common::math::Vec2d center_path_point_vec(center_path_point.x(),
                                                center_path_point.y());
      if (hdmap::HDMapUtil::MapForPlanning().GetSpeedBumps(
              path_point_enu, search_distance, &speed_bumps) == 0) {
        for (const auto& bumps : speed_bumps) {
          if (bumps == nullptr) {
            continue;
          }
          for (const auto& Curve : bumps->speed_bump().position()) {
            for (const auto& seg : Curve.segment()) {
              if (seg.line_segment().point().empty()) {
                continue;
              }
              common::math::LineSegment2d speed_bump_seg(
                  {seg.line_segment().point().begin()->x(),
                   seg.line_segment().point().begin()->y()},
                  {seg.line_segment().point().rbegin()->x(),
                   seg.line_segment().point().rbegin()->y()});
              const auto min_distance =
                  speed_bump_seg.DistanceTo(center_path_point_vec);
              const auto speed_bump_speed_limit =
                  common::math::InterpolationOne(min_distance,
                                                 distance_to_speed_bump,
                                                 speed_limit_speed_bump);
              speed_limit_info.decision_speed_limit =
                  fmin(speed_limit_info.decision_speed_limit,
                       speed_bump_speed_limit);
            }
          }
        }
      }
    }

    // (3) speed limit from path curvature, limit by centripetal force
    // (acceleration)
    if (reference_line_s > valid_kappa_start_s &&
        reference_line_s < valid_kappa_end_s) {
      auto kappa = fabs(
          is_acc_mode
              ? reference_line_.GetReferencePoint(frenet_path.at(i).s()).kappa()
              : discretized_path.at(i).kappa());
      auto centric_accel = common::math::InterpolationOne(
          kappa, centric_accel_calibration_table.first,
          centric_accel_calibration_table.second);
      if (is_avp_mode) {
        if (frenet_path.is_forward_path()) {
          const double distance_unit =
              speed_bounds_config_.slow_down_preview_distance_unit();
          for (int i = 1; i * distance_unit <=
                          speed_bounds_config_.slow_down_preview_distance_s();
               i++) {
            auto preview_ref_kappa = fabs(
                path_data_.GetPathPointWithPathS(path_s + i * distance_unit)
                    .kappa());
            if (preview_ref_kappa > kappa &&
                preview_ref_kappa >
                    speed_bounds_config_.slow_down_preview_kappa()) {
              kappa = preview_ref_kappa;
            }
          }
          centric_accel = common::math::InterpolationOne(
              kappa, centric_accel_calibration_table_avp_forward_.first,
              centric_accel_calibration_table_avp_forward_.second);
        } else {
          centric_accel = common::math::InterpolationOne(
              kappa, centric_accel_calibration_table_avp_reverse_.first,
              centric_accel_calibration_table_avp_reverse_.second);
        }
      } else {
        const double time_limit = 3.0;
        if ((active_time_ < (time_limit + 0.1) && active_time_ > -0.1) ||
            current_centric_accel_ > (centric_accel - 0.01)) {
          centric_accel = std::max(
              current_centric_accel_ -
                  active_time_ *
                      ((current_centric_accel_ - centric_accel) / time_limit),
              centric_accel);
        }
      }
      speed_limit_info.curvature_speed_limit =
          std::sqrt(centric_accel /
                    std::fmax(kappa, speed_bounds_config_.minimal_kappa()));
      if (std::isinf(valid_kappa_start_index)) {
        valid_kappa_start_index = static_cast<int>(i);
      }
    } else {
      speed_limit_info.curvature_speed_limit = FLAGS_planning_upper_speed_limit;
    }

    SpeedLimitByModeChange(path_s, delta_s, is_acc_mode, current_state,
                           &speed_limit_info);
    speed_limit_info.curvature_speed_limit =
        fmax(speed_bounds_config_.lowest_speed(),
             speed_limit_info.curvature_speed_limit);
    curvature_speed_limit_data.AppendSpeedLimit(
        path_s, speed_limit_info.curvature_speed_limit);
    curvature_speed_limit_data.AppendSpeedLimitInfo(path_s, speed_limit_info);
    // (4) speed limit from nudge obstacles
    // TODO(all): in future, expand the speed limit not only to obstacles with
    // nudge decisions.
    if (FLAGS_enable_nudge_slowdown) {
      speed_limit_info.decision_speed_limit =
          fmin(speed_limit_info.decision_speed_limit,
               GetNudgeSpeedLimit(frenet_path.at(i), obstacles,
                                  speed_limit_info.map_speed_limit));
    }

    // (5) speed limit from free space
    if (!path_data_.GetFreeSpaceSegments().empty() && is_avp_mode) {
      speed_limit_info.decision_speed_limit =
          fmin(speed_limit_info.decision_speed_limit,
               GetFreeSpaceSpeedLimit(discretized_path.at(i), box_id_type,
                                      path_data_.GetFreeSpaceSegments(),
                                      frenet_path.is_forward_path(),
                                      reference_line_info.GetCruiseSpeed()));
    }

// (5.5)speed limit from steerwheel angle diff in AVP mode
#ifndef FOR_BAIDU_SIMULATION
    if (is_avp_mode && !FLAGS_use_lapa_read_map_simulation) {
      speed_limit_info.decision_speed_limit = fmin(
          speed_limit_info.decision_speed_limit, steer_angle_diff_speed_limit);
    }
#endif

    // (6) speed limit from pedestrian
    const auto s_interval =
        (i + 1 < discretized_path.size())
            ? (discretized_path.at(i + 1).s() - discretized_path.at(i).s())
            : 0.0;
    min_pedestrian_speed_limit =
        fmin(min_pedestrian_speed_limit,
             GetPedestrianSpeedLimit(discretized_path.at(i), s_interval,
                                     pedestrians_point, is_avp_mode));
    speed_limit_info.pedestrian_speed_limit = min_pedestrian_speed_limit;
    // speed_limit_info.decision_speed_limit =
    //     fmax(speed_bounds_config_.lowest_speed(),
    //          speed_limit_info.decision_speed_limit);

    // (7) speed limit from max decel
    double vehicle_speed_limit_max_dec =
        TL::planning::util::GetSpeedLimitWithDeceMaxAtS(path_s, state_all_);

    // (8) speed limit from dkappa
    if (FLAGS_use_dkappa_speed_limit && i > 0 &&
        reference_line_s > valid_kappa_start_s &&
        reference_line_s < valid_kappa_end_s) {
      speed_limit_info.curvature_speed_limit =
          fmin(speed_limit_info.curvature_speed_limit,
               GetDKappaSpeedLimit(discretized_path.at(i - 1),
                                   discretized_path.at(i)));
    }

    // (9) speed limit from cone
    speed_limit_info.cone_speed_limit =
        GetConeSpeedLimit(discretized_path.at(i));

    // (9) final speed limit
    speed_limit_info.speed_limit =
        std::min({speed_limit_info.map_speed_limit,
                  speed_limit_info.decision_speed_limit,
                  speed_limit_info.curvature_speed_limit});
    speed_limit_info.speed_limit =
        std::max({speed_bounds_config_.lowest_speed(),
                  vehicle_speed_limit_max_dec, speed_limit_info.speed_limit});

    ADEBUG << "path_s:" << path_s
           << "  speed_map:" << speed_limit_info.map_speed_limit
           << "  speed_centri:" << speed_limit_info.curvature_speed_limit
           << "  speed_decision:" << speed_limit_info.decision_speed_limit
           << "  vehicle_speed_limit_max_dec:" << vehicle_speed_limit_max_dec
           << "  final_speed_limit:" << speed_limit_info.speed_limit
           << "  kappa:" << discretized_path.at(i).kappa();
    speed_limit_info.speed_limit =
        std::fmax(speed_limit_info.speed_limit, vehicle_speed_limit_max_dec);
    speed_limit_data->AppendSpeedLimit(path_s, speed_limit_info.speed_limit);
    speed_limit_data->AppendSpeedLimitInfo(path_s, speed_limit_info);
  }

  if (!reference_line_info.IsChangeLanePath()) {
    speed_limit_data->SmoothCurvatureSpeedLimit(
        valid_kappa_start_index,
        static_cast<int>(speed_bounds_config_.curvature_preview_distance() /
                         fmax(frenet_path.GetSpaceResolution(), 1e-5)));
  }

  ADEBUG << "   max_centric_acceleration_limit:"
         << speed_bounds_config_.max_centric_acceleration_limit();
  curvature_speed_limit_data_last_ = curvature_speed_limit_data;
  discretized_path_last_ = discretized_path;
  is_acc_mode_last_ = is_acc_mode;
  last_state_ = current_state;
  last_function_status_ = current_function_status;
  return Status::OK();
}

void SpeedLimitDecider::SpeedLimitByModeChange(
    const double path_s, const double delta_s, const bool is_acc_mode,
    const functionmanager::PerceptionSubState current_state,
    SpeedLimitInfo* speed_limit_info) {
  if (discretized_path_last_valid_ ||
      curvature_speed_limit_data_last_.speed_limit_points().size() < 2) {
    return;
  }
  double last_path_s = path_s + delta_s;
  auto comp = [](const auto& speed_limit_point, const double last_path_s) {
    return speed_limit_point.first < last_path_s;
  };
  auto iter_lower = std::lower_bound(
      curvature_speed_limit_data_last_.speed_limit_points().begin(),
      curvature_speed_limit_data_last_.speed_limit_points().end(), last_path_s,
      comp);
  std::pair<double, double> path_info_last;
  if (iter_lower ==
      curvature_speed_limit_data_last_.speed_limit_points().begin()) {
    path_info_last =
        curvature_speed_limit_data_last_.speed_limit_points().front();
  } else {
    if (iter_lower ==
        curvature_speed_limit_data_last_.speed_limit_points().end()) {
      path_info_last =
          curvature_speed_limit_data_last_.speed_limit_points().back();
    } else {
      const auto iter_low = iter_lower - 1;
      path_info_last.first =
          std::abs(iter_low->first - iter_lower->first) > 0.01
              ? iter_lower->first + ((path_s + delta_s) - iter_lower->first) /
                                        (iter_low->first - iter_lower->first)
              : iter_lower->first;
      path_info_last.second =
          std::abs(iter_low->first - iter_lower->first) > 0.01
              ? iter_lower->second + ((path_s + delta_s) - iter_lower->first) /
                                         (iter_low->first - iter_lower->first) *
                                         (iter_low->second - iter_lower->second)
              : iter_lower->second;
    }
  }
  if (((mode_change_count_ < 20 && mode_change_count_ > 0) &&
       std::abs(speed_limit_info->curvature_speed_limit -
                path_info_last.second) > 2) ||
      (is_acc_mode && is_acc_mode_last_ &&
       (current_state == functionmanager::CRUISE_TYPE) &&
       !mode_change_valid_)) {
    speed_limit_info->curvature_speed_limit =
        0.2 *
            (speed_limit_info->curvature_speed_limit - path_info_last.second) +
        path_info_last.second;
  }
}

double SpeedLimitDecider::EgoMovingStep(const LocalView& local_view) {
  const double v = local_view.GetVehicleState()->linear_velocity();
  const double t = 0.1;
  const double dx = v * t;
  const double dy = 0.5 * local_view.GetSubjectKappa() * std::pow(dx, 2);
  return std::sqrt(dx * dx + dy * dy);
}

double SpeedLimitDecider::GetFreeSpaceSpeedLimit(
    const common::PathPoint& path_point,
    const std::unordered_map<uint32_t, perception::PerceptionObstacle_Type>&
        box_id_type,
    const std::vector<FreeSpaceSegment>& freespace_segments,
    const bool is_forward, const double cruise_speed) {
  static constexpr double kA = 6;
  static constexpr double kB = -10.6;
  static constexpr double kAPedestrian = 6.5;
  static constexpr double kBPedestrian = -10;
  static constexpr double kAKappa = -5.8;
  static constexpr double kBKappa = 130;
  static constexpr double kMinRation = 0.2;
  static constexpr double kFsFilter = 2.0;
  const auto kappa = fabs(path_point.kappa());
  double min_limit_speed =
      is_forward ? speed_bounds_config_.min_freespace_limit_speed_forward()
                 : speed_bounds_config_.min_freespace_limit_speed_reverse();
  double min_limit_speed_straight_road =
      speed_bounds_config_.freespace_limit_speed_straight_road();
  double min_limit_speed_kappa =
      std::max(0.0, min_limit_speed_straight_road - min_limit_speed) /
          (1.0 + exp(kAKappa + kBKappa * kappa)) +
      min_limit_speed;
  const auto ego_polygon = common::VehicleConfigHelper::GetPolygon2dWithBuffer(
      path_point.x(), path_point.y(), path_point.theta());
  const auto ego_box = common::VehicleConfigHelper::GetBoundingBox(path_point);
  double near_distance = std::numeric_limits<double>::max();
  double near_distance_pedestrian = std::numeric_limits<double>::max();
  for (const auto& fs_segment : freespace_segments) {
    if (ego_box.DistanceTo(fs_segment.segment) > kFsFilter) {
      continue;
    }
    const auto iter = box_id_type.find(fs_segment.obstacleId);
    if (fs_segment.isLinkObjFusion && iter != box_id_type.end()) {
      continue;
    }
    const auto ego_distance_to_fs_segment =
        ego_polygon.DistanceTo(fs_segment.segment);
    near_distance = fmin(near_distance, ego_distance_to_fs_segment);
    if (fs_segment.cls_type == perception::FreeSpaceOut::PEDESTRAIN) {
      near_distance_pedestrian =
          fmin(near_distance_pedestrian, ego_distance_to_fs_segment);
    }
  }

  double ratio = std::max(kMinRation, cruise_speed - min_limit_speed_kappa);
  double speed_limit_fs = ratio * (1.0 / (1.0 + exp(kA + kB * near_distance))) +
                          min_limit_speed_kappa;
  double speed_limit_pedestrian =
      (std::max(kMinRation, cruise_speed - min_limit_speed) /
       (1.0 + exp(kAPedestrian + kBPedestrian * near_distance_pedestrian))) +
      min_limit_speed;
  return fmin(speed_limit_pedestrian, speed_limit_fs);
}

double SpeedLimitDecider::GetAvpSteerDiffSpeedLimit(
    const double kappa, const double real_steer_angle,
    const double cruise_speed) {
  static constexpr double kMaxNegativeKappa = -0.25;
  static constexpr double kMaxPositiveKappa = 0.25;
  static constexpr double kA = 25;
  static constexpr double kB = -0.16;
  static constexpr double kMinRation = 0.0;
  double steer_angle_diff_speed_limit =
      speed_bounds_config_.steer_angle_diff_speed_limit();
  const auto max_negative_steer_angle_deg =
      -vehicle_param_.max_steer_angle() * 180.0 / M_PI;
  const auto max_positive_steer_angle_deg =
      vehicle_param_.max_steer_angle() * 180.0 / M_PI;
  const auto path_point_kappa =
      common::math::Clamp(kappa, kMaxNegativeKappa, kMaxPositiveKappa);
  const double ego_real_steer_angle =
      common::math::Clamp(real_steer_angle, max_negative_steer_angle_deg,
                          max_positive_steer_angle_deg);
  // Calculate the front wheel angle (in deg)
  double front_wheel_angle =
      atan(vehicle_param_.wheel_base() * path_point_kappa) * 180.0 / M_PI;
  // Calculate the steering wheel angle (in deg)
  double expected_steering_wheel_angle = common::math::Clamp(
      vehicle_param_.steer_ratio() * front_wheel_angle,
      max_negative_steer_angle_deg, max_positive_steer_angle_deg);
  double ratio =
      std::min(kMinRation, steer_angle_diff_speed_limit - cruise_speed);
  double angle_diff =
      std::fabs(ego_real_steer_angle - expected_steering_wheel_angle);
  double speed_limit =
      ratio * (1.0 / (1.0 + exp(kA + kB * angle_diff))) + cruise_speed;

  return speed_limit;
}

double SpeedLimitDecider::GetNudgeSpeedLimit(
    const common::FrenetFramePoint& frenet_frame_point,
    const IndexedList<std::string, Obstacle>& obstacles,
    double map_speed_limit) const {
  double nudge_speed_limit = FLAGS_planning_upper_speed_limit;
  const double collision_safety_range =
      speed_bounds_config_.collision_safety_range();
  for (const auto* ptr_obstacle : obstacles.Items()) {
    if (ptr_obstacle->IsVirtual()) {
      continue;
    }
    if (!ptr_obstacle->LateralDecision().has_nudge()) {
      continue;
    }

    /* ref line:
     * -------------------------------
     *    start_s   end_s
     * ------|  adc   |---------------
     * ------------|  obstacle |------
     */

    // TODO(all): potential problem here;
    // frenet and cartesian coordinates are mixed.
    const double vehicle_front_s =
        frenet_frame_point.s() + vehicle_param_.front_edge_to_center();
    const double vehicle_back_s =
        frenet_frame_point.s() - vehicle_param_.back_edge_to_center();
    const double obstacle_front_s =
        ptr_obstacle->PerceptionSLBoundary().end_s();
    const double obstacle_back_s =
        ptr_obstacle->PerceptionSLBoundary().start_s();

    if (vehicle_front_s < obstacle_back_s ||
        vehicle_back_s > obstacle_front_s) {
      continue;
    }

    const auto& nudge_decision = ptr_obstacle->LateralDecision().nudge();

    // Please notice the differences between adc_l and frenet_point_l
    const double frenet_point_l = frenet_frame_point.l();

    // obstacle is on the right of ego vehicle (at path point i)
    bool is_close_on_left =
        (nudge_decision.type() == ObjectNudge::LEFT_NUDGE) &&
        (frenet_point_l - vehicle_param_.right_edge_to_center() -
             collision_safety_range <
         ptr_obstacle->PerceptionSLBoundary().end_l());

    // obstacle is on the left of ego vehicle (at path point i)
    bool is_close_on_right =
        (nudge_decision.type() == ObjectNudge::RIGHT_NUDGE) &&
        (ptr_obstacle->PerceptionSLBoundary().start_l() -
             collision_safety_range <
         frenet_point_l + vehicle_param_.left_edge_to_center());

    // TODO(all): dynamic obstacles do not have nudge decision
    if (is_close_on_left || is_close_on_right) {
      double nudge_speed_ratio = 1.0;
      if (ptr_obstacle->IsStatic()) {
        nudge_speed_ratio = speed_bounds_config_.static_obs_nudge_speed_ratio();
      } else {
        nudge_speed_ratio =
            speed_bounds_config_.dynamic_obs_nudge_speed_ratio();
      }
      nudge_speed_limit =
          fmin(nudge_speed_limit, nudge_speed_ratio * map_speed_limit);
    }
  }
  return nudge_speed_limit;
}

double SpeedLimitDecider::GetPedestrianSpeedLimit(
    const common::PathPoint& path_point, double point_s_interval,
    const std::vector<PedestrianPoint>& pedestrians_point,
    const bool is_avp_mode) const {
  const auto s_unit = speed_bounds_config_.pedestrian_speed_limit_s_unit();
  const auto l_unit = speed_bounds_config_.pedestrian_speed_limit_l_unit();
  const auto s_count = static_cast<int>(round(point_s_interval / s_unit)) + 1;
  const auto l_count =
      static_cast<int>(round(vehicle_param_.width() / l_unit)) + 1;

  double max_risk = std::numeric_limits<double>::lowest();

  // const auto temp1 =
  //     1 / (2 * M_PI * sigma_x * sigma_y * sqrt(1 - pow(correlation, 2)));
  const double front_edge_to_center =
      is_avp_mode ? 0.0 : vehicle_param_.front_edge_to_center();
  const double risk_exponent = is_avp_mode ? 1.0 : 1.25;
  const auto cos_path_point_theta = cos(path_point.theta());
  const auto sin_path_point_theta = sin(path_point.theta());

  for (int i = 0; i <= s_count; ++i) {
    const auto s = front_edge_to_center + i * s_unit;
    const auto x1 = path_point.x() + s * cos_path_point_theta;
    const auto y1 = path_point.y() + s * sin_path_point_theta;
    for (int j = 0; j <= l_count; ++j) {
      const auto l = -vehicle_param_.left_edge_to_center() + j * l_unit;
      const auto x = x1 + l * sin_path_point_theta;
      const auto y = y1 - l * cos_path_point_theta;
      for (const auto& pedestrian_point : pedestrians_point) {
        const auto risk =
            ((-pow((x - pedestrian_point.mean_x) * pedestrian_point.sigma_x,
                   2) -
              pow((y - pedestrian_point.mean_y) * pedestrian_point.sigma_y, 2) +
              pedestrian_point.double_correlation *
                  (x - pedestrian_point.mean_x) *
                  (y - pedestrian_point.mean_y) *
                  (pedestrian_point.product_sigma_xy)) *
             pedestrian_point.temp2);
        if (std::isnan(risk)) {
          continue;
        }
        if (risk > max_risk) {
          max_risk = risk;
        }
      }
    }
  }

  auto limit_speed = pow(-max_risk, risk_exponent) /
                     speed_bounds_config_.pedestrian_speed_limit_ratio();
  limit_speed = std::min(limit_speed, FLAGS_planning_upper_speed_limit);
  limit_speed = std::max(limit_speed, 0.0);

  return limit_speed;
}

double SpeedLimitDecider::GetDKappaSpeedLimit(
    const common::PathPoint& path_point,
    const common::PathPoint& next_path_point) const {
  const auto delta_steer_angle =
      fabs(atan(vehicle_param_.wheel_base() * next_path_point.kappa()) -
           atan(vehicle_param_.wheel_base() * path_point.kappa())) *
      vehicle_param_.steer_ratio() * 180.0 / M_PI;
  const auto delta_s = next_path_point.s() - path_point.s();
  const auto& calibration_infos =
      speed_bounds_config_.max_steer_angle_speed_calibration_table()
          .calibration_info();
  for (auto iter = calibration_infos.rbegin(); iter != calibration_infos.rend();
       ++iter) {
    const auto t = delta_steer_angle / iter->max_steer_angle_speed();
    if (common::math::double_type::SeemsEqual(t, 0.0)) {
      return FLAGS_speed_upper_bound;
    }
    const auto v = delta_s / t;
    if (v > iter->speed() / 3.6) {
      return v;
    }
  }
  return 0.0;
}

bool SpeedLimitDecider::LoadCentricAccelCalibrationTableReverse() {
  const auto calbration_size =
      speed_bounds_config_.max_lateral_acc_calbration_table_reverse()
          .calibration_info_size();
  if (calbration_size < 2) {
    return false;
  }

  centric_accel_calibration_table_avp_reverse_.first.reserve(calbration_size);
  centric_accel_calibration_table_avp_reverse_.second.reserve(calbration_size);
  for (int i = calbration_size - 1; i >= 0; --i) {
    const auto& pair =
        speed_bounds_config_.max_lateral_acc_calbration_table_reverse()
            .calibration_info(i);
    if (common::math::double_type::SeemsEqual(pair.radius(), 0.0)) {
      centric_accel_calibration_table_avp_reverse_.first.emplace_back(
          std::numeric_limits<double>::max());
    } else {
      centric_accel_calibration_table_avp_reverse_.first.emplace_back(
          1.0 / pair.radius());
    }

    centric_accel_calibration_table_avp_reverse_.second.emplace_back(
        pair.max_lateral_acc());
  }

  return true;
}

bool SpeedLimitDecider::LoadCentricAccelCalibrationTableForward() {
  const auto calbration_size =
      speed_bounds_config_.max_lateral_acc_calbration_table_forward()
          .calibration_info_size();
  if (calbration_size < 2) {
    return false;
  }

  centric_accel_calibration_table_avp_forward_.first.reserve(calbration_size);
  centric_accel_calibration_table_avp_forward_.second.reserve(calbration_size);
  for (int i = calbration_size - 1; i >= 0; --i) {
    const auto& pair =
        speed_bounds_config_.max_lateral_acc_calbration_table_forward()
            .calibration_info(i);
    if (common::math::double_type::SeemsEqual(pair.radius(), 0.0)) {
      centric_accel_calibration_table_avp_forward_.first.emplace_back(
          std::numeric_limits<double>::max());
    } else {
      centric_accel_calibration_table_avp_forward_.first.emplace_back(
          1.0 / pair.radius());
    }

    centric_accel_calibration_table_avp_forward_.second.emplace_back(
        pair.max_lateral_acc());
  }

  return true;
}

void SpeedLimitDecider::GetConeSpeedLimitSegments(
    const ReferenceLineInfo& reference_line_info) {
  double KBufferLfet = 1.5;
  double KBufferRight = 1.5;

  if (left_cone_speed_limit_mode_) {
    KBufferLfet = 2.5;
  } else if (right_cone_speed_limit_mode_) {
    KBufferRight = 2.5;
  }
  const auto& path_envelope = reference_line_info.path_data().GetPathEnvelope();
  auto adc_frenet_l_upper = path_envelope.max_ref_l + KBufferLfet;
  auto adc_frenet_l_lower = path_envelope.min_ref_l - KBufferRight;
  const auto& adc_middle_l =
      (path_envelope.max_ref_l + path_envelope.min_ref_l) / 2;
  double sum_left_l_dis = 0;
  double sum_right_l_dis = 0;
  double left_cone_size = 0;
  double right_cone_size = 0;

  for (const auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (obstacle == nullptr || !obstacle->IsCone() ||
        !obstacle->path_st_boundary().IsEmpty()) {
      continue;
    }

    // 判断有效锥桶个数
    const double obs_middle_l = (obstacle->PerceptionSLBoundary().start_l() +
                                 obstacle->PerceptionSLBoundary().end_l()) *
                                0.5;
    const double obs_middle_s = (obstacle->PerceptionSLBoundary().start_s() +
                                 obstacle->PerceptionSLBoundary().end_s()) *
                                0.5;
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    reference_line_info.reference_line().GetLaneWidth(
        obs_middle_s, &lane_left_width, &lane_right_width);
    if ((obs_middle_l < adc_frenet_l_upper ||
         obs_middle_l < lane_left_width + 0.3) &&
        obs_middle_l > adc_middle_l) {
      sum_left_l_dis += fabs(obs_middle_l - path_envelope.max_ref_l);
      left_cone_speed_limit_segment_.end_s =
          fmax(obs_middle_s, left_cone_speed_limit_segment_.end_s);
      left_cone_speed_limit_segment_.start_s =
          fmin(obs_middle_s, left_cone_speed_limit_segment_.start_s);
      left_cone_size++;
    } else if ((obs_middle_l > adc_frenet_l_lower ||
                obs_middle_l > -lane_right_width - 0.3) &&
               obs_middle_l < adc_middle_l) {
      sum_right_l_dis += fabs(obs_middle_l - path_envelope.min_ref_l);
      right_cone_speed_limit_segment_.end_s =
          fmax(obs_middle_s, right_cone_speed_limit_segment_.end_s);
      right_cone_speed_limit_segment_.start_s =
          fmin(obs_middle_s, right_cone_speed_limit_segment_.start_s);
      right_cone_size++;
    }
  }

  if (left_cone_size >= kEffectiveCone) {
    left_cone_speed_limit_segment_.ave_l = sum_left_l_dis / left_cone_size;
  }
  if (right_cone_size >= kEffectiveCone) {
    right_cone_speed_limit_segment_.ave_l = sum_right_l_dis / right_cone_size;
  }
  // 判断是否进入锥桶限速
  if (right_cone_size >= kEffectiveCone && !right_cone_speed_limit_mode_) {
    right_cone_speed_limit_mode_ = true;

  } else if (right_cone_size == 0 && right_cone_speed_limit_mode_) {
    right_cone_speed_limit_mode_ = false;
    right_cone_speed_limit_segment_.end_s = 0;
    right_cone_speed_limit_segment_.start_s = 0;
    right_cone_speed_limit_segment_.ave_l = 0;
    right_cone_speed_limit_segment_.speed_limit = FLAGS_speed_upper_bound;
  }
  if (left_cone_size >= kEffectiveCone && !left_cone_speed_limit_mode_) {
    left_cone_speed_limit_mode_ = true;

  } else if (left_cone_size == 0 && left_cone_speed_limit_mode_) {
    left_cone_speed_limit_mode_ = false;
    left_cone_speed_limit_segment_.end_s = 0;
    left_cone_speed_limit_segment_.start_s = 0;
    left_cone_speed_limit_segment_.ave_l = 0;
    left_cone_speed_limit_segment_.speed_limit = FLAGS_speed_upper_bound;
  }
  // 计算锥桶限速段
  if (right_cone_speed_limit_mode_) {
    right_cone_speed_limit_segment_.speed_limit =
        common::math::InterpolationOne(right_cone_speed_limit_segment_.ave_l,
                                       cone_l_conf_, cone_v_conf_);
  }
  if (left_cone_speed_limit_mode_) {
    left_cone_speed_limit_segment_.speed_limit = common::math::InterpolationOne(
        left_cone_speed_limit_segment_.ave_l, cone_l_conf_, cone_v_conf_);
  }
}

double SpeedLimitDecider::GetConeSpeedLimit(
    const common::PathPoint& path_point) const {
  const auto& adc_s = path_point.s() - vehicle_param_.back_edge_to_center();
  if ((adc_s > right_cone_speed_limit_segment_.start_s &&
       adc_s < right_cone_speed_limit_segment_.end_s) ||
      (adc_s > left_cone_speed_limit_segment_.start_s &&
       adc_s < left_cone_speed_limit_segment_.end_s)) {
    return fmin(right_cone_speed_limit_segment_.speed_limit,
                left_cone_speed_limit_segment_.speed_limit);
  }
  return FLAGS_speed_upper_bound;
}

}  // namespace planning
}  // namespace TL
