/******************************************************************************
 * Copyright 2017 The TL Authors. All Rights Reserved.
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
// NOLINTBEGIN
#include "planning/self_simulator/sim_control/sim_control.h"
#include <math.h>
#include <cmath>
#include <memory>

#include "common/configs/vehicle_config_helper.h"
#include "common/file/file.h"
#include "common/file/log.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/quaternion.h"
#include "common/time/clock.h"
#include "common/util/message_util.h"
#include "common/util/util.h"
#include "common/utm_projection/coordinate_convertor.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/util/util.h"
#include "planning/pnc_map/pnc_map.h"

#include "proto/common/types.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/routing/poi.pb.h"
#include "proto/soc/chassis.pb.h"

#define SIM_DUMMY_OBS (1)
#if SIM_DUMMY_OBS
#include "planning/self_simulator/sim_dummy_obs/obs_manager.h"
#endif

namespace TL {
namespace planning {

using TL::common::Clock;
using TL::soc::Chassis;
using TL::soc::WarningSwitch;
using TL::soc::WarningSwitchMemory;
// using TL::common::Quaternion;
using TL::common::TrajectoryPoint;
using TL::common::math::HeadingToQuaternion;
using TL::common::math::InterpolateUsingLinearApproximation;
using TL::common::util::FillHeader;
using TL::hdmap::HDMapUtil;
using TL::localization::Localization;
using TL::perception::PerceptionObstacles;
using TL::planning::ADCTrajectory;
// using TL::prediction::PredictionObstacles;
using TL::routing::RoutingResponse;

SimControl::SimControl() {
#if SIM_DUMMY_OBS
  TL::simdummy::PerceptionObsManager::getInstance().InitializeObsList();
#endif
}

void SimControl::Init() {
  adc_trajectory_ = std::make_shared<ADCTrajectory>();
}

void SimControl::InitStartPoint(double start_velocity,
                                double start_acceleration) {
  if (FLAGS_use_ehp) {
    InitStartPointForEHP(start_velocity, start_acceleration);
  } else if (FLAGS_use_hdmap_mode) {
    InitStartPointForHDMap(start_velocity, start_acceleration);
  } else {
    InitStartPointForHDMap(start_velocity, start_acceleration);
  }
}

void SimControl::InitStartPointForHDMap(double start_velocity,
                                        double start_acceleration) {
  TrajectoryPoint point;
  // Use the latest localization position as start point,
  // fall back to a dummy point from map
  if (!local_view_ || !local_view_->HasLocalization()) {
    start_point_from_localization_ = false;
    TL::common::PointENU start_point;
    hdmap::LaneInfoConstPtr nearest_lane;
    double s = 0.0;
    double l = 0.0;
    double theta = 0.0;
    start_point.set_x(0.0);
    start_point.set_y(0.0);
    routing::POI poi;
    if (common::GetProtoFromASCIIFile(hdmap::EndWayPointFile(), &poi) &&
        poi.landmark_size() > 0 && poi.landmark(0).waypoint_size() > 0) {
      const auto& pose = poi.landmark(0).waypoint(0).pose();
      start_point.set_x(pose.x());
      start_point.set_y(pose.y());
      HDMapUtil::BaseMap().GetNearestLane(start_point, &nearest_lane, &s, &l);
    } else {
      HDMapUtil::BaseMap().GetNearestLane(start_point, &nearest_lane, &s, &l);
      start_point = nearest_lane->GetSmoothPoint(0.0);
    }
    if (!nearest_lane) {
      AWARN << "Failed to get a dummy start point from map!";
      return;
    }
    point.mutable_path_point()->set_x(start_point.x());
    point.mutable_path_point()->set_y(start_point.y());
    point.mutable_path_point()->set_z(start_point.z());
    theta = nearest_lane->Heading(s);
    point.mutable_path_point()->set_theta(theta);
    point.set_v(start_velocity);
    point.set_a(start_acceleration);
    SetStartPoint(point);
    planning_zone_ = FLAGS_local_utm_zone_id;
  }
}

bool SimControl::InitStartPointForEHP(double start_velocity,
                                      double start_acceleration) {
  // load landmark from file
  routing::POI poi;
  if (!common::GetProtoFromASCIIFile(hdmap::EndWayPointFile(), &poi)) {
    return false;
  }

  for (auto& landmark : *poi.mutable_landmark()) {
    if (landmark.name() == "ehp_start_point_utm") {
      for (auto& waypoint : *landmark.mutable_waypoint()) {
        auto x = waypoint.mutable_pose()->x();
        auto y = waypoint.mutable_pose()->y();
        auto z = waypoint.mutable_pose()->z();
        planning_zone_ = std::stoi(waypoint.id());
        waypoint.mutable_pose()->set_x(x);
        waypoint.mutable_pose()->set_y(y);
        waypoint.mutable_pose()->set_z(z);
        break;
      }
      break;
    } else if (landmark.name() == "ehp_start_point") {
      for (auto& waypoint : *landmark.mutable_waypoint()) {
        auto x = waypoint.mutable_pose()->x();
        auto y = waypoint.mutable_pose()->y();
        auto z = waypoint.mutable_pose()->z();
        current_lon_ = x;
        planning_zone_ = x / 6 + 31;
        if (std::abs(x) < 180) {
          common::coordinate_convertor::GCS2UTM(planning_zone_, &x, &y);
          waypoint.mutable_pose()->set_x(x);
          waypoint.mutable_pose()->set_y(y);
          waypoint.mutable_pose()->set_z(z);
          auto heading = (90.0 - waypoint.heading()) / 180 * M_PI;
          if (heading < -M_PI) {
            waypoint.set_heading(heading + 2 * M_PI);
          } else {
            waypoint.set_heading(heading);
          }
        }
        break;
      }
      break;
    }
  }

  // make sure there is at least one waypoint
  if (poi.landmark_size() <= 0 || poi.landmark().at(0).waypoint_size() <= 0) {
    AERROR << "there must be at least one waypoint";
    return false;
  }

  TrajectoryPoint point;
  auto& waypoint = poi.landmark().at(0).waypoint().at(0);
  point.mutable_path_point()->set_x(waypoint.pose().x());
  point.mutable_path_point()->set_y(waypoint.pose().y());
  point.mutable_path_point()->set_z(0.0);
  point.mutable_path_point()->set_theta(waypoint.heading());
  point.set_v(start_velocity);
  point.set_a(start_acceleration);
  point.set_da(0.0);

  SetStartPoint(point);
  return true;
}

void SimControl::SetStartPoint(const TrajectoryPoint& start_point) {
  next_point_ = start_point;
  prev_point_index_ = 0;
  next_point_index_ = 0;
  received_planning_ = false;
}

void SimControl::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  InternalReset();
}

void SimControl::InternalReset() {
  re_routing_triggered_ = false;
  send_dummy_perception_ = true;
  ClearPlanning();
}

void SimControl::ClearPlanning() {
  received_planning_ = false;
}

void SimControl::OnRoutingResponse(
    const std::shared_ptr<const routing::RoutingResponse>& routing) {
  return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }

  if (routing->routing_request().waypoint_size() < 2) {
    return;
  }

  // CHECK_GE(routing->routing_request().waypoint_size(), 2)
  //     << "routing should have at least two waypoints";
  const auto& start_pose = routing->routing_request().waypoint(0).pose();

  // If this is from a planning re-routing request, or the start point has
  // been
  // initialized by an actual localization pose, don't reset the start point.
  // when use ehp, the routing response will change
  if (!FLAGS_use_ehp) {
    re_routing_triggered_ =
        hdmap::PncMap::IsNewRouting(*routing, *current_routing_);
    if (re_routing_triggered_) {
      current_routing_ = routing;
      ClearPlanning();

      TrajectoryPoint point;
      point.mutable_path_point()->set_x(start_pose.x());
      point.mutable_path_point()->set_y(start_pose.y());
      point.set_a(next_point_.has_a() ? next_point_.a() : 0.0);
      point.set_v(next_point_.has_v() ? next_point_.v() : 0.0);
      double theta = 0.0;
      double s = 0.0;
      double l = 0.0;
      hdmap::LaneInfoConstPtr nearest_lane;
      HDMapUtil::MapForPlanning().GetNearestLane(start_pose, &nearest_lane, &s,
                                                 &l);
      if (!nearest_lane) {
        AWARN << "Failed to get a dummy start point from map!";
        return;
      }
      theta = nearest_lane->Heading(s);
      point.mutable_path_point()->set_theta(theta);
      SetStartPoint(point);
    }
  }
}

void SimControl::Start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    InternalReset();
    enabled_ = true;
  }
}

void SimControl::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (enabled_) {
    enabled_ = false;
  }
}

void SimControl::OnPlanning(
    const std::shared_ptr<const ADCTrajectory>& adc_trajectory) {
  std::lock_guard<std::mutex> lock(mutex_);
  adc_trajectory_->CopyFrom(*adc_trajectory);
  if (adc_trajectory_->is_vehicle_reference_frame()) {
    common::Pose pose;
    if (adc_trajectory_->debug().planning_data().adc_position().has_pose()) {
      pose = adc_trajectory_->debug().planning_data().adc_position().pose();
    } else {
      pose = adc_position_;
    }
    for (int i = 0; i < adc_trajectory_->trajectory_point_size(); i++) {
      auto* point = adc_trajectory_->mutable_trajectory_point()->Mutable(i);
      Eigen::Vector2d enu_point = TL::common::math::RotateVector2d(
          {point->path_point().x(), point->path_point().y()}, pose.heading());
      enu_point.x() += pose.position().x();
      enu_point.y() += pose.position().y();
      point->mutable_path_point()->set_x(enu_point.x());
      point->mutable_path_point()->set_y(enu_point.y());
      point->mutable_path_point()->set_theta(pose.heading());
    }
  }
  if (adc_trajectory_->utm_zone_id() != 0) {
    planning_zone_ = adc_trajectory_->utm_zone_id();
  }
  if (!enabled_) {
    return;
  }

  if (adc_trajectory_->trajectory_point_size() >= 2) {
    prev_point_index_ = 0;
    next_point_index_ = 0;
    received_planning_ = true;
  } else {
    received_planning_ = false;
  }
}

void SimControl::Freeze() {
  next_point_.set_v(0.0);
  next_point_.set_a(0.0);
  prev_point_ = next_point_;
}

void SimControl::Process(const std::shared_ptr<LocalView>& local_view,
                         std::shared_ptr<hdmap::HDMap> map_ptr) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_started_) {
    InitStartPoint(next_point_.has_v() ? next_point_.v() : 0.0,
                   next_point_.has_a() ? next_point_.a() : 0.0);
    is_started_ = true;
  }
  if (local_view == nullptr) {
    return;
  }
  local_view_ = local_view;

  TrajectoryPoint trajectory_point;
  Chassis::GearPosition gear_position;
  if (!PerfectControlModel(adc_trajectory_, &trajectory_point,
                           &gear_position)) {
    AERROR << "Failed to calculate next point with perfect control model";
    return;
  }
  PublishChassis(trajectory_point, gear_position);
  PublishLocalization(trajectory_point);
  PublishDummyPerception(adc_trajectory_, map_ptr);
}

bool SimControl::PerfectControlModel(
    const std::shared_ptr<const TL::planning::ADCTrajectory>& adc_trajectory,
    TrajectoryPoint* point, Chassis::GearPosition* gear_position) {
  // Result of the interpolation.
  auto current_time = Clock::NowInSeconds();
  const auto& trajectory_points = adc_trajectory->trajectory_point();
  *gear_position = adc_trajectory->gear();

  if (!received_planning_) {
    prev_point_ = next_point_;
  } else {
    if (adc_trajectory->estop().is_estop() ||
        next_point_index_ >= trajectory_points.size()) {
      // Freeze the car when there's an estop or the current trajectory has
      // been exhausted.
      Freeze();
    } else {
      // Determine the status of the car based on received planning message.
      while (next_point_index_ < trajectory_points.size() &&
             current_time >
                 trajectory_points.Get(next_point_index_).relative_time() +
                     adc_trajectory->header().data_stamp()) {
        ++next_point_index_;
      }

      if (next_point_index_ >= trajectory_points.size()) {
        next_point_index_ = trajectory_points.size() - 1;
      }

      if (next_point_index_ == 0) {
        AERROR << "First trajectory point is a future point!";
        return false;
      }

      prev_point_index_ = next_point_index_ - 1;

      next_point_ = trajectory_points.Get(next_point_index_);
      prev_point_ = trajectory_points.Get(prev_point_index_);
    }
  }

  if (current_time >
      next_point_.relative_time() + adc_trajectory->header().data_stamp()) {
    // Don't try to extrapolate if relative_time passes last point
    *point = next_point_;
  } else {
    *point = InterpolateUsingLinearApproximation(
        prev_point_, next_point_,
        current_time - adc_trajectory->header().data_stamp());
  }
  return true;
}

void SimControl::PublishChassis(  // NOLINT
    const TL::common::TrajectoryPoint& trajectory_point,
    Chassis::GearPosition gear_position) {
  auto chassis = std::make_shared<Chassis>();
  FillHeader("SelfSimulator", chassis.get());

  chassis->set_engine_started(true);
  chassis->set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  chassis->set_gear_location(gear_position);

  chassis->set_speed_mps(static_cast<float>(trajectory_point.v()));
  if (gear_position == soc::Chassis::GEAR_REVERSE) {
    chassis->set_speed_mps(-chassis->speed_mps());
  }
  chassis->set_speed_display(
      static_cast<int32_t>(fabs(chassis->speed_mps() * 3.6)));
  const auto& vehicle_params =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  if (trajectory_point.a() > 0) {
    chassis->set_throttle_percentage(trajectory_point.a() /
                                     vehicle_params.max_acceleration() * 100.0);
    chassis->set_brake_percentage(0.0);
  } else {
    chassis->set_throttle_percentage(0.0);
    chassis->set_brake_percentage(trajectory_point.a() /
                                  vehicle_params.max_deceleration() * 100.0);
  }

  double steering_angle = std::atan(vehicle_params.wheel_base() *
                                    trajectory_point.path_point().kappa()) *
                          vehicle_params.steer_ratio();
  chassis->set_steering_angle(steering_angle / M_PI * 180.0);
  chassis->set_steering_percentage(steering_angle /
                                   vehicle_params.max_steer_angle() * 100.0);
  chassis->set_yaw_rate(trajectory_point.path_point().kappa() *
                        trajectory_point.v());
  chassis->mutable_imu_acc()->set_y(trajectory_point.a());
  chassis->set_battery_soc_percentage(66);
  const auto& fct_nnp_in = local_view_->GetFunctionManagerIn()->fct_nnp_in();
  if (fct_nnp_in.has_turnlight_reqst()) {
    const auto& fct_nnp_in_turn_light = fct_nnp_in.turnlight_reqst();
    ADEBUG << "fct_nnp_in_turn_light: "
           << functionmanager::TurnLightReq_Name(fct_nnp_in_turn_light);
    switch (fct_nnp_in_turn_light) {
      case functionmanager::TurnLightReq::RIGHT_TURN_LIGHT:
        chassis->mutable_signal()->set_turn_switch(
            common::TurnLightSwitchStatus::RIGHT_LEVEL_1);
        break;
      case functionmanager::TurnLightReq::RIGHT_TURN_LIGHT_STATUS:
        chassis->mutable_signal()->set_turn_switch(
            common::TurnLightSwitchStatus::RIGHT_LEVEL_2);
        break;
      case functionmanager::TurnLightReq::LEFT_TURN_LIGHT:
        chassis->mutable_signal()->set_turn_switch(
            common::TurnLightSwitchStatus::LEFT_LEVEL_1);
        break;
      case functionmanager::TurnLightReq::LEFT_TURN_LIGHT_STATUS:
        chassis->mutable_signal()->set_turn_switch(
            common::TurnLightSwitchStatus::LEFT_LEVEL_2);
        break;
      case functionmanager::TurnLightReq::NO_REQ_LIGHT:
        chassis->mutable_signal()->set_turn_switch(
            common::TurnLightSwitchStatus::NONE_REQUEST);
        break;
      default:
        chassis->mutable_signal()->set_turn_switch(
            common::TurnLightSwitchStatus::ERROR);
        break;
    }
  }

  ADEBUG << "chassis turn_switch(): "
         << common::TurnLightSwitchStatus_Name(
                chassis->mutable_signal()->turn_switch());

  chassis->mutable_warning_switch_mem()->set_dow_on_off_set_mem(
      WarningSwitchMemory::ON);
  chassis->mutable_warning_switch_mem()->set_lca_on_off_set_mem(
      WarningSwitchMemory::ON);
  chassis->mutable_warning_switch_mem()->set_fcta_on_off_set_mem(
      WarningSwitchMemory::ON);
  chassis->mutable_warning_switch_mem()->set_rcta_on_off_set_mem(
      WarningSwitchMemory::ON);
  chassis->mutable_warning_switch_mem()->set_rcw_on_off_set_mem(
      WarningSwitchMemory::ON);

  chassis->mutable_warning_switch_from_cdcs()->set_dow_on_off_set(
      WarningSwitch::ON);
  chassis->mutable_warning_switch_from_cdcs()->set_lca_on_off_set(
      WarningSwitch::ON);
  chassis->mutable_warning_switch_from_cdcs()->set_fcta_on_off_set(
      WarningSwitch::ON);
  chassis->mutable_warning_switch_from_cdcs()->set_rcta_on_off_set(
      WarningSwitch::ON);
  chassis->mutable_warning_switch_from_cdcs()->set_rcw_on_off_set(
      WarningSwitch::ON);

  chassis->mutable_vehicle_cfg()->set_dow(true);
  chassis->mutable_vehicle_cfg()->set_lca(true);
  chassis->mutable_vehicle_cfg()->set_fcta(true);
  chassis->mutable_vehicle_cfg()->set_rcta(true);
  chassis->mutable_vehicle_cfg()->set_rcw(true);

  if (local_view_->HasChassis()) {
    chassis->mutable_switch_info()->CopyFrom(
        local_view_->GetChassis()->switch_info());
  }

  local_view_->SetChassisPtr(chassis);
}

void SimControl::PublishLocalization(const TrajectoryPoint& point) {
  auto localization = std::make_shared<Localization>();
  FillHeader("SelfSimulator", localization.get());

  auto* pose = localization->mutable_pose();
  auto prev = prev_point_.path_point();
  auto next = next_point_.path_point();

  // Set position lat long
  pose->mutable_position()->set_x(point.path_point().x());
  pose->mutable_position()->set_y(point.path_point().y());
  pose->mutable_position()->set_z(point.path_point().z());
  double lon = point.path_point().x();
  double lat = point.path_point().y();
  common::coordinate_convertor::UTM2GCS(planning_zone_, &lon, &lat);
  // pose->mutable_wgs()->set_x(lon);
  // pose->mutable_wgs()->set_y(lat);
  // pose->mutable_wgs()->set_z(0.0);
  current_lon_ = lon;
  pose->mutable_gcj02()->set_x(lon);
  pose->mutable_gcj02()->set_y(lat);
  pose->mutable_gcj02()->set_z(0.0);

  int zone_current = current_lon_ / 6 + 31;
  int zone_nearest =
      (static_cast<int>(current_lon_) % 6 >= 3 ? 1 : -1) + zone_current;
  if (planning_zone_ == zone_current) {
    pose->mutable_pos_utm_01()->set_x(point.path_point().x());
    pose->mutable_pos_utm_01()->set_y(point.path_point().y());
    pose->mutable_pos_utm_01()->set_z(point.path_point().z());
    pose->set_utm_zone_01(zone_current);
    common::coordinate_convertor::GCS2UTM(zone_nearest, &lon, &lat);
    pose->mutable_pos_utm_02()->set_x(lon);
    pose->mutable_pos_utm_02()->set_y(lat);
    pose->mutable_pos_utm_02()->set_z(point.path_point().z());
    pose->set_utm_zone_02(zone_nearest);
  } else if (planning_zone_ == zone_nearest) {
    pose->mutable_pos_utm_02()->set_x(point.path_point().x());
    pose->mutable_pos_utm_02()->set_y(point.path_point().y());
    pose->mutable_pos_utm_02()->set_z(point.path_point().z());
    pose->set_utm_zone_02(zone_nearest);
    common::coordinate_convertor::GCS2UTM(zone_current, &lon, &lat);
    pose->mutable_pos_utm_01()->set_x(lon);
    pose->mutable_pos_utm_01()->set_y(lat);
    pose->mutable_pos_utm_01()->set_z(point.path_point().z());
    pose->set_utm_zone_01(zone_current);
  } else {
    AERROR << "UTM zone error";
  }
  // 如果读取地图进行仿真，强制utm带号不再动态变化，在这里置为默认值
  if (!FLAGS_use_ehp) {
    pose->set_using_utm_zone(planning_zone_);
  }

  if (FLAGS_use_ehp && FLAGS_enable_planning_self_simulator) {
    pose->set_using_utm_zone(planning_zone_);
  }

  // Set orientation and heading
  double cur_theta = point.path_point().theta();

  Eigen::Quaternion<double> cur_orientation =
      HeadingToQuaternion<double>(cur_theta);
  pose->mutable_quaternion()->set_w(cur_orientation.w());
  pose->mutable_quaternion()->set_x(cur_orientation.x());
  pose->mutable_quaternion()->set_y(cur_orientation.y());
  pose->mutable_quaternion()->set_z(cur_orientation.z());
  double heading_error_from_gcs_to_utm =
      common::coordinate_convertor::HeadingError(
          planning_zone_, pose->gcj02().x(), pose->gcj02().y());
  pose->set_heading(cur_theta);
  pose->set_heading_gcs(cur_theta - heading_error_from_gcs_to_utm);

  // Set linear_velocity
  pose->mutable_linear_velocity()->set_x(std::cos(cur_theta) * point.v());
  pose->mutable_linear_velocity()->set_y(std::sin(cur_theta) * point.v());
  pose->mutable_linear_velocity()->set_z(0);
  common::util::TransformToVRF(pose->linear_velocity(), pose->quaternion(),
                               pose->mutable_linear_velocity_vrf());

  // Set angular_velocity in both map reference frame and vehicle reference
  // frame
  pose->mutable_angular_velocity()->set_x(0);
  pose->mutable_angular_velocity()->set_y(0);
  pose->mutable_angular_velocity()->set_z(point.v() *
                                          point.path_point().kappa());

  common::util::TransformToVRF(pose->angular_velocity(), pose->quaternion(),
                               pose->mutable_angular_velocity_vrf());

  // Set linear_acceleration in both map reference frame and vehicle reference
  // frame
  auto* linear_acceleration = pose->mutable_linear_acceleration();
  linear_acceleration->set_x(std::cos(cur_theta) * point.a());
  linear_acceleration->set_y(std::sin(cur_theta) * point.a());
  linear_acceleration->set_z(0);

  common::util::TransformToVRF(pose->linear_acceleration(), pose->quaternion(),
                               pose->mutable_linear_acceleration_vrf());
  localization->set_location_state(2);
  local_view_->SetLocalizationPtr(localization);

  adc_position_.CopyFrom(localization->pose());
}

void SimControl::PublishDummyPerception(
    const std::shared_ptr<const TL::planning::ADCTrajectory>&
        adc_trajectory_ptr,
    std::shared_ptr<hdmap::HDMap> map_ptr) {
  auto perception = std::make_shared<PerceptionObstacles>();
  if (!send_dummy_perception_) {
    return;
  }
  FillHeader("SelfSimulator", perception.get());
#if SIM_DUMMY_OBS
  double delta_time = 0;
  if (time_stamp_prev < 0.01) {
    time_stamp_prev = common::Clock::NowInSeconds();
  } else {
    delta_time = common::Clock::NowInSeconds() - time_stamp_prev;
    time_stamp_prev = common::Clock::NowInSeconds();
  }
  auto adc_path_point = std::make_shared<common::PathPoint>();
  adc_path_point->set_x(adc_position_.position().x());
  adc_path_point->set_y(adc_position_.position().y());
  adc_path_point->set_z(adc_position_.position().z());
  if (adc_trajectory_ptr && FLAGS_enable_sim_dummy_obs) {
    TL::simdummy::PerceptionObsManager::getInstance().Update(
        delta_time, adc_trajectory_ptr, adc_path_point, perception, local_view_,
        map_ptr);
  }
#endif
  auto lane_markers = std::make_shared<perception::LaneMarkers>();
  lane_markers->mutable_front_left_lane_marker()->set_lane_type(
      hdmap::LaneBoundaryType::DOTTED_WHITE);
  lane_markers->mutable_front_left_lane_marker()->set_quality(1.0);
  lane_markers->mutable_front_left_lane_marker()->set_c0_position(1.7);
  lane_markers->mutable_front_left_lane_marker()->set_c1_heading_angle(0.0);
  lane_markers->mutable_front_left_lane_marker()->set_c2_curvature(0.0);
  lane_markers->mutable_front_left_lane_marker()->set_c3_curvature_derivative(
      0.0);
  lane_markers->mutable_front_left_lane_marker()->set_view_range(50.0);
  lane_markers->mutable_front_left_lane_marker()->set_longitude_start(0.0);
  lane_markers->mutable_front_left_lane_marker()->set_longitude_end(50.0);
  lane_markers->mutable_front_right_lane_marker()->set_lane_type(
      hdmap::LaneBoundaryType::DOTTED_WHITE);
  lane_markers->mutable_front_right_lane_marker()->set_quality(1.0);
  lane_markers->mutable_front_right_lane_marker()->set_c0_position(-1.7);
  lane_markers->mutable_front_right_lane_marker()->set_c1_heading_angle(0.0);
  lane_markers->mutable_front_right_lane_marker()->set_c2_curvature(0.0);
  lane_markers->mutable_front_right_lane_marker()->set_c3_curvature_derivative(
      0.0);
  lane_markers->mutable_front_right_lane_marker()->set_view_range(50.0);
  lane_markers->mutable_front_right_lane_marker()->set_longitude_start(0.0);
  lane_markers->mutable_front_right_lane_marker()->set_longitude_end(50.0);

  auto* front_next_right_lane_marker =
      lane_markers->add_front_next_right_lane_marker();
  front_next_right_lane_marker->set_lane_type(
      hdmap::LaneBoundaryType::DOTTED_WHITE);
  front_next_right_lane_marker->set_quality(1.0);
  front_next_right_lane_marker->set_c0_position(-5.1);
  front_next_right_lane_marker->set_c1_heading_angle(0.0);
  front_next_right_lane_marker->set_c2_curvature(0.0);
  front_next_right_lane_marker->set_c3_curvature_derivative(0.0);
  front_next_right_lane_marker->set_view_range(50.0);
  front_next_right_lane_marker->set_longitude_start(0.0);
  front_next_right_lane_marker->set_longitude_end(50.0);

  auto* front_next_left_lane_marker =
      lane_markers->add_front_next_left_lane_marker();
  front_next_left_lane_marker->set_lane_type(
      hdmap::LaneBoundaryType::DOTTED_WHITE);
  front_next_left_lane_marker->set_quality(1.0);
  front_next_left_lane_marker->set_c0_position(5.1);
  front_next_left_lane_marker->set_c1_heading_angle(0.0);
  front_next_left_lane_marker->set_c2_curvature(0.0);
  front_next_left_lane_marker->set_c3_curvature_derivative(0.0);
  front_next_left_lane_marker->set_view_range(50.0);
  front_next_left_lane_marker->set_longitude_start(0.0);
  front_next_left_lane_marker->set_longitude_end(50.0);

  perception->mutable_lane_marker()->CopyFrom(*lane_markers);
  local_view_->SetPerceptionObstaclesPtr(perception);
  local_view_->SetPerceptionObstaclesMinieyePtr(perception);
  local_view_->SetLaneMarkersPtr(lane_markers);
  local_view_->SetLaneMarkersMinieyePtr(lane_markers);
}

void SimControl::AddStaticObsatclesBothAndSingleSide(
    const std::shared_ptr<TL::perception::PerceptionObstacles>& perception) {
  auto* obs = perception->add_perception_obstacle();
  obs->set_id(2);
  obs->mutable_position()->set_x(264648.39);
  obs->mutable_position()->set_y(3380090.54);
  obs->mutable_position()->set_z(0.5);
  obs->set_type(perception::PerceptionObstacle::VEHICLE);
  obs->set_theta(0.0);
  obs->set_length(5.0);
  obs->set_width(2.5);
  obs->set_height(2.0);
  obs->mutable_velocity()->set_x(0.0);
  obs->mutable_velocity()->set_y(0.0);
  obs->mutable_velocity()->set_z(0.0);
  obs->mutable_acceleration()->set_x(0.0);
  obs->mutable_acceleration()->set_y(0.0);
  obs->mutable_acceleration()->set_z(0.0);
  obs->mutable_bbox2d()->set_xmax(2.5);
  obs->mutable_bbox2d()->set_xmin(2.5);
  obs->mutable_bbox2d()->set_ymin(5.0);
  obs->mutable_bbox2d()->set_ymin(5.0);

  auto* obs_1 = perception->add_perception_obstacle();
  obs_1->set_id(3);
  obs_1->mutable_position()->set_x(264695.52);
  obs_1->mutable_position()->set_y(3380095.42);
  obs_1->mutable_position()->set_z(0.5);
  obs_1->set_type(perception::PerceptionObstacle::VEHICLE);
  obs_1->set_theta(0.0);
  obs_1->set_length(5.0);
  obs_1->set_width(2.5);
  obs_1->set_height(2.0);
  obs_1->mutable_velocity()->set_x(0.0);
  obs_1->mutable_velocity()->set_y(0.0);
  obs_1->mutable_velocity()->set_z(0.0);
  obs_1->mutable_acceleration()->set_x(0.0);
  obs_1->mutable_acceleration()->set_y(0.0);
  obs_1->mutable_acceleration()->set_z(0.0);
  obs_1->mutable_bbox2d()->set_xmax(2.5);
  obs_1->mutable_bbox2d()->set_xmin(2.5);
  obs_1->mutable_bbox2d()->set_ymin(5.0);
  obs_1->mutable_bbox2d()->set_ymin(5.0);

  auto* obs_2 = perception->add_perception_obstacle();
  obs_2->set_id(4);
  obs_2->mutable_position()->set_x(264734.48);
  obs_2->mutable_position()->set_y(3380109.88);
  obs_2->mutable_position()->set_z(0.5);
  obs_2->set_type(perception::PerceptionObstacle::VEHICLE);
  obs_2->set_theta(0.0);
  obs_2->set_length(5.0);
  obs_2->set_width(2.5);
  obs_2->set_height(2.0);
  obs_2->mutable_velocity()->set_x(0.0);
  obs_2->mutable_velocity()->set_y(0.0);
  obs_2->mutable_velocity()->set_z(0.0);
  obs_2->mutable_acceleration()->set_x(0.0);
  obs_2->mutable_acceleration()->set_y(0.0);
  obs_2->mutable_acceleration()->set_z(0.0);
  obs_2->mutable_bbox2d()->set_xmax(2.5);
  obs_2->mutable_bbox2d()->set_xmin(2.5);
  obs_2->mutable_bbox2d()->set_ymin(5.0);
  obs_2->mutable_bbox2d()->set_ymin(5.0);

  auto* obs_3 = perception->add_perception_obstacle();
  obs_3->set_id(5);
  obs_3->mutable_position()->set_x(264757.93);
  obs_3->mutable_position()->set_y(3380114.80);
  obs_3->mutable_position()->set_z(0.5);
  obs_3->set_type(perception::PerceptionObstacle::VEHICLE);
  obs_3->set_theta(0.0);
  obs_3->set_length(5.0);
  obs_3->set_width(2.5);
  obs_3->set_height(2.0);
  obs_3->mutable_velocity()->set_x(0.0);
  obs_3->mutable_velocity()->set_y(0.0);
  obs_3->mutable_velocity()->set_z(0.0);
  obs_3->mutable_acceleration()->set_x(0.0);
  obs_3->mutable_acceleration()->set_y(0.0);
  obs_3->mutable_acceleration()->set_z(0.0);
  obs_3->mutable_bbox2d()->set_xmax(2.5);
  obs_3->mutable_bbox2d()->set_xmin(2.5);
  obs_3->mutable_bbox2d()->set_ymin(5.0);
  obs_3->mutable_bbox2d()->set_ymin(5.0);
}

void SimControl::AddStaticObsatclesSideBySide(
    const std::shared_ptr<TL::perception::PerceptionObstacles>& perception) {
  auto* obs = perception->add_perception_obstacle();
  obs->set_id(2);
  obs->mutable_position()->set_x(264648.45);
  obs->mutable_position()->set_y(3380091.02);
  obs->mutable_position()->set_z(0.5);
  obs->set_type(perception::PerceptionObstacle::VEHICLE);
  obs->set_theta(0.0);
  obs->set_length(5.0);
  obs->set_width(2.5);
  obs->set_height(2.0);
  obs->mutable_velocity()->set_x(0.0);
  obs->mutable_velocity()->set_y(0.0);
  obs->mutable_velocity()->set_z(0.0);
  obs->mutable_acceleration()->set_x(0.0);
  obs->mutable_acceleration()->set_y(0.0);
  obs->mutable_acceleration()->set_z(0.0);
  obs->mutable_bbox2d()->set_xmax(2.5);
  obs->mutable_bbox2d()->set_xmin(2.5);
  obs->mutable_bbox2d()->set_ymin(5.0);
  obs->mutable_bbox2d()->set_ymin(5.0);

  auto* obs_1 = perception->add_perception_obstacle();
  obs_1->set_id(3);
  obs_1->mutable_position()->set_x(264695.99);
  obs_1->mutable_position()->set_y(3380094.98);
  obs_1->mutable_position()->set_z(0.5);
  obs_1->set_type(perception::PerceptionObstacle::VEHICLE);
  obs_1->set_theta(0.0);
  obs_1->set_length(5.0);
  obs_1->set_width(2.5);
  obs_1->set_height(2.0);
  obs_1->mutable_velocity()->set_x(0.0);
  obs_1->mutable_velocity()->set_y(0.0);
  obs_1->mutable_velocity()->set_z(0.0);
  obs_1->mutable_acceleration()->set_x(0.0);
  obs_1->mutable_acceleration()->set_y(0.0);
  obs_1->mutable_acceleration()->set_z(0.0);
  obs_1->mutable_bbox2d()->set_xmax(2.5);
  obs_1->mutable_bbox2d()->set_xmin(2.5);
  obs_1->mutable_bbox2d()->set_ymin(5.0);
  obs_1->mutable_bbox2d()->set_ymin(5.0);
  auto* obs_2 = perception->add_perception_obstacle();
  obs_2->set_id(4);
  obs_2->mutable_position()->set_x(264653.80);
  obs_2->mutable_position()->set_y(3380085.13);
  obs_2->mutable_position()->set_z(0.5);
  obs_2->set_type(perception::PerceptionObstacle::VEHICLE);
  obs_2->set_theta(0.0);
  obs_2->set_length(5.0);
  obs_2->set_width(2.5);
  obs_2->set_height(2.0);
  obs_2->mutable_velocity()->set_x(0.0);
  obs_2->mutable_velocity()->set_y(0.0);
  obs_2->mutable_velocity()->set_z(0.0);
  obs_2->mutable_acceleration()->set_x(0.0);
  obs_2->mutable_acceleration()->set_y(0.0);
  obs_2->mutable_acceleration()->set_z(0.0);
  obs_2->mutable_bbox2d()->set_xmax(2.5);
  obs_2->mutable_bbox2d()->set_xmin(2.5);
  obs_2->mutable_bbox2d()->set_ymin(5.0);
  obs_2->mutable_bbox2d()->set_ymin(5.0);

  auto* obs_3 = perception->add_perception_obstacle();
  obs_3->set_id(5);
  obs_3->mutable_position()->set_x(264697.60);
  obs_3->mutable_position()->set_y(3380102.42);
  obs_3->mutable_position()->set_z(0.5);
  obs_3->set_type(perception::PerceptionObstacle::VEHICLE);
  obs_3->set_theta(0.0);
  obs_3->set_length(5.0);
  obs_3->set_width(2.5);
  obs_3->set_height(2.0);
  obs_3->mutable_velocity()->set_x(0.0);
  obs_3->mutable_velocity()->set_y(0.0);
  obs_3->mutable_velocity()->set_z(0.0);
  obs_3->mutable_acceleration()->set_x(0.0);
  obs_3->mutable_acceleration()->set_y(0.0);
  obs_3->mutable_acceleration()->set_z(0.0);
  obs_3->mutable_bbox2d()->set_xmax(2.5);
  obs_3->mutable_bbox2d()->set_xmin(2.5);
  obs_3->mutable_bbox2d()->set_ymin(5.0);
  obs_3->mutable_bbox2d()->set_ymin(5.0);
}

// NOLINTEND
}  // namespace planning
}  // namespace TL
