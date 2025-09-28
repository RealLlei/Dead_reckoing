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
 * @file frame.cc
 **/
#include "planning/common/frame.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "absl/strings/str_cat.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/time/clock.h"
#include "common/util/point_factory.h"
#include "map/hdmap/hdmap_util.h"
#include "map/hdmap/path.h"
#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/util/util.h"
#include "planning/prediction/common/prediction_gflags.h"
#include "planning/reference_line/reference_line_provider.h"

#include "proto/fsm/avp_fct.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/perception/transport_element.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {

using TL::common::Clock;  // NOLINT
using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::math::Box2d;
using TL::common::math::Polygon2d;
using TL::common::math::double_type::Compare;
using TL::prediction::PredictionObstacles;

DrivingAction Frame::pad_msg_driving_action_ = DrivingAction::NONE;

FrameHistory::FrameHistory()
    : IndexedQueue<uint32_t, Frame>(FLAGS_max_frame_history_num) {}

Frame::Frame(uint32_t sequence_num) : sequence_num_(sequence_num) {}

Frame::Frame(uint32_t sequence_num,
             const std::shared_ptr<LocalView>& local_view,
             common::TrajectoryPoint planning_start_point,
             common::VehicleState vehicle_state,
             ReferenceLineProvider* reference_line_provider)
    : sequence_num_(sequence_num),
      local_view_(local_view),
      planning_start_point_(std::move(planning_start_point)),
      vehicle_state_(std::move(vehicle_state)),
      reference_line_provider_(reference_line_provider) {}

// monitor_logger_buffer_(common::monitor::MonitorMessageItem::PLANNING) {}

Frame::Frame(uint32_t sequence_num,
             const std::shared_ptr<LocalView>& local_view,
             const common::TrajectoryPoint& planning_start_point,
             const common::VehicleState& vehicle_state)
    : Frame(sequence_num, local_view, planning_start_point, vehicle_state,
            nullptr) {}

const common::TrajectoryPoint& Frame::PlanningStartPoint() const {
  return planning_start_point_;
}

const common::VehicleState& Frame::vehicle_state() const {
  return vehicle_state_;
}

bool Frame::Rerouting(PlanningContext* planning_context) {
  ADEBUG << "start rerouting";
  if (!local_view_->HasRoutingResponse()) {
    AERROR << "No previous routing available";
    return false;
  }
  if (!hdmap_) {
    AERROR << "Invalid HD Map.";
    return false;
  }
  auto request = local_view_->GetRoutingResponse()->routing_request();
  request.clear_header();

  auto point = common::util::PointFactory::ToPointENU(vehicle_state_);
  double s = 0.0;
  double l = 0.0;
  hdmap::LaneInfoConstPtr lane;
  if (hdmap_->GetNearestLaneWithHeading(point, 5.0, vehicle_state_.heading(),
                                        M_PI / 3.0, &lane, &s, &l) != 0) {
    AERROR << "Failed to find nearest lane from map at position: "
           << point.DebugString() << ", heading:" << vehicle_state_.heading();
    return false;
  }
  request.clear_waypoint();
  auto* start_point = request.add_waypoint();
  start_point->set_id(lane->id().id());
  start_point->set_s(s);
  start_point->mutable_pose()->CopyFrom(point);
  for (const auto& waypoint : future_route_waypoints_) {
    // reference_line_provider_->FutureRouteWaypoints()) {
    request.add_waypoint()->CopyFrom(waypoint);
  }
  if (request.waypoint_size() <= 1) {
    AERROR << "Failed to find future waypoints";
    return false;
  }

  auto* rerouting =
      planning_context->mutable_planning_status()->mutable_rerouting();
  rerouting->set_need_rerouting(true);
  *rerouting->mutable_routing_request() = request;

  // monitor_logger_buffer_.INFO("Planning send Rerouting request");
  return true;
}

const std::list<ReferenceLineInfo>& Frame::reference_line_info() const {
  return reference_line_info_;
}

std::list<ReferenceLineInfo>* Frame::mutable_reference_line_info() {
  return &reference_line_info_;
}

void Frame::UpdateReferenceLinePriority(
    const std::map<std::string, uint32_t>& id_to_priority) {
  for (const auto& pair : id_to_priority) {
    const auto id = pair.first;
    const auto priority = pair.second;
    auto ref_line_info_itr =
        std::find_if(reference_line_info_.begin(), reference_line_info_.end(),
                     [&id](const ReferenceLineInfo& ref_line_info) {
                       return ref_line_info.Lanes().Id() == id;
                     });
    if (ref_line_info_itr != reference_line_info_.end()) {
      ref_line_info_itr->SetPriority(priority);
    }
  }
}

bool Frame::CreateReferenceLineInfo(
    const std::list<std::shared_ptr<ReferenceLine>>& reference_lines,
    const std::list<hdmap::RouteSegments>& segments) {
  reference_line_info_.clear();
  if (reference_lines.empty() || segments.empty() ||
      (reference_lines.size() != segments.size())) {
    return false;
  }
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();

  auto vehicle_state_ptr = std::make_shared<common::VehicleState>();
  vehicle_state_ptr->CopyFrom(vehicle_state_);

  while (ref_line_iter != reference_lines.end()) {
    if (segments_iter->StopForDestination()) {
      is_near_destination_ = true;
    }
    reference_line_info_.emplace_back(vehicle_state_ptr, planning_start_point_,
                                      *ref_line_iter, *segments_iter);
    if (cur_state_machine_ ==
        functionmanager::MachineStateType::HISTORY_TRACE_TYPE) {
      reference_line_info_.back().SetIsHistoryTrace();
    }
    ++ref_line_iter;
    ++segments_iter;
  }

  if (reference_line_info_.size() == 2) {
    common::math::Vec2d xy_point(vehicle_state_ptr->x(),
                                 vehicle_state_ptr->y());
    common::SLPoint first_sl;
    if (!reference_line_info_.front().reference_line().XYToSL(xy_point,
                                                              &first_sl)) {
      return false;
    }
    common::SLPoint second_sl;
    if (!reference_line_info_.back().reference_line().XYToSL(xy_point,
                                                             &second_sl)) {
      return false;
    }
    const double offset = first_sl.l() - second_sl.l();
    reference_line_info_.front().SetOffsetToOtherReferenceLine(offset);
    reference_line_info_.back().SetOffsetToOtherReferenceLine(-offset);
  }

  bool has_valid_reference_line = false;
  for (auto& ref_info : reference_line_info_) {
    if (!ref_info.Init(obstacles(), *local_view_->GetFunctionManagerIn())) {
      AERROR << "Failed to init reference line";
    } else {
      has_valid_reference_line = true;
    }
  }
  return has_valid_reference_line;
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
std::shared_ptr<Obstacle> Frame::CreateStopObstacle(
    ReferenceLineInfo* const reference_line_info,
    const std::string& obstacle_id, const double obstacle_s) {
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto& reference_line = reference_line_info->reference_line();
  const double box_center_s = obstacle_s + FLAGS_virtual_stop_wall_length / 2.0;
  auto box_center = reference_line.GetReferencePoint(box_center_s);
  double heading = reference_line.GetReferencePoint(obstacle_s).heading();
  static constexpr double kStopWallWidth = 10.0;
  Box2d stop_wall_box{box_center, heading, FLAGS_virtual_stop_wall_length,
                      kStopWallWidth};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
std::shared_ptr<Obstacle> Frame::CreateStopObstacle(
    const std::string& obstacle_id, const std::string& lane_id,
    const double lane_s) {
  if (!hdmap_) {
    AERROR << "Invalid HD Map.";
    return nullptr;
  }
  const auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(lane_id));
  if (!lane) {
    AERROR << "Failed to find lane[" << lane_id << "]";
    return nullptr;
  }

  double dest_lane_s = std::max(0.0, lane_s);
  auto dest_point = lane->GetSmoothPoint(dest_lane_s);

  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  lane->GetWidth(dest_lane_s, &lane_left_width, &lane_right_width);

  Box2d stop_wall_box{{dest_point.x(), dest_point.y()},
                      lane->Heading(dest_lane_s),
                      FLAGS_virtual_stop_wall_length,
                      lane_left_width + lane_right_width};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * @brief: create static virtual object with lane width,
 */
std::shared_ptr<Obstacle> Frame::CreateStaticObstacle(
    ReferenceLineInfo* const reference_line_info,
    const std::string& obstacle_id, const double obstacle_start_s,
    const double obstacle_end_s) {
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  if (std::fabs(obstacle_start_s - obstacle_end_s) < 1e-2) {
    AERROR << "obstacle start_s almost equal end_s!";
    return nullptr;
  }

  const auto& reference_line = reference_line_info->reference_line();

  // start_xy
  common::SLPoint sl_point;
  sl_point.set_s(obstacle_start_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_start_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_start_xy)) {
    AERROR << "Failed to get start_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  // end_xy
  sl_point.set_s(obstacle_end_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_end_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_end_xy)) {
    AERROR << "Failed to get end_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  if (!reference_line.GetLaneWidth(obstacle_start_s, &left_lane_width,
                                   &right_lane_width)) {
    AERROR << "Failed to get lane width at s[" << obstacle_start_s << "]";
    return nullptr;
  }

  common::math::Box2d obstacle_box{
      common::math::LineSegment2d(obstacle_start_xy, obstacle_end_xy),
      left_lane_width + right_lane_width};

  if (obstacle_box.width() < 1e-3 || obstacle_box.length() < 1e-3) {
    AERROR << "obstacle is too small!";
    return nullptr;
  }

  return CreateStaticVirtualObstacle(obstacle_id, obstacle_box);
}

std::shared_ptr<Obstacle> Frame::CreateStaticVirtualObstacle(
    const std::string& id, const Box2d& box) {
  const auto* object = obstacles_.Find(id);
  if (object != nullptr) {
    AWARN << "obstacle " << id << " already exist.";
    return *object;
  }
  auto* ptr =
      obstacles_.Add(id, Obstacle::CreateStaticVirtualObstacles(id, box));
  if (ptr == nullptr) {
    AERROR << "Failed to create virtual obstacle " << id;
  }
  return *ptr;
}

Status Frame::Init(
    const common::VehicleState* vehicle_state,
    const std::list<std::shared_ptr<ReferenceLine>>& reference_lines,
    const std::list<hdmap::RouteSegments>& segments,
    const std::vector<routing::LaneWaypoint>& future_route_waypoints,
    const EgoInfo* ego_info) {
  // TODO(QiL): refactor this to avoid redundant nullptr checks in scenarios.
  auto status = InitFrameData(vehicle_state, ego_info);
  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  if (!CreateReferenceLineInfo(reference_lines, segments)) {
    const std::string msg = "Failed to init reference line info.";
    AERROR << msg;
    return Status(ErrorCode::CORE_FRAME_INITFRAME_ERROR, msg);
  }
  future_route_waypoints_ = future_route_waypoints;

  return Status::OK();
}

Status Frame::InitForOpenSpace(const common::VehicleState* vehicle_state,
                               const EgoInfo* ego_info) {
  return InitFrameData(vehicle_state, ego_info);
}

TL::common::Status Frame::OpenSpaceCollisionCheck(
    const std::vector<std::pair<common::math::LineSegment2d, double>>* const
        obs_segments_pair_ptr) {
  const auto adc_polygon = common::VehicleConfigHelper::GetPolygon2dWithBuffer(
      vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.heading());
  bool is_collision = false;
  if (nullptr != obs_segments_pair_ptr) {
    for (const auto& obs : *obs_segments_pair_ptr) {
      if (adc_polygon.HasOverlap(obs.first)) {
        AINFO << "ADC is collsion with seg: " << obs.first.DebugString();
        is_collision = true;
        break;
      }
    }
  } else {
    for (const auto& obstacle : obstacles_.Items()) {
      if ((*obstacle)->IsVirtual() || (*obstacle)->IsLowHeight()) {
        continue;
      }
      const auto& obstacle_polygon = (*obstacle)->PerceptionPolygon();
      if (obstacle_polygon.HasOverlap(adc_polygon)) {
        AINFO << "ADC is collsion with polygon: "
              << obstacle_polygon.DebugString();
        is_collision = true;
        break;
      }
    }
  }
  if (is_collision) {
    const double direction =
        vehicle_state_.gear() == soc::Chassis::GEAR_REVERSE ? -1.0 : 1.0;
    static constexpr double kCollisionAccel = -1.0;
    const bool is_real_collision =
        !IsVehicleStandStill() &&
        direction * vehicle_state_.linear_acceleration() < kCollisionAccel;
    return is_real_collision
               ? Status(ErrorCode::PLANNER_PARKING_COLLISION_ERROR,
                        "vehicle collision with obstacle and big deceleration")
               : Status(ErrorCode::CORE_FRAME_INITFRAME_ERROR,
                        "ADC is collsion with obstacle");
  }
  return Status::OK();
}

Status Frame::InitFrameData(const common::VehicleState* vehicle_state,
                            const EgoInfo* /*ego_info*/) {
  hdmap_ = hdmap::HDMapUtil::MapPtrForPlanning();
  CHECK_NOTNULL(hdmap_);
  vehicle_state_ = *vehicle_state;
  if (!util::IsVehicleStateValid(vehicle_state_)) {
    AERROR << "Adc init point is not set";
    return Status(ErrorCode::CORE_FRAME_INITFRAME_ERROR,
                  "Adc init point is not set");
  }
  ADEBUG << "Enabled align prediction time ? : " << std::boolalpha
         << FLAGS_align_prediction_time;

  {
    if (!local_view_->HasFunctionManagerIn()) {
      const std::string msg = "LocalView has no function_manager_in.";
      AERROR << msg;
      return Status(ErrorCode::CORE_FRAME_INITFRAME_ERROR, msg);
    }

    const auto is_acc_mode =
        (local_view_->HasFunctionManagerOut() &&
         local_view_->GetFunctionManagerOut() != nullptr &&
         local_view_->GetFunctionManagerOut()->fsm_state() ==
             functionmanager::MachineStateType::PERCEPTION_TYPE &&
         local_view_->GetFunctionManagerOut()->perception_sub_state() ==
             functionmanager::PerceptionSubState::CRUISE_TYPE);
    const auto& fct_avp_in = local_view_->GetFunctionManagerIn()->fct_avp_in();

    SetEgoPredTrajectory();  // set ego prediction trajectory

    std::list<std::shared_ptr<Obstacle>> ptr_list =
        Obstacle::CreateObstacles(*local_view_->GetPredictionObstacles());
    for (auto& ptr : ptr_list) {
      if (ptr->IsUssObs() &&
          (cur_state_machine_ ==
               functionmanager::MachineStateType::HISTORY_TRACE_TYPE ||
           (cur_state_machine_ ==
                functionmanager::MachineStateType::HDMAP_AVP_TYPE &&
            fct_avp_in.sys_run_state() !=
                functionmanager::AvpFctIn::PARKING))) {
        continue;
      }

      if (is_acc_mode) {
        if (local_view_->GetCruiseTargetId().empty() ||
            (std::find(local_view_->GetCruiseTargetId().begin(),
                       local_view_->GetCruiseTargetId().end(),
                       ptr->PerceptionId()) ==
             local_view_->GetCruiseTargetId().end())) {
          continue;
        }

        ObjectDecisionType ignore;
        ignore.mutable_ignore();
        ptr->AddLateralDecision("acc_filter", ignore);
      }

      AddObstacle(ptr);
    }
  }
  // if (std::fabs(planning_start_point_.v()) < 1e-3 &&
  //     (!local_view_->HasFunctionManagerIn() ||
  //      local_view_->GetFunctionManagerIn()->ta_pilot_mode() !=
  //          functionmanager::TaPilotMode::AVP)) {
  //   const auto* collision_obstacle = FindCollisionObstacle(ego_info);
  //   if (collision_obstacle != nullptr) {
  //     const std::string msg = absl::StrCat("Found collision with obstacle: ",
  //                                          collision_obstacle->Id());
  //     AERROR << msg;
  //     // monitor_logger_buffer_.ERROR(msg);
  //     return Status(ErrorCode::CORE_FRAME_INITFRAME_COLLISION_ERROR, msg);
  //   }
  // }

  ReadTrafficLights();

  ReadPadMsgDrivingAction();

  return Status::OK();
}

void Frame::SetEgoPredTrajectory() {
  // find ego in prediction
  ego_prediction_trajectories_.Clear();
  if (local_view_->GetPredictionObstacles() == nullptr ||
      local_view_->GetPredictionObstacles()->prediction_obstacle().empty()) {
    return;
  }
  const auto& prediction_obstacles = local_view_->GetPredictionObstacles();
  for (const auto& pred : prediction_obstacles->prediction_obstacle()) {
    if (FLAGS_ego_vehicle_id == pred.perception_obstacle().id()) {
      if (!pred.trajectory().empty()) {
        ego_prediction_trajectories_.CopyFrom(pred.trajectory());
      }
      break;
    }
  }
}

const Obstacle* Frame::FindCollisionObstacle(const EgoInfo* ego_info) const {
  if (obstacles_.Items().empty()) {
    return nullptr;
  }
  bool use_precise_check =
      local_view_->HasFunctionManagerIn() &&
      (local_view_->GetFunctionManagerIn()->ta_pilot_mode() ==
       functionmanager::AVP);

  const auto& adc_polygon =
      use_precise_check
          ? common::VehicleConfigHelper::GetPolygon2dWithBuffer(
                ego_info->vehicle_state().x(), ego_info->vehicle_state().y(),
                ego_info->vehicle_state().heading())
          : Polygon2d(ego_info->ego_box());
  for (const auto& obstacle : obstacles_.Items()) {
    if ((*obstacle)->IsVirtual() || (*obstacle)->IsLowHeight()) {
      continue;
    }

    const auto& obstacle_polygon = (*obstacle)->PerceptionPolygon();
    if (obstacle_polygon.HasOverlap(adc_polygon)) {
      return (*obstacle).get();
    }
  }
  return nullptr;
}

uint32_t Frame::SequenceNum() const {
  return sequence_num_;
}

std::string Frame::DebugString() const {
  return absl::StrCat("Frame: ", sequence_num_);
}

void Frame::RecordInputDebug(planning_internal::Debug* const debug) {
  if (debug == nullptr) {
    ADEBUG << "Skip record input into debug";
    return;
  }
  // auto* planning_debug_data = debug->mutable_planning_data();
  // auto* adc_position = planning_debug_data->mutable_adc_position();
  // if (local_view_->HasLocalization()) {
  //   adc_position->CopyFrom(*local_view_->GetLocalization());
  // }

  // auto debug_chassis = planning_debug_data->mutable_chassis();
  // if (local_view_->HasChassis()) {
  //   debug_chassis->CopyFrom(*local_view_->GetChassis());
  // }

  // if (local_view_->HasPredictionObstacles()) {
  //   planning_debug_data->mutable_prediction_header()->CopyFrom(
  //       local_view_->GetPredictionObstacles()->header());
  // }

  /*
  auto navigation_hdmap = AdapterManager::GetNavigationHdmap();
  if (!navigation_hdmap->Empty()) {
    planning_debug_data->mutable_navigation_hdmap()->mutable_header()->CopyFrom(
        navigation_hdmap->GetLatestObserved().header());
  }
  */
}

void Frame::AlignPredictionTime(const double planning_start_time,
                                PredictionObstacles* prediction_obstacles) {
  if ((prediction_obstacles == nullptr) ||
      !prediction_obstacles->has_header() ||
      !prediction_obstacles->header().has_data_stamp()) {
    return;
  }
  double prediction_header_time = prediction_obstacles->header().data_stamp();
  for (auto& obstacle : *prediction_obstacles->mutable_prediction_obstacle()) {
    for (auto& trajectory : *obstacle.mutable_trajectory()) {
      for (auto& point : *trajectory.mutable_trajectory_point()) {
        point.set_relative_time(prediction_header_time + point.relative_time() -
                                planning_start_time);
      }
      if (!trajectory.trajectory_point().empty() &&
          trajectory.trajectory_point().begin()->relative_time() < 0) {
        auto it = trajectory.trajectory_point().begin();
        while (it != trajectory.trajectory_point().end() &&
               it->relative_time() < 0) {
          ++it;
        }
        trajectory.mutable_trajectory_point()->erase(
            trajectory.trajectory_point().begin(), it);
      }
    }
  }
}

Obstacle* Frame::Find(const std::string& id) {
  return obstacles_.Find(id)->get();
}

void Frame::AddObstacle(const std::shared_ptr<Obstacle>& obstacle) {
  obstacles_.Add(obstacle->Id(), obstacle);
}

void Frame::ReadTrafficLights() {
  traffic_lights_.clear();
  if (!local_view_->HasTrafficLightDetection()) {
    return;
  }
  const auto& traffic_lights = local_view_->GetTrafficLightDetection();
  if (traffic_lights == nullptr || !traffic_lights->has_passable_info()) {
    return;
  }
  const auto& passable_info = traffic_lights->passable_info();
  auto& left_light = traffic_lights_["left"];
  left_light.set_id("left");
  left_light.set_color(passable_info.turn_left());
  left_light.set_confidence(1.0);

  auto& right_light = traffic_lights_["right"];
  right_light.set_id("left");
  right_light.set_color(passable_info.turn_right());
  right_light.set_confidence(1.0);

  auto& straight_light = traffic_lights_["straight"];
  straight_light.set_id("straight");
  straight_light.set_color(passable_info.straight());
  straight_light.set_confidence(1.0);

  auto& turn_around_light = traffic_lights_["u_turn"];
  turn_around_light.set_id("u_turn");
  turn_around_light.set_color(passable_info.turn_around());
  turn_around_light.set_confidence(1.0);
}

perception::TrafficLight Frame::GetSignal(
    const std::string& traffic_light_id) const {
  if (traffic_lights_.count(traffic_light_id) == 0) {
    perception::TrafficLight traffic_light;
    traffic_light.set_id(traffic_light_id);
    traffic_light.set_color(global_traffic_light_color_);
    traffic_light.set_confidence(0.0);
    // traffic_light.set_tracking_time(0.0);
    return traffic_light;
  }
  return traffic_lights_.at(traffic_light_id);
}

perception::TrafficLight Frame::GetSignalByTurnType(
    const hdmap::Lane::LaneTurn& turn_type) const {
  std::string traffic_light_id = "straight";
  if (turn_type == hdmap::Lane::LEFT_TURN) {
    traffic_light_id = "left";
  } else if (turn_type == hdmap::Lane::U_TURN) {
    traffic_light_id = "u_turn";
  } else if (turn_type == hdmap::Lane::RIGHT_TURN) {
    traffic_light_id = "right";
  } else {
    traffic_light_id = "straight";
  }
  return GetSignal(traffic_light_id);
}

void Frame::ReadPadMsgDrivingAction() {
  if (local_view_->HasPadMessage()) {
    if (local_view_->GetPadMessage()->has_action()) {
      pad_msg_driving_action_ = local_view_->GetPadMessage()->action();
    }
  }
}

void Frame::ResetPadMsgDrivingAction() {
  pad_msg_driving_action_ = DrivingAction::NONE;
}

void Frame::SetMachineStateType(
    const functionmanager::MachineStateType& state_type) {
  cur_state_machine_ = state_type;
}

void Frame::SetIsStateChange(const bool is_state_change) {
  is_state_change_ = is_state_change;
}

const ReferenceLineInfo* Frame::FindDriveReferenceLineInfo() {
  double min_cost = std::numeric_limits<double>::infinity();
  drive_reference_line_info_ = nullptr;
  if (reference_line_info_.empty()) {
    return drive_reference_line_info_;
  }

  for (auto& reference_line_info : reference_line_info_) {
    reference_line_info.AddCost(
        reference_line_info.reference_line().GetPriority());
    ADEBUG << "Final Decision reference line is_drivable = "
           << reference_line_info.IsDrivable()
           << " cost = " << reference_line_info.Cost() << " lanes include ";
    for (const auto& id : reference_line_info.TargetLaneId()) {
      ADEBUG << id.DebugString();
    }
    if (reference_line_info.IsDrivable() &&
        Compare(reference_line_info.Cost(), min_cost) < 0) {
      drive_reference_line_info_ = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  if (drive_reference_line_info_ == nullptr) {
    AERROR << "there is no drive reference line ";
    return drive_reference_line_info_;
  }
  ADEBUG << "best referencelineinfo ";
  for (const auto& id : drive_reference_line_info_->TargetLaneId()) {
    ADEBUG << id.DebugString();
  }

  return drive_reference_line_info_;
}

const ReferenceLineInfo* Frame::FindTargetReferenceLineInfo() {
  const ReferenceLineInfo* target_reference_line_info = nullptr;
  for (const auto& reference_line_info : reference_line_info_) {
    if (reference_line_info.IsChangeLanePath()) {
      return &reference_line_info;
    }
    target_reference_line_info = &reference_line_info;
  }
  return target_reference_line_info;
}

const ReferenceLineInfo* Frame::FindFailedReferenceLineInfo() {
  for (const auto& reference_line_info : reference_line_info_) {
    // Find the unsuccessful lane-change path
    if (!reference_line_info.IsDrivable() &&
        reference_line_info.IsChangeLanePath()) {
      return &reference_line_info;
    }
  }
  return nullptr;
}

const ReferenceLineInfo* Frame::DriveReferenceLineInfo() const {
  return drive_reference_line_info_;
}

std::vector<const std::shared_ptr<Obstacle>*> Frame::obstacles() const {
  return obstacles_.Items();
}

void Frame::SetTargetGear(const soc::Chassis::GearPosition& gear) {
  target_gear_ = gear;
}

const soc::Chassis::GearPosition& Frame::GetTargetGear() const {
  return target_gear_;
}

}  // namespace planning
}  // namespace TL
