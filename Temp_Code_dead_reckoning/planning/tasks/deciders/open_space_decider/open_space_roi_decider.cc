/******************************************************************************
 * Copyright 2019 The TL Authors. All Rights Reserved.
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

#include "planning/tasks/deciders/open_space_decider/open_space_roi_decider.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/configs/config_gflags.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/math/box2d.h"
#include "common/math/double_type.h"
#include "common/math/line_segment2d.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "planning/common/open_space_info.h"

#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/tasks/deciders/open_space_decider/open_space_fine_tuning.h"
#include "planning/tasks/deciders/open_space_decider/open_space_obstacle.h"
#include "planning/tasks/task.h"
#include "planning/proto/speed_evaluator_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/perception_parking_lot.pb.h"
#include "proto/planning/planning_internal.pb.h"
#include "proto/planning/planning_status.pb.h"

namespace TL {
namespace planning {

using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::math::Vec2d;
using TL::planning::AVPStatus;

namespace {
constexpr double kEpsilon = 1.0e-3;
constexpr double kDeadScenarioLaneWidth = 5.5;
}  // namespace

OpenSpaceRoiDecider::OpenSpaceRoiDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector),
      vehicle_params_(
          TL::common::VehicleConfigHelper::GetConfig().vehicle_param()),
      injector_(injector) {
  open_space_obstacle_ = std::make_shared<OpenSpaceObstacle>(config);
  open_space_fine_tuning_ =
      std::make_shared<OpenSpaceFineTuning>(config, injector);
}

Status OpenSpaceRoiDecider::Reset() {
  // nothing to reset
  init_adc_point_.Clear();
  road_width_ = FLAGS_open_space_lane_width;
  is_entered_lateral_slot_domain_ = false;
  has_valid_history_path_ = false;
  last_slack_dist_vec_.clear();
  sensor_config_state_ = SensorConfigState::NONINITAILZED;
  park_lot_map_.clear();
  open_space_fine_tuning_->Reset();
  AINFO << "roi decider is reseted";
  return Status::OK();
}

void OpenSpaceRoiDecider::CheckReceiveParkinglot(
    const perception::ParkingLotOut::ParkType& park_type,
    const ParkingLotVertexType& parking_spot_enu,
    ParkLotStatus* const park_lot_status_ptr) {
  if (nullptr == park_lot_status_ptr) {
    return;
  }
  if (!common::math::Polygon2d::IsConvexPolygon(
          {parking_spot_enu.at(0), parking_spot_enu.at(1),
           parking_spot_enu.at(2), parking_spot_enu.at(3)})) {
    AERROR << " Perception lot is not convex!";
    *park_lot_status_ptr = NONCONVEX;
    return;
  }
  int spot_vertice_num = static_cast<int>(parking_spot_enu.size());
  for (int i = 0; i < spot_vertice_num; ++i) {
    int prev_idx = (i - 1 + spot_vertice_num) % spot_vertice_num;
    int next_idx = (i + 1) % spot_vertice_num;
    if (std::fabs(common::math::CrossProd(
            parking_spot_enu.at(i), parking_spot_enu.at(prev_idx),
            parking_spot_enu.at(next_idx))) < kEpsilon) {
      AERROR << " Perception lot vertices are collinear!";
      *park_lot_status_ptr = SMALL;
      return;
    }
  }
  const auto& left_top = parking_spot_enu.at(0);
  const auto& left_down = parking_spot_enu.at(1);
  const auto& right_down = parking_spot_enu.at(2);
  const auto& right_top = parking_spot_enu.at(3);
  double min_width = vehicle_params_.width();
  double top_to_down_dis =
      0.5 * (left_top.DistanceTo(left_down) + right_top.DistanceTo(right_down));
  double left_to_right_dis =
      0.5 * (left_top.DistanceTo(right_top) + left_down.DistanceTo(right_down));
  if (park_type == TL::perception::ParkingLotOut_ParkType_VERTICAL ||
      park_type == TL::perception::ParkingLotOut_ParkType_OBLIQUE) {
    // It is hard to detect slot's bottom edge
    // use "front_edge_to_center" to avoid slot checking failures
    min_width += config_.open_space_roi_decider_config()
                     .vertical_min_parklot_lateral_buffer();
    if (top_to_down_dis < vehicle_params_.length() +
                              config_.open_space_roi_decider_config()
                                  .vertical_slot_longitudinal_buffer() ||
        min_width - left_to_right_dis > kEpsilon) {
      AERROR << " Perception lot is too small for vertical lot!";
      *park_lot_status_ptr = SMALL;
    }
  } else if (park_type == TL::perception::ParkingLotOut_ParkType_LATERAL) {
    min_width += config_.open_space_roi_decider_config()
                     .lateral_min_parklot_lateral_buffer();
    if (top_to_down_dis < min_width ||
        left_to_right_dis < vehicle_params_.length() +
                                config_.open_space_roi_decider_config()
                                    .lateral_slot_longitudinal_buffer()) {
      AERROR << " Perception lot is too small for lateral lot!";
      *park_lot_status_ptr = SMALL;
    }
  }
}

Status OpenSpaceRoiDecider::Process(Frame* frame) {
  CHECK_NOTNULL(injector_);
  if (frame == nullptr) {
    const std::string msg =
        "Invalid frame, fail to process the OpenSpaceRoiDecider.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, msg);
  }
  const auto& previous_frame = injector_->frame_history()->Latest();
  if (frame->local_view().HasFunctionManagerIn() &&
      frame->local_view()
              .GetFunctionManagerIn()
              ->fct_avp_in()
              .sys_run_state() == functionmanager::AvpFctIn::PARKING &&
      previous_frame != nullptr &&
      previous_frame->local_view().HasFunctionManagerIn() &&
      previous_frame->local_view()
              .GetFunctionManagerIn()
              ->fct_avp_in()
              .sys_run_state() == functionmanager::AvpFctIn::PARKSTART) {
    Reset();
  }
  // init member parameters
  vehicle_state_ = frame->vehicle_state();
  parking_type_ = injector_->planning_context()
                      ->planning_status()
                      .avp_status()
                      .parking_type();
  const auto free_space_array_ptr =
      frame->local_view().HasFreeSpaceOutArray()
          ? frame->local_view().GetFreeSpaceOutArray()
          : std::make_shared<const perception::FreeSpaceOutArray>();
  common::PathPoint veh_point;
  veh_point.set_x(vehicle_state_.x());
  veh_point.set_y(vehicle_state_.y());
  veh_point.set_theta(vehicle_state_.heading());
  if (!init_adc_point_.has_x() || !init_adc_point_.has_y() ||
      !init_adc_point_.has_theta() ||
      (previous_frame != nullptr &&
       previous_frame->open_space_info().partitioned_paths().path_type ==
           planning_internal::PathUpdateStatus::TRACE_PATH)) {
    init_adc_point_.CopyFrom(veh_point);
  }
  if (previous_frame != nullptr && !has_valid_history_path_) {
    has_valid_history_path_ =
        previous_frame->open_space_info().is_partitioned_paths_valid();
  }
  SensorStateDecider();

  UpdateReplanInfo(frame);

  UpdateSpeedBumpInfo(frame);

  // load park slot
  GetParkingSpots(frame, &park_lot_map_);

  // set info for spd
  if (previous_frame != nullptr) {
    // init wheel mask info
    frame->mutable_open_space_info()->set_is_consider_wheel_mask(
        previous_frame->open_space_info().is_consider_wheel_mask());
    frame->mutable_open_space_info()->set_open_space_wheel_mask_box(
        previous_frame->open_space_info().open_space_wheel_mask_box());
  }
  const auto& target_id = frame_->open_space_info().open_space_path_info_id();
  if (park_lot_map_.find(target_id) != park_lot_map_.end()) {
    const auto& park_lot_info = park_lot_map_[target_id];
    open_space_obstacle_->UpdateOpenSpaceInfoForSpd(
        parking_type_, park_lot_info.park_type, park_lot_info.vertices,
        free_space_array_ptr, init_adc_point_,
        frame->mutable_open_space_info());
    // update wheel mask info
    UpdateWheelMaskToOpenSpace(frame, park_lot_info);
  }

  auto status = open_space_obstacle_->Init(free_space_array_ptr,
                                           frame->GetObstacleList(), veh_point);
  if (status != Status::OK()) {
    AERROR << status.error_message();
    return status;
  }

  status = InputValidCheck(park_lot_map_);
  if (status != Status::OK()) {
    AERROR << status.error_message();
    return status;
  }

  status = SetOpenSpacePathInfo(
      frame, park_lot_map_,
      frame->mutable_open_space_info()->mutable_open_space_path_info_map());
  if (status != Status::OK()) {
    AERROR << status.error_message();
    return status;
  }

  status = UpdateTargetPathInfo(
      frame, park_lot_map_,
      frame->mutable_open_space_info()->mutbale_open_space_path_info());
  if (status != Status::OK()) {
    AERROR << status.error_message();
    return status;
  }
  const auto* open_space_obs =
      &(frame->open_space_info().open_space_path_info().obstacles_segments_vec);
  status = frame->OpenSpaceCollisionCheck(open_space_obs);
  if (status != Status::OK()) {
    AERROR << status.error_message();
    return status;
  }

  RecordDebugInfo(frame);
  return Status::OK();
}

void OpenSpaceRoiDecider::RecordDebugInfo(Frame* const frame) {
  if (!FLAGS_enable_record_debug) {
    return;
  }
  auto* ptr_debug = frame->mutable_open_space_info()
                        ->mutable_debug()
                        ->mutable_planning_data()
                        ->mutable_open_space();
  ptr_debug->set_target_lon_adjust_dis(
      open_space_fine_tuning_->lon_fine_tune_dis());
  ptr_debug->set_target_lat_adjust_dis(
      open_space_fine_tuning_->lat_fine_tune_dis());
  ptr_debug->set_target_yaw_adjust_rad(
      open_space_fine_tuning_->yaw_fine_tune_rad());
  ptr_debug->set_end_pose_lon_error(end_pose_lon_error_);
  ptr_debug->set_end_pose_lat_error(end_pose_lat_error_);
  ptr_debug->set_end_pose_yaw_error(end_pose_yaw_error_);
  ptr_debug->clear_dest_polygon_point();
  for (const auto& point : std::get<0>(frame->open_space_info()
                                           .open_space_path_info()
                                           .dest_region_with_angle)
                               .points()) {
    auto* debug_polygon_point = ptr_debug->add_dest_polygon_point();
    debug_polygon_point->set_x(point.x());
    debug_polygon_point->set_y(point.y());
  }
  double x_diff = vehicle_state_.x() - init_adc_point_.x();
  double y_diff = vehicle_state_.y() - init_adc_point_.y();
  auto dis_receive = std::sqrt(x_diff * x_diff + y_diff * y_diff);
  ptr_debug->set_receive_location_distance(dis_receive);
}

void OpenSpaceRoiDecider::UpdateWheelMaskToOpenSpace(
    Frame* const frame, const ParkLotInfo& park_lot_info) {
  if (nullptr == frame) {
    AERROR << "frame is nullptr";
    return;
  }
  const auto& target_park_lot_wheel_mask = park_lot_info.wheel_mask;
  if (std::get<0>(target_park_lot_wheel_mask)) {
    frame->mutable_open_space_info()->set_is_consider_wheel_mask(true);
    auto axis =
        common::math::LineSegment2d(std::get<1>(target_park_lot_wheel_mask),
                                    std::get<2>(target_park_lot_wheel_mask));
    static constexpr double kEpsilon = 0.05;
    if (axis.length() < kEpsilon) {
      // wheel mask is too short to use
      // construct axis based wheel mask and park lot vertices
      if (park_lot_info.park_type == perception::ParkingLotOut::LATERAL) {
        axis = common::math::LineSegment2d(park_lot_info.vertices.at(0),
                                           park_lot_info.vertices.at(1));
        axis.Translate(axis.DistanceTo(std::get<1>(target_park_lot_wheel_mask)),
                       axis.heading() + M_PI_2);
      } else {
        axis = common::math::LineSegment2d(park_lot_info.vertices.at(3),
                                           park_lot_info.vertices.at(0));
        const auto ld2lt =
            park_lot_info.vertices.at(0) - park_lot_info.vertices.at(1);
        const auto lt2rt =
            park_lot_info.vertices.at(3) - park_lot_info.vertices.at(0);
        const double project_ratio = fabs(sin(ld2lt.Angle() - lt2rt.Angle()));
        if (project_ratio <= kEpsilon) {
          AERROR << "park slot is not legal";
          return;
        }
        axis.Translate(
            axis.DistanceTo(std::get<1>(target_park_lot_wheel_mask)) /
                project_ratio,
            ld2lt.Angle() + M_PI);
      }
    }
    const auto wheel_mask_box = common::math::Box2d(
        axis, config_.open_space_roi_decider_config().wheel_mask_box_width());
    frame->mutable_open_space_info()->set_open_space_wheel_mask_box(
        wheel_mask_box);
  }
}

Status OpenSpaceRoiDecider::SetOpenSpacePathInfo(
    Frame* const frame,
    const std::unordered_map<uint32_t, ParkLotInfo>& park_lot_map,
    OpenSpacePathInfoMap* const open_space_path_info_map_ptr) {
  if (nullptr == frame) {
    const std::string msg = "frame is null";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, msg);
  }
  if (nullptr == open_space_path_info_map_ptr) {
    const std::string msg = "path info map is null";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, msg);
  }

  auto status = Status::OK();
  switch (parking_type_) {
    case TL::planning::AVPStatus_ParkingType_PARKING_IN:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_LEFT:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_RIGHT:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_FRONT:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_BACK:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_NNS: {
      status = SetOpenSpacePathInfoBasedOnParklot(frame, park_lot_map,
                                                  open_space_path_info_map_ptr);
      break;
    }
    case TL::planning::AVPStatus_ParkingType_TEST_CONTROL_MODE: {
      status =
          SetOpenSpacePathInfoForTestMode(frame, open_space_path_info_map_ptr);
      break;
    }
    case TL::planning::AVPStatus_ParkingType_NNS_ADJUST: {
      status = SetOpenSpacePathInfoForNNSAdjust(frame);
      break;
    }
    default: {
      // TODO(sim): NTP_ADJUST
      const std::string msg = "undefined parking type";
      AERROR << msg;
      status = Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, msg);
      break;
    }
  }
  return status;
}

Status OpenSpaceRoiDecider::UpdateTargetPathInfo(
    Frame* const frame,
    const std::unordered_map<uint32_t, ParkLotInfo>& park_lot_map,
    OpenSpacePathInfo* const open_space_path_info_ptr) {
  if (nullptr == frame || nullptr == open_space_path_info_ptr) {
    const std::string msg = "frame or open_space_path_info_ptr is nullptr";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, msg);
  }
  auto status = open_space_path_info_ptr->status;
  if (status != Status::OK()) {
    AERROR << status.error_message();
    return status;
  }
  uint32_t replan = 0;
  switch (parking_type_) {
    case TL::planning::AVPStatus_ParkingType_PARKING_IN:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_LEFT:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_RIGHT:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_FRONT:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_BACK:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_NNS: {
      const auto& target_id =
          frame->open_space_info().open_space_path_info_id();
      if (park_lot_map.find(target_id) == park_lot_map.end()) {
        const std::string msg = "no target path id in map";
        AERROR << msg;
        status = Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, msg);
      }
      const auto& park_lot_info = park_lot_map.at(target_id);
      const auto pre_end_pose_enu = GetPreEndPose();
      auto& origin_target_pose = open_space_path_info_ptr->end_point;
      const bool is_pre_end_pose_valid = pre_end_pose_enu.has_x() &&
                                         pre_end_pose_enu.has_y() &&
                                         pre_end_pose_enu.has_theta();
      bool is_preplan =
          frame->local_view().HasFunctionManagerIn() &&
          frame->local_view()
                  .GetFunctionManagerIn()
                  ->fct_avp_in()
                  .sys_run_state() == functionmanager::AvpFctIn::PARKSTART;
      const bool use_pre_end_pose =
          !is_preplan && is_pre_end_pose_valid &&
          !park_lot_info.is_parking_lot_update &&
          !open_space_fine_tuning_->is_fine_tune_update() &&
          injector_->planning_context()
                  ->planning_status()
                  .open_space()
                  .replan() == 0;
      if (use_pre_end_pose) {
        origin_target_pose = pre_end_pose_enu;
      }
      replan = GetParkingReplanStatus(park_lot_info.park_type,
                                      park_lot_info.vertices, pre_end_pose_enu,
                                      origin_target_pose);
      CaculateDestRegion(frame, origin_target_pose, park_lot_info.park_type,
                         park_lot_info.vertices,
                         &(open_space_path_info_ptr->dest_region_with_angle));

      break;
    }
    case TL::planning::AVPStatus_ParkingType_TEST_CONTROL_MODE:
    case TL::planning::AVPStatus_ParkingType_NNS_ADJUST: {
      replan = GetNNSAdjustReplanStatus();
      break;
    }
    default: {
      // TODO(sim): NNS_ADJUST
      const std::string msg = "undefined parking type";
      AERROR << msg;
      status = Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, msg);
      // break;
    }
  }
  for (int i = OpenSpaceStatus::Replan_MIN; i <= OpenSpaceStatus::Replan_MAX;
       ++i) {
    if (OpenSpaceStatus::Replan_IsValid(i) &&
        (replan & static_cast<uint32_t>(i)) != 0U) {
      OpenSpaceInfo::UpdateReplanStatus(static_cast<OpenSpaceStatus::Replan>(i),
                                        injector_->planning_context()
                                            ->mutable_planning_status()
                                            ->mutable_open_space());
    }
  }
  return status;
}

common::Status OpenSpaceRoiDecider::SetOpenSpacePathInfoBasedOnParklot(
    Frame* const frame,
    const std::unordered_map<uint32_t, ParkLotInfo>& park_lot_map,
    OpenSpacePathInfoMap* const open_space_path_info_map_ptr) {
  if (nullptr == open_space_path_info_map_ptr) {
    const std::string msg = "path info map is null";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, msg);
  }
  const auto& previous_frame = injector_->frame_history()->Latest();
  planning_internal::AvpSpeedPlanCollisionInfo spd_collision_info;
  bool replan_triggered_by_speed_plan = false;
  if (previous_frame != nullptr &&
      (previous_frame->open_space_info().replan_triggered_by_speed_plan() ||
       previous_frame->open_space_info().current_path_has_collision_risk())) {
    spd_collision_info =
        previous_frame->open_space_info().speed_plan_collision_info();
    replan_triggered_by_speed_plan =
        previous_frame->open_space_info().replan_triggered_by_speed_plan();
  }
  // send top K parking lot to update path info
  auto top_k_parking_lots = TopKParkingLots(frame, park_lot_map);
  const auto& free_space_array_ptr =
      frame->local_view().HasFreeSpaceOutArray()
          ? frame->local_view().GetFreeSpaceOutArray()
          : nullptr;
  common::PathPoint veh_point;
  veh_point.set_x(vehicle_state_.x());
  veh_point.set_y(vehicle_state_.y());
  veh_point.set_theta(vehicle_state_.heading());
  OpenSpacePathInfo open_space_path_info;
  AINFO << "top_k_parking_lots size: " << top_k_parking_lots.size();
  for (const uint32_t idx : top_k_parking_lots) {
    ADEBUG << "idx " << idx;
    if (open_space_path_info_map_ptr->find(idx) ==  // NOLINT
        open_space_path_info_map_ptr->end()) {
      open_space_path_info_map_ptr->emplace(idx, open_space_path_info);
    }
    auto& open_space_path_info_tmp =
        open_space_path_info_map_ptr->at(idx);  // NOLINT
    SetOrigin(park_lot_map.at(idx).vertices, &open_space_path_info_tmp.origin,
              &open_space_path_info_tmp.rotate_angle);
    auto* const is_parking_inwards =
        &open_space_path_info_tmp.open_space_env_structured_info
             .is_parking_inwards;
    const bool init_end_pose_suc = InitEndPoseBaseSlot(
        frame, park_lot_map.at(idx).park_type, park_lot_map.at(idx).vertices,
        park_lot_map.at(idx).is_right_side, park_lot_map.at(idx).wheel_mask,
        &open_space_path_info_tmp.end_point, is_parking_inwards);
    if (!init_end_pose_suc) {
      open_space_path_info_tmp.status = Status(
          ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, "init end pose failed");
      continue;
    }
    const auto scenario_diffculty_type =
        open_space_obstacle_->ScenarioDiffcultyDecison(
            parking_type_, park_lot_map.at(idx), free_space_array_ptr);
    uint32_t pre_scenario_diffculty_type = NORMAL_SCENARIO;
    if (previous_frame != nullptr &&
        previous_frame->open_space_info().open_space_path_info_map().find(
            idx) != previous_frame  // NOLINT
                        ->open_space_info()
                        .open_space_path_info_map()
                        .end()) {
      pre_scenario_diffculty_type =
          previous_frame->open_space_info()
              .open_space_path_info_map()
              .at(idx)  // NOLINT
              .open_space_env_structured_info.parking_scenario_diffculty_type;
    }
    open_space_path_info_tmp.open_space_env_structured_info
        .parking_scenario_diffculty_type =
        scenario_diffculty_type | pre_scenario_diffculty_type;
    if ((open_space_path_info_tmp.open_space_env_structured_info
             .parking_scenario_diffculty_type &
         DEADEND_SCENARIO) != 0) {
      road_width_ = kDeadScenarioLaneWidth;
    }
    std::vector<common::math::LineSegment2d> inner_roi_boundary;
    std::vector<common::math::LineSegment2d> outer_roi_boundary;
    GetParkingBoundary(
        frame, open_space_path_info_tmp.origin,
        open_space_path_info_tmp.rotate_angle, park_lot_map.at(idx).park_type,
        park_lot_map.at(idx).vertices, open_space_path_info_tmp.end_point,
        &inner_roi_boundary, &outer_roi_boundary,
        &open_space_path_info_tmp.roi_xy_boundary);
    const bool is_adc_in_roi = IsVehicleInRoi(
        open_space_path_info_tmp.origin, open_space_path_info_tmp.rotate_angle,
        open_space_path_info_tmp.roi_xy_boundary);
    const auto is_trace_path_valid =
        AVPStatus::PARKING_IN == parking_type_ &&
        frame->local_view().HasParkingLotOutArray() &&
        IsValidTracePath(
            frame->local_view().GetParkingLotOutArray()->traced_path(),
            park_lot_map.at(idx).vertices,
            &open_space_path_info_tmp.trace_path);
    if (!is_adc_in_roi && !is_trace_path_valid) {
      open_space_path_info_tmp.status = Status(
          ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, "adc is out of roi");
      continue;
    }
    ObsFilterMap obs_filter_map;
    const bool is_slot_inner_fs_valid =
        IsSlotInnerFsValid(*is_parking_inwards, park_lot_map.at(idx).park_type,
                           park_lot_map.at(idx).vertices,
                           park_lot_map.at(idx).is_high_quality_triggered);
    const bool is_narrow_spot_scenario =
        (open_space_path_info.open_space_env_structured_info
             .parking_scenario_diffculty_type &
         NARROW_SPOT_SCENARIO) != 0;
    SetObsFilterStrategy(frame, park_lot_map.at(idx), outer_roi_boundary,
                         is_slot_inner_fs_valid, *is_parking_inwards,
                         is_narrow_spot_scenario, &obs_filter_map);
    bool is_lateral_park_out =
        parking_type_ != TL::planning::AVPStatus::PARKING_IN &&
        park_lot_map.at(idx).park_type == perception::ParkingLotOut::LATERAL;
    std::vector<std::pair<common::math::LineSegment2d, double>>
        linked_obstacles_segments_vec;
    std::vector<std::pair<common::math::LineSegment2d, double>>
        high_curb_obstacles_segments_vec;
    open_space_path_info_tmp.status = open_space_obstacle_->LoadObs(
        park_lot_map.at(idx).vertices, obs_filter_map, spd_collision_info,
        is_lateral_park_out, replan_triggered_by_speed_plan, false,
        frame->GetObstacleList(),
        &open_space_path_info_tmp.obstacles_segments_vec,
        &linked_obstacles_segments_vec, &high_curb_obstacles_segments_vec,
        &open_space_path_info_tmp.low_fs_obstacles_segments_vec);
    if (open_space_path_info_tmp.status != Status::OK()) {
      continue;
    }
    common::math::LineSegment2d reference_curb;
    if ((parking_type_ == planning::AVPStatus::PARKING_OUT_LEFT ||
         parking_type_ == planning::AVPStatus::PARKING_OUT_RIGHT) &&
        !open_space_obstacle_->GetParkOutCurbSeg(
            parking_type_, park_lot_map.at(idx).park_type,
            park_lot_map.at(idx).vertices, free_space_array_ptr,
            &reference_curb)) {
      ADEBUG << "get park out curb heading failed ";
    }
    open_space_fine_tuning_->Process(
        is_entered_lateral_slot_domain_,
        frame_->open_space_info().is_consider_wheel_mask(), parking_type_,
        park_lot_map.at(idx), open_space_path_info_tmp,
        linked_obstacles_segments_vec, high_curb_obstacles_segments_vec,
        veh_point, reference_curb, &open_space_path_info_tmp.end_point);
    // TODO(jyw): move out this
    open_space_obstacle_->AddVirtualObs(
        parking_type_, park_lot_map.at(idx), open_space_path_info_tmp,
        free_space_array_ptr, frame->GetObstacleList(), inner_roi_boundary,
        init_adc_point_, veh_point,
        frame->open_space_info().is_consider_wheel_mask(),
        frame->open_space_info().open_space_wheel_mask_box(),
        spd_collision_info, &open_space_path_info_tmp.obstacles_segments_vec);
    CaculateDestRegion(frame, open_space_path_info_tmp.end_point,
                       park_lot_map.at(idx).park_type,
                       park_lot_map.at(idx).vertices,
                       &open_space_path_info_tmp.dest_region_with_angle);
    ParkingScenarioTypeDecision(
        park_lot_map.at(idx).is_right_side, park_lot_map.at(idx).park_type,
        &open_space_path_info_tmp.open_space_env_structured_info
             .parking_scenario_type);
  }
  return Status::OK();
}

std::vector<uint32_t> OpenSpaceRoiDecider::TopKParkingLots(  // NOLINT
    Frame* const frame,
    const std::unordered_map<uint32_t, ParkLotInfo>& park_lot_map) {
  const auto& optimal_id = frame->open_space_info().open_space_path_info_id();
  std::vector<uint32_t> top_k_parking_lots;
  if (park_lot_map.find(optimal_id) == park_lot_map.end()) {
    return top_k_parking_lots;
  }
  if (frame->local_view().HasFunctionManagerIn() &&
      frame->local_view()
              .GetFunctionManagerIn()
              ->fct_avp_in()
              .sys_run_state() == functionmanager::AvpFctIn::PARKING) {
    if (park_lot_map.at(optimal_id).status == ParkLotStatus::NORMAL) {
      top_k_parking_lots.emplace_back(optimal_id);
    }
    return top_k_parking_lots;
  }
  // get all free park lot
  for (const auto& park_lot_pair : park_lot_map) {
    if (park_lot_pair.second.status == ParkLotStatus::NORMAL ||
        park_lot_pair.first == optimal_id) {
      top_k_parking_lots.emplace_back(park_lot_pair.first);
    }
  }
  // sort park lot
  const auto& optimal_park_lot = park_lot_map.at(optimal_id);
  Vec2d optimal_top_edge_center =
      0.5 * (optimal_park_lot.vertices[0] + optimal_park_lot.vertices[3]);
  auto dis_to_optimal = [&](uint32_t idx) {
    return optimal_top_edge_center.DistanceTo(
        0.5 *
        (park_lot_map.at(idx).vertices[0] + park_lot_map.at(idx).vertices[3]));
  };
  std::sort(top_k_parking_lots.begin(), top_k_parking_lots.end(),
            [&](uint32_t idx1, uint32_t idx2) {
              return dis_to_optimal(idx1) < dis_to_optimal(idx2);
            });
  static constexpr size_t kNum = 3;
  return std::vector<uint32_t>(  // NOLINT
      top_k_parking_lots.begin(),
      top_k_parking_lots.begin() +
          std::min(top_k_parking_lots.size(), kNum));  // NOLINT
}

Status OpenSpaceRoiDecider::SetOpenSpacePathInfoForTestMode(
    Frame* const frame,
    OpenSpacePathInfoMap* const open_space_path_info_map_ptr) {
  if (nullptr == open_space_path_info_map_ptr) {
    const std::string msg = "path info map is null";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, msg);
  }
  const uint32_t idx = frame->open_space_info().open_space_path_info_id();
  if (open_space_path_info_map_ptr->find(idx) ==  // NOLINT
      open_space_path_info_map_ptr->end()) {
    OpenSpacePathInfo open_space_path_info;
    open_space_path_info_map_ptr->emplace(idx, open_space_path_info);
  }
  auto& open_space_path_info_tmp =
      open_space_path_info_map_ptr->at(idx);  // NOLINT

  if (!init_adc_point_.has_x() || !init_adc_point_.has_y() ||
      !init_adc_point_.has_theta()) {
    const std::string msg = " init pose is not initalized";
    AERROR << msg;
    open_space_path_info_tmp.status =
        Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, msg);
    return open_space_path_info_tmp.status;
  }
  open_space_path_info_tmp.origin.set_x(0.0);
  open_space_path_info_tmp.origin.set_y(0.0);
  open_space_path_info_tmp.rotate_angle = 0.0;
  const double end_pose_x_rfu = FLAGS_ctl_calibr_x;
  const double end_pose_y_rfu = FLAGS_ctl_calibr_y;
  const double end_pose_theta_rfu = FLAGS_ctl_calibr_theta;
  auto end_pose_xy = common::math::RFUToENU(
      end_pose_x_rfu, end_pose_y_rfu, init_adc_point_.x(), init_adc_point_.y(),
      init_adc_point_.theta());
  open_space_path_info_tmp.end_point.set_x(end_pose_xy.first);
  open_space_path_info_tmp.end_point.set_y(end_pose_xy.second);
  open_space_path_info_tmp.end_point.set_theta(common::math::NormalizeAngle(
      init_adc_point_.theta() + end_pose_theta_rfu));

  GetStaticBoundary(open_space_path_info_tmp.origin,
                    open_space_path_info_tmp.rotate_angle,
                    &open_space_path_info_tmp.roi_xy_boundary);
  if (!IsVehicleInRoi(open_space_path_info_tmp.origin,
                      open_space_path_info_tmp.rotate_angle,
                      open_space_path_info_tmp.roi_xy_boundary)) {
    const std::string msg = "adc is out of roi";
    AERROR << msg;
    open_space_path_info_tmp.status =
        Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, msg);
    return open_space_path_info_tmp.status;
  }
  return Status::OK();
}

Status OpenSpaceRoiDecider::SetOpenSpacePathInfoForNNSAdjust(
    Frame* const frame) {
  auto* open_space_path_info_ptr =
      frame->mutable_open_space_info()->mutbale_open_space_path_info();
  // set ParkingScenarioType(the open space info here is the optimial info, normally is the only one)
  open_space_path_info_ptr->open_space_env_structured_info
      .parking_scenario_type =
      ParkingScenarioType::FREESPACE_FORWARD_EXPLORATION;
  open_space_path_info_ptr->open_space_env_structured_info
      .is_in_nns_adjust_scenario = true;
  open_space_path_info_ptr->origin.set_x(
      open_space_path_info_ptr->end_point.x());
  open_space_path_info_ptr->origin.set_y(
      open_space_path_info_ptr->end_point.y());
  open_space_path_info_ptr->rotate_angle =
      (open_space_path_info_ptr->end_point.theta());
  std::vector<common::math::Vec2d> inner_roi_vertex;
  std::vector<common::math::Vec2d> outer_roi_vertex;
  std::vector<common::math::LineSegment2d> inner_roi_boundary;
  std::vector<common::math::LineSegment2d> outer_roi_boundary;
  GetDynamicBoundary(frame->open_space_info().stuck_scenario_roi(),
                     &inner_roi_vertex);
  CalculateOuterRoi(inner_roi_vertex, &outer_roi_vertex);
  TransRoiAndSetBoundary(
      open_space_path_info_ptr->origin, open_space_path_info_ptr->rotate_angle,
      &inner_roi_vertex, &outer_roi_vertex, &inner_roi_boundary,
      &outer_roi_boundary, &(open_space_path_info_ptr->roi_xy_boundary));
  // check if vehicle is in ROI
  if (!IsVehicleInRoi(open_space_path_info_ptr->origin,
                      open_space_path_info_ptr->rotate_angle,
                      open_space_path_info_ptr->roi_xy_boundary)) {
    const std::string msg = "adc is out of roi";
    AINFO << msg;
    open_space_path_info_ptr->status =
        Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, msg);
  }

  ParkLotInfo park_lot_info;
  common::PathPoint veh_point;
  veh_point.set_x(vehicle_state_.x());
  veh_point.set_y(vehicle_state_.y());
  veh_point.set_theta(vehicle_state_.heading());
  ObsFilterMap obs_filter_map;
  // common obs
  ObsFilter common_filter;
  common_filter.use_obstacle =
      config_.open_space_roi_decider_config().enable_perception_obstacles();
  if (outer_roi_vertex.size() > 2) {
    common_filter.filter_areas.emplace_back(outer_roi_vertex, false);
    obs_filter_map.emplace(COMMON_OBS, common_filter);
  }
  // always update obstalce
  std::vector<std::pair<common::math::LineSegment2d, double>>
      linked_obstacles_segments_vec;
  std::vector<std::pair<common::math::LineSegment2d, double>>
      high_curb_obstacles_segments_vec;
  const auto& previous_frame = injector_->frame_history()->Latest();
  planning_internal::AvpSpeedPlanCollisionInfo spd_collision_info;
  if (previous_frame != nullptr) {
    spd_collision_info =
        previous_frame->open_space_info().speed_plan_collision_info();
  }
  auto status = open_space_obstacle_->LoadObs(
      park_lot_info.vertices, obs_filter_map, spd_collision_info, false, false,
      true, frame->GetObstacleList(),
      &(open_space_path_info_ptr->obstacles_segments_vec),
      &linked_obstacles_segments_vec, &high_curb_obstacles_segments_vec,
      &(open_space_path_info_ptr->low_fs_obstacles_segments_vec));
  open_space_obstacle_->AddVirtualObs(
      parking_type_, park_lot_info, *open_space_path_info_ptr,
      frame->local_view().GetFreeSpaceOutArray(), frame->GetObstacleList(),
      inner_roi_boundary, init_adc_point_, veh_point,
      frame->open_space_info().is_consider_wheel_mask(),
      frame->open_space_info().open_space_wheel_mask_box(), spd_collision_info,
      &(open_space_path_info_ptr->obstacles_segments_vec));
  // TODO(lsy): remove cut off trace path
  CutOffTracePath(open_space_path_info_ptr->obstacles_segments_vec,
                  &open_space_path_info_ptr->trace_path);
  if (status != Status::OK()) {
    return status;
  }

  return Status::OK();
}

void OpenSpaceRoiDecider::CutOffTracePath(
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec,
    DiscretizedPath* const trace_path_ptr) {
  if (nullptr == trace_path_ptr) {
    return;
  }
  auto iter = std::find_if(
      trace_path_ptr->begin(), trace_path_ptr->end(),
      [&obstacles_segments_vec](const common::PathPoint& point) {
        return common::math::CheckCollisionWithVehiclePolygon2d(
            point.x(), point.y(), point.theta(), obstacles_segments_vec);
      });
  trace_path_ptr->erase(iter, trace_path_ptr->end());
}

void OpenSpaceRoiDecider::UpdateReplanInfo(Frame* const frame) {
  if (nullptr == frame) {
    AERROR << "frame checkcheck fails";
    return;
  }
  // replan triggered by enter lateral slot specified region
  const auto& previous_frame = injector_->frame_history()->Latest();
  if (frame->local_view().GetGuardTriggeredFlag()) {
    OpenSpaceInfo::UpdateReplanStatus(
        TL::planning::OpenSpaceStatus::BLOCK_BY_USS,
        injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_open_space());
    ADEBUG << "replan by uss, add uss collision obstacles";
  }
  if (previous_frame != nullptr &&
      previous_frame->open_space_info().replan_triggered_by_speed_plan()) {
    OpenSpaceInfo::UpdateReplanStatus(
        TL::planning::OpenSpaceStatus::BLOCK_BY_STATIC_OBSTACLE,
        injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_open_space());
  }
  if (nullptr == previous_frame ||
      previous_frame->open_space_info().partitioned_paths().path_set.empty()) {
    OpenSpaceInfo::UpdateReplanStatus(
        TL::planning::OpenSpaceStatus::NO_VALID_PATH,
        injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_open_space());
  }
  if (previous_frame != nullptr &&
      previous_frame->open_space_info().current_path_has_collision_risk()) {
    OpenSpaceInfo::UpdateReplanStatus(
        TL::planning::OpenSpaceStatus::REPLAN_FOR_SPEED_WARN,
        injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_open_space());
  }
}

void OpenSpaceRoiDecider::UpdateSpeedBumpInfo(Frame* const frame) {
  if (nullptr == frame) {
    AERROR << "frame is nullptr";
    return;
  }
  frame_->mutable_open_space_info()->clear_speed_bump_segments();
  const auto& obstacles = frame->GetObstacleList();
  for (const auto& obs : obstacles->Items()) {
    if (obs == nullptr ||
        (*obs)->Perception().sub_type() !=
            perception::PerceptionObstacle::ST_SPEEDBUMP ||
        (*obs)->PerceptionBoundingBox().length() < kEpsilon) {
      continue;
    }
    const auto& center_point = (*obs)->PerceptionBoundingBox().center();
    const auto& speed_bump_half_length =
        (*obs)->PerceptionBoundingBox().half_length();
    const common::math::LineSegment2d speed_bump_seg(
        center_point + speed_bump_half_length *
                           Vec2d::CreateUnitVec2d(
                               (*obs)->PerceptionBoundingBox().heading()),
        center_point - speed_bump_half_length *
                           Vec2d::CreateUnitVec2d(
                               (*obs)->PerceptionBoundingBox().heading()));
    frame_->mutable_open_space_info()->add_speed_bump_segments(speed_bump_seg);
  }
}

void OpenSpaceRoiDecider::SetObsFilterStrategy(
    Frame* const frame, const ParkLotInfo& park_lot_info,
    const std::vector<common::math::LineSegment2d>& roi_boundary,
    const bool is_slot_inner_fs_valid, const bool is_parking_inwards,
    const bool is_narrow_spot_scenario,
    ObsFilterMap* const obs_filter_map_ptr) {
  if (frame == nullptr || obs_filter_map_ptr == nullptr) {
    AERROR << "obs_filter_strategy inputs check fails";
    return;
  }
  // uss
  ObsFilter uss_filter;
  uss_filter.use_obstacle = frame->local_view().GetGuardTriggeredFlag();
  obs_filter_map_ptr->emplace(USS_OBS, uss_filter);

  // freespace
  ObsFilter fs_filter;
  fs_filter.use_obstacle = true;
  ObsFilter low_fs_filter;
  low_fs_filter.use_obstacle = true;
  ObsFilter high_curb_fs_filter;
  high_curb_fs_filter.use_obstacle = true;
  const auto& fct_in = frame->local_view().GetFunctionManagerIn()->fct_avp_in();
  if (!park_lot_info.vertices.empty()) {
    switch (fct_in.sys_command()) {
      case functionmanager::AvpFctIn::SYSTEMON:
      case functionmanager::AvpFctIn::PARKINCONTROL: {
        if (TL::perception::ParkingLotOut_ParkType_LATERAL ==
            park_lot_info.park_type) {
          LateralParkInFilter(park_lot_info.vertices, is_slot_inner_fs_valid,
                              park_lot_info.sensor_type, &fs_filter);
        } else {
          // currently including vertical and oblique
          VerticalParkInFilter(park_lot_info.vertices, is_slot_inner_fs_valid,
                               is_parking_inwards, is_narrow_spot_scenario,
                               &fs_filter);
        }
        low_fs_filter = fs_filter;
        break;
      }
      case functionmanager::AvpFctIn::LEFTPARKOUTCONTROL:
      case functionmanager::AvpFctIn::RIGHTPARKOUTCONTROL:
      case functionmanager::AvpFctIn::FRONTPARKOUTCONTROL:
      case functionmanager::AvpFctIn::BACKPARKOUTCONTROL:
      case functionmanager::AvpFctIn::NNSCONTROL:
      case functionmanager::AvpFctIn::NTPCONTROL: {
        if (perception::ParkingLotOut::VERTICAL == park_lot_info.park_type ||
            perception::ParkingLotOut::OBLIQUE == park_lot_info.park_type) {
          VerticalParkOutFilter(&fs_filter);
          low_fs_filter = fs_filter;
          VerticalParkOutLowFsFilter(&low_fs_filter);
        }
        break;
      }
      default:
        break;
    }
  }
  obs_filter_map_ptr->emplace(FREE_SPACE_OBS, fs_filter);
  obs_filter_map_ptr->emplace(LOW_FREE_SPACE_OBS, low_fs_filter);

  // box
  ObsFilter box_filter = fs_filter;
  // ignore bbox when using hz object fusion obstacles
  box_filter.use_obstacle =
      config_.open_space_roi_decider_config().enable_consider_obstacle_box() &&
      frame->local_view().HasPerceptionObstacles() &&
      frame->local_view().GetPerceptionObstacles()->has_header() &&
      (!frame->local_view().GetPerceptionObstacles()->header().has_frame_id() ||
       frame->local_view().GetPerceptionObstacles()->header().frame_id() !=
           "object_fusion");
  obs_filter_map_ptr->emplace(BOX_OBS, box_filter);

  // wheel_mask obs
  ObsFilter wheel_mask_filter;
  wheel_mask_filter.use_obstacle = true;
  if (park_lot_info.vertices.size() > 3) {
    std::vector<Vec2d> spot_vertex = {
        park_lot_info.vertices.at(0), park_lot_info.vertices.at(1),
        park_lot_info.vertices.at(2), park_lot_info.vertices.at(3)};
    wheel_mask_filter.filter_areas.emplace_back(spot_vertex, true);
    const auto lt2rt = common::math::LineSegment2d(
        park_lot_info.vertices.at(0), park_lot_info.vertices.at(3));
    const auto lt2ld = common::math::LineSegment2d(
        park_lot_info.vertices.at(0), park_lot_info.vertices.at(1));
    auto filter_line = lt2rt;
    filter_line.Translate(0.5 * lt2ld.length(), lt2ld.heading());
    wheel_mask_filter.filter_planes.emplace_back(filter_line, false);
  }
  obs_filter_map_ptr->emplace(WHEEL_MASK_OBS, wheel_mask_filter);

  // high curb obs for fine tune
  high_curb_fs_filter = wheel_mask_filter;
  obs_filter_map_ptr->emplace(HIGH_CURB_FREE_SPACE_OBS, high_curb_fs_filter);

  // common obs
  ObsFilter common_filter;
  common_filter.use_obstacle =
      config_.open_space_roi_decider_config().enable_perception_obstacles();
  if (roi_boundary.size() > 2) {
    std::vector<Vec2d> roi_vertex;
    roi_vertex.reserve(roi_boundary.size() + 1);
    for (const auto& boundary : roi_boundary) {
      roi_vertex.emplace_back(boundary.start());
    }
    roi_vertex.emplace_back(roi_boundary.back().end());
    common_filter.filter_areas.emplace_back(roi_vertex, false);
  }
  obs_filter_map_ptr->emplace(COMMON_OBS, common_filter);
}

bool OpenSpaceRoiDecider::IsSlotInnerFsValid(
    const bool is_parking_inwards,
    const perception::ParkingLotOut::ParkType& park_lot_type,
    const ParkingLotVertexType& parking_spot_enu,
    const bool is_high_quality_triggered) {
  if (!has_valid_history_path_) {
    return false;
  }
  bool is_slot_inner_fs_valid =
      perception::ParkingLotOut::LATERAL == park_lot_type
          ? is_entered_lateral_slot_domain_
          : is_high_quality_triggered;
  const auto slot_entrance =
      0.5 * (parking_spot_enu.at(0) + parking_spot_enu.at(3));
  const auto rear_center = Vec2d(vehicle_state_.x(), vehicle_state_.y());
  const auto front_center =
      Vec2d(vehicle_state_.x(), vehicle_state_.y()) +
      vehicle_params_.wheel_base() *
          Vec2d::CreateUnitVec2d(vehicle_state_.heading());
  const double dist_adc_to_slot_entrance =
      is_parking_inwards ? (front_center - slot_entrance).Length()
                         : (rear_center - slot_entrance).Length();
  is_slot_inner_fs_valid |=
      dist_adc_to_slot_entrance <
      config_.open_space_roi_decider_config().use_slot_inner_fs_radius();
  return is_slot_inner_fs_valid;
}

void OpenSpaceRoiDecider::VerticalParkInFilter(
    const ParkingLotVertexType& parking_spot_enu,
    const bool is_slot_inner_fs_valid, const bool is_parking_inwards,
    const bool is_narrow_spot_scenario, ObsFilter* const obs_filter) {
  // filter slot nearby fs with buffer in vertical/oblique park in scenario
  const auto ld2lt = parking_spot_enu[0] - parking_spot_enu[1];
  const auto lt2rt = parking_spot_enu[3] - parking_spot_enu[0];
  const double project_ratio = fabs(sin(ld2lt.Angle() - lt2rt.Angle()));
  if (project_ratio <= kEpsilon) {
    AERROR << "park slot is not legal";
    return;
  }
  auto free_space_filter_plane =
      common::math::LineSegment2d(parking_spot_enu[1], parking_spot_enu[2]);
  obs_filter->filter_planes.emplace_back(free_space_filter_plane, false);

  double extra_filter_length = is_slot_inner_fs_valid ? 0.0 : 0.5;
  const double filter_width = (vehicle_params_.width() +
                               config_.open_space_roi_decider_config()
                                   .vertical_park_in_lateral_fs_filter_dist()) /
                              project_ratio;
  const double filter_length =
      vehicle_params_.length() +
      config_.open_space_roi_decider_config()
          .vertical_park_in_longitudinal_fs_filter_dist() +
      filter_width * fabs(cos(ld2lt.Angle() - lt2rt.Angle())) +
      extra_filter_length;
  auto left_top =
      0.5 * (parking_spot_enu[0] + parking_spot_enu[3]) +
      extra_filter_length *
          common::math::Vec2d::CreateUnitVec2d(ld2lt.Angle()) +
      0.5 * filter_width *
          common::math::Vec2d::CreateUnitVec2d(lt2rt.Angle() + M_PI);
  const auto left_down =
      left_top + filter_length *
                     common::math::Vec2d::CreateUnitVec2d(ld2lt.Angle() + M_PI);
  auto right_top =
      left_top +
      filter_width * common::math::Vec2d::CreateUnitVec2d(lt2rt.Angle());
  const auto right_down =
      left_down +
      filter_width * common::math::Vec2d::CreateUnitVec2d(lt2rt.Angle());

  if (is_slot_inner_fs_valid) {
    // ignore obs below high quality area
    const auto adc_bottom =
        Vec2d(vehicle_state_.x(), vehicle_state_.y()) +
        vehicle_params_.back_edge_to_center() *
            Vec2d::CreateUnitVec2d(vehicle_state_.heading() + M_PI);
    const auto adc_top = Vec2d(vehicle_state_.x(), vehicle_state_.y()) +
                         vehicle_params_.front_edge_to_center() *
                             Vec2d::CreateUnitVec2d(vehicle_state_.heading());
    const auto adc_bottom_seg = common::math::LineSegment2d(
        adc_bottom + filter_width * Vec2d::CreateUnitVec2d(
                                        vehicle_state_.heading() + M_PI_2),
        adc_bottom + filter_width * Vec2d::CreateUnitVec2d(
                                        vehicle_state_.heading() - M_PI_2));
    const auto adc_top_seg = common::math::LineSegment2d(
        adc_top + filter_width *
                      Vec2d::CreateUnitVec2d(vehicle_state_.heading() + M_PI_2),
        adc_top + filter_width * Vec2d::CreateUnitVec2d(
                                     vehicle_state_.heading() - M_PI_2));
    Vec2d left_point = left_top;
    Vec2d right_point = right_top;
    if (is_parking_inwards) {
      adc_top_seg.GetIntersect(common::math::LineSegment2d(left_down, left_top),
                               &left_point);
      adc_top_seg.GetIntersect(
          common::math::LineSegment2d(right_down, right_top), &right_point);
    } else {
      adc_bottom_seg.GetIntersect(
          common::math::LineSegment2d(left_down, left_top), &left_point);
      adc_bottom_seg.GetIntersect(
          common::math::LineSegment2d(right_down, right_top), &right_point);
    }
    left_entered_depth_ =
        std::max(left_point.DistanceTo(left_top), left_entered_depth_);
    right_entered_depth_ =
        std::max(right_point.DistanceTo(right_top), right_entered_depth_);
    const double valid_extra_depth = is_narrow_spot_scenario ? 0.5 : 1.5;
    if (left_entered_depth_ + valid_extra_depth > filter_length ||
        right_entered_depth_ + valid_extra_depth > filter_length) {
      ADEBUG << "enter very deep, filter nothing";
      return;
    }
    left_top += (left_entered_depth_ + valid_extra_depth) *
                common::math::Vec2d::CreateUnitVec2d(ld2lt.Angle() + M_PI);
    right_top += (right_entered_depth_ + valid_extra_depth) *
                 common::math::Vec2d::CreateUnitVec2d(ld2lt.Angle() + M_PI);
  }
  const auto free_space_filter_points =
      std::vector<Vec2d>{left_top, left_down, right_down, right_top};
  if (common::math::Polygon2d::IsConvexPolygon(free_space_filter_points)) {
    obs_filter->filter_areas.emplace_back(free_space_filter_points, true);
  }
}

void OpenSpaceRoiDecider::LateralParkInFilter(
    const ParkingLotVertexType& parking_spot_enu,
    const bool is_slot_inner_fs_valid,
    const perception::ParkingLotOut::SenType& sensor_type,
    ObsFilter* const obs_filter) {
  if (nullptr == obs_filter) {
    return;
  }
  if (is_slot_inner_fs_valid) {
    return;
  }
  static constexpr double kAdjustBuffer = 0.3;
  static constexpr double kFilterBufferForUssLot = 2.3;
  std::vector<Vec2d> free_space_filter_points(std::begin(parking_spot_enu),
                                              std::end(parking_spot_enu));
  const auto lt2ld_unit_vec = Vec2d::CreateUnitVec2d(
      (free_space_filter_points[1] - free_space_filter_points[0]).Angle());
  if (sensor_type == perception::ParkingLotOut::USS) {
    free_space_filter_points[1] =
        free_space_filter_points[0] + kFilterBufferForUssLot * lt2ld_unit_vec;
    free_space_filter_points[2] =
        free_space_filter_points[3] + kFilterBufferForUssLot * lt2ld_unit_vec;
  } else {
    free_space_filter_points[1] -= kAdjustBuffer * lt2ld_unit_vec;
    free_space_filter_points[2] -= kAdjustBuffer * lt2ld_unit_vec;
  }
  if (common::math::Polygon2d::IsConvexPolygon(free_space_filter_points)) {
    obs_filter->filter_areas.emplace_back(free_space_filter_points, true);
  }
}

void OpenSpaceRoiDecider::VerticalParkOutFilter(ObsFilter* const obs_filter) {
  Vec2d init_adc_pose_vec = Vec2d(init_adc_point_.x(), init_adc_point_.y());
  auto adc_bottom_line = common::math::LineSegment2d(
      init_adc_pose_vec,
      init_adc_pose_vec +
          Vec2d::CreateUnitVec2d(init_adc_point_.theta() - M_PI_2));
  adc_bottom_line.Translate(vehicle_params_.back_edge_to_center(),
                            init_adc_point_.theta() + M_PI);
  // ignore obs below init adc pose
  obs_filter->filter_planes.emplace_back(adc_bottom_line, false);
}

void OpenSpaceRoiDecider::VerticalParkOutLowFsFilter(
    ObsFilter* const obs_filter) {
  common::math::LineSegment2d rear_center_to_right_wheel(
      {init_adc_point_.x(), init_adc_point_.y()},
      {init_adc_point_.x() + sin(init_adc_point_.theta()),
       init_adc_point_.y() - cos(init_adc_point_.theta())});
  obs_filter->filter_planes.emplace_back(rear_center_to_right_wheel, false);
}

void OpenSpaceRoiDecider::GetParkingSpots(
    Frame* const frame,
    std::unordered_map<uint32_t, ParkLotInfo>* const park_lot_map_ptr) {
  if (nullptr == frame || nullptr == park_lot_map_ptr) {
    AERROR << "GetParkingSpots error; frame or park_lot_map_ptr is nullptr";
    return;
  }
  const bool is_apa_mode = (parking_type_ == AVPStatus::PARKING_IN ||
                            parking_type_ == AVPStatus::PARKING_OUT_LEFT ||
                            parking_type_ == AVPStatus::PARKING_OUT_RIGHT ||
                            parking_type_ == AVPStatus::PARKING_OUT_FRONT ||
                            parking_type_ == AVPStatus::PARKING_OUT_BACK ||
                            parking_type_ == AVPStatus::PARKING_OUT_NNS);

  if (!is_apa_mode) {
    return;
  }

  if (!frame->local_view().HasParkingLotOutArray()) {
    return;
  }
  frame_->mutable_open_space_info()->set_open_space_path_info_id(
      static_cast<int32_t>(
          frame->local_view().GetParkingLotOutArray()->opt_parking_seq()));
  TransParkLotsToOpenSpace(frame->local_view().GetParkingLotOutArray(),
                           park_lot_map_ptr);
}

common::Status OpenSpaceRoiDecider::InputValidCheck(
    const std::unordered_map<uint32_t, ParkLotInfo>& park_lot_map) {
  std::string error_msg;
  switch (parking_type_) {
    case TL::planning::AVPStatus_ParkingType_PARKING_IN:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_LEFT:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_RIGHT:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_FRONT:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_BACK:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_NNS: {
      const uint32_t opt_parking_id =
          frame_->open_space_info().open_space_path_info_id();
      if (park_lot_map.find(opt_parking_id) == park_lot_map.end()) {
        error_msg = "No optimal parking lot";
        break;
      }
      if (park_lot_map.at(opt_parking_id).status != NORMAL) {
        error_msg =
            "Parking lot is not normal " +
            ParkLotStatusToString(park_lot_map.at(opt_parking_id).status);
        break;
      }
      frame_->set_parking_lot_vertices(
          park_lot_map.at(opt_parking_id).vertices);
      break;
    }
    case TL::planning::AVPStatus_ParkingType_NNS_ADJUST: {
      if (!frame_->open_space_info().open_space_path_info().end_point.has_x() ||
          !frame_->open_space_info().open_space_path_info().end_point.has_y() ||
          !frame_->open_space_info()
               .open_space_path_info()
               .end_point.has_theta() ||
          std::get<0>(frame_->open_space_info()
                          .open_space_path_info()
                          .dest_region_with_angle)
                  .num_points() < 3) {
        error_msg = " NNS Adjust has not set end pose info";
      }
      break;
    }
    case TL::planning::AVPStatus_ParkingType_TEST_CONTROL_MODE: {
      AINFO << "The scene is control test scene";
      break;
    }
    default:
      error_msg = "Waiting for parking command";
      break;
  }
  if (!error_msg.empty()) {
    return Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, error_msg);
  }
  return Status::OK();
}

void OpenSpaceRoiDecider::TransParkLotsToOpenSpace(
    const std::shared_ptr<const perception::ParkingLotOutArray>& park_lot_array,
    std::unordered_map<uint32_t, ParkLotInfo>* const park_lot_map_ptr) {
  if (nullptr == park_lot_array || nullptr == park_lot_map_ptr) {
    AERROR << "TransParkLotsToOpenSpace error; park_lot_array or "
              "park_lot_map_ptr is nullptr";
    return;
  }
  const auto& previous_frame = injector_->frame_history()->Latest();
  const bool is_on_trace_path =
      previous_frame != nullptr &&
      previous_frame->open_space_info().partitioned_paths().path_type ==
          planning_internal::PathUpdateStatus::TRACE_PATH;
  auto update_park_lot = [&](const uint32_t parking_seq,
                             const ParkLotInfo& park_lot_value) {
    if (park_lot_map_ptr->find(parking_seq) == park_lot_map_ptr->end()) {
      park_lot_map_ptr->emplace(parking_seq, park_lot_value);
    } else {
      if (park_lot_value.is_parking_lot_update) {
        park_lot_map_ptr->at(parking_seq) = park_lot_value;
        park_lot_map_ptr->at(parking_seq).is_high_quality_triggered = true;
      } else if (park_lot_map_ptr->at(parking_seq).status != NORMAL ||
                 is_on_trace_path) {
        park_lot_map_ptr->at(parking_seq) = park_lot_value;
      } else {
        park_lot_map_ptr->at(parking_seq).is_parking_lot_update = false;
        park_lot_map_ptr->at(parking_seq).park_type = park_lot_value.park_type;
        park_lot_map_ptr->at(parking_seq).wheel_mask =
            park_lot_value.wheel_mask;
        park_lot_map_ptr->at(parking_seq).status = park_lot_value.status;
        park_lot_map_ptr->at(parking_seq).is_right_side =
            park_lot_value.is_right_side;
        park_lot_map_ptr->at(parking_seq).sensor_type =
            park_lot_value.sensor_type;
        park_lot_map_ptr->at(parking_seq).is_narrow_spot =
            park_lot_value.is_narrow_spot;
      }
    }
  };
  for (const auto& origin_park_lot : park_lot_array->parking_lots()) {
    if (!origin_park_lot.has_parking_seq()) {
      continue;
    }
    ParkLotInfo park_lot_value;
    const auto& parking_seq = origin_park_lot.parking_seq();
    park_lot_value.park_type = origin_park_lot.type();
    const size_t vertex_num = origin_park_lot.pts_enu_size();
    if (!FLAGS_enable_ignore_slot_status &&
        parking_type_ == planning::AVPStatus::PARKING_IN &&
        origin_park_lot.status() != TL::perception::ParkingLotOut::FREE) {
      park_lot_value.status = UNFREE;
      update_park_lot(parking_seq, park_lot_value);
      continue;
    }
    if (vertex_num < kParkingLotVertexNum) {
      park_lot_value.status = INCOMPLETE;
      update_park_lot(parking_seq, park_lot_value);
      continue;
    }
    for (const auto& parking_lot_point : origin_park_lot.pts_enu()) {
      size_t idx = 0;
      switch (parking_lot_point.position()) {
        case TL::perception::PSPoint_Position_TOP_LEFT: {
          idx = 0;
          break;
        }
        case TL::perception::PSPoint_Position_BOTTOM_LEFT: {
          idx = 1;
          break;
        }
        case TL::perception::PSPoint_Position_BOTTOM_RIGHT: {
          idx = 2;
          break;
        }
        case TL::perception::PSPoint_Position_TOP_RIGHT: {
          idx = 3;
          break;
        }
        case TL::perception::PSPoint_Position_STOP_LEFT: {
          idx = 4;
          std::get<1>(park_lot_value.wheel_mask) = {
              parking_lot_point.point().x(), parking_lot_point.point().y()};
          std::get<0>(park_lot_value.wheel_mask) =
              FLAGS_enable_consider_wheel_mask;
          break;
        }
        case TL::perception::PSPoint_Position_STOP_RIGHT: {
          idx = 5;
          std::get<2>(park_lot_value.wheel_mask) = {
              parking_lot_point.point().x(), parking_lot_point.point().y()};
          std::get<0>(park_lot_value.wheel_mask) =
              FLAGS_enable_consider_wheel_mask;
          break;
        }
        default: {
          AERROR << "Space Perception transform parking point is fail!";
          break;
        }
      }
#ifdef FOR_BAIDU_SIMULATION
      std::get<0>(park_lot_value.wheel_mask) = false;
#endif
      if (idx >= vertex_num) {
        park_lot_value.status = POSITION_ERROR;
        update_park_lot(parking_seq, park_lot_value);
        continue;
      }
      if (parking_lot_point.position() == perception::PSPoint::TOP_LEFT ||
          parking_lot_point.position() == perception::PSPoint::TOP_RIGHT) {
        park_lot_value.is_parking_lot_update =
            parking_lot_point.quality() == TL::perception::PSPoint::HIGH;
      }
      if (idx < park_lot_value.vertices.size()) {
        park_lot_value.vertices.at(idx) =
            Vec2d(parking_lot_point.point().x(), parking_lot_point.point().y());
      }
    }
    park_lot_value.is_narrow_spot =
        origin_park_lot.park_size() == perception ::ParkingLotOut::NARROW;
    CheckReceiveParkinglot(park_lot_value.park_type, park_lot_value.vertices,
                           &(park_lot_value.status));
    park_lot_value.is_right_side = IsParkLotInRightSide(
        park_lot_value.vertices.at(0), park_lot_value.vertices.at(3));
    park_lot_value.sensor_type = origin_park_lot.sensor_type();
    update_park_lot(parking_seq, park_lot_value);
  }
}

bool OpenSpaceRoiDecider::IsParkLotInRightSide(const Vec2d& left_top_enu,
                                               const Vec2d& right_top_enu) {
  auto left_top_vrf = common::math::ENUToRFU(
      left_top_enu.x(), left_top_enu.y(), init_adc_point_.x(),
      init_adc_point_.y(), init_adc_point_.theta());
  auto right_top_vrf = common::math::ENUToRFU(
      right_top_enu.x(), right_top_enu.y(), init_adc_point_.x(),
      init_adc_point_.y(), init_adc_point_.theta());
  return right_top_vrf.second > left_top_vrf.second;
}

bool OpenSpaceRoiDecider::InitEndPoseBaseSlot(
    Frame* const frame, const perception::ParkingLotOut::ParkType& park_type,
    const ParkingLotVertexType& vertices, const bool is_right_side,
    const std::tuple<bool, Vec2d, Vec2d>& wheel_mask,
    common::PathPoint* const end_pose_enu_ptr,
    bool* const is_parking_inwards_ptr) {
  if (nullptr == frame || nullptr == end_pose_enu_ptr ||
      nullptr == is_parking_inwards_ptr) {
    return false;
  }
  // end_pose(x, y, heading, speed)
  // Speed is set to be always zero now for parking
  // init park out target pose
  int target_direction = 1;
  switch (parking_type_) {
    case TL::planning::AVPStatus_ParkingType_PARKING_IN: {
      // rotate the points to have the lane to be horizontal to x axis positive
      // direction and scale them base on the origin point
      SetParkingSpotEndPose(frame, park_type, vertices, is_right_side,
                            wheel_mask, end_pose_enu_ptr,
                            is_parking_inwards_ptr);
      ADEBUG << "Parking in scenarios";
      break;
    }
    case TL::planning::AVPStatus_ParkingType_TEST_CONTROL_MODE: {
      if (!init_adc_point_.has_x() || !init_adc_point_.has_y() ||
          !init_adc_point_.has_theta()) {
        AERROR << " init pose is not initalized";
        return false;
      }
      const double end_pose_x_rfu = FLAGS_ctl_calibr_x;
      const double end_pose_y_rfu = FLAGS_ctl_calibr_y;
      const double end_pose_theta_rfu = FLAGS_ctl_calibr_theta;
      auto end_pose_xy = common::math::RFUToENU(
          end_pose_x_rfu, end_pose_y_rfu, init_adc_point_.x(),
          init_adc_point_.y(), init_adc_point_.theta());
      end_pose_enu_ptr->set_x(end_pose_xy.first);
      end_pose_enu_ptr->set_y(end_pose_xy.second);
      end_pose_enu_ptr->set_theta(common::math::NormalizeAngle(
          init_adc_point_.theta() + end_pose_theta_rfu));
      ADEBUG << "Ctrl calibration scenarios";
      break;
    }
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_LEFT:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_RIGHT:
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_NNS: {
      double park_out_distance =
          park_type == TL::perception::ParkingLotOut::LATERAL
              ? config_.open_space_roi_decider_config()
                    .lateral_park_out_distance()
              : config_.open_space_roi_decider_config()
                    .vertical_park_out_distance();
      CaculateParkingOutTarget(frame, park_type, vertices,
                               target_direction * park_out_distance,
                               end_pose_enu_ptr);
      ADEBUG << "Parking out scenarios";
      break;
    }
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_BACK:
      target_direction = -1;
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_FRONT: {
      common::math::LineSegment2d down_to_target(
          (vertices[1] + vertices[2]) / 2.0, (vertices[0] + vertices[3]) / 2.0);
      double park_out_angle = down_to_target.heading();
      if (target_direction < 0) {
        down_to_target.Extend(vehicle_params_.wheel_base());
        park_out_angle = common::math::NormalizeAngle(park_out_angle + M_PI);
      }
      const common::math::Vec2d& park_out_position = down_to_target.end();
      end_pose_enu_ptr->set_x(park_out_position.x());
      end_pose_enu_ptr->set_y(park_out_position.y());
      end_pose_enu_ptr->set_theta(park_out_angle);
      ADEBUG << "verticle front Parking out scenarios";
      break;
    }
    default:
      return false;
  }
  return true;
}

void OpenSpaceRoiDecider::CaculateParkingOutTarget(
    Frame* const frame, const perception::ParkingLotOut::ParkType& park_type,
    const ParkingLotVertexType& vertices, double kSTargetBuffer,
    common::PathPoint* const end_pose_ptr) {
  *end_pose_ptr = GetPreEndPose();
  if (end_pose_ptr->has_x() && end_pose_ptr->has_y() &&
      end_pose_ptr->has_theta()) {
    return;
  }

  // init pos
  const auto& reference_line =
      frame->reference_line_info().front().reference_line();

  // common::SLPoint target_sl;
  double target_s = 0.0;
  double heading_bias = 0.0;

  common::SLPoint left_top_sl;
  reference_line.XYToSL({vertices[0].x(), vertices[0].y()}, &left_top_sl);
  if (park_type != TL::perception::ParkingLotOut::LATERAL) {
    is_park_out_along_road_ =
        (left_top_sl.l() <= 0 &&
         parking_type_ == AVPStatus::PARKING_OUT_RIGHT) ||
        (left_top_sl.l() > 0 && parking_type_ == AVPStatus::PARKING_OUT_LEFT);
  } else {
    double diff_angle = fabs(common::math::AngleDiff(
        reference_line.GetReferencePoint(left_top_sl.s()).heading(),
        init_adc_point_.theta()));
    is_park_out_along_road_ = diff_angle < M_PI_2;
  }

  if (!is_park_out_along_road_) {
    kSTargetBuffer *= -1;
    heading_bias += M_PI;
  }
  common::SLPoint init_sl;
  reference_line.XYToSL({init_adc_point_.x(), init_adc_point_.y()}, &init_sl);

  target_s = init_sl.s() + kSTargetBuffer;
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  const auto target_point = reference_line.GetReferencePoint(target_s);
  const double park_out_height =
      (park_type == TL::perception::ParkingLotOut::LATERAL)
          ? config_.open_space_roi_decider_config().lateral_park_out_height()
          : config_.open_space_roi_decider_config().vertical_park_out_height();
  reference_line.GetLaneWidth(target_s, &lane_left_width, &lane_right_width);
  double delta_width = 0.0;
  if (init_sl.l() < 0.0) {
    delta_width = -std::max(lane_right_width - park_out_height, 0.0);
  } else {
    delta_width = std::max(lane_left_width - park_out_height, 0.0);
  }
  common::SLPoint target_sl;
  target_sl.set_s(target_s);
  target_sl.set_l(delta_width);
  common::math::Vec2d xy_point;
  reference_line.SLToXY(target_sl, &xy_point);
  end_pose_ptr->set_x(xy_point.x());
  end_pose_ptr->set_y(xy_point.y());
  end_pose_ptr->set_theta(
      common::math::NormalizeAngle(target_point.heading() + heading_bias));
}

void OpenSpaceRoiDecider::SetParkingSpotEndPose(
    Frame* const frame, const perception::ParkingLotOut::ParkType& park_type,
    const ParkingLotVertexType& vertices, const bool is_right_side,
    const std::tuple<bool, Vec2d, Vec2d>& wheel_mask,
    common::PathPoint* const end_pose_ptr, bool* const is_parking_inwards_ptr) {
  UNUSED(frame);
  Vec2d stop_left;
  Vec2d stop_right;
  const auto& left_top = vertices[0];
  const auto& left_down = vertices[1];
  const auto& right_down = vertices[2];
  const auto& right_top = vertices[3];

  if (std::get<0>(wheel_mask)) {
    ADEBUG << " AVP -- AVP:consider wheel mask scenario";
    stop_left = std::get<1>(wheel_mask);
    stop_right = std::get<2>(wheel_mask);
  }
  AINFO << "park_type " << park_type;
  // calculate end_x, end_y, end_z
  switch (park_type) {
    case TL::perception::ParkingLotOut_ParkType_VERTICAL:
    case TL::perception::ParkingLotOut_ParkType_OBLIQUE: {
      AINFO << "SetNonLateralSlotEndPose ";
      SetNonLateralSlotEndPose(park_type, left_top, left_down, right_top,
                               right_down, stop_left, stop_right, is_right_side,
                               end_pose_ptr, is_parking_inwards_ptr);
      break;
    }
    case TL::perception::ParkingLotOut_ParkType_LATERAL: {
      AINFO << "SetLateralSlotEndPose ";
      SetLateralSlotEndPose(is_right_side, left_top, left_down, right_top,
                            right_down, stop_left, stop_right, end_pose_ptr);
      break;
    }
    default:
      break;
  }
}

void OpenSpaceRoiDecider::SetNonLateralSlotEndPose(
    const perception::ParkingLotOut::ParkType& park_type, const Vec2d& left_top,
    const Vec2d& left_down, const Vec2d& right_top, const Vec2d& right_down,
    const Vec2d& /*stop_left*/, const Vec2d& /*stop_right*/,
    const bool is_right_side, common::PathPoint* const end_pose_ptr,
    bool* const is_parking_inwards_ptr) {
  // first,  if oblique, obtain the corresponding slot point with respect to
  // vertical slot
  TL::common::math::Vec2d projected_left_top = left_top;
  TL::common::math::Vec2d projected_left_down = left_down;
  TL::common::math::Vec2d projected_right_top = right_top;
  TL::common::math::Vec2d projected_right_down = right_down;
  double parking_depth_buffer =
      config_.open_space_roi_decider_config().vertical_parking_depth_buffer();
  // default parking oreintation is anti-inwards
  if (park_type == TL::perception::ParkingLotOut_ParkType_OBLIQUE) {
    parking_depth_buffer =
        config_.open_space_roi_decider_config().oblique_parking_depth_buffer();
    // calculate the cos and sin of OBLIQUE angle
    Vec2d down_top_unit_vector = left_top - left_down;
    down_top_unit_vector.Normalize();
    Vec2d left_right_unit_vector = right_down - left_down;
    left_right_unit_vector.Normalize();
    const double projected_length =
        left_right_unit_vector.InnerProd(down_top_unit_vector) *
        (right_down - left_down).Length();
    if (projected_length > 0) {
      // project left down and right top point
      projected_left_down =
          projected_left_down + down_top_unit_vector * projected_length;
      projected_right_top =
          projected_right_top - down_top_unit_vector * projected_length;
    } else {
      // project left top and right down point
      projected_left_top =
          projected_left_top + down_top_unit_vector * projected_length;
      projected_right_down =
          projected_right_down - down_top_unit_vector * projected_length;
    }
    *is_parking_inwards_ptr =
        is_right_side
            ? left_right_unit_vector.InnerProd(down_top_unit_vector) < 0
            : left_right_unit_vector.InnerProd(down_top_unit_vector) > 0;
    *is_parking_inwards_ptr &= FLAGS_avp_enable_parking_inwards;
  }
  // manually set is_parking_inwards, default is false
  if (park_type == TL::perception::ParkingLotOut_ParkType_VERTICAL) {
    *is_parking_inwards_ptr = FLAGS_avp_enable_parking_inwards;
  }
  // second calculate endpose with projected slot point
  double end_x = 0.0;
  double end_y = 0.0;
  double end_phi = 0.0;
  // Step 1 set end pose heading

  const double parking_spot_heading =
      (projected_left_down - projected_left_top).Angle();
  end_phi = *is_parking_inwards_ptr
                ? parking_spot_heading
                : common::math::NormalizeAngle(parking_spot_heading + M_PI);
  // step 2 set end pose x and y

  const double top_to_down_distance =
      (projected_left_top - projected_left_down).Length();
  const Vec2d top_middle_point =
      (projected_left_top + projected_right_top) * 0.5;
  const Vec2d down_middle_point =
      (projected_left_down + projected_right_down) * 0.5;
  double dis_move = 0.0;
  // heading in  or not
  if (*is_parking_inwards_ptr) {
    dis_move = (std::max(3.0 * top_to_down_distance / 4.0,
                         vehicle_params_.front_edge_to_center()) +
                parking_depth_buffer);
    end_x = down_middle_point.x() - dis_move * cos(parking_spot_heading);
    end_y = down_middle_point.y() - dis_move * sin(parking_spot_heading);
  } else {
    dis_move = vehicle_params_.front_edge_to_center() + parking_depth_buffer;
    end_x = top_middle_point.x() + dis_move * cos(parking_spot_heading);
    end_y = top_middle_point.y() + dis_move * sin(parking_spot_heading);
  }
  end_pose_ptr->set_x(end_x);
  end_pose_ptr->set_y(end_y);
  end_pose_ptr->set_theta(end_phi);
}

void OpenSpaceRoiDecider::SetLateralSlotEndPose(
    const bool is_right_side, const Vec2d& left_top, const Vec2d& /*left_down*/,
    const Vec2d& right_top, const Vec2d& right_down, const Vec2d& stop_left,
    const Vec2d& stop_right, common::PathPoint* const end_pose_ptr) {
  UNUSED(stop_left);
  UNUSED(stop_right);
  double lt_rt_angle = (right_top - left_top).Angle();
  double rt_rd_angle = (right_down - right_top).Angle();
  double end_phi = is_right_side
                       ? lt_rt_angle
                       : common::math::NormalizeAngle(lt_rt_angle + M_PI);

  double rear_axis_to_center =
      vehicle_params_.length() * 0.5 - vehicle_params_.back_edge_to_center();
  double park_depth =
      ((config_.open_space_roi_decider_config().lateral_parking_depth_buffer() +
        vehicle_params_.left_edge_to_center()) /
       abs(sin(rt_rd_angle - lt_rt_angle)));
#ifdef FOR_BAIDU_SIMULATION
  park_depth = std::max(park_depth, 0.5 * (right_down - right_top).Length());
#endif
  Vec2d top_edge_center = (left_top + right_top) * 0.5;
  Vec2d end_position =
      top_edge_center +
      Vec2d::CreateUnitVec2d(end_phi + M_PI) * rear_axis_to_center +
      Vec2d::CreateUnitVec2d(rt_rd_angle) * park_depth;

  if (frame_->open_space_info().is_consider_wheel_mask()) {
    auto wheel_mask =
        frame_->open_space_info().open_space_wheel_mask_box().center() -
        left_top;
    wheel_mask.SelfRotate(-lt_rt_angle);
    const double wheelmask_end_pose_x =
        is_right_side
            ? wheel_mask.x() + config_.open_space_speed_optimizer_config()
                                   .wheel_mask_to_wheel_base_distance()
            : wheel_mask.x() - config_.open_space_speed_optimizer_config()
                                   .wheel_mask_to_wheel_base_distance();
    constexpr double kEndPoseToEdgeDis = 0.1;
    const double edge_end_pose_x =
        is_right_side
            ? vehicle_params_.back_edge_to_center() + kEndPoseToEdgeDis
            : left_top.DistanceTo(right_top) -
                  vehicle_params_.back_edge_to_center() - kEndPoseToEdgeDis;
    auto end_point = end_position - left_top;
    end_point.SelfRotate(-lt_rt_angle);
    double position_diff = 0.0;
    position_diff =
        is_right_side
            ? end_point.x() - std::max(wheelmask_end_pose_x, edge_end_pose_x)
            : std::min(wheelmask_end_pose_x, edge_end_pose_x) - end_point.x();
    if (position_diff > kEpsilon) {
      end_position =
          end_position + Vec2d::CreateUnitVec2d(end_phi + M_PI) * position_diff;
    }
  }

  end_pose_ptr->set_x(end_position.x());
  end_pose_ptr->set_y(end_position.y());
  end_pose_ptr->set_theta(end_phi);
}

void OpenSpaceRoiDecider::GetStaticBoundary(
    const Vec2d& origin_point, double origin_heading,
    std::vector<double>* const roi_xy_boundary) {
  // In Ctrl calibration scene, roi is fixed with adc_init_pose
  std::vector<Vec2d> inner_roi_vertex;
  inner_roi_vertex.resize(4);
  double roi_width = vehicle_params_.width() + 5 * FLAGS_direct_move_roi_width;
  double roi_forward_length =
      vehicle_params_.front_edge_to_center() + 3 * FLAGS_direct_move_roi_length;
  double roi_backward_length =
      vehicle_params_.back_edge_to_center() + 3 * FLAGS_direct_move_roi_length;
  double cos_heading = cos(init_adc_point_.theta());
  double sin_heading = sin(init_adc_point_.theta());
  // left_top
  double x_lt = init_adc_point_.x() + roi_forward_length * cos_heading -
                0.5 * roi_width * sin_heading;
  double y_lt = init_adc_point_.y() + roi_forward_length * sin_heading +
                0.5 * roi_width * cos_heading;
  inner_roi_vertex[0] = {x_lt, y_lt};
  // right_top
  double x_rt = init_adc_point_.x() + roi_forward_length * cos_heading +
                0.5 * roi_width * sin_heading;
  double y_rt = init_adc_point_.y() + roi_forward_length * sin_heading -
                0.5 * roi_width * cos_heading;
  inner_roi_vertex[1] = {x_rt, y_rt};

  // right_down
  double x_rd = init_adc_point_.x() - roi_backward_length * cos_heading +
                0.5 * roi_width * sin_heading;
  double y_rd = init_adc_point_.y() - roi_backward_length * sin_heading -
                0.5 * roi_width * cos_heading;
  inner_roi_vertex[2] = {x_rd, y_rd};

  // left_down
  double x_ld = init_adc_point_.x() - roi_backward_length * cos_heading -
                0.5 * roi_width * sin_heading;
  double y_ld = init_adc_point_.y() - roi_backward_length * sin_heading +
                0.5 * roi_width * cos_heading;
  inner_roi_vertex[3] = {x_ld, y_ld};
  std::vector<common::math::LineSegment2d> inner_roi_boundary;
  inner_roi_boundary.resize(4);
  inner_roi_boundary.at(0) = {inner_roi_vertex[0], inner_roi_vertex[1]};
  inner_roi_boundary.at(1) = {inner_roi_vertex[1], inner_roi_vertex[2]};
  inner_roi_boundary.at(2) = {inner_roi_vertex[2], inner_roi_vertex[3]};
  inner_roi_boundary.at(3) = {inner_roi_vertex[3], inner_roi_vertex[0]};
  TransRoiAndSetBoundary(origin_point, origin_heading, &inner_roi_vertex,
                         &inner_roi_vertex, &inner_roi_boundary,
                         &inner_roi_boundary, roi_xy_boundary);
}

void OpenSpaceRoiDecider::GetDynamicBoundary(
    const std::vector<double>& roi_box,
    std::vector<common::math::Vec2d>* const roi_vertex) {
  if (nullptr == roi_vertex) {
    return;
  }
  roi_vertex->resize(4);
  if (roi_box.empty() || roi_box.size() != 4) {
    AERROR << "roi points from cruise size:" << roi_box.size();
    return;
  }
  roi_vertex->at(0) = Vec2d(roi_box[1], roi_box[3]);
  roi_vertex->at(1) = Vec2d(roi_box[0], roi_box[3]);
  roi_vertex->at(2) = Vec2d(roi_box[0], roi_box[2]);
  roi_vertex->at(3) = Vec2d(roi_box[1], roi_box[2]);
}

void OpenSpaceRoiDecider::CalculateInnerRoi(
    const perception::ParkingLotOut::ParkType& park_type,
    const ParkingLotVertexType& spot_vertices, const bool is_use_larger_roi,
    std::vector<Vec2d>* const inner_roi_vertex) {
  auto left_top = spot_vertices[0];
  auto left_down = spot_vertices[1];
  auto right_down = spot_vertices[2];
  auto right_top = spot_vertices[3];
  common::PathPoint adc_point;
  adc_point.set_x(vehicle_state_.x());
  adc_point.set_y(vehicle_state_.y());
  adc_point.set_theta(vehicle_state_.heading());
  const double adc_height =
      CalculateAdcHeight(adc_point,
                         common::math::LineSegment2d(left_down, right_down)) -
      left_top.DistanceTo(left_down);
  static constexpr double kDefaultAcAroundRange = 2.5;

  road_width_ = fmax(road_width_, (adc_height + kDefaultAcAroundRange) / 2);
  const auto top_edge = common::math::LineSegment2d(left_top, right_top);
  auto left_edge = common::math::LineSegment2d(left_top, left_down);
  auto right_edge = common::math::LineSegment2d(right_top, right_down);
  double start_s =
      config_.open_space_roi_decider_config().roi_longitudinal_range_start() -
      top_edge.length() / 2;
  double end_s =
      config_.open_space_roi_decider_config().roi_longitudinal_range_end() -
      top_edge.length() / 2;
  const bool is_park_in =
      parking_type_ == TL::planning::AVPStatus_ParkingType_PARKING_IN;
  if (is_use_larger_roi) {
    ADEBUG << " sensor config is with lidar, use larger roi boundary";
    common::math::LineSegment2d roi_top(left_top, right_top);
    const double slot_height = roi_top.DistanceTo(left_down);
    roi_top.Translate(
        config_.open_space_roi_decider_config().lateral_expand_bottom_buffer() +
            slot_height,
        roi_top.heading() - M_PI_2);
    roi_top.Translate(start_s, roi_top.heading() + M_PI);
    inner_roi_vertex->reserve(4);
    inner_roi_vertex->push_back(roi_top.start());
    roi_top.Extend(start_s + end_s);
    inner_roi_vertex->push_back(roi_top.end());
    roi_top.Translate(2 * road_width_ + slot_height +
                          config_.open_space_roi_decider_config()
                              .lateral_expand_bottom_buffer(),
                      roi_top.heading() + M_PI_2);
    inner_roi_vertex->push_back(roi_top.end());
    inner_roi_vertex->push_back(roi_top.start());
    return;
  }
  // decider slot nearby roi boundary
  std::vector<Vec2d> common_vec2d = {left_top, left_down, right_down,
                                     right_top};
  double lat_edge_expand_buffer = 0.0;
  switch (park_type) {
    case TL::perception::ParkingLotOut::LATERAL: {
      lat_edge_expand_buffer =
          vehicle_params_.length() +
          config_.open_space_roi_decider_config().lateral_expand_buffer() -
          top_edge.length();
      left_edge.Extend(config_.open_space_roi_decider_config()
                           .lateral_expand_bottom_buffer());
      right_edge.Extend(config_.open_space_roi_decider_config()
                            .lateral_expand_bottom_buffer());
      break;
    }
    case TL::perception::ParkingLotOut::VERTICAL:
    case TL::perception::ParkingLotOut::OBLIQUE: {
      const double project_ratio =
          fabs(sin(left_edge.heading() - top_edge.heading()));
      if (project_ratio <= kEpsilon) {
        AERROR << "park slot is not legal";
        return;
      }
      double lat_expand_buffer =
          is_park_in
              ? config_.open_space_roi_decider_config().vertical_expand_buffer()
              : config_.open_space_roi_decider_config()
                    .park_out_vertical_expand_buffer();
      lat_edge_expand_buffer =
          (vehicle_params_.width() + lat_expand_buffer) / project_ratio -
          top_edge.length();
      break;
    }
    default: {
      lat_edge_expand_buffer =
          vehicle_params_.width() +
          config_.open_space_roi_decider_config().vertical_expand_buffer() -
          top_edge.length();
      break;
    }
  }
  constexpr double kMinExpandBuffer = 0.1;
  if (lat_edge_expand_buffer > kMinExpandBuffer) {
    left_edge.Translate(0.5 * lat_edge_expand_buffer,
                        top_edge.heading() + M_PI);
    right_edge.Translate(0.5 * lat_edge_expand_buffer, top_edge.heading());
    common_vec2d[0] = left_edge.start();
    common_vec2d[1] = left_edge.end();
    common_vec2d[2] = right_edge.end();
    common_vec2d[3] = right_edge.start();
  }
  inner_roi_vertex->reserve(8);
  common::math::LineSegment2d roi_lane(left_top, right_top);
  roi_lane.Translate(start_s, roi_lane.heading() + M_PI);
  inner_roi_vertex->push_back(roi_lane.start());
  // insert slot nearby roi boundary
  inner_roi_vertex->insert(inner_roi_vertex->end(), common_vec2d.begin(),
                           common_vec2d.end());
  // insert other search domain boundary
  roi_lane.Extend(start_s + end_s);
  inner_roi_vertex->push_back(roi_lane.end());
  roi_lane.Translate(2 * road_width_, roi_lane.heading() + M_PI_2);
  inner_roi_vertex->push_back(roi_lane.end());
  inner_roi_vertex->push_back(roi_lane.start());
}

double OpenSpaceRoiDecider::CalculateAdcHeight(
    const common::PathPoint& adc_point,
    const common::math::LineSegment2d& left_to_right_bottom) {
  const auto adc_box = common::VehicleConfigHelper::GetBoundingBox(adc_point);
  double height = 0.0;
  double point_dist = 0.0;
  for (const auto& point : adc_box.GetAllCorners()) {
    point_dist = fabs(left_to_right_bottom.ProductOntoUnit(point));
    height = fmax(point_dist, height);
  }
  ADEBUG << "adc height is: " << height;
  return height;
}

bool OpenSpaceRoiDecider::IsUseLargerRoi(
    const ParkingLotVertexType& spot_vertices) {
  if (sensor_config_state_ != SensorConfigState::LIDAR) {
    return false;
  }
  if (parking_type_ == AVPStatus::PARKING_IN) {
    return true;
  }
  static constexpr double kEnableOuterFsAdcHeight = 1.0;
  const auto adc_point = Vec2d(vehicle_state_.x(), vehicle_state_.y());
  return common::math::LineSegment2d(spot_vertices.at(0), spot_vertices.at(3))
             .ProductOntoUnit(adc_point) > kEnableOuterFsAdcHeight;
}

double OpenSpaceRoiDecider::CalculateAdcDis2ParkBottom(
    const common::PathPoint& adc_point,
    const common::math::LineSegment2d& left_to_right_bottom) {
  double bottom_dist = 100.0;
  double point_dist = 0.0;
  const auto adc_box = common::VehicleConfigHelper::GetBoundingBox(adc_point);
  for (const auto& point : adc_box.GetAllCorners()) {
    point_dist = fabs(left_to_right_bottom.ProductOntoUnit(point));
    bottom_dist = fmin(point_dist, bottom_dist);
  }
  ADEBUG << "adc dist to park bottom is: " << bottom_dist;
  return bottom_dist;
}

void OpenSpaceRoiDecider::CalculateOuterRoi(
    const std::vector<Vec2d>& inner_roi_vertex,
    std::vector<Vec2d>* const outer_roi_vertex) {
  outer_roi_vertex->assign(inner_roi_vertex.begin(), inner_roi_vertex.end());
  int roi_edge_num = static_cast<int>(outer_roi_vertex->size());
  if (roi_edge_num < 2) {
    return;
  }
  const double slack_distance = 0.5;
  for (int i = 0; i < roi_edge_num; ++i) {
    int pre_idx = (i - 1 + roi_edge_num) % roi_edge_num;
    int next_idx = (i + 1) % roi_edge_num;
    Vec2d vec1 =
        slack_distance *
        Vec2d::CreateUnitVec2d(
            (inner_roi_vertex.at(i) - inner_roi_vertex.at(pre_idx)).Angle());
    Vec2d vec2 =
        slack_distance *
        Vec2d::CreateUnitVec2d(
            (inner_roi_vertex.at(next_idx) - inner_roi_vertex.at(i)).Angle());

    Vec2d vec_merge = vec1 + vec2;
    vec_merge.SelfRotate(-M_PI_2);
    double length =
        fabs(slack_distance / sin(vec_merge.Angle() - vec1.Angle()));
    Vec2d vec = inner_roi_vertex.at(i) +
                length * Vec2d::CreateUnitVec2d(vec_merge.Angle());

    outer_roi_vertex->at(i) = vec;
  }
}

bool OpenSpaceRoiDecider::IsVehicleInRoi(
    const Vec2d& origin_point, double origin_heading,
    const std::vector<double>& roi_xy_boundary) {
  auto vehicle_xy = Vec2d(vehicle_state_.x(), vehicle_state_.y());
  vehicle_xy -= origin_point;
  vehicle_xy.SelfRotate(-origin_heading);

  AINFO << "vehicle_xy x " << vehicle_xy.x() << " y " << vehicle_xy.y();
  AINFO << "roi_xy_boundary " << roi_xy_boundary[0] << " " << roi_xy_boundary[1]
        << " " << roi_xy_boundary[2] << " " << roi_xy_boundary[3];
  if (vehicle_xy.x() < roi_xy_boundary[0] ||
      vehicle_xy.x() > roi_xy_boundary[1] ||
      vehicle_xy.y() < roi_xy_boundary[2] ||
      vehicle_xy.y() > roi_xy_boundary[3]) {
    AERROR << "vehicle outside of xy boundary of parking ROI";
    return false;
  }
  return true;
}

bool OpenSpaceRoiDecider::IsValidTracePath(  // NOLINT
    const RepeatedPtrField<::TL::perception::ParkingPathPoint>&
        origin_trace_path,
    const ParkingLotVertexType& vertices,
    DiscretizedPath* const trace_path_ptr) {
  if (nullptr == trace_path_ptr) {
    ADEBUG << "Invalid input to IsValidTracePath";
    return false;
  }
  if (origin_trace_path.empty()) {
    ADEBUG << "trace path is empty";
    return false;
  }
  const auto& previous_frame = injector_->frame_history()->Latest();
  if (nullptr != previous_frame) {
    const auto& partitioned_paths =
        previous_frame->open_space_info().partitioned_paths();
    if (partitioned_paths.path_type ==
            planning_internal::PathUpdateStatus::TRACE_PATH &&
        !partitioned_paths.path_set.empty() &&
        partitioned_paths.path_set.at(partitioned_paths.path_idx)
                .first.Length() > kEpsilon) {
      *trace_path_ptr =
          partitioned_paths.path_set.at(partitioned_paths.path_idx).first;
      return true;
    }
  }

  if (!ChoosePartitionTracePath(origin_trace_path, trace_path_ptr)) {
    return false;
  }
  // top edge center project to path
  // clip adc to top edge center path
  Vec2d top_edge_mid = 0.5 * (vertices[0] + vertices[3]);
  common::SLPoint top_edge_mid_sl;
  if (!trace_path_ptr->XYToSL(top_edge_mid.x(), top_edge_mid.y(),
                              &top_edge_mid_sl)) {
    trace_path_ptr->clear();
    ADEBUG << "top_edge_mid project failed";
    return false;
  }
  auto iter = std::find_if(
      trace_path_ptr->begin(), trace_path_ptr->end(),
      [&top_edge_mid_sl](const common::PathPoint& path_point) {
        return top_edge_mid_sl.s() + 2 * kMinTraceDis <= path_point.s();
      });
  trace_path_ptr->erase(iter, trace_path_ptr->end());
  if (trace_path_ptr->empty() || trace_path_ptr->back().s() < kMinTraceDis) {
    trace_path_ptr->clear();
    ADEBUG << "trace path is too short, size =  " << trace_path_ptr->size();
    return false;
  }
  return true;
}

bool OpenSpaceRoiDecider::ChoosePartitionTracePath(
    const RepeatedPtrField<::TL::perception::ParkingPathPoint>& trace_path,
    DiscretizedPath* const chosen_trace_path) {
  if (nullptr == chosen_trace_path || trace_path.empty() ||
      !trace_path.begin()->has_gear() || !trace_path.begin()->has_x() ||
      !trace_path.begin()->has_y() || !trace_path.begin()->has_yaw()) {
    return false;
  }

  std::vector<PathGearPair> trace_path_set;
  PathGearPair partition_path;
  common::PathPoint path_point;
  double s = 0;
  trace_path_set.clear();

  auto current_gear = trace_path.begin()->gear();
  path_point.set_x(trace_path.begin()->x());
  path_point.set_y(trace_path.begin()->y());
  path_point.set_theta(trace_path.begin()->yaw());
  path_point.set_s(s);
  partition_path.first.emplace_back(path_point);
  partition_path.second = MapParkingGearToChassisGear(current_gear);
  for (int i = 1; i < trace_path.size(); ++i) {
    if (!trace_path.at(i).has_gear() || !trace_path.at(i).has_x() ||
        !trace_path.at(i).has_y() || !trace_path.at(i).has_yaw()) {
      return false;
    }
    if (trace_path.at(i).gear() != current_gear) {
      trace_path_set.emplace_back(partition_path);
      current_gear = trace_path.at(i).gear();
      partition_path.first.clear();
      partition_path.second = MapParkingGearToChassisGear(current_gear);
    }
    path_point.set_x(trace_path.at(i).x());
    path_point.set_y(trace_path.at(i).y());
    path_point.set_theta(trace_path.at(i).yaw());
    if (partition_path.first.empty()) {
      s = 0.0;
    } else {
      double dx = trace_path.at(i).x() - partition_path.first.back().x();
      double dy = trace_path.at(i).y() - partition_path.first.back().y();
      s = partition_path.first.back().s() + std::hypot(dx, dy);
    }
    path_point.set_s(s);
    partition_path.first.emplace_back(path_point);
  }
  trace_path_set.emplace_back(partition_path);

  common::SLPoint start_point_sl;
  double max_length = 0.0;
  for (const auto& path : trace_path_set) {
    if (path.second != soc::Chassis::GEAR_DRIVE ||
        !path.first.XYToSL(vehicle_state_.x(), vehicle_state_.y(),
                           &start_point_sl)) {
      continue;
    }
    const auto matched_point = path.first.Evaluate(start_point_sl.s());
    if (std::fabs(common::math::AngleDiff(matched_point.theta(),
                                          vehicle_state_.heading())) < M_PI_2 &&
        path.first.Length() > max_length) {
      max_length = path.first.Length();
      *chosen_trace_path = path.first;
    }
  }
  return std::fabs(max_length) > kEpsilon;
}

void OpenSpaceRoiDecider::GetParkingBoundary(
    Frame* const frame, const Vec2d& origin_point, const double origin_heading,
    const perception::ParkingLotOut::ParkType& park_type,
    const ParkingLotVertexType& vertices, const common::PathPoint& end_pose_enu,
    std::vector<common::math::LineSegment2d>* const inner_roi_boundary,
    std::vector<common::math::LineSegment2d>* const outer_roi_boundary,
    std::vector<double>* const roi_xy_boundary) {
  UNUSED(frame);
  std::vector<Vec2d> inner_roi_vertex;
  std::vector<Vec2d> outer_roi_vertex;
  is_use_larger_roi_ = is_use_larger_roi_ || IsUseLargerRoi(vertices);
  CalculateInnerRoi(park_type, vertices, is_use_larger_roi_, &inner_roi_vertex);
  RoiBoundarySlack(end_pose_enu, is_use_larger_roi_, &inner_roi_vertex);
  CalculateOuterRoi(inner_roi_vertex, &outer_roi_vertex);
  TransRoiAndSetBoundary(origin_point, origin_heading, &inner_roi_vertex,
                         &outer_roi_vertex, inner_roi_boundary,
                         outer_roi_boundary, roi_xy_boundary);
}

void OpenSpaceRoiDecider::TransRoiAndSetBoundary(
    const Vec2d& origin_point, const double origin_heading,
    std::vector<Vec2d>* const inner_roi_vertex,
    std::vector<Vec2d>* const outer_roi_vertex,
    std::vector<common::math::LineSegment2d>* const inner_roi_boundary,
    std::vector<common::math::LineSegment2d>* const outer_roi_boundary,
    std::vector<double>* const roi_xy_boundary) {
  for (size_t i = 0; i + 1 < inner_roi_vertex->size(); ++i) {
    inner_roi_boundary->push_back(
        {inner_roi_vertex->at(i), inner_roi_vertex->at(i + 1)});
  }
  for (size_t i = 0; i + 1 < outer_roi_vertex->size(); ++i) {
    outer_roi_boundary->push_back(
        {outer_roi_vertex->at(i), outer_roi_vertex->at(i + 1)});
  }
  for (auto& point : *inner_roi_vertex) {
    point -= origin_point;
    point.SelfRotate(-origin_heading);
  }
  // Get xy boundary
  auto xminmax = std::minmax_element(
      inner_roi_vertex->begin(), inner_roi_vertex->end(),
      [](const Vec2d& a, const Vec2d& b) { return a.x() < b.x(); });
  auto yminmax = std::minmax_element(
      inner_roi_vertex->begin(), inner_roi_vertex->end(),
      [](const Vec2d& a, const Vec2d& b) { return a.y() < b.y(); });
  *roi_xy_boundary = {xminmax.first->x(), xminmax.second->x(),
                      yminmax.first->y(), yminmax.second->y()};
}

void OpenSpaceRoiDecider::RoiBoundarySlack(
    const common::PathPoint& end_pose_enu, const bool is_use_larger_roi,
    std::vector<Vec2d>* const inner_roi_vertex) {
  if (nullptr == inner_roi_vertex || is_use_larger_roi) {
    ADEBUG << "Skip roi slack";
    return;
  }
  // if roi boundary has collision with veh
  // translate the roi boundary
  /*
   ------------------------------
                                |
                                |
                                |
   --------------    ------------
                |   |
                |   |
                -----
  */
  // slack with current and end pose
  const double default_slack_distance =
      config_.open_space_roi_decider_config().roi_slack_distance();
  common::PathPoint adc_pose;
  adc_pose.set_x(vehicle_state_.x());
  adc_pose.set_y(vehicle_state_.y());
  adc_pose.set_theta(vehicle_state_.heading());
  auto adc_box = common::VehicleConfigHelper::GetBoundingBox(
      adc_pose, kADCBoxEps, kADCBoxEps);
  auto end_pose_box = common::VehicleConfigHelper::GetBoundingBox(
      end_pose_enu, kADCBoxEps, kADCBoxEps);
  if (TL::planning::AVPStatus_ParkingType_PARKING_IN == parking_type_) {
    adc_box.LateralExtend(
        config_.open_space_roi_decider_config().roi_edge_buffer());
  }
  if (last_slack_dist_vec_.size() != inner_roi_vertex->size()) {
    last_slack_dist_vec_.clear();
    last_slack_dist_vec_.resize(inner_roi_vertex->size());
  }
  for (size_t i = 1; i < inner_roi_vertex->size(); ++i) {
    common::math::LineSegment2d roi_lane(inner_roi_vertex->at(i - 1),
                                         inner_roi_vertex->at(i));
    common::math::LineSegment2d pre_roi_lane{
        inner_roi_vertex->at((i - 2 + inner_roi_vertex->size()) %
                             inner_roi_vertex->size()),
        inner_roi_vertex->at((i - 1 + inner_roi_vertex->size()) %
                             inner_roi_vertex->size())};
    pre_roi_lane.Translate(default_slack_distance,
                           pre_roi_lane.heading() + M_PI);
    pre_roi_lane.Extend(2 * default_slack_distance);
    common::math::LineSegment2d next_roi_lane{
        inner_roi_vertex->at((i + inner_roi_vertex->size()) %
                             inner_roi_vertex->size()),
        inner_roi_vertex->at((i + 1 + inner_roi_vertex->size()) %
                             inner_roi_vertex->size())};
    next_roi_lane.Translate(default_slack_distance,
                            next_roi_lane.heading() + M_PI);
    next_roi_lane.Extend(2 * default_slack_distance);
    size_t pre_idx =
        (i - 1 + inner_roi_vertex->size()) % inner_roi_vertex->size();
    size_t next_idx = (i + 1) % inner_roi_vertex->size();
    Vec2d vec1 = Vec2d::CreateUnitVec2d(
        (inner_roi_vertex->at(i) - inner_roi_vertex->at(pre_idx)).Angle());
    Vec2d vec2 = Vec2d::CreateUnitVec2d(
        (inner_roi_vertex->at(next_idx) - inner_roi_vertex->at(i)).Angle());
    double angle_bias = vec1.CrossProd(vec2) > 0 ? M_PI : 0;
    roi_lane.Translate(last_slack_dist_vec_.at(i), angle_bias + vec2.Angle());
    if (adc_box.HasOverlap(roi_lane) || end_pose_box.HasOverlap(roi_lane)) {
      ADEBUG << "slack roi boundary has collision with current pose "
             << adc_box.HasOverlap(roi_lane);
      ADEBUG << "slack roi boundary has collision with end pose "
             << end_pose_box.HasOverlap(roi_lane);
      AINFO << "slack roi boundary has collision with current or end pose "
            << i;
      roi_lane.Translate(default_slack_distance, angle_bias + vec2.Angle());
      last_slack_dist_vec_.at(i) += default_slack_distance;
    }
    Vec2d slack_start_point = roi_lane.start();
    Vec2d slack_end_point = roi_lane.end();
    if (i < inner_roi_vertex->size() - 1) {
      roi_lane.GetIntersect(pre_roi_lane, &slack_start_point);
      roi_lane.GetIntersect(next_roi_lane, &slack_end_point);
    }
    inner_roi_vertex->at(i - 1) = slack_start_point;
    inner_roi_vertex->at(i) = slack_end_point;
  }
}

uint32_t OpenSpaceRoiDecider::GetParkingReplanStatus(
    const perception::ParkingLotOut::ParkType& park_type,
    const ParkingLotVertexType& vertices,
    const common::PathPoint& pre_end_pose_enu,
    const common::PathPoint& end_pose_enu) {
  uint32_t replan_status = 0;
  auto path_type = planning_internal::PathUpdateStatus::DEFAULT;
  const auto& previous_frame = injector_->frame_history()->Latest();
  if (previous_frame != nullptr &&
      previous_frame->local_view().HasFunctionManagerIn() &&
      previous_frame->local_view()
              .GetFunctionManagerIn()
              ->fct_avp_in()
              .sys_run_state() != functionmanager::AvpFctIn::PARKSTART) {
    path_type = previous_frame->open_space_info().partitioned_paths().path_type;
  } else if (frame_->open_space_info()
                 .open_space_path_info()
                 .trace_path.Length() > kEpsilon) {
    path_type = planning_internal::PathUpdateStatus::TRACE_PATH;
  }
  if (path_type == planning_internal::PathUpdateStatus::TRACE_PATH) {
    replan_status += static_cast<uint32_t>(OpenSpaceStatus::TRACE_REPLAN);
  }
  if (!pre_end_pose_enu.has_x() || !pre_end_pose_enu.has_y() ||
      !pre_end_pose_enu.has_theta()) {
    replan_status += static_cast<uint32_t>(OpenSpaceStatus::TARGET_UPDATE);
    return replan_status;
  }
  if (nullptr != previous_frame) {
    common::PathPoint adc_point;
    adc_point.set_x(vehicle_state_.x());
    adc_point.set_y(vehicle_state_.y());
    adc_point.set_theta(vehicle_state_.heading());
    const auto bottom_dist =
        CalculateAdcDis2ParkBottom(adc_point, {vertices[1], vertices[2]});
    frame_->mutable_open_space_info()->set_adc_bottom_dist(bottom_dist);
    const auto& parking_scenario_type =
        previous_frame->open_space_info()
            .open_space_path_info()
            .open_space_env_structured_info.parking_scenario_type;
    const bool is_lat_park_in =
        parking_scenario_type == ParkingScenarioType::LEFT_LATERAL_PARKING_IN ||
        parking_scenario_type == ParkingScenarioType::RIGHT_LATERAL_PARKING_IN;
    if (is_lat_park_in && !is_entered_lateral_slot_domain_ &&
        frame_->open_space_info().entered_lateral_parking_slot(false)) {
      ADEBUG << "lateral park in scenario, adc enter parking domain now, "
                "trigger replan";
      is_entered_lateral_slot_domain_ = true;
      replan_status +=
          static_cast<uint32_t>(OpenSpaceStatus::ENTER_SPECIAL_DOMAIN);
    }
  }
  double lon_dis_threshold(0);
  double lat_dis_threshold(0);
  double angle_threshold(0);
  EndPoseThresholdManager(park_type, vertices, &lon_dis_threshold,
                          &lat_dis_threshold, &angle_threshold);
  const bool is_lat_spot =
      (park_type == TL::perception::ParkingLotOut::LATERAL);
  const bool is_pose_diff = IsTargetPoseDiffLargerThanThreshold(
      end_pose_enu, pre_end_pose_enu, lon_dis_threshold, lat_dis_threshold,
      angle_threshold, is_lat_spot);
  if (path_type == planning_internal::PathUpdateStatus::GEOMETRY_ADJUST) {
    ADEBUG << "disable target update in current frame when path_type = "
           << path_type;
  } else if (is_pose_diff) {
    const bool is_lat_spot =
        (park_type == TL::perception::ParkingLotOut::LATERAL);
    AttitudeParameters standard_attitude_params =
        is_lat_spot ? config_.open_space_roi_decider_config()
                          .lateral_park_in_target_thresold()
                    : config_.open_space_roi_decider_config()
                          .vertical_park_in_target_thresold();
    replan_status +=
        IsTargetPoseDiffLargerThanThreshold(
            end_pose_enu, pre_end_pose_enu, standard_attitude_params.vertical(),
            standard_attitude_params.horizontal(),
            standard_attitude_params.angle(), is_lat_spot)
            ? static_cast<uint32_t>(OpenSpaceStatus::TARGET_UPDATE)
            : static_cast<uint32_t>(OpenSpaceStatus::TARGET_UPDATE_SLIGHTLY);
  }
  return replan_status;
}

uint32_t OpenSpaceRoiDecider::GetNNSAdjustReplanStatus() {
  uint32_t replan_status = 0;
  if (HasNNSAdjustTraceReplan()) {
    replan_status += static_cast<uint32_t>(OpenSpaceStatus::TRACE_REPLAN);
  }
  const auto& pre_end_pose = GetPreEndPose();
  const DestRegionWithAng& dest_region_with_angle =
      frame_->open_space_info().open_space_path_info().dest_region_with_angle;
  const auto& destregion_polygon = std::get<0>(dest_region_with_angle);
  const double destregion_fromangle = std::get<1>(dest_region_with_angle);
  const double destregion_toangle = std::get<2>(dest_region_with_angle);
  const bool target_pose_changed =
      !destregion_polygon.IsPointIn(
          Vec2d(pre_end_pose.x(), pre_end_pose.y())) ||
      !common::math::AngleInRange(pre_end_pose.theta(), destregion_fromangle,
                                  destregion_toangle);
  if (frame_->open_space_info().is_stuck_roi_updated() || target_pose_changed) {
    replan_status += static_cast<uint32_t>(OpenSpaceStatus::TARGET_UPDATE);
  }

  return replan_status;
}

bool OpenSpaceRoiDecider::HasNNSAdjustTraceReplan() {
  if (frame_->open_space_info().open_space_path_info().trace_path.Length() <
      kEpsilon) {
    return false;
  }
  const auto& previous_frame = injector_->frame_history()->Latest();
  if (nullptr == previous_frame) {
    return true;
  }
  const auto& pre_open_space_info = previous_frame->open_space_info();
  return !pre_open_space_info.is_on_open_space_trajectory() ||
         pre_open_space_info.partitioned_paths().path_type ==
             planning_internal::PathUpdateStatus::TRACE_PATH;
}

void OpenSpaceRoiDecider::EndPoseThresholdManager(
    const perception::ParkingLotOut::ParkType& park_type,
    const ParkingLotVertexType& vertices, double* const lon_threhold,
    double* const lat_threhold, double* const angle_threhold) {
  if (nullptr == lon_threhold || nullptr == lat_threhold ||
      nullptr == angle_threhold) {
    return;
  }
  // if park in
  if (AVPStatus::PARKING_IN != parking_type_) {
    return;
  }
  const auto& open_space_config = config_.open_space_roi_decider_config();
  const bool is_reach_precise_target = injector_->planning_context()
                                           ->planning_status()
                                           .open_space()
                                           .is_reach_precise_target();
  const bool is_lat_spot =
      (park_type == TL::perception::ParkingLotOut::LATERAL);
  // init
  AttitudeParameters attitude_params =
      is_lat_spot ? open_space_config.lateral_park_in_target_thresold()
                  : open_space_config.vertical_park_in_target_thresold();
  if (!is_reach_precise_target) {
    attitude_params = is_lat_spot
                          ? open_space_config.park_in_lateral_end_pose_relax()
                          : open_space_config.park_in_vertical_end_pose_relax();
  } else {
    ParkLotAttitudeConfTable park_lot_attitude_conf_table;
    if (is_lat_spot &&
        open_space_config.has_lateral_park_lot_attitude_conf_table()) {
      park_lot_attitude_conf_table =
          open_space_config.lateral_park_lot_attitude_conf_table();
    }
    if (!is_lat_spot &&
        open_space_config.has_vertical_park_lot_attitude_conf_table()) {
      park_lot_attitude_conf_table =
          open_space_config.vertical_park_lot_attitude_conf_table();
    }
    if (!park_lot_attitude_conf_table.park_lot_size().empty()) {
      // evaluate thresold based on park lot size
      target_slot_witdh_ =
          std::min(target_slot_witdh_, (vertices[0] - vertices[3]).Length());
      double lat = common::math::InterpolationOne(
          target_slot_witdh_, park_lot_attitude_conf_table.park_lot_size(),
          park_lot_attitude_conf_table.horizontal());
      double lon = common::math::InterpolationOne(
          target_slot_witdh_, park_lot_attitude_conf_table.park_lot_size(),
          park_lot_attitude_conf_table.vertical());
      double ang = common::math::InterpolationOne(
          target_slot_witdh_, park_lot_attitude_conf_table.park_lot_size(),
          park_lot_attitude_conf_table.angle());
      attitude_params.set_horizontal(lat);
      attitude_params.set_vertical(lon);
      attitude_params.set_angle(ang);
    }
  }
  *lon_threhold = attitude_params.vertical();
  *lat_threhold = attitude_params.horizontal();
  *angle_threhold = attitude_params.angle();
  ADEBUG << "attitude_params " << attitude_params.DebugString();
}

void OpenSpaceRoiDecider::CaculateDestRegion(
    Frame* const frame, const common::PathPoint& end_pose_enu,
    const perception::ParkingLotOut::ParkType& park_type,
    const ParkingLotVertexType& vertices,
    DestRegionWithAng* const dest_region_with_angle) {
  if (nullptr == dest_region_with_angle) {
    AERROR << "dest_region_with_angle is nullptr";
    return;
  }
  const auto& roi_conf = config_.open_space_roi_decider_config();
  switch (parking_type_) {
    case TL::planning::AVPStatus::PARKING_IN: {
      CalculateParkingInRegion(park_type, end_pose_enu, dest_region_with_angle);
      break;
    }
    case TL::planning::AVPStatus::PARKING_OUT_LEFT: {
      double min_lat_dis =
          park_type == TL::perception::ParkingLotOut::VERTICAL
              ? roi_conf.vertical_park_out_left_attitude().min_lat_dis() +
                    vehicle_params_.left_edge_to_center()
              : roi_conf.lateral_park_out_left_attitude().min_lat_dis() +
                    vehicle_params_.left_edge_to_center();
      double max_lat_dis =
          park_type == TL::perception::ParkingLotOut::VERTICAL
              ? roi_conf.vertical_park_out_left_attitude().max_lat_dis() +
                    vehicle_params_.left_edge_to_center()
              : roi_conf.lateral_park_out_left_attitude().max_lat_dis() +
                    vehicle_params_.left_edge_to_center();
      CalculateParkingOutRegion(frame, min_lat_dis, max_lat_dis, end_pose_enu,
                                vertices, dest_region_with_angle);
      break;
    }
    case TL::planning::AVPStatus::PARKING_OUT_RIGHT: {
      double min_lat_dis =
          park_type == TL::perception::ParkingLotOut::VERTICAL
              ? roi_conf.vertical_park_out_right_attitude().min_lat_dis() +
                    vehicle_params_.left_edge_to_center()
              : roi_conf.lateral_park_out_right_attitude().min_lat_dis() +
                    vehicle_params_.left_edge_to_center();
      double max_lat_dis =
          park_type == TL::perception::ParkingLotOut::VERTICAL
              ? roi_conf.vertical_park_out_right_attitude().max_lat_dis() +
                    vehicle_params_.left_edge_to_center()
              : roi_conf.lateral_park_out_right_attitude().max_lat_dis() +
                    vehicle_params_.left_edge_to_center();
      // slack base on road
      CalculateParkingOutRegion(frame, min_lat_dis, max_lat_dis, end_pose_enu,
                                vertices, dest_region_with_angle);
      break;
    }
    case TL::planning::AVPStatus::PARKING_OUT_NNS: {
      double min_lat_dis = 0.0;
      double max_lat_dis = INFINITY;
      // slack base on road
      CalculateParkingOutRegion(frame, min_lat_dis, max_lat_dis, end_pose_enu,
                                vertices, dest_region_with_angle);
      break;
    }
    case TL::planning::AVPStatus::PARKING_OUT_FRONT:
    case TL::planning::AVPStatus::PARKING_OUT_BACK: {
      // slack with default value
      std::vector<Vec2d> dest_region_polygon_points;
      static constexpr double kRegionBuffer = 0.01;
      dest_region_polygon_points.emplace_back(end_pose_enu.x() - kRegionBuffer,
                                              end_pose_enu.y() - kRegionBuffer);
      dest_region_polygon_points.emplace_back(end_pose_enu.x() - kRegionBuffer,
                                              end_pose_enu.y() + kRegionBuffer);
      dest_region_polygon_points.emplace_back(end_pose_enu.x() + kRegionBuffer,
                                              end_pose_enu.y() + kRegionBuffer);
      dest_region_polygon_points.emplace_back(end_pose_enu.x() + kRegionBuffer,
                                              end_pose_enu.y() - kRegionBuffer);
      *dest_region_with_angle = {
          common::math::Polygon2d(dest_region_polygon_points),
          end_pose_enu.theta() - kRegionBuffer,
          end_pose_enu.theta() + kRegionBuffer};
      break;
    }
    case TL::planning::AVPStatus::NNS_ADJUST: {
      ADEBUG << "NNS_ADJUST dest region is set already";
      break;
    }
    default:
      break;
  }
}

void OpenSpaceRoiDecider::CalculateParkingInRegion(
    const perception::ParkingLotOut::ParkType& park_type,
    const common::PathPoint& end_pose_enu,
    DestRegionWithAng* const dest_region_with_angle) {
  const bool is_lat_spot =
      (park_type == TL::perception::ParkingLotOut::LATERAL);
  const auto& end_pose_relax = is_lat_spot
                                   ? config_.open_space_roi_decider_config()
                                         .park_in_lateral_end_pose_relax()
                                   : config_.open_space_roi_decider_config()
                                         .park_in_vertical_end_pose_relax();
  const double end_pose_relax_box_length = end_pose_relax.vertical();
  const double end_pose_relax_box_width = end_pose_relax.horizontal();
  const double theta_relax = end_pose_relax.angle();
  std::get<0>(*dest_region_with_angle) =
      common::math::Polygon2d(common::math::Box2d(
          Vec2d(end_pose_enu.x(), end_pose_enu.y()), end_pose_enu.theta(),
          end_pose_relax_box_length, end_pose_relax_box_width));
  std::get<1>(*dest_region_with_angle) = end_pose_enu.theta() - theta_relax;
  std::get<2>(*dest_region_with_angle) = end_pose_enu.theta() + theta_relax;
}

void OpenSpaceRoiDecider::CalculateParkingOutRegion(
    Frame* const frame, const double min_lat_dis, const double max_lat_dis,
    const common::PathPoint& end_pose_enu, const ParkingLotVertexType& vertices,
    DestRegionWithAng* const dest_region_with_angle) {
  const auto& reference_line =
      frame->reference_line_info().front().reference_line();
  common::SLPoint target_sl;
  reference_line.XYToSL({end_pose_enu.x(), end_pose_enu.y()}, &target_sl);
  common::SLPoint top_edge_middle_point_sl;
  reference_line.XYToSL((vertices[0] + vertices[3]) * 0.5,
                        &top_edge_middle_point_sl);

  const auto& nearby_path =
      frame->reference_line_info().front().reference_line().GetMapPath();
  double s1 = std::min(top_edge_middle_point_sl.s(),
                       target_sl.s() - FLAGS_park_out_distance);
  double s2 = std::max(top_edge_middle_point_sl.s(),
                       target_sl.s() + FLAGS_park_out_distance);
  double l1 = std::min(
      nearby_path.GetLaneLeftWidth(s1) - vehicle_params_.left_edge_to_center(),
      top_edge_middle_point_sl.l() + max_lat_dis);
  double l2 = std::max(-1 * nearby_path.GetLaneRightWidth(s2),
                       top_edge_middle_point_sl.l() + min_lat_dis);
  std::vector<common::SLPoint> boundary_sl(4);
  boundary_sl[0].set_s(s1);
  boundary_sl[0].set_l(l1);
  boundary_sl[1].set_s(s1);
  boundary_sl[1].set_l(l2);
  boundary_sl[2].set_s(s2);
  boundary_sl[2].set_l(l2);
  boundary_sl[3].set_s(s2);
  boundary_sl[3].set_l(l1);

  std::vector<Vec2d> parking_out_region_points(boundary_sl.size());

  for (size_t i = 0; i < boundary_sl.size(); ++i) {
    reference_line.SLToXY(boundary_sl[i], &parking_out_region_points[i]);
  }
  auto dest_region = common::math::Polygon2d(parking_out_region_points);
  const auto end_pose_enu_vec = Vec2d(end_pose_enu.x(), end_pose_enu.y());
  const auto unit_s_vec = Vec2d::CreateUnitVec2d(end_pose_enu.theta());
  const auto unit_l_vec = Vec2d::CreateUnitVec2d(end_pose_enu.theta() - M_PI_2);
  // decide if end pose heading is parallel to reference line
  if (std::fabs(dest_region.line_segments().at(1).unit_direction().CrossProd(
          unit_s_vec)) >
      sin(config_.open_space_roi_decider_config().park_out_angle_limit())) {
    parking_out_region_points[0] = end_pose_enu_vec +
                                   FLAGS_park_out_distance * unit_s_vec +
                                   l1 * unit_l_vec;
    parking_out_region_points[1] = end_pose_enu_vec +
                                   FLAGS_park_out_distance * unit_s_vec -
                                   l2 * unit_l_vec;
    parking_out_region_points[2] = end_pose_enu_vec -
                                   FLAGS_park_out_distance * unit_s_vec -
                                   l2 * unit_l_vec;
    parking_out_region_points[3] = end_pose_enu_vec -
                                   FLAGS_park_out_distance * unit_s_vec +
                                   l1 * unit_l_vec;
  }
  std::get<0>(*dest_region_with_angle) =
      common::math::Polygon2d(parking_out_region_points);
  std::get<1>(*dest_region_with_angle) =
      end_pose_enu.theta() -
      config_.open_space_roi_decider_config().park_out_angle_limit();
  std::get<2>(*dest_region_with_angle) =
      end_pose_enu.theta() +
      config_.open_space_roi_decider_config().park_out_angle_limit();
}

// decide parking  scenario type
void OpenSpaceRoiDecider::ParkingScenarioTypeDecision(
    const bool is_right_side,
    const perception::ParkingLotOut::ParkType& park_type,
    ParkingScenarioType* const parking_scenario_type_ptr) {
  if (nullptr == parking_scenario_type_ptr) {
    return;
  }
  *parking_scenario_type_ptr = ParkingScenarioType::DEFAULT_TYPE;
  switch (parking_type_) {
    case TL::planning::AVPStatus_ParkingType_PARKING_IN: {
      // decide based on left or right
      switch (park_type) {
        case TL::perception::ParkingLotOut_ParkType_VERTICAL: {
          *parking_scenario_type_ptr =
              is_right_side ? ParkingScenarioType::RIGHT_VERTICAL_PARKING_IN
                            : ParkingScenarioType::LEFT_VERTICAL_PARKING_IN;
          break;
        }
        case TL::perception::ParkingLotOut_ParkType_OBLIQUE: {
          *parking_scenario_type_ptr =
              is_right_side ? ParkingScenarioType::RIGHT_OBLIQUE_PARKING_IN
                            : ParkingScenarioType::LEFT_OBLIQUE_PARKING_IN;
          break;
        }
        case TL::perception::ParkingLotOut_ParkType_LATERAL: {
          *parking_scenario_type_ptr =
              is_right_side ? ParkingScenarioType::RIGHT_LATERAL_PARKING_IN
                            : ParkingScenarioType::LEFT_LATERAL_PARKING_IN;
          break;
        }
        default: {
          *parking_scenario_type_ptr = ParkingScenarioType::DEFAULT_TYPE;
          break;
        }
      }
      break;
    }
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_LEFT: {
      switch (park_type) {
        case TL::perception::ParkingLotOut_ParkType_VERTICAL: {
          *parking_scenario_type_ptr =
              ParkingScenarioType::LEFT_VERTICAL_PARKING_OUT;
          break;
        }
        case TL::perception::ParkingLotOut_ParkType_OBLIQUE: {
          *parking_scenario_type_ptr =
              ParkingScenarioType::LEFT_OBLIQUE_PARKING_OUT;
          break;
        }
        case TL::perception::ParkingLotOut_ParkType_LATERAL: {
          *parking_scenario_type_ptr =
              ParkingScenarioType::LEFT_LATERAL_PARKING_OUT;
          break;
        }
        default:
          *parking_scenario_type_ptr = ParkingScenarioType::DEFAULT_TYPE;
          break;
      }
      break;
    }
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_RIGHT: {
      switch (park_type) {
        case TL::perception::ParkingLotOut_ParkType_VERTICAL: {
          *parking_scenario_type_ptr =
              ParkingScenarioType::RIGHT_VERTICAL_PARKING_OUT;
          break;
        }
        case TL::perception::ParkingLotOut_ParkType_OBLIQUE: {
          *parking_scenario_type_ptr =
              ParkingScenarioType::RIGHT_OBLIQUE_PARKING_OUT;
          break;
        }
        case TL::perception::ParkingLotOut_ParkType_LATERAL: {
          *parking_scenario_type_ptr =
              ParkingScenarioType::RIGHT_LATERAL_PARKING_OUT;
          break;
        }
        default:
          *parking_scenario_type_ptr = ParkingScenarioType::DEFAULT_TYPE;
          break;
      }
      break;
    }
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_FRONT: {
      switch (park_type) {
        case TL::perception::ParkingLotOut_ParkType_VERTICAL: {
          *parking_scenario_type_ptr =
              ParkingScenarioType::FORWARD_VERTICAL_PARKING_OUT;
          break;
        }
        case TL::perception::ParkingLotOut_ParkType_OBLIQUE: {
          *parking_scenario_type_ptr =
              ParkingScenarioType::FORWARD_OBLIQUE_PARKING_OUT;
          break;
        }
        default:
          *parking_scenario_type_ptr = ParkingScenarioType::DEFAULT_TYPE;
          break;
      }
      break;
    }
    case TL::planning::AVPStatus_ParkingType_PARKING_OUT_BACK: {
      switch (park_type) {
        case TL::perception::ParkingLotOut_ParkType_VERTICAL: {
          *parking_scenario_type_ptr =
              ParkingScenarioType::BACKWARD_VERTICAL_PARKING_OUT;
          break;
        }
        case TL::perception::ParkingLotOut_ParkType_OBLIQUE: {
          *parking_scenario_type_ptr =
              ParkingScenarioType::BACKWARD_OBLIQUE_PARKING_OUT;
          break;
        }
        default:
          *parking_scenario_type_ptr = ParkingScenarioType::DEFAULT_TYPE;
          break;
      }
      break;
    }
    case TL::planning::AVPStatus_ParkingType_TEST_CONTROL_MODE: {
      *parking_scenario_type_ptr =
          ParkingScenarioType::CONTROL_CALIBRATION_MODE;
      break;
    }
    default: {
      *parking_scenario_type_ptr = ParkingScenarioType::DEFAULT_TYPE;
      break;
    }
  }
}

void OpenSpaceRoiDecider::SensorStateDecider() {
  if (sensor_config_state_ != SensorConfigState::NONINITAILZED) {
    return;
  }
#ifdef FOR_BAIDU_SIMULATION
  sensor_config_state_ = SensorConfigState::LIDAR;
  return;
#endif
  if (!frame_->local_view().HasFreeSpaceOutArray()) {
    ADEBUG << "perception has no free space output, use only uss";
    sensor_config_state_ = SensorConfigState::USS;
    return;
  }
  if (frame_->local_view().GetFreeSpaceOutArray()->freespace_out().empty()) {
    ADEBUG << "perception has free space output, but has no freespace obstacle";
    sensor_config_state_ = SensorConfigState::CAMERA;
    return;
  }
  sensor_config_state_ = SensorConfigState::CAMERA;
  if (!config_.open_space_roi_decider_config()
           .enable_adapt_lidar_sensor_roi()) {
    ADEBUG << " lida roi is disabled";
    return;
  }
  for (const auto& free_space_item :
       frame_->local_view().GetFreeSpaceOutArray()->freespace_out()) {
    if (free_space_item.has_sensor_type() &&
        (free_space_item.sensor_type() == perception::FreeSpaceOut::LIDAR ||
         free_space_item.sensor_type() ==
             perception::FreeSpaceOut::LIDAR_CAM_FUSION ||
         free_space_item.sensor_type() ==
             perception::FreeSpaceOut::USS_LIDAR_CAM_FUSION)) {
      ADEBUG << "freespace has lidar output";
      sensor_config_state_ = SensorConfigState::LIDAR;
      return;
    }
  }
}

common::PathPoint OpenSpaceRoiDecider::GetPreEndPose() {
  common::PathPoint previous_end_pose_enu;
  const auto& previous_frame = injector_->frame_history()->Latest();
  if (nullptr == previous_frame) {
    return previous_end_pose_enu;
  }

  const auto& pre_partitioned_paths =
      previous_frame->open_space_info().partitioned_paths();
  const bool is_apa_path =
      pre_partitioned_paths.path_type !=
          planning_internal::PathUpdateStatus::TRACE_PATH &&
      pre_partitioned_paths.path_type !=
          planning_internal::PathUpdateStatus::CRUISE_PATH;
  if (!is_apa_path) {
    return previous_end_pose_enu;
  }

  const auto& pre_open_space_path_info_map =
      previous_frame->open_space_info().open_space_path_info_map();
  const auto& pre_open_space_path_info_id =
      previous_frame->open_space_info().open_space_path_info_id();
  if (pre_open_space_path_info_map.find(pre_open_space_path_info_id) ==
      pre_open_space_path_info_map.end()) {
    AERROR << "can not find previous path info";
    return previous_end_pose_enu;
  }
  // default value
  previous_end_pose_enu =
      pre_open_space_path_info_map.at(pre_open_space_path_info_id).end_point;
  if (TL::planning::AVPStatus::PARKING_IN == parking_type_ ||
      TL::planning::AVPStatus::NNS_ADJUST == parking_type_) {
    if (previous_frame->open_space_info().is_partitioned_paths_valid()) {
      previous_end_pose_enu =
          pre_partitioned_paths.path_set.back().first.back();
    }
  }
  return previous_end_pose_enu;
}

bool OpenSpaceRoiDecider::IsTargetPoseDiffLargerThanThreshold(
    const common::PathPoint& p_a, const common::PathPoint& p_b,
    const double lon_dis_threshold, const double lat_dis_threshold,
    const double angle_thresold, const bool is_lat_spot) {
  const auto pa_to_pb = Vec2d(p_b.x() - p_a.x(), p_b.y() - p_a.y());
  const auto unit_vec = Vec2d::CreateUnitVec2d(p_a.theta());
  const double inner_prod = unit_vec.InnerProd(pa_to_pb);
  const double cross_prod = unit_vec.CrossProd(pa_to_pb);
  end_pose_lon_error_ = is_lat_spot ? cross_prod : inner_prod;
  // left: positive; right: negative
  end_pose_lat_error_ = is_lat_spot ? inner_prod : cross_prod;
  end_pose_yaw_error_ = common::math::NormalizeAngle(p_a.theta() - p_b.theta());
  ADEBUG << "end_pose_lat_error =  " << end_pose_lat_error_
         << " ; end_pose_lon_error = " << end_pose_lon_error_
         << " ; end_pose_yaw_error = " << end_pose_yaw_error_
         << " ; lon_dis_threshold " << lon_dis_threshold
         << " ; lat_dis_threshold " << lat_dis_threshold << " ; angle_thresold "
         << angle_thresold << " front rear lat error "
         << end_pose_lat_error_ -
                vehicle_params_.wheel_base() * end_pose_yaw_error_;
  return fabs(end_pose_lon_error_) > lon_dis_threshold ||
         fabs(end_pose_lat_error_) > lat_dis_threshold ||
         fabs(end_pose_yaw_error_) > angle_thresold ||
         fabs(end_pose_lat_error_ -
              vehicle_params_.wheel_base() * end_pose_yaw_error_) >
             lat_dis_threshold;
}

}  // namespace planning
}  // namespace TL
