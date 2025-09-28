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

#include "planning/tasks/deciders/path_decider/path_decider.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/match.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/math/line_segment2d.h"
#include "common/math/linear_interpolation.h"
#include "common/math/polygon2d.h"
#include "planning/common/obstacle.h"
#include "planning/common/path/path_data.h"
#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/util/common.h"
#include "planning/common/util/util.h"

#include "proto/perception/perception_freespace.pb.h"
#include "proto/planning/decision.pb.h"

namespace TL {
namespace planning {
constexpr double kStaticReverseObs = 2.0;

using TL::common::ErrorCode;
using TL::common::PathPoint;
using TL::common::Status;

PathDecider::PathDecider(const TaskConfig& config,
                         const std::shared_ptr<DependencyInjector>& injector)
    : Task(config, injector),
      path_decider_config_(config.path_decider_config()) {}

Status PathDecider::Execute(Frame* frame,
                            ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Process(reference_line_info, reference_line_info->path_data(),
                 reference_line_info->path_decision());
}

Status PathDecider::Process(const ReferenceLineInfo* reference_line_info,
                            const PathData& path_data,
                            PathDecision* const path_decision) {
#ifndef ISMDC
  if (FLAGS_enable_plot_speed_l_buffer) {
    PlotCollisionBufferBox();
  }
#endif

  // skip path_decider if reused path
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    return Status::OK();
  }

  std::string blocking_obstacle_id;
  if (reference_line_info->GetBlockingObstacle() != nullptr) {
    blocking_obstacle_id = reference_line_info->GetBlockingObstacle()->Id();
  }
  std::unordered_map<uint32_t, perception::PerceptionObstacle_Type>
      filter_box_id_type;
  filter_box_id_type.clear();
  if (nullptr != path_decision) {
    MakeObjectFilterDecision(*path_decision, &filter_box_id_type);
  }

  if (!MakeObjectDecision(path_data, blocking_obstacle_id, path_decision,
                          filter_box_id_type)) {
    const std::string msg = "Failed to make decision based on tunnel";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_PATHDECIDER_ERROR, msg);
  }

  MakeFreeSpaceSegmentDecision(path_decision);
  // AERROR<<"path_label:"<<reference_line_info->path_data().path_label()
  // << "
  // path_end_s:"<<reference_line_info->path_data().discretized_path().back().s();
  return Status::OK();
}

void PathDecider::MakeObjectFilterDecision(
    const PathDecision& path_decision,
    std::unordered_map<uint32_t, perception::PerceptionObstacle_Type>* const
        filter_box_id_type) {
  if (nullptr == reference_line_info_ || nullptr == frame_ ||
      nullptr == filter_box_id_type ||
      path_decision.obstacles().Items().empty()) {
    AERROR << "path_decision etc input is nullptr!";
    return;
  }

  if (!frame_->local_view().HasVehicleState() ||
      !frame_->PlanningStartPoint().has_path_point()) {
    AERROR << "no planning start point.";
    return;
  }
  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  auto* freespace_segments =
      reference_line_info_->mutable_path_data()->MutableFreeSpaceSegments();
  freespace_segments->clear();
  std::unordered_map<uint32_t, perception::FreeSpaceOut::ClassType>
      fs_segment_linked_box_id_type;
  if (frame_->local_view().HasFreeSpaceOutArray()) {
    util::UpdateFreeSpaceSegmentsByPath(
        discretized_path, frame_->local_view().GetFreeSpaceOutArray(),
        freespace_segments);
    for (const auto& segment_info : *freespace_segments) {
      if (segment_info.isLinkObjFusion) {
        fs_segment_linked_box_id_type.insert(
            std::make_pair(segment_info.obstacleId, segment_info.cls_type));
      }
    }
  }

  if (fs_segment_linked_box_id_type.empty()) {
    return;
  }
  const auto& start_point = frame_->PlanningStartPoint().path_point();
  const auto adc_box = common::VehicleConfigHelper::GetBoundingBox(
      start_point,
      path_decider_config_.ntp_cruise_freespace_ego_box_fillter_buffer(),
      path_decider_config_.ntp_cruise_freespace_ego_box_fillter_buffer());

  for (const auto* const obstacle : path_decision.obstacles().Items()) {
    const auto& sl_boundary = obstacle->PerceptionSLBoundary();
    if (sl_boundary.end_s() <
            reference_line_info_->AdcSlBoundary().start_s() -
                path_decider_config_
                    .ntp_cruise_freespace_ego_box_fillter_buffer() ||
        sl_boundary.start_s() >
            reference_line_info_->AdcSlBoundary().end_s() +
                path_decider_config_
                    .ntp_cruise_freespace_ego_box_fillter_buffer()) {
      continue;
    }
    const auto iter =
        fs_segment_linked_box_id_type.find(obstacle->Perception().id());
    if (obstacle->IsStatic() && iter != fs_segment_linked_box_id_type.end() &&
        obstacle->PerceptionBoundingBox().HasOverlap(adc_box)) {
      filter_box_id_type->insert(std::make_pair(obstacle->Perception().id(),
                                                obstacle->Perception().type()));
    }
  }
}

bool PathDecider::MakeObjectDecision(
    const PathData& path_data, const std::string& blocking_obstacle_id,
    PathDecision* const path_decision,
    const std::unordered_map<uint32_t, perception::PerceptionObstacle_Type>&
        filter_box_id_type) {
  if (!MakeStaticObstacleDecision(path_data, blocking_obstacle_id,
                                  path_decision, filter_box_id_type)) {
    AERROR << "Failed to make decisions for static obstacles";
    return false;
  }
  return true;
}

// TODO(jiacheng): eventually this entire "path_decider" should be retired.
// Before it gets retired, its logics are slightly modified so that everything
// still works well for now.
bool PathDecider::MakeStaticObstacleDecision(
    const PathData& path_data, const std::string& blocking_obstacle_id,
    PathDecision* const path_decision,
    const std::unordered_map<uint32_t, perception::PerceptionObstacle_Type>&
        filter_box_id_type) {
  // Sanity checks and get important values.
  if (path_decision == nullptr) {
    AERROR << "path_decision is nullptr!";
    return false;
  }

  const auto& frenet_path = path_data.frenet_frame_path();
  if (frenet_path.empty()) {
    AERROR << "Path is empty.";
    return false;
  }
  const double half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  const double lateral_radius = half_width + FLAGS_lateral_ignore_buffer;
  const bool is_forward_path =
      reference_line_info_->path_data().frenet_frame_path().is_forward_path();
  bool is_behind_adc_block_obs = false;
  bool is_avp_mode = false;
  if (frame_->local_view().HasFunctionManagerIn()) {
    is_avp_mode = functionmanager::AVP ==
                  frame_->local_view().GetFunctionManagerIn()->ta_pilot_mode();
  }
  // Go through every obstacle and make decisions.
  // nearest_object_decision_stop_obs_id is adc to stop.
  std::string nearest_object_decision_stop_obs_id;
  for (const auto* const obstacle : path_decision->obstacles().Items()) {
    // const std::string& obstacle_id = obstacle->Id();
    const std::string obstacle_type_name =
        PerceptionObstacle_Type_Name(obstacle->Perception().type());
    // ADEBUG << "obstacle_id[" << obstacle_id << "] type[" <<
    // obstacle_type_name
    //        << "]";
    // 行车模式下低速行人作为静止目标处理
    const auto is_nnp_ignore_pedstrian =
        !is_avp_mode &&
        obstacle->Perception().type() ==
            TL::perception::PerceptionObstacle::PEDESTRIAN &&
        obstacle->Perception().has_velocity_flu() &&
        std::hypot(obstacle->Perception().velocity_flu().x(),
                   obstacle->Perception().velocity_flu().y()) >
            kStaticReverseObs;

    // build stop wall for moving pedestrian in NTP && TBA,
    // skip virtual obstacles.
    if ((!obstacle->IsStaticWithoutIgnore() &&
         (obstacle->Perception().type() !=
              TL::perception::PerceptionObstacle::PEDESTRIAN ||
          is_nnp_ignore_pedstrian)) ||
        obstacle->IsVirtual()) {
      continue;
    }

    // - skip decision making for obstacles with IGNORE/STOP decisions already.
    if (obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_ignore() &&
        obstacle->HasLateralDecision() &&
        obstacle->LateralDecision().has_ignore()) {
      continue;
    }
    if (obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_stop()) {
      // STOP decision
      continue;
    }

    const auto iter = filter_box_id_type.find(obstacle->Perception().id());
    // ntp 下过滤自车近处的已关联fs的bbox;
    if (iter != filter_box_id_type.end() && is_avp_mode) {
      continue;
    }

    const auto& sl_boundary = obstacle->PerceptionSLBoundary();
    const auto& vehicle_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    // - add STOP decision for blocking obstacles.
    if (obstacle->Id() == blocking_obstacle_id &&
        !injector_->planning_context()
             ->planning_status()
             .path_decider()
             .is_in_path_lane_borrow_scenario()) {
      if (sl_boundary.end_s() <
          frenet_path.front().s() + vehicle_param.front_edge_to_center()) {
        is_behind_adc_block_obs = true;
      } else {
        // Add stop decision
        ADEBUG << "Blocking obstacle = " << blocking_obstacle_id;
        ObjectDecisionType object_decision;
        if (is_forward_path) {
          *object_decision.mutable_stop() = GenerateObjectStopDecision(
              *obstacle, obstacle->PerceptionSLBoundary().start_s());
        } else {
          *object_decision.mutable_stop() = GenerateObjectStopDecision(
              *obstacle, obstacle->PerceptionSLBoundary().end_s());
        }
        path_decision->AddLongitudinalDecision("PathDecider/blocking_obstacle",
                                               obstacle->Id(), object_decision);
        continue;
      }
    }
    // - skip decision making for clear-zone obstacles.
    if (obstacle->reference_line_st_boundary().boundary_type() ==
        STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    // 0. IGNORE by default and if obstacle is not in path s at all.
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();
    if ((is_forward_path &&
         (sl_boundary.end_s() <
              frenet_path.front().s() - vehicle_param.back_edge_to_center() ||
          sl_boundary.start_s() >
              frenet_path.back().s() + vehicle_param.front_edge_to_center())) ||
        (!is_forward_path &&
         (sl_boundary.end_s() <
              frenet_path.back().s() - vehicle_param.back_edge_to_center() ||
          sl_boundary.start_s() > frenet_path.front().s() +
                                      vehicle_param.front_edge_to_center()))) {
      path_decision->AddLongitudinalDecision("PathDecider/not-in-s",
                                             obstacle->Id(), object_decision);
      path_decision->AddLateralDecision("PathDecider/not-in-s", obstacle->Id(),
                                        object_decision);
      continue;
    }

    if (frame_ != nullptr && frame_->local_view().HasFunctionManagerOut() &&
        frame_->local_view().GetFunctionManagerOut() != nullptr &&
        frame_->local_view().GetFunctionManagerOut()->fsm_state() ==
            functionmanager::MachineStateType::PERCEPTION_TYPE &&
        frame_->local_view().GetFunctionManagerOut()->perception_sub_state() ==
            functionmanager::PerceptionSubState::CRUISE_TYPE &&
        (!frame_->local_view().GetCruiseTargetId().empty() &&
         std::find(frame_->local_view().GetCruiseTargetId().begin(),
                   frame_->local_view().GetCruiseTargetId().end(),
                   obstacle->PerceptionId()) !=
             frame_->local_view().GetCruiseTargetId().end())) {
      double collision_path_point_s =
          is_forward_path ? obstacle->PerceptionSLBoundary().start_s()
                          : obstacle->PerceptionSLBoundary().end_s();
      ObjectDecisionType object_decision;
      *object_decision.mutable_stop() =
          GenerateObjectStopDecision(*obstacle, collision_path_point_s);

      if (path_decision->MergeWithMainStop(
              object_decision.stop(), obstacle->Id(),
              reference_line_info_->reference_line(),
              reference_line_info_->AdcSlBoundary(), is_forward_path)) {
        nearest_object_decision_stop_obs_id = obstacle->Id();
        path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                               obstacle->Id(), object_decision);
      } else {
        ObjectDecisionType object_decision;
        object_decision.mutable_ignore();
        path_decision->AddLongitudinalDecision("PathDecider/not-nearest-stop",
                                               obstacle->Id(), object_decision);
      }
      continue;
    }

    const auto& frenet_point = frenet_path.GetNearestPoint(sl_boundary);
    const double curr_l = frenet_point.l();
    // const double min_nudge_l =
    // half_width + config_.path_decider_config().static_obstacle_buffer() / 2;
    if (curr_l - lateral_radius > sl_boundary.end_l() ||
        curr_l + lateral_radius < sl_boundary.start_l()) {
      // 1. IGNORE if laterally too far away.
      path_decision->AddLateralDecision("PathDecider/not-in-l", obstacle->Id(),
                                        object_decision);
    } else if (absl::StrContains(path_data.path_label(), "fallback") ||
               path_data.is_beyond_bound_optimize_result() || is_avp_mode ||
               is_behind_adc_block_obs) {
      size_t collision_path_point_index = 0;
      double collision_path_point_kappa_abs_max = 0.0;
      if (IsADCOverlappingWithObstacle(*obstacle,
                                       &collision_path_point_index)) {
        // 1.5 AVP mode using precise collision test
        if (is_avp_mode && !IsADCOverlappingWithObstaclePrecise(
                               *obstacle, &collision_path_point_index)) {
          continue;
        }

        // 2. STOP if laterally too overlapping.
        double collision_path_point_s = reference_line_info_->path_data()
                                            .frenet_frame_path()
                                            .at(collision_path_point_index)
                                            .s();
        if (!reference_line_info_->path_data()
                 .frenet_frame_path()
                 .is_forward_path()) {
          collision_path_point_s -= vehicle_param.back_edge_to_center();
        } else {
          collision_path_point_s += vehicle_param.front_edge_to_center();
        }
        size_t start_range_index =
            (collision_path_point_index >
             path_decider_config_.path_point_find_max_kappa_range())
                ? (collision_path_point_index -
                   path_decider_config_.path_point_find_max_kappa_range())
                : 0;
        size_t end_range_index = std::min(
            collision_path_point_index +
                path_decider_config_.path_point_find_max_kappa_range(),
            reference_line_info_->path_data().discretized_path().size() - 1);

        for (size_t i = start_range_index; i <= end_range_index; i++) {
          collision_path_point_kappa_abs_max =
              std::max(collision_path_point_kappa_abs_max,
                       std::abs(reference_line_info_->path_data()
                                    .discretized_path()
                                    .at(i)
                                    .kappa()));
        }

        // for nnp/adas, use obstacle start_s/end_s
        if (frame_ != nullptr && frame_->local_view().HasFunctionManagerIn() &&
            frame_->local_view().GetFunctionManagerIn()->ta_pilot_mode() !=
                functionmanager::AVP) {
          collision_path_point_s =
              is_forward_path ? obstacle->PerceptionSLBoundary().start_s()
                              : obstacle->PerceptionSLBoundary().end_s();
        }

        *object_decision.mutable_stop() =
            GenerateObjectStopDecision(*obstacle, collision_path_point_s,
                                       &collision_path_point_kappa_abs_max);

        if (path_decision->MergeWithMainStop(
                object_decision.stop(), obstacle->Id(),
                reference_line_info_->reference_line(),
                reference_line_info_->AdcSlBoundary(), is_forward_path)) {
          nearest_object_decision_stop_obs_id = obstacle->Id();
          path_decision->AddLongitudinalDecision(
              "PathDecider/nearest-stop", obstacle->Id(), object_decision);
        } else {
          ObjectDecisionType object_decision;
          object_decision.mutable_ignore();
          path_decision->AddLongitudinalDecision(
              "PathDecider/not-nearest-stop", obstacle->Id(), object_decision);
        }
      }
    }
  }

  // process obstacles with decider tags [PathDecider/nearest-stop] and avoid
  // planning error.
  if (!nearest_object_decision_stop_obs_id.empty()) {
    for (const auto* const obstacle : path_decision->obstacles().Items()) {
      if (obstacle->Id() != nearest_object_decision_stop_obs_id &&
          obstacle->HasLongitudinalDecision() &&
          obstacle->LongitudinalDecision().has_stop() &&
          std::find(
              obstacle->decider_tags().begin(), obstacle->decider_tags().end(),
              "PathDecider/nearest-stop") != obstacle->decider_tags().end()) {
        ObjectDecisionType object_decision;
        object_decision.mutable_ignore();
        path_decision->AddLongitudinalDecision("PathDecider/not-nearest-stop",
                                               obstacle->Id(), object_decision);
        continue;
      }
    }
  }
  return true;
}

bool PathDecider::IsADCOverlappingWithObstacle(
    const Obstacle& obs, size_t* const collision_path_point_index) const {
  bool is_avp_mode = false;
  if (frame_->local_view().HasFunctionManagerIn()) {
    is_avp_mode = functionmanager::AVP ==
                  frame_->local_view().GetFunctionManagerIn()->ta_pilot_mode();
  }

  const double lateral_buffer = GetADCLBuffer(obs, is_avp_mode);
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  size_t i_path_point = 0;
  for (const PathPoint& adc_path_point :
       reference_line_info_->path_data().discretized_path()) {
    // Convert reference point from center of rear axis to center of ADC.
    double left_box_big_buffer = lateral_buffer;
    double right_box_big_buffer = lateral_buffer;
    Vec2d ego_center_map_frame((vehicle_param.front_edge_to_center() -
                                vehicle_param.back_edge_to_center()) /
                                   2,
                               (vehicle_param.left_edge_to_center() -
                                vehicle_param.right_edge_to_center()) /
                                   2);
    ego_center_map_frame.SelfRotate(adc_path_point.theta());
    ego_center_map_frame.set_x(ego_center_map_frame.x() + adc_path_point.x());
    ego_center_map_frame.set_y(ego_center_map_frame.y() + adc_path_point.y());
    double box_width =
        vehicle_param.width() + left_box_big_buffer + right_box_big_buffer;
    if (is_avp_mode &&
        reference_line_info_->path_data()
            .frenet_frame_path()
            .is_forward_path() &&
        fabs(adc_path_point.kappa()) >
            path_decider_config_.ntp_use_big_buffer_kappa()) {
      if (adc_path_point.kappa() > 0.0) {
        right_box_big_buffer =
            path_decider_config_.ntp_cruise_box_collision_buffer_in_big_kappa();
      } else if (adc_path_point.kappa() < 0.0) {
        left_box_big_buffer =
            path_decider_config_.ntp_cruise_box_collision_buffer_in_big_kappa();
      }
      double ego_center_to_big_buffer =
          (vehicle_param.width() + left_box_big_buffer + right_box_big_buffer) /
              2 -
          vehicle_param.width() / 2 - right_box_big_buffer;
      double x_tans_dis =
          ego_center_to_big_buffer * std::cos(common::math::NormalizeAngle(
                                         adc_path_point.theta() + M_PI / 2));
      double y_tans_dis =
          ego_center_to_big_buffer * std::sin(common::math::NormalizeAngle(
                                         adc_path_point.theta() + M_PI / 2));
      ego_center_map_frame.set_x(ego_center_map_frame.x() + x_tans_dis);
      ego_center_map_frame.set_y(ego_center_map_frame.y() + y_tans_dis);
      box_width =
          vehicle_param.width() + left_box_big_buffer + right_box_big_buffer;
    }

    // Compute the ADC bounding box.
    common::math::Box2d adc_box(ego_center_map_frame, adc_path_point.theta(),
                                vehicle_param.length(), box_width);

    // Check whether ADC bounding box overlaps with obstacle bounding box.
    if (obs.PerceptionBoundingBox().HasOverlap(adc_box)) {
      if (collision_path_point_index != nullptr) {
        *collision_path_point_index = i_path_point;
      }
      return true;
    }
    ++i_path_point;
  }
  return false;
}

bool PathDecider::IsADCOverlappingWithObstaclePrecise(
    const Obstacle& obs, size_t* const collision_path_point_index) const {
  if (collision_path_point_index == nullptr) {
    return false;
  }
  static constexpr double kA = 7;
  static constexpr double kB = -1.7;
  static constexpr double kMinRation = 0.0;
  double left_buffer =
      path_decider_config_.ntp_cruise_precise_collision_left_buffer();
  double right_buffer =
      path_decider_config_.ntp_cruise_precise_collision_right_buffer();
  double average_buffer = (left_buffer + right_buffer) / 2;
  double buffer_in_big_kappa =
      path_decider_config_.ntp_cruise_precise_collision_buffer_in_big_kappa();
  double front_buffer =
      path_decider_config_.ntp_cruise_precise_collision_front_buffer();
  size_t start_range_index =
      (*collision_path_point_index >
       path_decider_config_.ntp_find_collision_point_precise_range())
          ? (*collision_path_point_index -
             path_decider_config_.ntp_find_collision_point_precise_range())
          : 0;
  double lateral_diff = 0.0;
  auto* avoid_alongside_debug = reference_line_info_->mutable_debug()
                                    ->mutable_avoid_alongside_decider_info();
  if (frame_ != nullptr && frame_->local_view().HasVehicleState() &&
      frame_->PlanningStartPoint().has_path_point()) {
    double angle_diff =
        std::fabs(common::math::NormalizeAngle(
            frame_->local_view().GetVehicleState()->heading() -
            frame_->PlanningStartPoint().path_point().theta())) *
        180.0 / M_PI;
    double ratio = std::max(kMinRation, buffer_in_big_kappa - average_buffer);
    buffer_in_big_kappa =
        ratio * (1.0 / (1.0 + exp(kA + kB * angle_diff))) + average_buffer;
    Vec2d curr_to_ego = {frame_->local_view().GetVehicleState()->x() -
                             frame_->PlanningStartPoint().path_point().x(),
                         frame_->local_view().GetVehicleState()->y() -
                             frame_->PlanningStartPoint().path_point().y()};
    Vec2d path_point_unit = Vec2d::CreateUnitVec2d(
        frame_->PlanningStartPoint().path_point().theta());
    lateral_diff = path_point_unit.CrossProd(curr_to_ego);
  }

  for (size_t i = start_range_index;
       i < reference_line_info_->path_data().discretized_path().size(); i++) {
    const auto adc_path_point =
        reference_line_info_->path_data().discretized_path().at(i);
    if (fabs(adc_path_point.kappa()) >
            path_decider_config_.ntp_use_big_buffer_kappa() &&
        reference_line_info_->path_data()
            .frenet_frame_path()
            .is_forward_path()) {
      front_buffer =
          path_decider_config_
              .ntp_cruise_precise_front_collision_buffer_in_big_kappa();
      if (adc_path_point.kappa() > 0) {
        right_buffer = buffer_in_big_kappa;
      } else if (adc_path_point.kappa() < 0) {
        left_buffer = buffer_in_big_kappa;
      }
    }
    if (lateral_diff < 0) {
      right_buffer += fabs(lateral_diff);
    } else {
      left_buffer += lateral_diff;
    }

    auto precise_polygon = common::VehicleConfigHelper::GetPolygon2dWithBuffer(
        adc_path_point.x(), adc_path_point.y(), adc_path_point.theta(),
        front_buffer,
        path_decider_config_.ntp_cruise_precise_collision_rear_buffer(),
        left_buffer, right_buffer);
    if (obs.PerceptionPolygon().HasOverlap(precise_polygon)) {
      *collision_path_point_index = i;
      auto* debug_obs_box = avoid_alongside_debug->add_obs_box();
      for (const auto& point : precise_polygon.points()) {
        auto* debug_obs_point = debug_obs_box->add_point();
        debug_obs_point->set_x(point.x());
        debug_obs_point->set_y(point.y());
      }
      return true;
    }
    left_buffer =
        path_decider_config_.ntp_cruise_precise_collision_left_buffer();
    right_buffer =
        path_decider_config_.ntp_cruise_precise_collision_right_buffer();
    front_buffer =
        path_decider_config_.ntp_cruise_precise_collision_front_buffer();
  }
  return false;
}

void PathDecider::PlotCollisionBufferBox() {
  const double lateral_buffer = FLAGS_adc_l_buffer_for_static_obstacle_avp_mode;
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  //  Add fake decisioninfo to show speed l buffer in DV
  auto* avoid_alongside_debug = reference_line_info_->mutable_debug()
                                    ->mutable_avoid_alongside_decider_info();
  for (const PathPoint& adc_path_point :
       reference_line_info_->path_data().discretized_path()) {
    // Convert reference point from center of rear axis to center of ADC.
    Vec2d ego_center_map_frame((vehicle_param.front_edge_to_center() -
                                vehicle_param.back_edge_to_center()) /
                                   2,
                               (vehicle_param.left_edge_to_center() -
                                vehicle_param.right_edge_to_center()) /
                                   2);
    ego_center_map_frame.SelfRotate(adc_path_point.theta());
    ego_center_map_frame.set_x(ego_center_map_frame.x() + adc_path_point.x());
    ego_center_map_frame.set_y(ego_center_map_frame.y() + adc_path_point.y());

    // Compute the ADC bounding box.
    common::math::Box2d adc_box(ego_center_map_frame, adc_path_point.theta(),
                                vehicle_param.length(),
                                vehicle_param.width() + lateral_buffer * 2);
    // Compute the precise ADC bounding box.
    auto precise_polygon = common::VehicleConfigHelper::GetPolygon2dWithBuffer(
        adc_path_point.x(), adc_path_point.y(), adc_path_point.theta(),
        path_decider_config_.ntp_cruise_precise_collision_front_buffer(),
        path_decider_config_.ntp_cruise_precise_collision_rear_buffer(),
        path_decider_config_.ntp_cruise_precise_collision_left_buffer(),
        path_decider_config_.ntp_cruise_precise_collision_right_buffer());

    // Set ADC box in pathpoint
    auto* debug_adc_box = avoid_alongside_debug->add_adc_box();
    auto* debug_obs_box = avoid_alongside_debug->add_obs_box();
    for (const auto& point : adc_box.GetAllCorners()) {
      auto* debug_adc_point = debug_adc_box->add_point();
      debug_adc_point->set_x(point.x());
      debug_adc_point->set_y(point.y());
    }
    for (const auto& point : precise_polygon.points()) {
      auto* debug_obs_point = debug_obs_box->add_point();
      debug_obs_point->set_x(point.x());
      debug_obs_point->set_y(point.y());
    }
  }
}

ObjectStop PathDecider::GenerateObjectStopDecision(
    const Obstacle& obstacle, const double collision_edge_s,
    const double* kappa) const {
  ObjectStop object_stop;
  double stop_distance = GetStopDistance(obstacle, kappa);
  object_stop.set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
  object_stop.set_distance_s(-stop_distance);

  double stop_ref_s = collision_edge_s - stop_distance;
  if (!reference_line_info_->path_data()
           .frenet_frame_path()
           .is_forward_path()) {
    stop_ref_s =
        collision_edge_s +
        config_.path_decider_config().static_obstacle_stop_buffer_in_tba();
  }
  const auto& stop_ref_point =
      reference_line_info_->reference_line().GetReferencePoint(stop_ref_s);
  object_stop.mutable_stop_point()->set_x(stop_ref_point.x());
  object_stop.mutable_stop_point()->set_y(stop_ref_point.y());
  object_stop.set_stop_heading(stop_ref_point.heading());
  return object_stop;
}

void PathDecider::MakeFreeSpaceSegmentDecision(
    PathDecision* const path_decision) {
  if (nullptr == reference_line_info_ || nullptr == frame_ ||
      nullptr == path_decision) {
    return;
  }

  static constexpr double kAmplificationRatio = 1.2;
  static constexpr double kA = 11;
  static constexpr double kB = -1.7;
  static constexpr double kMinRation = 0.0;
  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  const auto& freespace_segments =
      reference_line_info_->path_data().GetFreeSpaceSegments();
  std::vector<std::pair<common::math::LineSegment2d, double>>
      fs_segment_collision_big_buffer_check;
  std::vector<std::pair<common::math::LineSegment2d, double>>
      fs_segment_collision_little_buffer_check;
  std::unordered_map<uint32_t, perception::PerceptionObstacle_Type> box_id_type;

  const auto& start_point = frame_->PlanningStartPoint().path_point();
  const auto adc_box = common::VehicleConfigHelper::GetBoundingBox(
      start_point,
      path_decider_config_.ntp_cruise_freespace_ego_box_fillter_buffer(),
      path_decider_config_.ntp_cruise_freespace_ego_box_fillter_buffer());
  for (const auto* const obstacle : path_decision->obstacles().Items()) {
    if (obstacle->IsStatic()) {
      box_id_type.insert(std::make_pair(obstacle->Perception().id(),
                                        obstacle->Perception().type()));
    }
  }

  for (const auto& segment_info : freespace_segments) {
    //   使用0.2的侧向buffer的动、静态fs类型
    bool is_use_big_buffer_moving_cls =
        segment_info.cls_type == perception::FreeSpaceOut::UNKOWN_CLASS ||
        segment_info.cls_type == perception::FreeSpaceOut::VEHICLE ||
        segment_info.cls_type == perception::FreeSpaceOut::PEDESTRAIN;
    //   使用0.2的侧向buffer的纯静态fs类型
    bool is_use_big_buffer_static_cls =
        (segment_info.cls_type == perception::FreeSpaceOut::CURBSTONE &&
         segment_info.height_type == perception::FreeSpaceOut::OVERDRIVABLE) ||
        segment_info.cls_type == perception::FreeSpaceOut::OTHER_CLASS;
    //   使用0.1的侧向buffer的纯静态fs类型
    bool is_use_little_buffer_cls =
        (segment_info.cls_type == perception::FreeSpaceOut::CURBSTONE &&
         segment_info.height_type == perception::FreeSpaceOut::UNDERDRIVABLE) ||
        segment_info.cls_type == perception::FreeSpaceOut::CONE_POLE;
    const auto iter = box_id_type.find(segment_info.obstacleId);
    if (is_use_little_buffer_cls) {
      fs_segment_collision_little_buffer_check.emplace_back(
          segment_info.segment, 0.0);
    }
    if (is_use_big_buffer_static_cls) {
      fs_segment_collision_big_buffer_check.emplace_back(segment_info.segment,
                                                         0.0);
    }

    if (is_use_big_buffer_moving_cls &&
        (adc_box.HasOverlap(segment_info.segment) ||
         (segment_info.isLinkObjFusion && iter != box_id_type.end()))) {
      fs_segment_collision_big_buffer_check.emplace_back(segment_info.segment,
                                                         0.0);
    }
  }

  if (fs_segment_collision_big_buffer_check.empty() &&
      fs_segment_collision_little_buffer_check.empty()) {
    return;
  }

  int i = 0;
  double buffer_in_big_kappa =
      path_decider_config_.ntp_cruise_freespace_collision_buffer_in_big_kappa();
  double lateral_diff = 0.0;
  const auto fs_collision_buffer =
      path_decider_config_.ntp_cruise_freespace_collision_buffer();
  auto* avoid_alongside_debug = reference_line_info_->mutable_debug()
                                    ->mutable_avoid_alongside_decider_info();
  if (frame_ != nullptr && frame_->local_view().HasVehicleState() &&
      frame_->PlanningStartPoint().has_path_point()) {
    double angle_diff =
        std::fabs(common::math::NormalizeAngle(
            frame_->local_view().GetVehicleState()->heading() -
            frame_->PlanningStartPoint().path_point().theta())) *
        180.0 / M_PI;
    double ratio =
        std::max(kMinRation, buffer_in_big_kappa - fs_collision_buffer);
    buffer_in_big_kappa =
        ratio * (1.0 / (1.0 + exp(kA + kB * angle_diff))) + fs_collision_buffer;
    Vec2d curr_to_ego = {frame_->local_view().GetVehicleState()->x() -
                             frame_->PlanningStartPoint().path_point().x(),
                         frame_->local_view().GetVehicleState()->y() -
                             frame_->PlanningStartPoint().path_point().y()};
    Vec2d path_point_unit = Vec2d::CreateUnitVec2d(
        frame_->PlanningStartPoint().path_point().theta());
    lateral_diff = path_point_unit.CrossProd(curr_to_ego);
  }

  for (const auto& path_point : discretized_path) {
    double left_buffer = fs_collision_buffer;
    double right_buffer = fs_collision_buffer;
    if (fabs(path_point.kappa()) >
            path_decider_config_.ntp_use_big_buffer_kappa_freespace() &&
        reference_line_info_->path_data()
            .frenet_frame_path()
            .is_forward_path()) {
      if (path_point.kappa() > 0) {
        right_buffer = buffer_in_big_kappa;
      } else if (path_point.kappa() < 0) {
        left_buffer = buffer_in_big_kappa;
      }
    }
    if (lateral_diff < 0) {
      right_buffer += fabs(lateral_diff);
    } else {
      left_buffer += lateral_diff;
    }
    // big buffer is 0.1 bigger than little buffer (base 0.1->0.2)
    double left_buffer_big =
        left_buffer +
        path_decider_config_
            .ntp_cruise_freespace_collision_buffer_bigger_offset();
    double right_buffer_big =
        right_buffer +
        path_decider_config_
            .ntp_cruise_freespace_collision_buffer_bigger_offset();
    const auto ego_polygon_little_buffer =
        common::VehicleConfigHelper::GetPolygon2dWithBuffer(
            path_point.x(), path_point.y(), path_point.theta(),
            fs_collision_buffer, fs_collision_buffer, left_buffer,
            right_buffer);
    const auto ego_polygon_big_buffer =
        common::VehicleConfigHelper::GetPolygon2dWithBuffer(
            path_point.x(), path_point.y(), path_point.theta(),
            fs_collision_buffer, fs_collision_buffer, left_buffer_big,
            right_buffer_big);
    const auto vehicle_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    const double diff = (vehicle_param.front_edge_to_center() -
                         vehicle_param.back_edge_to_center()) /
                        2.0;
    common::math::Vec2d polygon2d_center(
        path_point.x() + diff * std::cos(path_point.theta()),
        path_point.y() + diff * std::sin(path_point.theta()));
    const auto unit_vec2d =
        common::math::Vec2d::CreateUnitVec2d(path_point.theta());
    const double euclidean_filter_distance =
        std::hypot(vehicle_param.length(), vehicle_param.width()) / 2.0 *
        kAmplificationRatio;
    const double longitudinal_filter_distance =
        vehicle_param.length() / 2.0 * kAmplificationRatio;
    const double lateral_filter_distance =
        vehicle_param.width() / 2.0 * kAmplificationRatio;
    if (common::math::CheckCollisionWithVehiclePolygon2d(
            ego_polygon_little_buffer, polygon2d_center, unit_vec2d,
            fs_segment_collision_little_buffer_check, euclidean_filter_distance,
            longitudinal_filter_distance, lateral_filter_distance) ||
        common::math::CheckCollisionWithVehiclePolygon2d(
            ego_polygon_big_buffer, polygon2d_center, unit_vec2d,
            fs_segment_collision_big_buffer_check, euclidean_filter_distance,
            longitudinal_filter_distance, lateral_filter_distance)) {
      std::vector<std::string> wait_for_obstacle_ids;
      std::string virtual_obstacle_id = "FREESPACE_SEGMET";
      int n = i;
      double center_to_front_or_back_edge =
          reference_line_info_->path_data()
                  .frenet_frame_path()
                  .is_forward_path()
              ? vehicle_param.front_edge_to_center()
              : vehicle_param.back_edge_to_center();
      for (; n < discretized_path.size(); n++) {
        if (discretized_path.at(n).s() - discretized_path.at(i).s() >
            center_to_front_or_back_edge) {
          break;
        }
      }
      i = static_cast<int>(std::fmin(n, discretized_path.size() - 1));
      auto* debug_adc_box_little = avoid_alongside_debug->add_adc_box();
      for (const auto& point : ego_polygon_little_buffer.points()) {
        auto* debug_adc_point = debug_adc_box_little->add_point();
        debug_adc_point->set_x(point.x());
        debug_adc_point->set_y(point.y());
      }
      auto* debug_adc_box_big = avoid_alongside_debug->add_adc_box();
      for (const auto& point : ego_polygon_big_buffer.points()) {
        auto* debug_adc_point = debug_adc_box_big->add_point();
        debug_adc_point->set_x(point.x());
        debug_adc_point->set_y(point.y());
      }
      util::BuildStopDecision(
          virtual_obstacle_id,
          reference_line_info_->path_data().frenet_frame_path().at(i).s(),
          config_.path_decider_config().freespace_stop_buffer(),
          StopReasonCode::STOP_REASON_FREESPACE, wait_for_obstacle_ids,
          "PathDecider", frame_, reference_line_info_);
      break;
    }
    ++i;
  }
}

double PathDecider::GetADCLBuffer(const Obstacle& obs, bool is_avp_mode) const {
  if (is_avp_mode) {
    return FLAGS_adc_l_buffer_for_static_obstacle_avp_mode;
  }

  bool is_stop_obstacle = false;
  if (injector_ == nullptr || injector_->frame_history() == nullptr ||
      injector_->frame_history()->Latest() == nullptr ||
      injector_->frame_history()->Latest()->DriveReferenceLineInfo() ==
          nullptr) {
    is_stop_obstacle = false;
  } else {
    const auto* last_obstacle = injector_->frame_history()
                                    ->Latest()
                                    ->DriveReferenceLineInfo()
                                    ->path_decision()
                                    .Find(obs.Id());
    is_stop_obstacle = last_obstacle != nullptr &&
                       last_obstacle->LongitudinalDecision().has_stop();
  }

  return is_stop_obstacle
             ? common::math::InterpolationOne(
                   frame_->PlanningStartPoint().v(),
                   path_decider_config_.adc_l_buffer_for_stop_obstacle()
                       .speed(),
                   path_decider_config_.adc_l_buffer_for_stop_obstacle()
                       .buffer())
             : common::math::InterpolationOne(
                   frame_->PlanningStartPoint().v(),
                   path_decider_config_.adc_l_buffer_for_static_obstacle()
                       .speed(),
                   path_decider_config_.adc_l_buffer_for_static_obstacle()
                       .buffer());
}

double PathDecider::GetStopDistance(const Obstacle& obstacle,
                                    const double* kappa) const {
  double stop_distance = obstacle.NormalStopDistance();
  const auto fsm_state =
      frame_->local_view().HasFunctionManagerOut()
          ? frame_->local_view().GetFunctionManagerOut()->fsm_state()
          : functionmanager::MachineStateType::INITIAL_TYPE;
  if (fsm_state != functionmanager::MachineStateType::HDMAP_AVP_TYPE) {
    return stop_distance;
  }

  // stop distance for reverse vehicle
  if (obstacle.Perception().type() ==
      TL::perception::PerceptionObstacle::VEHICLE) {
    const auto& reference_point =
        reference_line_info_->reference_line().GetReferencePoint(
            obstacle.PerceptionSLBoundary().start_s());
    const double angle_diff = fabs(common::math::NormalizeAngle(
        obstacle.Perception().theta() - reference_point.heading()));
    if (angle_diff >
        path_decider_config_.ntp_reverse_vehicle_heading_diff_threshold() *
            M_PI) {
      return path_decider_config_.ntp_reverse_vehicle_stop_distance();
    }
  }

  if (kappa != nullptr) {
    double stop_dis_calc_func_parm_c_use = 0.0;
    if (obstacle.Perception().type() ==
        TL::perception::PerceptionObstacle::PEDESTRIAN) {
      stop_dis_calc_func_parm_c_use =
          path_decider_config_.stop_dis_calc_func_parm_c_pedestrian();
    } else {
      stop_dis_calc_func_parm_c_use =
          path_decider_config_.stop_dis_calc_func_parm_c();
    }
    stop_distance =
        (path_decider_config_.stop_dis_calc_func_parm_k() -
         stop_dis_calc_func_parm_c_use) /
            (1 + exp(*kappa * path_decider_config_.stop_dis_calc_func_parm_b() +
                     path_decider_config_.stop_dis_calc_func_parm_a())) +
        stop_dis_calc_func_parm_c_use;
  }

  return stop_distance;
}

}  // namespace planning
}  // namespace TL
