/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  open_space_straight_path_provider.cc
 */

#include "planning/tasks/optimizers/open_space_straight_path/open_space_straight_path_provider.h"

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/math/vec2d.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/planning_gflags.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/planning/planning_status.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::math::Vec2d;

OpenSpaceStraightPathProvider::OpenSpaceStraightPathProvider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : PathOptimizer(config, injector) {}

Status OpenSpaceStraightPathProvider::Process() {
  ADEBUG << "OpenSpaceStraightPathProvider start";
  if (frame_ == nullptr || injector_ == nullptr) {
    const std::string msg =
        "Invalid frame_ or injector, fail to process the "
        "OpenSpaceStraightPathProvider.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_STRAIGHTPATH_ERROR, msg);
  }
  const auto& parking_type = injector_->planning_context()
                                 ->planning_status()
                                 .avp_status()
                                 .parking_type();
  if ((parking_type != TL::planning::AVPStatus::DIRECT_FORWARD) &&
      (parking_type != TL::planning::AVPStatus::DIRECT_BACKWARD)) {
    const std::string msg = "parking_type is not direct control";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_STRAIGHTPATH_ERROR, msg);
  }
  auto* chosen_partitioned_path_ptr =
      frame_->mutable_open_space_info()->mutable_chosen_partitioned_path();
  const double move_direction =
      parking_type == TL::planning::AVPStatus::DIRECT_FORWARD ? 1.0 : -1.0;
  chosen_partitioned_path_ptr->second =
      move_direction > 0
          ? soc::Chassis::GearPosition::Chassis_GearPosition_GEAR_DRIVE
          : soc::Chassis::GearPosition::Chassis_GearPosition_GEAR_REVERSE;
  auto* path_data = &chosen_partitioned_path_ptr->first;
  auto start_point = frame_->PlanningStartPoint();
  const auto& vehicle_state = frame_->vehicle_state();
  if (NeedStopDecision(move_direction, vehicle_state)) {
    frame_->mutable_open_space_info()->set_is_stop_path(true);
    UpdateOpenSpaceStartPoint(vehicle_state, &start_point);
    path_data->GenerateStopPath(
        start_point.path_point().x(), start_point.path_point().y(),
        start_point.path_point().theta(), start_point.path_point().kappa());
    return Status::OK();
  }
  if (last_parking_type_ != parking_type) {
    original_point_ = Vec2d(vehicle_state.x(), vehicle_state.y());
    original_direction_ =
        common::math::Vec2d::CreateUnitVec2d(vehicle_state.heading());
    last_parking_type_ = parking_type;
    UpdateOpenSpaceStartPoint(vehicle_state, &start_point);
    AINFO << "change direction: " << move_direction;
  }
  GenerateDirectMovingPath(start_point, move_direction, path_data);
  return Status::OK();
}

void OpenSpaceStraightPathProvider::UpdateOpenSpaceStartPoint(
    const common::VehicleState& vehicle_state,
    common::TrajectoryPoint* const start_point) {
  start_point->mutable_path_point()->set_x(vehicle_state.x());
  start_point->mutable_path_point()->set_y(vehicle_state.y());
  start_point->mutable_path_point()->set_theta(vehicle_state.heading());
  start_point->mutable_path_point()->set_s(0);
  start_point->mutable_path_point()->set_kappa(0);
  start_point->mutable_path_point()->set_dkappa(0);
  frame_->mutable_open_space_info()->set_is_gear_changed(true);
}

bool OpenSpaceStraightPathProvider::NeedStopDecision(
    const double moving_direction, const common::VehicleState& vehicle_state) {
  soc::Chassis::GearPosition target_gear =
      moving_direction > 0
          ? soc::Chassis::GearPosition::Chassis_GearPosition_GEAR_DRIVE
          : soc::Chassis::GearPosition::Chassis_GearPosition_GEAR_REVERSE;
  if (target_gear != vehicle_state.gear()) {
    AINFO << "desire gear and current gear not math, target gear" << target_gear
          << "actual gear:" << vehicle_state.gear();
    return true;
  }
  const double vel_longitudal_spd =
      vehicle_state.pose().linear_velocity_vrf().y();
  if ((vel_longitudal_spd * moving_direction < 0.0) &&
      !frame_->IsVehicleStandStill()) {
    AINFO << "stop decison made, due to: velocity and move direction "
             "contradiction";
    return true;
  }
  return false;
}

void OpenSpaceStraightPathProvider::GenerateDirectMovingPath(
    const common::TrajectoryPoint& start_point, const double direction,
    planning::DiscretizedPath* const path_data) const {
  // generate path by interpolation
  if (path_data == nullptr) {
    return;
  }
  common::math::Vec2d start_to_original(
      (start_point.path_point().x() - original_point_.x()),
      (start_point.path_point().y() - original_point_.y()));
  auto start_point_projection =
      start_to_original.InnerProd(original_direction_) * original_direction_ +
      original_point_;
  double temp_x = start_point_projection.x();
  double temp_y = start_point_projection.y();
  double theta = original_direction_.Angle();
  static constexpr double kStepSize = 0.1;
  const size_t num_of_knots =
      static_cast<size_t>(FLAGS_direct_move_length / kStepSize) + 1;
  path_data->clear();
  path_data->reserve(num_of_knots);
  double temp_s = 0;
  common::PathPoint path_point;
  for (size_t i = 0; i < num_of_knots; i++) {
    path_point.set_x(temp_x);
    path_point.set_y(temp_y);
    path_point.set_theta(theta);
    path_point.set_s(temp_s);
    path_data->emplace_back(path_point);
    temp_x += direction * kStepSize * cos(theta);
    temp_y += direction * kStepSize * sin(theta);
    temp_s += kStepSize;
  }
}

}  // namespace planning
}  // namespace TL
