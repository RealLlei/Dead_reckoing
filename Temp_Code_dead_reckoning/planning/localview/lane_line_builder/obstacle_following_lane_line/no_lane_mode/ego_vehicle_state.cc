/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/04/21
 *****************************************************************************/

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/ego_vehicle_state.h"

#include "common/math/box2d.h"

namespace TL {
namespace planning {
namespace nolane {

EgoVehicleState::EgoVehicleState()
    : vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
}

void EgoVehicleState::UpdateEgoState(
    const TL::common::VehicleState& vehicle_state) {
  vehicle_state_present_.CopyFrom(vehicle_state);
  if (vehicle_state_history_.size() > max_size_ - 1) {
    vehicle_state_history_.pop_back();
  }
  vehicle_state_history_.emplace_front(vehicle_state);
  CalculateMinTurnRadius();
  // lp: TODO convert to present coordinate system.
  if (debug_flag[3]) {
    AERROR << PRECISION(3) << "veh_x:" << vehicle_state_present_.x()
           << " veh_y:" << vehicle_state_present_.y()
           << "  veh_theta:" << vehicle_state_present_.heading()
           << "  veh_v:" << vehicle_state_present_.linear_velocity();
    Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                                vehicle_param_.back_edge_to_center()) *
                                   0.5,
                               (vehicle_param_.left_edge_to_center() -
                                vehicle_param_.right_edge_to_center()) *
                                   0.5);
    ego_center_map_frame.SelfRotate(vehicle_state_present_.heading());
    ego_center_map_frame.set_x(ego_center_map_frame.x() +
                               vehicle_state_present_.x());
    ego_center_map_frame.set_y(ego_center_map_frame.y() +
                               vehicle_state_present_.y());

    // Compute the ADC bounding box.
    TL::common::math::Box2d adc_box(
        ego_center_map_frame, vehicle_state_present_.heading(),
        vehicle_param_.length(), vehicle_param_.width());
    for (auto& po : adc_box.GetAllCorners()) {
      AERROR << PRECISION(3) << "ego_point_x:" << po.x() << "   "
             << "ego_point_y:" << po.y();
    }
    AERROR << PRECISION(3)
           << "ego_point_x:" << adc_box.GetAllCorners().front().x() << "   "
           << "ego_point_y:" << adc_box.GetAllCorners().front().y();
  }

  // lp: sent to cyber
  SendToCyber();
}

const common::VehicleState& EgoVehicleState::GetVehicleStatePresent() const {
  return vehicle_state_present_;
}

void EgoVehicleState::SetVehicleStatePresent(
    const common::VehicleState& vehicleStatePresent) {
  vehicle_state_present_ = vehicleStatePresent;
}

const common::VehicleParam& EgoVehicleState::GetVehicleParam() const {
  return vehicle_param_;
}

void EgoVehicleState::SetVehicleParam(
    const common::VehicleParam& vehicleParam) {
  vehicle_param_ = vehicleParam;
}

const std::list<TL::common::VehicleState>&
EgoVehicleState::GetVehicleStateHistory() const {
  return vehicle_state_history_;
}

const common::SLPoint& EgoVehicleState::GetVehicleSl() const {
  return vehicle_sl_;
}

void EgoVehicleState::SetVehicleStateHistory(
    const std::list<TL::common::VehicleState>& vehicleStateHistory) {
  vehicle_state_history_ = vehicleStateHistory;
}

void EgoVehicleState::SetVehicleSl(const common::SLPoint& vehicleSl) {
  vehicle_sl_ = vehicleSl;
}

void EgoVehicleState::SendToCyber() {
  auto ego_info = ptr_without_lane->mutable_ego_info();

  ego_info->mutable_ego_param()->set_brand(
      ::TL::common::VehicleBrand::NETA_EP31);

  auto VehStateSet = [](const ::TL::common::VehicleState& veh_source,
                        ::TL::common::VehicleState* const veh_des) {
    veh_des->set_timestamp(veh_source.timestamp());
    veh_des->set_x(veh_source.x());
    veh_des->set_y(veh_source.y());
    veh_des->set_heading(veh_source.heading());
    veh_des->set_linear_velocity(veh_source.linear_velocity());
    // veh_des->set_linear_acceleration(veh_source.linear_acceleration());
  };

  VehStateSet(vehicle_state_present_, ego_info->mutable_ego_present_state());

  for (const auto& veh_state : vehicle_state_history_) {
    VehStateSet(veh_state, ego_info->add_ego_state_history());
  }
}

void EgoVehicleState::ProjectionEgoSL(
    const std::shared_ptr<ReferenceLine>& prev_reference_line_ptr) {
  prev_reference_line_ptr->XYToSL(
      {vehicle_state_present_.x(), vehicle_state_present_.y()}, &vehicle_sl_);
}

void EgoVehicleState::CalculateMinTurnRadius() {
  const double min_radius = vehicle_param_.min_turn_radius();
  const double veh_speed = vehicle_state_present_.linear_velocity();
  if (veh_speed < 1) {
    min_turn_radius_ = fmax(min_radius, 15);
  } else if (veh_speed < 10) {
    min_turn_radius_ = 40;
  } else if (veh_speed < 30) {
    min_turn_radius_ = 80;
  } else {
    min_turn_radius_ = 200;
  }
}
}  // namespace nolane
}  // namespace planning
}  // namespace TL
