/*
 * Copyright (c) TL auto Co., Ltd. 2023-2024. All rights reserved.
 */
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/vehicle_state_decider.h"

#include <cmath>

#include "common/math/math_utils.h"
#include "common/status/status.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"

namespace TL {
namespace planning {
namespace missilelane {
using functionmanager::FctToNnpInput;
using TL::common::math::Clamp;

Status MissileVehicleState::Init(
    const std::shared_ptr<LocalViewData>& local_view_data) {
  local_view_data_ = local_view_data;
  return Status::OK();
}

bool MissileVehicleState::Update(
    const std::shared_ptr<LocalView>& local_view,
    const functionmanager::FunctionManagerOut& to_fct) {
  if (!local_view->HasVehicleState() || !local_view->HasFunctionManagerIn() ||
      !local_view->GetChassis()) {
    return false;
  }
  const auto sub_per_state = to_fct.perception_sub_state();
  const auto ta_pilot_mode =
      local_view->GetFunctionManagerIn()->ta_pilot_mode();
  is_missile_mode_active_ =
      (sub_per_state == functionmanager::PerceptionSubState::NOLANE_TYPE &&
       ta_pilot_mode == functionmanager::ADAS);
  FctToNnpInput::NPILOT_State pilot_state = FctToNnpInput::PILOT_OFF;
  FctToNnpInput::ADCS8_ACCState acc_sytate = FctToNnpInput::ACC_OFF;
  functionmanager::NNPSysState nnp_state =
      functionmanager::NNPSysState::NNPS_OFF;
  if (local_view->HasFunctionManagerIn()) {
    auto fct_in = local_view->GetFunctionManagerIn()->fct_nnp_in();
    acc_sytate = fct_in.acc_state();
    pilot_state = fct_in.npilot_state();
    nnp_state = fct_in.nnp_sysstate();
  }
  // is_only_acc_active_ = local_view->GetChassis()->driving_mode() == TL::soc::Chassis::AUTO_SPEED_ONLY;
  is_only_acc_active_ =
      (acc_sytate == FctToNnpInput::ACC_ACTIVE) &&
      (pilot_state != FctToNnpInput::PILOT_ACTIVE &&
       pilot_state != FctToNnpInput::PILOT_SUSPEND) &&
      (nnp_state != functionmanager::NNPSysState::NNPS_ACTIVE &&
       nnp_state != functionmanager::NNPSysState::NNPS_OVERRIDE);
  is_pilot_active_ = (pilot_state == FctToNnpInput::PILOT_ACTIVE ||
                      pilot_state == FctToNnpInput::PILOT_SUSPEND);
  vehicle_state_ = local_view->GetVehicleState();
  auto spd = vehicle_state_->linear_velocity();
  auto yaw_rate = vehicle_state_->angular_velocity();
  // 后续需要每周期计算
  double ts = 0.1;
  dx_ = spd * ts;
  dphi_ = yaw_rate * ts;
  // lateral vehicle dynamics
  // yaw_rate        1        wheel steering angle
  //—————————— ～= ————————— = —————————————————————— = kappa
  //   speed       Radius           wheelbase
  dy_ = dx_ * dx_ * (history_yaw_rate_ / Clamp(spd, 0.001, 1000.0)) * 0.5;
  history_yaw_rate_ = yaw_rate;

  trans_matrix_(0, 0) = std::cos(dphi_);
  trans_matrix_(0, 1) = std::sin(dphi_);
  trans_matrix_(0, 2) = -(dx_ * std::cos(dphi_) + dy_ * std::sin(dphi_));
  trans_matrix_(1, 0) = -std::sin(dphi_);
  trans_matrix_(1, 1) = std::cos(dphi_);
  trans_matrix_(1, 2) = (dx_ * std::sin(dphi_) - dy_ * std::cos(dphi_));
  trans_matrix_(2, 2) = 1;
  ADEBUG << "dx: " << dx_ << " , dy: " << dy_ << " , dphi: " << dphi_;
  CalculateDyByCTRV(yaw_rate, spd, ts);
  return true;
}

// CTRV计算dy
void MissileVehicleState::CalculateDyByCTRV(double yaw_rate, double speed,
                                            double ts) {
  const double radius = SafeDivide(speed, yaw_rate, (0.001));
  constexpr double k_delta_yaw_rate = 0.15;

  bool is_yawrate_positive =
      (!is_yawrate_positive_debounce_) &&
      (std::abs(yaw_rate) <= k_delta_yaw_rate / (180 / M_PI));
  double delta_faik = yaw_rate * ts;
  double faik = delta_faik + faik_last_;
  is_yawrate_positive_debounce_ =
      yawrate_rise_debounce_.DealDebounce(is_yawrate_positive);
  dy_ctrv_ = radius * (1.0 - std::cos(delta_faik));
  // AERROR << "dy_ctrv_: " << dy_ctrv_ << "yaw_rate_degs" << yaw_rate
  //        << " is_yawrate_positive: " << is_yawrate_positive << "faik: " << faik
  //        << "faik_last_" << faik_last_ << "delta_faik: " << delta_faik
  //        << " radius:" << radius << "cos:" << cos(faik_last_) - cos(faik);
  // 更新历史值
  // double dy = radius * (1.0 - cos(delta_faik));
  // AERROR << " -------NEW dy: " << dy << " ,old dy: " << dy_ctrv_ * 10.0;
  // AERROR << "is_yawrate_positive_debounce: " << is_yawrate_positive_debounce_
  //        << " , yaw_rate: " << yaw_rate << " , last yaw_rate: " << faik_last_;

  faik_last_ = is_yawrate_positive_debounce_ ? 0.0 : faik;
}

Matrix MissileVehicleState::TransMatrix(double dx, double dphi) {
  Matrix trans_matrix{Matrix::Zero(3, 3)};
  // history_yaw_rate可能不准，但是影响不会太大
  double dy = dx * dx *
              (history_yaw_rate_ /
               Clamp(vehicle_state_->linear_velocity(), 0.001, 1000.0)) *
              0.5;
  trans_matrix(0, 0) = std::cos(dphi);
  trans_matrix(0, 1) = std::sin(dphi);
  trans_matrix(0, 2) = -(dx * std::cos(dphi) + dy * std::sin(dphi));
  trans_matrix(1, 0) = -std::sin(dphi);
  trans_matrix(1, 1) = std::cos(dphi);
  trans_matrix(1, 2) = (dx * std::sin(dphi) - dy * std::cos(dphi));
  trans_matrix(2, 2) = 1;
  return trans_matrix;
}

double MissileVehicleState::SafeDivide(double nom, double denom,
                                       double threshold) {
  double denominator = 0.0;
  if (fabs(denom) > threshold) {
    denominator = denom;
  } else if (denom > 0) {
    denominator = threshold;
  } else {
    denominator = -threshold;
  }
  return nom / denominator;
}

}  // namespace missilelane
}  // namespace planning
}  // namespace TL
