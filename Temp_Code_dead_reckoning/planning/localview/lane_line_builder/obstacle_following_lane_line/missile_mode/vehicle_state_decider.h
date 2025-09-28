/*
 * Copyright (c) TL auto Co., Ltd. 2023-2024. All rights reserved.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Eigen/LU"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "planning/localview/local_view.h"
#include "planning/localview/localview_comdata_manager.h"

namespace TL {
namespace planning {
namespace missilelane {
using TL::common::Status;
using Matrix = Eigen::MatrixXd;
using TL::common::math::Vec2d;

class MissileVehicleState {
 public:
  MissileVehicleState() = default;
  ~MissileVehicleState() = default;
  Status Init(const std::shared_ptr<LocalViewData>& local_view_data);
  bool Update(const std::shared_ptr<LocalView>& local_view,
              const functionmanager::FunctionManagerOut& to_fct);

  const std::shared_ptr<const common::VehicleState>& vehicle_state() {
    return vehicle_state_;
  }

  const Matrix& TransMatrix() { return trans_matrix_; }

  Matrix TransMatrix(double dx, double dphi);

  double dx() const { return dx_; }

  double dy() const { return dy_; }

  double dy_ctrv() const { return dy_ctrv_; }

  double dphi() const { return dphi_; }

  double spd() const {
    return (vehicle_state_ == nullptr) ? 0.0
                                       : vehicle_state_->linear_velocity();
  }

  double yaw_rate() const {
    return (vehicle_state_ == nullptr) ? 0.0
                                       : vehicle_state_->angular_velocity();
  }

  bool is_missile_mode_active() const { return is_missile_mode_active_; }

  bool is_only_acc_active() const { return is_only_acc_active_; }

  bool is_pilot_active() const { return is_pilot_active_; }

  std::shared_ptr<LocalViewData> local_view_data() { return local_view_data_; }

 private:
  void CalculateDyByCTRV(double yaw_rate, double speed, double ts);
  static double SafeDivide(double nom, double denom, double threshold);
  std::shared_ptr<const common::VehicleState> vehicle_state_{nullptr};
  std::shared_ptr<LocalViewData> local_view_data_ = nullptr;
  double dx_{0.0};
  // dy默认通过曲率求得
  double dy_{0.0};
  double dy_ctrv_{0.0};
  double dphi_{0.0};
  Matrix trans_matrix_{Matrix::Zero(3, 3)};
  double history_yaw_rate_{0.0};
  // 可以添加历史点的推算记录
  std::vector<Vec2d> ego_points_{};
  lanelineprocess::DebounceModule yawrate_rise_debounce_{2, 0, 0.1};
  double faik_last_{0.0};
  bool is_yawrate_positive_debounce_{false};
  bool is_missile_mode_active_{false};
  bool is_only_acc_active_{false};
  bool is_pilot_active_{true};
};
}  // namespace missilelane
}  // namespace planning
}  // namespace TL
