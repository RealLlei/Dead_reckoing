/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description: constraints.h
 */
#pragma once
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>
#include "common/util/util.h"

namespace TL::planning::game_common {

struct ObstacleConstraintsParam {
  double length = 1.98;  // EP40
  double width = 4.952;  // EP40
  double t_safe = 0.1;
  double s_safe_a = 10.0;
  double s_safe_b = 4.0;
  double ego_rad = 2.0;
  double ego_lf = 1.49;
  double ego_lr = 1.49;
  double q1_front = 2.75;
  double q2_front = 2.75;
  double q1_rear = 2.5;
  double q2_rear = 2.5;
};

struct ConstraintsParam {
  ObstacleConstraintsParam obstacle_constraints_param = {};
  double w_acc = 1.0;
  double w_yawrate = 3.0;
  double w_pos = 2.0;
  double w_vel = 0.5;
  int num_states = 4;
  int num_ctrls = 2;
  int horizon = 20;
  double desired_speed = 0.0;
  double max_acc = 2.5;    // EP40
  double min_acc = -4.95;  // EP40
  double q1_acc = 1.0;
  double q2_acc = 1.0;
  double steer_limit_lower = -8.0345;  // EP40
  double steer_limit_upper = 8.0345;   // EP40
  double q1_yawrate = 1.0;
  double q2_yawrate = 1.0;
  double wheel_base = 2.96;    // EP40
  double steer_ratio = 15.17;  // EP40
};

class BarrierFunction {
 public:
  static std::tuple<double, Eigen::VectorXd, Eigen::MatrixXd>
  ExpBarrierFunction(const double q1, const double q2, const double c,
                     const Eigen::VectorXd& c_dot) {
    const double b = q1 * std::exp(q2 * c);
    ADEBUG << "b: "
           << "\n"
           << b;
    const Eigen::VectorXd b_dot = q1 * q2 * std::exp(q2 * c) * c_dot;
    ADEBUG << "b_dot: "
           << "\n"
           << b_dot;
    const Eigen::MatrixXd b_ddot =
        q1 * (q2 * q2) * std::exp(q2 * c) * c_dot * c_dot.transpose();
    ADEBUG << "b_ddot: "
           << "\n"
           << b_ddot;
    return {b, b_dot, b_ddot};
  }
};

class ObstacleConstraint {
 private:
  ObstacleConstraintsParam obstacle_constraints_param_ = {};

 public:
  ObstacleConstraint() = default;

  explicit ObstacleConstraint(
      const ObstacleConstraintsParam& obstacle_constraints_param)
      : obstacle_constraints_param_(obstacle_constraints_param) {}

  std::pair<Eigen::VectorXd, Eigen::MatrixXd> GetObstacleCostDerivatives(
      const Eigen::Matrix<double, 4, Eigen::Dynamic>& obs_traj, int i,
      const Eigen::Vector4d& ego_state) const {
    // traj ==> (x, y, v, heading)
    ADEBUG << "###obs_traj: "
           << "\n"
           << obs_traj;
    const double speed = std::fabs(obs_traj(2, i));
    const double a = obstacle_constraints_param_.length +
                     speed * obstacle_constraints_param_.t_safe +
                     obstacle_constraints_param_.s_safe_a +
                     obstacle_constraints_param_.ego_rad;
    const double b = obstacle_constraints_param_.width +
                     obstacle_constraints_param_.s_safe_b +
                     obstacle_constraints_param_.ego_rad;

    Eigen::Matrix4d P1 = Eigen::Matrix4d::Zero();
    P1(0, 0) = 1.0 / (a * a);
    P1(1, 1) = 1.0 / (b * b);

    const double theta = obs_traj(3, i);
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Zero();
    transformation_matrix.block<2, 2>(0, 0) << std::cos(theta), std::sin(theta),
        -std::sin(theta), std::cos(theta);
    // front circle
    const Eigen::Vector4d ego_front =
        ego_state +
        Eigen::Vector4d(
            std::cos(ego_state(3)) * obstacle_constraints_param_.ego_lf,
            std::sin(ego_state(3)) * obstacle_constraints_param_.ego_lf, 0, 0);
    // x - x0
    Eigen::Vector4d diff = transformation_matrix *
                           (ego_front - obs_traj.col(i));  // m * n ==> 4 * 1
    double c = 1.0 - diff.transpose() * P1 * diff;
    Eigen::Vector4d c_dot = -2 * P1 * diff;  // m * n ==> 4 * 1
    const auto& [b_f, b_dot_f, b_ddot_f] = BarrierFunction::ExpBarrierFunction(
        obstacle_constraints_param_.q1_front,
        obstacle_constraints_param_.q2_front, c, c_dot);

    // rear circle
    const Eigen::Vector4d ego_rear =
        ego_state -
        Eigen::Vector4d(
            std::cos(ego_state(3)) * obstacle_constraints_param_.ego_lr,
            std::sin(ego_state(3)) * obstacle_constraints_param_.ego_lr, 0, 0);
    diff = transformation_matrix * (ego_rear - obs_traj.col(i));
    c = 1.0 - diff.transpose() * P1 * diff;
    c_dot = -2 * P1 * diff;
    const auto& [b_r, b_dot_r, b_ddot_r] = BarrierFunction::ExpBarrierFunction(
        obstacle_constraints_param_.q1_rear,
        obstacle_constraints_param_.q2_rear, c, c_dot);

    return {b_dot_f + b_dot_r, b_ddot_f + b_ddot_r};
  }
};

class Constraints {
 public:
  Constraints() = default;
  explicit Constraints(const ConstraintsParam& constraints_param)
      : constraints_param_(constraints_param) {
    control_cost_(0, 0) = constraints_param.w_acc;
    control_cost_(1, 1) = constraints_param.w_yawrate;
    state_cost_(0, 0) = constraints_param.w_pos;
    state_cost_(1, 1) = constraints_param.w_pos;
    state_cost_(2, 2) = constraints_param.w_vel;

    obs_constraints_.clear();
    obs_constraints_.resize(num_obs_);
    for (int i = 0; i < num_obs_; ++i) {
      obs_constraints_[i] =
          ObstacleConstraint(constraints_param.obstacle_constraints_param);
    }
  }

  std::pair<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>>
  GetStateCostDerivatives(
      const Eigen::Matrix<double, 4, Eigen::Dynamic>& ego_state,
      const Eigen::Matrix<double, 4, Eigen::Dynamic>& obs_traj,
      const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_traj) {
    Eigen::MatrixXd l_x = Eigen::MatrixXd::Zero(constraints_param_.num_states,
                                                constraints_param_.horizon);
    std::vector<Eigen::MatrixXd> l_xx(
        constraints_param_.horizon,
        Eigen::MatrixXd::Zero(constraints_param_.num_states,
                              constraints_param_.num_states));

    for (int i = 0; i < constraints_param_.horizon; ++i) {
      double min_distance = std::numeric_limits<double>::max();
      int min_index = 0;
      for (int j = 0; j < ref_traj.cols(); ++j) {
        double distance =
            (ego_state.col(i).block(0, 0, 2, 1) - ref_traj.col(j)).norm();
        if (distance < min_distance) {
          min_distance = distance;
          min_index = j;
        }
      }
      Eigen::Vector4d state_diff(
          ego_state(0, i) - ref_traj(0, min_index),
          ego_state(1, i) - ref_traj(1, min_index),
          ego_state(2, i) - constraints_param_.desired_speed, 0);
      Eigen::Vector4d traj_cost = 2 * state_cost_ * state_diff;
      Eigen::Vector4d l_x_i = traj_cost;
      Eigen::Matrix4d l_xx_i = 2 * state_cost_;

      for (int j = 0; j < num_obs_; ++j) {
        const auto& [b_dot_obs, b_ddot_obs] =
            obs_constraints_[j].GetObstacleCostDerivatives(obs_traj, i,
                                                           ego_state.col(i));
        l_x_i += b_dot_obs;
        l_xx_i += b_ddot_obs;
      }
      l_x.col(i) = l_x_i;
      l_xx[i] = l_xx_i;
    }
    return {l_x, l_xx};
  }

  std::pair<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>>
  GetControlCostDerivatives(
      const Eigen::Matrix<double, 2, Eigen::Dynamic>& control,
      const Eigen::Matrix<double, 4, Eigen::Dynamic>& ego_state) {
    const Eigen::Vector2d P1 = {1.0, 0.0};
    const Eigen::Vector2d P2 = {0.0, 1.0};
    Eigen::MatrixXd l_u = Eigen::MatrixXd::Zero(constraints_param_.num_ctrls,
                                                constraints_param_.horizon);
    std::vector<Eigen::MatrixXd> l_uu(
        constraints_param_.horizon,
        Eigen::MatrixXd::Zero(constraints_param_.num_ctrls,
                              constraints_param_.num_ctrls));
    for (int i = 0; i < constraints_param_.horizon; ++i) {
      // Acceleration Barrier Max
      double c = control.col(i).transpose() * P1 - constraints_param_.max_acc;
      const auto& [b_1, b_dot_1, b_ddot_1] =
          BarrierFunction::ExpBarrierFunction(constraints_param_.q1_acc,
                                              constraints_param_.q2_acc, c, P1);
      // Acceleration Barrier Min
      c = constraints_param_.min_acc - control.col(i).transpose() * P1;
      const auto& [b_2, b_dot_2, b_ddot_2] =
          BarrierFunction::ExpBarrierFunction(
              constraints_param_.q1_acc, constraints_param_.q2_acc, c, -P1);
      // Yawrate Barrier Max
      const double velocity = ego_state(2, i);
      c = control.col(i).transpose() * P2 -
          velocity *
              std::tan(constraints_param_.steer_limit_upper /
                       constraints_param_.steer_ratio) /
              constraints_param_.wheel_base;
      const auto& [b_3, b_dot_3, b_ddot_3] =
          BarrierFunction::ExpBarrierFunction(constraints_param_.q1_yawrate,
                                              constraints_param_.q2_yawrate, c,
                                              P2);
      // Yawrate Barrier Min
      c = velocity *
              std::tan(constraints_param_.steer_limit_lower /
                       constraints_param_.steer_ratio) /
              constraints_param_.wheel_base -
          control.col(i).transpose() * P2;
      const auto& [b_4, b_dot_4, b_ddot_4] =
          BarrierFunction::ExpBarrierFunction(constraints_param_.q1_yawrate,
                                              constraints_param_.q2_yawrate, c,
                                              -P2);
      ADEBUG << "###cd i:" << i << ", b_dot_1: "
             << "\n"
             << b_dot_1 << "\n"
             << "b_dot_2"
             << "\n"
             << b_dot_2 << "\n"
             << "b_dot_3"
             << "\n"
             << b_dot_3 << "\n"
             << "b_dot_4"
             << "\n"
             << b_dot_4;
      ADEBUG << "###cd i:" << i << ", b_ddot_1: "
             << "\n"
             << b_ddot_1 << "\n"
             << "b_ddot_2"
             << "\n"
             << b_ddot_2 << "\n"
             << "b_ddot_3"
             << "\n"
             << b_ddot_3 << "\n"
             << "b_ddot_4"
             << "\n"
             << b_ddot_4;
      const Eigen::VectorXd l_u_i =
          b_dot_1 + b_dot_2 + b_dot_3 + b_dot_4 +
          (2 * control.col(i).transpose() * control_cost_).transpose();
      const Eigen::MatrixXd l_uu_i =
          b_ddot_1 + b_ddot_2 + b_ddot_3 + b_ddot_4 + 2 * control_cost_;
      l_u.col(i) = l_u_i;
      l_uu[i] = l_uu_i;
      ADEBUG << "###l_uu_i: "
             << "\n"
             << l_uu_i;
    }
    return {l_u, l_uu};
  }

  std::tuple<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>, Eigen::MatrixXd,
             std::vector<Eigen::MatrixXd>, std::vector<Eigen::MatrixXd>>
  GetCostDerivatives(const Eigen::Matrix<double, 2, Eigen::Dynamic>& control,
                     const Eigen::Matrix<double, 4, Eigen::Dynamic>& ego_state,
                     const Eigen::Matrix<double, 4, Eigen::Dynamic>& obs_traj,
                     const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_traj) {
    const auto& [l_u, l_uu] = GetControlCostDerivatives(control, ego_state);
    const auto& [l_x, l_xx] =
        GetStateCostDerivatives(ego_state, obs_traj, ref_traj);
    const std::vector<Eigen::MatrixXd> l_ux(
        constraints_param_.horizon,
        Eigen::MatrixXd::Zero(constraints_param_.num_ctrls,
                              constraints_param_.num_states));
    return std::make_tuple(l_x, l_xx, l_u, l_uu, l_ux);
  }

  double GetTotalCost(
      const Eigen::Matrix<double, 4, Eigen::Dynamic>& ego_state,
      const Eigen::Matrix<double, 2, Eigen::Dynamic>& control,
      const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_traj) {
    double J = 0.0;
    for (int i = 0; i < constraints_param_.horizon; ++i) {
      double min_distance = std::numeric_limits<double>::max();
      int min_index = 0;
      for (int j = 0; j < ref_traj.cols(); ++j) {
        double distance =
            (ego_state.col(i).block(0, 0, 2, 1) - ref_traj.col(j)).norm();
        if (distance < min_distance) {
          min_distance = distance;
          min_index = j;
        }
      }
      Eigen::Vector4d state_diff(
          ego_state(0, i) - ref_traj(0, min_index),
          ego_state(1, i) - ref_traj(1, min_index),
          ego_state(2, i) - constraints_param_.desired_speed, 0);
      double c_state = state_diff.transpose() * state_cost_ * state_diff;
      double c_ctrl =
          control.col(i).transpose() * control_cost_ * control.col(i);
      J = J + c_state + c_ctrl;
    }
    return J;
  }

 private:
  ConstraintsParam constraints_param_ = {};
  Eigen::Matrix2d control_cost_ = Eigen::Matrix2d::Zero();
  Eigen::Matrix4d state_cost_ = Eigen::Matrix4d::Zero();
  int num_obs_ = 1;
  std::vector<ObstacleConstraint> obs_constraints_ = {};
};
}  // namespace TL::planning::game_common
