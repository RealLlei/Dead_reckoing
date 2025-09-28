/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  kinematic_model.h
 */

#pragma once
#include "Eigen/Dense"

struct Param {
  int state_dim = 4;
  int control_dim = 2;
  int max_outter_iter = 20;
  int max_inner_iter = 30;
  double wheelbase = 2.96;
  double max_acc = 1.5;
  double min_acc = -1.5;
  double max_w = 0.55;
  double min_w = -0.55;
  double dt = 0.2;
  double max_spd = 10.0;
  double w_x = 0.0;
  double w_y = 0.0;
  double w_spd = 0.0;
  double w_theta = 0.0;
  double w_acc = 0.0;
  double w_w = 0.5;
  double factor = 5.0;
  double accuracy = 1e-5;
};

class KinematicModel {
 public:
  explicit KinematicModel(Param& param) : param_(param) {}

  ~KinematicModel() = default;

  /**
   * @brief Simulate the trajectory
   * 
   * @param cur_state 
   * @param cur_control 
   * @param next_state 
   */
  void Simulate(const Eigen::Ref<const Eigen::VectorXd>& cur_state,
                Eigen::Ref<Eigen::VectorXd> cur_control,
                Eigen::Ref<Eigen::VectorXd> next_state) const;

  /**
   * @brief Get the State First Derivative
   * 
   * @param theta 
   * @param spd 
   * @param acc 
   * @param state_first_derivative 
   */
  void StateJacobian(double theta, double spd, double acc,
                     Eigen::Ref<Eigen::MatrixXd> state_first_derivative) const;

  /**
   * @brief Get the Control First Derivative
   * 
   * @param theta 
   * @param control_first_derivative 
   */
  void ControlJacobian(
      double theta, Eigen::Ref<Eigen::MatrixXd> control_first_derivative) const;

 private:
  const Param param_;
};
