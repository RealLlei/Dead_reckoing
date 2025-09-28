/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  al_ilqr.cc
 */

#include "planning/open_space/ilqr_smoother/al_ilqr.h"
#include <iostream>
#include <iterator>
#include <memory>
#include <vector>
#include "common/file/log.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "planning/open_space/ilqr_smoother/control_constrain.h"
#include "planning/open_space/ilqr_smoother/endpose_constrain.h"
#include "planning/open_space/ilqr_smoother/obs_constrain.h"

ALILQR::ALILQR(const Param& param) : param_(param) {
  kinematic_model_ptr_ = std::make_shared<KinematicModel>(param_);
  weight_ = Eigen::MatrixXd::Zero(6, 6);
  weight_(0, 0) = param_.w_x;
  weight_(1, 1) = param_.w_y;
  weight_(2, 2) = param_.w_spd;
  weight_(3, 3) = param_.w_theta;
  weight_(4, 4) = param_.w_acc;
  weight_(5, 5) = param_.w_w;
}

void ALILQR::ComputeCubicCurveCoefficients(const double x1, const double y1,
                                           const double dy1, const double x2,
                                           const double y2, const double dy2,
                                           Eigen::VectorXd* const coef) {
  Eigen::Matrix4d A;
  A << 1, x1, x1 * x1, x1 * x1 * x1, 1, x2, x2 * x2, x2 * x2 * x2, 0, 1, 2 * x1,
      3 * x1 * x1, 0, 1, 2 * x2, 3 * x2 * x2;
  Eigen::Vector4d b;
  b << y1, y2, dy1, dy2;
  if (A.cols() != b.rows()) {
    AERROR << "illegal matrix operation!";
    return;
  }
  *coef = A.inverse() * b;
}

void ALILQR::SetEndPoseConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& end_pose,
    const std::pair<double, double>& pose_relax) {
  if (nullptr == al_costs_vec_[horizon_]) {
    al_costs_vec_[horizon_] = std::make_shared<ALCost>(param_);
  }
  const auto end_pose_x_constrain = std::make_shared<EndPoseXConstrain>();
  end_pose_x_constrain->Init(pose_relax);
  al_costs_vec_[horizon_]->SetConstraint(end_pose_x_constrain);
  const auto end_pose_y_constrain = std::make_shared<EndPoseYConstrain>();
  end_pose_y_constrain->Init(end_pose);
  al_costs_vec_[horizon_]->SetConstraint(end_pose_y_constrain);
  const auto end_pose_theta_constrain =
      std::make_shared<EndPoseThetaConstrain>();
  end_pose_theta_constrain->Init(end_pose);
  al_costs_vec_[horizon_]->SetConstraint(end_pose_theta_constrain);
}

void ALILQR::SetControlBound(const int direction) {
  for (int i = 0; i < horizon_; ++i) {
    if (nullptr == al_costs_vec_[i]) {
      al_costs_vec_[i] = std::make_shared<ALCost>(param_);
    }
    const auto control_constrain = std::make_shared<ControlConstrain>();
    control_constrain->Init(param_.max_acc, param_.min_acc, param_.max_w,
                            param_.min_w, direction);
    al_costs_vec_[i]->SetConstraint(control_constrain);
  }
}

void ALILQR::SetObsConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& initial_line,
    const std::vector<std::pair<TL::common::math::Vec2d, double>>& obs) {
  for (int i = 0; i < horizon_; ++i) {
    if (nullptr == al_costs_vec_[i]) {
      al_costs_vec_[i] = std::make_shared<ALCost>(param_);
    }
    for (const auto& obs_pair : obs) {
      const auto obs_constrain = std::make_shared<ObsConstrain>();
      if (obs_constrain->IsTooFarToConsider(initial_line.row(i), obs_pair.first,
                                            obs_pair.second)) {
        continue;
      }
      obs_constrain->Init(obs_pair.first, obs_pair.second);
      al_costs_vec_[i]->SetConstraint(obs_constrain);
    }
  }
}

bool ALILQR::GetOptimal(
    const int direction, const Eigen::Ref<const Eigen::MatrixXd>& initial_line,
    const Eigen::Ref<const Eigen::MatrixXd>& ref_line,
    const std::vector<std::pair<TL::common::math::Vec2d, double>>& obs,
    const std::pair<double, double>& pose_relax,
    Eigen::Ref<Eigen::MatrixXd> x_op, Eigen::Ref<Eigen::MatrixXd> u_op) {
  // get init traj based on init_state and init_control_seq
  if (ref_line.rows() < 2) {
    return false;
  }
  horizon_ = static_cast<int>(ref_line.rows()) - 1;
  MatrixType xu_tmp(horizon_ + 1, param_.state_dim + param_.control_dim);
  al_costs_vec_ = std::vector<std::shared_ptr<ALCost>>(horizon_ + 1);
  xu_tmp.setZero();
  auto x_tmp = xu_tmp.leftCols(param_.state_dim);
  auto u_tmp = xu_tmp.topRows(horizon_).rightCols(param_.control_dim);
  const auto& init_state = initial_line.row(0).head(param_.state_dim);
  Eigen::MatrixXd init_control_seq =
      initial_line.block(0, param_.state_dim, horizon_, param_.control_dim);

  x_tmp.row(0) = init_state;
  u_tmp.block(0, 0, horizon_, param_.control_dim) = init_control_seq;
  std::vector<MatrixType> K(horizon_,
                            MatrixType(param_.state_dim, param_.control_dim));
  MatrixType d(horizon_, param_.control_dim);
  regularization_.InitRegularization();
  //   Eigen::Ref<const Eigen::VectorXd> end_pose =ref_line.row(ref_line.rows() - 1).transpose();
  const auto& end_pose = ref_line.row(horizon_).head(param_.state_dim);
  SetEndPoseConstraint(end_pose, pose_relax);
  SetControlBound(direction);
  UNUSED(obs);
  // SetObsConstraint(initial_line, obs);
  ADEBUG << "settle done all constraints ";
  // outter loop for ILQR
  double pre_cost = INFINITY;
  double cur_cost = INFINITY;
  Simulate(init_state, init_control_seq, x_tmp);
  // init
  x_op = x_tmp;
  u_op = init_control_seq;
  double pre_constraint_vio = MaxConstraintViolation(xu_tmp);
  ADEBUG << "start iteration " << pre_constraint_vio;
  for (int i = 0; i < param_.max_outter_iter; ++i) {
    ADEBUG << "outter iteration : " << i << " param_.max_outter_iter "
           << param_.max_outter_iter;
    pre_cost = CaculateCost(ref_line, xu_tmp);
    cur_cost = pre_cost;
    if (cur_cost < param_.accuracy) {
      ADEBUG << "success";
      return true;
    }
    // inner loop for LQR
    for (int j = 0; j < param_.max_inner_iter; ++j) {
      ADEBUG << "inner iteration: " << j;
      EvaluateDerivative(ref_line, xu_tmp);
      // backward
      if (!Backward(x_op, u_op, &K, d)) {
        AERROR << "Backward failed";
        return false;
      }
      // forward, update u_tmp and x_tmp
      Forward(init_state, K, d, u_tmp, x_tmp);

      // get cost
      cur_cost = CaculateCost(ref_line, xu_tmp);
      ADEBUG << " CaculateCost " << cur_cost;

      // compute constrain vio
      double constraint_vio = MaxConstraintViolation(xu_tmp);
      const double cost_diff = cur_cost - pre_cost;
      const double constraint_vio_diff = constraint_vio - pre_constraint_vio;
      // update
      if (cost_diff < 0 || constraint_vio_diff < 0) {
        pre_cost = cur_cost;
        pre_constraint_vio = constraint_vio;
        x_op = x_tmp;
        u_op = u_tmp;
        if (cost_diff >= 0 && !regularization_.is_const_rho) {
          regularization_.IncreaseRegularization();
          ADEBUG << "increase rho to " << regularization_.rho;
        }
      } else {
        x_tmp = x_op;
        u_tmp = u_op;
        ADEBUG << "-------------------- shoot over ---------------------" << j;
        break;
      }
      // TODO(sim): use 'and' or 'or' instead
      if (fabs(cost_diff) < 1.0e-1 && fabs(constraint_vio_diff) < 1e-3) {
        ADEBUG << "---------------- Tolerance reached -----------------" << j;
        break;
      }
    }
    // update dual variable
    UpdateDualVar(xu_tmp);
    ADEBUG << "outter iteration finish ------------ " << i
           << " param_.max_outter_iter " << param_.max_outter_iter;
  }
  return MaxConstraintViolation(xu_tmp) < param_.accuracy;
}

void ALILQR::GetInitial(const bool is_lat_spot, const double x1,
                        const double y1, const double theta1, const double x2,
                        const double y2, const double theta2,
                        Eigen::Ref<Eigen::MatrixXd> xu) const {
  if (!is_lat_spot) {
    return;
  }
  auto horizon = xu.rows() - 1;
  Eigen::VectorXd coef;
  ComputeCubicCurveCoefficients(x1, y1, std::tan(theta1), x2, y2,
                                std::tan(theta2), &coef);
  const double dx = (x2 - x1) / static_cast<double>(horizon);
  double y = 0.0;
  double x = x1;
  for (int i = 0; i <= horizon; ++i) {
    x = x1 + i * dx;
    y = ((coef[3] * x + coef[2]) * x + coef[1]) * x + coef[0];
    xu.row(i) << x, y, 0.0, 0.0, 0.0, 0.0;
  }
  for (int i = 1; i <= horizon; ++i) {
    double dx = xu(i - 1, 0) - xu(i, 0);
    double dy = xu(i - 1, 1) - xu(i, 1);
    xu(i - 1, 2) = -std::hypot(dx, dy) / param_.dt;
    xu(i, 3) = std::atan2(dy, dx);
  }
  xu(0, 3) = theta1;
  xu(horizon, 2) = xu(horizon - 1, 2);

  for (int i = 0; i < horizon; ++i) {
    xu(i, 4) = (xu(i + 1, 2) - xu(i, 2)) / param_.dt;
    xu(i, 5) =
        TL::common::math::AngleDiff(xu(i, 3), xu(i + 1, 3)) / param_.dt;
  }
}

void ALILQR::Simulate(const Eigen::Ref<const Eigen::VectorXd>& init_state,
                      Eigen::Ref<Eigen::MatrixXd> u,
                      Eigen::Ref<Eigen::MatrixXd> x) {
  if (x.rows() != horizon_ + 1 || x.cols() != param_.state_dim) {
    return;
  }
  x.row(0) = init_state;
  Eigen::VectorXd next_state = x.row(1).transpose();
  Eigen::VectorXd cur_control;

  for (int i = 0; i < horizon_; i++) {
    Eigen::Ref<const Eigen::VectorXd> cur_state = x.row(i).transpose();
    cur_control = u.row(i).transpose();
    kinematic_model_ptr_->Simulate(cur_state, cur_control, next_state);
    x.row(i + 1) = next_state.transpose();
    u.row(i) = cur_control.transpose();
  }
}

bool ALILQR::Backward(const Eigen::Ref<const Eigen::MatrixXd>& x,
                      const Eigen::Ref<const Eigen::MatrixXd>& u,
                      std::vector<MatrixType>* const K,
                      Eigen::Ref<Eigen::MatrixXd> d) {
  if (u.rows() != horizon_ || u.cols() != param_.control_dim) {
    return false;
  }
  if (al_costs_vec_.size() != horizon_ + 1) {
    return false;
  }

  // TODO(sim): optimize this
  Eigen::VectorXd Q_x;
  Eigen::VectorXd Q_u;
  Eigen::MatrixXd Q_xx;
  Eigen::MatrixXd Q_ux;
  Eigen::MatrixXd Q_uu;
  Eigen::MatrixXd Q_uu_inv;
  Eigen::MatrixXd state_jac =
      Eigen::MatrixXd::Zero(param_.state_dim, param_.state_dim);
  Eigen::MatrixXd control_jac =
      Eigen::MatrixXd::Zero(param_.state_dim, param_.control_dim);
  Eigen::VectorXd V_x = al_costs_vec_[horizon_]->Getdx();    // i+1
  Eigen::MatrixXd V_xx = al_costs_vec_[horizon_]->Getddx();  // i+1
  bool debug = false;
  bool derivative_debug = false;
  const auto is_positive_definite = [](const Eigen::Ref<Eigen::MatrixXd>& mat) {
    const auto row = mat.rows();
    Eigen::MatrixXd sub_matrix(row, row);
    for (int k = 1; k <= row; k++) {
      sub_matrix = mat.block(0, 0, k, k);
      if (sub_matrix.determinant() <= 1e-8) {
        return false;
      }
    }
    return true;
  };
  int i = horizon_ - 1;
  while (i >= 0) {
    if (nullptr == al_costs_vec_[i]) {
      continue;
    }
    kinematic_model_ptr_->StateJacobian(x(i + 1, 3), x(i + 1, 2), u(i, 0),
                                        state_jac);
    kinematic_model_ptr_->ControlJacobian(x(i + 1, 3), control_jac);
    Q_x = al_costs_vec_[i]->Getdx() + state_jac.transpose() * V_x;
    Q_u = al_costs_vec_[i]->Getdu() + control_jac.transpose() * V_x;
    Q_xx =
        al_costs_vec_[i]->Getddx() + state_jac.transpose() * V_xx * state_jac;
    Q_ux = al_costs_vec_[i]->Getdudx() +
           control_jac.transpose() * V_xx * state_jac;
    Q_uu = al_costs_vec_[i]->Getddu() +
           control_jac.transpose() * V_xx * control_jac +
           regularization_.rho * Eigen::Matrix2d::Identity();
    if (!regularization_.is_const_rho &&
        regularization_.CheckRegularization() && !is_positive_definite(Q_uu)) {
      regularization_.IncreaseRegularization();
      ADEBUG << "increase rho to " << regularization_.rho;
      // reset and backward from the end
      i = horizon_ - 1;
      V_x = al_costs_vec_[horizon_]->Getdx();
      V_xx = al_costs_vec_[horizon_]->Getddx();
      continue;
    }
    Q_uu_inv = Q_uu.inverse();
    K->at(i) = -Q_uu_inv * Q_ux;
    d.row(i) = -Q_uu_inv * Q_u;
#ifndef ISORIN
#ifndef ISMDC
    if (derivative_debug) {
      ADEBUG << "----------------- i -----------------" << i;
      ADEBUG << "dx\n" << al_costs_vec_[i]->Getdx();
      ADEBUG << "du\n" << al_costs_vec_[i]->Getdu();
      ADEBUG << "ddx\n" << al_costs_vec_[i]->Getddx();
      ADEBUG << "dux\n" << al_costs_vec_[i]->Getdudx();
      ADEBUG << "ddu\n" << al_costs_vec_[i]->Getddu();
      ADEBUG << "fdx\n" << state_jac;
      ADEBUG << "fdu\n" << control_jac;
      ADEBUG << "V_x\n" << V_x;
      ADEBUG << "V_xx\n" << V_xx;
    }
    if (debug) {
      ADEBUG << "rho\n" << regularization_.rho;
      ADEBUG << "Q_x\n" << Q_x;
      ADEBUG << "Q_u\n" << Q_u;
      ADEBUG << "Q_xx\n" << Q_xx;
      ADEBUG << "Q_ux\n" << Q_ux;
      ADEBUG << "Q_uu\n" << Q_uu;
      ADEBUG << "Q_uu_inv\n" << Q_uu_inv;
      ADEBUG << "K->at(i)\n" << K->at(i);
      ADEBUG << "d.row(i)\n" << d.row(i);
    }
#endif
#endif
    V_xx = Q_xx + K->at(i).transpose() * Q_uu * K->at(i) +
           K->at(i).transpose() * Q_ux + Q_ux.transpose() * K->at(i);
    V_x = Q_x + K->at(i).transpose() * Q_uu * d.row(i).transpose() +
          K->at(i).transpose() * Q_u + Q_ux.transpose() * d.row(i).transpose();
    i--;
  }
  if (!regularization_.is_const_rho && regularization_.CheckRegularization()) {
    regularization_.DecreaseRegularization();
    ADEBUG << "decrease rho to " << regularization_.rho;
  }
  return true;
}

void ALILQR::Forward(const Eigen::Ref<const Eigen::VectorXd>& /*init_state*/,
                     const std::vector<MatrixType>& K,
                     const Eigen::Ref<Eigen::MatrixXd>& d,
                     Eigen::Ref<Eigen::MatrixXd> u,
                     Eigen::Ref<Eigen::MatrixXd> x) {
  if (u.rows() != horizon_ || u.cols() != param_.control_dim ||
      x.rows() != horizon_ + 1 || x.cols() != param_.state_dim) {
    AERROR << "Invalid state vector for forward ";
    return;
  }
  const Eigen::MatrixXd ref_x = x;
  Eigen::VectorXd next_state = x.row(1).transpose();
  Eigen::VectorXd cur_control;

  for (int i = 0; i < horizon_; i++) {
    Eigen::Ref<const Eigen::VectorXd> cur_state = x.row(i).transpose();
    cur_control = u.row(i).transpose() + d.row(i).transpose() +
                  K.at(i) * (x.row(i) - ref_x.row(i)).transpose();
    kinematic_model_ptr_->Simulate(cur_state, cur_control, next_state);
    x.row(i + 1) = next_state.transpose();
    u.row(i) = cur_control.transpose();
  }
}

double ALILQR::CaculateCost(const Eigen::Ref<const Eigen::MatrixXd>& ref_line,
                            const Eigen::Ref<const Eigen::MatrixXd>& xu) {
  double cost = 0;
  for (int i = 0; i < horizon_; i++) {
    const auto diff = xu.row(i) - ref_line.row(i);
    cost += (diff * weight_ * diff.transpose())(0, 0) +
            al_costs_vec_[i]->CalculateConstraintsCost(xu.row(i).transpose());
  }
  return cost;
}

void ALILQR::EvaluateDerivative(
    const Eigen::Ref<const Eigen::MatrixXd>& ref_line,
    const Eigen::Ref<const Eigen::MatrixXd>& xu) {
  for (int i = 0; i <= horizon_; i++) {
    al_costs_vec_[i]->EvaluateDerivative(xu.row(i).transpose(),
                                         ref_line.row(i).transpose());
  }
}

double ALILQR::MaxConstraintViolation(
    const Eigen::Ref<const Eigen::MatrixXd>& xu) {
  double vio = 0.0;
  for (int i = 0; i <= horizon_; i++) {
    vio = std::max(vio, al_costs_vec_[i]->MaxVio(xu.row(i).transpose()));
    ADEBUG << "vio" << vio;
  }
  return vio;
}

void ALILQR::UpdateDualVar(const Eigen::Ref<const Eigen::MatrixXd>& xu) {
  for (int i = 0; i <= horizon_; i++) {
    al_costs_vec_[i]->UpdateDualVar(xu.row(i).transpose());
  }
}
