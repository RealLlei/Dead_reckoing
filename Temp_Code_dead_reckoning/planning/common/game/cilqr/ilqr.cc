/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description: ilqr.cc
 */
#include "planning/common/game/cilqr/ilqr.h"
#include <limits>
#include <utility>
#include <vector>
#include "common/util/util.h"

namespace TL::planning::game_common {

Eigen::MatrixXd iLQR::GetNominalTrajectory(const Eigen::VectorXd& X_0,
                                           const Eigen::MatrixXd& U) {
  if (U.cols() != ilqr_param_.horizon) {
    ADEBUG << "U size error!";
    return {};
  }
  Eigen::MatrixXd X(ilqr_param_.num_states, ilqr_param_.horizon + 1);
  X.col(0) = X_0;
  for (int i = 0; i < ilqr_param_.horizon; ++i) {
    X.col(i + 1) = vehicle_model_.ForwardSimulate(X.col(i), U.col(i));
  }
  return X;
}

std::pair<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>> iLQR::BackwardPass(
    const Eigen::MatrixXd& X, const Eigen::MatrixXd& U,
    const Eigen::Matrix<double, 4, Eigen::Dynamic>& obs_traj, double lamb,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_traj) {
  // 获取Q函数关于状态和控制的导数
  const auto& cost_derivs = constraints_.GetCostDerivatives(
      U, X.block(0, 1, X.rows(), X.cols() - 1), obs_traj, ref_traj);
  const auto& l_x = std::get<0>(cost_derivs);
  const auto& l_xx = std::get<1>(cost_derivs);
  const auto& l_u = std::get<2>(cost_derivs);
  const auto& l_uu = std::get<3>(cost_derivs);
  const auto& l_ux = std::get<4>(cost_derivs);
  std::vector<Eigen::Matrix4d> df_dx(ilqr_param_.horizon);
  for (int i = 0; i < ilqr_param_.horizon; ++i) {
    const auto& df_dx_i = vehicle_model_.GetAMatrix(X(2, i), X(3, i), U(0, i));
    df_dx[i] = df_dx_i;
  }
  std::vector<Eigen::MatrixXd> df_du(ilqr_param_.horizon);
  for (int i = 0; i < ilqr_param_.horizon; ++i) {
    const auto df_du_i = vehicle_model_.GetBMatrix(X(3, i));
    df_du[i] = df_du_i;
  }
  Eigen::VectorXd V_x = l_x.col(l_x.cols() - 1);
  Eigen::MatrixXd V_xx = l_xx[l_xx.size() - 1];
  // 初始化k和K
  Eigen::MatrixXd k(ilqr_param_.num_ctrls, ilqr_param_.horizon);
  std::vector<Eigen::MatrixXd> K(ilqr_param_.horizon);
  // 反向传播
  Eigen::VectorXd Q_x(ilqr_param_.num_states, ilqr_param_.horizon);
  Eigen::VectorXd Q_u(ilqr_param_.num_ctrls, ilqr_param_.horizon);
  std::vector<Eigen::MatrixXd> Q_xx(ilqr_param_.horizon);
  std::vector<Eigen::MatrixXd> Q_ux(ilqr_param_.horizon);
  std::vector<Eigen::MatrixXd> Q_uu(ilqr_param_.horizon);
  for (int i = ilqr_param_.horizon - 1; i >= 0; --i) {
    Q_x = l_x.col(i) + df_dx[i].transpose() * V_x;
    Q_u = l_u.col(i) + df_du[i].transpose() * V_x;
    Q_xx[i] = l_xx[i] + df_dx[i].transpose() * V_xx * df_dx[i];
    Q_ux[i] = l_ux[i] + df_du[i].transpose() * V_xx * df_dx[i];
    Q_uu[i] = l_uu[i] + df_du[i].transpose() * V_xx * df_du[i];

    // 对Q_uu进行特征值分解，并处理负特征值
    Eigen::EigenSolver<Eigen::MatrixXd> esolver(Q_uu[i]);
    Eigen::VectorXd Q_uu_evals = esolver.eigenvalues().real();
    Eigen::MatrixXd Q_uu_evecs = esolver.eigenvectors().eval().real();
    ADEBUG << "###Q_uu: "
           << "\n"
           << Q_uu[i];
    ADEBUG << "###Q_uu_evals: "
           << "\n"
           << Q_uu_evals;
    ADEBUG << "###Q_uu_evecs: "
           << "\n"
           << Q_uu_evecs;
    for (int j = 0; j < Q_uu_evals.size(); ++j) {
      if (Q_uu_evals(j) < 0) {
        Q_uu_evals(j) = 0;
      }
      Q_uu_evals(j) += lamb;
    }
    Eigen::MatrixXd Q_uu_inv = Q_uu_evecs *
                               Q_uu_evals.cwiseInverse().asDiagonal() *
                               Q_uu_evecs.transpose();
    // 计算前馈和反馈项
    k.col(i) = -Q_uu_inv * Q_u;
    K[i] = -Q_uu_inv * Q_ux[i];

    // 更新值函数
    V_x = Q_x - K[i].transpose() * Q_uu[i] * k.col(i);
    V_xx = Q_xx[i] - K[i].transpose() * Q_uu[i] * K[i];
    ADEBUG << "###Q_uu_inv: "
           << "\n"
           << Q_uu_inv;
    ADEBUG << "###V_x: "
           << "\n"
           << V_x;
    ADEBUG << "###V_xx: "
           << "\n"
           << V_xx;
  }

  return {k, K};
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> iLQR::ForwardPass(
    const Eigen::MatrixXd& X, const Eigen::MatrixXd& U,
    const Eigen::MatrixXd& k, const std::vector<Eigen::MatrixXd>& K) {
  Eigen::MatrixXd X_new =
      Eigen::MatrixXd::Zero(ilqr_param_.num_states, ilqr_param_.horizon + 1);
  X_new.col(0) = X.col(0);

  Eigen::MatrixXd U_new =
      Eigen::MatrixXd::Zero(ilqr_param_.num_ctrls, ilqr_param_.horizon);

  for (int i = 0; i < ilqr_param_.horizon; ++i) {
    U_new.col(i) = U.col(i) + k.col(i) + K[i] * (X_new.col(i) - X.col(i));
    X_new.col(i + 1) =
        vehicle_model_.ForwardSimulate(X_new.col(i), U_new.col(i));
  }

  return {X_new, U_new};
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> iLQR::GetOptimalControlStateSeq(
    const Eigen::VectorXd& X_0, const Eigen::MatrixXd& U_init,
    const Eigen::MatrixXd& obs_traj,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_traj) {
  Eigen::MatrixXd U = U_init;
  Eigen::MatrixXd X = GetNominalTrajectory(X_0, U_init);

  double J_old = std::numeric_limits<double>::max();
  double lamb = 1.0;  // 正则化参数
  for (int iter = 0; iter < ilqr_param_.max_iters; ++iter) {
    const auto& [k, K] = BackwardPass(X, U, obs_traj, lamb, ref_traj);
    const auto& [X_new, U_new] = ForwardPass(X, U, k, K);
    double J_new = constraints_.GetTotalCost(X_new, U_new, ref_traj);
    if (J_new < J_old) {
      X = X_new;
      U = U_new;
      lamb /= ilqr_param_.lamb_factor;
      if (std::abs(J_old - J_new) < ilqr_param_.tolerance) {
        ADEBUG << "Tolerance reached";
        break;
      }
    } else {
      lamb *= ilqr_param_.lamb_factor;
      if (lamb > ilqr_param_.max_lamb) {
        break;
      }
    }
    J_old = J_new;
  }
  return {X, U};
}

}  // namespace TL::planning::game_common
