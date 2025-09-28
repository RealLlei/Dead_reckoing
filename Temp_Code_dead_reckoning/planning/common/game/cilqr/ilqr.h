/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description: ilqr.h
 */
#pragma once
#include <utility>
#include <vector>
#include "planning/common/game/cilqr/constraints.h"
#include "planning/common/game/cilqr/linear_model.h"
#include "planning/common/reference_line_info.h"

namespace TL::planning::game_common {
struct iLQRParam {
  int num_states = 4;
  int num_ctrls = 2;
  int horizon = 20;
  int max_iters = 20;
  double tolerance = 1e-4;
  double lamb_factor = 10.0;
  double max_lamb = 1000.0;
};

class iLQR {
 private:
  LinearModel vehicle_model_ = {};
  Constraints constraints_ = {};
  iLQRParam ilqr_param_ = {};
  ReferenceLineInfo* reference_line_info_ = nullptr;

 public:
  iLQR() = default;

  iLQR(const iLQRParam& ilqr_param, const ModelParam& model_param,
       const ConstraintsParam& constraints_param)
      : ilqr_param_(ilqr_param) {
    vehicle_model_ = LinearModel(model_param);
    constraints_ = Constraints(constraints_param);
  }

  ~iLQR() = default;

  /**
   * @brief set_reference_line_info.
   * @param reference_line_info
   */
  void set_reference_line_info(ReferenceLineInfo* reference_line_info) {
    reference_line_info_ = reference_line_info;
  }

  /**
   * @brief GetNominalTrajectory.
   * @param X_0
   * @param U
   * @return NominalTrajectory
   */
  Eigen::MatrixXd GetNominalTrajectory(const Eigen::VectorXd& X_0,
                                       const Eigen::MatrixXd& U);

  /**
   * @brief BackwardPass.
   * @param X
   * @param U
   * @param obs_traj
   * @param lamb
   * @param ref_traj
   * @return k, K
   */
  std::pair<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>> BackwardPass(
      const Eigen::MatrixXd& X, const Eigen::MatrixXd& U,
      const Eigen::Matrix<double, 4, Eigen::Dynamic>& obs_traj, double lamb,
      const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_traj);

  /**
   * @brief ForwardPass.
   * @param X
   * @param U
   * @param k
   * @param K
   * @param ref_traj
   * @return X_new, U_new
   */
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ForwardPass(
      const Eigen::MatrixXd& X, const Eigen::MatrixXd& U,
      const Eigen::MatrixXd& k, const std::vector<Eigen::MatrixXd>& K);

  /**
   * @brief GetOptimalControlStateSeq.
   * @param X_0
   * @param U_init
   * @param obs_traj
   * @param ref_traj
   * @return X, U
   */
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> GetOptimalControlStateSeq(
      const Eigen::VectorXd& X_0, const Eigen::MatrixXd& U_init,
      const Eigen::MatrixXd& obs_traj,
      const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_traj);
};
}  // namespace TL::planning::game_common
