/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  common control mpc osqp interface
 * Author: ROC
 */

#include "control/common/mpc_osqp.h"
#include <iomanip>

namespace TL {
namespace common {
namespace math {
MpcOsqp::MpcOsqp(const Eigen::MatrixXd& matrix_a,
                 const Eigen::MatrixXd& matrix_b,
                 const Eigen::MatrixXd& matrix_q,
                 const Eigen::MatrixXd& matrix_r,
                 const Eigen::MatrixXd& matrix_initial_x,
                 const Eigen::MatrixXd& matrix_u_lower,
                 const Eigen::MatrixXd& matrix_u_upper,
                 const Eigen::MatrixXd& matrix_x_lower,
                 const Eigen::MatrixXd& matrix_x_upper,
                 const Eigen::MatrixXd& matrix_x_ref, const int max_iter,
                 const int horizon, const double eps_abs,
                 const double front_angle_prev, const double increment_limit,
                 const size_t horizon_start_control,
                 const Eigen::VectorXd& matrix_u_incre_upper,
                 const Eigen::VectorXd& matrix_u_incre_lower)
    : matrix_a_(matrix_a),
      matrix_b_(matrix_b),
      matrix_q_(matrix_q),
      matrix_r_(matrix_r),
      matrix_initial_x_(matrix_initial_x),
      matrix_u_lower_(matrix_u_lower),
      matrix_u_upper_(matrix_u_upper),
      matrix_x_lower_(matrix_x_lower),
      matrix_x_upper_(matrix_x_upper),
      matrix_x_ref_(matrix_x_ref),
      max_iteration_(max_iter),
      horizon_(horizon),
      eps_abs_(eps_abs),
      front_wheel_turn_angle_prev_(front_angle_prev),
      front_wheel_angle_increment_(increment_limit),
      horizon_start_control_(horizon_start_control),
      control_increment_limit_upper(matrix_u_incre_upper),
      control_increment_limit_lower(matrix_u_incre_lower) {
  state_dim_ = matrix_b.rows();
  control_dim_ = matrix_b.cols();
  ADEBUG << "state_dim " << state_dim_;
  ADEBUG << "control_dim " << control_dim_;
  num_param_ = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
}

void MpcOsqp::CalculateKernel(std::vector<c_float>* P_data,
                              std::vector<c_int>* P_indices,
                              std::vector<c_int>* P_indptr) {
  // col1:(row,val),...; col2:(row,val),....; ...
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(num_param_);
  size_t value_index = 0;
  // state and terminal state
  for (size_t i = 0; i <= horizon_; ++i) {
    for (size_t j = 0; j < state_dim_; ++j) {
      // (row, val)
      if (i < horizon_start_control_) {
        matrix_q_ = Eigen::MatrixXd::Ones(state_dim_, state_dim_) * 0.000001;
      }
      columns[i * state_dim_ + j].emplace_back(i * state_dim_ + j,
                                               matrix_q_(j, j));
      ++value_index;
    }
  }
  // control
  const size_t state_total_dim = state_dim_ * (horizon_ + 1);
  for (size_t i = 0; i < horizon_; ++i) {
    for (size_t j = 0; j < control_dim_; ++j) {
      // (row, val)
      columns[i * control_dim_ + j + state_total_dim].emplace_back(
          state_total_dim + i * control_dim_ + j, matrix_r_(j, j));
      ++value_index;
    }
  }
  CHECK_EQ(value_index, num_param_);

  int ind_p = 0;
  for (size_t i = 0; i < num_param_; ++i) {
    // TODO(SHU) Check this
    P_indptr->emplace_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      P_data->emplace_back(row_data_pair.second);    // val
      P_indices->emplace_back(row_data_pair.first);  // row
      ++ind_p;
    }
  }
  P_indptr->emplace_back(ind_p);
}

// reference is always zero
void MpcOsqp::CalculateGradient() {
  // populate the gradient vector
  gradient_ = Eigen::VectorXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  for (size_t i = 0; i < horizon_ + 1; i++) {
    if (i < horizon_start_control_) {
      matrix_q_ = Eigen::MatrixXd::Ones(state_dim_, state_dim_) * 0.000001;
    }
    gradient_.block(i * state_dim_, 0, state_dim_, 1) =
        -1.0 * matrix_q_ *
        matrix_x_ref_.block(i * state_dim_, 0, state_dim_, 1);
  }
  ADEBUG << "Gradient_mat";
  // ADEBUG << gradient_;
}

// equality constraints x(k+1) = A*x(k)
void MpcOsqp::CalculateEqualityConstraint(std::vector<c_float>* A_data,
                                          std::vector<c_int>* A_indices,
                                          std::vector<c_int>* A_indptr) {
  static constexpr double kEpsilon = 1e-6;
  // block matrix
  Eigen::MatrixXd matrix_constraint = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) +
          control_dim_ * horizon_ + horizon_ - 1,
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_);
  Eigen::MatrixXd state_identity_mat = Eigen::MatrixXd::Identity(
      state_dim_ * (horizon_ + 1), state_dim_ * (horizon_ + 1));
  // ADEBUG << "state_identity_mat" << state_identity_mat;

  matrix_constraint.block(0, 0, state_dim_ * (horizon_ + 1),
                          state_dim_ * (horizon_ + 1)) =
      -1 * state_identity_mat;
  ADEBUG << "matrix_constraint";
  // ADEBUG << matrix_constraint;

  Eigen::MatrixXd control_identity_mat =
      Eigen::MatrixXd::Identity(control_dim_, control_dim_);

  for (size_t i = 0; i < horizon_; i++) {
    matrix_constraint.block((i + 1) * state_dim_, i * state_dim_, state_dim_,
                            state_dim_) = matrix_a_;
  }

  ADEBUG << "matrix_constraint with A";
  // ADEBUG << matrix_constraint;

  for (size_t i = 0; i < horizon_; i++) {
    matrix_constraint.block((i + 1) * state_dim_,
                            i * control_dim_ + (horizon_ + 1) * state_dim_,
                            state_dim_, control_dim_) = matrix_b_;
  }
  ADEBUG << "matrix_constraint with B";
  // ADEBUG << matrix_constraint;

  Eigen::MatrixXd all_identity_mat =
      Eigen::MatrixXd::Identity(num_param_, num_param_);

  matrix_constraint.block(state_dim_ * (horizon_ + 1), 0, num_param_,
                          num_param_) = all_identity_mat;
  ADEBUG << "matrix_constraint with I";
  // ADEBUG << matrix_constraint;

  Eigen::MatrixXd control_increment_mat = Eigen::MatrixXd::Zero(1, 3);
  control_increment_mat(0, 0) = 1;
  control_increment_mat(0, 2) = -1;
  for (size_t i = 0; i < horizon_ - 1; i++) {
    matrix_constraint.block(
        2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_ + i,
        state_dim_ * (horizon_ + 1) + i * control_dim_, 1, 3) =
        control_increment_mat;
  }

  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(num_param_ + 1);
  int value_index = 0;
  // state and terminal state
  for (size_t i = 0; i < num_param_; ++i) {  // col
    for (size_t j = 0;
         j < num_param_ + state_dim_ * (horizon_ + 1) + horizon_ - 1;
         ++j)  // row
      if (std::fabs(matrix_constraint(j, i)) > kEpsilon) {
        // (row, val)
        columns[i].emplace_back(j, matrix_constraint(j, i));
        ++value_index;
      }
  }
  ADEBUG << "value_index";
  // ADEBUG << value_index;
  int ind_A = 0;
  for (size_t i = 0; i < num_param_; ++i) {
    A_indptr->emplace_back(ind_A);
    for (const auto& row_data_pair : columns[i]) {
      A_data->emplace_back(row_data_pair.second);    // value
      A_indices->emplace_back(row_data_pair.first);  // row
      ++ind_A;
    }
  }
  A_indptr->emplace_back(ind_A);
}

void MpcOsqp::CalculateConstraintVectors() {
  // evaluate the lower and the upper inequality vectors
  Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  for (size_t i = 0; i < horizon_; i++) {
    lowerInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                          control_dim_, 1) = matrix_u_lower_;
    upperInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                          control_dim_, 1) = matrix_u_upper_;
  }
  ADEBUG << " matrix_u_lower_";
  for (size_t i = 0; i < horizon_ + 1; i++) {
    lowerInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_lower_;
    upperInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_upper_;
  }
  ADEBUG << " matrix_x_lower_";

  // evaluate the lower and the upper equality vectors
  Eigen::VectorXd lowerEquality =
      Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1), 1);
  Eigen::VectorXd upperEquality =
      Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1), 1);
  Eigen::VectorXd m = matrix_initial_x_;

  lowerEquality.block(0, 0, state_dim_, 1) = -1 * m;

  upperEquality.block(0, 0, state_dim_, 1) = -1 * m;
  ADEBUG << " matrix_initial_x_";

  // merge inequality and equality vectors
  lowerBound_ = Eigen::MatrixXd::Zero(
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_ + horizon_ - 1,
      1);
  lowerBound_ << lowerEquality, lowerInequality, control_increment_limit_lower;
  ADEBUG << " lowerBound_ ";
  upperBound_ = Eigen::MatrixXd::Zero(
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_ + horizon_ - 1,
      1);
  upperBound_ << upperEquality, upperInequality, control_increment_limit_upper;
  Eigen::VectorXd end_state =
      matrix_x_ref_.block(horizon_ * state_dim_, 0, state_dim_, 1);
  // lowerBound_.block((2 * horizon_ + 1) * state_dim_, 0, 1, 1) =
  //     end_state.block(0, 0, 1, 1);
  // upperBound_.block((2 * horizon_ + 1) * state_dim_, 0, 1, 1) =
  //     end_state.block(0, 0, 1, 1);
  lowerBound_(2 * state_dim_ * (horizon_ + 1), 0) =
      front_wheel_turn_angle_prev_ - front_wheel_angle_increment_;
  upperBound_(2 * state_dim_ * (horizon_ + 1), 0) =
      front_wheel_turn_angle_prev_ + front_wheel_angle_increment_;

  ADEBUG << " upperBound_";
}

OSQPSettings* MpcOsqp::Settings() {
  // default setting
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  if (settings == nullptr) {
    return nullptr;
  } else {
    osqp_set_default_settings(settings);
    settings->polish = true;
    settings->scaled_termination = true;
    settings->verbose = false;
    settings->max_iter = max_iteration_;
    settings->eps_abs = eps_abs_;
    return settings;
  }
}

OSQPData* MpcOsqp::Data() {
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  size_t kernel_dim = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
  size_t num_affine_constraint =
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_ + horizon_ - 1;
  if (data == nullptr) {
    return nullptr;
  } else {
    data->n = kernel_dim;
    data->m = num_affine_constraint;
    std::vector<c_float> P_data;
    std::vector<c_int> P_indices;
    std::vector<c_int> P_indptr;
    ADEBUG << "before CalculateKernel";
    CalculateKernel(&P_data, &P_indices, &P_indptr);

    ADEBUG << "CalculateKernel done";
    data->P =
        csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
                   CopyData(P_indices), CopyData(P_indptr));
    ADEBUG << "Get P matrix";
    data->q = gradient_.data();
    ADEBUG << "before CalculateEqualityConstraint";
    std::vector<c_float> A_data;
    std::vector<c_int> A_indices;
    std::vector<c_int> A_indptr;
    CalculateEqualityConstraint(&A_data, &A_indices, &A_indptr);
    ADEBUG << "CalculateEqualityConstraint done";
    data->A =
        csc_matrix(state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) +
                       control_dim_ * horizon_ + horizon_ - 1,
                   kernel_dim, A_data.size(), CopyData(A_data),
                   CopyData(A_indices), CopyData(A_indptr));
    ADEBUG << "Get A matrix";
    data->l = lowerBound_.data();
    data->u = upperBound_.data();
    return data;
  }
}

void MpcOsqp::FreeData(OSQPData* data) {
  c_free(data->A);
  c_free(data->P);
  c_free(data);
}

bool MpcOsqp::Solve(std::vector<double>* control_cmd,
                    TL::control::SimpleMPCDebug* debug) {
  ADEBUG << "Before Calc Gradient";
  CalculateGradient();
  ADEBUG << "After Calc Gradient";
  CalculateConstraintVectors();
  ADEBUG << "MPC2Matrix";

  OSQPData* data = Data();
  ADEBUG << "OSQP data done";
  // ADEBUG << "OSQP data n " << data->n;
  // ADEBUG << "OSQP data m " << data->m;
  // for (int i = 0; i < data->n; ++i) {
  //   ADEBUG << "OSQP data q " << i << ":" << (data->q)[i];
  // }
  // ADEBUG << "OSQP data l " << data->l;
  // for (int i = 0; i < data->m; ++i) {
  //   ADEBUG << "OSQP data l " << i << ":" << (data->l)[i];
  // }
  // ADEBUG << "OSQP data u " << data->u;
  // for (int i = 0; i < data->m; ++i) {
  //   ADEBUG << "OSQP data u " << i << ":" << (data->u)[i];
  // }

  OSQPSettings* settings = Settings();
  ADEBUG << "OSQP setting done";
  OSQPWorkspace* osqp_workspace = nullptr;
  osqp_setup(&osqp_workspace, data, settings);
  // osqp_workspace = osqp_setup(data, settings);
  ADEBUG << "OSQP workspace ready";
  osqp_solve(osqp_workspace);

  auto status = osqp_workspace->info->status_val;
  ADEBUG << "status: " << status;
  // check status
  if (status < 0 || (status != 1 && status != 2)) {
    AERROR << "failed optimization status:\t" << osqp_workspace->info->status;
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);
    return false;
  } else if (osqp_workspace->solution == nullptr) {
    AERROR << "The solution from OSQP is nullptr";
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);
    return false;
  }
  size_t first_control = state_dim_ * (horizon_ + 1);
  // must be less than 50, or it can't get the value.
  size_t horizon_limit = horizon_ > 25 ? 25 : horizon_;
  for (size_t i = 0; i < control_dim_ * horizon_limit; ++i) {
    control_cmd->at(i) = osqp_workspace->solution->x[i + first_control];
    debug->add_control_cmd_calculate(control_cmd->at(i));
  }

  // Eigen::MatrixXd tmp_control_vector = Eigen::MatrixXd::Zero(control_dim_,
  // 1); Eigen::MatrixXd tmp_state = matrix_initial_x_;
  // ADEBUG << "init_state:\n" << tmp_state;
  // for (size_t i = 0; i < horizon_; ++i) {
  //  tmp_control_vector(0, 0) =
  //      osqp_workspace->solution->x[first_control + i * 2];
  //  tmp_control_vector(1, 0) =
  //      osqp_workspace->solution->x[first_control + i * 2 + 1];
  //  tmp_state = matrix_a_ * tmp_state + matrix_b_ * tmp_control_vector;
  //  ADEBUG << "control_cmd0:" << tmp_control_vector(0, 0)
  //         << "   control_cmd1:" << tmp_control_vector(1, 0)
  //         << "   tmp_state:" << tmp_state(0, 0);
  //}

  ADEBUG << "solved result ";
  auto x = osqp_workspace->solution->x;
  for (int i = 0; i <= horizon_; i++) {
    //   ADEBUG << "out_state " << std::setw(2) << i << " " << std::setw(8)
    //          << x[i * state_dim_ + 0] << " " << std::setw(8)
    //          << x[i * state_dim_ + 1] << " " << std::setw(8)
    //          << x[i * state_dim_ + 2] << " " << std::setw(8)
    //          << x[i * state_dim_ + 3] << " " << std::setw(8)
    //          << x[i * state_dim_ + 4] << " " << std::setw(8)
    //          << x[i * state_dim_ + 5] << " control " << std::setw(8)
    //          << x[state_dim_ * (horizon_ + 1) + i * control_dim_] << " "
    //          << std::setw(8)
    //          << x[state_dim_ * (horizon_ + 1) + i * control_dim_ + 1];
    auto state_out = debug->add_calculate_state();
    state_out->set_y(x[i * state_dim_ + 0]);
    state_out->set_y_derivative(x[i * state_dim_ + 1]);
    state_out->set_theta(x[i * state_dim_ + 2]);
    state_out->set_kappa(x[i * state_dim_ + 3]);
    state_out->set_s(x[i * state_dim_ + 4]);
    state_out->set_x_derivative(x[i * state_dim_ + 5]);
  }

  for (int i = 0; i <= horizon_; i++) {
    //   ADEBUG << "reference " << std::setw(2) << i << " " << std::setw(8)
    //          << matrix_x_ref_(i * state_dim_ + 0, 0) << " " << std::setw(8)
    //          << matrix_x_ref_(i * state_dim_ + 1, 0) << " " << std::setw(8)
    //          << matrix_x_ref_(i * state_dim_ + 2, 0) << " " << std::setw(8)
    //          << matrix_x_ref_(i * state_dim_ + 3, 0) << " " << std::setw(8)
    //          << matrix_x_ref_(i * state_dim_ + 4, 0) << " " << std::setw(8)
    //          << matrix_x_ref_(i * state_dim_ + 5, 0);
    auto state_in = debug->add_reference_state();
    state_in->set_y(matrix_x_ref_(i * state_dim_ + 0, 0));
    state_in->set_y_derivative(matrix_x_ref_(i * state_dim_ + 1, 0));
    state_in->set_theta(matrix_x_ref_(i * state_dim_ + 2, 0));
    state_in->set_kappa(matrix_x_ref_(i * state_dim_ + 3, 0));
    state_in->set_s(matrix_x_ref_(i * state_dim_ + 4, 0));
    state_in->set_x_derivative(matrix_x_ref_(i * state_dim_ + 5, 0));
  }
  // ADEBUG << "init state " << matrix_initial_x_(0, 0) << " "
  //        << matrix_initial_x_(1, 0) << " " << matrix_initial_x_(2, 0) << " "
  //        << matrix_initial_x_(3, 0) << " " << matrix_initial_x_(4, 0) << " "
  //        << matrix_initial_x_(5, 0);

  // Cleanup
  osqp_cleanup(osqp_workspace);
  FreeData(data);
  c_free(settings);
  return true;
}

}  // namespace math
}  // namespace common
}  // namespace TL
