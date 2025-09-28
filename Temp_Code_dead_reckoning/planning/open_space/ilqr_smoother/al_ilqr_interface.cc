/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  al_ilqr_interface.cc
 */

#include "planning/open_space/ilqr_smoother/al_ilqr_interface.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <utility>
#include "common/file/log.h"
#include "planning/open_space/ilqr_smoother/al_ilqr.h"

void ALILQR_INTERFACE::Init(
    const double x1, const double y1, const double theta1, const double x2,
    const double y2, const double theta2, const bool has_no_shift,
    const bool is_lat_spot, const int direction,
    const std::vector<std::pair<TL::common::math::LineSegment2d, double>>&
        obstacles_segments_vec) {
  double dist = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
  double acc = direction > 0 ? std::max(0.5 * param_.max_acc, 0.01)
                             : std::min(0.5 * param_.min_acc, -0.01);
  int horizon = std::ceil(std::sqrt(2 * dist / std::fabs(acc)) / param_.dt);
  // horizon = 60;
  xu_ref_ = Eigen::MatrixXd::Zero(horizon + 1, 6);
  if (horizon < 1) {
    AERROR << "horizon must be greater than 0 ";
    return;
  }
  has_no_shift_ = has_no_shift;
  is_lat_spot_ = is_lat_spot;
  pose_relax_ = CalculatePoseRelax(has_no_shift_, x1, y1, theta1, x2, y2,
                                   theta2, obstacles_segments_vec);
  for (int i = 0; i < horizon; i++) {
    xu_ref_.row(i) << x1, y1, 0.0, theta1, acc, 0.0;
  }
  xu_ref_.row(horizon) << x2, y2, 0.0, theta2, 0.0, 0.0;
  xu_initial_ = xu_ref_;
  al_ilqr_->GetInitial(is_lat_spot_, x1, y1, theta1,
                       0.5 * (pose_relax_.first + pose_relax_.second), y2,
                       theta2, xu_initial_);
  direction_ = direction;
  TransObsSegToVec(obstacles_segments_vec, &obs_vec_);
}

void ALILQR_INTERFACE::TransObsSegToVec(
    const std::vector<std::pair<TL::common::math::LineSegment2d, double>>&
        obs_segs,
    std::vector<std::pair<TL::common::math::Vec2d, double>>* const
        obs_vecs) {
  if (nullptr == obs_vecs) {
    return;
  }
  const auto is_same_point = [](const TL::common::math::Vec2d& vec1,
                                const TL::common::math::Vec2d& vec2) {
    constexpr double kEpsilon = 1e-3;
    return fabs(vec1.x() - vec2.x()) < kEpsilon &&
           fabs(vec1.y() - vec2.y()) < kEpsilon;
  };
  obs_vecs->clear();
  TL::common::math::Vec2d last_obs = {0.0, 0.0};
  for (const auto& obs_seg : obs_segs) {
    if (!is_same_point(last_obs, obs_seg.first.start())) {
      obs_vecs->emplace_back(obs_seg.first.start(), obs_seg.second);
    }
    obs_vecs->emplace_back(obs_seg.first.end(), obs_seg.second);
    last_obs = obs_seg.first.end();
  }
}

std::pair<double, double> ALILQR_INTERFACE::CalculatePoseRelax(
    const bool has_no_shift, const double x1, const double y1,
    const double theta1, const double x2, const double y2, const double theta2,
    const std::vector<std::pair<TL::common::math::LineSegment2d, double>>&
        obstacles_segments_vec) {
  double max_relax_x = x2;
  double min_relax_x = x2;
  if (has_no_shift) {
    return {min_relax_x, max_relax_x};
  }
  const double min_R = 5.0;
  const double diff_theta = theta1 - theta2;
  double theta = std::asin((1 + cos(diff_theta) - (y1 - y2) / min_R) / 2);

  const bool is_right_side = std::fabs(theta2) < 1e-3;
  const double kPoseRelaxThreshold =
      std::min(std::max(0.0, is_right_side ? (x2 - x1) : (x1 - x2)) + 3.0, 5.0);
  if (is_right_side) {
    while (min_relax_x > x2 - kPoseRelaxThreshold) {
      if (TL ::common::math::CheckCollisionWithVehiclePolygon2d(
              min_relax_x, y2, theta2, obstacles_segments_vec)) {
        break;
      }
      min_relax_x -= 0.2;
    }
    max_relax_x = std::min(
        0.0, x1 - min_R * (2 * std::sin(theta) - std::sin(diff_theta)));
  } else {
    while (max_relax_x < x2 + kPoseRelaxThreshold) {
      if (TL ::common::math::CheckCollisionWithVehiclePolygon2d(
              max_relax_x, y2, theta2, obstacles_segments_vec)) {
        break;
      }
      max_relax_x += 0.2;
    }
    min_relax_x = std::max(
        0.0, x1 + min_R * (2 * std::sin(theta) - std::sin(diff_theta)));
  }
  if (min_relax_x > max_relax_x) {
    if (is_right_side) {
      max_relax_x = min_relax_x;
    } else {
      min_relax_x = max_relax_x;
    }
  }
  return {min_relax_x, max_relax_x};
}

void ALILQR_INTERFACE::Interpolate(
    double step_size, const Eigen::Ref<const Eigen::MatrixXd>& x_op,
    const Eigen::Ref<const Eigen::MatrixXd>& u_op, std::vector<double>* const x,
    std::vector<double>* const y, std::vector<double>* const theta) {
  if (nullptr == x || nullptr == y || nullptr == theta) {
    return;
  }
  if (x_op.rows() < 2 || x_op.rows() < u_op.rows()) {
    return;
  }

  x->clear();
  y->clear();
  theta->clear();
  x->emplace_back(x_op(0, 0));
  y->emplace_back(x_op(0, 1));
  theta->emplace_back(x_op(0, 3));
  double acc_s = step_size;

  for (int i = 0; i < x_op.rows() - 1; ++i) {
    const double dis = std::sqrt(std::pow(x_op(i, 0) - x_op(i + 1, 0), 2) +
                                 std::pow(x_op(i, 1) - x_op(i + 1, 1), 2));
    while (acc_s < dis) {
      double t = acc_s / dis;
      x->push_back((1 - t) * x_op(i, 0) + t * x_op(i + 1, 0));
      y->push_back((1 - t) * x_op(i, 1) + t * x_op(i + 1, 1));
      theta->push_back((1 - t) * x_op(i, 3) + t * x_op(i + 1, 3));
      acc_s += step_size;
    }
    acc_s -= dis;
  }
  x->emplace_back(x_op(x_op.rows() - 1, 0));
  y->emplace_back(x_op(x_op.rows() - 1, 1));
  theta->emplace_back(x_op(x_op.rows() - 1, 3));
}

bool ALILQR_INTERFACE::GetOptimal(std::vector<double>* const x,
                                  std::vector<double>* const y,
                                  std::vector<double>* const theta) {
  if (!has_no_shift_ && !is_lat_spot_) {
    return false;
  }
  Eigen::MatrixXd x_op =
      Eigen::MatrixXd::Zero(xu_ref_.rows(), param_.state_dim);
  Eigen::MatrixXd u_op =
      Eigen::MatrixXd::Zero(xu_ref_.rows() - 1, param_.control_dim);
  bool ret = al_ilqr_->GetOptimal(direction_, xu_initial_, xu_ref_, obs_vec_,
                                  pose_relax_, x_op, u_op);
  Eigen::RowVectorXd residual =
      (x_op.row(x_op.rows() - 1) - xu_ref_.row(xu_ref_.rows() - 1).head(4));
  Eigen::RowVectorXd valid_residual(3);
  valid_residual << residual(0), residual(1), residual(3);
  double res = valid_residual.cwiseAbs().maxCoeff();

  if (ret) {
    Interpolate(0.1, x_op, u_op, x, y, theta);
    if (!has_no_shift_) {
      if (std::fabs(residual(0)) < 0.2) {
        ADEBUG << "path length is less than 0.2 meter";
        return false;
      }
      double step_size = 0.1;
      double acc_s = step_size;
      double end_x = x_op(x_op.rows() - 1, 0);
      double end_y = x_op(x_op.rows() - 1, 1);
      double end_theta = x_op(x_op.rows() - 1, 3);
      const bool is_end_forward = std::fabs(end_theta) < 1e-3;
      while (acc_s < std::fabs(residual(0))) {
        x->push_back(is_end_forward ? end_x + acc_s : end_x - acc_s);
        y->push_back(end_y);
        theta->push_back(end_theta);
        acc_s += step_size;
      }
      x->push_back(xu_ref_(xu_ref_.rows() - 1, 0));
      y->push_back(end_y);
      theta->push_back(end_theta);
      res = std::max(std::fabs(residual(1)), std::fabs(residual(3)));
    }
    ret = res < param_.accuracy;
  }
  ADEBUG << " res: " << res << " ret " << ret;
  return ret;
}
