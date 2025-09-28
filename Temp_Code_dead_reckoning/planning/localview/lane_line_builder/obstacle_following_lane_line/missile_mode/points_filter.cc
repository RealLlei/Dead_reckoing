/*
 * Copyright (c) TL auto Co., Ltd. 2023-2024. All rights reserved.
 */

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/points_filter.h"
#include <algorithm>
#include <cstddef>
#include <utility>

#include "Eigen/LU"
#include "common/math/curve_fitting.h"
#include "common/math/math_utils.h"

namespace TL {
namespace planning {
namespace missilelane {

using Matrix = Eigen::MatrixXd;
using TL::common::math::Clamp;
using TL::common::math::FitPolynomial;

PointsFilter::PointsFilter(
    const planning::PerceptionMapConfig& config,
    const std::shared_ptr<MissileVehicleState>& vehicle_state)
    : config_(config), vehicle_state_(vehicle_state) {}

Status PointsFilter::Init() {
  LoadKalmanGainScheduler(config_.lanemarker_filter_config());
  return Status::OK();
}

void PointsFilter::LoadKalmanGainScheduler(
    const planning::LanemarkerFilterConfig& filter_conf) {
  const auto& length_gain_scheduler =
      filter_conf.lanemarker_length_gain_scheduler();
  const auto& diff_length_gain_scheduler =
      filter_conf.lanemarker_difflength_gain_scheduler();
  ADEBUG << "Kalman gain scheduler loaded";
  Interpolation1D::DataType xy1{};
  Interpolation1D::DataType xy2{};
  for (const auto& scheduler : length_gain_scheduler.scheduler()) {
    xy1.emplace_back(std::make_pair(scheduler.length(), scheduler.ratio()));
  }
  for (const auto& scheduler : diff_length_gain_scheduler.scheduler()) {
    xy2.emplace_back(std::make_pair(scheduler.length(), scheduler.ratio()));
  }
  // length_interpolation_.reset(new Interpolation1D);
  length_interpolation_ = std::make_unique<Interpolation1D>();
  ACHECK(length_interpolation_->Init(xy1))
      << "Fail to load length gain scheduler";

  // diff_length_interpolation_.reset(new Interpolation1D);
  diff_length_interpolation_ = std::make_unique<Interpolation1D>();
  ACHECK(diff_length_interpolation_->Init(xy2))
      << "Fail to load diff length gain scheduler";
}

std::vector<Vec2d> PointsFilter::Filter(const LaneMarker& lanemarker) {
  has_no_history_points_ = history_points_.size() <= 2;
  double view_range = lanemarker.view_range();
  Matrix pre_points_x;
  Matrix pre_points_y;
  Matrix measure_points_y;
  std::vector<Vec2d> output_points;
  if (has_no_history_points_) {
    CreatInitLaneMarkerPoints(lanemarker, &history_points_);
    ADEBUG << "has no left history points!";
    output_points = history_points_;
  } else {
    if (!DoPredictUpdate(view_range, &history_points_, &pre_points_x,
                         &pre_points_y)) {
      ADEBUG << "Do left predictupdate failed!";
      return std::vector<Vec2d>{};
    }
    DoMeasureUpdate(lanemarker, pre_points_x, &measure_points_y);
    Matrix left_kalman_gain;
    bool reset{false};
    bool spl_flag{false};
    auto v_spd = vehicle_state_->spd();
    DoKalmanGainUpdate(pre_points_x, pre_points_y, measure_points_y, view_range,
                       reset, v_spd, spl_flag, &output_points,
                       &left_kalman_gain);
  }
  return output_points;
}

bool PointsFilter::DoPredictUpdate(double view_range,
                                   std::vector<Vec2d>* history_points,
                                   Eigen::MatrixXd* pre_points_x,
                                   Eigen::MatrixXd* pre_points_y) {
  const double step = config_.lanemarker_filter_config().step_distance();
  double start_range = -config_.lanemarker_back_length();
  double end_range = view_range;
  if (history_points->size() < 3) {
    ADEBUG << "points_size: " << history_points->size() << ", too low!!!";
    return false;
  }
  const int N = 3;
  std::vector<double> coff = FitPolynomial<N>(*history_points);
  history_points->clear();
  int max_index = floor((end_range - start_range) / step + 1);
  for (int i = 0; i < max_index; i++) {
    auto x = i * step + start_range;
    double y = coff[0] + coff[1] * x + coff[2] * x * x + coff[3] * x * x * x;
    history_points->push_back(Vec2d(x, y));
  }

  ADEBUG << "-------------after adjust history POINTS ----------------";
  for (size_t i = 0; i < history_points->size(); i++) {
    ADEBUG << "index[" << i << "], x = " << history_points->at(i).x()
           << ", y = " << history_points->at(i).y();
  }
  int point_size = static_cast<int>(history_points->size());
  Matrix buffer_point = Matrix::Zero(3, point_size);
  Matrix buffer_hist_point = Matrix::Zero(3, point_size);
  *pre_points_x = Matrix::Zero(1, point_size);
  *pre_points_y = Matrix::Zero(1, point_size);
  for (int i = 0; i < point_size; i++) {
    buffer_point(0, i) = history_points->at(i).x();
    buffer_point(1, i) = history_points->at(i).y();
    buffer_point(2, i) = 1;
  }
  buffer_hist_point = vehicle_state_->TransMatrix() * buffer_point;
  for (int i = 0; i < point_size; i++) {
    (*pre_points_x)(0, i) = buffer_hist_point(0, i);
    (*pre_points_y)(0, i) = buffer_hist_point(1, i);
  }
  return true;
}

bool PointsFilter::DoKalmanGainUpdate(
    const Eigen::MatrixXd& pre_points_x, const Eigen::MatrixXd& pre_points_y,
    const Eigen::MatrixXd& measure_points_y, const double view_range,
    const bool reset, const double speed, const bool splitlane_flag,
    std::vector<Vec2d>* esti_points, Eigen::MatrixXd* kalman_gain) {
  const int point_size = pre_points_x.cols();  // NOLINT
  ADEBUG << "pre_points_size = " << point_size;
  Matrix kal_gc = Matrix::Constant(1, point_size, 1);
  if ((speed > config_.lanemarker_filter_config().vehicle_speed_thd()) &&
      !reset) {
    const double k_length =
        std::pow(length_interpolation_->Interpolate(view_range), 2);
    ADEBUG << "view_range: " << view_range
           << ", k_length*k_length: " << k_length;
    for (int i = 0; i < point_size; i++) {
      // const double k_diff_length = diff_length_interpolation_->Interpolate(
      //     view_range - pre_points_x(0, i));
      kal_gc(0, i) = Clamp(((5 * i + 1.0) / point_size), 0.1, 1.0);
      // kal_gc(0, i) = 0.9;
      // AERROR << "diff_length_k["<< i << "]: " << k_diff_length << ", kal_gc:
      // " << kal_gc(0,i);
    }
  }
  Matrix kal_gc_tmp;
  if (splitlane_flag) {
    // 将所有值设置为1
    kal_gc_tmp = Matrix::Constant(1, point_size, 0);
  } else {
    kal_gc_tmp = kal_gc;
  }
  *kalman_gain = kal_gc_tmp;
  for (int i = 0; i < pre_points_x.size(); i++) {
    double x = pre_points_x(0, i);
    // y = pre * (1 - k) + now * k;
    double y = pre_points_y(0, i) * (1 - kal_gc_tmp(0, i)) +
               kal_gc_tmp(0, i) * measure_points_y(0, i);
    esti_points->push_back(Vec2d(x, y));
  }
  ADEBUG << "in gain esti_points = " << esti_points->size();
  return true;
}

void PointsFilter::CreatInitLaneMarkerPoints(const LaneMarker& input_lanemarker,
                                             std::vector<Vec2d>* history_esti) {
  history_esti->clear();
  const double lanemarker_size =
      Clamp(input_lanemarker.view_range(),
            config_.lanemarker_filter_config().low_viewrange(),
            config_.lanemarker_filter_config().upper_viewrange());
  const double back_start = config_.lanemarker_back_length();
  const double step = config_.lanemarker_filter_config().step_distance();
  int max_index = floor((lanemarker_size + back_start) / step + 1);
  for (int i = 0; i < max_index; i++) {
    double x = i * step - back_start;
    double y = input_lanemarker.c0_position() +
               input_lanemarker.c1_heading_angle() * x +
               input_lanemarker.c2_curvature() * x * x +
               input_lanemarker.c3_curvature_derivative() * x * x * x;
    history_esti->push_back(Vec2d(x, y));
  }
  ADEBUG << "Init lanemarker points size: " << history_esti->size()
         << ",start_point_x: " << history_esti->front().x()
         << ", start_point_y: " << history_esti->front().y()
         << "; end_point_x: " << history_esti->back().x()
         << ", end_point_y: " << history_esti->back().y();
}

bool PointsFilter::DoMeasureUpdate(const LaneMarker& copy_lanemarker,
                                   const Eigen::MatrixXd& pre_points_x,
                                   Eigen::MatrixXd* measure_points_y) {
  measure_points_y->resize(1, pre_points_x.cols());
  double x_2 = 0.0;
  double x_3 = 0.0;
  for (int i = 0; i < pre_points_x.cols(); i++) {
    x_2 = pre_points_x(0, i) * pre_points_x(0, i);
    x_3 = x_2 * pre_points_x(0, i);
    (*measure_points_y)(0, i) =
        copy_lanemarker.c0_position() +
        copy_lanemarker.c1_heading_angle() * pre_points_x(0, i) +
        copy_lanemarker.c2_curvature() * x_2 +
        copy_lanemarker.c3_curvature_derivative() * x_3;
  }
  return true;
}

}  // namespace missilelane
}  // namespace planning
}  // namespace TL
