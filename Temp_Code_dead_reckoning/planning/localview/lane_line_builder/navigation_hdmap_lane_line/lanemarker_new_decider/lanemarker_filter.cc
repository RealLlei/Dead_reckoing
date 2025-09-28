/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lanemarker_filter.h"

#include <algorithm>
#include <cstddef>
#include <utility>

#include "common/math/curve_fitting.h"
#include "common/math/math_utils.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
using TL::common::math::Clamp;
using Matrix = Eigen::MatrixXd;
using TL::common::math::FitPolynomial;
using TL::common::math::InterpolateVec2dPoints;

LaneMarkerFilter::LaneMarkerFilter(const planning::PerceptionMapConfig& config)
    : config_(config) {}

Status LaneMarkerFilter::Init() {
  LoadKalmanGainScheduler(config_.lanemarker_filter_config());
  return Status::OK();
}

void LaneMarkerFilter::LoadKalmanGainScheduler(
    const planning::LanemarkerFilterConfig& filter_conf) {
  const auto& length_gain_scheduler =
      filter_conf.lanemarker_length_gain_scheduler();
  const auto& diff_length_gain_scheduler =
      filter_conf.lanemarker_difflength_gain_scheduler();
  AINFO << "Kalman gain scheduler loaded";
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

void LaneMarkerFilter::ComputeFilterData(const DeciderData& decider_data) {
  const double v_spd = decider_data.filter_vehicle_state.vehicle_speed_average;
  const double yaw_rate = decider_data.filter_vehicle_state.vehicle_yaw_rate;
  const double ts = config_.main_loop_time();
  CalculateDxyphi(v_spd, yaw_rate, ts);
}

void LaneMarkerFilter::LeftLaneFilter(DeciderData* decider_data) {
  ADEBUG << "LeftLaneFilter Process!"
         << decider_data->original_lanemarkers.header().seq();
  const double v_spd = decider_data->filter_vehicle_state.vehicle_speed_average;
  auto* filter_debug =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lane_filter_info();
  filter_debug->set_dx(dx_);
  filter_debug->set_dy(dy_);
  filter_debug->set_dphi(dphi_);
  auto* left_filter_debug = filter_debug->add_lane_filter_info();
  left_filter_debug->set_name("left_filter_info");
  // const LaneMarker now_lanemarker =
  //     decider_data->copy_lanemarkers.left_lane_marker();
  const LaneMarker now_lanemarker =
      decider_data->copy_lanemarkers.front_left_lane_marker();
  const double view_range = now_lanemarker.view_range();
  const bool reset = decider_data->lane_reset_out.left_lane_reset.is_lane_reset;
  const bool spl_flag =
      decider_data->lane_reset_out.left_lane_reset.split_lane_flag;
  const bool change_flag =
      decider_data->lane_change_out.left_lanechange_mnt_flag;
  left_filter_debug->set_view_range(view_range);
  bool is_left_copy =
      decider_data->lanemarker_lanline_debug->lanemarker_decider_debug()
          .lanemarkers_copy_info()
          .left_is_copy();
  if (decider_data->lane_markers_state.left_lanemarker_state ==
          LaneMarkerState::BAD_LANEMARKER &&
      !is_left_copy) {
    left_history_esti_.clear();
    ADEBUG << "has no left lane marker!"
           << " view_range: " << view_range;
    return;
  }
  ADEBUG << "original left_history_esti_size = " << left_history_esti_.size();
  if (left_history_esti_.size() <= 2 || is_left_copy) {
    CreatInitLaneMarkerPoints(now_lanemarker, &left_history_esti_);
    has_no_left_history_points_ = true;
  } else {
    has_no_left_history_points_ = false;
  }
  Matrix pre_points_x;
  Matrix pre_points_y;
  Matrix measure_points_y;
  std::vector<Vec2d> esti_points;
  if (has_no_left_history_points_) {
    ADEBUG << "has no left history points!";
    esti_points = left_history_esti_;
  } else {
    if (!DoPredictUpdate(view_range, &left_history_esti_, &pre_points_x,
                         &pre_points_y)) {
      AERROR << "Do left predictupdate failed!";
      return;
    }
    DoMeasureUpdate(now_lanemarker, pre_points_x, &measure_points_y);
    Matrix left_kalman_gain;
    DoKalmanGainUpdate(pre_points_x, pre_points_y, measure_points_y, view_range,
                       reset, v_spd, spl_flag, change_flag, &esti_points,
                       &left_kalman_gain);
    SetDebugInfo(left_filter_debug, left_history_esti_, pre_points_x,
                 pre_points_y, measure_points_y, left_kalman_gain,
                 has_no_left_history_points_);

    // AddEstiPoints(now_lanemarker, &esti_points);
    left_history_esti_ = esti_points;
  }
  ADEBUG << "left esti_points_size: " << esti_points.size();
  (*decider_data)
      .lane_marker_points.insert(std::make_pair("left", esti_points));
}

void LaneMarkerFilter::RightLaneFilter(DeciderData* decider_data) {
  ADEBUG << "RightLaneFilter Process!"
         << decider_data->original_lanemarkers.header().seq();
  const double v_spd = decider_data->filter_vehicle_state.vehicle_speed_average;
  auto* right_filter_debug =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lane_filter_info()
          ->add_lane_filter_info();
  right_filter_debug->set_name("right_filter_debug");
  // const LaneMarker now_lanemarker =
  //     decider_data->copy_lanemarkers.right_lane_marker();
  const LaneMarker now_lanemarker =
      decider_data->copy_lanemarkers.front_right_lane_marker();
  const double view_range = now_lanemarker.view_range();
  const bool reset =
      decider_data->lane_reset_out.right_lane_reset.is_lane_reset;
  const bool spl_flag =
      decider_data->lane_reset_out.right_lane_reset.split_lane_flag;
  const bool change_flag =
      decider_data->lane_change_out.right_lanechange_mnt_flag;
  right_filter_debug->set_view_range(view_range);
  bool is_right_copy =
      decider_data->lanemarker_lanline_debug->lanemarker_decider_debug()
          .lanemarkers_copy_info()
          .right_is_copy();
  if (decider_data->lane_markers_state.right_lanemarker_state ==
          LaneMarkerState::BAD_LANEMARKER &&
      !is_right_copy) {
    right_history_esti_.clear();
    ADEBUG << "has no right lane marker!"
           << " view_range: " << view_range;
    return;
  }
  ADEBUG << "original right_history_esti_size = " << right_history_esti_.size();
  if (right_history_esti_.size() <= 2 || is_right_copy) {
    CreatInitLaneMarkerPoints(now_lanemarker, &right_history_esti_);
    has_no_right_history_points_ = true;
  } else {
    has_no_right_history_points_ = false;
  }

  Matrix pre_points_x;
  Matrix pre_points_y;
  Matrix measure_points_y;
  std::vector<Vec2d> esti_points;
  if (has_no_right_history_points_) {
    ADEBUG << "has no right history points!";
    esti_points = right_history_esti_;
  } else {
    if (!DoPredictUpdate(view_range, &right_history_esti_, &pre_points_x,
                         &pre_points_y)) {
      AERROR << "Do right predictupdate failed!";
      return;
    }
    DoMeasureUpdate(now_lanemarker, pre_points_x, &measure_points_y);
    Matrix right_kalman_gain;
    DoKalmanGainUpdate(pre_points_x, pre_points_y, measure_points_y, view_range,
                       reset, v_spd, spl_flag, change_flag, &esti_points,
                       &right_kalman_gain);
    SetDebugInfo(right_filter_debug, right_history_esti_, pre_points_x,
                 pre_points_y, measure_points_y, right_kalman_gain,
                 has_no_right_history_points_);

    // AddEstiPoints(now_lanemarker, &esti_points);
    right_history_esti_ = esti_points;
  }
  ADEBUG << "right esti_points_size: " << esti_points.size();
  (*decider_data)
      .lane_marker_points.insert(std::make_pair("right", esti_points));
}

void LaneMarkerFilter::NLeftLaneFilter(DeciderData* decider_data) {
  ADEBUG << "NLeftLaneFilter Process!"
         << decider_data->original_lanemarkers.header().seq();
  const double v_spd = decider_data->filter_vehicle_state.vehicle_speed_average;
  auto* next_left_filter_debug =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lane_filter_info()
          ->add_lane_filter_info();
  next_left_filter_debug->set_name("next_left_filter_debug");
  // const LaneMarker now_lanemarker =
  //     decider_data->copy_lanemarkers.next_left_lane_marker().at(0);
  const LaneMarker now_lanemarker =
      decider_data->copy_lanemarkers.front_next_left_lane_marker().at(0);
  const double view_range = now_lanemarker.view_range();
  next_left_filter_debug->set_view_range(view_range);
  const bool reset =
      decider_data->lane_reset_out.next_left_lane_reset.is_lane_reset;
  const bool spl_flag =
      decider_data->lane_reset_out.next_left_lane_reset.split_lane_flag;
  bool is_next_left_copy =
      decider_data->lanemarker_lanline_debug->lanemarker_decider_debug()
          .lanemarkers_copy_info()
          .next_left_is_copy();
  if (decider_data->lane_markers_state.next_left_lanemarker_state ==
          LaneMarkerState::BAD_LANEMARKER &&
      !is_next_left_copy) {
    next_left_history_esti_.clear();
    ADEBUG << "has no next left lane marker!"
           << " view_range: " << view_range;
    return;
  }
  ADEBUG << "original next_left_history_esti_size = "
         << next_left_history_esti_.size();
  if (next_left_history_esti_.size() <= 1 || is_next_left_copy) {
    CreatInitLaneMarkerPoints(now_lanemarker, &next_left_history_esti_);
    has_no_next_left_history_points_ = true;
  } else {
    has_no_next_left_history_points_ = false;
  }

  Matrix pre_points_x;
  Matrix pre_points_y;
  Matrix measure_points_y;
  std::vector<Vec2d> esti_points;
  if (has_no_next_left_history_points_) {
    ADEBUG << "has no next left history points!";
    esti_points = next_left_history_esti_;
  } else {
    if (!DoPredictUpdate(view_range, &next_left_history_esti_, &pre_points_x,
                         &pre_points_y)) {
      AERROR << "Do next left predictupdate failed!";
      return;
    }
    DoMeasureUpdate(now_lanemarker, pre_points_x, &measure_points_y);
    Matrix next_left_kalman_gain;
    DoKalmanGainUpdate(pre_points_x, pre_points_y, measure_points_y, view_range,
                       reset, v_spd, spl_flag, false, &esti_points,
                       &next_left_kalman_gain);
    // AddEstiPoints(now_lanemarker, &esti_points);
    SetDebugInfo(next_left_filter_debug, next_left_history_esti_, pre_points_x,
                 pre_points_y, measure_points_y, next_left_kalman_gain,
                 has_no_next_left_history_points_);

    next_left_history_esti_ = esti_points;
  }
  ADEBUG << "next left esti_points_size: " << esti_points.size();
  (*decider_data)
      .lane_marker_points.insert(std::make_pair("next_left", esti_points));
}

void LaneMarkerFilter::NRightLaneFilter(DeciderData* decider_data) {
  ADEBUG << "NRightLaneFilter Process!"
         << decider_data->original_lanemarkers.header().seq();
  const double v_spd = decider_data->filter_vehicle_state.vehicle_speed_average;
  auto* next_right_filter_debug =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lane_filter_info()
          ->add_lane_filter_info();
  next_right_filter_debug->set_name("next_right_filter_debug");
  // const LaneMarker now_lanemarker =
  //     decider_data->copy_lanemarkers.next_right_lane_marker().at(0);
  const LaneMarker now_lanemarker =
      decider_data->copy_lanemarkers.front_next_right_lane_marker().at(0);
  const double view_range = now_lanemarker.view_range();
  next_right_filter_debug->set_view_range(view_range);
  const bool reset =
      decider_data->lane_reset_out.next_right_lane_reset.is_lane_reset;
  const bool spl_flag =
      decider_data->lane_reset_out.next_right_lane_reset.split_lane_flag;
  bool is_next_right_copy =
      decider_data->lanemarker_lanline_debug->lanemarker_decider_debug()
          .lanemarkers_copy_info()
          .next_right_is_copy();
  if (decider_data->lane_markers_state.next_right_lanemarker_state ==
          LaneMarkerState::BAD_LANEMARKER &&
      !is_next_right_copy) {
    next_right_history_esti_.clear();
    ADEBUG << "has no next right lane marker!"
           << " view_range: " << view_range;
    return;
  }
  ADEBUG << "original next_right_history_esti_size = "
         << next_right_history_esti_.size();
  if (next_right_history_esti_.size() <= 1 || is_next_right_copy) {
    CreatInitLaneMarkerPoints(now_lanemarker, &next_right_history_esti_);
    has_no_next_right_history_points_ = true;
  } else {
    has_no_next_right_history_points_ = false;
  }

  Matrix pre_points_x;
  Matrix pre_points_y;
  Matrix measure_points_y;
  std::vector<Vec2d> esti_points;
  if (has_no_next_right_history_points_) {
    ADEBUG << "has no next right history points!";
    esti_points = next_right_history_esti_;
  } else {
    if (!DoPredictUpdate(view_range, &next_right_history_esti_, &pre_points_x,
                         &pre_points_y)) {
      AERROR << "Do next right predictupdate failed!";
      return;
    }
    DoMeasureUpdate(now_lanemarker, pre_points_x, &measure_points_y);
    Matrix next_right_kalman_gain;
    DoKalmanGainUpdate(pre_points_x, pre_points_y, measure_points_y, view_range,
                       reset, v_spd, spl_flag, false, &esti_points,
                       &next_right_kalman_gain);
    // AddEstiPoints(now_lanemarker, &esti_points);
    SetDebugInfo(next_right_filter_debug, next_right_history_esti_,
                 pre_points_x, pre_points_y, measure_points_y,
                 next_right_kalman_gain, has_no_next_right_history_points_);

    next_right_history_esti_ = esti_points;
  }
  ADEBUG << "next right esti_points_size: " << esti_points.size();
  (*decider_data)
      .lane_marker_points.insert(std::make_pair("next_right", esti_points));
}

void LaneMarkerFilter::CalculateDxyphi(const double input_spd,
                                       const double input_yaw_rate,
                                       const double ts) {
  dx_ = input_spd * ts;
  dphi_ = input_yaw_rate * ts;
  dy_ = dx_ * dx_ * (history_yaw_rate_ / Clamp(input_spd, 0.001, 1000.0)) * 0.5;
  history_yaw_rate_ = input_yaw_rate;
  ADEBUG << "dx: " << dx_ << ", dy: " << dy_ << ", dphi: " << dphi_;
}

bool LaneMarkerFilter::DoPredictUpdate(const double view_range,
                                       std::vector<Vec2d>* const history_esti,
                                       Eigen::MatrixXd* const pre_points_x,
                                       Eigen::MatrixXd* const pre_points_y) {
  Matrix trans_matrix = Matrix::Zero(3, 3);
  trans_matrix(0, 0) = std::cos(dphi_);
  trans_matrix(0, 1) = std::sin(dphi_);
  trans_matrix(0, 2) = -(dx_ * std::cos(dphi_) + dy_ * std::sin(dphi_));
  trans_matrix(1, 0) = -std::sin(dphi_);
  trans_matrix(1, 1) = std::cos(dphi_);
  trans_matrix(1, 2) = (dx_ * std::sin(dphi_) - dy_ * std::cos(dphi_));
  trans_matrix(2, 2) = 1;
  if (!AdjustEstiPoints(view_range, history_esti)) {
    AERROR << "Adjust history points failed!";
    return false;
  }
  ADEBUG << "-------------after adjust history POINTS ----------------";
  for (size_t i = 0; i < history_esti->size(); i++) {
    ADEBUG << "index[" << i << "], x = " << history_esti->at(i).x()
           << ", y = " << history_esti->at(i).y();
  }
  Matrix buffer_point = Matrix::Zero(3, history_esti->size());       //  NOLINT
  Matrix buffer_hist_point = Matrix::Zero(3, history_esti->size());  //  NOLINT
  *pre_points_x = Matrix::Zero(1, history_esti->size());             //  NOLINT
  *pre_points_y = Matrix::Zero(1, history_esti->size());             //  NOLINT
  for (int i = 0; i < static_cast<int>(history_esti->size()); i++) {
    buffer_point(0, i) = history_esti->at(i).x();
    buffer_point(1, i) = history_esti->at(i).y();
    buffer_point(2, i) = 1;
  }
  buffer_hist_point = trans_matrix * buffer_point;
  for (int i = 0; i < static_cast<int>(history_esti->size()); i++) {
    (*pre_points_x)(0, i) = buffer_hist_point(0, i);
    (*pre_points_y)(0, i) = buffer_hist_point(1, i);
  }
  return true;
}

bool LaneMarkerFilter::AdjustEstiPoints(
    const double view_range, std::vector<Vec2d>* const history_esti) {
  ADEBUG << "Before adjust history points, size: " << history_esti->size();
  const double step = config_.lanemarker_filter_config().step_distance();
  double start_range = -config_.lanemarker_back_length();
  double end_range =
      Clamp(view_range, config_.lanemarker_filter_config().low_viewrange(),
            config_.lanemarker_filter_config().upper_viewrange());
  // return AdjustEstiPointsFromExtend(start_range, end_range, history_esti);
  return AdjustEstiPointsFromFit(start_range, end_range, step, history_esti);
}

bool LaneMarkerFilter::AdjustEstiPointsFromExtend(
    const double start_range, const double end_range,
    std::vector<Vec2d>* const points) {
  int start_index = points->size() - 1;  //  NOLINT
  int end_index = 0;
  for (int i = 0; i < static_cast<int>(points->size()); i++) {
    if (points->at(i).x() > start_range) {
      start_index = i;
      break;
    }
  }
  if (end_range >= points->back().x()) {
    end_index = points->size() - 1;  //  NOLINT
  } else {
    for (int i = points->size() - 1; i >= 0; i--) {  //  NOLINT
      if (points->at(i).x() < end_range) {
        end_index = i;
        break;
      }
    }
  }
  if (start_index >= end_index) {
    AERROR << "start_index greater than end_index! start_index: " << start_index
           << ", end_index: " << end_index;
    AERROR << "history_points_start_x: " << points->front().x()
           << ", history_points_end_X: " << points->back().x()
           << ", view_range: " << end_range;
    return false;
  }
  std::vector<Vec2d> history_points;
  for (int i = start_index; i < end_index + 1; i++) {
    history_points.push_back(points->at(i));
  }
  const auto size = history_points.size();
  // 将历史的点先进行延长，再使用，这样可以避免滤波后延长导致点的跳动，
  // TODO(fml) 考虑长度突然增加过多导致延长不可靠的情况
  if (end_range >= history_points.back().x() && size > kMagicNumber5) {
    double delta_x =
        ((history_points.back().x() - history_points.at(size - 3).x()) / 2 +
         (history_points.at(size - 2).x() - history_points.at(size - 4).x()) /
             2) /
        2;
    double delta_y =
        ((history_points.back().y() - history_points.at(size - 3).y()) / 2 +
         (history_points.at(size - 2).y() - history_points.at(size - 4).y()) /
             2) /
        2;
    double x = history_points.back().x();
    double y = history_points.back().y();
    while (x <= end_range) {
      x += delta_x;
      y += delta_y;
      history_points.emplace_back(x, y);
    }
  }
  points->swap(history_points);
  ADEBUG << "After adjust history points,size: " << points->size();
  return true;
}

bool LaneMarkerFilter::AdjustEstiPointsFromFit(
    const double start_range, const double end_range, const double step,
    std::vector<Vec2d>* const points) {
  if (points->size() < 3) {
    AERROR << "points_size: " << points->size() << ", too low!!!";
    return false;
  }
  const int N = 3;
  std::vector<double> coff = FitPolynomial<N>(*points);
  points->clear();
  int max_index = floor((end_range - start_range) / step + 1);
  for (int i = 0; i < max_index; i++) {
    auto x = i * step + start_range;
    double y = coff[0] + coff[1] * x + coff[2] * x * x + coff[3] * x * x * x;
    points->push_back(Vec2d(x, y));
  }
  return true;
}

bool LaneMarkerFilter::DoMeasureUpdate(
    const LaneMarker& copy_lanemarker, const Eigen::MatrixXd& pre_points_x,
    Eigen::MatrixXd* const measure_points_y) {
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

// deal change state
bool LaneMarkerFilter::DoKalmanGainUpdate(
    const Eigen::MatrixXd& pre_points_x, const Eigen::MatrixXd& pre_points_y,
    const Eigen::MatrixXd& measure_points_y, const double view_range,
    const bool reset, const double speed, const bool splitlane_flag,
    const bool lane_change_flag, std::vector<Vec2d>* const esti_points,
    Eigen::MatrixXd* const kalman_gain) {
  const int point_size = pre_points_x.cols();  //  NOLINT
  ADEBUG << "pre_points_size = " << point_size;
  Matrix kal_gc = Matrix::Constant(1, point_size, 1);
  if ((speed > config_.lanemarker_filter_config().vehicle_speed_thd()) &&
      !reset) {
    const double k_length =
        std::pow(length_interpolation_->Interpolate(view_range), 2);
    ADEBUG << "view_range: " << view_range
           << ", k_length*k_length: " << k_length;
    for (int i = 0; i < point_size; i++) {
      const double k_diff_length = diff_length_interpolation_->Interpolate(
          view_range - pre_points_x(0, i));
      kal_gc(0, i) = k_diff_length * k_length;
      // AERROR << "diff_length_k["<< i << "]: " << k_diff_length << ", kal_gc:
      // " << kal_gc(0,i);
    }
  }
  Matrix kal_gc_tmp;
  if (lane_change_flag || splitlane_flag) {
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

void LaneMarkerFilter::CreatInitLaneMarkerPoints(
    const LaneMarker& input_lanemarker,
    std::vector<Vec2d>* const history_esti) {
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

bool LaneMarkerFilter::Interpolation(std::vector<Vec2d>* const esti_points) {
  if (esti_points->size() < 2) {
    AERROR << "esti_points_size < 2!";
    return false;
  }
  std::size_t index_greater_zero = 0;
  Vec2d zero_point;
  for (std::size_t i = 0; i < esti_points->size(); i++) {
    if (esti_points->at(i).x() > 0) {
      index_greater_zero = i;
      break;
    }
  }
  if (index_greater_zero == 0) {
    AERROR << "has no back points!";
    return false;
  }
  const double x2 = esti_points->at(index_greater_zero).x();
  const double y2 = esti_points->at(index_greater_zero).y();
  const double x0 = esti_points->at(index_greater_zero - 1).x();
  const double y0 = esti_points->at(index_greater_zero - 1).y();
  // (x0,y0)------(x1,y1)------(x2,y2)
  // (x1 - x0)/(x2 - x0) = (y1 - y0)/(y2 - y0)
  // x1 = 0;
  const double y1 = y0 - (x0 / (x2 - x0)) * (y2 - y0);
  zero_point = Vec2d(0, y1);
  std::vector<Vec2d> back_points;
  back_points.push_back(zero_point);
  for (int i = index_greater_zero - 1; i >= 0; i--) {  //  NOLINT
    back_points.push_back(esti_points->at(i));
  }
  back_points = InterpolateVec2dPoints(back_points, 1.0);

  std::vector<Vec2d> front_points;
  front_points.push_back(zero_point);
  front_points.insert(front_points.end(),
                      esti_points->begin() + index_greater_zero,  //  NOLINT
                      esti_points->end());
  front_points = InterpolateVec2dPoints(front_points, 1.0);

  esti_points->clear();
  for (int i = back_points.size() - 1; i >= 0; i--) {  //  NOLINT
    esti_points->push_back(back_points.at(i));
  }
  esti_points->insert(esti_points->end(), front_points.begin() + 1,
                      front_points.end());
  return true;
}

void LaneMarkerFilter::SetDebugInfo(LanePointsFilter* const filter_debug,
                                    const std::vector<Vec2d>& history_esti,
                                    const Eigen::MatrixXd& pre_points_x,
                                    const Eigen::MatrixXd& pre_points_y,
                                    const Eigen::MatrixXd& measure_points_y,
                                    const Eigen::MatrixXd& kalman_gain,
                                    const bool has_history_points) {
  if (FLAGS_lane_line_decider_debug_flag == 1) {
    filter_debug->set_has_history_points(!has_history_points);
    for (const auto& point : history_esti) {
      auto* pos = filter_debug->add_history_esti_points();
      pos->set_x(point.x());
      pos->set_y(point.y());
    }
    for (int i = 0; i < pre_points_x.cols(); i++) {
      auto* points_p = filter_debug->add_predict_points();
      auto* points_m = filter_debug->add_measure_points();
      points_p->set_x(pre_points_x(0, i));
      points_p->set_y(pre_points_y(0, i));
      points_m->set_x(pre_points_x(0, i));
      points_m->set_y(measure_points_y(0, i));
      filter_debug->add_kalman_gain(kalman_gain(0, i));
    }
  }
}
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
