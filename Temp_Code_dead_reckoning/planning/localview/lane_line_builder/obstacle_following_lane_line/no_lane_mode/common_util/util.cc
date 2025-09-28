/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: Lingpeng
 * Created time: 2022/05/25
 *****************************************************************************/
#define NOLANE_UTIL
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/common_util/util.h"

#include <limits>
#include <list>
#include <map>
#include <memory>
#include <numeric>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace TL {
namespace planning {
namespace nolane {

StateType operator-(const StateType& lhs, const StateType& rhs) {
  StateType rev;
  rev.time_stamp = lhs.time_stamp - rhs.time_stamp;
  rev.position.x = lhs.position.x - rhs.position.x;
  rev.position.y = lhs.position.y - rhs.position.y;
  rev.theta = lhs.theta - rhs.theta;

  return rev;
}

void SetDebugFlag() {
  debug_flag.set(0, true);    // lp: debug in lane_center_line.cc
  debug_flag.set(1, false);   // lp: debug matrix result
  debug_flag.set(2, true);    // lp: debug in fit_polynomial_curve.cc
  debug_flag.set(3, true);    // lp: debug in fit_curve.cc
  debug_flag.set(4, true);    // lp: print time-stamp and period end
  debug_flag.set(5, true);    // lp: debug obstacle history trajectory.
  debug_flag.set(6, true);    // lp: cost debug
  debug_flag.set(7, true);    // lp: cost ref point in lane_center_line.cc
  debug_flag.set(8, true);    // lp: debug in fit piecewise
  debug_flag.set(9, true);    // lp: debug in fit piecewise
  debug_flag.set(10, true);   // lp: ego sl is still in previous lane.
  debug_flag.set(11, true);   // lp: enable fit piecewise function.
  debug_flag.set(12, true);   // lp: obstacle info logging
  debug_flag.set(13, false);  // lp: debug in fit piecewise
  debug_flag.set(14, true);   // lp: state_sl info
  debug_flag.set(15, true);   // lp: moving behavior
  debug_flag.set(16, true);   // lp: log previous lane
  debug_flag.set(17, false);  // lp: log previous lane
  debug_flag.set(18, true);   // lp: log previous reference line
  debug_flag.set(19, true);   // lp: ego pos x, pos y...
  debug_flag.set(20, false);  // lp: ego obs traj_connect
  debug_flag.set(21, true);   // lp: ego obs traj_connect
  debug_flag.set(22, true);   // lp: ego obs traj_connect
  debug_flag.set(23, false);  // lp: ego obs traj_connect
  debug_flag.set(24, false);  // lp: routing information
  debug_flag.set(25, true);   // lp: routing information
  debug_flag.set(26, true);   // lp: routing information
  debug_flag.set(27, true);   // lp: routing information
  debug_flag.set(28, true);   // lp: routing information
  debug_flag.set(29, false);  // lp: routing information
  debug_flag.set(30, true);   // lp: routing information
  debug_flag.set(31, true);   // lp: debug in fit piecewise curve
  debug_flag.set(32, true);   // lp: routing information
  debug_flag.set(33, true);   // lp: routing information
  debug_flag.set(34, true);   // lp: routing information
  debug_flag.set(35, false);  // lp: routing information
  debug_flag.set(36, true);   // lp: routing information
  debug_flag.set(37, true);   // lp: routing information
  debug_flag.set(38, true);   // lp: routing information
  debug_flag.set(39, true);   // lp: routing information
  debug_flag.set(40, true);   // lp: routing information
  debug_flag.set(41, true);   // lp: routing information
  debug_flag.set(42, true);   // lp: routing information
  debug_flag.set(43, true);   // lp: routing information
  debug_flag.set(44, true);   // lp: routing information
  debug_flag.set(45, true);   // lp: routing information
  debug_flag.set(46, true);   // lp: routing information
  debug_flag.set(46, true);
  debug_flag.set(47, true);
  debug_flag.set(48, true);
  debug_flag.set(49, true);
  debug_flag.set(50, true);
  debug_flag.set(51, true);
  debug_flag.set(52, true);
}

void SetCyberFlag() {
  cyber_flag.set(0, true);  // lp: centerline
  cyber_flag.set(1, true);  // lp: fit curve coef
  cyber_flag.set(2, true);  // lp: trajectory start index
  cyber_flag.set(3, true);  // lp: followed obs info
  cyber_flag.set(4, true);  // lp: perception obs
  cyber_flag.set(6, true);  // lp: debug obs
  cyber_flag.set(7, true);  // lp: debug obs
  cyber_flag.set(8, true);  // lp: debug obs

  // cyber_flag.reset();// lp: not send anything at all.
}

void SetAllDebugFlag() {
  if (FLAGS_nolane_log_none) {
    debug_flag.reset();  // lp: not debug at all
  } else if (FLAGS_nolane_log_all) {
    debug_flag.set();
  }
}

void SetLogFlag() {
  debug_flag.set(22, false);
  // debug_flag.set(2, false);
}

NoLaneToCyber::NoLaneToCyber(const std::shared_ptr<LocalView>& local_view)
    : local_view_(local_view) {
  time_start_ = TL::common::Clock::NowInSeconds();
  if (ptr_without_lane_) {
    ptr_without_lane_->Clear();
  } else {
    AERROR << "ptr_without_lane_ is nullptr.";
    ptr_without_lane_ = std::make_shared<TL::planning::WithoutLaneFollow>();
  }
  TL::common::util::FillHeader("without_lane_kernel",
                                  ptr_without_lane_.get());
  SetDebugFlag();
  SetAllDebugFlag();
  SetLogFlag();
  SetCyberFlag();
}

NoLaneToCyber::~NoLaneToCyber() {
  if (ptr_without_lane_) {
    time_end_ = TL::common::Clock::NowInSeconds();
    ptr_without_lane_->mutable_debug()->set_time_stamp(
        (time_end_ - time_start_) * 1000);
    auto time_elapsed = ptr_without_lane_->mutable_debug()->add_temp_value();
    time_elapsed->set_name("time_elapsed_ms");
    time_elapsed->set_d1((time_end_ - time_start_) * 1000);
    if (debug_flag[52]) {
      AERROR << "no_lane_time_elapsed:" << (time_end_ - time_start_) * 1000;
    }
    local_view_->SetWithoutLaneFollowPtr(
        std::make_shared<TL::planning::WithoutLaneFollow>(
            std::move(*ptr_without_lane_)));
  }
}

const std::shared_ptr<TL::planning::WithoutLaneFollow>&
NoLaneToCyber::GetPtrWithoutLane() {
  return ptr_without_lane_;
}

std::shared_ptr<TL::planning::WithoutLaneFollow>
    NoLaneToCyber::ptr_without_lane_ =
        std::make_shared<TL::planning::WithoutLaneFollow>();

const std::shared_ptr<TL::planning::WithoutLaneFollow>& ptr_without_lane =
    NoLaneToCyber::GetPtrWithoutLane();

Eigen::VectorXd PolynomialConnectTwoPoint(
    const std::array<double, 3>& start_point,
    const std::array<double, 3>& end_point) {
  double start_time = 0.0;
  if (debug_flag[1]) {
    start_time = std::chrono::steady_clock::now().time_since_epoch().count();
  }
  double x_start = start_point[0];
  double x_start_2 = x_start * x_start;
  double x_start_3 = x_start_2 * x_start;
  double x_end = end_point[0];
  double x_end_2 = x_end * x_end;
  double x_end_3 = x_end_2 * x_end;
  Eigen::Matrix4d A_mat;
  Eigen::Vector4d Y_mat;
  A_mat << x_start_3, x_start_2, x_start, 1.0, x_end_3, x_end_2, x_end, 1.0,
      3.0 * x_start_2, 2.0 * x_start, 1.0, 0, 3.0 * x_end_2, 2.0 * x_end, 1, 0;
  Y_mat << start_point[1], end_point[1], start_point[2], end_point[2];
  auto coef = A_mat.inverse() * Y_mat;
  if (debug_flag[1]) {
    double end_time =
        std::chrono::steady_clock::now().time_since_epoch().count();
    AERROR << PRECISION(5) << "start_state:[" << start_point[0] << ","
           << start_point[1] << "," << start_point[2] << "]  end_state:["
           << end_point[0] << "," << end_point[1] << "," << end_point[2]
           << "]. elapsed time:"
           << static_cast<double>((end_time - start_time) / 1e6) << "(ms)";
    AERROR << "A_matrix:";
    AERROR << PRECISION(3) << "[" << A_mat(0, 0) << "," << A_mat(0, 1) << ","
           << A_mat(0, 2) << "," << A_mat(0, 3) << "]";
    AERROR << PRECISION(3) << "[" << A_mat(1, 0) << "," << A_mat(1, 1) << ","
           << A_mat(1, 2) << "," << A_mat(1, 3) << "]";
    AERROR << PRECISION(3) << "[" << A_mat(2, 0) << "," << A_mat(2, 1) << ","
           << A_mat(2, 2) << "," << A_mat(2, 3) << "]";
    AERROR << PRECISION(3) << "[" << A_mat(3, 0) << "," << A_mat(3, 1) << ","
           << A_mat(3, 2) << "," << A_mat(3, 3) << "]";
    AERROR << PRECISION(3) << "Y_matrix:[" << Y_mat(0, 0) << "," << Y_mat(1, 0)
           << "," << Y_mat(2, 0) << "," << Y_mat(3, 0) << "]";

    double ego_y_c = coef(0, 0) * x_start_3 + coef(1, 0) * x_start_2 +
                     coef(2, 0) * x_start + coef(3, 0);
    double obs_y_c = coef(0, 0) * x_end_3 + coef(1, 0) * x_end_2 +
                     coef(2, 0) * x_end + coef(3, 0);
    double ego_theta_c =
        coef(0, 0) * 3 * x_start_2 + coef(1, 0) * 2 * x_start + coef(2, 0);
    double obs_theta_c =
        coef(0, 0) * 3 * x_end_2 + coef(1, 0) * 2 * x_end + coef(2, 0);
    auto Y_c = A_mat * coef;
    AERROR << PRECISION(4) << "y_start     matrix:" << Y_c(0, 0)
           << "  calc:" << ego_y_c << "  real:" << start_point[1];
    AERROR << PRECISION(4) << "y_end       matrix:" << Y_c(1, 0)
           << "  calc:" << obs_y_c << "  real:" << end_point[1];
    AERROR << PRECISION(4) << "slope_start matrix:" << Y_c(2, 0)
           << "  calc:" << ego_theta_c << "  real:" << start_point[2];
    AERROR << PRECISION(4) << "slope_end   matrix:" << Y_c(3, 0)
           << "  calc:" << obs_theta_c << "  real:" << end_point[2];
    AERROR << PRECISION(4) << "coef:[" << coef(0, 0) << "," << coef(1, 0) << ","
           << coef(2, 0) << "," << coef(3, 0) << "]";
  }
  return coef;
}

double GetPolyValue(const std::vector<double>& coef_i, double x) {
  double res_val = 0.0;
  size_t coef_size = coef_i.size();
  for (int i = 0; i < coef_i.size(); ++i) {
    res_val += coef_i[i] * std::pow(x, coef_size - 1 - i);
  }
  return res_val;
}

double GetCurvature(const std::vector<double>& coef_1,
                    const std::vector<double>& coef_2, double x) {
  return GetPolyValue(coef_2, x) /
         std::pow((1 + std::pow(GetPolyValue(coef_1, x), 2)), 1.5);
}

double MaxCurvatureInPolynomial(const std::vector<double>& coef, double x_min,
                                double x_max, int interval_size,
                                const double precision) {
  // lp: min-max value of curvature function is too difficult to get use
  // analytical solution.Therefore following code calculate curvature use
  // numeric solutions instead.
  // lp: 1order differential
  std::vector<double> coef_1{3 * coef[0], 2 * coef[1], coef[2]};
  // lp: 2order differential
  std::vector<double> coef_2{6 * coef[0], 2 * coef[1]};
  if (DefinitelyGreater(x_min, x_max)) {
    // std::swap(x_min, x_max);
    double temp = x_max;
    x_max = x_min;
    x_min = temp;
  }
  const double x_min_t = x_min;
  const double x_max_t = x_max;
  double x = x_min;
  double max_curvature = GetCurvature(coef_1, coef_2, x_min);
  double curvature_temp = 0;
  double interval = (x_max - x_min) / interval_size;
  int counter = 0;
  while ((x += interval) <= x_max) {
    ++counter;
    curvature_temp = fabs(GetCurvature(coef_1, coef_2, x));
    if (max_curvature < curvature_temp) {
      max_curvature = curvature_temp;
    } else {
      if (max_curvature - curvature_temp > precision || interval > 0.1) {
        x_min = x - 2 * interval;
        x_max = x;
        interval = (x_max - x_min) / interval_size;
        max_curvature = GetCurvature(coef_1, coef_2, x_min);
        x = x_min;
      } else {
        break;
      }
    }
  }
  if (debug_flag[39]) {
    std::string coef_str = "";
    TL::common::util::vec2str(coef, &coef_str);
    AERROR << "curvature_found_after_counter:" << counter
           << "  coef:" << coef_str << "  x_min:" << x_min_t
           << "  x_max:" << x_max_t << "  max_curvature:" << max_curvature;
  }

  // lp: get extreme value in [x_min,x_max] in curvature function
  return fabs(max_curvature);
}

namespace StaticFeature {
double CalculateVariance(const std::vector<double>& data) {
  if (!DataValidationCheck(data))
    return 0;
  double mean = CalculateMeanValue(data);
  double variance = 0.0;
  std::for_each(data.begin(), data.end(),
                [&](double val) { variance += (val - mean) * (val - mean); });
  return variance;
}

double CalculateSkewness(const std::vector<double>& data) {
  int data_size = data.size();
  ACHECK(data_size < 2) << "size must be bigger than 2.";
  double mean = CalculateMeanValue(data);
  double numerator = 0.0;
  double denominator = 0.0;
  double val_temp = 0.0;
  std::for_each(data.begin(), data.end(), [&](double val) {
    val_temp = (val - mean) * (val - mean);
    denominator += val_temp;
    numerator += val_temp * (val - mean);
  });

  numerator = numerator / data_size;
  denominator = denominator / data_size;
  denominator = sqrt(denominator * denominator * denominator);
  if (fabs(denominator) < 0.0001) {
    return std::numeric_limits<double>::infinity();
  }

  return numerator / denominator;
}

double CalculateKurtosis(const std::vector<double>& data) {
  int data_size = data.size();
  ACHECK(data_size < 2) << "size must be bigger than 2.";
  double mean = CalculateMeanValue(data);
  double numerator = 0.0;
  double denominator = 0.0;
  double val_temp = 0.0;
  std::for_each(data.begin(), data.end(), [&](double val) {
    val_temp = (val - mean) * (val - mean);
    denominator += val_temp;
    numerator += val_temp * val_temp;
  });

  numerator = numerator / data_size;
  denominator = denominator / data_size;
  denominator = denominator * denominator;

  if (fabs(denominator) < 0.0001) {
    return std::numeric_limits<double>::infinity();
  }

  return numerator / denominator - 3.0;
}

StaticRes CalculateStaticFeature(const std::vector<double>& data) {
  if (data.empty())
    return {};
  StaticRes rev;

  int data_size = data.size();
  double mean = CalculateMeanValue(data);
  double numerator_skewness = 0.0;
  double numerator_kurtosis = 0.0;
  double temp_v = 0.0;
  double variance = 0.0;

  rev.mean = mean;
  rev.init = true;
  rev.size = data_size;

  std::for_each(data.begin(), data.end(), [&](double val) {
    temp_v = (val - mean) * (val - mean);
    variance += temp_v;
    numerator_skewness += temp_v * (val - mean);
    numerator_kurtosis += temp_v * temp_v;
  });
  if (data.size() < 3) {
    return rev;
  }

  numerator_skewness /= data_size;
  numerator_kurtosis /= data_size;

  rev.variance = data_size == 1 ? 0.000001 : variance / (data_size - 1);
  rev.std_error = sqrt(rev.variance);

  variance /= data_size;
  rev.skewness = numerator_skewness / sqrt(variance * variance * variance);
  rev.kurtosis = numerator_kurtosis / (variance * variance) - 3.0;
  rev.calc_complete = true;

  return rev;
}
}  // namespace StaticFeature
}  // namespace nolane
}  // namespace planning
}  // namespace TL
