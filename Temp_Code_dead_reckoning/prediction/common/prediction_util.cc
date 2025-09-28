/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "planning/prediction/common/prediction_util.h"

#include <algorithm>
#include <cmath>
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "planning/prediction/common/prediction_gflags.h"
#include "proto/common/pnc_point.pb.h"

using TL::common::PathPoint;
using TL::common::TrajectoryPoint;

namespace TL {
namespace prediction {
namespace prediction_util {

double Normalize(const double value, const double mean, const double std) {
  const double eps = 1e-10;
  return (value - mean) / (std + eps);
}

double Relu(const double value) {
  return (value > 0.0) ? value : 0.0;
}

double ComputeMean(const std::vector<double>& nums, size_t start, size_t end) {
  int count = 0;
  double sum = 0.0;
  for (size_t i = start; i <= end && i < nums.size(); i++) {
    sum += nums[i];
    ++count;
  }
  return (count == 0) ? 0.0 : sum / count;
}

std::vector<double> Softmax(const std::vector<double>& value, bool use_exp) {
  std::vector<double> result;
  double sum = 0.0;
  for (double v : value) {
    double exp_value = std::max(0.001, v);
    if (use_exp) {
      exp_value = std::exp(v);
    }
    sum += exp_value;
    result.push_back(exp_value);
  }
  for (std::size_t i = 0; i < value.size(); ++i) {
    result[i] = result[i] / sum;
  }
  return result;
}

// int SolveQuadraticEquation(const std::vector<double>& coefficients,
//                            std::pair<double, double>* roots) {
//   if (coefficients.size() != 3) {
//     return -1;
//   }
//   const double a = coefficients[0];
//   const double b = coefficients[1];
//   const double c = coefficients[2];
//   if (std::fabs(a) <= std::numeric_limits<double>::epsilon()) {
//     return -1;
//   }

//   double delta = b * b - 4.0 * a * c;
//   if (delta < 0.0) {
//     return -1;
//   }

//   double sqrt_delta = std::sqrt(delta);
//   roots->first = (-b + sqrt_delta) * 0.5 / a;
//   roots->second = (-b - sqrt_delta) * 0.5 / a;
//   return 0;
// }

void GetCubicPolynomial(double x0, double dx0, double x1, double dx1, double p,
                        std::array<double, 4>* coefficients) {
  coefficients->operator[](0) = x0;
  coefficients->operator[](1) = dx0;
  double p2 = p * p;
  double p3 = p2 * p;
  double tmp_var1 = (dx1 - dx0) * p;
  double tmp_var2 = x1 - x0 - dx0 * p;
  coefficients->operator[](2) = (3.0 * tmp_var2 - tmp_var1) / p2;
  coefficients->operator[](3) = (tmp_var1 - 2.0 * tmp_var2) / p3;
}

void GetQuarticPolynomial(double x0, double dx0, double ddx0, double dx1,
                          double ddx1, double p,
                          std::array<double, 5>* coefficients) {
  coefficients->operator[](0) = x0;
  coefficients->operator[](1) = dx0;
  coefficients->operator[](2) = 0.5 * ddx0;
  double b0 = dx1 - ddx0 * p - dx0;
  double b1 = ddx1 - ddx0;
  double p2 = p * p;
  double p3 = p2 * p;
  coefficients->operator[](3) = b0 / p2 - b1 / 3.0 / p;
  coefficients->operator[](4) = -0.5 / p3 * b0 + 0.25 / p2 * b1;
}

std::pair<double, double> GetQuarticPolynomial(
    double x0, double dx0, double ddx0, std::array<double, 5>* coefficients,
    double dddx0) {
  if (fabs(dddx0) < common::math::kMathEpsilon) {
    AERROR << "0.5dddx0 is too small, reset to 1.0";
    dddx0 = 1.0;
  }
  coefficients->operator[](0) = x0;
  coefficients->operator[](1) = dx0;
  coefficients->operator[](2) = 0.5 * ddx0;
  coefficients->operator[](3) = 0.0;
  coefficients->operator[](4) = 0.0;
  double t_end = 0.0;
  double v_end = dx0;
  if (fabs(ddx0) > common::math::kMathEpsilon) {
    t_end = fabs(ddx0 / dddx0);
    // ddx = ddx0 * (t / p - 1) ^ 2
    coefficients->operator[](3) = -1.0 / 3.0 * (ddx0 / t_end);
    coefficients->operator[](4) = ddx0 / 12.0 / t_end / t_end;
    v_end = coefficients->operator[](1) +
            2 * coefficients->operator[](2) * t_end +
            3 * coefficients->operator[](3) * t_end * t_end +
            4 * coefficients->operator[](4) * t_end * t_end * t_end;
  }
  return {v_end, t_end};
}

// double EvaluateQuinticPolynomial(const std::array<double, 6>& coeffs,
//                                  const double t, const uint32_t order,
//                                  const double end_t, const double end_v) {
//   if (t >= end_t) {
//     switch (order) {
//       case 0: {
//         double end_value =
//             ((((coeffs[5] * end_t + coeffs[4]) * end_t + coeffs[3]) * end_t +
//               coeffs[2]) *
//                  end_t +
//              coeffs[1]) *
//                 end_t +
//             coeffs[0];
//         return end_value + end_v * (t - end_t);
//       }
//       case 1: {
//         return end_v;
//       }
//       default: {
//         return 0.0;
//       }
//     }
//   }
//   switch (order) {
//     case 0: {
//       return ((((coeffs[5] * t + coeffs[4]) * t + coeffs[3]) * t + coeffs[2]) *
//                   t +
//               coeffs[1]) *
//                  t +
//              coeffs[0];
//     }
//     case 1: {
//       return (((5.0 * coeffs[5] * t + 4.0 * coeffs[4]) * t + 3.0 * coeffs[3]) *
//                   t +
//               2.0 * coeffs[2]) *
//                  t +
//              coeffs[1];
//     }
//     case 2: {
//       return (((20.0 * coeffs[5] * t + 12.0 * coeffs[4]) * t) +
//               6.0 * coeffs[3]) *
//                  t +
//              2.0 * coeffs[2];
//     }
//     case 3: {
//       return (60.0 * coeffs[5] * t + 24.0 * coeffs[4]) * t + 6.0 * coeffs[3];
//     }
//     case 4: {
//       return 120.0 * coeffs[5] * t + 24.0 * coeffs[4];
//     }
//     case 5: {
//       return 120.0 * coeffs[5];
//     }
//     default:
//       return 0.0;
//   }
// }

double EvaluateQuarticPolynomial(const std::array<double, 5>& coeffs,
                                 const double t, const uint32_t order,
                                 const double end_t, const double end_v) {
  if (t >= end_t) {
    switch (order) {
      case 0: {
        double end_value =
            (((coeffs[4] * end_t + coeffs[3]) * end_t + coeffs[2]) * end_t +
             coeffs[1]) *
                end_t +
            coeffs[0];
        return end_value + (t - end_t) * end_v;
      }
      case 1: {
        return end_v;
      }
      default: {
        return 0.0;
      }
    }
  }
  switch (order) {
    case 0: {
      return (((coeffs[4] * t + coeffs[3]) * t + coeffs[2]) * t + coeffs[1]) *
                 t +
             coeffs[0];
    }
    case 1: {
      return ((4.0 * coeffs[4] * t + 3.0 * coeffs[3]) * t + 2.0 * coeffs[2]) *
                 t +
             coeffs[1];
    }
    case 2: {
      return (12.0 * coeffs[4] * t + 6.0 * coeffs[3]) * t + 2.0 * coeffs[2];
    }
    case 3: {
      return 24.0 * coeffs[4] * t + 6.0 * coeffs[3];
    }
    case 4: {
      return 24.0 * coeffs[4];
    }
    default:
      return 0.0;
  }
}

double EvaluateCubicPolynomial(const std::array<double, 4>& coefs,
                               const double t, const uint32_t order,
                               const double end_t, const double end_v) {
  if (t > end_t) {
    switch (order) {
      case 0: {
        double end_value =
            ((coefs[3] * end_t + coefs[2]) * end_t + coefs[1]) * end_t +
            coefs[0];
        return end_value + (t - end_t) * end_v;
      }
      case 1: {
        return end_v;
      }
      default: {
        return 0.0;
      }
    }
  }

  switch (order) {
    case 0: {
      return ((coefs[3] * t + coefs[2]) * t + coefs[1]) * t + coefs[0];
    }
    case 1: {
      return (3.0 * coefs[3] * t + 2.0 * coefs[2]) * t + coefs[1];
    }
    case 2: {
      return 6.0 * coefs[3] * t + 2.0 * coefs[2];
    }
    case 3: {
      return 6.0 * coefs[3];
    }
    default:
      return 0.0;
  }
}

double GetSByConstantAcceleration(const double v0, const double acceleration,
                                  const double t) {
  if (acceleration > -FLAGS_double_precision) {
    return v0 * t + 0.5 * acceleration * t * t;
  }
  double t_stop = v0 / (-acceleration);
  double t_actual = std::min(t, t_stop);
  return v0 * t_actual + 0.5 * acceleration * t_actual * t_actual;
}

void TranslatePoint(const double translate_x, const double translate_y,
                    TrajectoryPoint* point) {
  if (point == nullptr || !point->has_path_point()) {
    AERROR << "Point is nullptr or has NO path_point.";
    return;
  }
  const double original_x = point->path_point().x();
  const double original_y = point->path_point().y();
  point->mutable_path_point()->set_x(original_x + translate_x);
  point->mutable_path_point()->set_y(original_y + translate_y);
}

void DrawStitchStopTrajectory(double period,
                              prediction::Trajectory* trajectory) {
  if (trajectory->trajectory_point().empty()) {
    AERROR << "DrawStitchStopTrajectory failed as no point in trajectory.";
    return;
  }
  TrajectoryPoint joint_point;
  joint_point.CopyFrom(
      trajectory->trajectory_point(trajectory->trajectory_point_size() - 1));
  joint_point.set_v(0.0);
  joint_point.set_a(0.0);
  double relative_time = joint_point.relative_time() + period;
  while (relative_time < FLAGS_prediction_trajectory_time_length) {
    joint_point.set_relative_time(relative_time);
    trajectory->add_trajectory_point()->CopyFrom(joint_point);
    relative_time += period;
  }
}

void GenerateFreeMoveTrajectoryPoints(
    Eigen::Matrix<double, 6, 1>* state,
    const Eigen::Matrix<double, 6, 6>& transition, double theta,
    const double start_time, const std::size_t num, const double period,
    double still_speed_th, prediction::Trajectory* trajectory) {

  double x = (*state)(0, 0);
  double y = (*state)(1, 0);
  double v_x = (*state)(2, 0);
  double v_y = (*state)(3, 0);
  double acc_x = (*state)(4, 0);
  double acc_y = (*state)(5, 0);

  double start_heading = std::atan2(v_y, v_x);

  for (std::size_t i = 0; i < num; ++i) {
    double speed = std::hypot(v_x, v_y);
    speed = std::fmin(speed, FLAGS_vehicle_max_speed);
    double acc = 0.0;

    if (!trajectory->trajectory_point().empty()) {
      if (speed < still_speed_th) {
        DrawStitchStopTrajectory(period, trajectory);
        break;
      }
      const auto& prev_trajectory_point =
          trajectory->mutable_trajectory_point()->rbegin();
      PathPoint* prev_path_point = prev_trajectory_point->mutable_path_point();
      theta = std::atan2(y - prev_path_point->y(), x - prev_path_point->x());
      double prev_theta = prev_path_point->theta();
      // if the angle diff is larger than 90 degree, then make it stop.
      if (fabs(common::math::AngleDiff(prev_theta, theta)) > M_PI / 2 ||
          fabs(common::math::AngleDiff(prev_theta, start_heading)) > M_PI / 2) {
        DrawStitchStopTrajectory(period, trajectory);
        break;
      }
      prev_path_point->set_theta(theta);
      acc = (speed - prev_trajectory_point->v()) / period;
      prev_trajectory_point->set_a(acc);
    }

    // update state
    (*state)(2, 0) = v_x;
    (*state)(3, 0) = v_y;
    (*state)(4, 0) = acc_x;
    (*state)(5, 0) = acc_y;

    // obtain position
    x = (*state)(0, 0);
    y = (*state)(1, 0);

    // Generate trajectory point
    auto* trajectory_point = trajectory->add_trajectory_point();
    auto* path_point = trajectory_point->mutable_path_point();
    path_point->set_x(x);
    path_point->set_y(y);
    path_point->set_theta(theta);
    trajectory_point->set_v(speed);
    trajectory_point->set_a(acc);
    trajectory_point->set_relative_time(start_time +
                                        static_cast<double>(i) * period);

    // Update position, velocity and acceleration
    (*state) = transition * (*state);
    x = (*state)(0, 0);
    y = (*state)(1, 0);
    v_x = (*state)(2, 0);
    v_y = (*state)(3, 0);
    acc_x = (*state)(4, 0);
    acc_y = (*state)(5, 0);
  }
}

// double AdjustSpeedByCurvature(const double speed, const double curvature) {
//   if (std::abs(curvature) < FLAGS_turning_curvature_lower_bound) {
//     return speed;
//   }
//   if (std::abs(curvature) > FLAGS_turning_curvature_upper_bound) {
//     return FLAGS_speed_at_upper_curvature;
//   }
//   return TL::common::math::lerp(
//       FLAGS_speed_at_lower_curvature, FLAGS_turning_curvature_lower_bound,
//       FLAGS_speed_at_upper_curvature, FLAGS_turning_curvature_upper_bound,
//       curvature);
// }

std::pair<double, double> WorldToObjCoord(
    std::pair<double, double> input_world_coord,
    std::pair<double, double> obj_world_coord, double obj_world_angle) {
  double x_diff = input_world_coord.first - obj_world_coord.first;
  double y_diff = input_world_coord.second - obj_world_coord.second;
  double sin_theta = std::sin(obj_world_angle);
  double cos_theta = std::cos(obj_world_angle);
  double x = cos_theta * x_diff + sin_theta * y_diff;
  double y = cos_theta * y_diff - sin_theta * x_diff;
  return std::make_pair(x, y);
}

std::pair<double, double> WorldToObjCoordNorth(
    std::pair<double, double> input_world_coord,
    std::pair<double, double> obj_world_coord, double obj_world_angle) {
  double x_diff = input_world_coord.first - obj_world_coord.first;
  double y_diff = input_world_coord.second - obj_world_coord.second;
  double sin_theta = std::sin(obj_world_angle);
  double cos_theta = std::cos(obj_world_angle);
  double x = sin_theta * x_diff - cos_theta * y_diff;
  double y = cos_theta * x_diff + sin_theta * y_diff;
  return std::make_pair(x, y);
}

double WorldAngleToObjAngle(double input_world_angle, double obj_world_angle) {
  return common::math::NormalizeAngle(input_world_angle - obj_world_angle);
}

// Eigen::MatrixXf VectorToMatrixXf(const std::vector<double>& nums,
//                                  const int start_index, const int end_index) {
//   CHECK_LT(start_index, end_index);
//   CHECK_GE(start_index, 0);
//   CHECK_LE(end_index, static_cast<int>(nums.size()));
//   Eigen::MatrixXf output_matrix;
//   output_matrix.resize(1, end_index - start_index);
//   for (int i = start_index; i < end_index; ++i) {
//     output_matrix(0, i - start_index) = static_cast<float>(nums[i]);
//   }
//   return output_matrix;
// }

// Eigen::MatrixXf VectorToMatrixXf(const std::vector<double>& nums,
//                                  const int start_index, const int end_index,
//                                  const int output_num_row,
//                                  const int output_num_col) {
//   CHECK_LT(start_index, end_index);
//   CHECK_GE(start_index, 0);
//   CHECK_LE(end_index, static_cast<int>(nums.size()));
//   CHECK_EQ(end_index - start_index, output_num_row * output_num_col);
//   Eigen::MatrixXf output_matrix;
//   output_matrix.resize(output_num_row, output_num_col);
//   int input_index = start_index;
//   for (int i = 0; i < output_num_row; ++i) {
//     for (int j = 0; j < output_num_col; ++j) {
//       output_matrix(i, j) = static_cast<float>(nums[input_index]);
//       ++input_index;
//     }
//   }
//   CHECK_EQ(input_index, end_index);
//   return output_matrix;
// }

/*
  * when p1,p2,p3 counter-clockwise, the result is positive and
  * when p1,p2,p3 clockwise, the result is negative.
  */
double point_on_line_side(const common::Point2D& p1, const common::Point2D& p2,
                          const common::Point2D& p3) {
  return ((p1.x() - p3.x()) * (p2.y() - p3.y()) -
          (p1.y() - p3.y()) * (p2.x() - p3.x()));
}

Eigen::Vector2d EvaluateCubicBezierPoint(
    const std::vector<Eigen::Vector2d>& control_points, double t) {
  double a = std::max(t, 0.0);
  a = std::min(a, 1.0);

  double b = 1 - a;
  double coeff0 = b * b * b;
  double coeff1 = b * b * a;
  double coeff2 = b * a * a;
  double coeff3 = a * a * a;

  double x =
      coeff0 * control_points[0].x() + 3 * coeff1 * control_points[1].x() +
      3 * coeff2 * control_points[2].x() + coeff3 * control_points[3].x();
  double y =
      coeff0 * control_points[0].y() + 3 * coeff1 * control_points[1].y() +
      3 * coeff2 * control_points[2].y() + coeff3 * control_points[3].y();

  return Eigen::Vector2d{x, y};
}

}  // namespace prediction_util
}  // namespace prediction
}  // namespace TL
