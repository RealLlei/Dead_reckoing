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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *implied. See the License for the specific language governing
 *permissions and limitations under the License.
 *****************************************************************************/

#pragma once

#include <limits>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "common/file/log.h"
#include "common/math/math_utils.h"
#include "planning/prediction/common/prediction_gflags.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/prediction/feature.pb.h"

namespace TL {
namespace prediction {
namespace prediction_util {
/**
 * @brief Normalize the value by specified mean and standard deviation.
 * @param value The value to be normalized.
 * @param mean The mean used for normalization.
 * @param std The standard deviation used for normalization.
 * @return The normalized value.
 */
double Normalize(double value, double mean, double std);

// Helper function for computing the mean value of a vector.
double ComputeMean(const std::vector<double>& nums, size_t start, size_t end);

/**
 * @brief RELU function used in neural networks as an activation function.
 * @param value The input.
 * @return The output of RELU function.
 */
double Relu(double value);

/**
 * @brief Softmax function used in neural networks as an activation function.
 * @param vector The input.
 * @return The output of Softmax function.
 */
std::vector<double> Softmax(const std::vector<double>& value,
                            bool use_exp = true);

void DrawStitchStopTrajectory(double period,
                              prediction::Trajectory* trajectory);

// /**
//  * @brief Solve quadratic equation.
//  * @param coefficients The coefficients of quadratic equation.
//  * @param roots Two roots of the equation if any.
//  * @return An integer indicating the success of solving equation.
//  */
// int SolveQuadraticEquation(const std::vector<double>& coefficients,
//                            std::pair<double, double>* roots);
/**
* @brief get cubic polynomial coefficients for the lat/lon prediction
*
* @param x0 x0 position at the start of the lane
* @param dx0 dx0 velocity at the start of the lane
* @param x1  x1 position at the end of the t
* @param dx1 dx1 velocity at the end of the lane
* @param p time to reach the end of the lane
* @param coefficients polynomial coefficients
*/

void GetCubicPolynomial(double x0, double dx0, double x1, double dx1, double p,
                        std::array<double, 4>* coefficients);
/**
 * @brief Get the quartic polynomial coefficients
 *
 * @param x0
 * @param dx0
 * @param ddx0
 * @param dx1
 * @param ddx1
 * @param p end time
 * @param coefficients
 */
void GetQuarticPolynomial(double x0, double dx0, double ddx0, double dx1,
                          double ddx1, double p,
                          std::array<double, 5>* coefficients);
/**
 * @brief Get the quartic polynomial coefficients
 *
 * @param x0
 * @param dx0
 * @param ddx0
 * @param dddx0
 * @param coefficients
 */
std::pair<double, double> GetQuarticPolynomial(
    double x0, double dx0, double ddx0, std::array<double, 5>* coefficients,
    double dddx0 = 0.5);
// /**
//  * @brief Evaluate quintic polynomial.
//  * @param coefficients of the quintic polynomial, lower to higher.
//  * @param parameter of the quintic polynomial.
//  * @return order of derivative to evaluate.
//  */
// double EvaluateQuinticPolynomial(const std::array<double, 6>& coeffs,
//                                  const double t, const uint32_t order,
//                                  const double end_t, const double end_v);

/**
 * @brief Evaluate quartic polynomial.
 * @param coefficients of the quartic polynomial, lower to higher.
 * @param parameter of the quartic polynomial.
 * @return order of derivative to evaluate.
 */
double EvaluateQuarticPolynomial(const std::array<double, 5>& coeffs, double t,
                                 uint32_t order, double end_t, double end_v);

/**
 * @brief Evaluate cubic polynomial.
 * @param coefficients of the cubic polynomial, lower to higher.
 * @param parameter of the cubic polynomial.
 * @param end_t ending time for extrapolation.
 * @param end_v ending velocity for extrapolation.
 * @return order of derivative to evaluate.
 */
double EvaluateCubicPolynomial(
    const std::array<double, 4>& coefs, double t, uint32_t order,
    double end_t = std::numeric_limits<double>::infinity(), double end_v = 0.0);

template <std::size_t N>
std::array<double, 2 * N - 2> ComputePolynomial(
    const std::array<double, N - 1>& start_state,
    const std::array<double, N - 1>& end_state, double param);

template <>
inline std::array<double, 4> ComputePolynomial<3>(
    const std::array<double, 2>& start_state,
    const std::array<double, 2>& end_state, const double param) {
  std::array<double, 4> coefs{};
  coefs[0] = start_state[0];
  coefs[1] = start_state[1];

  auto m0 = end_state[0] - start_state[0] - start_state[1] * param;
  auto m1 = end_state[1] - start_state[1];

  auto param_p3 = param * param * param;
  coefs[3] = (m1 * param - 2.0 * m0) / param_p3;

  coefs[2] = (m1 - 3.0 * coefs[3] * param * param) / param * 0.5;
  return coefs;
}

double GetSByConstantAcceleration(double v0, double acceleration, double t);

/**
 * @brief Translate a point.
 * @param translate_x The translation along x-axis.
 * @param translate_y The translation along y-axis.
 * @param point The point to be translated.
 */
void TranslatePoint(double translate_x, double translate_y,
                    common::TrajectoryPoint* point);

/**
 * @brief Generate a set of free move trajectory points
 * @param state matrix
 * @param transition matrix
 * @param heading
 * @param start time
 * @param total number of generated trajectory points required
 * @param trajectory point interval period
 * @param generated trajectory points
 */
void GenerateFreeMoveTrajectoryPoints(
    Eigen::Matrix<double, 6, 1>* state,
    const Eigen::Matrix<double, 6, 6>& transition, double theta,
    double start_time, std::size_t num, double period, double still_speed_th,
    prediction::Trajectory* trajectory);

void GenerateStitchStopTrajectory(double period,
                                  prediction::Trajectory* trajectory);

// /**
//  * @brief Adjust a speed value according to a curvature. If the input speed
//  *        is okay on the input curvature, return the original speed, otherwise,
//  *        adjust the speed.
//  * @param speed The original speed value.
//  * @param curvature The curvature value.
//  * @return The adjusted speed according to the curvature.
//  */
// double AdjustSpeedByCurvature(const double speed, const double curvature);

// Helper function to convert world coordinates to relative coordinates
// around the obstacle of interest.
std::pair<double, double> WorldToObjCoord(
    std::pair<double, double> input_world_coord,
    std::pair<double, double> obj_world_coord, double obj_world_angle);

std::pair<double, double> WorldToObjCoordNorth(
    std::pair<double, double> input_world_coord,
    std::pair<double, double> obj_world_coord, double obj_world_angle);

double WorldAngleToObjAngle(double input_world_angle, double obj_world_angle);

// Eigen::MatrixXf VectorToMatrixXf(const std::vector<double>& nums,
//                                  const int start_index, const int end_index);

// Eigen::MatrixXf VectorToMatrixXf(const std::vector<double>& nums,
//                                  const int start_index, const int end_index,
//                                  const int output_num_row,
//                                  const int output_num_col);

/*
  * when p1,p2,p3 counter-clockwise, the result is positive and
  * when p1,p2,p3 clockwise, the result is negative.
  */
double point_on_line_side(const common::Point2D& p1, const common::Point2D& p2,
                          const common::Point2D& p3);

/**
 * @brief Evaluate cubic bezier point
 * @param control_points The bezier curve control points.
 * @param t from 0 to 1.
 * @param return one bezier point.
 */
Eigen::Vector2d EvaluateCubicBezierPoint(
    const std::vector<Eigen::Vector2d>& control_points, double t);

}  // namespace prediction_util
}  // namespace prediction
}  // namespace TL
