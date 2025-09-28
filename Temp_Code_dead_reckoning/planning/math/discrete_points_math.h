/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning discrete points math calculate
 * Author: ROC
 */

#pragma once

#include <limits>
#include <map>
#include <type_traits>
#include <utility>
#include <vector>

#include "osqp/osqp.h"

namespace TL {
namespace planning {

namespace DiscretePointsMath {
/**
 * @brief Compute Discrete Points Profile(c_float type value)
 *
 * @param xy_points
 * @param eval_x
 * @param eval_y
 * @param eval_heading
 * @param accumulated_s
 * @param kappas
 * @param dkappas
 * @return true
 * @return false
 */
bool ComputeDiscretePointsProfile(
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<c_float>* eval_x, std::vector<c_float>* eval_y,
    std::vector<c_float>* eval_heading, std::vector<c_float>* accumulated_s,
    std::vector<c_float>* kappas, std::vector<c_float>* dkappas);

/**
 * @brief Compute Discrete Points Profile(c_float type value)
 *
 * @param xy_points
 * @param eval_x
 * @param eval_y
 * @param eval_heading
 * @return true
 * @return false
 */
bool ComputeDiscretePointsProfile(
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<c_float>* eval_x, std::vector<c_float>* eval_y,
    std::vector<c_float>* eval_heading);

/**
 * @brief Compute Discrete Points Profile(double type value)
 *
 * @param xy_points
 * @param headings
 * @param accumulated_s
 * @param kappas
 * @param dkappas
 * @return true
 * @return false
 */
bool ComputeDiscretePointsProfile(
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<double>* headings, std::vector<double>* accumulated_s,
    std::vector<double>* kappas, std::vector<double>* dkappas);

/**
 * @brief Reference Line Smoother Solution To OptXY
 *
 * @param solution
 * @param opt_x
 * @param opt_y
 * @param xy_points
 */
void SolutionToOptXY(const std::vector<c_float>& solution,
                     std::vector<c_float>* opt_x, std::vector<c_float>* opt_y,
                     std::vector<std::pair<double, double>>* xy_points);

/**
 * @brief Compute Cosine Theorem Theta
 *
 * @param s_a Both sides of angle A(a_b) a side lane length
 * @param s_b Both sides of angle A(a_b) b side lane length
 * @param s_c Opposite side of angle A(a_b) c side lane length
 * @return double
 */
double ComputeCosineTheoremTheta(double s_a, double s_b, double s_c);

/**
 * @brief Compute Two Points Distance
 *
 * @param point_a_x point A(x,y) x value
 * @param point_a_y point A(x,y) y value
 * @param point_b_x point B(x,y) x value
 * @param point_b_y point B(x,y) y value
 * @return double
 */
double ComputeTwoPointDistance(double point_a_x, double point_a_y,
                               double point_b_x, double point_b_y);

template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
AlmostEqual(T x, T y, int ulp) {
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return fabs(x - y) <= std::numeric_limits<T>::epsilon() * fabs(x + y) * ulp
         // unless the result is subnormal
         || fabs(x - y) < std::numeric_limits<T>::min();
}
};  // namespace DiscretePointsMath
}  // namespace planning
}  // namespace TL
