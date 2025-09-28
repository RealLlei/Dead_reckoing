/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#include "planning/self_simulator/sim_dummy_obs/common/util_com.h"

namespace TL {
namespace simdummy {

bool CoordinateSystemEarth2Ego(double x1, double y1, double theta, double x2,
                               double y2,
                               std::pair<double, double>* const new_point) {
  if (new_point != nullptr) {
    double delta_y = y2 - y1;
    double delta_x = x2 - x1;
    double theta_x2 = atan(delta_y / delta_x);
    double vector_modulus_length = sqrt(delta_x * delta_x + delta_y * delta_y);
    new_point->first = vector_modulus_length * cos(theta_x2 - theta);
    new_point->second = vector_modulus_length * sin(theta_x2 - theta);
    return true;
  } else {
    return false;
  }
}

bool CoordinateSystemEgo2Earth(double x1, double y1, double theta, double x2,
                               double y2,
                               std::pair<double, double>* const new_point) {
  if (new_point != nullptr) {
    // lp: system rotate
    double neg_theta = -theta;
    double x2_rot = x2 * cos(neg_theta) + y2 * sin(neg_theta);
    double y2_rot = y2 * cos(neg_theta) - x2 * sin(neg_theta);
    // lp: system pan
    new_point->first = x1 + x2_rot;
    new_point->second = y1 + y2_rot;
    return true;
  } else {
    return false;
  }
}

}  // namespace simdummy
}  // namespace TL
