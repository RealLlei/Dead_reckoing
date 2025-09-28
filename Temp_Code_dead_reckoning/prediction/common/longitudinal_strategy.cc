/******************************************************************************
 * Copyright
 *
*/

#include "planning/prediction/common/longitudinal_strategy.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>

#include "planning/prediction/common/prediction_gflags.h"
#include "planning/prediction/common/prediction_util.h"
#include "planning/prediction/proto/dbn_model.pb.h"
#include "proto/common/types.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/prediction/feature.pb.h"

namespace TL {
namespace prediction {
using TL::prediction::prediction_util::EvaluateQuarticPolynomial;
using TL::prediction::prediction_util::GetQuarticPolynomial;

void PolynomialStrategy::Init(const std::array<double, 3>& init_s,
                              const LaneSequence& lane_sequence) {
  // assumed end point method
  // lon_end_vt_ = ComputeLonEndState(init_s, lane_sequence);
  // GetQuarticPolynomial(init_s[0], init_s[1], init_s[2], lon_end_vt_.first, 0.0,
  //                      lon_end_vt_.second, &longitudinal_coeffs_);
  UNUSED(lane_sequence);

  // const jerk method
  lon_end_vt_ = GetQuarticPolynomial(init_s[0], init_s[1], init_s[2],
                                     &longitudinal_coeffs_);
  ADEBUG << "using polynomial strategy,init_v :" << init_s[1]
         << ", init a:" << init_s[2] << "lon_end_v : " << lon_end_vt_.first
         << ",lon_end_t" << lon_end_vt_.second;
}

std::array<double, 3> PolynomialStrategy::GenerateLongitudinalState(
    double relative_time, double period) {
  double curr_s =
      EvaluateQuarticPolynomial(longitudinal_coeffs_, relative_time, 0,
                                lon_end_vt_.second, lon_end_vt_.first);
  double prev_s = (relative_time + FLAGS_double_precision > period)
                      ? EvaluateQuarticPolynomial(
                            longitudinal_coeffs_, relative_time - period, 0,
                            lon_end_vt_.second, lon_end_vt_.first)
                      : 0.0;
  double lane_speed =
      EvaluateQuarticPolynomial(longitudinal_coeffs_, relative_time, 1,
                                lon_end_vt_.second, lon_end_vt_.first);
  double lane_acc =
      EvaluateQuarticPolynomial(longitudinal_coeffs_, relative_time, 2,
                                lon_end_vt_.second, lon_end_vt_.first);
  return {std::max(0.0, (curr_s - prev_s)), lane_speed, lane_acc};
}

void IDMStrategy::Init(const std::array<double, 3>& init_s,
                       const LaneSequence& lane_sequence) {
  prev_s_ = init_s[0];
  lane_speed_ = init_s[1];
  lane_acc_ = init_s[2];
  const auto& follow_obstacle = lane_sequence.front_nearest_obstacle();
  follow_v_ = follow_obstacle.speed();
  follow_s_ = follow_obstacle.s();
  ADEBUG << "using IDM strategy, init_s : " << prev_s_ << ",init_v"
         << lane_speed_ << ",init_a :" << lane_acc_ << ",follow_s:" << follow_s_
         << ",follow_v:" << follow_v_;
}

std::array<double, 3> IDMStrategy::GenerateLongitudinalState(
    double relative_time, double period) {
  static constexpr double safe_time = 0.3;
  static constexpr double safe_distance = 5.0;
  static constexpr double max_jerk = 1.0;

  double s_safe = safe_distance + std::max(0.0, lane_speed_ * safe_time);
  double delta_s = follow_s_ + relative_time * follow_v_ - prev_s_;

  double acc_safe = FLAGS_vehicle_min_linear_acc;
  if (follow_v_ > FLAGS_double_precision && delta_s > FLAGS_double_precision) {
    acc_safe = FLAGS_vehicle_max_linear_acc *
               (1 - (lane_speed_ / follow_v_) -
                (s_safe / delta_s) * (s_safe / delta_s));
  }
  acc_safe = common::math::Clamp(acc_safe, FLAGS_vehicle_min_linear_acc,
                                 FLAGS_vehicle_max_linear_acc);

  if (std::abs(acc_safe - lane_acc_) / period > max_jerk) {
    if (acc_safe > lane_acc_) {
      acc_safe = lane_acc_ + max_jerk * period;
    } else {
      acc_safe = lane_acc_ - max_jerk * period;
    }
  }

  if (relative_time + FLAGS_double_precision < period) {
    return {0.0, lane_speed_, lane_acc_};
  }

  double period_s = lane_speed_ * period + 0.5 * acc_safe * period * period;
  lane_speed_ = std::max(0.0, lane_speed_ + acc_safe * period);

  prev_s_ += period_s;
  lane_acc_ = acc_safe;
  return {period_s, lane_speed_, lane_acc_};
}

void ConstAccStrategy::Init(const std::array<double, 3>& init_s,
                            const LaneSequence& lane_sequence) {
  UNUSED(lane_sequence);
  ADEBUG << "using ConstAcc strategy";
  lane_speed_ = init_s[1];
  lane_acc_ = init_s[2];
}

std::array<double, 3> ConstAccStrategy::GenerateLongitudinalState(
    double relative_time, double period) {
  if (relative_time + FLAGS_double_precision < period) {
    return {0.0, lane_speed_, lane_acc_};
  }
  if (lane_speed_ <= FLAGS_double_precision) {
    lane_speed_ = 0.0;
    lane_acc_ = 0.0;
  }

  double delta_s = lane_speed_ * period + 0.5 * lane_acc_ * period * period;
  lane_speed_ += lane_acc_ * period;
  return {delta_s, lane_speed_, lane_acc_};
}

std::pair<double, double> PolynomialStrategy::ComputeLonEndState(
    const std::array<double, 3>& init_s, const LaneSequence& lane_sequence) {
  UNUSED(lane_sequence);
  // If the max. curvature is small (almost straight lane),
  // then predict that the obstacle will keep current speed.
  double v_init = init_s[1];
  static constexpr double k_acc_high_pass = 0.2;

  // if acceleration, acc lasts 2 seconds
  // elif braking , acc last 4 seconds
  // else : acc = 0

  double end_v = init_s[1];
  double end_t = FLAGS_prediction_trajectory_time_length;

  if (init_s[2] > k_acc_high_pass) {
    end_t = FLAGS_prediction_trajectory_time_length / 4;
    end_v = v_init + init_s[2] * end_t;
  } else if (init_s[2] < -k_acc_high_pass) {
    // end_t = FLAGS_prediction_trajectory_time_length / 2;
    end_t = fmin(-init_s[1] / init_s[2],
                 FLAGS_prediction_trajectory_time_length / 2);
    end_v = v_init + init_s[2] * end_t;
  }
  end_v = fmax(0.0, end_v);
  end_t = fmax(FLAGS_min_poly_time_length, end_t);

  ADEBUG << "init v:" << init_s[1] << " , init a:" << init_s[2]
         << "end_v : " << end_v << "end_t : " << end_t;
  return {end_v, end_t};
}

}  // namespace prediction
}  // namespace TL
