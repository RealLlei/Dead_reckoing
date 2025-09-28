/******************************************************************************
 * Copyright
 *
*/
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "proto/prediction/feature.pb.h"

namespace TL {
namespace prediction {

/**
 * @brief Base Class of Longitudinal Strategies
 *
 */
class LongitudinalStrategy {
 public:
  LongitudinalStrategy() = default;
  virtual ~LongitudinalStrategy() = default;

  virtual void Init(const std::array<double, 3>& init_s,
                    const LaneSequence& lane_sequence) = 0;
  virtual std::array<double, 3> GenerateLongitudinalState(double relative_time,
                                                          double period) = 0;
};

/**
 * @brief Using polynomial function to generate st curve
 * 
 */
class PolynomialStrategy : public LongitudinalStrategy {
 public:
  PolynomialStrategy() = default;

  void Init(const std::array<double, 3>& init_s,
            const LaneSequence& lane_sequence) override;
  std::array<double, 3> GenerateLongitudinalState(double relative_time,
                                                  double period) override;

 private:
  static std::pair<double, double> ComputeLonEndState(
      const std::array<double, 3>& init_s, const LaneSequence& lane_sequence);

 private:
  std::array<double, 5> longitudinal_coeffs_{};
  std::pair<double, double> lon_end_vt_;
};

/**
 * @brief Using IDM to generate longitudinal state
 * 
 */
class IDMStrategy : public LongitudinalStrategy {
 public:
  IDMStrategy() = default;
  void Init(const std::array<double, 3>& init_s,
            const LaneSequence& lane_sequence) override;
  std::array<double, 3> GenerateLongitudinalState(double relative_time,
                                                  double period) override;

 private:
  double prev_s_ = 0.0;
  double lane_speed_ = 0.0;
  double lane_acc_ = 0.0;
  double follow_s_ = 0.0;
  double follow_v_ = 0.0;
};

/**
 * @brief Using const acc model
 * 
 */
class ConstAccStrategy : public LongitudinalStrategy {
 public:
  ConstAccStrategy() = default;
  void Init(const std::array<double, 3>& init_s,
            const LaneSequence& lane_sequence) override;
  std::array<double, 3> GenerateLongitudinalState(double relative_time,
                                                  double period) override;

 private:
  double lane_speed_{};
  double lane_acc_{};
};

}  // namespace prediction
}  // namespace TL
