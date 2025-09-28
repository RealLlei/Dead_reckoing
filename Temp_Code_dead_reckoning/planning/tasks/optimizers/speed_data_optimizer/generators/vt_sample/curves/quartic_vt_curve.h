/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file quartic_vt_curve.h
 **/

#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "planning/math/curve1d/cubic_polynomial_curve1d.h"
#include "planning/math/curve1d/quadratic_polynomial_curve1d.h"
#include "planning/math/curve1d/quartic_polynomial_curve1d.h"
#include "planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_curve.h"

#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class QuarticVTCurve
 * @brief quartic st curve
 */
class alignas(CACHELINE_SIZE) QuarticVTCurve : public SpeedCurve {
 public:
  explicit QuarticVTCurve(const SpeedCurveConfig& config);
  ~QuarticVTCurve() override = default;

  /**
   * @brief Init curve
   *
   * @param init_point curve start point
   * @param end_t end time
   * @param end_s end s 
   * @param end_v end speed
   * @param end_a end accel
   * @param target vt sample target
   * @return true init successed
   * @return false init failed
   */
  bool InitST(const common::TrajectoryPoint& init_point, double end_t,
              double end_s, double end_v, double end_a,
              const SpeedCurveTarget& target);

  /**
   * @brief Init curve
   *
   * @param init_point curve start point
   * @param end_t end time
   * @param end_v end v 
   * @param end_a end accel
   * @param end_j end jerk
   * @param target vt sample target
   * @return true init successed
   * @return false init failedV
   */
  bool InitVT(const common::TrajectoryPoint& init_point, double end_t,
              double end_v, double end_a, double end_j,
              const SpeedCurveTarget& target);

  /**
   * @brief Discretize the curve to vt graph points
   *
   * @param total_t total time length
   * @param unit_t sample interval
   * @param process_peak_value true: process peak value, false ignore peak value
   * @param vt_graph_points
   * @return int
   */
  int Discretize(double total_t, double unit_t, bool process_peak_value,
                 std::vector<SpeedCurvePoint>* vt_graph_points) const override;

  /**
   * @brief Get st graph point
   *
   * @param relative_t
   * @return StGraphPoint
   */
  [[nodiscard]] SpeedCurvePoint GetPoint(double relative_t) const override;

  /**
   * @brief Set start point
   *
   * @param estimated_start_time
   * @return true
   * @return false
   */
  bool SetStartPoint(double estimated_start_time) override;

  /**
   * @brief get s length
   *
   * @return double
   */
  [[nodiscard]] double GetSLength() const override {
    return end_s_ + extend_s_ - start_s_;
  }

  /**
   * @brief Clone curve
   * 
   * @return std::shared_ptr<VtSampleCurve> 
   */
  [[nodiscard]] std::shared_ptr<SpeedCurve> Clone() const override;

  /**
   * @brief Get debug string
   * 
   * @return std::string 
   */
  [[nodiscard]] std::string DebugString() const override;

 private:
  QuinticPolynomialCurve1d s_piece_;
  QuarticPolynomialCurve1d v_piece_;
  CubicPolynomialCurve1d a_piece_;
  QuadraticPolynomialCurve1d j_piece_;
  double extend_t_ = 0.0;
  double extend_s_ = 0.0;
};

}  // namespace planning
}  // namespace TL
