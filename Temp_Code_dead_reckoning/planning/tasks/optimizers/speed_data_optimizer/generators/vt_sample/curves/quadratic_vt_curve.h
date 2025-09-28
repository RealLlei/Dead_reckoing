/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file quadratic_vt_curve.h
 **/

#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "planning/math/curve1d/quadratic_polynomial_curve1d.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_curve.h"

#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class VtSampleCurve
 * @brief vt sample curve
 */
class alignas(CACHELINE_SIZE) QuadraticVTCurve : public SpeedCurve {
 public:
  explicit QuadraticVTCurve(const SpeedCurveConfig& config);
  ~QuadraticVTCurve() override = default;

  /**
   * @brief Init piecewise jerk curve
   *
   * @param init_point curve start point
   * @param params t and j of each piece
   * @param target vt sample target
   * @return true init successed
   * @return false init failed
   */
  bool InitPiecewiseJerkCurve(
      const common::TrajectoryPoint& init_point,
      const std::vector<std::pair<double, double>>& params,
      const SpeedCurveTarget& target, bool allow_negative_speed = false);

  /**
   * @brief Init piecewise accel curve
   *
   * @param init_point curve start point
   * @param params t and a of each piece
   * @param target vt sample target
   * @return true init successed
   * @return false init failed
   */
  bool InitPiecewiseAccelCurve(
      const common::TrajectoryPoint& init_point,
      const std::vector<std::pair<double, double>>& params, double end_accel,
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
   * @brief Get vt graph point
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
  std::array<QuadraticPolynomialCurve1d, 4> v_pieces_;
  std::array<double, 5> accumulated_time_ = {};
  std::size_t piece_count_ = 3;
  bool allow_negative_speed_ = false;
};

}  // namespace planning
}  // namespace TL
