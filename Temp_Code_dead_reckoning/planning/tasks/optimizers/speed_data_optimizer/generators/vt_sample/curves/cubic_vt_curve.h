/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file cubic_vt_curve.h
 **/

#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "planning/math/curve1d/cubic_polynomial_curve1d.h"
#include "planning/math/curve1d/linear_polynomial_curve1d.h"
#include "planning/math/curve1d/quadratic_polynomial_curve1d.h"
#include "planning/math/curve1d/quartic_polynomial_curve1d.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_curve.h"

#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class CubicVTCurve
 * @brief cubic vt curve
 */
class alignas(CACHELINE_SIZE) CubicVTCurve : public SpeedCurve {
 public:
  explicit CubicVTCurve(const SpeedCurveConfig& config);
  ~CubicVTCurve() override = default;

  /**
   * @brief Init curve
   *
   * @param init_point curve start point
   * @param end_t end time
   * @param end_v end speed
   * @param end_a end accel
   * @param target vt sample target
   * @return true init successed
   * @return false init failed
   */
  bool Init(const common::TrajectoryPoint& init_point, double end_t,
            double end_v, double end_a, const SpeedCurveTarget& target);

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
   * @brief Clone curve
   * 
   * @return std::shared_ptr<SpeedCurve> 
   */
  std::shared_ptr<SpeedCurve> Clone() const override;

  /**
   * @brief Get debug string
   * 
   * @return std::string 
   */
  std::string DebugString() const override;

 private:
  QuarticPolynomialCurve1d s_piece_;
  CubicPolynomialCurve1d v_piece_;
  QuadraticPolynomialCurve1d a_piece_;
  LinearPolynomialCurve1d j_piece_;
};

}  // namespace planning
}  // namespace TL
