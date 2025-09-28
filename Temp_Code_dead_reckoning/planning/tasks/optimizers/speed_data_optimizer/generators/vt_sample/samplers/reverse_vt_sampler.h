/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file reverse_normal_vt_sampler.h
 **/

#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/curves/quadratic_vt_curve.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/samplers/vt_sampler.h"

#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

/**
 * @class ReverseVtSampler
 * @brief this class is used to sample normal vt curve
 */
class ReverseVtSampler : public VtSampler {
 public:
  /**
   * @brief Construct a new normal vt sampler object
   *
   * @param config
   * @param cache
   */
  ReverseVtSampler(const VtSamplerConfig& sampler_config,
                   const SpeedCurveConfig& curve_config);

  /**
   * @brief Generate guide vt sample curves
   *
   * @param init_point planning start point
   */
  void SampleGuideCurves(const common::TrajectoryPoint& init_point,
                         const SpeedCache& cache) override;

  /**
   * @brief Generate safe vt sample curves
   *
   * @param init_point planning start point
   */
  void SampleSafeCurves(
      const common::TrajectoryPoint& init_point, const SpeedCache& cache,
      const SLTObstacleCache& obstacle_cache, const bool& set_extend,
      const std::vector<double>& sample_time_extend_time) override;
  /**
   * @brief Generate vt sample curves for normal type, this means accel is
   * limited in [vt_sample_config_.min_normal_accel(),
   * vt_sample_config_.max_normal_accel()] and jerk is limited in
   * [vt_sample_config_.min_normal_jerk(),
   * vt_sample_config_.max_normal_jerk()]
   *
   * @param init_point planning start point
   * @param reference_line_info current reference_line info
   */
  void SampleNormalCurves(
      const common::TrajectoryPoint& init_point,
      const ReferenceLineInfo& reference_line_info, const SpeedCache& cache,
      const bool& set_extend,
      const std::vector<double>& sample_time_extend_time) override;

  /**
   * @brief Add last frame selected curve
   *
   * @param last_curve
   * @return true
   * @return false
   */
  bool AddLastCurve(const std::shared_ptr<SpeedCurve>& last_curve,
                    const SpeedCache& cache) override;

  /**
   * @brief Generate fallback speed data
   *
   * @param init_point planning start point
   * @param cost vt sample cost
   * @param speed_data speed data
   * @return true successed
   * @return false failed
   */
  bool GenerateFallbackSpeedData(
      const common::TrajectoryPoint& init_point,
      const ReferenceLineInfo& reference_line_info, const SpeedCache& cache,
      const std::shared_ptr<SpeedEvaluator>& evaluator,
      SpeedData* speed_data) override;

 private:
  /**
   * @brief Generate vt sample curves in cruise mode for normal type
   *
   * @param init_point planning start point
   * @param vt_sample_curves generated curves
   */
  void SampleCruiseCurves(const common::TrajectoryPoint& init_point,
                          const SpeedCache& cache);

  /**
   * @brief Generate vt sample curves in stop mode for normal type
   *
   * @param init_point planning start point
   * @param vt_sample_curves generated curves
   */
  void SampleStopCurves(const common::TrajectoryPoint& init_point,
                        const SpeedCache& cache);

  /**
   * @brief Generate vt sample curves in stop mode for normal type
   *
   * @param init_point planning start point
   * @param vt_sample_curves generated curves
   */
  void SampleUnknownCurves(const common::TrajectoryPoint& init_point,
                           const SpeedCache& cache);

  /**
   * @brief Check if adc need stop curve
   *
   * @return true
   * @return false
   */
  [[nodiscard]] bool CheckNeedStopCurve(double start_v, double stop_s);

  /**
   * @brief Check if adc need unknown curve
   *
   * @return true
   * @return false
   */
  [[nodiscard]] bool CheckNeedUnknownCurve() const {
    // if expected stop s is inf, we do not need stop curve
    return !std::isinf(min_obstacle_st_boundary_);
  }

  /**
   * @brief generate vt sample curve according to curve start point, end_t,
   * end_v and end_a
   *
   * @param init_point curve start point
   * @param end_t curve t length
   * @param end_v curve end speed
   * @param vt_sample_curves generated curves
   * @return int generated curves count
   */
  int SampleCurveForSpeed(const common::TrajectoryPoint& init_point,
                          double end_v, const SpeedCurveTarget& target);

  /**
   * @brief generate vt sample curve according to curve start point, end_s,
   * end_v and end_s
   *
   * @param init_point curve start point
   * @param end_s curve s length
   * @param vt_sample_curves vt sample target
   * @return int generated curves count
   */
  int SampleCurveForLength(const common::TrajectoryPoint& init_point,
                           double end_s, const SpeedCurveTarget& target);

  ReverseVtSamplerConfig config_;
  std::vector<std::shared_ptr<QuadraticVTCurve>> quadratic_vt_curves_;
  std::size_t quadratic_vt_curve_count_ = 0;
  double min_obstacle_st_boundary_ = std::numeric_limits<double>::infinity();
  std::vector<double> sample_accels_;
};

}  // namespace planning
}  // namespace TL
