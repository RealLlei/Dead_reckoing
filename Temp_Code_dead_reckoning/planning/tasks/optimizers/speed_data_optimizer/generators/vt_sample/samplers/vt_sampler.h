/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_optimizer.h
 **/

#pragma once

#include <memory>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/rules/vt_sample_rule.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

/**
 * @class VtSampler
 * @brief this class is base vt sampler
 */
class VtSampler {
 public:
  /**
   * @brief Construct a new vt sampler object
   *
   * @param config
   * @param cache
   */
  VtSampler(const VtSamplerConfig& sampler_config,
            const SpeedCurveConfig& curve_config);
  virtual ~VtSampler() = default;

  /**
   * @brief Generate guide vt curves
   *
   * @param init_point planning start point
   */
  virtual void SampleGuideCurves(const common::TrajectoryPoint& init_point,
                                 const SpeedCache& cache) = 0;

  /**
   * @brief Generate safe vt curves
   *
   * @param init_point planning start point
   */
  virtual void SampleSafeCurves(
      const common::TrajectoryPoint& init_point, const SpeedCache& cache,
      const SLTObstacleCache& obstacle_cache, const bool& set_extend,
      const std::vector<double>& sample_time_extend_time) = 0;
  /**
   * @brief Generate normal vt curves
   *
   * @param init_point planning start point
   * @param reference_line_info current reference_line info&
   */
  virtual void SampleNormalCurves(
      const common::TrajectoryPoint& init_point,
      const ReferenceLineInfo& reference_line_info, const SpeedCache& cache,
      const bool& set_extend,
      const std::vector<double>& sample_time_extend_time) = 0;

  /**
   * @brief Get guide vt curves
   *
   * @return const std::vector<VtSampleCurve>
   */
  [[nodiscard]] const std::vector<SpeedCurveCostResult>&
  GetGuideCurveCostResults() const {
    return guide_curve_cost_results_;
  }

  /**
   * @brief Get comfortable vt curves
   *
   * @return const std::vector<VtSampleCurve>
   */
  [[nodiscard]] const std::vector<SpeedCurveCostResult>&
  GetSafeCurveCostResults() const {
    return safe_curve_cost_results_;
  }

  /**
   * @brief Get normal vt curves
   *
   * @return const std::vector<VtSampleCurve>
   */
  [[nodiscard]] const std::vector<SpeedCurveCostResult>&
  GetNormalCurveCostResults() const {
    return normal_curve_cost_results_;
  }

  /**
   * @brief Get mutable guide vt curves
   *
   * @return const std::vector<VtSampleCurve>
   */
  [[nodiscard]] std::vector<SpeedCurveCostResult>*
  GetMutableGuideCurveCostResults() {
    return &guide_curve_cost_results_;
  }

  /**
   * @brief Get mutable comfortable vt curves
   *
   * @return const std::vector<VtSampleCurve>
   */
  [[nodiscard]] std::vector<SpeedCurveCostResult>*
  GetMutableSafeCurveCostResults() {
    return &safe_curve_cost_results_;
  }

  /**
   * @brief Get mutable normal v curves
   *
   * @return const std::vector<VtSampleCurve>
   */
  std::vector<SpeedCurveCostResult>* GetMutableNormalCurveCostResults() {
    return &normal_curve_cost_results_;
  }

  /**
   * @brief Get curve count
   *
   * @return int
   */
  [[nodiscard]] std::size_t GetGuideCurveCount() const {
    return guide_curve_count_;
  }

  /**
   * @brief Get curve count
   *
   * @return int
   */
  [[nodiscard]] std::size_t GetSafeCurveCount() const {
    return safe_curve_count_;
  }

  /**
   * @brief Get curve count
   *
   * @return int
   */
  [[nodiscard]] std::size_t GetNormalCurveCount() const {
    return normal_curve_count_;
  }

  /**
   * @brief Add last frame selected curve
   *
   * @param last_curve
   * @return true
   * @return false
   */
  virtual bool AddLastCurve(const std::shared_ptr<SpeedCurve>& last_curve,
                            const SpeedCache& cache) {
    UNUSED(last_curve);
    UNUSED(cache);
    return false;
  }

  /**
   * @brief Generate accelerate fallback speed data
   *
   * @param init_point planning start point
   * @param min_v
   * @param max_v
   * @param min_acce
   * @param min_jerk
   * @param speed_data generated speed data
   * @return true successed
   * @return false failed
   */
  static bool GenerateAccelerateFallbackSpeedData(
      const common::TrajectoryPoint& init_point, double min_v, double max_v,
      double max_acce, double max_jerk, SpeedData* speed_data);

  /**
   * @brief Generate decelerate fallback speed data
   *
   * @param init_point planning start point
   * @param cache speed cache
   * @param speed_data generated speed data
   * @return true successed
   * @return false failed
   */
  static bool GenerateDecelerateFallbackSpeedData(
      const common::TrajectoryPoint& init_point, const SpeedCache& cache,
      SpeedData* speed_data);

  /**
   * @brief Generate const decel speed data
   *
   * @param speed planning start speed
   * @param stop_decel stop decel
   * @param standstill_decel standstill decel
   * @param speed_data speed data
   * @return true successed
   * @return false failed
   */
  static bool GenerateConstDecelSpeedData(double speed, double stop_decel,
                                          double standstill_decel,
                                          SpeedData* speed_data);

  /**
   * @brief Generate fallback speed data
   *
   * @param init_point planning start point
   * @param cost vt sample cost
   * @param speed_data speed data
   * @return true successed
   * @return false failed
   */
  virtual bool GenerateFallbackSpeedData(
      const common::TrajectoryPoint& init_point,
      const ReferenceLineInfo& reference_line_info, const SpeedCache& cache,
      const std::shared_ptr<SpeedEvaluator>& evaluator,
      SpeedData* speed_data) = 0;

  [[nodiscard]] double GetStopTime() const { return stop_time_; }

  void SetStopTime(double stop_time) { stop_time_ = stop_time; }

 protected:
  const VtSamplerConfig config_;
  std::vector<SpeedCurveCostResult> guide_curve_cost_results_;
  std::vector<SpeedCurveCostResult> safe_curve_cost_results_;
  std::vector<SpeedCurveCostResult> normal_curve_cost_results_;
  std::size_t guide_curve_count_ = 0;
  std::size_t safe_curve_count_ = 0;
  std::size_t normal_curve_count_ = 0;
  std::vector<std::shared_ptr<VtSampleRule>> rules_;
  double stop_time_ = -0.1;
};

}  // namespace planning
}  // namespace TL
