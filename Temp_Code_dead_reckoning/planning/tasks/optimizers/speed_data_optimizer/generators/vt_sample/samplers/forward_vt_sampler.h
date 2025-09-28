/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file forward_normal_vt_sampler.h
 **/

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/curves/cubic_vt_curve.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/curves/quadratic_vt_curve.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/curves/quartic_vt_curve.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/samplers/vt_sampler.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

/**
 * @class ForwardVtSampler
 * @brief this class is used to sample normal vt curve
 */
class ForwardVtSampler : public VtSampler {
 public:
  /**
   * @brief Construct a new normal vt sampler object
   *
   * @param config
   * @param cache
   */
  ForwardVtSampler(const VtSamplerConfig& sampler_config,
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
   * @param obstacle_cache obstacle info
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
   * @brief Generate merge stop vt sample curves
   *
   * @param init_point planning start point
   * @param reference_line_info current reference line info
   */
  void SampleMergeStopCurves(const common::TrajectoryPoint& init_point,
                             const ReferenceLineInfo& reference_line_info,
                             const SpeedCache& cache);

  /**
   * @brief 
   * 
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

  /**
   * @brief Get curve count
   *
   * @return int
   */
  [[nodiscard]] std::size_t GetMergeStopCurveCount() const {
    return merge_stop_curve_count_;
  }

  /**
   * @brief Get mutable normal v curves
   *
   * @return const std::vector<VtSampleCurve>
   */
  std::vector<SpeedCurveCostResult>* GetMutableMergeStopCurveCostResults() {
    return &merge_stop_curve_cost_results_;
  }

  const std::vector<double>& GetSampleFollowTimes() {
    return sample_follow_times_;
  }

 private:
  std::size_t CollectCurves(
      std::vector<SpeedCurveCostResult>* curve_cost_results);

  /**
   * @brief Generate cruise vt sample curves
   *
   * @param init_point planning start point
   */
  void SampleCruiseCurves(const common::TrajectoryPoint& init_point,
                          const SpeedCache& cache);

  /**
   * @brief Generate stop vt sample curves
   *
   * @param init_point planning start point
   */
  void SampleStopCurves(const common::TrajectoryPoint& init_point,
                        const SpeedCache& cache);

  /**
   * @brief Generate vt sample curves in follow mode for normal type
   *
   * @param init_point planning start point
   * @param reference_line_info current reference line info
   */
  void SampleFollowCurves(const common::TrajectoryPoint& init_point,
                          const ReferenceLineInfo& reference_line_info,
                          const SpeedCache& cache, const bool& set_extend,
                          const std::vector<double>& sample_time_extend_time);
  /**
   * @brief Check if adc need stop curve
   *
   * @return true
   * @return false
   */
  [[nodiscard]] static bool CheckNeedStopCurve(const SpeedCache& cache) {
    // if expected stop s is inf, we do not need stop curve
    const auto expected_stop_s = cache.GetBasicCache().GetExpectedStopS();
    const auto& obstacle_caches = cache.GetSafeSTObstacleCaches();
    return !std::isinf(expected_stop_s) &&
           std::none_of(obstacle_caches.begin(), obstacle_caches.end(),
                        [&](const auto& obstacle_cache) {
                          const auto* obstacle = obstacle_cache->GetObstacle();
                          return obstacle != nullptr &&
                                 !obstacle->path_st_boundary().IsEmpty() &&
                                 obstacle->path_st_boundary().min_s() <
                                     cache.GetBasicCache().GetExpectedStopS();
                        });
  }

  /**
   * @brief Check if adc need cruise curve
   *
   * @return true
   * @return false
   */
  [[nodiscard]] static bool CheckNeedCruiseCurve(const SpeedCache& cache) {
    // if there are obstacles in st graph, adc can not cruise
    // if there are gap lead / tail obstacle, adc can not cruise
    // if there are big cars, adc can not cruise
    return cache.IsEmpty() &&
           std::isinf(cache.GetBasicCache().GetExpectedStopS());
  }

  /**
   * @brief Check if adc need follow curve
   *
   * @return true
   * @return false
   */
  [[nodiscard]] static bool CheckNeedFollowCurve(const SpeedCache& cache) {
    return cache.GetFollowSTObstacleCache() != nullptr;
  }

  [[nodiscard]] bool CheckNeedUnknownCurve(const SpeedCache& cache) const {
    // check if we need unknown curves
    // if there are obstacles in st graph, we need unknown curves
    // if there are gap lead / tail obstacle, we need unknown curves
    // if there are big cars, we need unknown curves
    // if we have expected_stop_s, we need unknown curves
    return !cache.IsEmpty() || cache.GetBasicCache().GetHasPedestrian() ||
           !std::isinf(cache.GetBasicCache().GetExpectedStopS()) ||
           (quadratic_vt_curve_count_ + cubic_vt_curve_count_ +
                quartic_vt_curve_count_ ==
            0);
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
  int SampleQuadraticVTCurveForEndTimeSpeedAccel(
      const common::TrajectoryPoint& init_point, double end_t, double end_v,
      double end_a, const SpeedCurveTarget& target);

  /**
   * @brief generate quadratic vt sample curve according to curve start point, start jerks,
   * and end_jerks
   *
   * @param init_point curve start point
   * @param middle_accels curve middle accels
   * @param start_jerks curve start jerks
   * @param end_jerks curve end jerks
   * @param vt_sample_curves vt sample target
   * @return int generated curves count
   */
  int SampleQuadraticVTCurveForStartJerkEndJerk(
      const common::TrajectoryPoint& init_point,
      const std::vector<double>& middle_accels,
      const std::vector<double>& start_jerks,
      const std::vector<double>& end_jerks, const SpeedCurveTarget& target);

  /**
   * @brief generate vt sample curve according to curve start point, end_t, 
   * end_s, end_v and end_a
   *
   * @param init_point curve start point
   * @param end_t curve t length
   * @param end_s curve s length
   * @param end_v curve end speed
   * @param end_a curve end accel
   * @param vt_sample_curves generated curves
   * @return int generated curves count
   */
  int SampleQuarticVTCurveForEndTimeLengthSpeedAccel(
      const common::TrajectoryPoint& init_point, double end_t, double end_s,
      double end_v, double end_a, const SpeedCurveTarget& target);

  /**
   * @brief generate vt sample curve according to curve start point, end_t, 
   * end_v and end_a
   *
   * @param init_point curve start point
   * @param end_t curve t length
   * @param end_v curve end speed
   * @param end_a curve end accel
   * @param vt_sample_curves generated curvesd
   * @return int generated curves count
   */
  int SampleCubicVTCurveForEndTimeSpeedAccel(
      const common::TrajectoryPoint& init_point, double end_t, double end_v,
      double end_a, const SpeedCurveTarget& target);

  /**
   * @brief generate const decel stop curve
   *
   * @param init_point curve start point
   * @param end_s curve s length
   * @param vt_sample_curves generated curves
   * @return int generated curves count
   */
  int SampleConstDecelStopCurve(const common::TrajectoryPoint& init_point,
                                double end_s, double farthest_end_s,
                                const SpeedCurveTarget& target);

  /**
   * @brief Sample time for cruise from start_time
   *
   * @param start_time
   * @return std::vector<double>
   */
  std::vector<double> SampleCruiseTime(double start_time) const;

  /**
   * @brief Sample time for stop
   *
   * @param start_time
   * @return std::vector<double>
   */
  void SampleStopTime();

  /**
   * @brief Sample time for cruise from start_time
   *
   * @param start_time
   * @return std::vector<double>
   */
  void SampleCruiseTime();

  /**
   * @brief Sample time for follow
   *
   */
  void SampleFollowTime();

  /**
   * @brief Sample time for unknown
   *
   */
  void SampleUnknownTime();

  /**
   * @brief Sample jerk for curve
   *
   */
  void SampleAccel();

  /**
   * @brief Sample jerk for curve
   *
   */
  void SampleJerk();

  // quadratic_vt_curves_ used for cruise stop and unknown
  ForwardVtSamplerConfig config_;
  std::vector<std::shared_ptr<QuadraticVTCurve>> quadratic_vt_curves_;
  std::size_t start_quadratic_vt_curve_index_ = 0;
  std::size_t quadratic_vt_curve_count_ = 0;
  // cubic_vt_curves_ used for cruise
  std::vector<std::shared_ptr<CubicVTCurve>> cubic_vt_curves_;
  std::size_t start_cubic_vt_curve_index_ = 0;
  std::size_t cubic_vt_curve_count_ = 0;
  // quartic_vt_curves_ used for follow
  std::vector<std::shared_ptr<QuarticVTCurve>> quartic_vt_curves_;
  std::size_t start_quartic_vt_curve_index_ = 0;
  std::size_t quartic_vt_curve_count_ = 0;
  std::vector<double> sample_stop_times_;
  std::vector<double> sample_cruise_times_;
  std::vector<double> sample_follow_times_;
  std::vector<double> sample_unknown_times_;
  std::vector<double> sample_dense_unknown_times_;
  std::vector<double> sample_accels_;
  std::vector<double> sample_jerks_;
  std::vector<SpeedCurveCostResult> merge_stop_curve_cost_results_;
  std::size_t merge_stop_curve_count_ = 0;
  common::VehicleParam vehicle_param_;
  int max_curve_count_ = 0;
};

}  // namespace planning
}  // namespace TL
