/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_generator.h
 **/

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/forward_gear_speed_data_generator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/samplers/forward_vt_sampler.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/samplers/reverse_vt_sampler.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/samplers/vt_sampler.h"

#include "planning/proto/task_config.pb.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class ForwardVtSampleGenerator
 * @brief this class is used to generate speed_data using vt sample method
 */
class ForwardVtSampleGenerator : public ForwardGearSpeedDataGenerator {
 public:
  explicit ForwardVtSampleGenerator(const SpeedDataGeneratorConfig& config);

  /**
   * @brief use vt sample generator to generate speed data
   * 
   * @param injector 
   * @param frame 
   * @param reference_line_info 
   * @param init_point 
   * @param cache 
   * @param evaluator 
   * @param speed_data 
   * @return true 
   * @return false 
   */
  bool Process(const std::shared_ptr<DependencyInjector>& injector,
               Frame* frame, ReferenceLineInfo* reference_line_info,
               const common::TrajectoryPoint& init_point, SpeedCache* cache,
               const std::shared_ptr<SpeedEvaluator>& evaluator,
               SpeedData* speed_data) override;

  /**
   * @brief ignore back obstacle, use vt sample generator to generate speed data 
   * 
   * @param frame 
   * @param reference_line_info 
   * @param init_point 
   * @param cache 
   * @param evaluator 
   * @param speed_data 
   * @return true 
   * @return false 
   */
  bool GeneratorSpeedDataWithoutBackObstacle(
      const Frame* frame, const ReferenceLineInfo* reference_line_info,
      const common::TrajectoryPoint& init_point, const SpeedCache& cache,
      const std::shared_ptr<SpeedEvaluator>& evaluator,
      SpeedData* speed_data) override;

  /**
   * @brief use vt sample generator to generate merge stop speed data 
   * 
   * @param frame 
   * @param reference_line_info 
   * @param init_point 
   * @param cache 
   * @param evaluator 
   * @param speed_data 
   * @return true 
   * @return false 
   */
  bool GeneratorMergeStopSpeedData(
      const Frame* frame, const ReferenceLineInfo* reference_line_info,
      const common::TrajectoryPoint& init_point, const SpeedCache& cache,
      const std::shared_ptr<SpeedEvaluator>& evaluator,
      SpeedData* speed_data) override;

  /**
   * @brief use vt sample generator to generate fallback speed data
   * 
   * @param reference_line_info 
   * @param init_point 
   * @param cache 
   * @param evaluator 
   * @param speed_data 
   * @return true 
   * @return false 
   */
  bool GenerateFallbackSpeedData(
      const ReferenceLineInfo* reference_line_info,
      const common::TrajectoryPoint& init_point, const SpeedCache& cache,
      const std::shared_ptr<SpeedEvaluator>& evaluator,
      SpeedData* speed_data) override;

 private:
  /**
   * @brief use vt sample optimizer to generate speed data
   *
   * @param path_data path planning result
   * @param init_point planning start point
   * @param speed_data vt sample optimizer result
   * @return common::Status
   */
  bool Optimize(Frame* frame, ReferenceLineInfo* reference_line_info,
                const common::TrajectoryPoint& init_point, SpeedCache* cache,
                const std::shared_ptr<SpeedEvaluator>& evaluator,
                SpeedData* speed_data);

  /**
   * @brief Calculate info from last frame
   *
   * @param init_point
   */
  void CalculateInfoFromLastFrame(
      const std::shared_ptr<DependencyInjector>& injector, const Frame* frame,
      const common::TrajectoryPoint& init_point, const SpeedCache& cache);

  /**
   * @brief Convert vt sample curve to speed data
   *
   * @param curve vt sample curve
   * @param speed_data speed data
   * @return true successed
   * @return false failed
   */
  bool GenerateSpeedData(const common::TrajectoryPoint& init_point,
                         const std::shared_ptr<SpeedCurve>& curve,
                         const SpeedCache& cache, SpeedData* speed_data);

  /**
   * @brief 
   * 
   * @param cache 
   * @param evaluator 
   */
  void SelectBestNormalCurveWithMergeStopS(
      const ReferenceLineInfo* reference_line_info, const SpeedCache& cache,
      const std::shared_ptr<SpeedEvaluator>& evaluator);

  /**
   * @brief Check if follow this obstacle
   *
   * @param obstacle_cache speed cache
   */
  static bool CheckIfFollow(const ReferenceLineInfo& reference_line_info,
                            const SpeedCache& cache,
                            const STObstacleCache& obstacle_cache);

  /**
   * @brief check if follow cutout obstacle
   * 
   * @param reference_line_info 
   * @param cache 
   * @param obstacle_cache 
   * @return true 
   * @return false 
   */
  static bool CheckIfFollowCutoutObstacle(
      const ReferenceLineInfo& reference_line_info, const SpeedCache& cache,
      const STObstacleCache& obstacle_cache);

  /**
   * @brief Smooth cruise target speed
   *
   * @param path_data path planning result
   */
  static void CalculateFollowObstacle(const Frame& frame,
                                      ReferenceLineInfo* reference_line_info,
                                      const common::TrajectoryPoint& init_point,
                                      SpeedCache* cache);
  /**
   * @brief 
   * 
   * @param init_point 
   * @param cache 
   * @return true 
   * @return false 
   */
  bool CheckIfFollowDangerous(const common::TrajectoryPoint& init_point,
                              const SpeedCache& cache);
  /**
   * @brief 
   * 
   * @param init_point 
   * @param cache 
   * @return true 
   * @return false 
   */
  bool CheckIfStopDangerous(const common::TrajectoryPoint& init_point,
                            const SpeedCache& cache,
                            const ReferenceLineInfo* reference_line_info);
  /**
   * @brief Generate standstill speed data
   *
   * @param init_point planning start point
   * @param speed_data speed data
   * @return true successed
   * @return false failed
   */
  bool GenerateStandStillSpeedData(const common::TrajectoryPoint& init_point,
                                   SpeedData* speed_data);

  /**
   * @brief 
   * 
   * @param init_point 
   * @param speed_data 
   */
  void Clamp(const common::TrajectoryPoint& init_point, const SpeedCache& cache,
             SpeedData* speed_data) const;

  /**
   * @brief Calculate cost for vt sample curves
   *
   * @param vt_sample_curves vt sample curves
   * @return std::tuple<const VtSampleCurve*, double, double> selected curve
   * info, first element: curve pointer, second element: total cost, third
   * element: safety cost
   */
  void CalculateCost(const ReferenceLineInfo* reference_line_info,
                     const SpeedCache& cache,
                     const std::shared_ptr<SpeedEvaluator>& evaluator,
                     std::vector<SpeedCurveCostResult>* cost_results,
                     std::size_t start_curve_index,
                     std::size_t end_curve_index);

  /**
   * @brief Calculate cost for vt sample curves
   *
   * @param vt_sample_curves vt sample curves
   * @return std::tuple<const VtSampleCurve*, double, double> selected curve
   * info, first element: curve pointer, second element: total cost, third
   * element: safety cost
   */
  void CalculateCostForMergeStopDistance(
      const ReferenceLineInfo* reference_line_info, const SpeedCache& cache,
      const std::shared_ptr<SpeedEvaluator>& evaluator,
      std::vector<SpeedCurveCostResult>* cost_results,
      std::size_t start_curve_index, std::size_t end_curve_index);

  /**
   * @brief Calculate obstacle locations
   * 
   * @param init_point 
   */
  void CalculateObstacleSTLocations(
      const ReferenceLineInfo* reference_line_info,
      const common::TrajectoryPoint& init_point, SpeedCache* cache,
      const std::shared_ptr<SpeedEvaluator>& evaluator);

  /**
   * @brief Calculate safe curves
   * 
   * @param init_point 
   */
  void CalculateSafeCurves(const ReferenceLineInfo* reference_line_info,
                           const common::TrajectoryPoint& init_point,
                           SpeedCache* cache,
                           const std::shared_ptr<SpeedEvaluator>& evaluator);

  std::shared_ptr<SpeedCurve> SelectBestCurve(
      const ReferenceLineInfo* reference_line_info,
      const common::TrajectoryPoint& init_point, const SpeedCache& cache,
      const std::shared_ptr<SpeedEvaluator>& evaluator);

  /**
   * @brief Check if resuse stop curve
   *
   * @param cache
   * @return true: reuse last frame stop curve
   * @return false: do not reuse last frame stop curve
   */
  bool CheckIfReuseStopCurve(const SpeedCache& cache);

  /**
   * @brief Check if curve is dangerous
   * 
   * @param curve 
   * @return true: dangerous
   * @return false: safe 
   */
  bool CheckIfDangerous(const Frame& frame, const SpeedCache& cache,
                        const std::shared_ptr<SpeedCurve>& curve);

  /**
   * @brief Check if curve is dangerous with static obstacles
   * 
   * @param curve 
   * @return true: dangerous
   * @return false: safe 
   */
  bool CheckIfDangerousWithStaticObstacles(
      const Frame& frame, const SpeedCache& cache,
      const std::shared_ptr<SpeedCurve>& curve);

  /**
   * @brief Check if curve is dangerous with dynamic obstacles
   * 
   * @param curve 
   * @return true: dangerous
   * @return false: safe 
   */
  bool CheckIfDangerousWithDynamicObstacles(
      const SpeedCache& cache, const std::shared_ptr<SpeedCurve>& curve);

  /**
   * @brief print curve info
   *
   * @param curve
   * @return std::string curve info string
   */
  static std::string PrintCurveInfo(
      const ReferenceLineInfo* reference_line_info, const SpeedCache& cache,
      const SpeedCurveCostResult& cost_result);

  /**
   * @brief record debug info
   *
   * @param path_data
   */
  void RecordDebug(ReferenceLineInfo* reference_line_info,
                   const SpeedCache& cache);

  // optimizer config
  VtSampleGeneratorConfig config_;
  // sampler
  std::shared_ptr<ForwardVtSampler> current_sampler_;
  // last frame selected curve
  SpeedCurveCostResult last_cost_result_;
  // current frame selected curve
  const SpeedCurveCostResult* min_cost_result_ = nullptr;
  // whether follow scenario is dangerous
  bool is_follow_dangerous_ = false;
  bool is_stop_dangerous_ = false;
  // whether selected curve is dangerous
  bool is_dangerous_ = false;
  //
  common::VehicleParam vehicle_param_;
  // for debug
  std::shared_ptr<SpeedEvaluator> evaluator_;
  std::vector<std::vector<double>> extend_sample_times_{2,
                                                        std::vector<double>{}};
};

}  // namespace planning
}  // namespace TL
