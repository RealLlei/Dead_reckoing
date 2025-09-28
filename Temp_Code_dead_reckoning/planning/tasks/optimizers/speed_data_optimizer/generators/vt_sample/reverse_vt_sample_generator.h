/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_generator.h
 **/

#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/reverse_gear_speed_data_generator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/samplers/reverse_vt_sampler.h"

#include "planning/proto/task_config.pb.h"
#include "proto/planning/planning_internal.pb.h"

namespace TL {
namespace planning {

/**
 * @class ReverseVtSampleGenerator
 * @brief this class is used to generate speed_data using vt sample method
 */
class ReverseVtSampleGenerator : public ReverseGearSpeedDataGenerator {
 public:
  explicit ReverseVtSampleGenerator(const SpeedDataGeneratorConfig& config);

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
                         SpeedData* speed_data);

 private:
  /**
   * @brief Create speed scenario manager
   *
   */
  void CreateScenarioManager();

  /**
   * @brief Create speed curve samples
   *
   */
  void CreateSamplers();

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
   * @brief Check if follow this obstacle
   *
   * @param obstacle_cache speed cache
   */
  static bool CheckIfFollow(const Frame& frame,
                            const ReferenceLineInfo& reference_line_info,
                            const STObstacleCache& obstacle_cache);

  /**
   * @brief Smooth cruise target speed
   *
   * @param path_data path planning result
   */
  static void CalculateFollowObstacle(
      const Frame& frame, const ReferenceLineInfo& reference_line_info,
      const common::TrajectoryPoint& init_point, SpeedCache* cache);

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
  bool CheckIfDangerous(const SpeedCache& cache,
                        const std::shared_ptr<SpeedCurve>& curve);

  /**
   * @brief Check if curve is dangerous with static obstacles
   * 
   * @param curve 
   * @return true: dangerous
   * @return false: safe 
   */
  bool CheckIfDangerousWithStaticObstacles(
      const SpeedCache& cache, const std::shared_ptr<SpeedCurve>& curve);

  /**
   * @brief Check if curve is dangerous with dynamic obstacles
   * 
   * @param curve 
   * @return true: dangerous
   * @return false: safe 
   */
  static bool CheckIfDangerousWithDynamicObstacles(
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
  std::shared_ptr<ReverseVtSampler> current_sampler_;
  // last frame selected curve
  SpeedCurveCostResult last_cost_result_;
  // current frame selected curve
  const SpeedCurveCostResult* min_cost_result_ = nullptr;
  // whether selected curve is dangerous
  bool is_dangerous_ = false;
  //
  common::VehicleParam vehicle_param_;
  // for debug
  std::shared_ptr<SpeedEvaluator> evaluator_;
};

}  // namespace planning
}  // namespace TL
