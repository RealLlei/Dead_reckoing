/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file forward_gear_scenario.h
 **/

#pragma once

#include <memory>

#include "planning/common/frame.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_scenario.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class ForwardGearScenario
 * @brief This class defines the forward gear scenario
 */
class ForwardGearScenario : public SpeedScenario {
 public:
  explicit ForwardGearScenario(const SpeedScenarioConfig& config);

 private:
  bool PreProcess(const std::shared_ptr<DependencyInjector>& injector,
                  Frame* frame, ReferenceLineInfo* reference_line_info,
                  common::TrajectoryPoint* init_point,
                  SpeedCache* cache) override;

  bool PostProcess(Frame* frame, ReferenceLineInfo* reference_line_info,
                   const common::TrajectoryPoint& init_point, SpeedCache* cache,
                   const std::shared_ptr<SpeedDataGenerator>& generator,
                   SpeedData* speed_data) override;

  bool CheckIfKeepStandStill(Frame* frame,
                             const ReferenceLineInfo& reference_line_info,
                             SpeedCache* cache,
                             common::TrajectoryPoint* init_point) override;

  bool GenerateStandStillSpeedData(const common::TrajectoryPoint& init_point,
                                   SpeedData* speed_data) override;

  /**
   * @brief 
   * 
   * @param reference_line_info 
   * @param cache 
   * @param evaluator 
   */
  void SelectFollowTime(ReferenceLineInfo* reference_line_info,
                        SpeedCache* cache,
                        const std::shared_ptr<SpeedEvaluator>& evaluator);

  void UpdateFollowCost(const SpeedCache& cache);

  /**
  * @brief 
  * 
  * @param cache 
  * @param speed_data 
  */
  void CheckIfStableFollow(SpeedCache* cache,
                           const SpeedData& speed_data) const;

  /**
   * @brief 
   * 
   * @param init_point 
   * @param cache 
   * @param obstacle_cache 
   * @return true 
   * @return false 
   */
  bool CheckIfBeginNudge(const common::TrajectoryPoint& init_point,
                         const SpeedCache& cache,
                         const NudgeObstacleCache& obstacle_cache) const;

  /**
   * @brief Make longitudinal nudge decision
   * 
   * @param reference_line_info 
   * @param init_point 
   * @param cache 
   */
  void MakeNudgeDecision(ReferenceLineInfo* reference_line_info,
                         const common::TrajectoryPoint& init_point,
                         const SpeedCache& cache);

  /**
   * @brief Mark overtake nudge obstacle
   * 
   * @param reference_line_info 
   * @param cache 
   */
  static void MarkOvertakeForNudgeObstacle(
      ReferenceLineInfo* reference_line_info, const SpeedCache& cache);

  /**
   * @brief Check if cut in begin
   * 
   * @param frame 
   * @param reference_line_info 
   * @param init_point 
   * @param cache 
   */
  void CheckIfCutInBegin(const Frame& frame,
                         const ReferenceLineInfo& reference_line_info,
                         const common::TrajectoryPoint& init_point,
                         SpeedCache* cache);

  /**
   * @brief Check if cut in finish
   * 
   * @param cache 
   * @param speed_data 
   * @return true: dangerous
   * @return false: safe 
   */
  void CheckIfCutInFinish(SpeedCache* cache, const SpeedData& speed_data);

  common::VehicleParam vehicle_param_;
};

}  // namespace planning
}  // namespace TL
