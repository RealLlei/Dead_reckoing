/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_scenario.h
 **/

#pragma once

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "planning/common/frame.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_stage.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

/**
 * @class SpeedScenarioManager
 * @brief This class defines the speed scenario manage method
 */
class SpeedScenario {
 public:
  explicit SpeedScenario(const SpeedScenarioConfig& config);
  virtual ~SpeedScenario() = default;

  /**
   * @brief Init
   * 
   */
  bool Init();

  /**
   * @brief Main process
   * 
   * @param injector 
   * @param frame current frame
   * @param reference_line_info current reference line info
   * @param init_point current planning start point
   * @param cache speed cache
   * @param generator speed data generator
   * @param speed_data output speed data
   * @return true 
   * @return false 
   */
  virtual bool Process(const std::shared_ptr<DependencyInjector>& injector,
                       Frame* frame, ReferenceLineInfo* reference_line_info,
                       const common::TrajectoryPoint& init_point,
                       SpeedCache* cache,
                       const std::shared_ptr<SpeedDataGenerator>& generator,
                       SpeedData* speed_data);

  const SpeedScenarioConfig& GetScenarioConfig() const {
    return scenario_config_;
  }

  const std::shared_ptr<SpeedStage>& GetCurrentStage() const {
    return current_stage_;
  }

 private:
  /**
   * @brief Pre process
   * 
   * @param injector 
   * @param frame 
   * @param reference_line_info 
   * @param init_point 
   * @param cache 
   * @return true 
   * @return false 
   */
  virtual bool PreProcess(const std::shared_ptr<DependencyInjector>& injector,
                          Frame* frame, ReferenceLineInfo* reference_line_info,
                          common::TrajectoryPoint* init_point,
                          SpeedCache* cache) = 0;

  /**
   * @brief Post process
   * 
   * @param frame 
   * @param reference_line_info 
   * @param init_point 
   * @param cache 
   * @param generator 
   * @param speed_data 
   * @return true 
   * @return false 
   */
  virtual bool PostProcess(Frame* frame, ReferenceLineInfo* reference_line_info,
                           const common::TrajectoryPoint& init_point,
                           SpeedCache* cache,
                           const std::shared_ptr<SpeedDataGenerator>& generator,
                           SpeedData* speed_data) = 0;

  virtual bool CheckIfKeepStandStill(
      Frame* frame, const ReferenceLineInfo& reference_line_info,
      SpeedCache* cache, common::TrajectoryPoint* init_point) = 0;

  /**
   * @brief Load accel limit table
   * 
   * @param cache 
   */
  void LoadAccelLimitTable(SpeedCache* cache);

  /**
   * @brief Load jerk limit table
   * 
   * @param cache 
   */
  void LoadJerkLimitTable(SpeedCache* cache);

  /**
   * @brief Generate standstill speed data
   *
   * @param init_point planning start point
   * @param speed_data speed data
   * @return true successed
   * @return false failed
   */
  virtual bool GenerateStandStillSpeedData(
      const common::TrajectoryPoint& init_point, SpeedData* speed_data) = 0;

  SpeedScenarioConfig scenario_config_;
  std::shared_ptr<SpeedStage> current_stage_;
  std::map<SpeedStageConfig::StageType, std::shared_ptr<SpeedStage>> stages_;
  bool is_enter_scenario_ = true;
};

}  // namespace planning
}  // namespace TL
