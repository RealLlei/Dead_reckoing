/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_scenario_manager.h
 **/

#pragma once

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "planning/common/frame.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_scenario.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class SpeedScenarioManager
 * @brief This class defines the speed scenario manage method
 */
class SpeedScenarioManager {
 public:
  explicit SpeedScenarioManager(const SpeedScenarioManagerConfig& config);

  /**
   * @brief Update
   * 
   * @param injector injector
   * @param frame current frame  
   * @param reference_line_info current reference line info
   * @param cache speed cache  
   */
  bool Update(const std::shared_ptr<DependencyInjector>& injector, Frame* frame,
              ReferenceLineInfo* reference_line_info, SpeedCache* cache);

  /**
   * @brief Get current scenario type
   * 
   * @return SpeedScenarioManagerConfig::ScenarioType 
   */
  [[nodiscard]] SpeedScenarioConfig::ScenarioType GetCurrentScenarioType()
      const {
    return current_scenario_type_;
  }

  /**
   * @brief Get current scenario type
   * 
   * @return SpeedScenarioManagerConfig::ScenarioType 
   */
  [[nodiscard]] const std::shared_ptr<SpeedScenario>& GetCurrentScenario()
      const {
    return current_scenario_;
  }

  /**
   * @brief Get current scenario type
   * 
   * @return SpeedScenarioManagerConfig::ScenarioType 
   */
  [[nodiscard]] std::shared_ptr<SpeedScenario> GetMutableCurrentScenario() {
    return current_scenario_;
  }

 private:
  /**
   * @brief Select scenario
   * 
   * @param frame current frame 
   * @param reference_line_info current reference line info
   * @param cache speed cache  
   */
  SpeedScenarioConfig::ScenarioType SelectScenario(
      Frame* frame, ReferenceLineInfo* reference_line_info,
      const SpeedCache& cache);

  /**
   * @brief Check if lane change return
   * 
   * @param injector 
   * @param frame current frame
   * @param reference_line_info current reference line info
   * @return true 
   * @return false 
   */
  static bool CheckIfLaneChangeReturn(
      const std::shared_ptr<DependencyInjector>& injector, const Frame& frame,
      const ReferenceLineInfo& reference_line_info);

  hdmap::LaneMergeInfo lane_merge_info_;
  SpeedScenarioManagerConfig config_;
  SpeedScenarioConfig::ScenarioType current_scenario_type_ =
      SpeedScenarioConfig::LANE_KEEP_SCENARIO;
  std::shared_ptr<SpeedScenario> current_scenario_;
  std::map<SpeedScenarioConfig::ScenarioType, std::shared_ptr<SpeedScenario>>
      scenarios_;
};

}  // namespace planning
}  // namespace TL
