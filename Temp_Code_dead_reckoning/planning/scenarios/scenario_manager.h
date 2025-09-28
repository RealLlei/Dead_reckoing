/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>
#include <unordered_map>

#include "planning/scenarios/scenario.h"

#include "planning/proto/planning_config.pb.h"

namespace TL {
namespace planning {
namespace scenario {

class ScenarioManager final {
 public:
  ScenarioManager() = delete;

  explicit ScenarioManager(const std::shared_ptr<DependencyInjector>& injector);

  bool Init(const PlanningConfig& planning_config);

  Scenario* mutable_scenario() { return current_scenario_.get(); }

  DependencyInjector* injector() { return injector_.get(); }

  void Update(const common::TrajectoryPoint& ego_point, const Frame& frame);

 private:
  void Observe(const Frame& frame);

  std::unique_ptr<Scenario> CreateScenario(
      const ScenarioStatus::ScenarioType& scenario_type);

  void RegisterScenarios();

  ScenarioStatus::ScenarioType SelectBareIntersectionScenario(
      const Frame& frame, const hdmap::PathOverlap& pnc_junction_overlap);

  //   ScenarioStatus::ScenarioType SelectPullOverScenario(const Frame& frame);

  ScenarioStatus::ScenarioType SelectPadMsgScenario();

  /**
   * @brief select ncp lane follow scenario
   * 
   * @param frame 
   * @return ScenarioStatus::ScenarioType 
   */
  ScenarioStatus::ScenarioType SelectNCPLaneFollowScenario(const Frame& frame);

  /**
   * @brief Whether can it choose the interception scenario?
   *
   * @param frame Frame
   * @return true: can select interception scenario.
   * @return false: can not select interception scenario.
   */
  static bool CanSelectInterceptionScenario(const Frame& frame);

  ScenarioStatus::ScenarioType SelectInterceptionScenario(const Frame& frame);

  ScenarioStatus::ScenarioType SelectStopSignScenario(
      const Frame& frame, const hdmap::PathOverlap& stop_sign_overlap);

  ScenarioStatus::ScenarioType SelectTrafficLightScenario(
      const Frame& frame, const hdmap::PathOverlap& traffic_light_overlap);

  ScenarioStatus::ScenarioType SelectValetParkingScenario(const Frame& frame);

  ScenarioStatus::ScenarioType SelectYieldSignScenario(
      const Frame& frame, const hdmap::PathOverlap& yield_sign_overlap);

  ScenarioStatus::ScenarioType SelectParkAndGoScenario(const Frame& frame);

  void ScenarioDispatch(const Frame& frame);
  //   ScenarioStatus::ScenarioType ScenarioDispatchLearning();
  ScenarioStatus::ScenarioType ScenarioDispatchNonLearning(const Frame& frame);

  static bool IsBareIntersectionScenario(
      const ScenarioStatus::ScenarioType& scenario_type);
  static bool IsStopSignScenario(
      const ScenarioStatus::ScenarioType& scenario_type);
  static bool IsTrafficLightScenario(
      const ScenarioStatus::ScenarioType& scenario_type);
  static bool IsYieldSignScenario(
      const ScenarioStatus::ScenarioType& scenario_type);

  void UpdatePlanningContext(const Frame& frame,
                             const ScenarioStatus::ScenarioType& scenario_type);

  void UpdatePlanningContextBareIntersectionScenario(
      const Frame& frame, const ScenarioStatus::ScenarioType& scenario_type);

  void UpdatePlanningContextEmergencyStopcenario(
      const Frame& frame, const ScenarioStatus::ScenarioType& scenario_type);

  void UpdatePlanningContextPullOverScenario(
      const Frame& frame, const ScenarioStatus::ScenarioType& scenario_type);

  void UpdatePlanningContextStopSignScenario(
      const Frame& frame, const ScenarioStatus::ScenarioType& scenario_type);

  void UpdatePlanningContextTrafficLightScenario(
      const Frame& frame, const ScenarioStatus::ScenarioType& scenario_type);

  void UpdatePlanningContextYieldSignScenario(
      const Frame& frame, const ScenarioStatus::ScenarioType& scenario_type);

  void UpdatePlanningContextValetParkingScenario(
      const Frame& frame, const ScenarioStatus::ScenarioType& scenario_type);

 private:
  std::shared_ptr<DependencyInjector> injector_;
  PlanningConfig planning_config_;
  std::unordered_map<ScenarioStatus::ScenarioType, ScenarioConfig,
                     std::hash<int>>
      config_map_;
  std::unique_ptr<Scenario> current_scenario_;
  ScenarioStatus::ScenarioType default_scenario_type_;
  ScenarioContext scenario_context_;
  std::unordered_map<ReferenceLineInfo::OverlapType, hdmap::PathOverlap,
                     std::hash<int>>
      first_encountered_overlap_map_;
};

}  // namespace scenario
}  // namespace planning
}  // namespace TL
