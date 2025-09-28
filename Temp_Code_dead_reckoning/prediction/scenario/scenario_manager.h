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
#include <set>
#include <string>
#include <vector>

#include "common/status/status.h"
#include "planning/localview/local_view.h"
#include "planning/prediction/scenario/analyzer/scenario_analyzer.h"
#include "planning/prediction/scenario/feature_extractor/feature_extractor.h"
#include "planning/prediction/scenario/scenario_features/cruise_scenario_features.h"
#include "proto/fsm/function_manager.pb.h"

namespace TL {
namespace prediction {

class ScenarioManager {
 public:
  /**
   * @brief Constructor
   */
  ScenarioManager() {
    current_scenario_.set_type(Scenario::UNKNOWN);
    routing_lane_ids_.clear();
  }

  /**
   * @brief Destructor
   */
  ~ScenarioManager() = default;

  /**
   * @brief Run LocalView analysis
   */
  common::Status ProcessLocalView(
      const std::shared_ptr<TL::planning::LocalView>& local_view);

  /**
   * @brief Perception provider changed
   */
  bool PerceptionProviderChanging() const;

  /**
   * @brief IsPERCEP_MAP
   */
  bool PilotType() { return map_type_ == navigation_hdmap::MapMsg::PERCEP_MAP; }

  bool VehicleReferenceFrame() {
    return fsm_state_ == functionmanager::PERCEPTION_TYPE;
  }

  /**
   * @brief Run scenario analysis
   */
  void ProcessContainer(
      const std::shared_ptr<ContainerManager>& container_manager);

  void SetScenarioConfig(double speed_mps);
  /**
   * @brief Get scenario analysis result
   */
  Scenario scenario() const;

  /**
   * @brief Get routing lane set
   */
  std::set<std::string> RoutingLaneIds() const;

 private:
  Scenario current_scenario_;
  std::vector<mapping::RoadEdge> road_edges_;
  functionmanager::TaPilotMode pilot_mode_ = functionmanager::NNP;
  navigation_hdmap::MapMsg::MapType map_type_ =
      navigation_hdmap::MapMsg::INVALID;
  functionmanager::MachineStateType fsm_state_ = functionmanager::INITIAL_TYPE;
  functionmanager::PerceptionSubState perception_sub_state_ =
      functionmanager::SUB_INITIAL_TYPE;

  bool clear_container_ = false;
  std::set<std::string> routing_lane_ids_;
};

}  // namespace prediction
}  // namespace TL
