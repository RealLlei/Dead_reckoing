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

/**
 * @file
 **/

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "common/status/status.h"
#include "planning/common/frame.h"
#include "planning/scenarios/stage.h"

#include "planning/proto/planning_config.pb.h"

namespace TL {
namespace planning {
namespace scenario {

struct ScenarioContext {};

class Scenario {
 public:
  enum ScenarioState {
    STATUS_UNKNOWN = 0,
    STATUS_PROCESSING = 1,
    STATUS_DONE = 2,
  };

  Scenario(const ScenarioConfig& config, const ScenarioContext* context,
           const std::shared_ptr<DependencyInjector>& injector);

  static bool LoadConfig(const std::string& config_file,
                         ScenarioConfig* config);

  virtual ~Scenario() = default;

  ScenarioStatus::ScenarioType scenario_type() const {
    return config_.scenario_type();
  }

  /**
   * Each scenario should define its own stages object's creation
   * scenario will call stage's Stage::Process function following a configured
   * order, The return value of Stage::Process function determines the
   * transition from one stage to another.
   */
  virtual std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config,
      const std::shared_ptr<DependencyInjector>& injector) = 0;

  // Each scenario should define its own transfer condition, i.e., when it
  // should allow to transfer from other scenario to itself.
  virtual bool IsTransferable(const Scenario& other_scenario,
                              const Frame& frame) {
    UNUSED(other_scenario);
    UNUSED(frame);
    return true;
  }

  virtual std::pair<ScenarioState, common::Status> Process(
      const common::TrajectoryPoint& planning_init_point, Frame* frame);

  const ScenarioState& GetStatus() const { return scenario_status_.first; }

  ScenarioStatus::StageType GetStage() const {
    return current_stage_ ? current_stage_->stage_type()
                          : ScenarioStatus::NO_STAGE;
  }

  virtual void Init();

  const std::string& Name() const;

  const std::string& GetMsg() const { return msg_; }

  ScenarioStatus::StageType SelectInitStageType(
      const functionmanager::FunctionManagerOut& fct_out) const;

  void CreateCurrentStage(const ScenarioStatus::ScenarioType& type,
                          bool fsm_state_change);

 protected:
  std::pair<ScenarioState, common::Status> scenario_status_ =
      std::make_pair(STATUS_UNKNOWN, common::Status::OK());
  std::unique_ptr<Stage> current_stage_;
  ScenarioConfig config_;
  std::unordered_map<ScenarioStatus::StageType,
                     const ScenarioConfig::StageConfig*, std::hash<int>>
      stage_config_map_;
  const ScenarioContext* scenario_context_ = nullptr;
  std::string name_;
  std::string msg_;  // debug msg
  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace scenario
}  // namespace planning
}  // namespace TL
