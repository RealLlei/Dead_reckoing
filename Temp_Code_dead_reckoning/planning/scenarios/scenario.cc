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

#include "planning/scenarios/scenario.h"

#include <string>

#include "common/file/file.h"
#include "planning/common/frame.h"
#include "planning/common/planning_context.h"
#include "proto/fsm/avp_fct.pb.h"

namespace TL {
namespace planning {
namespace scenario {

using functionmanager::MachineStateType;
using TL::common::ErrorCode;
using TL::common::Status;

Scenario::Scenario(const ScenarioConfig& config, const ScenarioContext* context,
                   const std::shared_ptr<DependencyInjector>& injector)
    : config_(config),
      scenario_context_(context),
      name_(ScenarioStatus::ScenarioType_Name(config.scenario_type())),
      injector_(injector) {}

bool Scenario::LoadConfig(const std::string& config_file,
                          ScenarioConfig* config) {
  return TL::common::GetProtoFromFile(config_file, config);
}

void Scenario::Init() {
  ACHECK(!config_.stage_type().empty());

  // set scenario_type in PlanningContext
  TL::planning::ScenarioStatus* scenario = nullptr;
  scenario = injector_->planning_context()
                 ->mutable_planning_status()
                 ->mutable_scenario();
  scenario->Clear();
  scenario->set_scenario_type(scenario_type());

  stage_config_map_ =
      std::unordered_map<ScenarioStatus::StageType,
                         const ScenarioConfig::StageConfig*, std::hash<int>>();
  for (const auto& stage_config : config_.stage_config()) {
    stage_config_map_[stage_config.stage_type()] = &stage_config;
  }
  for (int i = 0; i < config_.stage_type_size(); ++i) {
    auto stage_type = config_.stage_type(i);
    ACHECK(common::util::ContainsKey(stage_config_map_, stage_type))
        << "stage type : " << ScenarioStatus::StageType_Name(stage_type)
        << " has no config";
  }
  ADEBUG << "init stage "
         << ScenarioStatus::StageType_Name(config_.stage_type(0));
  const auto init_stage_type =
      SelectInitStageType(injector_->planning_context()
                              ->mutable_planning_status()
                              ->function_manager_out());
  current_stage_ = nullptr;
  current_stage_ = CreateStage(*stage_config_map_[init_stage_type], injector_);
}

std::pair<Scenario::ScenarioState, Status> Scenario::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  CHECK_NOTNULL(injector_);
  CreateCurrentStage(scenario_type(), frame->GetIsStateChange());
  if (current_stage_ == nullptr) {
    std::string msg;
    msg = Name() + "/Current stage is a null pointer.";
    AERROR << msg;
    return std::make_pair(
        STATUS_UNKNOWN,
        Status(ErrorCode::CORE_PLANNING_STAGECREATE_ERROR, msg));
  }
  if (current_stage_->stage_type() == ScenarioStatus::NO_STAGE) {
    scenario_status_ = std::make_pair(STATUS_DONE, Status::OK());
    return scenario_status_;
  }

  auto ret = current_stage_->Process(planning_init_point, frame);
  std::string prefix;
  prefix = Name() + "/" + current_stage_->Name() + "/";
  std::string msg;
  msg = ret.second.error_message().empty()
            ? prefix + "OK"
            : prefix + ret.second.error_message();
  ADEBUG << msg;
  switch (ret.first) {
    case Stage::ERROR: {
      AERROR << "Stage '" << current_stage_->Name() << "' returns error";
      scenario_status_ =
          std::make_pair(STATUS_UNKNOWN, Status(ret.second.code(), msg));
      break;
    }
    case Stage::RUNNING: {
      scenario_status_ =
          std::make_pair(STATUS_PROCESSING, Status(ret.second.code(), msg));
      break;
    }
    case Stage::FINISHED: {
      auto next_stage = current_stage_->NextStage();
      if (next_stage != current_stage_->stage_type()) {
        AINFO << "switch stage from " << current_stage_->Name() << " to "
              << ScenarioStatus::StageType_Name(next_stage);
        if (next_stage == ScenarioStatus::NO_STAGE) {
          scenario_status_ =
              std::make_pair(STATUS_DONE, Status(ret.second.code(), msg));
          return scenario_status_;
        }
        if (stage_config_map_.find(next_stage) == stage_config_map_.end()) {
          AERROR << "Failed to find config for stage: " << next_stage;
          scenario_status_ =
              std::make_pair(STATUS_UNKNOWN, Status(ret.second.code(), msg));
          return scenario_status_;
        }
        current_stage_ = CreateStage(*stage_config_map_[next_stage], injector_);
        if (current_stage_ == nullptr) {
          AWARN << "Current stage is a null pointer.";
          return std::make_pair(STATUS_UNKNOWN, Status(ret.second.code(), msg));
        }
      }
      if (current_stage_ != nullptr &&
          current_stage_->stage_type() != ScenarioStatus::NO_STAGE) {
        scenario_status_ =
            std::make_pair(STATUS_PROCESSING, Status(ret.second.code(), msg));
      } else {
        scenario_status_ =
            std::make_pair(STATUS_DONE, Status(ret.second.code(), msg));
      }
      break;
    }
    default: {
      AWARN << "Unexpected Stage return value: " << ret.first << " "
            << ret.second.ToString();
      scenario_status_ =
          std::make_pair(STATUS_UNKNOWN, Status(ret.second.code(), msg));
    }
  }

  return scenario_status_;
}

const std::string& Scenario::Name() const {
  return name_;
}

ScenarioStatus::StageType Scenario::SelectInitStageType(
    const functionmanager::FunctionManagerOut& fct_out) const {
  if (fct_out.fsm_state() != MachineStateType::APA_TYPE &&
      fct_out.fsm_state() != MachineStateType::HDMAP_AVP_TYPE &&
      fct_out.fsm_state() != MachineStateType::HISTORY_TRACE_TYPE) {
    return config_.stage_type(0);
  }

  ScenarioStatus::StageType init_stage_type = ScenarioStatus::NO_STAGE;
  switch (fct_out.avp_fct_out().state_type()) {
    case functionmanager::AvpFctIn::APA:
    case functionmanager::AvpFctIn::RPA:
    case functionmanager::AvpFctIn::DAPA:
    case functionmanager::AvpFctIn::LAPA_MAPPING:
    case functionmanager::AvpFctIn::ISM:
    case functionmanager::AvpFctIn::NTP:
      ACHECK(common::util::ContainsKey(stage_config_map_,
                                       ScenarioStatus::VALET_PARKING_PARKING))
          << "stage type : "
          << ScenarioStatus::StageType_Name(
                 ScenarioStatus::VALET_PARKING_PARKING)
          << " has no config";
      init_stage_type = ScenarioStatus::VALET_PARKING_PARKING;
      break;
    case functionmanager::AvpFctIn::TBA:
    case functionmanager::AvpFctIn::LAPA:
    case functionmanager::AvpFctIn::LOCALIZATION:
    case functionmanager::AvpFctIn::LOCALIZATION_BACKGROUND:
      ACHECK(common::util::ContainsKey(stage_config_map_,
                                       ScenarioStatus::VALET_PARKING_CRUISE))
          << "stage type : "
          << ScenarioStatus::StageType_Name(
                 ScenarioStatus::VALET_PARKING_CRUISE)
          << " has no config";
      init_stage_type = ScenarioStatus::VALET_PARKING_CRUISE;
      break;
    default:
      ACHECK(false) << "Unexpected state_type, fct_out: "
                    << fct_out.DebugString();
  }
  AINFO << "init_stage_type: "
        << ScenarioStatus::StageType_Name(init_stage_type);
  return init_stage_type;
}

void Scenario::CreateCurrentStage(const ScenarioStatus::ScenarioType& type,
                                  const bool fsm_state_change) {
  if (type != ScenarioStatus::VALET_PARKING) {
    return;
  }
  if (scenario_status_.first != STATUS_DONE) {
    AINFO << "scenario_status_: " << scenario_status_.first
          << " is not done: " << STATUS_DONE;
    return;
  }
  if (!fsm_state_change) {
    AINFO << "fsm_state_change is false.";
    return;
  }
  const auto& fct_out = injector_->planning_context()
                            ->mutable_planning_status()
                            ->function_manager_out();
  const auto stage_type = SelectInitStageType(fct_out);
  current_stage_ = CreateStage(*stage_config_map_[stage_type], injector_);
}

}  // namespace scenario
}  // namespace planning
}  // namespace TL
