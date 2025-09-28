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

#include "planning/prediction/scenario/scenario_manager.h"
#include "common/file/log.h"
#include "planning/prediction/scenario/prioritization/obstacles_prioritizer.h"
#include "planning/prediction/scenario/right_of_way/right_of_way.h"

#include "planning/prediction/common/prediction_gflags.h"
#include "proto/fsm/nnp_fct.pb.h"

namespace TL {
namespace prediction {

using TL::common::Status;
using TL::common::adapter::AdapterConfig;

Status ScenarioManager::ProcessLocalView(
    const std::shared_ptr<TL::planning::LocalView>& local_view) {
  current_scenario_.set_type(Scenario::UNKNOWN);
  clear_container_ = false;
  auto status = Status::OK();

  if (!local_view->HasFunctionManagerIn() ||
      !local_view->HasFunctionManagerOut()) {
    AERROR << "fct in : " << local_view->HasFunctionManagerIn()
           << " fct out : " << local_view->HasFunctionManagerOut();
    return Status(common::ErrorCode::PREDICTION_INPUT_ERROR,
                  "Cannot obtain fct status from localview!");
  }

  const auto& fct_in = local_view->GetFunctionManagerIn();
  const auto& fct_out = local_view->GetFunctionManagerOut();
  functionmanager::TaPilotMode pilot_mode = functionmanager::NNP;
  if (!fct_in->has_ta_pilot_mode()) {
    AINFO << "fct in has no pilot mode";
    status = Status(common::ErrorCode::PREDICTION_INPUT_ERROR,
                    "Cannot obtain pilot mode from fct!");
  } else {
    pilot_mode = fct_in->ta_pilot_mode();
  }
  // if (local_view->GetFunctionManagerIn()->ta_pilot_mode() ==
  //         TL::functionmanager::AVP &&
  //     (!function_manager_out->has_avp_fct_out() ||
  //      !function_manager_out->avp_fct_out().has_stage_type())) {
  //   status_error = true;
  // }

  if (pilot_mode != pilot_mode_) {
    if (pilot_mode == functionmanager::AVP ||
        pilot_mode_ == functionmanager::AVP) {
      clear_container_ = true;
    }
    pilot_mode_ = pilot_mode;
  }

  if (pilot_mode_ == TL::functionmanager::NNP ||
      pilot_mode_ == TL::functionmanager::ADAS ||
      pilot_mode_ == TL::functionmanager::NCP ||
      pilot_mode_ == TL::functionmanager::NO_CONTROL) {
    current_scenario_.set_type(Scenario::CRUISE_HIGHWAY);
  } else if (pilot_mode_ == TL::functionmanager::AVP) {
    auto avp_sys_mode = fct_in->fct_avp_in().sys_mode();
    if (avp_sys_mode == functionmanager::AvpFctIn::NOSTTYPE ||
        avp_sys_mode == functionmanager::AvpFctIn::APA ||
        avp_sys_mode == functionmanager::AvpFctIn::RPA ||
        avp_sys_mode == functionmanager::AvpFctIn::DAPA) {
      current_scenario_.set_type(Scenario::PARKING);
    } else {
      current_scenario_.set_type(Scenario::CRUISE_URBAN);
    }
    // if (local_view->GetFunctionManagerOut()->avp_fct_out().stage_type() ==
    //     TL::functionmanager::AvpFctOut::CRUISING) {
    //   current_scenario_.set_type(Scenario::CRUISE_URBAN);
    // }
    // if (local_view->GetFunctionManagerOut()->avp_fct_out().stage_type() ==
    //     TL::functionmanager::AvpFctOut::PARKING) {
    //   current_scenario_.set_type(Scenario::PARKING);
    // }
  }
  double speed_mps = 0.0;
  if (local_view->HasValidChassisHeader()) {
    speed_mps = local_view->GetChassis()->speed_mps();
  }
  ScenarioManager::SetScenarioConfig(speed_mps);
  // NNP/AVP sub state
  if (fct_out->has_fsm_state() && fct_out->has_perception_sub_state()) {
    fsm_state_ = fct_out->fsm_state();
    perception_sub_state_ = fct_out->perception_sub_state();
  }

  if (fct_out->has_localization_maptype()) {
    map_type_ = fct_out->localization_maptype();
  }
  if (FLAGS_save_routing_lane_ids && local_view->HasRoutingResponse() &&
      local_view->GetRoutingResponse()->road_size() > 0) {
    routing_lane_ids_.clear();
    for (const auto& road : local_view->GetRoutingResponse()->road()) {
      for (const auto& passage : road.passage()) {
        for (const auto& segment : passage.segment()) {
          routing_lane_ids_.emplace(segment.id());
        }
      }
    }
  } else {
    ADEBUG << "Routing lane is not available at current frame";
  }

  // refresh road_edges
  road_edges_.clear();
  if (local_view->HasValidLocalMapHeader() &&
      local_view->GetLocalMap()->road_edges_size() > 0) {
    for (const auto& road_edge : local_view->GetLocalMap()->road_edges()) {
      const auto& lanepos = road_edge.lanepos();
      if (lanepos == mapping::LanePositionType::LanePositionType_EGO_LEFT ||
          lanepos == mapping::LanePositionType::LanePositionType_EGO_RIGHT ||
          lanepos == mapping::LanePositionType::LanePositionType_OTHER) {
        road_edges_.emplace_back(road_edge);
      }
    }
  }

  return status;
}

void ScenarioManager::SetScenarioConfig(double speed_mps) {
  switch (current_scenario_.type()) {
    case Scenario::CRUISE_HIGHWAY: {
      FLAGS_prediction_pedestrian_gaussian_info_sd_scale = 1.0;
      FLAGS_slow_obstacle_speed_threshold = 2.0;
      if (speed_mps < 10.0) {
        FLAGS_still_obstacle_speed_threshold = 0.3;
        FLAGS_still_obstacle_speed_threshold_upper = 0.6;
      } else if (speed_mps > 20.0) {
        FLAGS_still_obstacle_speed_threshold = 0.8;
        FLAGS_still_obstacle_speed_threshold_upper = 1.2;
      } else {
        FLAGS_still_obstacle_speed_threshold = 0.05 * speed_mps - 0.2;
        FLAGS_still_obstacle_speed_threshold_upper = 0.06 * speed_mps;
      }
      break;
    }
    case Scenario::CRUISE_URBAN: {
      FLAGS_prediction_pedestrian_gaussian_info_sd_scale = 2.0;
      FLAGS_slow_obstacle_speed_threshold = 5.0;
      FLAGS_still_obstacle_speed_threshold = 0.1;
      FLAGS_still_obstacle_speed_threshold_upper = 0.1;
      break;
    }
    case Scenario::PARKING: {
      FLAGS_prediction_pedestrian_gaussian_info_sd_scale = 2.0;
      FLAGS_slow_obstacle_speed_threshold = 5.0;
      FLAGS_still_obstacle_speed_threshold = 0.3;
      FLAGS_still_obstacle_speed_threshold_upper = 0.6;
      break;
    }
    default: {
      FLAGS_prediction_pedestrian_gaussian_info_sd_scale = 1.0;
      FLAGS_slow_obstacle_speed_threshold = 2.0;
      if (speed_mps < 10.0) {
        FLAGS_still_obstacle_speed_threshold = 0.3;
        FLAGS_still_obstacle_speed_threshold_upper = 0.6;
      } else if (speed_mps > 20.0) {
        FLAGS_still_obstacle_speed_threshold = 0.8;
        FLAGS_still_obstacle_speed_threshold_upper = 1.2;
      } else {
        FLAGS_still_obstacle_speed_threshold = 0.05 * speed_mps - 0.2;
        FLAGS_still_obstacle_speed_threshold_upper = 0.06 * speed_mps;
      }
      break;
    }
  }
}

bool ScenarioManager::PerceptionProviderChanging() const {
  return clear_container_;
}

void ScenarioManager::ProcessContainer(
    const std::shared_ptr<ContainerManager>& container_manager) {

  ObstaclesPrioritizer obstacles_prioritizer(container_manager);

  // Get obstacles_container
  auto* ptr_obstacles_container =
      container_manager->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);

  ptr_obstacles_container->BuildEgoLaneGraph();

  // Ignore some obstacles
  ObstaclesPrioritizer::AssignIgnoreLevel(road_edges_, ptr_obstacles_container,
                                          &current_scenario_);

  // Build lane graph
  ptr_obstacles_container->BuildLaneGraph();

  // Assign CautionLevel for obstacles
  obstacles_prioritizer.AssignCautionLevel(ptr_obstacles_container);

  // Analyze RightOfWay for the caution obstacles
  // RightOfWay::Analyze(container_manager.get());

  // auto environment_features =
  //     FeatureExtractor::ExtractEnvironmentFeatures(container_manager);

  // auto ptr_scenario_features = ScenarioAnalyzer::Analyze(environment_features);

  // current_scenario_ = ptr_scenario_features->scenario();

  // TODO(all) other functionalities including lane, junction filters
}

std::set<std::string> ScenarioManager::RoutingLaneIds() const {
  return routing_lane_ids_;
}

Scenario ScenarioManager::scenario() const {
  return current_scenario_;
}

}  // namespace prediction
}  // namespace TL
