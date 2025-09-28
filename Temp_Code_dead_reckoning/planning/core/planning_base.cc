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

#include "planning/core/planning_base.h"

#include <cstdint>
#include <utility>

#include "common/file/file.h"
#include "common/time/clock.h"
#include "common/utm_projection/coordinate_convertor.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/tasks/optimizers/joint_optimizer/joint_scenarios/joint_scenario_factory.h"
#include "planning/tasks/optimizers/joint_optimizer/joint_scenarios/joint_stage_factory.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_cost_factory.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator_factory.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/speed_data_generator_factory.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_scenario_factory.h"
#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/speed_stage_factory.h"

#include "planning/tasks/task_factory.h"

#include "proto/planning/planning_internal.pb.h"
#include "proto/routing/poi.pb.h"

namespace TL {
namespace planning {

using TL::common::Status;

PlanningBase::PlanningBase(const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {
  local_view_ = std::make_shared<LocalView>();
  fault_collect_ = std::make_unique<FaultCollect>();
  event_collect_ = std::make_unique<EventCollect>();
  eth_hmi_ = std::make_unique<EthHmi>();
  last_function_manager_out_ =
      std::make_unique<functionmanager::FunctionManagerOut>();
}

PlanningBase::~PlanningBase() {}  // NOLINT

Status PlanningBase::Init(const PlanningConfig& config) {
  injector_->planning_context()->Init();
  TaskFactory::Init(config, injector_);
  SpeedEvaluatorFactory::Init();
  SpeedCostFactory::Init();
  SpeedStageFactory::Init();
  SpeedScenarioFactory::Init();
  JointScenarioFactory::Init();
  JointStageFactory::Init();
  SpeedDataGeneratorFactory::Init();
  return Status::OK();
}

Status PlanningBase::Stop() {
  return Status::OK();
}

void PlanningBase::FillPlanningPb(ADCTrajectory* const trajectory_pb) {
  if (local_view_->HasValidPerceptionObstaclesHeader()) {
    // trajectory_pb->mutable_header()->set_lidar_timestamp(
    //     local_view_->GetPredictionObstacles()->header().lidar_timestamp());
    trajectory_pb->mutable_header()->mutable_sensor_stamp()->set_camera_stamp(
        local_view_->GetPerceptionObstacles()->header().data_stamp() * 1000);
    // trajectory_pb->mutable_header()->set_radar_timestamp(
    //     local_view_->GetPredictionObstacles()->header().radar_timestamp());
  }
  if (local_view_->HasValidLocalizationHeader()) {
    trajectory_pb->mutable_debug()->mutable_localization()->CopyFrom(
        *local_view_->GetLocalization());
  }
#ifdef FOR_BAIDU_SIMULATION
  if (local_view_->HasPredictionObstacles()) {
    trajectory_pb->mutable_debug()
        ->mutable_planning_data()
        ->mutable_prediction_header()
        ->CopyFrom(local_view_->GetPredictionObstacles()->header());
  }
  if (local_view_->HasValidLocalizationHeader() &&
      !trajectory_pb->debug().planning_data().has_adc_position()) {
    trajectory_pb->mutable_debug()
        ->mutable_planning_data()
        ->mutable_adc_position()
        ->mutable_header()
        ->CopyFrom(local_view_->GetLocalization()->header());
  }

  if (local_view_->HasValidChassisHeader() &&
      !trajectory_pb->debug().planning_data().has_chassis()) {
    trajectory_pb->mutable_debug()
        ->mutable_planning_data()
        ->mutable_chassis()
        ->mutable_header()
        ->CopyFrom(local_view_->GetChassis()->header());
  }
  if (local_view_->HasValidRoutingResponseHeader()) {
    trajectory_pb->mutable_debug()
        ->mutable_planning_data()
        ->mutable_routing()
        ->mutable_header()
        ->CopyFrom(local_view_->GetRoutingResponse()->header());
  }
#endif
}

void PlanningBase::ProcessOutputData(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  eth_hmi_->ProcessEthHmi(local_view, ptr_trajectory_pb);
  if (FLAGS_enable_event_collect) {
    event_collect_->ProcessEventCollect(local_view, ptr_trajectory_pb.get());
  }
  if (FLAGS_enable_fault_collect) {
    fault_collect_->ProcessFaultCollect(local_view, ptr_trajectory_pb.get());
  }
  {
    std::lock_guard<std::mutex> lock(last_function_manager_out_mutex_);
    last_function_manager_out_->CopyFrom(
        ptr_trajectory_pb->function_manager_out());
  }
  if (FLAGS_export_local_view_to_file) {
    ExportLocalViewToFile(local_view,
                          local_view->GetADCTrajectory()->header().seq());
  }
}

void PlanningBase::ExportLocalViewToFile(
    const std::shared_ptr<LocalView>& local_view, const uint32_t sequence_num) {
  std::string output_path = "output/" + std::to_string(sequence_num) + "/";
  if (local_view->HasMapMsg()) {
    common::SetProtoToASCIIFile(*local_view->GetMapMsg(),
                                output_path + "map_msg.pb.txt");
  }

  if (local_view->HasRoutingResponse()) {
    common::SetProtoToASCIIFile(*local_view->GetRoutingResponse(),
                                output_path + "routing.pb.txt");
  }

  if (local_view->HasPerceptionObstacles()) {
    common::SetProtoToASCIIFile(*local_view->GetPerceptionObstacles(),
                                output_path + "perception.pb.txt");
  }

  if (local_view->HasLocalization()) {
    common::SetProtoToASCIIFile(*local_view->GetLocalization(),
                                output_path + "localization.pb.txt");
  }

  if (local_view->HasChassis()) {
    common::SetProtoToASCIIFile(*local_view->GetChassis(),
                                output_path + "chassis.pb.txt");
  }

  if (local_view->HasFunctionManagerIn()) {
    common::SetProtoToASCIIFile(*local_view->GetFunctionManagerIn(),
                                output_path + "fct_in.pb.txt");
  }

  if (local_view->HasFunctionManagerOut()) {
    common::SetProtoToASCIIFile(*local_view->GetFunctionManagerOut(),
                                output_path + "fct_out.pb.txt");
  }

  if (local_view->HasParkingLotOutArray()) {
    common::SetProtoToASCIIFile(*local_view->GetParkingLotOutArray(),
                                output_path + "parking_lot.pb.txt");
  }

  if (local_view->HasFreeSpaceOutArray()) {
    common::SetProtoToASCIIFile(*local_view->GetFreeSpaceOutArray(),
                                output_path + "free_space.pb.txt");
  }
}

}  // namespace planning
}  // namespace TL
