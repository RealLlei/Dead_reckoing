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

#include <list>
#include <memory>
#include <string>
#include <vector>

#include "common/status/status.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "map/ehr/ehr.h"
#include "map/hdmap/hdmap.h"
#include "planning/common/dependency_injector.h"
#include "planning/common/frame.h"
#include "planning/common/trajectory/publishable_trajectory.h"
#include "planning/hmi/eth_hmi.h"
#include "planning/localview/local_view.h"
#include "planning/planner/planner.h"
#include "planning/planner/planner_dispatcher.h"
#include "planning/trigger/event_collect.h"
#include "planning/trigger/fault_collect.h"

#include "common/util/base/bounded_queue.h"
#include "planning/proto/planning_config.pb.h"
#include "planning/proto/traffic_rule_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/dreamview/chart.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"
#include "proto/routing/routing.pb.h"
#include "proto/soc/chassis.pb.h"

/**
 * @namespace TL::planning
 * @brief TL::planning
 */
namespace TL {
namespace planning {

using LocalViewListPtr = std::shared_ptr<std::list<std::shared_ptr<LocalView>>>;
using PublishQueuePtr =
    std::shared_ptr<common::base::BoundedQueue<std::shared_ptr<LocalView>>>;

/**
 * @class planning
 *
 * @brief PlanningBase module main class.
 */
class PlanningBase {  // NOLINT
 public:
  PlanningBase() = delete;

  explicit PlanningBase(const std::shared_ptr<DependencyInjector>& injector);

  virtual ~PlanningBase();

  virtual TL::common::Status Init(const PlanningConfig& config);
  virtual TL::common::Status Stop() = 0;

  virtual std::string Name() const = 0;

  virtual void RunOnce(const std::shared_ptr<LocalView>& local_view) = 0;

  /**
   * @brief Break all wait to end multi-thread
   *
   */
  void BreakPublishQueue();

  /**
   * @brief Plan the trajectory given current vehicle state
   */
  virtual TL::common::Status Plan(
      double current_time_stamp,
      const ::google::protobuf::RepeatedPtrField<common::TrajectoryPoint>&
          stitching_trajectory,
      ADCTrajectory* trajectory) = 0;
  /**
   * @brief hmi/warning/trigger后处理
   * @param local_view
   * @param ptr_trajectory_pb
   */
  void ProcessOutputData(
      const std::shared_ptr<LocalView>& local_view,
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);
  virtual bool UpdateEHPData(
      const std::shared_ptr<TL::ehp::EHP>& ehp_message) = 0;

  void SetPublishQueue(const PublishQueuePtr& queue) { publish_queue_ = queue; }

  static void ExportLocalViewToFile(
      const std::shared_ptr<LocalView>& local_view, uint32_t sequence_num);

 protected:
  virtual void FillPlanningPb(ADCTrajectory* trajectory_pb);

  std::shared_ptr<LocalView> local_view_ = nullptr;

  double start_time_ = 0.0;
  size_t seq_num_ = 0;

  PlanningConfig config_;
  TrafficRuleConfigs traffic_rule_configs_;
  std::unique_ptr<Frame> frame_;
  std::unique_ptr<Planner> planner_;
  std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;
  std::shared_ptr<TL::common::VehicleState> last_vehicle_state_{nullptr};
  std::unique_ptr<PlannerDispatcher> planner_dispatcher_;
  std::shared_ptr<DependencyInjector> injector_;

  std::unique_ptr<FaultCollect> fault_collect_;
  std::unique_ptr<EventCollect> event_collect_;
  std::unique_ptr<EthHmi> eth_hmi_;
  std::unique_ptr<functionmanager::FunctionManagerOut>
      last_function_manager_out_;
  std::mutex last_function_manager_out_mutex_;
  PublishQueuePtr publish_queue_;
};

}  // namespace planning
}  // namespace TL
