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

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "planning/common/planning_gflags.h"
#include "planning/core/planning_base.h"

#include "apollo_proto/canbus/chassis.pb.h"
#include "apollo_proto/canbus/chassis_detail.pb.h"
#include "apollo_proto/control/mbd_control_debug.pb.h"
#include "apollo_proto/hmi/nns_location.pb.h"
#include "apollo_proto/hmi/nns_router.pb.h"
#include "apollo_proto/localization/localization.pb.h"
#include "apollo_proto/map/ehp.pb.h"
#include "apollo_proto/map/navigation.pb.h"
#include "apollo_proto/perception/perception_freespace.pb.h"
#include "apollo_proto/perception/perception_obstacle.pb.h"
#include "apollo_proto/perception/perception_parking_lot.pb.h"
#include "apollo_proto/planning/lanemarkers_lane_line.pb.h"
#include "apollo_proto/planning/pad_msg.pb.h"
#include "apollo_proto/planning/planning.pb.h"
#include "apollo_proto/planning/warning.pb.h"
#include "apollo_proto/planning/without_lane_follow.pb.h"
#include "apollo_proto/prediction/prediction_obstacle.pb.h"
#include "apollo_proto/routing/routing.pb.h"
#include "apollo_proto/storytelling/story.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"

using apollo::hdmap::Map;
using apollo::perception::PerceptionObstacles;

namespace TL {
namespace planning {

class PlanningComponent final
    : public apollo::cyber::Component<apollo::perception::PerceptionObstacles,
                                      apollo::canbus::Chassis> {
 public:
  PlanningComponent() = default;

  ~PlanningComponent() override;

 public:
  bool Init() override;

  bool Proc(const std::shared_ptr<apollo::perception::PerceptionObstacles>&
                perception_obstacles,
            const std::shared_ptr<apollo::canbus::Chassis>& chassis) override;

  TL::common::Status Stop();

 private:
  /**
   * @brief 拆分序列化后的地图数据为n个包，每个周期发送一包，直至发送完毕
   *
   * @param local_view 获取地图数据
   * @param ptr_trajectory_pb 输出的轨迹数据
   */
  void SplitMap(
      const std::shared_ptr<TL::planning::LocalView>& local_view,
      const std::shared_ptr<TL::planning::ADCTrajectory>& ptr_trajectory_pb);
  /**
   * @brief 拆分字符串
   *
   * @param original 要拆分的字符串
   * @return true 拆分成功返回 true
   * @return false 拆分失败返回 false
   */
  bool SplitString(const std::string& original);
  void SelfSimulatorFunc();
  bool CheckInput(
      const std::shared_ptr<TL::planning::LocalView>& local_view);
  void PlanningWriterThread();
  void SelfSimulatorThread();

 private:
  std::shared_ptr<
      apollo::cyber::Reader<apollo::perception::TrafficLightDetection>>
      traffic_light_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::planning::PadMessage>>
      pad_msg_reader_;
  std::shared_ptr<
      apollo::cyber::Reader<apollo::functionmanager::FunctionManagerIn>>
      fct_msg_reader_;
  std::shared_ptr<
      apollo::cyber::Reader<TL::functionmanager::FunctionManagerIn>>
      fct_msg_TL_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::canbus::ChassisDetail>>
      chassis_detail_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<apollo::storytelling::Stories>>
      story_telling_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<apollo::routing::RoutingRequest>>
      routing_request_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<apollo::hdmap::MapStateData>>
      map_state_data_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<apollo::perception::ParkingLotOutArray>>
      space_perception_slotsinfo_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<apollo::ehp::EHP>> ehp_reader_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Reader<apollo::routing::RoutingResponse>>
      routing_response_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::perception::FreeSpaceOutArray>>
      space_perception_freespace_reader_;
  std::shared_ptr<
      apollo::cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::hmi::NNSRouteInfo>>
      nns_route_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<apollo::hmi::NNSLocFrame>>
      nns_location_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<apollo::control::MbdDebugFromMCU>>
      control_reader_ = nullptr;

  std::shared_ptr<apollo::cyber::Writer<apollo::planning::ADCTrajectory>>
      planning_writer_;
  std::shared_ptr<
      apollo::cyber::Writer<apollo::prediction::PredictionObstacles>>
      prediction_writer_;
  std::shared_ptr<apollo::cyber::Writer<apollo::routing::RoutingResponse>>
      routing_response_writer_;
  //   std::shared_ptr<cyber::Writer<planning::DebugData>>
  //       debug_related_data_writer_;
  std::shared_ptr<
      apollo::cyber::Writer<apollo::localization::LocalizationEstimate>>
      localization_writer_;
  std::shared_ptr<apollo::cyber::Writer<apollo::canbus::Chassis>>
      chassis_writer_;
  std::shared_ptr<
      apollo::cyber::Writer<apollo::perception::PerceptionObstacles>>
      perception_writer_;
  std::shared_ptr<apollo::cyber::Writer<apollo::planning::WithoutLaneFollow>>
      without_lane_line_writer_;
  std::shared_ptr<apollo::cyber::Writer<apollo::planning::LanemarkersLaneLine>>
      lanemarkers_laneline_writer_;
  std::shared_ptr<apollo::cyber::Writer<apollo::hdmap::MapStateData>>
      map_state_data_writer_;
  std::unique_ptr<apollo::cyber::Timer> self_simulator_timer_;

  std::mutex mutex_;
  apollo::perception::TrafficLightDetection traffic_light_;
  apollo::routing::RoutingRequest routing_request_;
  apollo::routing::RoutingResponse routing_response_;
  apollo::navigation_hdmap::MapMsg map_msg_;
  apollo::perception::ParkingLotOutArray space_perception_slotsinfo_;
  apollo::perception::FreeSpaceOutArray space_perception_freespace_;
  apollo::localization::LocalizationEstimate localization_;
  apollo::planning::PadMessage pad_msg_;
  TL::functionmanager::FunctionManagerIn TL_fct_msg_;
  apollo::perception::LaneMarkers lane_marker_;
  apollo::hmi::NNSRouteInfo nns_route_;
  apollo::hmi::NNSLocFrame nns_location_;
  apollo::control::MbdDebugFromMCU control_data_;
  std::deque<apollo::hdmap::MapStateData> map_state_data_deque_;
  apollo::storytelling::Stories stories_;
  std::unique_ptr<TL::planning::PlanningBase> regular_planning_base_;
  std::unique_ptr<TL::planning::PlanningBase> guard_planning_base_;
  std::shared_ptr<TL::planning::DependencyInjector> injector_;
  TL::planning::PlanningConfig config_;
  std::future<void> writer_future_;
  std::future<void> self_simulator_future_;
  std::atomic<bool> is_self_simulator_thread_stop_{false};
  std::atomic<bool> is_pub_thread_stop_{false};
  std::atomic<bool> is_writer_running_{false};
  std::atomic<bool> is_updating_local_view_{false};
  std::vector<std::string> all_map_string_;
  std::shared_ptr<const ADCTrajectory> last_regular_adc_trajectory_;
  std::shared_ptr<TL::common::base::BoundedQueue<
      std::shared_ptr<TL::planning::LocalView>>>
      publish_queue_;
  TL::trigger::TriggerConfig trigger_config_{};
};

CYBER_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning
}  // namespace TL
