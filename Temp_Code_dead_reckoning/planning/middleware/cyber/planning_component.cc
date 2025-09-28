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
#include "planning/middleware/cyber/planning_component.h"

#include <list>
#include <memory>
#include <utility>
// NOLINTBEGIN
#include "boost/algorithm/string/split.hpp"
#include "common/adapters/adapter_gflags.h"
#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/file/log.h"
#include "common/util/convert_tool.h"
#include "common/util/message_util.h"
#include "common/util/util.h"
#include "map/ehr/amap/amap_ehr_impl.h"
#include "map/hdmap/hdmap_util.h"
#include "map/hdmap/path.h"
#include "planning/common/history.h"
#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/core/on_lane_planning.h"
#include "planning/core/safety_guard_planning.h"
#include "apollo_proto/fsm/function_manager.pb.h"
#include "apollo_proto/hmi/nns_location.pb.h"
#include "apollo_proto/localization/localization.pb.h"
#include "apollo_proto/perception/perception_parking_lot.pb.h"
#include "proto/perception/transport_element.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

using TL::hdmap::HDMapUtil;

PlanningComponent::~PlanningComponent() {
  Stop();
}

void SignalHandle(const char* data, int size) {
  std::ofstream fs(FLAGS_log_dir + "/" + "core_dump.log", std::ios::app);
  std::string str = std::string(data, size);
  fs << str;
  fs.close();
  AERROR << str;
}

bool PlanningComponent::Init() {
  google::InstallFailureSignalHandler();
  google::InstallFailureWriter(&SignalHandle);
  injector_ = std::make_shared<TL::planning::DependencyInjector>();
  regular_planning_base_ =
      std::make_unique<TL::planning::OnLanePlanning>(injector_);
  guard_planning_base_ =
      std::make_unique<TL::planning::SafetyGuardPlanning>(injector_);
  ACHECK(ComponentBase::GetProtoConfig(&config_))
      << "failed to load planning config file "
      << ComponentBase::ConfigFilePath();

  publish_queue_ = std::make_shared<TL::common::base::BoundedQueue<
      std::shared_ptr<TL::planning::LocalView>>>();
  publish_queue_->Init(3,
                       new TL::common::base::TimeoutBlockWaitStrategy(1000));
  regular_planning_base_->Init(config_);
  regular_planning_base_->SetPublishQueue(publish_queue_);

  guard_planning_base_->Init(config_);
  guard_planning_base_->SetPublishQueue(publish_queue_);

  writer_future_ = std::async(&PlanningComponent::PlanningWriterThread, this);
  // set default pilot mode to nnp
  TL_fct_msg_.set_ta_pilot_mode(functionmanager::NNP);
  traffic_light_reader_ =
      node_->CreateReader<apollo::perception::TrafficLightDetection>(
          config_.topic_config().traffic_light_detection_topic(),
          [this](
              const std::shared_ptr<apollo::perception::TrafficLightDetection>&
                  traffic_light) {
            ADEBUG
                << "Received traffic light data: run traffic light callback.";
            std::lock_guard<std::mutex> lock(mutex_);
            traffic_light_.CopyFrom(*traffic_light);
          });

  pad_msg_reader_ = node_->CreateReader<apollo::planning::PadMessage>(
      config_.topic_config().planning_pad_topic(),
      [this](const std::shared_ptr<apollo::planning::PadMessage>& pad_msg) {
        ADEBUG << "Received pad data: run pad callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        pad_msg_.CopyFrom(*pad_msg);
      });

  fct_msg_reader_ =
      node_->CreateReader<apollo::functionmanager::FunctionManagerIn>(
          config_.topic_config().planning_fct_topic(),
          [this](
              const std::shared_ptr<apollo::functionmanager::FunctionManagerIn>&
                  fct_msg) {
            ADEBUG << "Received pad data: run pad callback.";
            std::lock_guard<std::mutex> lock(mutex_);
            TL_fct_msg_.CopyFrom(*ConvertTool::ForwardFunctionManagerIn(
                std::make_shared<ApolloFunctionManagerIn>(*fct_msg)));
          });
  fct_msg_TL_reader_ =
      node_->CreateReader<TL::functionmanager::FunctionManagerIn>(
          "/TL/planning/fct",
          [this](
              const std::shared_ptr<TL::functionmanager::FunctionManagerIn>&
                  fct_msg) {
            ADEBUG << "Received pad data: run pad callback.";
            std::lock_guard<std::mutex> lock(mutex_);
            TL_fct_msg_.CopyFrom(*fct_msg);
            // fct_msg_.Clear();
          });

  story_telling_reader_ = node_->CreateReader<apollo::storytelling::Stories>(
      config_.topic_config().story_telling_topic(),
      [this](const std::shared_ptr<apollo::storytelling::Stories>& stories) {
        ADEBUG << "Received story_telling data: run story_telling callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        stories_.CopyFrom(*stories);
      });

  routing_request_reader_ = node_->CreateReader<
      apollo::routing::RoutingRequest>(
      FLAGS_routing_request_topic,
      [this](const std::shared_ptr<apollo::routing::RoutingRequest>&
                 routing_request) {
        ADEBUG
            << "Received routing_request data: run routing_request callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        routing_request_.CopyFrom(*routing_request);
      });
  if (FLAGS_using_record_ehp_data) {
    map_state_data_reader_ = node_->CreateReader<apollo::hdmap::MapStateData>(
        FLAGS_map_state_data_topic,
        [this](const std::shared_ptr<apollo::hdmap::MapStateData>&
                   map_state_data) {
          ADEBUG << "Received map state data: run map_state callback.";
          std::lock_guard<std::mutex> lock(mutex_);
          map_state_data_deque_.emplace_front(*map_state_data);
        });
  }

  routing_response_reader_ =
      node_->CreateReader<apollo::routing::RoutingResponse>(
          FLAGS_routing_response_topic,
          [this](const std::shared_ptr<apollo::routing::RoutingResponse>&
                     routing_response) {
            ADEBUG << "Received routing_response data,header: "
                   << routing_response->header().ShortDebugString();
            std::lock_guard<std::mutex> lock(mutex_);
            routing_response_.CopyFrom(*routing_response);
          });
  space_perception_slotsinfo_reader_ =
      node_->CreateReader<apollo::perception::ParkingLotOutArray>(
          "/apollo/space/parkinglot",
          [this](const std::shared_ptr<apollo::perception::ParkingLotOutArray>&
                     space_perception_slotsinfo) {
            ADEBUG << "Received perception parking_space data: run perfect "
                      "parking_space callback.";
            std::lock_guard<std::mutex> lock(mutex_);
            space_perception_slotsinfo_.CopyFrom(*space_perception_slotsinfo);
          });

  space_perception_freespace_reader_ =
      node_->CreateReader<apollo::perception::FreeSpaceOutArray>(
          "/apollo/space/freespace",
          [this](const std::shared_ptr<apollo::perception::FreeSpaceOutArray>&
                     space_perception_freespace) {
            ADEBUG << "Received perception free space data";
            std::lock_guard<std::mutex> lock(mutex_);
            space_perception_freespace_.CopyFrom(*space_perception_freespace);
          });
  localization_reader_ =
      node_->CreateReader<apollo::localization::LocalizationEstimate>(
          fLS::FLAGS_localization_topic,
          [this](
              const std::shared_ptr<apollo::localization::LocalizationEstimate>&
                  localization) {
            ADEBUG << "Received perception free space data";
            std::lock_guard<std::mutex> lock(mutex_);
            localization_.CopyFrom(*localization);
          });

  apollo::cyber::ReaderConfig ehp_reader_config;
  ehp_reader_config.channel_name = FLAGS_ehp_topic;
  ehp_reader_config.pending_queue_size = 100;

  ehp_reader_ = node_->CreateReader<apollo::ehp::EHP>(
      ehp_reader_config, [this](const std::shared_ptr<apollo::ehp::EHP>& ehp) {
        ADEBUG << "Received ehp data: run ehp data callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        regular_planning_base_->UpdateEHPData(ConvertTool::ForwardEhp(ehp));
      });

  nns_route_reader_ = node_->CreateReader<apollo::hmi::NNSRouteInfo>(
      FLAGS_nns_route_topic,
      [this](const std::shared_ptr<apollo::hmi::NNSRouteInfo>& nns_route) {
        ADEBUG << "Received nns route data";
        std::lock_guard<std::mutex> lock(mutex_);
        nns_route_.CopyFrom(*nns_route);
      });

  nns_location_reader_ = node_->CreateReader<apollo::hmi::NNSLocFrame>(
      FLAGS_nns_location_topic,
      [this](const std::shared_ptr<apollo::hmi::NNSLocFrame>& nns_location) {
        ADEBUG << "Received nns location data";
        std::lock_guard<std::mutex> lock(mutex_);
        nns_location_.CopyFrom(*nns_location);
      });

  planning_writer_ = node_->CreateWriter<apollo::planning::ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());

  routing_response_writer_ =
      node_->CreateWriter<apollo::routing::RoutingResponse>(
          FLAGS_routing_response_topic);
  // debug_related_data_writer_ =
  //     node_->CreateWriter<planning::DebugData>(FLAGS_debug_related_data_topic);
  prediction_writer_ =
      node_->CreateWriter<apollo::prediction::PredictionObstacles>(
          FLAGS_prediction_topic);
  without_lane_line_writer_ =
      node_->CreateWriter<apollo::planning::WithoutLaneFollow>(
          FLAGS_without_lane_line_topic);
  lanemarkers_laneline_writer_ =
      node_->CreateWriter<apollo::planning::LanemarkersLaneLine>(
          FLAGS_lanemarkers_laneline_topic);
  map_state_data_writer_ = node_->CreateWriter<apollo::hdmap::MapStateData>(
      FLAGS_map_state_data_topic);
  bool load_trigger_config = TL::common::GetProtoFromFile(
      FLAGS_default_trigger_config_file, &trigger_config_);
  if (!load_trigger_config) {
    AERROR << "Failed to load obs follow time config file ";
  }
  if (FLAGS_enable_planning_self_simulator) {
    localization_writer_ =
        node_->CreateWriter<apollo::localization::LocalizationEstimate>(
            FLAGS_localization_topic);
    chassis_writer_ =
        node_->CreateWriter<apollo::canbus::Chassis>(FLAGS_chassis_topic);

    perception_writer_ =
        node_->CreateWriter<apollo::perception::PerceptionObstacles>(
            FLAGS_perception_obstacle_topic);
    self_simulator_future_ =
        std::async(&PlanningComponent::SelfSimulatorThread, this);
  }

  TL::common::util::FillHeader("planning", &TL_fct_msg_);
  return true;
}

TL::common::Status PlanningComponent::Stop() {
  publish_queue_->BreakAllWait();
  is_pub_thread_stop_ = true;
  writer_future_.get();
  if (FLAGS_enable_planning_self_simulator) {
    is_self_simulator_thread_stop_ = true;
    self_simulator_future_.get();
  }
  return TL::common::Status(TL::common::ErrorCode::OK);
}

void PlanningComponent::SelfSimulatorFunc() {
  if (is_pub_thread_stop_) {
    return;
  }
  auto local_view = std::make_shared<LocalView>();
  local_view->SetFunctionManagerInPtr(
      std::make_shared<TLFunctionManagerIn>(TL_fct_msg_));
  // local_view->SetPadMessagePtr(std::make_shared<PadMessage>(pad_msg_));
  // local_view->SetLaneMarkersPtr(std::make_shared<LaneMarkers>());
  local_view->SetParkingLotOutArrayPtr(ConvertTool::ForwardParkingLotOutArray(
      std::make_shared<ApolloParkingLotOutArray>()));
  if (FLAGS_use_dv_routing) {
    local_view->SetRoutingRequestPtr(ConvertTool::ForwardRoutingRequest(
        std::make_shared<ApolloRoutingRequest>(routing_request_)));
    ADEBUG << " Dv_routing_request_ " << routing_request_.DebugString();
  }

  // if (FLAGS_use_dv_traffic_light) {
  //   local_view->SetTrafficLightDetectionPtr(
  //       ConvertTool::ForwardTrafficLightDetection(
  //           std::make_shared<ApolloTrafficLightDetection>(traffic_light_)));
  //   ADEBUG << " Dv_traffic_light_ " << traffic_light_.DebugString();
  // }

  regular_planning_base_->RunOnce(local_view);
  ADEBUG << "update local view " << local_view;
}

bool PlanningComponent::Proc(
    const std::shared_ptr<apollo::perception::PerceptionObstacles>&
        perception_obstacles,
    const std::shared_ptr<apollo::canbus::Chassis>& chassis) {
  if (is_pub_thread_stop_) {
    return true;
  }
  if (FLAGS_enable_planning_self_simulator) {
    return true;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (trigger_config_.has_nnp_config()) {
    TL_fct_msg_.mutable_trigger_config()->CopyFrom(trigger_config_);
    trigger_config_.Clear();
  } else {
    TL_fct_msg_.clear_trigger_config();
  }
  if (!TL_fct_msg_.has_ta_pilot_mode()) {
    AERROR << "fct_msg has no ta_pilot_mode: " << TL_fct_msg_.DebugString();
  }
  ADEBUG << "run planning proc , obstacle header : "
         << perception_obstacles->header().ShortDebugString()
         << " \tchassis header : " << chassis->header().ShortDebugString()
         << " \tlocalization header : "
         << localization_.header().ShortDebugString();
  auto local_view = std::make_shared<LocalView>();

  ACHECK(perception_obstacles != nullptr);

  // process fused input data
  if (perception_obstacles->has_lane_marker()) {
    auto TL_lane_marker =
        ConvertTool::ForwardLaneMarkers(std::make_shared<ApolloLaneMarkers>(
            perception_obstacles->lane_marker()));
    local_view->SetLaneMarkersPtr(TL_lane_marker);
    local_view->SetLaneMarkersMinieyePtr(TL_lane_marker);
    perception_obstacles->clear_lane_marker();
  } else {
    local_view->SetLaneMarkersPtr(
        std::make_shared<TL::perception::LaneMarkers>());
    local_view->SetLaneMarkersMinieyePtr(
        std::make_shared<TL::perception::LaneMarkers>());
  }
  TLPerceptionObstaclesPtr lhs =
      ConvertTool::ForwardPerceptionObstacles(perception_obstacles);
  local_view->SetPerceptionObstaclesPtr(lhs);
  local_view->SetPerceptionObstaclesMinieyePtr(lhs);

  if (!chassis->has_warning_switch_from_cdcs()) {
    chassis->mutable_warning_switch_from_cdcs()->set_dow_on_off_set(
        apollo::canbus::WarningSwitch::ON);
    chassis->mutable_warning_switch_from_cdcs()->set_lca_on_off_set(
        apollo::canbus::WarningSwitch::ON);
    chassis->mutable_warning_switch_from_cdcs()->set_fcta_on_off_set(
        apollo::canbus::WarningSwitch::ON);
    chassis->mutable_warning_switch_from_cdcs()->set_rcta_on_off_set(
        apollo::canbus::WarningSwitch::ON);
    chassis->mutable_warning_switch_from_cdcs()->set_rcw_on_off_set(
        apollo::canbus::WarningSwitch::ON);
  }
  if (!chassis->has_warning_switch_mem()) {
    chassis->mutable_warning_switch_mem()->set_dow_on_off_set_mem(
        apollo::canbus::WarningSwitchMemory::ON);
    chassis->mutable_warning_switch_mem()->set_lca_on_off_set_mem(
        apollo::canbus::WarningSwitchMemory::ON);
    chassis->mutable_warning_switch_mem()->set_fcta_on_off_set_mem(
        apollo::canbus::WarningSwitchMemory::ON);
    chassis->mutable_warning_switch_mem()->set_rcta_on_off_set_mem(
        apollo::canbus::WarningSwitchMemory::ON);
    chassis->mutable_warning_switch_mem()->set_rcw_on_off_set_mem(
        apollo::canbus::WarningSwitchMemory::ON);
  }
  if (!chassis->has_vehicle_cfg()) {
    chassis->mutable_vehicle_cfg()->set_dow(1);
    chassis->mutable_vehicle_cfg()->set_lca(1);
    chassis->mutable_vehicle_cfg()->set_fcta(1);
    chassis->mutable_vehicle_cfg()->set_rcta(1);
    chassis->mutable_vehicle_cfg()->set_rcw(1);
  }

  local_view->SetChassisPtr(ConvertTool::ForwardChassis(chassis));
  // 数据里已经有using utm zone并含有原始的utm1&2的信息，说明是回放的数据，
  // 这时候下游需要重新根据经纬度重新选择using_utm_zone的逻辑。而下游看到有
  // using_utm_zone是直接跳过选择逻辑不换utm zone的
  if (localization_.pose().has_using_utm_zone() &&
      localization_.pose().has_utm_zone_01() &&
      localization_.pose().has_utm_zone_02() &&
      localization_.pose().has_gcj02()) {
    auto fixed_localization =
        std::make_shared<apollo::localization::LocalizationEstimate>(
            localization_);
    fixed_localization->mutable_pose()->clear_using_utm_zone();
    local_view->SetLocalizationPtr(
        ConvertTool::ForwardLocalization(fixed_localization));
  } else if (TL_fct_msg_.ta_pilot_mode() == functionmanager::AVP) {
    auto fixed_localization =
        std::make_shared<apollo::localization::LocalizationEstimate>(
            localization_);
    fixed_localization->mutable_pose()->set_using_utm_zone(
        FLAGS_local_utm_zone_id);
    local_view->SetLocalizationPtr(
        ConvertTool::ForwardLocalization(fixed_localization));
  } else {
    local_view->SetLocalizationPtr(ConvertTool::ForwardLocalization(
        std::make_shared<apollo::localization::LocalizationEstimate>(
            localization_)));
  }

  if (traffic_light_.has_header()) {
    local_view->SetTransportElementPtr(ConvertTool::ForwardTransport(
        std::make_shared<ApolloTrafficLightDetection>(traffic_light_)));
  }
  local_view->SetPadMessagePtr(ConvertTool::ForwardPadMessage(
      std::make_shared<apollo::planning::PadMessage>(pad_msg_)));
  // local_view->SetStoriesPtr(std::make_shared<Stories>(stories_));
  local_view->SetParkingLotOutArrayPtr(ConvertTool::ForwardParkingLotOutArray(
      std::make_shared<ApolloParkingLotOutArray>(space_perception_slotsinfo_)));
  local_view->SetFreeSpaceOutArrayPtr(ConvertTool::ForwardFreeSpaceOutArray(
      std::make_shared<ApolloFreeSpaceOutArray>(space_perception_freespace_)));
  local_view->SetFunctionManagerInPtr(
      std::make_shared<TLFunctionManagerIn>(TL_fct_msg_));
  local_view->SetNNSLocFramePtr(ConvertTool::ForwardNNSLocFrame(
      std::make_shared<apollo::hmi::NNSLocFrame>(nns_location_)));
  local_view->SetNNSRouteInfoPtr(ConvertTool::ForwardNNSRouteInfo(
      std::make_shared<apollo::hmi::NNSRouteInfo>(nns_route_)));
  const auto& header = routing_request_.header();
  if (header.has_sequence_num() && header.has_timestamp_sec()) {
    local_view->SetRoutingRequestPtr(ConvertTool::ForwardRoutingRequest(
        std::make_shared<ApolloRoutingRequest>(routing_request_)));
  }
  // if (FLAGS_using_record_ehp_data && map_state_data_deque_.size() > 0) {
  //   local_view->SetMapStateDataPtr(
  //       std::make_shared<apollo::hdmap::MapStateData>(
  //           map_state_data_deque_.back()));
  //   map_state_data_deque_.pop_back();
  // }
  if (FLAGS_is_record_replay) {
    if (routing_response_.road_size() > 0) {
      if (routing_response_.measurement().info_size() > 0) {
        auto map_msg = std::make_shared<navigation_hdmap::MapMsg>();
        map_msg->ParseFromString(routing_response_.measurement().info(0));
        if (map_msg->has_hdmap() && map_msg->hdmap().lane_size() > 0) {
          local_view->SetMapMsgPtr(map_msg);
        }
        routing_response_.mutable_measurement()->clear_info();
      }
      local_view->SetRoutingResponsePtr(ConvertTool::ForwardRoutingResponse(
          std::make_shared<ApolloRoutingResponse>(routing_response_)));
    }
  }

  if (!CheckInput(local_view)) {
    AERROR << "Input check failed";
    return false;
  }

  guard_planning_base_->RunOnce(local_view);
  regular_planning_base_->RunOnce(local_view);
  return true;
}

void PlanningComponent::PlanningWriterThread() {
  pthread_setname_np(pthread_self(), "PlanningWriter");
  TL::common::Header last_prediction_header;
  // TL::common::Header last_routing_header;
  TL::common::Header last_without_lane_header;
  // // sleep for init
  std::shared_ptr<LocalView> last_local_view;
  while (!is_pub_thread_stop_) {
    std::shared_ptr<LocalView> retrived_local_view(nullptr);
    if (!publish_queue_->WaitDequeue(&retrived_local_view) ||
        retrived_local_view == nullptr) {
      continue;
    }

    // 更新last_regular_adc_trajectory_
    if (retrived_local_view->HasADCTrajectory()) {
      last_regular_adc_trajectory_ = retrived_local_view->GetADCTrajectory();
    }

    // 如果 last_local_view == retrived_local_view, 表示是已经发过的轨迹
    if (last_local_view == retrived_local_view) {
      continue;
    }
    last_local_view = retrived_local_view;

    // send guard trajectory
    if (retrived_local_view->HasADCTrajectoryGuard()) {
      auto guard_adctrajectory = *retrived_local_view->GetADCTrajectoryGuard();
      if (guard_adctrajectory.has_header() &&
          !guard_adctrajectory.trajectory_point().empty()) {
        guard_adctrajectory.set_gear(last_regular_adc_trajectory_->gear());
        guard_adctrajectory.mutable_debug()->mutable_planning_data()->CopyFrom(
            last_regular_adc_trajectory_->debug().planning_data());
        guard_adctrajectory.mutable_header()->mutable_status()->CopyFrom(
            last_regular_adc_trajectory_->header().status());
        planning_writer_->Write(ConvertTool::ReverseADCTrajectory(
            std::make_shared<TLADCTrajectory>(guard_adctrajectory)));
        if (FLAGS_export_local_view_to_file) {
          guard_planning_base_->ExportLocalViewToFile(
              retrived_local_view, guard_adctrajectory.header().seq());
        }
      }
      continue;
    }

    // 为了避免copy adc_trajectory, 且为了避免多线程数据竞争无法在 local_view 加 mutable 接口
    // 此为设计失误，不应该把所有东西都塞在 trajectory 里，造成很多数据处理时发生耦合
    auto ptr_trajectory_pb = std::const_pointer_cast<ADCTrajectory>(
        retrived_local_view->GetADCTrajectory());
    if (ptr_trajectory_pb == nullptr) {
      continue;
    }

    regular_planning_base_->ProcessOutputData(retrived_local_view,
                                              ptr_trajectory_pb);
    // send routing
    if (((!FLAGS_is_record_replay) &&
         (retrived_local_view->HasValidRoutingResponseHeader())) ||
        (FLAGS_enable_planning_self_simulator)) {
      auto current_routing = std::make_shared<TLRoutingResponse>();
      current_routing->CopyFrom(*retrived_local_view->GetRoutingResponse());
      auto apollo_routing =
          ConvertTool::ReverseRoutingResponse(current_routing);
      if (FLAGS_export_map_in_planning && retrived_local_view->HasMapMsg()) {
        ApolloMapPtr apollo_map = std::make_shared<ApolloMap>();
        if (retrived_local_view->GetMapMsg()->hdmap().has_header() &&
            retrived_local_view->GetMapMsg()->hdmap().lane_size() == 0) {
          std::string TL_map_string;
          retrived_local_view->GetHDMapPtr()->GetMapString(&TL_map_string);
          auto TL_map_msg = std::make_shared<TLMapMsg>();
          TL_map_msg->ParsePartialFromString(TL_map_string);
          apollo_map = ConvertTool::ReverseMap(
              std::make_shared<TLMap>(TL_map_msg->hdmap()));
        } else {
          apollo_map = ConvertTool::ReverseMap(std::make_shared<TLMap>(
              retrived_local_view->GetMapMsg()->hdmap()));
        }
        auto* map_string = apollo_routing->mutable_measurement()->add_info();
        auto apollo_map_msg = std::make_shared<ApolloMapMsg>();
        ConvertTool::ReverseHeader(apollo_map_msg->mutable_header(),
                                   retrived_local_view->GetMapMsg()->header());
        apollo_map_msg->mutable_hdmap()->Swap(apollo_map.get());
        apollo_map_msg->SerializeToString(map_string);
      }
      routing_response_writer_->Write(apollo_routing);
    }

    if (FLAGS_export_secret) {
      SplitMap(retrived_local_view, ptr_trajectory_pb);
    }
    // send planning
    ADEBUG << "planning_writer_->Write(ptr_trajectory_pb),trajectory length:"
           << ptr_trajectory_pb->total_path_length();
    ADEBUG << "obstalce header:"
           << retrived_local_view->GetPerceptionObstacles()
                  ->header()
                  .ShortDebugString();
    TLADCTrajectory TL_adc_trajectory_copy;
    TL_adc_trajectory_copy.CopyFrom(*ptr_trajectory_pb);
    planning_writer_->Write(ConvertTool::ReverseADCTrajectory(
        std::make_shared<TLADCTrajectory>(TL_adc_trajectory_copy)));

    if (retrived_local_view->HasValidWithoutLaneFollowHeader()) {
      auto ptr_without_lane_pb = retrived_local_view->GetWithoutLaneFollow();
      if (!common::util::IsProtoEqual(last_without_lane_header,
                                      ptr_without_lane_pb->header())) {
        TLWithoutLaneFollowPtr TL_without_lane_follow_ptr =
            std::make_shared<TLWithoutLaneFollow>();
        TL_without_lane_follow_ptr->CopyFrom(*ptr_without_lane_pb);
        without_lane_line_writer_->Write(ConvertTool::ReverseWithoutLaneFollow(
            TL_without_lane_follow_ptr));
      }
    }
    //   // send lanemarker lane line debug info
    //   if (retrived_local_view->HasLanemarkersLaneLine()) {
    //     lanemarkers_laneline_writer_->Write(
    //         *retrived_local_view->GetLanemarkersLaneLine());
    //   }

    //   // send map state data msg
    //   if (retrived_local_view->HasMapStateData() &&
    //       !FLAGS_using_record_ehp_data) {
    //     ADEBUG << "write map state data!!!";
    //     map_state_data_writer_->Write(*retrived_local_view->GetMapStateData());
    //   }
    //   // send prediction
    if (retrived_local_view->HasPredictionObstacles()) {
      auto current_prediction = retrived_local_view->GetPredictionObstacles();
      if (!common::util::IsProtoEqual(last_prediction_header,
                                      current_prediction->header())) {
        last_prediction_header = current_prediction->header();
        prediction_writer_->Write(ConvertTool::ReversePredictionObstacles(
            std::make_shared<TLPredictionObstacles>(*current_prediction)));
      }
    }
    // send other when simulation
    if (FLAGS_enable_planning_self_simulator) {
      if (retrived_local_view->HasChassis()) {
        TLChassisPtr TL_chassis_ptr = std::make_shared<TLChassis>();
        TL_chassis_ptr->CopyFrom(*retrived_local_view->GetChassis());
        chassis_writer_->Write(ConvertTool::ReverseChassis(TL_chassis_ptr));
      }

      if (retrived_local_view->HasLocalization()) {
        TLLocalizationPtr TL_localization_ptr =
            std::make_shared<TLLocalization>();
        TL_localization_ptr->CopyFrom(
            *retrived_local_view->GetLocalization());
        localization_writer_->Write(
            ConvertTool::ReverseLocalization(TL_localization_ptr));
      }

      if (retrived_local_view->HasPerceptionObstacles()) {
        TLPerceptionObstaclesPtr TL_perception_obs_ptr =
            std::make_shared<TLPerceptionObstacles>();
        TL_perception_obs_ptr->CopyFrom(
            *retrived_local_view->GetPerceptionObstacles());
        perception_writer_->Write(
            ConvertTool::ReversePerceptionObstacles(TL_perception_obs_ptr));
      }
    }
  }
}

bool PlanningComponent::SplitString(const std::string& original) {
  uint64 len = original.length();
  if (!len) {
    return false;
  }
  int num = 50;
  if (len < 10 * 1024 * 1024) {  // NOLINT
    num = 30;
  }
  auto split_size = len / num + 1;
  uint64 end = split_size;
  std::string s;
  all_map_string_.resize(num, "");
  int i = 0;
  for (uint64 start = 0; start < len;) {
    if (i == num - 1) {  // 针对最后一个分割串
      s = original.substr(start, len - start);
      all_map_string_[i] = s;
      return true;
    }
    s = original.substr(start, split_size);
    all_map_string_[i] = s;
    start = end;
    end = end + split_size;
    i++;
  }
  return false;
}

void PlanningComponent::SplitMap(
    const std::shared_ptr<TL::planning::LocalView>& local_view,
    const std::shared_ptr<TL::planning::ADCTrajectory>& ptr_trajectory_pb) {
  static int cur_map_seq = -1;
  static bool status = false;
  if (local_view->HasMapMsg() && local_view->GetMapMsg()->has_hdmap() &&
      local_view->GetMapMsg()->hdmap().has_header()) {
    cur_map_seq = 0;
    std::string map_string;
    local_view->GetHDMapPtr()->GetMapString(&map_string);
    status = SplitString(map_string);
  }
  if (status) {
    auto* rcw_debug = ptr_trajectory_pb->mutable_debug()
                          ->mutable_warning_output()
                          ->mutable_rcw_debug();
    auto* debug_obj = rcw_debug->obj_info_size()
                          ? rcw_debug->mutable_obj_info(0)->mutable_debug_obj()
                          : rcw_debug->add_obj_info()->mutable_debug_obj();
    common::util::FillHeader("debug_obj", debug_obj);
    debug_obj->mutable_header()->set_seq(cur_map_seq);
    debug_obj->set_obj(all_map_string_[cur_map_seq]);
    cur_map_seq++;
    if (cur_map_seq >= static_cast<int>(all_map_string_.size())) {
      status = false;
      return;
    }
  }
}

void PlanningComponent::SelfSimulatorThread() {
  // sleep for init
  static constexpr double kSelfSimulatorIntervalMs = 100;
  pthread_setname_np(pthread_self(), "SelfSimulator");
  while (!is_self_simulator_thread_stop_) {
    const double start_time = TL::common::Clock::NowInMicroseconds();
    SelfSimulatorFunc();
    const double sleep_time =
        kSelfSimulatorIntervalMs -
        (TL::common::Clock::NowInMicroseconds() - start_time);
    if (sleep_time > 0) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(sleep_time)));
    }
  }
}

bool PlanningComponent::CheckInput(
    const std::shared_ptr<TL::planning::LocalView>& local_view) {
  ADCTrajectory trajectory_pb;
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (!local_view->HasLocalization()) {
    not_ready->set_reason("localization not ready");
  } else if (!local_view->HasChassis()) {
    not_ready->set_reason("chassis not ready");
  }

  if (not_ready->has_reason()) {
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    common::util::FillHeader(node_->Name(), &trajectory_pb);
    planning_writer_->Write(ConvertTool::ReverseADCTrajectory(
        std::make_shared<TLADCTrajectory>(trajectory_pb)));
    return false;
  }

  return true;
}

// NOLINTEND
}  // namespace planning
}  // namespace TL
