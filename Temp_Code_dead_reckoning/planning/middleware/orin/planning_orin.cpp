//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#include "planning/middleware/orin/planning_orin.h"
#include <sys/types.h>

#include <atomic>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/file/log.h"
#include "common/math/quaternion.h"
#include "common/time/clock.h"
#include "common/util/message_util.h"
#include "common/util/proto_adptr/lanemarkers_proto_adptr.h"
#include "common/util/proto_adptr/proto_filler_adptr.h"
#include "planning/common/planning_gflags.h"
#include "planning/localview/middleware_local_view.h"
#include "planning/middleware/common/cpu_recorder.h"
#include "planning/middleware/common/pre_process.h"
#include "planning/middleware/common/zmq_debug.h"
#include "planning/middleware/orin/orin_switch_manager.h"
#include "planning/warning/lbs/common/TM_Global_Types.h"
#include "proto/common/types.pb.h"
#include "proto/fsm/avp_fct.pb.h"
#include "proto/fsm/soc_to_mcu.pb.h"
#include "proto/local_mapping/local_map.pb.h"
#include "proto/map/navigation.pb.h"
#include "proto/perception/transport_element.pb.h"

namespace TL {

using TL::common::Clock;
using TL::common::LanemarkersProtoAdptr::ConvertLanemarkers;
using TL::common::ProtoFiller::FillChassis;
using TL::common::ProtoFiller::FillFctIn;
using TL::common::ProtoFiller::FillFreeSpace;
using TL::common::ProtoFiller::FillLocation;
using TL::common::ProtoFiller::FillObs;
using TL::common::ProtoFiller::FillParkinglot;
using TL::common::ProtoFiller::FillSlamPose;
using TL::functionmanager::DriveMode;
using TL::functionmanager::NNPSysState;
#ifndef ISX86
using TL::planning::PreProcess::CheckInputData;
#endif
using TL::planning::PreProcess::ClearHeaderStatus;
using TL::planning::PreProcess::SaveInputData;
#ifndef ISX86
using TL::planning::ZmqDebug::SerializeProtoToString;
#endif
// NOLINTBEGIN

namespace {
constexpr int32_t kHour2Sec = 3600;
constexpr int32_t kHour2Min = 60;
constexpr int32_t kDay2Hour = 24;
constexpr int32_t kHourDiff = 8;
constexpr double kFrontEdge2Center = 3.893;
}  // namespace

Planning::Planning() {
  publish_queue_ = std::make_shared<
      TL::common::base::BoundedQueue<std::shared_ptr<LocalView>>>();
  publish_queue_->Init(FLAGS_local_view_queue_size,
                       new TL::common::base::TimeoutBlockWaitStrategy(
                           FLAGS_local_view_queue_timeout));
  planning_trigger_cfg_queue_.Init(
      5, new TL::common::base::TimeoutBlockWaitStrategy(
             FLAGS_local_view_queue_timeout));
}

// NOLINTEND

void Planning::RegisterMessageType() {
  REGISTER_PROTO_MESSAGE_TYPE("chassis", Chassis);
  REGISTER_PROTO_MESSAGE_TYPE("nnp_localization", Localization);
  REGISTER_PROTO_MESSAGE_TYPE("hpp_localization", Localization);
  REGISTER_PROTO_MESSAGE_TYPE("nnp_fusion_lane", TransportElement);
  REGISTER_PROTO_MESSAGE_TYPE("hpp_lane", TransportElement);
  REGISTER_PROTO_MESSAGE_TYPE("hpp_freespace", FreeSpaceOutArray);
  REGISTER_PROTO_MESSAGE_TYPE("nnp_fusion_freespace", FreeSpaceOutArray);
  REGISTER_PROTO_MESSAGE_TYPE("nnp_obj_fusion", PerceptionObstacles);
  REGISTER_PROTO_MESSAGE_TYPE("hpp_obj_fusion", PerceptionObstacles);
  REGISTER_PROTO_MESSAGE_TYPE("parking_lot", ParkingLotOutArray);
  REGISTER_PROTO_MESSAGE_TYPE("mcu_to_ego", FunctionManagerIn);
  REGISTER_PROTO_MESSAGE_TYPE("mcu_to_soc_pnc", AdasSomeipFromMCU);
  REGISTER_PROTO_MESSAGE_TYPE("ego_trajectory", ADCTrajectory);
  REGISTER_PROTO_MESSAGE_TYPE("routing", RoutingResponse);
  REGISTER_PROTO_MESSAGE_TYPE("prediction", PredictionObstacles);
  REGISTER_PROTO_MESSAGE_TYPE("map_fusion", MapMsg);
  REGISTER_PROTO_MESSAGE_TYPE("control_data", McuToSocPnc);
  REGISTER_PROTO_MESSAGE_TYPE("sm_to_mcu", StateMachine);
  REGISTER_PROTO_MESSAGE_TYPE("statemachine", StateMachine);
  REGISTER_PROTO_MESSAGE_TYPE("apa2mcu_chassis", Apa2Chassis);
  REGISTER_PROTO_MESSAGE_TYPE("ncp_tlr_msg", TrafficLightDetection);
  REGISTER_PROTO_MESSAGE_TYPE("nnp_local_map", LocalMap);
  REGISTER_PROTO_MESSAGE_TYPE("fct_in", FunctionManagerIn);

  control_data_ = std::make_shared<McuToSocPnc>();

  chassis_ = std::make_shared<Chassis>();
  obstacles_ = std::make_shared<PerceptionObstacles>();
  location_ = std::make_shared<Localization>();
  slam_map_pose_ = std::make_shared<TL::common::Pose>();
  transport_element_ = std::make_shared<TransportElement>();
  parking_lot_array_ = std::make_shared<ParkingLotOutArray>();
  free_space_array_ = std::make_shared<FreeSpaceOutArray>();
  lane_markers_ = std::make_shared<LaneMarkers>();
  fct_in_ = std::make_shared<FunctionManagerIn>();
  adas_someip_from_mcu_ = std::make_shared<AdasSomeipFromMCU>();
  traffic_light_detection_ = std::make_shared<TrafficLightDetection>();
  local_map_ = std::make_shared<LocalMap>();
  map_fusion_ = std::make_shared<MapMsg>();
}

template <typename ProtoDataType>
std::shared_ptr<ProtoDataType> Planning::GetDataByTopic(
    NodeBundle* const input, const std::string& topic, int* const ret) {
  if (input == nullptr || ret == nullptr) {
    *ret = 0;
    NODE_LOG_ERROR << "NodeBundle is nullptr";
    return std::make_shared<ProtoDataType>();
  }
  const auto base_ptr = input->GetOne(topic);
  if (base_ptr == nullptr) {
    *ret = 0;
    NODE_LOG_ERROR << "Fail to recv data from topic: " << topic;
    return std::make_shared<ProtoDataType>();
  }

  const auto& ptr_data =
      std::static_pointer_cast<ProtoDataType>(base_ptr->proto_msg);
  if (ptr_data == nullptr) {
    *ret = 0;
    NODE_LOG_ERROR << "Fail to get proto data from " << topic;
    return std::make_shared<ProtoDataType>();
  }
  *ret = 1;
  return ptr_data;
}

int32_t Planning::AlgInit() {
#ifndef ISX86
  TL::planning::WarningCfg::Init();
#endif
  PauseTrigger(avp_topic_);
  RegisterMessageType();
  injector_ = std::make_shared<DependencyInjector>();
  regular_planning_base_ = std::make_unique<OnLanePlanning>(injector_);
  regular_planning_base_->SetPublishQueue(publish_queue_);

  guard_planning_base_ = std::make_unique<SafetyGuardPlanning>(injector_);
  guard_planning_base_->SetPublishQueue(publish_queue_);

  ACHECK(TL::common::GetProtoFromFile(FLAGS_planning_default_config_file,
                                         &config_))
      << "failed to load planning config file "
      << FLAGS_planning_default_config_file;
  vehicle_param_ =
      TL::common::VehicleConfigHelper::GetConfig().vehicle_param();
  regular_planning_base_->Init(config_);
  guard_planning_base_->Init(config_);
  if (FLAGS_enable_viz) {
    zmq_sender_ = std::make_unique<TL::common::ZMQSender>();
    zmq_sender_->Init(std::to_string(FLAGS_zmq_data_port));
  }
  if (FLAGS_enable_dreamview_to_planning_zmq) {
    zmq_receiver_ = std::make_unique<ZMQ2Dreamview>();
    zmq_receiver_->Init();
    zmq_receiver_->Start();
  }
#ifndef ISX86
  std::string has_trigger_config{};
  auto* cfg_client = ConfigParam::Instance();
  if (cfg_client != nullptr) {
    cfg_client->GetParam<std::string>("TL/planning_trigger_config",
                                      has_trigger_config);
  }
  std::string trigger_config_file = FLAGS_default_trigger_config_file;
  if (!has_trigger_config.empty()) {
    trigger_config_file = FLAGS_orin_trigger_config_file;
  }
  bool load_trigger_config =
      TL::common::GetProtoFromFile(trigger_config_file, &trigger_config_);
  if (!load_trigger_config) {
    AERROR << "Failed to load obs follow time config file ";
  }
  TL::planning::WarningCfg::GetVehicleCfg(&vehicle_cfg_);
  TL::planning::WarningCfg::GetWarningSwitchMem(&warning_switch_mem_);
  TL::planning::WarningCfg::GetLonCtrlSetDisMem(&lon_ctrl_set_dis_mem_);
#endif
  orin_trigger_manager_ = std::make_unique<OrinTriggerManager>();
  metric_collect_ = std::make_unique<MetricCollect>();
  writer_thread_ = std::thread(&Planning::PubTrajectoryThread, this);
  writer_trigger_cgf_thread_ =
      std::thread(&Planning::PubTriggerCfgThread, this);
#ifndef ISX86
  auto planning_trigger_config = [&](const std::string& domain,
                                     const std::string& key,
                                     const uint8& value) {  // NOLINT
    if ((domain == "TL") && (key == "planning_trigger_config")) {
      has_new_trigger_config_ = true;
    }
  };
  if (cfg_client != nullptr) {
    cfg_client->MonitorParam<uint8>("TL/planning_trigger_config",
                                    planning_trigger_config);
  }
#endif
  arena_adapter_ = std::make_shared<common::memory::ArenaAdapter>(
      FLAGS_pb_mem_pool_block_count, FLAGS_pb_mem_pool_block_size);
#ifdef ISX86
  FLAGS_is_mcap_replay = true;
#endif
  return 0;
}

int32_t Planning::AlgProcessNNP(NodeBundle* const input,
                                const ProfileToken& token) {
  TL::common::sub_thread_name = nnp_topic_;
  const auto now_ms = TL::common::Time::Now().ToMillisecond();
  const auto need_return = now_ms < (last_process_ms_ + gap_);
  if (is_assure_run_once_ || need_return) {
    return 0;
  }
  last_process_ms_ = now_ms;
  is_assure_run_once_.store(true);
  const auto fct_in = GetFctIn(input);
  if (FunctionMonitor(fct_in->ta_pilot_mode(), nnp_topic_)) {
    is_assure_run_once_.store(false);
    return 0;
  }
  arena_ = arena_adapter_->CreateProtobufArena();
  fct_in_ = fct_in;
  UpdateLonCtrlSetDisMem();
  GetTriggerConfig();
  GetMapFusionData(input);
  GetLocation(input, true);
  GetChassis(input);
  GetLocalMap(input);
  GetTransportElement(input, true);
  GetTrafficLight(input);
  GetFreeSpace(input, true);
  GetObstacle(input, true);
  GetSomeipAdasFromMcu(input);
  ClearAVPData();
  /*process Planning*/
  ProcessPlanning(token);

  is_assure_run_once_.store(false);
  return 0;
}

void Planning::GetMapFusionData(NodeBundle* const input) {
  const std::string topic = "map_fusion";
  auto ret = 0;
  map_fusion_recieved_ = false;
  map_fusion_ = GetDataByTopic<MapMsg>(input, topic, &ret);
#ifdef ISX86
  SaveInputData<MapMsg>(map_fusion_, "fusionmap", sequence_num_);
#endif
#ifndef ISX86
  CheckInputData<MapMsg>(ret, 10300000, true, map_fusion_);
#endif
  if (map_fusion_->has_hdmap() && map_fusion_->hdmap().lane_size() != 0 &&
      map_fusion_->has_routing()) {
    map_fusion_recieved_ = true;
  }
}

int32_t Planning::AlgProcessAVP(NodeBundle* const input,
                                const ProfileToken& token) {
  TL::common::sub_thread_name = avp_topic_;
  if (is_assure_run_once_) {
    return 0;
  }
  is_assure_run_once_.store(true);
  const auto fct_in = GetFctIn(input);
  if (FunctionMonitor(fct_in->ta_pilot_mode(), avp_topic_)) {
    is_assure_run_once_.store(false);
    return 0;
  }
  fct_in_ = fct_in;
  arena_ = arena_adapter_->CreateProtobufArena();
  GetTriggerConfig();
  GetLocation(input, false);
  GetChassis(input);
  GetObstacle(input, false);
  GetParkingLot(input);
  GetFreeSpace(input, false);
  GetSomeipAdasFromMcu(input);
  ClearNNPData();
  /*process Planning*/
  ProcessPlanning(token);
  is_assure_run_once_.store(false);
  return 0;
}

void Planning::ClearNNPData() {
  ClearHeaderStatus<TransportElement>(transport_element_);
  ClearHeaderStatus<LocalMap>(local_map_);
  ClearHeaderStatus<MapMsg>(map_fusion_);
  ClearHeaderStatus<TrafficLightDetection>(traffic_light_detection_);
}

void Planning::ClearAVPData() {
  ClearHeaderStatus<ParkingLotOutArray>(parking_lot_array_);
}

void Planning::UpdateLonCtrlSetDisMem() {
  if (fct_in_ != nullptr && lon_ctrl_set_dis_mem_ > 0) {
    fct_in_->mutable_fct_nnp_in()->set_longitud_ctrl_setdistance(
        lon_ctrl_set_dis_mem_);
  }
}

void Planning::GetParkingLot(NodeBundle* const input) {
  auto ret = 0;
  parking_lot_array_ =
      GetDataByTopic<ParkingLotOutArray>(input, "parking_lot", &ret);
#ifdef ISX86
  SaveInputData<ParkingLotOutArray>(parking_lot_array_, "parking_lot",
                                    sequence_num_);
#endif
#ifndef ISX86
  CheckInputData<ParkingLotOutArray>(ret, 10500000, true, parking_lot_array_);
#endif
  FillParkinglot(parking_lot_array_, *location_);
}

std::shared_ptr<FunctionManagerIn> Planning::GetFctIn(NodeBundle* const input) {
  auto fct_in = std::make_shared<FunctionManagerIn>();
  if (input == nullptr) {
    fct_in->set_ta_pilot_mode(TaPilotMode::NNP);
    return fct_in;
  }
  if (FLAGS_enable_dreamview_to_planning_zmq) {
    return std::make_shared<FunctionManagerIn>(
        zmq_receiver_->GetFunctionManagerIn());
  }
  auto ret = 0;
#ifdef ISX86
  sequence_num_++;
  const std::string topic = FLAGS_use_original_fct_in ? "mcu_to_ego" : "fct_in";
  fct_in = GetDataByTopic<FunctionManagerIn>(input, topic, &ret);
  SaveInputData<FunctionManagerIn>(fct_in, "fct_in", sequence_num_);
#else
  fct_in = GetDataByTopic<FunctionManagerIn>(input, "mcu_to_ego", &ret);
  CheckInputData<FunctionManagerIn>(ret, 10700000, true, fct_in);
#endif
  if (FLAGS_enable_planning_self_simulator) {
    fct_in->set_ta_pilot_mode(TaPilotMode::NNP);
    return fct_in;
  }
  FillFctIn(fct_in);
  return fct_in;
}

void Planning::GetLocation(NodeBundle* const input, const bool is_nnp_mode) {
  const std::string topic =
      is_nnp_mode ? "nnp_localization" : "hpp_localization";
  auto ret = 0;
  location_ = GetDataByTopic<Localization>(input, topic, &ret);
#ifdef ISX86
  SaveInputData<Localization>(location_, "localization", sequence_num_);
#endif
#ifndef ISX86
  CheckInputData<Localization>(ret, 10200000, is_nnp_mode, location_);
#endif
  FillSlamPose(location_, is_nnp_mode, slam_map_pose_);
  FillLocation(location_, is_nnp_mode);
}

void Planning::GetFreeSpace(NodeBundle* const input, const bool is_nnp_mode) {
  const std::string topic =
      is_nnp_mode ? "nnp_fusion_freespace" : "hpp_freespace";
  auto ret = 0;
  free_space_array_ = GetDataByTopic<FreeSpaceOutArray>(input, topic, &ret);
#ifdef ISX86
  SaveInputData<FreeSpaceOutArray>(free_space_array_, "free_space",
                                   sequence_num_);
#endif
#ifndef ISX86
  CheckInputData<FreeSpaceOutArray>(ret, 10600000, is_nnp_mode,
                                    free_space_array_);
#endif
  FillFreeSpace(free_space_array_, *location_, is_nnp_mode);
}

void Planning::GetObstacle(NodeBundle* const input, const bool is_nnp_mode) {
  const std::string topic = is_nnp_mode ? "nnp_obj_fusion" : "hpp_obj_fusion";
  auto ret = 0;
  obstacles_ = GetDataByTopic<PerceptionObstacles>(input, topic, &ret);
#ifdef ISX86
  SaveInputData<PerceptionObstacles>(obstacles_, "perception", sequence_num_);
#endif
#ifndef ISX86
  CheckInputData<PerceptionObstacles>(ret, 10400000, is_nnp_mode, obstacles_);
#endif
  FillObs(obstacles_, *location_, is_nnp_mode);
  // obstacles_->mutable_lane_marker()->CopyFrom(*lane_markers_);
}

void Planning::GetChassis(NodeBundle* const input) {
  auto ret = 0;
  chassis_ = GetDataByTopic<Chassis>(input, "chassis", &ret);
#ifdef ISX86
  SaveInputData<Chassis>(chassis_, "Chassis", sequence_num_);
#endif
#ifndef ISX86
  CheckInputData<Chassis>(ret, 10100000, true, chassis_);
#endif
  FillChassis(chassis_, *fct_in_);
  chassis_->mutable_vehicle_cfg()->CopyFrom(vehicle_cfg_);
  chassis_->mutable_warning_switch_mem()->CopyFrom(warning_switch_mem_);
}

void Planning::GetTransportElement(NodeBundle* const input,
                                   const bool is_nnp_mode) {
  std::string topic = is_nnp_mode ? "nnp_fusion_lane" : "hpp_lane";
  auto ret = 0;
  transport_element_ = GetDataByTopic<TransportElement>(input, topic, &ret);
#ifndef ISX86
  CheckInputData<TransportElement>(ret, 10800000, is_nnp_mode,
                                   transport_element_);
#endif
  lane_markers_ =
      common::memory::ArenaAdapter::CreateMessage<LaneMarkers>(arena_);
  ConvertLanemarkers(transport_element_, local_map_, lane_markers_);
}

void Planning::GetTrafficLight(NodeBundle* input) {
  int ret = 0;
  traffic_light_detection_ =
      GetDataByTopic<TrafficLightDetection>(input, "ncp_tlr_msg", &ret);
#ifdef ISX86
  SaveInputData<TrafficLightDetection>(traffic_light_detection_, "tlr_msg",
                                       sequence_num_);
#endif
#ifndef ISX86
  CheckInputData<TrafficLightDetection>(ret, 10900000, true,
                                        traffic_light_detection_);
#endif
}

void Planning::GetLocalMap(NodeBundle* input) {
  int ret = 0;
  local_map_ = GetDataByTopic<LocalMap>(input, "nnp_local_map", &ret);
#ifdef ISX86
  SaveInputData<LocalMap>(local_map_, "localmap", sequence_num_);
#endif
#ifndef ISX86
  CheckInputData<LocalMap>(ret, 10800000, true, local_map_);
#endif
}

void Planning::GetSomeipAdasFromMcu(NodeBundle* const input) {
  int ret = 0;
  adas_someip_from_mcu_ =
      GetDataByTopic<AdasSomeipFromMCU>(input, "mcu_to_soc_pnc", &ret);
#ifdef ISX86
  SaveInputData<AdasSomeipFromMCU>(adas_someip_from_mcu_, "7k", sequence_num_);
#endif
  TL::common::util::FillHeader("planning_orin", adas_someip_from_mcu_.get());
  // ControlDataAnalysis
  control_data_ = std::make_shared<McuToSocPnc>();
  TL::common::ProtoFiller::ControlDataAnalysis(
      adas_someip_from_mcu_, control_data_, vehicle_param_, chassis_, fct_in_);
}

int32_t Planning::SelfSimulatorFunc(NodeBundle* const input) {
  if (is_pub_thread_stop_) {
    return -1;
  }
  fct_in_ = GetFctIn(input);
  fct_in_->set_ta_pilot_mode(TaPilotMode::NNP);
  GetChassis(input);
  auto local_view = std::make_shared<LocalView>();
  local_view->SetArenaPtr(arena_adapter_->CreateProtobufArena());
  local_view->SetChassisPtr(chassis_);
  local_view->SetFunctionManagerInPtr(fct_in_);
  local_view->SetLaneMarkersPtr(std::make_shared<LaneMarkers>(LaneMarkers()));
  local_view->SetParkingLotOutArrayPtr(parking_lot_array_);
  regular_planning_base_->RunOnce(local_view);
  return 0;
}

void Planning::GetTriggerConfig() {
  if (has_new_trigger_config_) {
    TL::common::GetProtoFromFile(FLAGS_orin_trigger_config_file,
                                    &trigger_config_);
    has_new_trigger_config_ = false;
  }
  if (trigger_config_.has_nnp_config()) {
    fct_in_->mutable_trigger_config()->CopyFrom(trigger_config_);
    trigger_config_.Clear();
  } else {
    fct_in_->clear_trigger_config();
  }
}

void Planning::ProcessPlanning(const ProfileToken& token) {
  if (is_pub_thread_stop_) {
    return;
  }

  if (FLAGS_enable_planning_self_simulator) {
    return;
  }
  auto local_view = std::make_shared<MiddleWareLocalView>();
  local_view->SetArenaPtr(arena_);
  local_view->SetChassisPtr(chassis_);
  local_view->SetMcuToSocPncPtr(control_data_);
  if (map_fusion_recieved_) {
    map_fusion_->mutable_hdmap()
        ->mutable_header()
        ->mutable_header()
        ->set_frame_id("map_fusion_hdmap");
    if (!map_fusion_->routing().header().has_data_stamp()) {
      map_fusion_->mutable_routing()->mutable_header()->set_data_stamp(
          map_fusion_->routing().header().publish_stamp());
    }
    map_fusion_->mutable_routing()->mutable_header()->set_frame_id(
        "map_fusion_routing");
    local_view->SetMapMsgPtr(map_fusion_);
  }
  local_view->SetLocalizationPtr(location_);
  local_view->SetSlamMapPosePtr(slam_map_pose_);
  local_view->SetLaneMarkersPtr(lane_markers_);
  local_view->SetPerceptionObstaclesPtr(obstacles_);
  local_view->SetLocalMapPtr(local_map_);
  local_view->SetParkingLotOutArrayPtr(parking_lot_array_);
  local_view->SetFreeSpaceOutArrayPtr(free_space_array_);
  local_view->SetFunctionManagerInPtr(fct_in_);
  local_view->SetTransportElementPtr(transport_element_);
  local_view->SetTrafficLightDetectionPtr(traffic_light_detection_);
  local_view->SetMbdDebugFromMCUPtr(
      std::make_shared<MbdDebugFromMCU>(mbd_debug_));
  local_view->SetAdasSomeipFromMCUPtr(adas_someip_from_mcu_);
  local_view->SetToken(token);
  // planning入口
  guard_planning_base_->RunOnce(local_view);
  regular_planning_base_->RunOnce(local_view);
}

void Planning::PubControlData(
    const std::shared_ptr<TL::control::McuToSocPnc>& ptr_mcu_to_soc_pnc,
    const ProfileToken& token) {
  ptr_mcu_to_soc_pnc->mutable_header()->set_publish_stamp(
      Clock::NowInSeconds());
  Send(token, "control_data", ptr_mcu_to_soc_pnc);
}

void Planning::PubFctIn(const std::shared_ptr<FunctionManagerIn>& ptr_fct_in,
                        const ProfileToken& token) {
  ptr_fct_in->mutable_header()->set_publish_stamp(Clock::NowInSeconds());
  Send(token, "fct_in", ptr_fct_in);
}

void Planning::PubTrajectory(
    const std::shared_ptr<ADCTrajectory>& ptr_adc_trajectory,
    const std::shared_ptr<LocalView>& local_view, const ProfileToken& token) {
  UpdateTopicHeaderInfo(ptr_adc_trajectory, local_view);
  // timestamp
  ptr_adc_trajectory->set_beijing_time(Clock::NowInSecondsForBeiJing());
  ptr_adc_trajectory->mutable_header()->set_publish_stamp(
      ptr_adc_trajectory->header().data_stamp());

  // pub signal for monitoring planning running status to mcu
  auto* mutable_u32 = ptr_adc_trajectory->mutable_soc_to_fct_bus()
                          ->mutable_soc_to_fct_bus_u32();
  static uint32 monitor_counter_u32 = 0;
  // 从0每100毫秒加1直到达到最大值4294967295，需要约42949672.95秒，或约11930.8小时，或约497.95天，所以不会溢出
  mutable_u32->set_trajectory_seq(++monitor_counter_u32);

  Send(token, "ego_trajectory", ptr_adc_trajectory);
}

void Planning::PubRoutingResponse(
    const std::shared_ptr<RoutingResponse>& routing_response,
    const ProfileToken& token) {
  routing_response->mutable_header()->set_publish_stamp(Clock::NowInSeconds());
  Send(token, "routing", routing_response);
}

void Planning::PubPredictionObstacles(
    const std::shared_ptr<PredictionObstacles>& ptr_prediction_obstacles,
    const ProfileToken& token) {
  ptr_prediction_obstacles->mutable_header()->set_publish_stamp(
      Clock::NowInSeconds());
  Send(token, "prediction", ptr_prediction_obstacles);
}

void Planning::PubFctBus(const std::shared_ptr<SocToFctBus>& ptr_soc_to_fct_bus,
                         const ProfileToken& token) {
  ptr_soc_to_fct_bus->mutable_header()->set_publish_stamp(
      Clock::NowInSeconds());
  Send(token, "soc_to_mcu_pnc", ptr_soc_to_fct_bus);
}

void Planning::ProcessWarningAndEthHmi(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<ADCTrajectory>& ptr_adc_trajectory,
    const uint8 mcu_platform_fault) {
  orin_trigger_manager_->GetWarningFaultStatus(
      ptr_adc_trajectory->mutable_debug()->mutable_monitor_fault_debug());
  if ((mcu_platform_fault & 0xFD) > 0) {
    auto* monitor_fault =
        ptr_adc_trajectory->mutable_debug()->mutable_monitor_fault_debug();
    monitor_fault->mutable_warning_fault()->set_dow(true);
    monitor_fault->mutable_warning_fault()->set_rcta(true);
    monitor_fault->mutable_warning_fault()->set_rcw(true);
    monitor_fault->mutable_warning_fault()->set_fcta(true);
    monitor_fault->mutable_warning_fault()->set_lca(true);
  }
  regular_planning_base_->ProcessOutputData(local_view, ptr_adc_trajectory);
}

void Planning::PubTriggerCfgThread() {
  pthread_setname_np(pthread_self(), "PubTriggerCfg");
  while (!is_pub_trigger_thread_stop_) {
    std::shared_ptr<TriggerAndCfgPub> trigger_cfg_pub(nullptr);
    if (!planning_trigger_cfg_queue_.WaitDequeue(&trigger_cfg_pub) ||
        trigger_cfg_pub == nullptr) {
      continue;
    }
#ifndef ISX86
    TL::planning::WarningCfg::SetWarningSwitchMem(
        trigger_cfg_pub->warning_status, &last_warning_state_);
    TL::planning::WarningCfg::SetLonCtrlSetDisMem(
        trigger_cfg_pub->adcs_longitud_ctrl_setdistance);
    orin_trigger_manager_->PubEventDataToTrigger(
        trigger_cfg_pub->event_trigger);
    orin_trigger_manager_->PubFaultDataToTrigger(
        trigger_cfg_pub->planning_fault,
        trigger_cfg_pub->is_state_ready_to_report);
#endif
  }
}

void Planning::PubTrajectoryThread() {
  pthread_setname_np(pthread_self(), "PubTrajectory");
  planning::FunctionStatistics done_guard("PubTrajectoryThread");
  std::shared_ptr<LocalView> last_local_view(nullptr);
  uint64_t pub_seq = 0;
  while (!is_pub_thread_stop_) {
    std::shared_ptr<LocalView> local_view(nullptr);
    if (!publish_queue_->WaitDequeue(&local_view) || local_view == nullptr) {
      continue;
    }
    double start_time = Clock::NowInSeconds();
    // 遇到新的一帧，发布轨迹
    auto token = GetToken(local_view);
    bool is_fusionmap = false;
    if (local_view->HasADCTrajectory()) {
      is_fusionmap =
          local_view->GetADCTrajectory()->has_function_manager_out() &&
          local_view->GetADCTrajectory()->function_manager_out().fsm_state() ==
              functionmanager::MachineStateType::HDMAP_TYPE;
#ifdef ISORIN
      auto ptr_fct_in = std::make_shared<FunctionManagerIn>(
          local_view->GetADCTrajectory()->function_manager_in());
      PubFctIn(ptr_fct_in, token);
#endif
    }
    if (last_local_view == local_view) {
      if (local_view->HasADCTrajectory()) {
        last_regular_adc_trajectory_ = local_view->GetADCTrajectory();
      }
      continue;
    }
    if (!ProcessGuardTrajectory(local_view, token)) {
      ProcessRegularTrajectory(local_view, token);
    }
    if (local_view->HasMcuToSocPnc()) {
      PubControlData(
          std::const_pointer_cast<McuToSocPnc>(local_view->GetMcuToSocPnc()),
          token);
    }

    if (local_view->HasPredictionObstacles()) {
      PubPredictionObstacles(std::const_pointer_cast<PredictionObstacles>(
                                 local_view->GetPredictionObstacles()),
                             token);
    }
    if ((local_view->HasRoutingResponse())) {
      auto current_routing =
          std::const_pointer_cast<TL::routing::RoutingResponse>(
              local_view->GetRoutingResponse());
      if (!is_fusionmap) {
        // send map msg
        std::string map_msg{};
        if (local_view->HasMapMsg()) {
          local_view->GetMapMsg()->SerializeToString(&map_msg);
        }
        current_routing->mutable_measurement()->add_info(map_msg);
      }
      PubRoutingResponse(current_routing, token);
    }
    last_local_view = local_view;
    std::stringstream ss;
    ss.precision(13);
    ss << "------PubTrajectoryThread------pub_seq: " << pub_seq++
       << " pub_use_time: " << (Clock::NowInSeconds() - start_time) * 1000
       << " endtime: " << Clock::NowInSeconds();
    AINFO << ss.str();
  }
}

bool Planning::ProcessGuardTrajectory(  // NOLINT
    const std::shared_ptr<LocalView>& current_local_view,
    const ProfileToken& token) {
  if (current_local_view == nullptr ||
      !current_local_view->HasADCTrajectoryGuard() ||
      current_local_view->HasADCTrajectory()) {
    return false;
  }
  auto guard_adctrajectory = std::const_pointer_cast<ADCTrajectory>(
      current_local_view->GetADCTrajectoryGuard());
  if (guard_adctrajectory == nullptr || !guard_adctrajectory->has_header() ||
      guard_adctrajectory->trajectory_point().empty()) {
    return false;
  }
  UpdateNTPDistanceAndTime(*current_local_view, guard_adctrajectory);
  if (last_regular_adc_trajectory_ != nullptr) {
    guard_adctrajectory->set_gear(last_regular_adc_trajectory_->gear());
  }
  const auto start_timestamp = Clock::NowInSeconds();
  UpdateFctOutByTrajectory(guard_adctrajectory);
  // PubFctOut2MCU(guard_adctrajectory, token);
  auto* pub_status =
      guard_adctrajectory->mutable_latency_stats()->add_task_stats();
  pub_status->set_name("process_guard_trajectory");
  pub_status->set_time_ms((Clock::NowInSeconds() - start_timestamp) * 1000);
  if (current_local_view->HasAdasSomeipFromMCU() &&
      current_local_view->GetAdasSomeipFromMCU()->has_header()) {
    auto* total_planning_status =
        guard_adctrajectory->mutable_latency_stats()->add_task_stats();
    total_planning_status->set_name("total_guard_planning_time");
    total_planning_status->set_time_ms(
        (Clock::NowInSeconds() -
         current_local_view->GetAdasSomeipFromMCU()->header().data_stamp()) *
        1000);
  }
  if (current_local_view->HasADCTrajectory()) {
    last_regular_adc_trajectory_ = current_local_view->GetADCTrajectory();
  }
  guard_adctrajectory->mutable_debug()->mutable_planning_data()->CopyFrom(
      last_regular_adc_trajectory_->debug().planning_data());
  guard_adctrajectory->mutable_header()->mutable_status()->CopyFrom(
      last_regular_adc_trajectory_->header().status());
  guard_adctrajectory->mutable_avp_to_hmi()->CopyFrom(
      last_regular_adc_trajectory_->avp_to_hmi());
  metric_collect_->UpdateMetricData(current_local_view);
  // 遇到on_lane_planning的轨迹，更新last_regular_adc_trajectory_，发布调试信息
  if (current_local_view->HasADCTrajectoryGuard()) {
    PublishDebug(current_local_view, *guard_adctrajectory);
  }
  // if (guard_adctrajectory->has_soc_to_fct_bus()) {
  //   PubFctBus(
  //       std::make_shared<SocToFctBus>(guard_adctrajectory->soc_to_fct_bus()),
  //       token);
  // }
  PubTrajectory(guard_adctrajectory, current_local_view, token);
  return true;
}

bool Planning::ProcessRegularTrajectory(
    const std::shared_ptr<LocalView>& retrived_local_view,
    const ProfileToken& token) {
  if (!retrived_local_view || !retrived_local_view->HasADCTrajectory()) {
    return false;
  }

  if (retrived_local_view->GetADCTrajectory()->has_header()) {
    const auto start_timestamp = Clock::NowInSeconds();
    // 为了避免copy adc_trajectory, 且为了避免多线程数据竞争无法在 local_view 加 mutable 接口
    // 此为设计失误，不应该把所有东西都塞在 trajectory 里，造成很多数据处理时发生耦合
    auto ptr_adc_trajectroy = std::const_pointer_cast<ADCTrajectory>(
        retrived_local_view->GetADCTrajectory());
    if (ptr_adc_trajectroy == nullptr) {
      return false;
    }
    UpdateNTPDistanceAndTime(*retrived_local_view, ptr_adc_trajectroy);
    uint8 mcu_platform_fault = 0x00;
    if (retrived_local_view->HasAdasSomeipFromMCU() &&
        retrived_local_view->GetAdasSomeipFromMCU()->adas_someip_size() > 837) {
      mcu_platform_fault =
          retrived_local_view->GetAdasSomeipFromMCU()->adas_someip(836);
    }
    ProcessWarningAndEthHmi(retrived_local_view, ptr_adc_trajectroy,
                            mcu_platform_fault);
    metric_collect_->UpdateMetricData(retrived_local_view);
    auto trigger_and_cfg_pub = std::make_shared<TriggerAndCfgPub>();
    PubTriggerAndCfg(ptr_adc_trajectroy, trigger_and_cfg_pub,
                     mcu_platform_fault);
    if (retrived_local_view->HasFunctionManagerIn() &&
        retrived_local_view->GetFunctionManagerIn()->ta_pilot_mode() ==
            TL::functionmanager::AVP &&
        retrived_local_view->GetFunctionManagerIn()->fct_avp_in().sys_mode() ==
            TL::functionmanager::AvpFctIn::APA &&
        (!retrived_local_view->HasFunctionManagerOut() ||
         !retrived_local_view->GetFunctionManagerOut()->avp_status()) &&
        last_regular_adc_trajectory_ != nullptr) {
      ptr_adc_trajectroy->mutable_avp_to_hmi()->CopyFrom(
          last_regular_adc_trajectory_->avp_to_hmi());
    }
    RecordCpuAndMemInfo(ptr_adc_trajectroy.get());
    auto* pub_status =
        ptr_adc_trajectroy->mutable_latency_stats()->add_task_stats();
    pub_status->set_name("process_regular_trajectory");
    pub_status->set_time_ms((Clock::NowInSeconds() - start_timestamp) * 1000);
    if (retrived_local_view->HasAdasSomeipFromMCU() &&
        retrived_local_view->GetAdasSomeipFromMCU()->has_header()) {
      auto* total_planning_status =
          ptr_adc_trajectroy->mutable_latency_stats()->add_task_stats();
      total_planning_status->set_name("total_regular_planning_time");
      total_planning_status->set_time_ms(
          (Clock::NowInSeconds() -
           retrived_local_view->GetAdasSomeipFromMCU()->header().data_stamp()) *
          1000);
    }
    last_regular_adc_trajectory_ = retrived_local_view->GetADCTrajectory();
    // 遇到on_lane_planning的轨迹，更新last_regular_adc_trajectory_，发布调试信息
    if (retrived_local_view->HasADCTrajectory()) {
      PublishDebug(retrived_local_view,
                   *retrived_local_view->GetADCTrajectory());
    }
    // if (ptr_adc_trajectroy->has_soc_to_fct_bus()) {
    //   PubFctBus(
    //       std::make_shared<SocToFctBus>(ptr_adc_trajectroy->soc_to_fct_bus()),
    //       token);
    // }

    PubTrajectory(ptr_adc_trajectroy, retrived_local_view, token);
    if (!is_pub_trigger_thread_stop_) {
      planning_trigger_cfg_queue_.ForceEnqueue(trigger_and_cfg_pub);
    }
  }
  return true;
}

void Planning::PubTriggerAndCfg(
    const std::shared_ptr<ADCTrajectory>& ptr_adc_trajectory,
    const std::shared_ptr<TriggerAndCfgPub>& trigger_and_cfg_pub,
    const uint8 mcu_platform_fault) {
  orin_trigger_manager_->GetWarningFaultStatus(
      ptr_adc_trajectory->mutable_debug()->mutable_monitor_fault_debug());
  if ((mcu_platform_fault & 0xFF) > 0) {
    auto* mutable_monitor_fault_clusters =
        ptr_adc_trajectory->mutable_debug()
            ->mutable_monitor_fault_debug()
            ->mutable_monitor_fault()
            ->mutable_monitor_fault_clusters();
    if (mutable_monitor_fault_clusters != nullptr) {
      std::stringstream ss{};
      ss << std::hex << std::setw(2) << std::setfill('0')
         << static_cast<int>(mcu_platform_fault);
      const std::string hex_str = "0x" + ss.str();
      (*mutable_monitor_fault_clusters)["mcu_platform_fault"] = hex_str;
    }
  }
  if (is_pub_trigger_thread_stop_) {
    return;
  }
  bool is_state_ready_to_report = true;
  const auto& fct_in = ptr_adc_trajectory->function_manager_in();
  const auto& nnp_state = fct_in.fct_nnp_in().nnp_sysstate();
  switch (fct_in.driver_mode()) {
    case DriveMode::NNP_LAT_ACTIVE_LGT_OVERRIDE:
    case DriveMode::NNP_LGT_ACTIVE_LAT_OVERRIDE:
    case DriveMode::NNP_LAT_LGT_ACTIVE:
      is_state_ready_to_report = nnp_state != NNPSysState::NNPS_OFF &&
                                 nnp_state != NNPSysState::NNPS_PASSIVE;
      break;
    case DriveMode::AVP_LAT_ACTIVE_LGT_OVERRIDE:
    case DriveMode::AVP_LGT_ACTIVE_LAT_OVERRIDE:
    case DriveMode::AVP_LAT_LGT_ACTIVE:
      break;
    default:
      is_state_ready_to_report = false;
  }
  {
    trigger_and_cfg_pub->warning_status = ptr_adc_trajectory->warning_status();
    trigger_and_cfg_pub->adcs_longitud_ctrl_setdistance =
        ptr_adc_trajectory->function_manager_out()
            .nnp_fct_out()
            .adcs_longitud_ctrl_setdistance();
    trigger_and_cfg_pub->planning_fault = ptr_adc_trajectory->planning_fault();
    trigger_and_cfg_pub->event_trigger = ptr_adc_trajectory->event_trigger();
    trigger_and_cfg_pub->is_state_ready_to_report = is_state_ready_to_report;
  }
}

void Planning::PublishDebug(                       // NOLINT
    const std::shared_ptr<LocalView>& local_view,  // NOLINT
    const ADCTrajectory& adctrajectory) {          // NOLINT
  if (!FLAGS_enable_viz) {
    return;
  }
  // NodeBundle output;
  std::vector<std::string> msg;
// hz_Adsfi::DebugPlanningFrame planning_debug;
#ifndef ISX86
  SerializeProtoToString(local_view, adctrajectory, &msg, zmq_sender_);
#endif
}

ProfileToken Planning::GetToken(  // NOLINT
    const std::shared_ptr<LocalView>& retrived_local_view) {
  const auto orin_local_view =
      std::dynamic_pointer_cast<MiddleWareLocalView>(retrived_local_view);
  if (orin_local_view != nullptr && !FLAGS_enable_planning_self_simulator) {
    return orin_local_view->GetToken();
  }
  return ProfileToken{};
}

template <typename ProtoDataType>
void Planning::Send(const ProfileToken& token, const std::string& tpoic,
                    const std::shared_ptr<ProtoDataType>& proto_data) {
  BaseDataTypePtr base_ptr(new BaseData);
  NodeBundle output;
  base_ptr->proto_msg = proto_data;
  output.Add(tpoic, base_ptr);
  if (FLAGS_enable_planning_self_simulator) {
    SendOutput(&output);
  } else {
    SendOutput(&output, token);
  }
}

void Planning::UpdateFctOutByTrajectory(  // NOLINT
    const std::shared_ptr<ADCTrajectory>& ptr_adc_trajectory) {
  if (ptr_adc_trajectory != nullptr && !trajectory_check_ok_) {
    ptr_adc_trajectory->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason("trajectory has nan number");
    auto* nnp_fault = ptr_adc_trajectory->mutable_function_manager_out()
                          ->mutable_nnp_fct_out()
                          ->mutable_nnp_software_fault();
    nnp_fault->set_plan_trajectory_success(false);
    nnp_fault->set_planning_success(false);
  }
}

bool Planning::FunctionMonitor(const TaPilotMode& ta_polit_mode,
                               const std::string& function_topic) {
  if (function_topic == avp_topic_ && ta_polit_mode != TaPilotMode::AVP) {
    PauseTrigger(avp_topic_);
    ResumeTrigger(nnp_topic_);
    AINFO << " mode changed to nnp, ta polit mode: " << ta_polit_mode;
    return true;
  } else if (function_topic == nnp_topic_ &&  // NOLINT
             ta_polit_mode == TaPilotMode::AVP) {
    PauseTrigger(nnp_topic_);
    ResumeTrigger(avp_topic_);
    AINFO << " mode changed to avp, ta polit mode: " << ta_polit_mode;
    return true;
  }
  return false;
}

void Planning::RecordCpuAndMemInfo(             // NOLINT
    ADCTrajectory* const ptr_adc_trajectory) {  // NOLINT
  if (ptr_adc_trajectory == nullptr) {
    return;
  }
  const auto planning_cpu = TL::planning::CpuRecorder::GetCpuUsageRatio();
  const auto planning_mem = TL::planning::CpuRecorder::GetMemoryUsage();
  const auto all_cpu = TL::planning::CpuRecorder::GetAllCpuData();
  const auto mem_free = TL::planning::CpuRecorder::GetSystemMemData().first;
  const auto mem_total =
      TL::planning::CpuRecorder::GetSystemMemData().second;
  auto* debug = ptr_adc_trajectory->mutable_debug()
                    ->mutable_planning_data()
                    ->mutable_mdc_cpu_info();
  debug->set_planning_cpu(planning_cpu);
  debug->set_planning_mem(planning_mem);
  debug->set_all_cpu_used(all_cpu);
  debug->set_mem_free(mem_free);
  debug->set_total_mem(mem_total);
}

void Planning::UpdateTopicHeaderInfo(  // NOLINT
    const std::shared_ptr<ADCTrajectory>& ptr_adc_trajectory,
    const std::shared_ptr<LocalView>& local_view) {
  auto* mubtable_headers =
      ptr_adc_trajectory->mutable_debug()->mutable_planning_data();
  if (mubtable_headers == nullptr) {
    return;
  }
  auto* mutable_header = mubtable_headers->add_headers();
  if (local_view->HasLocalization() && mutable_header != nullptr) {
    mutable_header->CopyFrom(local_view->GetLocalization()->header());
    mutable_header->set_frame_id("localization");
  }
  mutable_header = mubtable_headers->add_headers();
  if (local_view->HasMapMsg() && mutable_header != nullptr) {
    mutable_header->CopyFrom(local_view->GetMapMsg()->header());
    mutable_header->set_frame_id("map_msg");
  }
  mutable_header = mubtable_headers->add_headers();
  if (local_view->HasHDMap() && mutable_header != nullptr) {
    mutable_header->CopyFrom(
        local_view->GetHDMapPtr()->GetMapHeader().header());
    mutable_header->set_frame_id("hdmap");
  }
  mutable_header = mubtable_headers->add_headers();
  if (local_view->HasFunctionManagerIn() && mutable_header != nullptr) {
    mutable_header->CopyFrom(local_view->GetFunctionManagerIn()->header());
    mutable_header->set_frame_id("fct_in");
  }
  mutable_header = mubtable_headers->add_headers();
  if (local_view->HasPerceptionObstacles() && mutable_header != nullptr) {
    mutable_header->CopyFrom(local_view->GetPerceptionObstacles()->header());
    mutable_header->set_frame_id("per_obs");
  }
  mutable_header = mubtable_headers->add_headers();
  if (local_view->HasLocalMap() && mutable_header != nullptr) {
    mutable_header->CopyFrom(local_view->GetLocalMap()->header());
    mutable_header->set_frame_id("local_map");
  }
  mutable_header = mubtable_headers->add_headers();
  if (local_view->HasTransportElement() && mutable_header != nullptr) {
    mutable_header->CopyFrom(local_view->GetTransportElement()->header());
    mutable_header->set_frame_id("transport_element");
  }
  mutable_header = mubtable_headers->add_headers();
  if (local_view->HasParkingLotOutArray() && mutable_header != nullptr) {
    mutable_header->CopyFrom(local_view->GetParkingLotOutArray()->header());
    mutable_header->set_frame_id("parking_lot");
  }
  mutable_header = mubtable_headers->add_headers();
  if (local_view->HasFreeSpaceOutArray() && mutable_header != nullptr) {
    mutable_header->CopyFrom(local_view->GetFreeSpaceOutArray()->header());
    mutable_header->set_frame_id("free_space");
  }
  mutable_header = mubtable_headers->add_headers();
  if (local_view->HasTrafficLightDetection() && mutable_header != nullptr) {
    mutable_header->CopyFrom(local_view->GetTrafficLightDetection()->header());
    mutable_header->set_frame_id("traffic_light");
  }
  mutable_header = mubtable_headers->add_headers();
  if (local_view->HasChassis() && mutable_header != nullptr) {
    mutable_header->CopyFrom(local_view->GetChassis()->header());
    mutable_header->set_frame_id("chassis");
  }
  mutable_header = mubtable_headers->add_headers();
  if (local_view->HasPredictionObstacles() && mutable_header != nullptr) {
    mutable_header->CopyFrom(local_view->GetPredictionObstacles()->header());
  }
}

void Planning::AlgRelease() {
  planning::FunctionStatistics done_guard("Planning::AlgRelease");
  publish_queue_->BreakAllWait();
  planning_trigger_cfg_queue_.BreakAllWait();
  is_pub_thread_stop_ = true;
  is_pub_trigger_thread_stop_ = true;
  is_assure_run_once_.store(false);
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }
  if (writer_trigger_cgf_thread_.joinable()) {
    writer_trigger_cgf_thread_.join();
  }
  TL::planning::WarningCfg ::Release();
}

void Planning::UpdateNTPDistanceAndTime(
    const LocalView& local_view,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  const auto fct_in = local_view.GetFunctionManagerIn()->fct_avp_in();
  const auto cur_driving_mode = local_view.GetVehicleState()->driving_mode();
  if (fct_in.sys_mode() != functionmanager::AvpFctIn::LOCALIZATION &&
      fct_in.sys_mode() != functionmanager::AvpFctIn::LAPA) {
    all_his_time_ = 0.0;
    all_his_distance_ = 0.0;
    pre_distance_ = 0.0;
    pre_time_ = 0.0;
    pre_driving_mode_ = cur_driving_mode;
    return;
  }
  if (cur_driving_mode == soc::Chassis::COMPLETE_AUTO_DRIVE) {
    if (fct_in.sys_command() != functionmanager::AvpFctIn::PARKINCONTROL) {
      double remain_dis = local_view.GetFunctionManagerOut()
                              ->avp_fct_out()
                              .avp_localview_info()
                              .remain_distance() -
                          FLAGS_hdmap_avp_path_extend_buffer -
                          kFrontEdge2Center;
      if (pre_driving_mode_ == soc::Chassis::COMPLETE_AUTO_DRIVE) {
        all_his_distance_ += pre_distance_ - remain_dis;
        all_his_time_ += TL::common::Clock::NowInSeconds() - pre_time_;
      }
      pre_distance_ = remain_dis;
      pre_time_ = TL::common::Clock::NowInSeconds();
      UpdateLAPAParkingStageHmiData(remain_dis, ptr_trajectory_pb);
    } else {
      UpdateLAPAParkingStageHmiData(all_his_distance_, ptr_trajectory_pb);
    }
  }
  pre_driving_mode_ = cur_driving_mode;
}

void Planning::UpdateLAPAParkingStageHmiData(
    const double remain_dist,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (ptr_trajectory_pb == nullptr) {
    return;
  }
  auto* avp_to_hmi = ptr_trajectory_pb->mutable_avp_to_hmi();
  auto passed_time = all_his_time_;
  avp_to_hmi->set_park_time_remaining(static_cast<int32_t>(passed_time));
  // 巡航过程中显示剩余距离，泊入过程中显示巡航总长度
  avp_to_hmi->set_hpa_distance(static_cast<int32_t>(remain_dist));
  // 时分秒设置
  const auto passed_time_int32 = static_cast<int32_t>(passed_time);
  int32_t hour = (passed_time_int32 + kHourDiff * kHour2Sec) / (kHour2Sec);
  hour %= kDay2Hour;
  int32_t minute = (passed_time_int32 % kHour2Sec);
  minute /= kHour2Min;
  avp_to_hmi->set_hour_of_day(hour);
  if (passed_time_int32 % 60 > 30) {
    avp_to_hmi->set_minute_of_hour(minute + 1);
  } else {
    avp_to_hmi->set_minute_of_hour(minute);
  }
  avp_to_hmi->set_second_of_minute(0);
}

}  // namespace TL
