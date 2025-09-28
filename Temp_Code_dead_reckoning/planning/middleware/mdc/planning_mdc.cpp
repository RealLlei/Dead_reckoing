//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#include "planning/middleware/mdc/planning_mdc.h"

#include <atomic>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "absl/strings/match.h"
#include "ap-release/include/adsfi/include/data_types/debug/pbdebug.h"
#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/file/log.h"
#include "common/pb2struct/pb2struct.h"
#include "common/struct2pb/struct2pb.h"
#include "common/time/clock.h"
#include "common/util/message_util.h"
#include "config_client/include/hz_cfg_client.h"
#include "data_types/debug/adas_state_types.h"
#include "data_types/perception/perception_info.h"
#include "data_types/vehicle/canfd_msg.h"
#include "hz_cangen/include/hz_canagent.h"
#include "node/node_profiler_token.h"
#include "planning/localview/middleware_local_view.h"
#include "planning/middleware/common/cpu_recorder.h"
#include "planning/middleware/common/pre_process.h"
#include "planning/middleware/common/zmq_debug.h"
#include "planning/middleware/mdc/switch_manager.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/hmi/nns_router.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/perception/transport_element.pb.h"
#include "proto/planning/planning.pb.h"

namespace TL {
using TL::common::Clock;
using TL::common::GetProtoFromFile;
using TL::functionmanager::DriveMode;
using TL::functionmanager::NNPSysState;
using TL::functionmanager::WarningLevel;
using TL::planning::TrajectoryStitcher;

using TL::canstack::cangen::CanAgent;
using TL::config::client::HzCfgClient;
using TL::planning::PreProcess::CheckInputData;
using TL::planning::PreProcess::CheckPublishTimestamp;
using TL::planning::ZmqDebug::SerializeProtoToString;

Planning::Planning() {
  publish_queue_ = std::make_shared<
      TL::common::base::BoundedQueue<std::shared_ptr<LocalView>>>();
  publish_queue_->Init(FLAGS_local_view_queue_size,
                       new TL::common::base::TimeoutBlockWaitStrategy(
                           FLAGS_local_view_queue_timeout));
}

int32_t Planning::AlgProcessNNP(NodeBundle* const input,
                                const hz_Adsfi::ProfileToken& token) {
  TL::common::sub_thread_name = nnp_topic_;
  const auto now_ms = TL::common::Time::Now().ToMillisecond();
  const auto need_return = now_ms < (last_process_ms_ + gap_);
  if (is_assure_run_once_ || need_return) {
    return 1;
  }
  last_process_ms_ = now_ms;
  is_assure_run_once_.store(true);
  TL::functionmanager::FunctionManagerIn fct_in;
  if (FunctionMonitor(GetFctIn(input, &fct_in), nnp_topic_)) {
    is_assure_run_once_.store(false);
    last_process_ms_ = 0;
    return 1;
  }
  fct_in_.CopyFrom(fct_in);
  GetTriggerConfig();
  GetAdasDataFromMcu(input);
  GetMbdDebugFromMcu(input);
  GetLocation(input, true);
  GetChassis(input);
  GetLaneArray(input, true);
  GetObstacle(input, true);
  GetAebInfo(input);
  GetTsrData(input);
  GetMcu7kData(input);
  GetTrafficLightAndRouting(input);
  ClearDataByFctMode(true);
  /*process Planning*/
  ProcessPlanning(token);

  is_assure_run_once_.store(false);
  return 1;
}

int32_t Planning::AlgProcessAVP(NodeBundle* const input,
                                const hz_Adsfi::ProfileToken& token) {
  TL::common::sub_thread_name = avp_topic_;
  if (is_assure_run_once_) {
    return 1;
  }
  is_assure_run_once_.store(true);
  TL::functionmanager::FunctionManagerIn fct_in;
  if (FunctionMonitor(GetFctIn(input, &fct_in), avp_topic_)) {
    is_assure_run_once_.store(false);
    return 1;
  }
  fct_in_.CopyFrom(fct_in);
  GetTriggerConfig();
  GetAdasDataFromMcu(input);
  GetMbdDebugFromMcu(input);
  GetLocation(input, false);
  GetChassis(input);
  GetLaneArray(input, false);
  GetObstacle(input, false);
  GetParkingLot(input);
  GetFreeSpace(input, false);
  GetAebInfo(input);
  GetNnsRoute(input);
  GetNnsLocation(input);
  GetMcu7kData(input);
  /*process Planning*/
  ProcessPlanning(token);
  is_assure_run_once_.store(false);
  return 1;
}

void Planning::GetParkingLot(NodeBundle* const input) {
  const auto& ptr_parking_lot =
      std::static_pointer_cast<hz_Adsfi::AlgParkingLotOutArray>(
          input->GetOne("parking_lot"));
  int ret = 1;
  if (ptr_parking_lot != nullptr) {
    ret = Struct2ParkingLotOutPb(*ptr_parking_lot, location_.pose(),
                                 &parking_lot_array_);
  } else {
    AWARN << " can not get parking lot from topic parking_lot, set default "
             "data and report time out fault";
    hz_Adsfi::AlgParkingLotOutArray default_data;
    Struct2ParkingLotOutPb(default_data, location_.pose(), &parking_lot_array_);
  }
  CheckInputData(ret, 10500000, true, &parking_lot_array_);
}

TaPilotMode Planning::GetFctIn(
    NodeBundle* const input,
    TL::functionmanager::FunctionManagerIn* const fct_in) {
  if (fct_in == nullptr || input == nullptr) {
    return TaPilotMode::NNP;
  }
  if (FLAGS_enable_dreamview_to_planning_zmq) {
    fct_in->CopyFrom(zmq_receiver_->GetFunctionManagerIn());
  } else {
    const auto& ptr_fct_input =
        std::static_pointer_cast<hz_Adsfi::AlgMcu2EgoFrame>(
            input->GetOne("mcu_to_ego"));
    if (ptr_fct_input != nullptr) {
      Struct2FctInputPb(*ptr_fct_input, fct_in);
    } else {
      AWARN << " can not get fct from topic mcu_to_ego, set default data and "
               "report time out fault";
      hz_Adsfi::AlgMcu2EgoFrame default_data;
      Struct2FctInputPb(default_data, fct_in);
    }
    // if (fct_in->ta_pilot_mode() != TaPilotMode::AVP) {
    //   fct_in->set_ta_pilot_mode(TaPilotMode::NNP);
    // }
    CheckInputData(1, 10700000, true, fct_in);
  }
  return fct_in->ta_pilot_mode();
}

void Planning::GetLocation(NodeBundle* const input, const bool is_nnp_mode) {
  const std::string topic =
      is_nnp_mode ? "nnp_localization" : "hpp_localization";
  const auto& ptr_location =
      std::static_pointer_cast<hz_Adsfi::AlgLocation>(input->GetOne(topic));
  int ret = 1;
  if (ptr_location != nullptr) {
    ret = Struct2LocalizationPb(*ptr_location, &location_, is_nnp_mode);
  } else {
    AWARN << " can not get location from topic " << topic
          << " ,set default data and report time out fault";
    hz_Adsfi::AlgLocation default_data;
    Struct2LocalizationPb(default_data, &location_, is_nnp_mode);
  }
  CheckInputData(ret, 10200000, is_nnp_mode, &location_);
}

void Planning::GetFreeSpace(NodeBundle* const input, const bool is_nnp_mode) {
  const std::string topic =
      is_nnp_mode ? "nnp_fusion_freespace" : "hpp_freespace";
  const auto& ptr_free_space =
      std::static_pointer_cast<hz_Adsfi::AlgFreeSpaceOutArray>(
          input->GetOne(topic));
  int ret = 1;
  if (ptr_free_space != nullptr) {
    ret = Struct2FreeSpacePb(*ptr_free_space, &free_space_array_, location_);
  } else {
    AWARN << " can not get freespace from topic " << topic
          << " ,set default data and report time out fault";
    hz_Adsfi::AlgFreeSpaceOutArray default_data;
    Struct2FreeSpacePb(default_data, &free_space_array_, location_);
  }
  CheckInputData(ret, 10600000, is_nnp_mode, &free_space_array_);
}

void Planning::GetObstacle(NodeBundle* const input, const bool is_nnp_mode) {
  const std::string topic = is_nnp_mode ? "nnp_obj_fusion" : "hpp_obj_fusion";
  const auto& ptr_obs = std::static_pointer_cast<hz_Adsfi::AlgFusionOutArrayF>(
      input->GetOne(topic));
  int ret = 1;
  if (ptr_obs != nullptr) {
    ret = Struct2PerceptionObstaclesPb(*ptr_obs, location_.pose(), &obstacles_,
                                       is_nnp_mode);

  } else {
    AWARN << " can not get obstacle from topic " << topic
          << " ,set default data and report time out fault";
    hz_Adsfi::AlgFusionOutArrayF default_data;
    Struct2PerceptionObstaclesPb(default_data, location_.pose(), &obstacles_,
                                 is_nnp_mode);
  }
  CheckInputData(ret, 10400000, is_nnp_mode, &obstacles_);
  obstacles_.mutable_lane_marker()->CopyFrom(lane_markers_);
  if (is_nnp_mode) {
    obstacles_minieye_ = obstacles_;
  } else {
    obstacles_minieye_.Clear();
  }
}

void Planning::GetChassis(NodeBundle* const input) {
  const auto& ptr_chassis = std::static_pointer_cast<hz_Adsfi::AlgChassisInfo>(
      input->GetOne("chassis"));
  if (ptr_chassis != nullptr) {
    Struct2ChassisPb(*ptr_chassis, fct_in_, vehicle_param_, &chassis_);
  } else {
    AWARN << " can not get chassis from topic chassis,set default data and "
             "report time out fault";
    hz_Adsfi::AlgChassisInfo default_data;
    Struct2ChassisPb(default_data, fct_in_, vehicle_param_, &chassis_);
  }
  CheckInputData(1, 10100000, true, &chassis_);
  chassis_.mutable_vehicle_cfg()->CopyFrom(vehicle_cfg_);
  chassis_.mutable_warning_switch_mem()->CopyFrom(warning_switch_mem_);
}

void Planning::GetLaneArray(NodeBundle* const input, const bool is_nnp_mode) {
  std::shared_ptr<hz_Adsfi::AlgLaneDetectionOutArray> ptr_lane = nullptr;
  // std::string nnp_lane =
  //     FLAGS_use_TL_lane ? "TL_lane" : "nnp_fusion_lane";
  // std::string topic = is_nnp_mode ? nnp_lane : "hpp_lane";

  if (is_nnp_mode) {
    const auto& TL_lane =
        std::static_pointer_cast<hz_Adsfi::AlgLaneDetectionOutArray>(
            input->GetOne("TL_lane"));
    if (TL_lane != nullptr) {
      ptr_lane = TL_lane;
    } else {
      ptr_lane = std::static_pointer_cast<hz_Adsfi::AlgLaneDetectionOutArray>(
          input->GetOne("nnp_fusion_lane"));
    }
  } else {
    ptr_lane = std::static_pointer_cast<hz_Adsfi::AlgLaneDetectionOutArray>(
        input->GetOne("hpp_lane"));
  }

  int ret = 1;
  if (ptr_lane != nullptr) {
    ret = Struct2LaneMarkersPb(*ptr_lane, &lane_markers_);
  } else {
    AWARN << " can not get lane from topic  ,set default data and report time "
             "out fault";
    hz_Adsfi::AlgLaneDetectionOutArray default_data;
    Struct2LaneMarkersPb(default_data, &lane_markers_);
  }
  CheckInputData(ret, 10300000, is_nnp_mode, &lane_markers_);
  if (is_nnp_mode) {
    lane_markers_minieye_ = lane_markers_;
  } else {
    lane_markers_minieye_.Clear();
  }
}

void Planning::GetMbdDebugFromMcu(NodeBundle* const input) {
  const auto& ptr_mbd_debug =
      std::static_pointer_cast<hz_Adsfi::AlgMcuDebugFrame>(
          input->GetOne("mbd_debug"));
  if (ptr_mbd_debug != nullptr) {
    Struct2PbMbdDebug(*ptr_mbd_debug, &mbd_debug_);
    TL::common::util::FillHeader("mbd_debug", &mbd_debug_);
  } else {
    AWARN << " can not get mbd debug from topic mbd_debug";
    mbd_debug_.Clear();
  }
}

void Planning::GetAdasDataFromMcu(NodeBundle* input) {
  const auto& ptr_adas_data =
      std::static_pointer_cast<hz_Adsfi::AlgADAS_DataRecordFrame>(
          input->GetOne("adas_record"));
  if (ptr_adas_data != nullptr) {
    Struct2Pb_mcu_to_soc(*ptr_adas_data, &adas_data_);
    TL::common::util::FillHeader("adas_record", &adas_data_);
  } else {
    AWARN << " can not get mbd debug from topic adas_record";
    adas_data_.Clear();
  }
}

void Planning::GetAebInfo(NodeBundle* const input) {
  const auto& ptr_aeb_info =
      std::static_pointer_cast<hz_Adsfi::AlgAebToEgoFrame>(
          input->GetOne("aeb_to_ego"));
  if (ptr_aeb_info != nullptr) {
    fct_in_.mutable_fct_nnp_in()->mutable_aeb()->set_id(
        ptr_aeb_info->AEB_target_id);
    fct_in_.mutable_fct_nnp_in()->mutable_aeb()->set_state(
        (WarningLevel)(ptr_aeb_info->AEB_target_id != 0));
    fct_in_.mutable_fct_nnp_in()->mutable_fcw()->set_id(
        ptr_aeb_info->FCW_target_id);
    fct_in_.mutable_fct_nnp_in()->mutable_fcw()->set_state(
        (WarningLevel)ptr_aeb_info->FCW_state);
  } else {
    fct_in_.mutable_fct_nnp_in()->mutable_aeb()->Clear();
    fct_in_.mutable_fct_nnp_in()->mutable_fcw()->Clear();
    AWARN << " can not get aeb data from topic aeb_to_ego";
  }
}

void Planning::GetTsrData(NodeBundle* const input) {
  auto* tsr_info = fct_in_.mutable_fct_nnp_in()->mutable_tsr_info();
  const auto& ptr_tsr_info =
      std::static_pointer_cast<hz_Adsfi::AlgPerceptionInfoFrame>(
          input->GetOne("tsr"));
  if (ptr_tsr_info != nullptr) {
    tsr_info->set_valid(true);
    tsr_info->set_speed_limit_km(
        ptr_tsr_info->perception_info.tsr_info.spd_limit);
  } else {
    tsr_info->set_valid(false);
    tsr_info->set_speed_limit_km(0);
    AWARN << " can not get tsr data from topic tsr";
  }
}

void Planning::GetNnsLocation(NodeBundle* const input) {
  const auto& ptr_nns_loc =
      std::static_pointer_cast<hz_Adsfi::AlgHmiAvpLocFrame>(
          input->GetOne("nns_location"));
  if (ptr_nns_loc != nullptr) {
    if (Struct2NnsLocationPb(*ptr_nns_loc, &nns_location_) == 0) {
      AWARN << " nns location data error , has nan number";
    }
    if (!CheckPublishTimestamp(nns_location_.header().data_stamp())) {
      AWARN << " nns location time out ";
    }
  } else {
    AWARN << " can not get nns location data from topic nns_location";
  }
}

void Planning::GetNnsRoute(NodeBundle* const input) {
  const auto& ptr_nns_route =
      std::static_pointer_cast<hz_Adsfi::AlgNNSRouteInfo>(
          input->GetOne("nns_route"));
  if (ptr_nns_route != nullptr) {
    if (Struct2NnsRoutePb(*ptr_nns_route, &nns_route_) == 0) {
      AWARN << " nns route data error , has nan number";
    }
    if (!CheckPublishTimestamp(nns_route_.header().data_stamp())) {
      AWARN << " nns route time out ";
    }
  } else {
    AWARN << " can not get nns route data from topic nns_route";
  }
}

void Planning::GetMcu7kData(NodeBundle* const input) {
  const auto& mcu_7k_data =
      std::static_pointer_cast<hz_Adsfi::AlgAdasStateFrame>(
          input->GetOne("mcu_adas_state"));
  if (mcu_7k_data != nullptr) {
    TL::common::util::FillHeader("adas_someip_debug", &adas_7k_data_);
    Struct2PbAdasSomeip(*mcu_7k_data, &adas_7k_data_, vehicle_param_,
                        &chassis_);
    UpdateChassisBy7kData(*mcu_7k_data, &chassis_);
    auto* mutable_cdcs_speed_limit = fct_in_.mutable_fct_nnp_in()
                                         ->mutable_cdcs_info()
                                         ->mutable_cdcs_speed_limit();

    mutable_cdcs_speed_limit->set_camera_speed_limit_km(
        adas_7k_data_.cdcs18_cameraspeedlimit());
    mutable_cdcs_speed_limit->set_road_speed_limit(
        adas_7k_data_.cdcs18_roadspeedlimit());
    mutable_cdcs_speed_limit->set_camera_distance(
        adas_7k_data_.cdcs18_cameradistance());
  } else {
    AWARN << " can not get GetMcu7kData from topic mcu_adas_state";
  }
}

void Planning::GetTrafficLightAndRouting(NodeBundle* const input) {
  if (FLAGS_use_dv_traffic_light && zmq_receiver_ != nullptr) {
    traffic_light_.CopyFrom(zmq_receiver_->GetTransportElement());
  } else {
    const auto& traffic_light =
        std::static_pointer_cast<AlgDetectionOutArrayCameraF>(
            input->GetOne("traffic_light"));
    if (traffic_light != nullptr) {
      if (absl::StrContains(traffic_light->header.frameID, "fusion_camera")) {
        // minieye 数据，需要过滤
        traffic_light_ = TL_traffic_light_;
      } else {
        Struct2pbTrafficLight(*traffic_light, &origin_traffic_lights_);
        if (traffic_light_.header().frame_id().empty()) {
          TL::common::util::FillHeader("TL_traffic_light",
                                          &traffic_light_);
        }
        TL_traffic_light_ = traffic_light_;
      }

    } else {
      AWARN << " can not get traffic light from topic traffic_light";
    }
  }
  if (FLAGS_use_dv_routing && zmq_receiver_ != nullptr) {
    routing_request_.CopyFrom(zmq_receiver_->GetRoutingRequest());
  } else {
    routing_request_.Clear();
  }
}

int32_t Planning::SelfSimulatorFunc(NodeBundle* const input) {
  if (is_pub_thread_stop_) {
    return -1;
  }
  GetFctIn(input, &fct_in_);
  GetChassis(input);
  auto local_view = std::make_shared<LocalView>();
  local_view->SetChassisPtr(std::make_shared<Chassis>(chassis_));
  local_view->SetFunctionManagerInPtr(
      std::make_shared<TL::functionmanager::FunctionManagerIn>(fct_in_));
  local_view->SetPadMessagePtr(
      std::make_shared<TL::planning::PadMessage>(pad_msg_));
  local_view->SetLaneMarkersPtr(std::make_shared<LaneMarkers>(LaneMarkers()));
  local_view->SetParkingLotOutArrayPtr(
      std::make_shared<TL::perception::ParkingLotOutArray>(
          parking_lot_array_));
  local_view->SetMbdDebugFromMCUPtr(
      std::make_shared<TL::control::MbdDebugFromMCU>(mbd_debug_));
  regular_planning_base_->RunOnce(local_view);
  return 1;
}

int32_t Planning::AlgInit() {
  PauseTrigger(avp_topic_);
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
  if (FLAGS_enable_dreamview_to_planning_zmq || FLAGS_use_dv_traffic_light ||
      FLAGS_use_dv_routing) {
    zmq_receiver_ = std::make_unique<ZMQ2Dreamview>();
    zmq_receiver_->Init();
    zmq_receiver_->Start();
  }
  std::string has_trigger_config{};
  if (HzCfgClient::Instance() != nullptr) {
    HzCfgClient::Instance()->GetParam<std::string>(
        "TL", "planning_trigger_config", has_trigger_config);
  }
  std::string trigger_config_file = FLAGS_default_trigger_config_file;
  if (!has_trigger_config.empty()) {
    trigger_config_file = FLAGS_trigger_config_file;
  }
  bool load_trigger_config =
      GetProtoFromFile(trigger_config_file, &trigger_config_);
  if (!load_trigger_config) {
    AERROR << "Failed to load obs follow time config file ";
  }
  TL::planning::WarningCfg::Init();
  TL::planning::WarningCfg::GetVehicleCfg(&vehicle_cfg_);
  TL::planning::WarningCfg::GetWarningSwitchMem(&warning_switch_mem_);
  collect_trigger_manager_ = std::make_unique<CollectTriggerManager>();
  metric_collect_ = std::make_unique<MetricCollect>();
  writer_future_ = std::async(&Planning::PubTrajectoryThread, this);
  auto planning_trigger_config =
      [&](const std::string& domain, const std::string& key,
          const std::string& value, const uint8_t& paramType) {
        if ((domain == "TL") && (key == "planning_trigger_config")) {
          has_new_trigger_config_ = true;
        }
      };
  if (HzCfgClient::Instance() != nullptr) {
    HzCfgClient::Instance()->MonitorParamV2("TL", "planning_trigger_config",
                                            planning_trigger_config);
  }
  return 1;
}

int32_t Planning::GetEhpThread(NodeBundle* const input,
                               const hz_Adsfi::ProfileToken& token) {
  const auto ptr_map_message =
      std::static_pointer_cast<hz_Adsfi::AlgMapMessage>(
          input->GetOne("map_msg"));
  if (ptr_map_message != nullptr) {
    auto ehp = std::make_shared<TL::ehp::EHP>();
    Struct2EHPPb(*ptr_map_message, ehp.get());
    regular_planning_base_->UpdateEHPData(ehp);
  }
  return 1;
}

void Planning::GetTriggerConfig() {
  if (has_new_trigger_config_) {
    GetProtoFromFile(FLAGS_trigger_config_file, &trigger_config_);
    has_new_trigger_config_ = false;
  }
  if (trigger_config_.has_nnp_config()) {
    fct_in_.mutable_trigger_config()->CopyFrom(trigger_config_);
    trigger_config_.Clear();
  } else {
    fct_in_.clear_trigger_config();
  }
}

void Planning::ProcessPlanning(const hz_Adsfi::ProfileToken& token) {
  if (is_pub_thread_stop_) {
    return;
  }

  if (FLAGS_enable_planning_self_simulator) {
    return;
  }
  auto local_view = std::make_shared<MiddleWareLocalView>();
  local_view->SetChassisPtr(std::make_shared<Chassis>(chassis_));

  local_view->SetLocalizationPtr(std::make_shared<Localization>(location_));
  local_view->SetLaneMarkersPtr(std::make_shared<LaneMarkers>(lane_markers_));
  local_view->SetLaneMarkersMinieyePtr(
      std::make_shared<LaneMarkers>(lane_markers_minieye_));
  local_view->SetPerceptionObstaclesPtr(
      std::make_shared<TL::perception::PerceptionObstacles>(obstacles_));
  local_view->SetPerceptionObstaclesMinieyePtr(
      std::make_shared<TL::perception::PerceptionObstacles>(
          obstacles_minieye_));
  local_view->SetParkingLotOutArrayPtr(
      std::make_shared<TL::perception::ParkingLotOutArray>(
          parking_lot_array_));
  local_view->SetFreeSpaceOutArrayPtr(
      std::make_shared<TL::perception::FreeSpaceOutArray>(
          free_space_array_));
  local_view->SetFunctionManagerInPtr(
      std::make_shared<TL::functionmanager::FunctionManagerIn>(fct_in_));
  local_view->SetPadMessagePtr(
      std::make_shared<TL::planning::PadMessage>(pad_msg_));
  local_view->SetMbdDebugFromMCUPtr(
      std::make_shared<TL::control::MbdDebugFromMCU>(mbd_debug_));
  local_view->Setmcu_to_soc_DebugDataPtr(
      std::make_shared<TL::common::mcu_to_soc_DebugData>(adas_data_));
  local_view->SetNNSLocFramePtr(
      std::make_shared<TL::hmi::NNSLocFrame>(nns_location_));
  local_view->SetNNSRouteInfoPtr(
      std::make_shared<TL::hmi::NNSRouteInfo>(nns_route_));
  local_view->SetAdasSomeipFromMCUPtr(
      std::make_shared<TL::control::AdasSomeipFromMCU>(adas_7k_data_));
  if (traffic_light_.has_header()) {
    local_view->SetTransportElementPtr(
        std::make_shared<TransportElement>(traffic_light_));
  }

  if (routing_request_.has_header()) {
    local_view->SetRoutingRequestPtr(
        std::make_shared<TL::routing::RoutingRequest>(routing_request_));
  }
  local_view->SetToken(token);
  // planning入口
  guard_planning_base_->RunOnce(local_view);
  regular_planning_base_->RunOnce(local_view);
}

void Planning::PubTrajectory(const ADCTrajectory& adc_trajectory,
                             const hz_Adsfi::ProfileToken& token) {
  auto ptr_alg_adc_trajectory = std::make_shared<hz_Adsfi::AlgEgoTrajectory>();
  trajectory_check_ok_ =
      Pb2StructTrajectory(adc_trajectory, ptr_alg_adc_trajectory);
  Send(token, "ego_trajectory", ptr_alg_adc_trajectory);
}

void Planning::PubTrajectoryThread() {
  std::shared_ptr<LocalView> last_local_view(nullptr);
  while (!is_pub_thread_stop_) {
    std::shared_ptr<LocalView> local_view(nullptr);
    if (!publish_queue_->WaitDequeue(&local_view) || local_view == nullptr) {
      continue;
    }
    recv_queue_timestamp_ = Clock::NowInSeconds();
    // 遇到新的一帧，发布轨迹
    if (last_local_view != local_view) {
      if (!ProcessGuardTrajectory(local_view)) {
        ProcessRegularTrajectory(local_view);
      }
      metric_collect_->UpdateMetricData(local_view);
      last_local_view = local_view;
    }

    // 遇到on_lane_planning的轨迹，更新last_regular_adc_trajectory_，发布调试信息
    if (local_view->HasADCTrajectory()) {
      last_regular_adc_trajectory_ = local_view->GetADCTrajectory();
      PublishDebug(local_view);
    }
  }
}

void Planning::PubEthHmi(const ADCTrajectory& adc_trajectory,
                         const hz_Adsfi::ProfileToken& token) {
  const auto& ta_pilot_mode =
      adc_trajectory.function_manager_in().ta_pilot_mode();
  auto ptr_planning_dec = std::make_shared<hz_Adsfi::AlgPlanningDecisionInfo>();
  auto message_name = "";
  if (ta_pilot_mode == TaPilotMode::AVP) {
    message_name = "ego_planning_dec_avp";
  } else {
    message_name = "ego_planning_dec_nnp";
  }
  StructPlanningDecisionInfoForHmi(adc_trajectory, ptr_planning_dec.get());
  Send(token, message_name, ptr_planning_dec);
}

void Planning::PubFctOut2MCU(const ADCTrajectory& adc_trajectory,
                             const hz_Adsfi::ProfileToken& token) {
  auto ptr_fct_out = std::make_shared<hz_Adsfi::AlgEgo2McuFrame>();
  Pb2SrtcutFctOut(adc_trajectory.function_manager_out(), ptr_fct_out.get());
  Send(token, "ego_to_mcu", ptr_fct_out);
}

void Planning::PubWarning(const ADCTrajectory& adc_trajectory,
                          const hz_Adsfi::ProfileToken& token) {
  auto ptr_hmi_out = std::make_shared<hz_Adsfi::AlgEgoHmiFrame>();
  Pb2StructMcuHmi(adc_trajectory.warning_status(), adc_trajectory.avp_to_hmi(),
                  ptr_hmi_out.get());
  TL::planning::WarningCfg::SetWarningSwitchMem(
      adc_trajectory.warning_status(), &last_warning_state_);
  Send(token, "warning_info", ptr_hmi_out);
}

void Planning::ProcessWarningAndEthHmi(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<ADCTrajectory>& ptr_adc_trajectory) {
  collect_trigger_manager_->GetWarningFaultStatus(
      ptr_adc_trajectory->mutable_debug()->mutable_monitor_fault_debug());
  regular_planning_base_->ProcessOutputData(local_view, ptr_adc_trajectory);
  collect_trigger_manager_->PubEventDataToTrigger(
      ptr_adc_trajectory->event_trigger());

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
    case DriveMode::AVP_CRUSING:
      break;
    default:
      is_state_ready_to_report = false;
  }
  collect_trigger_manager_->PubFaultDataToTrigger(
      ptr_adc_trajectory->planning_fault(), is_state_ready_to_report);
}

void Planning::AlgRelease() {
  publish_queue_->BreakAllWait();
  is_pub_thread_stop_ = true;
  is_assure_run_once_.store(false);
  writer_future_.get();
  TL::planning::WarningCfg ::Release();
}

bool Planning::ProcessGuardTrajectory(
    const std::shared_ptr<LocalView>& current_local_view) {
  if (current_local_view == nullptr ||
      !current_local_view->HasADCTrajectoryGuard()) {
    return false;
  }

  auto guard_adctrajectory = *current_local_view->GetADCTrajectoryGuard();
  if (!guard_adctrajectory.has_header() ||
      guard_adctrajectory.trajectory_point().empty()) {
    return false;
  }

  if (last_regular_adc_trajectory_ != nullptr) {
    guard_adctrajectory.set_gear(last_regular_adc_trajectory_->gear());
  }
  const auto start_timestamp = Clock::NowInSeconds();
  const auto token = GetToken(current_local_view);
  PubTrajectory(guard_adctrajectory, token);
  UpdateFctOutByTrajectory(&guard_adctrajectory);
  PubFctOut2MCU(guard_adctrajectory, token);
  auto* pub_status =
      guard_adctrajectory.mutable_latency_stats()->add_task_stats();
  pub_status->set_name("process_guard_trajectory");
  pub_status->set_time_ms((Clock::NowInSeconds() - start_timestamp) * 1000);
  if (current_local_view->HasMbdDebugFromMCU() &&
      current_local_view->GetMbdDebugFromMCU()->has_header()) {
    auto* total_planning_status =
        guard_adctrajectory.mutable_latency_stats()->add_task_stats();
    total_planning_status->set_name("total_guard_planning_time");
    total_planning_status->set_time_ms(
        (Clock::NowInSeconds() -
         current_local_view->GetMbdDebugFromMCU()->header().data_stamp()) *
        1000);
  }

  return true;
}

bool Planning::ProcessRegularTrajectory(
    const std::shared_ptr<LocalView>& retrived_local_view) {
  if (!retrived_local_view || !retrived_local_view->HasADCTrajectory()) {
    return false;
  }

  auto token = GetToken(retrived_local_view);
  if (retrived_local_view->GetADCTrajectory()->has_header()) {
    const auto start_timestamp = Clock::NowInSeconds();
    PubTrajectory(*retrived_local_view->GetADCTrajectory(), token);

    // 为了避免copy adc_trajectory, 且为了避免多线程数据竞争无法在 local_view 加 mutable 接口
    // 此为设计失误，不应该把所有东西都塞在 trajectory 里，造成很多数据处理时发生耦合
    auto ptr_adc_trajectroy = std::const_pointer_cast<ADCTrajectory>(
        retrived_local_view->GetADCTrajectory());
    if (ptr_adc_trajectroy == nullptr) {
      return false;
    }
    ptr_adc_trajectroy->mutable_function_manager_out()
        ->mutable_nnp_fct_out()
        ->mutable_localview_time()
        ->set_end_time(recv_queue_timestamp_);
    ProcessWarningAndEthHmi(retrived_local_view, ptr_adc_trajectroy);
    PubFctOut2MCU(*ptr_adc_trajectroy, token);
    PubWarning(*ptr_adc_trajectroy, token);
    PubEthHmi(*ptr_adc_trajectroy, token);
    if (retrived_local_view->HasFunctionManagerIn() &&
        retrived_local_view->GetFunctionManagerIn()->ta_pilot_mode() ==
            TL::functionmanager::AVP &&
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
    if (retrived_local_view->HasMbdDebugFromMCU() &&
        retrived_local_view->GetMbdDebugFromMCU()->has_header()) {
      auto* total_planning_status =
          ptr_adc_trajectroy->mutable_latency_stats()->add_task_stats();
      total_planning_status->set_name("total_regular_planning_time");
      total_planning_status->set_time_ms(
          (Clock::NowInSeconds() -
           retrived_local_view->GetMbdDebugFromMCU()->header().data_stamp()) *
          1000);
    }
  }
  return true;
}

void Planning::PublishDebug(
    const std::shared_ptr<LocalView>& retrived_local_view) {
  if (!FLAGS_enable_viz) {
    return;
  }
  NodeBundle output;
  const auto& planning_debug = std::make_shared<hz_Adsfi::DebugPlanningFrame>();

  if (retrived_local_view->HasADCTrajectoryGuard()) {
    auto guard_adctrajectory = *retrived_local_view->GetADCTrajectoryGuard();
    guard_adctrajectory.mutable_debug()->mutable_planning_data()->CopyFrom(
        last_regular_adc_trajectory_->debug().planning_data());
    guard_adctrajectory.mutable_header()->mutable_status()->CopyFrom(
        last_regular_adc_trajectory_->header().status());
    guard_adctrajectory.mutable_avp_to_hmi()->CopyFrom(
        last_regular_adc_trajectory_->avp_to_hmi());

    SerializeProtoToString(retrived_local_view, guard_adctrajectory,
                           planning_debug, zmq_sender_);
    Send(GetToken(retrived_local_view), "planning_debug", planning_debug);
  } else if (retrived_local_view->HasADCTrajectory()) {
    SerializeProtoToString(retrived_local_view,
                           *retrived_local_view->GetADCTrajectory(),
                           planning_debug, zmq_sender_);
    Send(GetToken(retrived_local_view), "planning_debug", planning_debug);
  }
}

hz_Adsfi::ProfileToken Planning::GetToken(
    const std::shared_ptr<LocalView>& retrived_local_view) {
  const auto mdc_local_view =
      std::dynamic_pointer_cast<MiddleWareLocalView>(retrived_local_view);
  if (mdc_local_view != nullptr && !FLAGS_enable_planning_self_simulator) {
    return mdc_local_view->GetToken();
  } else {
    return hz_Adsfi::ProfileToken();
  }
}

void Planning::Send(const hz_Adsfi::ProfileToken& token,
                    const std::string& tpoic,
                    const std::shared_ptr<hz_Adsfi::AlgDataBase>& data) {
  NodeBundle output;
  output.Add(tpoic, data);
  if (FLAGS_enable_planning_self_simulator) {
    SendOutput(&output);
  } else {
    SendOutput(&output, token);
  }
}

void Planning::UpdateFctOutByTrajectory(
    ADCTrajectory* const ptr_adc_trajectory) {
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
  } else if (function_topic == nnp_topic_ &&
             ta_polit_mode == TaPilotMode::AVP) {
    PauseTrigger(nnp_topic_);
    ResumeTrigger(avp_topic_);
    AINFO << " mode changed to avp, ta polit mode: " << ta_polit_mode;
    return true;
  }
  return false;
}

void Planning::RecordCpuAndMemInfo(ADCTrajectory* const ptr_adc_trajectory) {
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

void Planning::ClearDataByFctMode(const bool is_nnp_mode) {
  if (is_nnp_mode) {
    free_space_array_.Clear();
    parking_lot_array_.Clear();
  }
}
}  // namespace TL
