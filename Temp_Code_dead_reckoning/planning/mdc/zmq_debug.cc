/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning debug
 */
// NOLINTBEGIN
#include "planning/mdc/zmq_debug.h"

#include "common/util/message_util.h"
#include "common/util/util.h"
#include "proto/common/header.pb.h"
#include "proto/hmi/nns_router.pb.h"

namespace TL {
namespace planning {
namespace ZmqDebug {

void SerializeProtoToString(
    const std::shared_ptr<LocalView>& local_view,
    const ADCTrajectory& adc_trajectory,
    std::vector<std::string>* const msg_vec,
    hz_Adsfi::DebugPlanningFrame* const planning_debug,
    const std::unique_ptr<apollo::common::ZMQSender>& zmq_sender,
    NodeBundle* const output) {
  msg_vec->clear();
  // adc_trajectory
  std::string one = "";

  adc_trajectory.SerializeToString(&one);
  planning_debug->msg_1 = one;

  msg_vec->push_back(std::move(one));

  // Localization
  one = "";
  if (local_view->HasLocalizationEstimate()) {
    local_view->GetLocalizationEstimate()->SerializeToString(&one);
  }
  planning_debug->msg_2 = one;
  msg_vec->push_back(std::move(one));
  // Chassis
  one = "";
  if (local_view->HasChassis()) {
    local_view->GetChassis()->SerializeToString(&one);
  }
  planning_debug->msg_3 = one;
  msg_vec->push_back(std::move(one));
  // Prediction
  one = "";
  if (local_view->HasPredictionObstacles()) {
    local_view->GetPredictionObstacles()->SerializeToString(&one);
  }
  planning_debug->msg_4 = one;
  msg_vec->push_back(std::move(one));

  // Perception
  one = "";
  if ((local_view->HasPerceptionObstacles())) {
    local_view->GetPerceptionObstacles()->SerializeToString(&one);
  }
  planning_debug->msg_5 = one;
  msg_vec->push_back(std::move(one));

  // ParkingSlots
  one = "";
  if ((local_view->HasParkingLotOutArray())) {
    local_view->GetParkingLotOutArray()->SerializeToString(&one);
  }
  planning_debug->msg_6 = one;
  msg_vec->push_back(std::move(one));

  // send map msg
  // send routing
  one = "";
  if (local_view->HasValidRoutingResponseHeader()) {
    auto current_routing = std::make_shared<apollo::routing::RoutingResponse>();
    current_routing->CopyFrom(*local_view->GetRoutingResponse());
    if (local_view->HasMapMsg()) {
      auto* map_string = current_routing->mutable_measurement()->add_info();
      if (local_view->GetMapMsg()->hdmap().has_header() &&
          local_view->GetMapMsg()->hdmap().lane_size() == 0) {
        local_view->GetHDMapPtr()->GetMapString(map_string);
      } else {
        std::string map_str;
        local_view->GetMapMsg()->SerializeToString(map_string);
      }
      if (adc_trajectory.has_nnp_hmi_output()) {
        const auto& nnp_hmi_output = adc_trajectory.nnp_hmi_output();
        if (nnp_hmi_output.has_cross_info_enu()) {
          auto* cross_info_enu_str =
              current_routing->mutable_measurement()->add_info();
          nnp_hmi_output.cross_info_enu().SerializeToString(cross_info_enu_str);
        }
        if (nnp_hmi_output.has_cross_info_vrf()) {
          auto* cross_info_vrf_str =
              current_routing->mutable_measurement()->add_info();
          nnp_hmi_output.cross_info_vrf().SerializeToString(cross_info_vrf_str);
        }
      }
      current_routing->SerializeToString(&one);
    }
  }

  planning_debug->msg_7 = one;
  msg_vec->push_back(std::move(one));

  // map state data
  one = "";
  if ((local_view->HasMapStateData())) {
    local_view->GetMapStateData()->SerializeToString(&one);
  }
  planning_debug->msg_8 = one;
  msg_vec->push_back(std::move(one));

  // freespaceInfo
  one = "";
  if ((local_view->HasFreeSpaceOutArray())) {
    local_view->GetFreeSpaceOutArray()->SerializeToString(&one);
  }
  planning_debug->msg_9 = one;
  msg_vec->push_back(std::move(one));

  // mbd debug
  one = "";
  if ((local_view->HasMbdDebugFromMCU())) {
    local_view->GetMbdDebugFromMCU()->SerializeToString(&one);
  }
  planning_debug->msg_10 = one;
  msg_vec->push_back(std::move(one));

  one = "";
  if ((local_view->HasFunctionManagerIn())) {
    local_view->GetFunctionManagerIn()->SerializeToString(&one);
  }
  planning_debug->msg_11 = one;
  msg_vec->push_back(std::move(one));

  one = "";
  if ((local_view->HasLanemarkersLaneLine())) {
    local_view->GetLanemarkersLaneLine()->SerializeToString(&one);
  }
  planning_debug->msg_12 = one;
  msg_vec->push_back(std::move(one));

  one = "";
  if ((local_view->HasAdasSomeipFromMCU())) {
    local_view->GetAdasSomeipFromMCU()->SerializeToString(&one);
  }
  planning_debug->msg_13 = one;
  msg_vec->push_back(std::move(one));

  one = "";
  if ((local_view->Hasmcu_to_soc_DebugData())) {
    local_view->Getmcu_to_soc_DebugData()->SerializeToString(&one);
  }
  planning_debug->msg_14 = one;
  msg_vec->push_back(std::move(one));

  one = "";
  apollo::hmi::NNSDebug nns_debug;
  if (local_view->HasNNSLocFrame()) {
    nns_debug.mutable_nns_location()->CopyFrom(*local_view->GetNNSLocFrame());
  }
  if (local_view->HasNNSRouteInfo()) {
    nns_debug.mutable_nns_route()->CopyFrom(*local_view->GetNNSRouteInfo());
  }
  nns_debug.SerializeToString(&one);
  planning_debug->msg_15 = one;
  msg_vec->push_back(std::move(one));

  // one = "";
  // if (local_view->HasTrafficLightDetection()) {
  //   local_view->GetTrafficLightDetection()->SerializeToString(&one);
  //   msg_vec->push_back(std::move(one));
  // }

  if (zmq_sender != nullptr) {
    zmq_sender->Process(*msg_vec);
  }
  if (output != nullptr) {
    auto ptr_debug =
        std::make_shared<hz_Adsfi::DebugPlanningFrame>(*planning_debug);
    output->Add("planning_debug", ptr_debug);
  }
}

}  // namespace ZmqDebug
}  // namespace planning
}  // namespace TL

// NOLINTEND
