/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/trigger/fault_collect.h"

#include <cstdint>
#include <memory>

#include "common/file/file.h"
#include "common/file/log.h"
#include "glog/logging.h"
#include "planning/common/planning_gflags.h"

#include "proto/common/error_code.pb.h"
#include "proto/planning/trigger.pb.h"

namespace TL {
namespace planning {

using TL::common::ErrorCode;

FaultCollect::FaultCollect() {
  auto planning_fm_error_code_map_file = FLAGS_planning_fm_error_code_map_file;
#ifdef ISORIN
  planning_fm_error_code_map_file = FLAGS_orin_planning_fm_error_code_map_file;
#endif
  ACHECK(TL::common::GetProtoFromFile(planning_fm_error_code_map_file,
                                         &planning_error_code_map_))
      << "failed to load planning error code map file "
      << planning_fm_error_code_map_file;
}

// 诊断输入数据
void FaultCollect::ProcessFaultCollect(
    const std::shared_ptr<LocalView>& local_view,
    ADCTrajectory* const ptr_trajectory_pb) {
  CHECK_NOTNULL(local_view);
  CHECK_NOTNULL(ptr_trajectory_pb);
  ptr_trajectory_pb_ = ptr_trajectory_pb;
  ProcessInputData(local_view);
}

void FaultCollect::ProcessInputData(  // NOLINT
    const std::shared_ptr<LocalView>& local_view) {
  if (local_view->HasPredictionObstacles()) {
    const auto& prediction = local_view->GetPredictionObstacles();
    if (prediction->has_header() && prediction->header().has_status() &&
        prediction->header().status().has_error_code()) {
      const auto& error_code = prediction->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasPerceptionObstacles()) {
    const auto& perception = local_view->GetPerceptionObstacles();
    if (perception->has_header() && perception->header().has_status() &&
        perception->header().status().has_error_code()) {
      const auto& error_code = perception->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasPerceptionObstaclesMinieye()) {
    const auto& perception = local_view->GetPerceptionObstaclesMinieye();
    if (perception->has_header() && perception->header().has_status() &&
        perception->header().status().has_error_code()) {
      const auto& error_code = perception->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasChassis()) {
    const auto& chassis = local_view->GetChassis();
    if (chassis->has_header() && chassis->header().has_status() &&
        chassis->header().status().has_error_code()) {
      const auto& error_code = chassis->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasLocalization()) {
    const auto& location = local_view->GetLocalization();
    if (location->has_header() && location->header().has_status() &&
        location->header().status().has_error_code()) {
      const auto& error_code = location->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasTransportElement()) {
    const auto& transport_element = local_view->GetTransportElement();
    if (transport_element->has_header() &&
        transport_element->header().has_status() &&
        transport_element->header().status().has_error_code()) {
      const auto& error_code =
          transport_element->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasRoutingResponse()) {
    const auto& routing_response = local_view->GetRoutingResponse();
    if (routing_response->has_header() &&
        routing_response->header().has_status() &&
        routing_response->header().status().has_error_code()) {
      const auto& error_code = routing_response->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasLaneMarkers()) {
    const auto& lane_markers = local_view->GetLaneMarkers();
    if (lane_markers->has_header() && lane_markers->header().has_status() &&
        lane_markers->header().status().has_error_code()) {
      const auto& error_code = lane_markers->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasLaneMarkersMinieye()) {
    const auto& lane_markers = local_view->GetLaneMarkersMinieye();
    if (lane_markers->has_header() && lane_markers->header().has_status() &&
        lane_markers->header().status().has_error_code()) {
      const auto& error_code = lane_markers->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasVehicleState()) {
    const auto& vehicle = local_view->GetVehicleState();
    if (vehicle->has_header() && vehicle->header().has_status() &&
        vehicle->header().status().has_error_code()) {
      const auto& error_code = vehicle->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasParkingLotOutArray()) {
    const auto& parking_lot = local_view->GetParkingLotOutArray();
    if (parking_lot->has_header() && parking_lot->header().has_status() &&
        parking_lot->header().status().has_error_code()) {
      const auto& error_code = parking_lot->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasFreeSpaceOutArray()) {
    const auto& freespace = local_view->GetFreeSpaceOutArray();
    if (freespace->has_header() && freespace->header().has_status() &&
        freespace->header().status().has_error_code()) {
      const auto& error_code = freespace->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasADCTrajectory()) {
    const auto& trajectory = local_view->GetADCTrajectory();
    if (trajectory->has_header() && trajectory->header().has_status() &&
        trajectory->header().status().has_error_code()) {
      const auto& error_code = trajectory->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasFunctionManagerIn()) {
    const auto& function_manager = local_view->GetFunctionManagerIn();
    if (function_manager->has_header() &&
        function_manager->header().has_status() &&
        function_manager->header().status().has_error_code()) {
      const auto& error_code = function_manager->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasLanemarkersLaneLine()) {
    const auto& lane_markers = local_view->GetLanemarkersLaneLine();
    if (lane_markers->has_header() && lane_markers->header().has_status() &&
        lane_markers->header().status().has_error_code()) {
      const auto& error_code = lane_markers->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
  if (local_view->HasWithoutLaneFollow()) {
    const auto& without_lane_follow = local_view->GetWithoutLaneFollow();
    if (without_lane_follow->has_header() &&
        without_lane_follow->header().has_status() &&
        without_lane_follow->header().status().has_error_code()) {
      const auto& error_code =
          without_lane_follow->header().status().error_code();
      AddFaultDataType(error_code);
    }
  }
}

void FaultCollect::AddFaultDataType(const ErrorCode& error_code) {
  const auto key = static_cast<int32_t>(error_code);
  if (planning_error_code_map_.planning_error_code_map().count(key) > 0) {
    const auto fault_report = static_cast<uint32_t>(
        planning_error_code_map_.planning_error_code_map().at(key));
    if (fault_report > 0) {
      auto* planning_fault = ptr_trajectory_pb_->mutable_planning_fault();
      planning_fault->add_fault_info()->set_type(fault_report);
      ADEBUG << "planning internal error code:" << key;
    }
  }
}

}  // namespace planning
}  // namespace TL
