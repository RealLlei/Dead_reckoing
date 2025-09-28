/******************************************************************************
 * Copyright 2023 The TL Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning/trigger/orin_trigger_manager.h"
#include <sys/types.h>
#include <memory>
#include <string>
#include <utility>

#include "common/configs/config_gflags.h"
#include "common/file/log.h"

namespace TL {
namespace planning {
// NOLINTBEGIN
static std::unordered_map<std::string, uint8> fault_clusters_mapping_{
    {"platform", 0xFF},
    {"dr", 0x60},
    {"local-location", 0x60},
    {"global-location", 0x60},
    {"fusion-map", 0x61},
    {"local-map", 0x60},
    {"fusion-obj", 0x60},
    {"fusion-parkinglot", 0xE0},
    {"uss-obstacle", 0x60},
    {"freespace", 0x60},
    {"hpp-location", 0xE0},
    {"slam", 0xE0},
    {"imu", 0x39},
    {"chassis", 0x39},
    {"flm", 0x39},
    {"flc", 0x39},
    {"frc", 0x39},
    {"frm", 0x39},
    {"rlm", 0x39},
    {"rlc", 0x39},
    {"rrc", 0x39},
    {"rrm", 0x39},
    {"sfr", 0x39},
    {"sfl", 0x39},
    {"srr", 0x39},
    {"srl", 0x39},
    {"front-fisheye-camera", 0x39},
    {"left-fisheye-camera", 0x39},
    {"right-fisheye-camera", 0x39},
    {"rear-fisheye-camera", 0x39}};
#ifndef ISX86
using TL::netaos::dc::DcResultCode;
#endif
OrinTriggerManager::OrinTriggerManager() {
  warning_fault_process_ = std::make_unique<WarningFaultProcess>();
#ifndef ISX86
  dc_client_ = std::make_unique<DcClient>();
  phm_client_ = std::make_unique<PHMClient>();
  dc_client_->Init("planning_trigger", 100);
#endif
}

OrinTriggerManager::~OrinTriggerManager() {  // NOLINT
#ifndef ISX86
  dc_client_->DeInit();
  if (phm_client_is_success_) {
    phm_client_->Deinit();
  }
#endif
}

void OrinTriggerManager::PubEventDataToTrigger(  // NOLINT
    const EventTrigger& event_trigger) const {
  if (!FLAGS_enable_event_collect) {
    return;
  }
  for (const auto& event_info : event_trigger.event_info()) {
    if (event_info.type() != 0U) {
#ifndef ISX86
      const auto type =
          dc_client_->CollectTrigger(static_cast<uint32_t>(event_info.type()));
      if (type != DcResultCode::DC_OK) {
        ADEBUG << "event Trigger Fail id " << int(type);
      }
#endif
    }
  }
}

void OrinTriggerManager::GetWarningFaultStatus(
    TL::soc::MonitorFaultDebug* const warning_switch_or_fault) {
  auto warning_fault = warning_fault_process_->GetWarningFault();
  *warning_switch_or_fault = std::move(warning_fault);
}

void OrinTriggerManager::InitFault() {  // NOLINT
  if (!FLAGS_enable_fault_collect) {
    return;
  }
#ifndef ISX86
  auto fault_cluster_callback =
      [this](const TL::netaos::phm::ReceiveFault_t& fault_cluster_data) {
        for (const auto& cluster : fault_cluster_data.faultCluster) {
          if (cluster.clusterName.empty()) {
            continue;
          }
          // orin聚类故障，在本上电周期，都是认为可以恢复的
          if (fault_clusters_mapping_.count(cluster.clusterName) > 0 &&
              (cluster.clusterValue &
               fault_clusters_mapping_.at(cluster.clusterName)) > 0x00) {
            monitor_fault_clusters_[cluster.clusterName] = cluster.clusterValue;
          } else if (monitor_fault_clusters_.count(cluster.clusterName) > 0) {
            monitor_fault_clusters_.erase(cluster.clusterName);
          }
        }
        const auto fault_event_id = static_cast<int32_t>(
            fault_cluster_data.faultId * 100 + fault_cluster_data.faultObj);
        // record the origin fault id only for debugging
        if (fault_cluster_data.faultStatus > 0) {
          monitor_fault_events_.insert(fault_event_id);
        } else if (monitor_fault_events_.find(fault_event_id) !=
                   monitor_fault_events_.end()) {
          monitor_fault_events_.erase(fault_event_id);
        }
        warning_fault_process_->UpdateWarningFaultStatus(
            monitor_fault_clusters_, monitor_fault_events_);
      };
  auto service_available_callback = [this](const bool bResult) {
    phm_client_is_success_ = bResult;
    AERROR << "service_available_callback is:" << bResult;
  };
  phm_client_->Init("/app/conf/planning/fm/orin/planning_fm_config.yaml",
                    service_available_callback, fault_cluster_callback);
  phm_client_->Start();
#endif
}

void OrinTriggerManager::PubFaultDataToTrigger(  // NOLINT
    const PlanningFault& planning_fault, bool is_state_ready_to_report) {
  if (!phm_client_is_success_) {
    InitFault();
  }
  if (!FLAGS_enable_fault_collect || !is_state_ready_to_report) {
    return;
  }
  for (int i = 0; i < planning_fault.fault_info_size(); i++) {
    const auto& fault_info = planning_fault.fault_info(i);
    if (fault_info.type() > 0) {
#ifndef ISX86
      const auto fault_id = static_cast<uint32>(fault_info.type() / 100);
      const auto fault_obj = static_cast<uint8>(fault_info.type() % 100);
      AERROR << "fault id:" << fault_id << " fault obj: " << fault_obj;
      netaos::phm::SendFault_t sendFault(fault_id, fault_obj, 1, true, 1000);
      sendFault.faultDebounce.debounceType =
          TL::netaos::phm::DebounceType::DEBOUNCE_TYPE_COUNT;
      sendFault.faultDebounce.debounceSetting.countDebounce.debounceCount = 1;
      sendFault.faultDebounce.debounceSetting.countDebounce.debounceTime = 1000;
      phm_client_->ReportFault(sendFault);
#endif
    }
  }
}

// NOLINTEND
}  // namespace planning
}  // namespace TL
