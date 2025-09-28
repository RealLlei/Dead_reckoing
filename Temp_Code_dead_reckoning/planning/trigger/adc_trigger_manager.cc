/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning/trigger/adc_trigger_manager.h"

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "./data_collect_client.h"
#include "common/configs/config_gflags.h"
#include "common/file/log.h"

namespace TL {
namespace planning {

using TL::datacollect::client::DataCollectClient;
using TL::datacollect::client::DataCollectResType;
using TL::fm::HzFMAgent;

CollectTriggerManager::CollectTriggerManager() {
  warning_fault_process_ = std::make_unique<WarningFaultProcess>();
  InitDataCollect();
  InitFault();
}

CollectTriggerManager::~CollectTriggerManager() {
  if (fault_agent_init_ && HzFMAgent::Instance()) {
    HzFMAgent::Instance()->DeInit();
  }
}

void CollectTriggerManager::InitDataCollect() {
  if (!FLAGS_enable_planning_trigger ||
      DataCollectClient::Instance() == nullptr) {
    return;
  }
  if (FLAGS_enable_event_collect && !data_collect_init_) {
    data_collect_init_ = DataCollectClient::Instance()->Init();
    ADEBUG << "data collect client init status: " << data_collect_init_;
  }
}

void CollectTriggerManager::InitFault() {
  if (!FLAGS_enable_planning_trigger || !HzFMAgent::Instance()) {
    return;
  }
  if (FLAGS_enable_fault_collect && !fault_agent_init_) {
    auto fault_agent_init_callback = [this](bool status) {
      fault_agent_init_ = status;
    };
    HzFMAgent::Instance()->Init(fault_agent_init_callback);
    ADEBUG << "fm agent init status: " << fault_agent_init_;
  }
}

void CollectTriggerManager::InitMonitorFault() {
  if (!HzFMAgent::Instance()) {
    return;
  }
  if (FLAGS_enable_fault_collect && !fault_monitor_init_) {
    auto fault_cluster_callback =
        [this](TL::fm::FaultCluster_t fault_cluster_data) {
          for (const auto& cluster : fault_cluster_data.cluster_map) {
            if (cluster.first.empty()) {
              continue;
            }
            monitor_fault_clusters_[cluster.first] = cluster.second;
          }
          // record the origin fault id only for debugging
          if (fault_cluster_data.fault_status) {
            monitor_fault_events_.insert(fault_cluster_data.fault_key);
          } else {
            monitor_fault_events_.erase(fault_cluster_data.fault_key);
          }
          warning_fault_process_->UpdateWarningFaultStatus(
              monitor_fault_clusters_, monitor_fault_events_);
        };
    auto fault_event_callback = [this](TL::fm::FaultData_t fault_data) {
      const auto fault_event_id =
          static_cast<int32_t>(fault_data.faultId * 100 + fault_data.faultObj);
      if (fault_data.faultStatus) {
        monitor_fault_events_.insert(fault_event_id);
      } else {
        monitor_fault_events_.erase(fault_event_id);
      }
      warning_fault_process_->UpdateWarningFaultStatus(monitor_fault_clusters_,
                                                       monitor_fault_events_);
    };

    // register interested fault clusters, fault events and callbacks
    HzFMAgent::Instance()->RegistPostProcessCallback(
        FLAGS_planning_fm_register_config_file, fault_cluster_callback,
        fault_event_callback);
    fault_monitor_init_ = true;
  }
}

void CollectTriggerManager::GetWarningFaultStatus(
    TL::soc::MonitorFaultDebug* const warning_switch_or_fault) {
  if (!fault_monitor_init_) {
    InitMonitorFault();
  }
  auto warning_fault = warning_fault_process_->GetWarningFault();
  *warning_switch_or_fault = std::move(warning_fault);
}

void CollectTriggerManager::PubEventDataToTrigger(
    const EventTrigger& event_trigger) {
  if (!FLAGS_enable_planning_trigger) {
    return;
  }
  InitDataCollect();
  if (FLAGS_enable_event_collect) {
    EventToTrigger(event_trigger);
  }
}

void CollectTriggerManager::PubFaultDataToTrigger(
    const PlanningFault& planning_fault, const bool is_state_ready_to_report) {
  if (!FLAGS_enable_planning_trigger || !is_state_ready_to_report) {
    return;
  }
  InitFault();
  if (FLAGS_enable_fault_collect) {
    FaultToTrigger(planning_fault);
  }
}

void CollectTriggerManager::EventToTrigger(  // NOLINT
    const EventTrigger& event_trigger) const {
  if (DataCollectClient::Instance() == nullptr) {
    return;
  }
  for (int i = 0; i < event_trigger.event_info_size(); i++) {
    const auto& event_info = event_trigger.event_info(i);
    if (event_info.type() != 0U) {
      const auto type = DataCollectClient::Instance()->CollectTrigger(
          static_cast<uint32_t>(event_info.type()));
      if (type != DataCollectResType::kDataCollectOK) {
        ADEBUG << "event Trigger Fail id " << int(type);
      }
    }
  }
}

void CollectTriggerManager::FaultToTrigger(  // NOLINT
    const PlanningFault& planning_fault) const {
  if (HzFMAgent::Instance() == nullptr) {
    return;
  }
  for (int i = 0; i < planning_fault.fault_info_size(); i++) {
    const auto& fault_info = planning_fault.fault_info(i);
    if (fault_info.type() > 0) {
      u_int16_t fault_id = fault_info.type() / 100;
      u_int16_t fault_obj = fault_info.type() % 100;
      AERROR << "fault id:" << fault_id << " fault obj: " << fault_obj;
      TL::fm::Fault* mutable_fault = nullptr;  // NOLINT
      mutable_fault = HzFMAgent::Instance()->GenFault(fault_id, fault_obj);
      if (mutable_fault != nullptr) {
        mutable_fault->UseCountBaseDebouncePolicy(1, 1000);
        HzFMAgent::Instance()->ReportFaultAsync(mutable_fault, 1);
      }
    }
  }
}
}  // namespace planning
}  // namespace TL
