/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning/trigger/warning_fault_process.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/file/log.h"

#include "proto/planning/trigger.pb.h"

namespace TL {
namespace planning {

using TL::soc::MonitorFaultDebug;

WarningFaultProcess::WarningFaultProcess() {
  for (const auto& function :
       {"lca", "dow", "rcw", "fcta", "rcta", "lat", "lon"}) {
    irreversible_fault_[function] = false;
    reversible_fault_[function] = false;
  }
  auto warning_inhibition_fault_mapping =
      FLAGS_warning_inhibition_fault_map_file;
#ifdef ISORIN
  warning_inhibition_fault_mapping =
      FLAGS_orin_warning_inhibition_fault_map_file;
#endif
  if (!TL::common::GetProtoFromFile(warning_inhibition_fault_mapping,
                                       &warning_inhibition_map_)) {
    AERROR << "failed to load warning inhibition map file "
           << warning_inhibition_fault_mapping;
  }
}

MonitorFaultDebug WarningFaultProcess::GetWarningFault() {  // NOLINT
  std::lock_guard<std::mutex> lock(fault_status_multithread_mutex_);
  return warning_switch_or_fault_;
}

void WarningFaultProcess::UpdateWarningFaultStatus(  // NOLINT
    const std::unordered_map<std::string, u_int8_t>& fault_clusters,
    const std::unordered_set<int32_t>& fault_events) {
  // UpdateIrreversibleFault(fault_clusters, fault_events);
  UpdateReversibleFault(fault_clusters, fault_events);
  {
    std::lock_guard<std::mutex> lock(fault_status_multithread_mutex_);
    auto* mutable_warning_fault =
        warning_switch_or_fault_.mutable_warning_fault();
    mutable_warning_fault->set_lca(reversible_fault_.at("lca"));
    mutable_warning_fault->set_dow(reversible_fault_.at("dow"));
    mutable_warning_fault->set_rcw(reversible_fault_.at("rcw"));
    mutable_warning_fault->set_fcta(reversible_fault_.at("fcta"));
    mutable_warning_fault->set_rcta(reversible_fault_.at("rcta"));
    // auto* mutable_planning_fault =
    //     warning_switch_or_fault_.mutable_planning_fault();
    // mutable_planning_fault->set_lat(reversible_fault_.at("lat"));
    // mutable_planning_fault->set_lon(reversible_fault_.at("lon"));
    soc::MonitorFault* mutable_monitor_fault = nullptr;
    mutable_monitor_fault = warning_switch_or_fault_.mutable_monitor_fault();
    auto* mutable_monitor_fault_clusters =
        mutable_monitor_fault->mutable_monitor_fault_clusters();
    mutable_monitor_fault_clusters->clear();
    for (const auto& fault_cluster : fault_clusters) {
      std::stringstream ss{};
      ss << std::hex << std::setw(2) << std::setfill('0')
         << static_cast<int>(fault_cluster.second);
      const std::string hex_str = "0x" + ss.str();
      (*mutable_monitor_fault_clusters)[fault_cluster.first] = hex_str;
    }

    mutable_monitor_fault->clear_monitor_fault_event();
    for (const auto& fault_event : fault_events) {
      mutable_monitor_fault->add_monitor_fault_event(fault_event);
    }
  }
}

void WarningFaultProcess::UpdateIrreversibleFault(
    const std::unordered_map<std::string, std::string>& fault_clusters,
    const std::unordered_set<int32_t>& fault_events) {
  for (const auto& function : {"lca", "dow", "rcw", "fcta", "rcta"}) {
    if (irreversible_fault_[function] ||
        warning_inhibition_map_.warning_fault_map().count(function) == 0) {
      continue;
    }

    const auto& cluster_map = warning_inhibition_map_.warning_fault_map()
                                  .at(function)
                                  .irreversible_inhibition_clusters();
    const auto& event_map = warning_inhibition_map_.warning_fault_map()
                                .at(function)
                                .irreversible_inhibition_events();
    for (const auto& cluster : fault_clusters) {
      const auto key = cluster.first + "-" + cluster.second;
      if (std::find(cluster_map.begin(), cluster_map.end(), key) !=
          cluster_map.end()) {
        irreversible_fault_[function] = true;
        break;
      }
    }
    for (const auto& event : fault_events) {
      if (std::find(event_map.begin(), event_map.end(), event) !=
          event_map.end()) {
        irreversible_fault_[function] = true;
        break;
      }
    }
  }
}

void WarningFaultProcess::UpdateReversibleFault(
    const std::unordered_map<std::string, u_int8_t>& fault_clusters,
    const std::unordered_set<int32_t>& fault_events) {
  for (const auto& function :
       {"lca", "dow", "rcw", "fcta", "rcta", "lat", "lon"}) {
    if (irreversible_fault_[function]) {
      reversible_fault_[function] = true;
      continue;
    }
    if (warning_inhibition_map_.warning_fault_map().count(function) == 0) {
      reversible_fault_[function] = false;
      continue;
    }

    const auto& cluster_map = warning_inhibition_map_.warning_fault_map()
                                  .at(function)
                                  .reversible_inhibition_clusters();
    const auto& event_map = warning_inhibition_map_.warning_fault_map()
                                .at(function)
                                .reversible_inhibition_events();
    bool cluster_matched = false;
    bool event_matched = false;
    for (const auto& cluster : fault_clusters) {
      bool is_ok_culuster = true;
      const auto key = cluster.first;
      is_ok_culuster = key != "platform" ||
                       (key == "platform" && (cluster.second & 0xFD) > 0);
      if (is_ok_culuster && std::find(cluster_map.begin(), cluster_map.end(),
                                      key) != cluster_map.end()) {
        cluster_matched = true;
        break;
      }
    }
    for (const auto& event : fault_events) {
      if (std::find(event_map.begin(), event_map.end(), event) !=
          event_map.end()) {
        event_matched = true;
        break;
      }
    }
    reversible_fault_[function] = cluster_matched || event_matched;
  }
}

}  // namespace planning
}  // namespace TL
