/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "proto/planning/trigger.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

class WarningFaultProcess {
 public:
  WarningFaultProcess();
  ~WarningFaultProcess() = default;
  /**
   * @brief Update warning function fault inhibition status
   *
   * @param fault_clusters monitored fault clusters from FM agent
   * @param fault_events monitored fault events from FM agent
   */
  void UpdateWarningFaultStatus(
      const std::unordered_map<std::string, u_int8_t>& fault_clusters,
      const std::unordered_set<int32_t>& fault_events);

  /**
   * @brief Get the Warning Fault object
   * @return soc::MonitorFaultDebug 
   */
  soc::MonitorFaultDebug GetWarningFault();

 private:
  /**
   * @brief Update irreversible warning fault status
   *
   * @param fault_clusters monitored fault clusters from FM agent
   * @param fault_events monitored fault events from FM agent
   */
  void UpdateIrreversibleFault(
      const std::unordered_map<std::string, std::string>& fault_clusters,
      const std::unordered_set<int32_t>& fault_events);

  /**
   * @brief Update reversible warning fault status
   *
   * @param fault_clusters monitored fault clusters from FM agent
   * @param fault_events monitored fault events from FM agent
   */
  void UpdateReversibleFault(
      const std::unordered_map<std::string, u_int8_t>& fault_clusters,
      const std::unordered_set<int32_t>& fault_events);

 private:
  WarningInhibitionMap warning_inhibition_map_;
  std::unordered_map<std::string, bool> irreversible_fault_ = {};
  std::unordered_map<std::string, bool> reversible_fault_ = {};
  std::mutex fault_status_multithread_mutex_;
  soc::MonitorFaultDebug warning_switch_or_fault_;
};

}  // namespace planning
}  // namespace TL
