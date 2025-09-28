/******************************************************************************
 * Copyright 2023 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once
#include <sys/types.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#ifndef ISX86
#include "dc/include/dc_client.h"
#include "phm/include/phm_client.h"
#endif
#include "planning/trigger/warning_fault_process.h"
#include "planning/warning/lbs/common/TM_Global_Types.h"

#include "proto/planning/trigger.pb.h"

namespace TL {
namespace planning {
#ifndef ISX86
using TL::netaos::dc::DcClient;
using TL::netaos::phm::PHMClient;
#endif
/**
 * @brief 数据收集上云和故障触发接口
 *
 */
class OrinTriggerManager {  // NOLINT
 public:
  OrinTriggerManager();
  ~OrinTriggerManager();

  /**
   * @brief 根据事件信息触发数据收集上云接口
   *
   * @param event_trigger 触发的事件
   */
  void PubEventDataToTrigger(const EventTrigger& event_trigger) const;

  /**
   * @brief 上报故障id
   * @param planning_fault 
   * @param is_state_ready_to_report 
   */
  void PubFaultDataToTrigger(const PlanningFault& planning_fault,
                             bool is_state_ready_to_report);

  /**
   * @brief Get the warning fault status
   *
   * @param warning_fault warning fault status
   */
  void GetWarningFaultStatus(soc::MonitorFaultDebug* warning_switch_or_fault);

 private:
  void InitFault();
#ifndef ISX86
  std::unique_ptr<DcClient> dc_client_ = nullptr;
  std::unique_ptr<PHMClient> phm_client_ = nullptr;
#endif
  std::unordered_map<std::string, u_int8_t> monitor_fault_clusters_ = {};
  std::unordered_set<int32_t> monitor_fault_events_ = {};
  std::unique_ptr<WarningFaultProcess> warning_fault_process_ = nullptr;
  bool phm_client_is_success_ = false;
};
}  // namespace planning
}  // namespace TL
