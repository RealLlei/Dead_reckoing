/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "planning/trigger/warning_fault_process.h"

#include "./hz_fm_agent.h"
#include "proto/planning/trigger.pb.h"
#include "proto/soc/chassis.pb.h"
#include "proto/soc/vehicle_cfg.pb.h"

namespace TL {
namespace planning {

/**
 * @brief 数据收集上云和故障触发接口
 *
 */
class CollectTriggerManager {  // NOLINT
 public:
  CollectTriggerManager();
  ~CollectTriggerManager();

  /**
   * @brief 根据事件信息触发数据收集上云接口
   *
   * @param event_trigger 触发的事件
   */
  void PubEventDataToTrigger(const EventTrigger& event_trigger);

  /**
   * @brief 根据收集的故障触发故障上报
   *
   * @param planning_fault 收集的故障信息
   * @param is_state_ready_to_report 是否处于允许故障上报状态
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
  /**
   * @brief 初始化数据收集接口
   *
   */
  void InitDataCollect();

  /**
   * @brief 初始化故障上报接口
   *
   */
  void InitFault();

  /**
   * @brief 初始化监测故障和warning后处理
   *
   */
  void InitMonitorFault();

  /**
   * @brief 触发数据收集
   *
   * @param event_trigger 触发的事件
   */
  void EventToTrigger(const EventTrigger& event_trigger) const;

  /**
   * @brief 触发故障上报
   *
   * @param planning_fault 故障码
   */
  void FaultToTrigger(const PlanningFault& planning_fault) const;

  bool fault_agent_init_ = false;
  bool data_collect_init_ = false;
  bool fault_monitor_init_ = false;
  std::unordered_map<std::string, std::string> monitor_fault_clusters_ = {};
  std::unordered_set<int32_t> monitor_fault_events_ = {};
  std::unique_ptr<WarningFaultProcess> warning_fault_process_ = nullptr;
};
}  // namespace planning
}  // namespace TL
