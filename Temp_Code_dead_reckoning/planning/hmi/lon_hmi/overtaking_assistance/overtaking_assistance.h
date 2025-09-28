/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <list>
#include <memory>
#include <string>
#include <utility>

#include "google/protobuf/stubs/port.h"
#include "planning/common/frame.h"
#include "planning/common/planning_gflags.h"
#include "planning/pnc_map/pnc_map.h"
#include "planning/proto/hmi_config.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {
static constexpr double kPlanningCycleTime = 0.1;

/**
 * @brief acc超车辅助
 *
 */
class OvertakingAssistance {
 public:
  OvertakingAssistance() = default;
  ~OvertakingAssistance() = default;

  /**
   * @brief init
   *
   * @param nnp_fct_in fct in 输入信息
   * @param speed_adapt_config 
   */
  void Init(const functionmanager::AccOvertakeFollowTimeConfig& config);
  /**
   * @brief 更新输入信息
   *
   * @param nnp_fct_in fct in 输入信息
   * @param chassis 底盘
   */
  void Update(functionmanager::FunctionManagerIn* nnp_fct_in,
              const TL::soc::Chassis& chassis);

  /**
   * @brief Get the Start object
   * 
   * @return true 
   * @return false 
   */
  bool GetStart() const { return start_; }

 private:
  TL::functionmanager::AccOvertakeFollowTimeConfig config_;
  double start_time_ = 0.0;
  bool start_ = false;
  int usr_override_cnt_ = 0;
};  // namespace planning
}  // namespace planning
}  // namespace TL
