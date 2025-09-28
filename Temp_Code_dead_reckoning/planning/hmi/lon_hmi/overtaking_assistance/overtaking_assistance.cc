/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/hmi/lon_hmi/overtaking_assistance/overtaking_assistance.h"
#include <sys/types.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include "common/file/log.h"
#include "common/time/clock.h"
#include "planning/common/frame.h"
#include "planning/common/util/common.h"
#include "planning/pnc_map/pnc_map.h"
#include "proto/common/vehicle_signal.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

void OvertakingAssistance::Init(
    const functionmanager::AccOvertakeFollowTimeConfig& config) {
  config_ = config;
}

void OvertakingAssistance::Update(
    functionmanager::FunctionManagerIn* const fct_in,
    const TL::soc::Chassis& chassis) {
  if (fct_in == nullptr || fct_in->adas_mode() != functionmanager::ACC ||
      !chassis.has_speed_display() ||
      (!start_ && chassis.speed_display() < config_.start_speed_km()) ||
      chassis.driving_mode() == soc::Chassis::COMPLETE_MANUAL) {
    start_ = false;
    start_time_ = 0.0;
    usr_override_cnt_ = 0;
    return;
  }
  if (!start_ &&
      (chassis.signal().turn_switch() == common::LEFT_LEVEL_1 ||
       chassis.signal().turn_switch() == common::LEFT_LEVEL_2) &&
      chassis.signal().turn_signal() ==
          TL::common::VehicleSignal::TURN_LEFT) {
    start_ = true;
    start_time_ = TL::common::Clock::NowInSeconds();
    return;
  }
  if (start_) {
    if (fct_in->fct_nnp_in().acc_state() ==
        functionmanager::FctToNnpInput::ACC_OVERRIDE) {
      usr_override_cnt_++;
    }
    if ((TL::common::Clock::NowInSeconds() - start_time_ -
         usr_override_cnt_ * kPlanningCycleTime) > config_.finsh_time() ||
        (chassis.signal().turn_switch() == common::RIGHT_LEVEL_1 ||
         chassis.signal().turn_switch() == common::RIGHT_LEVEL_2)) {
      start_ = false;
      start_time_ = 0.0;
      usr_override_cnt_ = 0;
    }
  }
}

}  // namespace planning
}  // namespace TL
