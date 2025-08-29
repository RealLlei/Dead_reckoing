/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  location多源融合定位demo头文件
 */
#pragma once
#ifndef ADSFI_SAMPLE_RTK_LOCATION_H
#define ADSFI_SAMPLE_RTK_LOCATION_H
#include <memory>
#include <string>

#include "adsf/node/node_base.h"
#include "common/file/file.h"
#include "common/time/clock.h"
#include "common/util/util.h"
#include "core/core.h"
#include "dead_reckoning/src/dead_reckoning_core.h"
#include "dead_reckoning/src/dead_reckoning_gflags.h"

#include "proto/fsm/function_manager.pb.h"
#include "proto/localization/localization.pb.h"

using apollo::soc::Chassis;
// using apollo::common::Clock;
using apollo::localization::Localization;
// using apollo::common:Pose;
using apollo::dead_reckoning::DeadReckoningCore;
using hz_Adsfi::NodeBase;
using hz_Adsfi::NodeBundle;

class DeadReckoning : public NodeBase {
 public:
  ~DeadReckoning() {}

  int32_t AlgInit();
  int32_t AlgProcess(NodeBundle* input);
  virtual void AlgRelease();

 private:
  void GetChassis(NodeBundle* const input);
  void ProcessDeadRocking();
  void PubLocation(const Localization& location);

 private:
  apollo::localization::Localization localization_;
  apollo::soc::Chassis chassis_;
  // The timer to publish dummy prediction
  // std::unique_ptr<cyber::Timer> sim_prediction_timer_;
  // Time interval of the timer, in milliseconds.
  static constexpr double kSimPredictionIntervalMs = 100;
  DeadReckoningCore dead_reckoning_core_;
  apollo::functionmanager::FunctionManagerIn fct_in_;
  apollo::common::VehicleParam vehicle_param_;
};
#endif  // ADSFI_SAMPLE_RTK_LOCATION_H
