/*
 * @LastEditors: Please set LastEditors
 */
/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  control检测demo头文件
 */

#ifndef SAMPLE_CONTROL_H
#define SAMPLE_CONTROL_H

#include <memory>
#include <queue>
#include <string>

#include "adsf/algorithm_base/control_base.h"
#include "common/util/macros.h"
#include "common/util/message_util.h"
#include "common/util/util.h"
#include "control/common/dependency_injector.h"
#include "control/controller/controller_agent.h"
#include "control/core/control_core.h"
#include "control/teleop.h"
#include "common/zmq_conventor/zmq_sender.h"
#include "core/core.h"
#include "proto/soc/chassis.pb.h"
#include "control/proto/control_cmd.pb.h"
#include "control/proto/control_conf.pb.h"
#include "control/proto/pad_msg.pb.h"
#include "control/proto/preprocessor.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/planning/planning.pb.h"
using TL::soc::Chassis;
using TL::common::Header;
using TL::common::Status;
using TL::control::ControlCommand;
using TL::control::ControlConf;
using TL::control::ControllerAgent;
using TL::control::DependencyInjector;
using TL::control::LocalView;
using TL::control::PadMessage;
using TL::localization::Localization;
using TL::planning::ADCTrajectory;
class Control : public Adsfi::ControlBase {
 public:
  explicit Control(std::string configFile) : Adsfi::ControlBase(configFile) {}
  ~Control() {}
  void Process();
  void Destroy(Adsfi::HafContext* context);
  void InitAdControl();
  void ProcessTeleop();

 private:
  void GetBody() const;
  void GetChassis();
  void GetEgoTrajectory();
  void GetLocation();
  bool ProcessAdControl();
  void MockChassis();
  void PubData(const ControlCommand& control_command);
  Status ProduceControlCommand(ControlCommand* control_command);
  Status CheckInput(const std::shared_ptr<LocalView>& local_view);
  Status CheckTimestamp(const std::shared_ptr<LocalView>& local_view);
  bool FeedTestData();
  ControlCommand MockControl();
  void SerializeProtoToString(const ControlCommand& control_command);

 private:
  TL::common::Time init_time_;
  Localization latest_localization_;
  Chassis latest_chassis_;
  TL::planning::ADCTrajectory latest_trajectory_;
  PadMessage pad_msg_;
  bool pad_received_ = false;

  unsigned int status_lost_ = 0;
  unsigned int status_sanity_check_failed_ = 0;
  unsigned int total_status_lost_ = 0;
  unsigned int total_status_sanity_check_failed_ = 0;
  ControlConf control_conf_;
  std::mutex mutex_;
  std::shared_ptr<LocalView> local_view_;
  Teleop::Teleop teleop_;
  TL::control::ControlCore control_core_;
  std::unique_ptr<TL::common::ZMQSender> zmq_sender_;
};

#endif  // SAMPLE_CONTROL_H
