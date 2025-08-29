/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  sim location
 */

#include <csignal>
#include <iostream>
#include <string>

#include "ara/exec/execution_client.h"
#include "common/time/clock.h"
#include "core/core.h"
#include "dead_reckoning/dead_reckoning.h"

void SignalHandle(const char* data, int size) {
  std::ofstream fs("glog_dump.log", std::ios::app);
  std::string str = std::string(data, size);
  fs << str;
  fs.close();
  LOG(ERROR) << str;
}

int32_t main(int argc, char** argv) {
  apollo::common::SetProcessStartFromRelativePath();
  apollo::common::module_name = "dr_location";
  signal(SIGPIPE, SIG_IGN);
  google::InstallFailureSignalHandler();
  google::InstallFailureWriter(&SignalHandle);
  google::ParseCommandLineFlags(&argc, &argv, true);
#if USE_HUAWEI_LOG
  FLAGS_minloglevel = google::FATAL;
#else
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
#endif
  DeadReckoning dead_reckoning;
  dead_reckoning.RegistAlgProcessFunc(
      "main_dr_location", std::bind(&DeadReckoning::AlgProcess, &dead_reckoning,
                                    std::placeholders::_1));
  dead_reckoning.Start(
      "manual_service/dead_reckoning/conf/dead_reckoning.yaml");
  dead_reckoning.NeedStopBlocking();
  dead_reckoning.Stop();
#if USE_HUAWEI_LOG
#else
  google::ShutdownGoogleLogging();
#endif
}
