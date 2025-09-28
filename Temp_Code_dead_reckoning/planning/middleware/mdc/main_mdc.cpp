
//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
#include <csignal>

#include "ara/exec/execution_client.h"
#include "common/time/clock.h"
#include "core/core.h"
#include "planning/common/planning_gflags.h"
#include "planning/middleware/mdc/planning_mdc.h"

void SignalHandle(const char* data, int size) {
  std::ofstream fs("/opt/usr/log/hz_log/planning_core_dump.log", std::ios::app);
  std::string str = std::string(data, size);
  fs << str;
  fs.close();
  LOG(ERROR) << str;
}

int32_t main(int argc, char** argv) {
  TL::common::SetProcessStartFromRelativePath();
  TL::common::module_name = "europa";
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

  TL::Planning planning;

  planning.RegistAlgProcessWithProfilerFunc(
      "ehr", std::bind(&TL::Planning::GetEhpThread, &planning,
                       std::placeholders::_1, std::placeholders::_2));

  if (FLAGS_enable_planning_self_simulator) {
    planning.RegistAlgProcessFunc(
        "self_sim", std::bind(&TL::Planning::SelfSimulatorFunc, &planning,
                              std::placeholders::_1));
  } else {
    planning.RegistAlgProcessWithProfilerFunc(
        "avp_main", std::bind(&TL::Planning::AlgProcessAVP, &planning,
                              std::placeholders::_1, std::placeholders::_2));
    planning.RegistAlgProcessWithProfilerFunc(
        "nnp_main", std::bind(&TL::Planning::AlgProcessNNP, &planning,
                              std::placeholders::_1, std::placeholders::_2));
  }
  planning.Start("runtime_service/planning/conf/planning_config.yaml");
  planning.NeedStopBlocking();
  planning.Stop();
#if USE_HUAWEI_LOG
#else
  google::ShutdownGoogleLogging();
#endif

  return 0;
}
