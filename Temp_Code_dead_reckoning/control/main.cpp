
// Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#include <csignal>
#include "common/time/clock.h"
#include "control/control.h"
#include "core/core.h"

sig_atomic_t g_stopFlag = 0;
constexpr uint32_t cycle_time = 10;

void INTSigHandler(int32_t num) {
  (void)num;
  g_stopFlag = 1;
  std::cout << "Signal Interactive attention received.\n";
}

int32_t main(int argc, char** argv) {
  signal(SIGINT, INTSigHandler);
  signal(SIGPIPE, SIG_IGN);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_minloglevel = 0;
  auto control = std::make_unique<Control>(
      "runtime_service/control/conf/control_config.yaml");
  Adsfi::HafContext context;
  Adsfi::HafStatus ret = control->Init(context);
  control->InitAdControl();
  if (ret != Adsfi::HAF_SUCCESS) {
    AINFO << "Control init failed!";
    control->Destroy(&context);
    return -1;
  }
  control->Sub();
  control->Pub();
  while (!g_stopFlag) {
    const double start_time = TL::common::Clock::NowInSeconds();
    control->Process();
    const double end_time = TL::common::Clock::NowInSeconds();
    double use_time = (end_time - start_time) * 1000;  // ms
    if (!FLAGS_enable_teleop) {
      AINFO << " *********** control use time : " << use_time;
    }
    double sleepTime = (cycle_time - use_time) * 1000;  // us;
    if (sleepTime > 0) {
      static_cast<void>(usleep(sleepTime));
    }
  }
  control->Destroy(&context);
  return 0;
}
