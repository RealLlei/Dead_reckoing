/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  sim location
 */

#include <csignal>
#include <iostream>
#include <string>

//#include "ara/exec/execution_client.h"
#include "dead_reckoning/dead_reckoning.h"

void SignalHandle(const char* data, int size) {
  std::ofstream fs("glog_dump.log", std::ios::app);
  std::string str = std::string(data, size);
  fs << str;
  fs.close();
  LOG(ERROR) << str;
}

int32_t main(int argc, char** argv) {

  DeadReckoning dead_reckoning;
//  dead_reckoning.RegistAlgProcessFunc(
//      "main_dr_location", std::bind(&DeadReckoning::AlgProcess, &dead_reckoning,
//                                    std::placeholders::_1));
  dead_reckoning.Start(
      "manual_service/dead_reckoning/conf/dead_reckoning.yaml");
  //dead_reckoning.NeedStopBlocking();
  dead_reckoning.Stop();
  return 0;
}
