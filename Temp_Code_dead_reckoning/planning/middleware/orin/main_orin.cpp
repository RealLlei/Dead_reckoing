
//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
#ifdef ISORIN
#include <mimalloc-new-delete.h>
#endif
#include <csignal>

#include "common/configs/config_gflags.h"
#include "common/time/clock.h"
#include "planning/common/function_statistics.h"
#include "planning/common/planning_gflags.h"
#include "planning/middleware/orin/planning_orin.h"

// NOLINTBEGIN
void SignalHandle(const char* data, int size) {
#ifdef ISX86
  auto* var = std::getenv("CYBER_PATH");
  std::string work_dir(var);
  std::ofstream fs(work_dir + "/data/log/planning_core_dump.log",
                   std::ios::app);
#else
  std::ofstream fs("/opt/usr/log/soc_log/planning_core_dump.log",
                   std::ios::app);
#endif
  std::string str = std::string(data, size);
  fs << str;
  fs.close();
  LOG(ERROR) << str;
}

void ResetPlanningYmal(const std::string& yaml_name) {
  auto* var = std::getenv("CYBER_PATH");
  std::string work_dir(var);
  std::string reset_yaml = work_dir + "/" + yaml_name;
  YAML::Node yaml_node = YAML::LoadFile(reset_yaml);
  if (yaml_node["log"].IsDefined() && yaml_node["log"]["file"].IsDefined()) {
    yaml_node["log"]["file"] = work_dir + "/data/log/";
  }
  if (yaml_node["trigger"].IsDefined()) {
    for (int i = 0; i < yaml_node["trigger"].size(); ++i) {
#ifdef ISNNP
      if (yaml_node["trigger"][i]["mainSources"].IsDefined() &&
          yaml_node["trigger"][i]["mainSources"].size() > 0) {
        yaml_node["trigger"][i]["mainSources"][0]["timeout"] = 2000000;
      }
      if (!FLAGS_use_original_fct_in &&
          yaml_node["trigger"][i]["auxSources"].IsDefined() &&
          yaml_node["trigger"][i]["auxSources"].size() > 0) {
        for (int j = 0; j < yaml_node["trigger"][i]["auxSources"].size(); j++) {
          if (yaml_node["trigger"][i]["auxSources"][j]["name"]
                  .as<std::string>() == "mcu_to_ego") {
            yaml_node["trigger"][i]["auxSources"][j]["name"] = "fct_in";
            break;
          }
        }
      }
#endif
#ifdef ISAVP
      FLAGS_use_original_fct_in = true;
      if (yaml_node["trigger"][i]["name"].IsDefined() &&
          yaml_node["trigger"][i]["name"].as<std::string>() == "avp_main") {
        yaml_node["trigger"][i]["type"] = "EVENT";
        yaml_node["trigger"][i].remove("period");
        yaml_node["trigger"][i]["mainSources"][0]["name"] = "mcu_to_ego";
        yaml_node["trigger"][i]["mainSources"][0]["timeout"] = 2000000;
        for (int j = 0; j < yaml_node["trigger"][i]["auxSources"].size(); j++) {
          if (yaml_node["trigger"][i]["auxSources"][j]["name"]
                  .as<std::string>() == "mcu_to_ego") {
            yaml_node["trigger"][i]["auxSources"][j].remove("mcu_to_ego");
            yaml_node["trigger"][i]["auxSources"].remove(j);
            break;
          }
        }
      }
#endif
    }
  }
#ifdef ISNNP
  if (!FLAGS_use_original_fct_in && yaml_node["recvInstances"].IsDefined()) {
    for (int i = 0; i < yaml_node["recvInstances"].size(); ++i) {
      if (yaml_node["recvInstances"][i]["name"].as<std::string>() ==
          "mcu_to_ego") {
        yaml_node["recvInstances"][i]["name"] = "fct_in";
        yaml_node["recvInstances"][i]["topic"] = "/planning/fct_in";
      }
    }
  }
#endif
  if (yaml_node["sendInstances"].IsDefined()) {
    for (int i = 0; i < yaml_node["sendInstances"].size(); ++i) {
      if (yaml_node["sendInstances"][i]["name"].as<std::string>() == "fct_in") {
        yaml_node["sendInstances"][i].remove("name");
        yaml_node["sendInstances"][i].remove("type");
        yaml_node["sendInstances"][i].remove("topic");
        yaml_node["sendInstances"][i].remove("domainId");
        yaml_node["sendInstances"].remove(i);
        break;
      }
    }
  }
  std::ofstream file(reset_yaml);
  file << yaml_node << std::endl;
  file.close();
  file.flush();
}

int32_t main(int argc, char** argv) {
  TL::common::SetProcessStartFromRelativePath();
  TL::common::module_name = "europa";
  signal(SIGPIPE, SIG_IGN);
  google::InstallFailureSignalHandler();
  google::InstallFailureWriter(&SignalHandle);
  google::ParseCommandLineFlags(&argc, &argv, true);

#if USE_ORIN_LOG
  FLAGS_minloglevel = google::FATAL;
#else
  google::InitGoogleLogging(argv[0]);
  // FLAGS_v = 4;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
#ifdef ISX86
  google::SetLogDestination(google::ERROR, "");
  google::SetLogDestination(google::WARNING, "");
  google::SetLogDestination(google::FATAL, "");
#endif
#endif
  TL::planning::FunctionStatistics done_guard("main_orin");
  TL::Planning planning;

  // planning.RegistAlgProcessWithProfilerFunc(
  //     "ehr", std::bind(&TL::Planning::GetEhpThread, &planning,
  //                      std::placeholders::_1, std::placeholders::_2));

  if (FLAGS_enable_planning_self_simulator) {
    planning.RegistAlgProcessFunc(
        "self_sim", std::bind(&TL::Planning::SelfSimulatorFunc, &planning,
                              std::placeholders::_1));
  } else {
    planning.RegistAlgProcessWithProfilerFunc(
        "nnp_main", std::bind(&TL::Planning::AlgProcessNNP, &planning,
                              std::placeholders::_1, std::placeholders::_2));
    planning.RegistAlgProcessWithProfilerFunc(
        "avp_main", std::bind(&TL::Planning::AlgProcessAVP, &planning,
                              std::placeholders::_1, std::placeholders::_2));
  }
  const std::string planning_conf_file =
      "runtime_service/planning/conf/planning_config.yaml";
#ifdef ISX86
  ResetPlanningYmal(planning_conf_file);
  planning.InitLoggerStandAlone(planning_conf_file);
  planning.Start(planning_conf_file, true);
#else
  if (FLAGS_enable_planning_self_simulator) {
    planning.Start("runtime_service/planning/conf/self_planning_config.yaml");
  } else {
    planning.Start(planning_conf_file);
  }
#endif
  planning.NeedStopBlocking();
  planning.Stop();
#if USE_ORIN_LOG
#else
  google::ShutdownGoogleLogging();
#endif
  return 0;
}

// NOLINTEND
