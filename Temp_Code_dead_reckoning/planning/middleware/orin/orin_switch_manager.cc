/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning
 */

#include "planning/middleware/orin/orin_switch_manager.h"
#include <cfg/cfg_data_def.h>
#include "common/configs//config_gflags.h"
#include "common/file/log.h"

using TL::soc::WarningSwitchMemory_Status;

namespace TL {
namespace planning {
namespace WarningCfg {

void Init() {
  auto* cfg_client = ConfigParam::Instance();
  if (cfg_client == nullptr) {
    return;
  }
  cfg_client->Init(2000);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void GetVehicleCfg(VehicleConfigure* const vehicle_cfg) {
  auto* cfg_client = ConfigParam::Instance();
  if (cfg_client == nullptr || vehicle_cfg == nullptr) {
    return;
  }
  vehicle_cfg->set_dow(static_cast<::google::protobuf::int32>(true));
  vehicle_cfg->set_rcw(static_cast<::google::protobuf::int32>(true));
  vehicle_cfg->set_fcta(static_cast<::google::protobuf::int32>(true));
  vehicle_cfg->set_rcta(static_cast<::google::protobuf::int32>(true));
  vehicle_cfg->set_lca(static_cast<::google::protobuf::int32>(true));

/*

reference:

set cmd in orin
set 2:
cfg set string dids/F170 "01 09 00 ff ff ff 78 02 09 0c 40 02 22 18 60 29 00 ff c0 fe ab 13 ff 8f e4 bf 50 d7 fd 86 40 88 0f bf 58 00 00 3c 00 e9 5a 8a 7c 47 fd f7 46 7f bf 23 2a f0 03 08 00 3f 0f 00" 2
set 1:
cfg set string dids/F170 "01 09 00 ff ff ff 78 02 09 0c 40 02 22 18 60 29 00 ff c0 fe ab 13 ff 8f e4 bf 50 d7 fd 86 40 88 0f bf 58 00 00 3c 00 e9 5a 8a 7c 47 fd f7 46 7f bf 23 2a f0 03 08 00 3f 0f 00" 2
set 0:
cfg set string dids/F170 "01 09 00 ff ff ff 78 02 09 0c 40 02 22 18 60 29 00 ff c0 fe ab 13 ff 8f e4 bf 50 d7 fd 86 40 88 0f bf 58 00 00 3c 00 e9 5a 8a 7c 47 fd f7 46 7f bf 23 2a f0 03 08 00 3f 07 00" 2

query vehiclecfg cmd  in orin
cfg get uint8_t vehiclecfg/intelligentControlPlatform

*/
  uint8_t cfg = 0;
  TL::netaos::cfg::CfgResultCode ret = TL::netaos::cfg::CfgResultCode::CONFIG_TIME_OUT;
  ret = cfg_client->GetParam<uint8_t>("vehiclecfg/intelligentControlPlatform",
                                      cfg);
  AINFO << " read intelligentControlPlatform status:" << ret << "value:" <<cfg;

  if ((TL::netaos::cfg::CfgResultCode::CONFIG_OK != ret) || (0x2 != cfg)) {
    // 读取配置字不成功 或 非滑板底盘 都返回为 主项目
    // 通过配置字区分EP41主项目和CTC项目,0X2:SICC代表滑板底盘，0X1:XPC代表主项目
    // 默认值为1
    cfg = 1;
  }
  vehicle_cfg->set_central_control_platform(cfg);

  // return;

  // ret = cfg_client->GetParam<uint8_t>("vehiclecfg/dow", cfg);
  // if (ret == 1) {
  //   vehicle_cfg->set_dow(cfg);
  // } else {
  //   vehicle_cfg->set_dow(true);
  // }
  // ret = cfg_client->GetParam<uint8_t>("vehiclecfg/rcw", cfg);
  // if (ret == 1) {
  //   vehicle_cfg->set_rcw(cfg);
  // } else {
  //   vehicle_cfg->set_rcw(true);
  // }
  // ret = cfg_client->GetParam<uint8_t>("vehiclecfg/fcta", cfg);
  // if (ret == 1) {
  //   vehicle_cfg->set_fcta(cfg);
  // } else {
  //   vehicle_cfg->set_fcta(true);
  // }
  // ret = cfg_client->GetParam<uint8_t>("vehiclecfg/rcta", cfg);
  // if (ret == 1) {
  //   vehicle_cfg->set_rcta(cfg);
  // } else {
  //   vehicle_cfg->set_rcta(true);
  // }
  // ret = cfg_client->GetParam<uint8_t>("vehiclecfg/parallelAuxiliaryLCA", cfg);
  // if (ret == 1) {
  //   vehicle_cfg->set_lca(cfg);
  // } else {
  //   vehicle_cfg->set_lca(true);
  // }
}

void GetLonCtrlSetDisMem(uint32_t* const acc_dis) {
  auto* cfg_client = ConfigParam::Instance();
  if (cfg_client == nullptr || acc_dis == nullptr) {
    *acc_dis = 0;
    return;
  }
  uint8_t cfg = 0;
  uint8_t ret = 0;
  ret = cfg_client->GetParam<uint8_t>("alg_mem/lon_ctrl_set_dis_mem", cfg);
  if (ret == 1) {
    *acc_dis = cfg;
  } else {
    *acc_dis = 3;
  }
}

void SetLonCtrlSetDisMem(int32_t acc_dis) {
  auto* cfg_client = ConfigParam::Instance();
  if (cfg_client == nullptr) {
    return;
  }
  static int32_t last_update_acc_dis = 0;
  if (acc_dis == last_update_acc_dis) {
    return;
  }
  int ret = cfg_client->SetParam<uint8_t>(
      "alg_mem/lon_ctrl_set_dis_mem", static_cast<uint8_t>(acc_dis),
      static_cast<TL::netaos::cfg::ConfigPersistType>(2));
  if (ret == 1) {
    last_update_acc_dis = acc_dis;
  } else {
    AERROR << "alg_mem/lon_ctrl_set_dis_mem fail:" << ret;
  }
}

void GetWarningSwitchMem(WarningSwitchMemory* const warning_switch_mem) {
  auto* cfg_client = ConfigParam::Instance();
  if (cfg_client == nullptr || warning_switch_mem == nullptr) {
    return;
  }
  if (FLAGS_global_enable_warning_mem) {
    warning_switch_mem->set_lca_on_off_set_mem(WarningSwitchMemory::ON);
    warning_switch_mem->set_fcta_on_off_set_mem(WarningSwitchMemory::ON);
    warning_switch_mem->set_rcta_on_off_set_mem(WarningSwitchMemory::ON);
    warning_switch_mem->set_rcw_on_off_set_mem(WarningSwitchMemory::ON);
    warning_switch_mem->set_dow_on_off_set_mem(WarningSwitchMemory::ON);
    static bool dow = false;
    static bool rcw = false;
    static bool lca = false;
    static bool rcta = false;
    static bool fcta = false;
    if (!lca) {
      auto ret = cfg_client->SetParam<uint8_t>(
          "alg_mem/lca_mem", static_cast<uint8_t>(2),
          static_cast<TL::netaos::cfg::ConfigPersistType>(2));
      lca = (ret == TL::netaos::cfg::CfgResultCode::CONFIG_OK);
    }
    if (!dow) {
      auto ret = cfg_client->SetParam<uint8_t>(
          "alg_mem/dow_mem", static_cast<uint8_t>(2),
          static_cast<TL::netaos::cfg::ConfigPersistType>(2));
      dow = (ret == TL::netaos::cfg::CfgResultCode::CONFIG_OK);
    }
    if (!rcta) {
      auto ret = cfg_client->SetParam<uint8_t>(
          "alg_mem/rcta_mem", static_cast<uint8_t>(2),
          static_cast<TL::netaos::cfg::ConfigPersistType>(2));
      rcta = (ret == TL::netaos::cfg::CfgResultCode::CONFIG_OK);
    }
    if (!fcta) {
      auto ret = cfg_client->SetParam<uint8_t>(
          "alg_mem/fcta_mem", static_cast<uint8_t>(2),
          static_cast<TL::netaos::cfg::ConfigPersistType>(2));
      fcta = (ret == TL::netaos::cfg::CfgResultCode::CONFIG_OK);
    }
    if (!rcw) {
      auto ret = cfg_client->SetParam<uint8_t>(
          "alg_mem/rcw_mem", static_cast<uint8_t>(2),
          static_cast<TL::netaos::cfg::ConfigPersistType>(2));
      rcw = (ret == TL::netaos::cfg::CfgResultCode::CONFIG_OK);
    }
    return;
  }
  uint8_t cfg = 0;
  uint8_t ret = 0;
  ret = cfg_client->GetParam<uint8_t>("alg_mem/dow_mem", cfg);
  if (ret == 1) {
    warning_switch_mem->set_dow_on_off_set_mem(
        static_cast<WarningSwitchMemory_Status>(cfg - 1));
  } else {
    AERROR << "dow_mem errror:" << ret;
  }
  ret = cfg_client->GetParam<uint8_t>("alg_mem/rcw_mem", cfg);
  if (ret == 1) {
    warning_switch_mem->set_rcw_on_off_set_mem(
        static_cast<WarningSwitchMemory_Status>(cfg - 1));
  } else {
    AERROR << "rcw_mem errror:" << ret;
  }
  ret = cfg_client->GetParam<uint8_t>("alg_mem/rcta_mem", cfg);
  if (ret == 1) {
    warning_switch_mem->set_rcta_on_off_set_mem(
        static_cast<WarningSwitchMemory_Status>(cfg - 1));
  } else {
    AERROR << "rcta_mem errror:" << ret;
  }
  ret = cfg_client->GetParam<uint8_t>("alg_mem/fcta_mem", cfg);
  if (ret == 1) {
    warning_switch_mem->set_fcta_on_off_set_mem(
        static_cast<WarningSwitchMemory_Status>(cfg - 1));
  } else {
    AERROR << "fcta_mem errror:" << ret;
  }
  ret = cfg_client->GetParam<uint8_t>("alg_mem/lca_mem", cfg);
  if (ret == 1) {
    warning_switch_mem->set_lca_on_off_set_mem(
        static_cast<WarningSwitchMemory_Status>(cfg - 1));
  } else {
    AERROR << "lca_mem errror:" << ret;
  }
}

void SetWarningSwitchMem(
    const TL::planning::WarningStatus& warning_status,
    TL::planning::warning::WarningStateSwitch* const last_warning_state) {
  if (FLAGS_global_enable_warning_mem) {
    return;
  }
  const auto lca_state_changed = OnSwitchStatusChanged(
      warning_status.lca_state(), &last_warning_state->LCA_state);

  const auto dow_state_changed = OnSwitchStatusChanged(
      warning_status.dow_state(), &last_warning_state->OSE_state);

  const auto rcta_state_changed = OnSwitchStatusChanged(
      warning_status.rcta_state(), &last_warning_state->RCTA_state);

  const auto fcta_state_changed = OnSwitchStatusChanged(
      warning_status.fcta_state(), &last_warning_state->FCTA_state);

  const auto rcw_state_changed = OnSwitchStatusChanged(
      warning_status.rcw_state(), &last_warning_state->RCW_state);

  if (!lca_state_changed && !dow_state_changed && !rcta_state_changed &&
      !fcta_state_changed && !rcw_state_changed) {
    return;
  }
  auto* cfg_client = ConfigParam::Instance();
  if (cfg_client == nullptr) {
    return;
  }
  if (lca_state_changed) {
    cfg_client->SetParam<uint8_t>(
        "alg_mem/lca_mem",
        static_cast<uint8_t>(static_cast<int>(warning_status.lca_state()) + 1),
        static_cast<TL::netaos::cfg::ConfigPersistType>(2));
  }
  if (dow_state_changed) {
    cfg_client->SetParam<uint8_t>(
        "alg_mem/dow_mem",
        static_cast<uint8_t>(static_cast<int>(warning_status.dow_state()) + 1),
        static_cast<TL::netaos::cfg::ConfigPersistType>(2));
  }
  if (rcta_state_changed) {
    cfg_client->SetParam<uint8_t>(
        "alg_mem/rcta_mem",
        static_cast<uint8_t>(static_cast<int>(warning_status.rcta_state()) + 1),
        static_cast<TL::netaos::cfg::ConfigPersistType>(2));
  }
  if (fcta_state_changed) {
    cfg_client->SetParam<uint8_t>(
        "alg_mem/fcta_mem",
        static_cast<uint8_t>(static_cast<int>(warning_status.fcta_state()) + 1),
        static_cast<TL::netaos::cfg::ConfigPersistType>(2));
  }
  if (rcw_state_changed) {
    cfg_client->SetParam<uint8_t>(
        "alg_mem/rcw_mem",
        static_cast<uint8_t>(static_cast<int>(warning_status.rcw_state()) + 1),
        static_cast<TL::netaos::cfg::ConfigPersistType>(2));
  }
}

bool OnSwitchStatusChanged(const bool cur_status, bool* const last_status) {
  const auto ret = cur_status != *last_status;
  *last_status = cur_status;
  return ret;
}

void Release() {
  ConfigParam::Instance()->DeInit();
}

}  // namespace WarningCfg
}  // namespace planning
}  // namespace TL
