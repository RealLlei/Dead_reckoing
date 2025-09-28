/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning
 */

#include "planning/middleware/mdc/switch_manager.h"
#include "adsf/node/node_base.h"
#include "common/configs//config_gflags.h"

using TL::soc::WarningSwitchMemory_Status;

namespace TL {
namespace planning {
namespace WarningCfg {

void Init() {
  auto* cfg_client = HzCfgClient::Instance();
  if (cfg_client == nullptr) {
    return;
  }
  cfg_client->Init();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void GetVehicleCfg(VehicleConfigure* const vehicle_cfg) {
  auto* cfg_client = HzCfgClient::Instance();
  if (cfg_client == nullptr || vehicle_cfg == nullptr) {
    return;
  }
  uint8_t cfg = 0;
  uint8_t ret = 0;
  ret = cfg_client->GetParam<uint8_t>("vehiclecfg", "dow", cfg);
  if (ret == 0) {
    vehicle_cfg->set_dow(cfg);
  }
  ret = cfg_client->GetParam<uint8_t>("vehiclecfg", "rcw", cfg);
  if (ret == 0) {
    vehicle_cfg->set_rcw(cfg);
  }
  ret = cfg_client->GetParam<uint8_t>("vehiclecfg", "fcta", cfg);
  if (ret == 0) {
    vehicle_cfg->set_fcta(cfg);
  }
  ret = cfg_client->GetParam<uint8_t>("vehiclecfg", "rcta", cfg);
  if (ret == 0) {
    vehicle_cfg->set_rcta(cfg);
  }
  ret =
      cfg_client->GetParam<uint8_t>("vehiclecfg", "parallelAuxiliaryLCA", cfg);
  if (ret == 0) {
    vehicle_cfg->set_lca(cfg);
  }
}

void GetWarningSwitchMem(WarningSwitchMemory* const warning_switch_mem) {
  auto* cfg_client = HzCfgClient::Instance();
  if (cfg_client == nullptr || warning_switch_mem == nullptr) {
    return;
  }
  if (FLAGS_global_enable_warning_mem) {
    warning_switch_mem->set_lca_on_off_set_mem(WarningSwitchMemory::ON);
    warning_switch_mem->set_fcta_on_off_set_mem(WarningSwitchMemory::ON);
    warning_switch_mem->set_rcta_on_off_set_mem(WarningSwitchMemory::ON);
    warning_switch_mem->set_rcw_on_off_set_mem(WarningSwitchMemory::ON);
    warning_switch_mem->set_dow_on_off_set_mem(WarningSwitchMemory::ON);
    return;
  }
  uint8_t cfg = 0;
  uint8_t ret = 0;
  ret = cfg_client->GetParam<uint8_t>("alg_mem", "dow_mem", cfg);
  if (ret == 0) {
    warning_switch_mem->set_dow_on_off_set_mem(
        static_cast<WarningSwitchMemory_Status>(cfg - 1));
  }
  ret = cfg_client->GetParam<uint8_t>("alg_mem", "rcw_mem", cfg);
  if (ret == 0) {
    warning_switch_mem->set_rcw_on_off_set_mem(
        static_cast<WarningSwitchMemory_Status>(cfg - 1));
  }
  ret = cfg_client->GetParam<uint8_t>("alg_mem", "rcta_mem", cfg);
  if (ret == 0) {
    warning_switch_mem->set_rcta_on_off_set_mem(
        static_cast<WarningSwitchMemory_Status>(cfg - 1));
  }
  ret = cfg_client->GetParam<uint8_t>("alg_mem", "fcta_mem", cfg);
  if (ret == 0) {
    warning_switch_mem->set_fcta_on_off_set_mem(
        static_cast<WarningSwitchMemory_Status>(cfg - 1));
  }
  ret = cfg_client->GetParam<uint8_t>("alg_mem", "lca_mem", cfg);
  if (ret == 0) {
    warning_switch_mem->set_lca_on_off_set_mem(
        static_cast<WarningSwitchMemory_Status>(cfg - 1));
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
  auto* cfg_client = HzCfgClient::Instance();
  if (cfg_client == nullptr) {
    return;
  }
  if (lca_state_changed) {
    cfg_client->SetParam<uint8_t>(
        "alg_mem", "lca_mem",
        static_cast<uint8_t>(static_cast<int>(warning_status.lca_state()) + 1));
  }
  if (dow_state_changed) {
    cfg_client->SetParam<uint8_t>(
        "alg_mem", "dow_mem",
        static_cast<uint8_t>(static_cast<int>(warning_status.dow_state()) + 1));
  }
  if (rcta_state_changed) {
    cfg_client->SetParam<uint8_t>(
        "alg_mem", "rcta_mem",
        static_cast<uint8_t>(static_cast<int>(warning_status.rcta_state()) +
                             1));
  }
  if (fcta_state_changed) {
    cfg_client->SetParam<uint8_t>(
        "alg_mem", "fcta_mem",
        static_cast<uint8_t>(static_cast<int>(warning_status.fcta_state()) +
                             1));
  }
  if (rcw_state_changed) {
    cfg_client->SetParam<uint8_t>(
        "alg_mem", "rcw_mem",
        static_cast<uint8_t>(static_cast<int>(warning_status.rcw_state()) + 1));
  }
}

bool OnSwitchStatusChanged(const bool cur_status, bool* const last_status) {
  const auto ret = cur_status != *last_status;
  *last_status = cur_status;
  return ret;
}

void Release() {
  HzCfgClient::Instance()->DeInit();
}

}  // namespace WarningCfg
}  // namespace planning
}  // namespace TL
