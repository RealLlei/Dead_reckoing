/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning
 */

/**
 * @file
 * @brief rarning swhitch manager
 */

#pragma once

#include "config_client/include/hz_cfg_client.h"
#include "planning/warning/warning.h"
#include "proto/planning/planning.pb.h"
#include "proto/soc/chassis.pb.h"
#include "proto/soc/vehicle_cfg.pb.h"

using TL::config::client::HzCfgClient;
using TL::soc::VehicleConfigure;
using TL::soc::WarningSwitchMemory;

namespace TL {
namespace planning {
namespace WarningCfg {

/**
 * init
 */
void Init();

/**
 * @brief 获取整车配置字
 */
void GetVehicleCfg(VehicleConfigure* vehicle_cfg);

/**
 * @brief Get the Warning Switch Mem object
 *
 */
void GetWarningSwitchMem(WarningSwitchMemory*);
/**
 * @brief Set the Warning Switch Mem object
 *
 * @param warning_status
 */
void SetWarningSwitchMem(
    const TL::planning::WarningStatus& warning_status,
    TL::planning::warning::WarningStateSwitch* last_warning_state);
/**
 * @brief check warning state changed
 *
 * @param cur_status
 * @param last_status
 * @return true warning state changed
 * @return false warning state not changed
 */
bool OnSwitchStatusChanged(bool cur_status, bool* last_status);
/**
 * @brief Release
 *
 */
void Release();

};  // namespace WarningCfg
}  // namespace planning
}  // namespace TL
