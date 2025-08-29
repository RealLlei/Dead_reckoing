/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  location 多源融合定位demo源文件
 */

#include "dead_reckoning/dead_reckoning.h"

#include "common/file/file.h"
#include "common/file/log.h"
#include "common/pb2struct/pb2struct.h"
#include "common/struct2pb/struct2pb.h"

int32_t DeadReckoning::AlgProcess(NodeBundle* input) {
  ///< receive
  GetChassis(input);
  ProcessDeadRocking();
  return 1;
}

void DeadReckoning::GetChassis(NodeBundle* const input) {
  const auto& ptr_chassis = std::static_pointer_cast<hz_Adsfi::AlgChassisInfo>(
      input->GetOne("chassis"));
  if (ptr_chassis != nullptr) {
    HAF_LOG_INFO << "chassis rec " << ptr_chassis->header.seq;
    Struct2ChassisPb(*ptr_chassis, fct_in_, vehicle_param_, &chassis_);
  } else {
    chassis_.Clear();
  }
}

void DeadReckoning::AlgRelease() {}

int32_t DeadReckoning::AlgInit() {
  vehicle_param_ =
      apollo::common::VehicleConfigHelper::GetConfig().vehicle_param();
  HAF_LOG_INFO << "DeadReckoning Init.";
  dead_reckoning_core_.Init();
  HAF_LOG_INFO << "DeadReckoning init done";
  return 1;
}

void DeadReckoning::ProcessDeadRocking() {
  auto chassis = std::make_shared<const Chassis>(chassis_);
  auto localization = std::make_shared<Localization>();
  dead_reckoning_core_.RunOnce(chassis, localization);
  if (localization->has_header()) {
    PubLocation(*localization);
  }
  HAF_LOG_INFO << "localization:" << localization->ShortDebugString();
}

void DeadReckoning::PubLocation(const Localization& location) {
  auto location_ptr = std::make_shared<hz_Adsfi::AlgLocation>();
  Pb2StructForLocation(location, location_ptr);
  NodeBundle output;
  output.Add("localization", location_ptr);
  SendOutput(&output);
}
