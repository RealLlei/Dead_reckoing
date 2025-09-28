/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning debug
 */

#pragma once

#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/struct2pb/struct2pb.h"
#include "common/util/message_util.h"
#include "common/util/util.h"
#include "planning/common/planning_gflags.h"
#include "proto/common/header.pb.h"

/**
 * @namespace TL::planning::CheckInput
 * @brief ozon::planning::CheckInput
 */
namespace TL {
namespace planning {
namespace PreProcess {
using TL::common::ErrorCode;
using TL::common::util::Int2ProtoEnum;

// NOLINTBEGIN
/**
 * @brief Check publish_stamp timestamp
 *
 * @param moudle_timestamp
 * @return true 数据时间差在范围之内
 * @return false 数据时间差在范围之外
 */
bool CheckPublishTimestamp(const double moudle_timestamp) {
  if (FLAGS_enable_skip_check_input || fLB::FLAGS_zmq_replay_data) {
    return true;
  }
  const double time_now = TL::common::Clock::NowInSeconds();
  const double time_diff = time_now - moudle_timestamp;

  if (fabs(time_diff) > FLAGS_input_data_publish_time_out_period_num) {
#ifndef ISX86
    std::stringstream ss;
    ss.precision(13);
    ss << " check publish_stamp time out ,time diff: " << time_diff
       << " curr time :" << time_now
       << " ,moudle_timestamp :" << moudle_timestamp;
    AWARN << ss.str();
#endif
    return false;
  }
  return true;
}

/**
 * @brief Check data_stamp timestamp
 *
 * @param moudle_timestamp
 * @return true 数据时间差在范围之内
 * @return false 数据时间差在范围之外
 */
bool CheckDataTimestamp(const double moudle_timestamp) {
  if (FLAGS_enable_skip_check_input || fLB::FLAGS_zmq_replay_data) {
    return true;
  }
  const double time_now = TL::common::Clock::NowInSeconds();
  const double time_diff = time_now - moudle_timestamp;

  if (fabs(time_diff) > FLAGS_input_data_data_time_out_period_num) {
#ifndef ISX86
    std::stringstream ss;
    ss.precision(13);
    ss << " check data_stamp time out ,time diff: " << time_diff
       << " curr time :" << time_now
       << " ,moudle_timestamp :" << moudle_timestamp;
    AWARN << ss.str();
#endif
    return false;
  }
  return true;
}

template <typename Proto>
void SaveInputData(const std::shared_ptr<Proto>& ptr_proto, std::string name,
                   uint32_t sequence_num) {
  if (!FLAGS_export_input_data_to_file) {
    return;
  }
  std::string output_file =
      "output/" + std::to_string(sequence_num) + "/" + name + ".pb.txt";
  common::SetProtoToASCIIFile(*ptr_proto, output_file);
}

template <typename Proto>
void ClearHeaderStatus(const std::shared_ptr<Proto>& ptr_proto) {
  if (ptr_proto != nullptr && ptr_proto->has_header() &&
      ptr_proto->header().has_status()) {
    ptr_proto->mutable_header()->clear_status();
  }
}

// NOLINTEND
/**
 * @brief check input data
 *
 * @param Proto
 * @param data_valid struct2pb return
 * @param error_code
 * @param ptr_proto
 */
template <typename Proto>
void CheckInputData(const int data_valid, const uint32_t error_code,
                    const bool is_nnp_mode,
                    const std::shared_ptr<Proto>& ptr_proto) {
  bool data_check_failed = false;
  uint32_t final_error_code = error_code;
  std::string msg = " ";
  if (data_valid) {
    bool data_stamp_valid = true;
    bool publish_stamp_valid = true;
    if (ptr_proto->header().has_data_stamp()) {
      data_stamp_valid = CheckDataTimestamp(ptr_proto->header().data_stamp());
    }
    if (ptr_proto->header().has_publish_stamp()) {
      publish_stamp_valid =
          CheckPublishTimestamp(ptr_proto->header().publish_stamp());
    }
    if (!data_stamp_valid || !publish_stamp_valid) {
      data_check_failed = true;
      final_error_code = is_nnp_mode ? error_code + 1000 : error_code + 3000;
      std::stringstream ss;
      ss.precision(13);
      ss << "input time out , current time : "
         << TL::common::Clock::NowInSeconds()
         << " ,data_stamp : " << ptr_proto->header().data_stamp()
         << " ,publish_stamp: " << ptr_proto->header().publish_stamp();
      msg = ss.str();
      AWARN << "data header : " << ptr_proto->header().ShortDebugString();
    }
  } else {
    msg = "input data error";
    AWARN << msg << "data header : " << ptr_proto->header().ShortDebugString();
    ptr_proto->Clear();
    data_check_failed = true;
    final_error_code = is_nnp_mode ? error_code + 2000 : error_code + 4000;
  }
  if (data_check_failed) {
    auto* header = ptr_proto->mutable_header();
    header->mutable_status()->set_error_code(
        Int2ProtoEnum(ErrorCode::OK, final_error_code));  // NOLINT
    header->mutable_status()->set_msg(msg);
  }
}

}  // namespace PreProcess
}  // namespace planning
}  // namespace TL
