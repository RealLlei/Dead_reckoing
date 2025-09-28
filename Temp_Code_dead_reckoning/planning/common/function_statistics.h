/***************************************************************************
*
* Copyright (c) 2024 TLauto.com, Inc. All Rights Reserved
*
**************************************************************************/

/**
* @file:   function_statistics.h
* @author: wangshounian(wangshounian@TLauto.com)
* @date:   2024/01/16 13:48:02
* @brief: 
*
**/

#pragma once

#include <string>

#include "common/file/log.h"
#include "common/time/clock.h"

namespace TL {
namespace planning {
// NOLINTBEGIN
class FunctionStatistics {
 public:
  explicit FunctionStatistics(const std::string& function_name)
      : function_name_(function_name) {
    start_time_ = common::Clock::NowInMicroseconds();
  }

  ~FunctionStatistics() {
#if defined(ISORIN)
    double end_time = common::Clock::NowInMicroseconds();
    AINFO << FIXED << SETPRECISION(3) << "Function: " << function_name_
          << " finish, start_time: " << start_time_
          << ", end_time: " << end_time
          << ", execution time: " << end_time - start_time_ << " ms";
#endif
  }

  // NOLINTEND
 private:
  double start_time_;
  std::string function_name_;
};

}  // namespace planning
}  // namespace TL
