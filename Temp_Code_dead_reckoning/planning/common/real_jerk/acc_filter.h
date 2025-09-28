#ifndef PLANNING_COMMON_REAL_JERK_ACC_FILTER_H
#define PLANNING_COMMON_REAL_JERK_ACC_FILTER_H

/******************************************************************************
 * Copyright 2023 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once
#include <sys/types.h>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include "common/filters/digital_filter.h"

namespace TL {
namespace planning {

using TL::common::DigitalFilter;

class AccFilter {
 public:
  AccFilter() = default;
  /**
   * @brief init filter
   * @param ts 
   * @param cut_off_freq 
   */
  void Init(double ts, double cut_off_freq);
  /**
   * @brief input val and output val
   * @param input_val 
   * @return double 
   */
  double OutputFilter(double input_val);

 private:
  DigitalFilter acc_digital_filter_;
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_REAL_JERK_ACC_FILTER_H
