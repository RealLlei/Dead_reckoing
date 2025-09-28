/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "planning/common/frame.h"
#include "planning/common/planning_gflags.h"
#include "planning/proto/hmi_config.pb.h"

namespace TL {
namespace planning {

class OverSpeedDecider {
 public:
  OverSpeedDecider() = default;
  ~OverSpeedDecider() = default;
  /**
   * @brief 
   * 
   * @param frame 
   * @param reference_line_info 
   */
  void Init(Frame* frame, ReferenceLineInfo* reference_line_info);
  /**
   * @brief 
   * 
   * @param target_is_on_main_road 
   * @param target_is_tunnel_lane 
   * @return true 
   * @return false 
   */
  bool CheckAllowOverSpeed(bool target_is_on_main_road,
                           bool target_is_tunnel_lane);

 private:
  /**
   * @brief 
   * 
   * @param frame 
   * @param reference_line_info 
   */
  void CaheADCLaneInfo(Frame* frame, ReferenceLineInfo* reference_line_info);

 private:
  bool last_is_on_main_road_ = false;
  bool adc_is_tunnel_lane_ = false;
  bool usr_adjust_cruise_speed_ = false;
  bool adc_is_main_road_ = true;
};

}  // namespace planning
}  // namespace TL
