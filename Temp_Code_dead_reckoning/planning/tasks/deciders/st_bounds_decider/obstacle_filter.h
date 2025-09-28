/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 *   @file obstacle_intention_processor.h
 **/

#pragma once

#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/status/status.h"
#include "planning/common/frame.h"
#include "planning/common/obstacle.h"
#include "planning/common/path_decision.h"
#include "planning/common/reference_line_info.h"

namespace TL {
namespace planning {

static constexpr double kReverseObsHeading = 2.617993878;
static constexpr int kVirtualLaneCheckCnt = 30;
static constexpr int kNormalLaneCheckCnt = 20;
static constexpr double kVirtualLaneIgnoreDec = -3.8;
static constexpr double kNormalLaneIgnoreDec = -2.5;

static constexpr double kMaxReverseConsiderDistance = 30.0;
static constexpr double kMaxNormalConsiderDistance = 60.0;

class ObstacleFilter {
  struct ReverseObsInfo {
    bool on_virtual_lane = false;
    int check_cnt = 0;
    double theta_diff = 0;
    double cal_dec = 0;
    double dis = std::numeric_limits<double>::infinity();
    bool ignore = false;
  };

 public:
  ObstacleFilter() = default;

  ~ObstacleFilter() = default;
  /**
   * @brief 
   * 
   * @param frame 
   * @param reference_line_info 
   */
  void Process(const Frame& frame, ReferenceLineInfo* reference_line_info);

 private:
  /**
  * @brief 
  * 
  * @param frame 
  * @param reference_line_info 
  */
  void IgnoreObstaclesOnVirtualLane(const Frame& frame,
                                    ReferenceLineInfo* reference_line_info);
  /**
   * @brief 
   * 
   * @param frame 
   * @param reference_line_info 
   */
  static void IgnoreOutLanePedAndBicycle(
      const Frame& frame, ReferenceLineInfo* reference_line_info);

 private:
  std::unordered_map<int32_t, ReverseObsInfo> reverse_obs_map_;
};

}  // namespace planning
}  // namespace TL
