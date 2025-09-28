/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#pragma once
#include <cassert>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "common/status/status.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_detm_reset.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_width_and_quality_monitor.h"
#include "planning/proto/navigation_hdmap_config.pb.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
using TL::common::Status;

class ResetAndQualityMonitor {
 public:
  ResetAndQualityMonitor();
  explicit ResetAndQualityMonitor(const planning::PerceptionMapConfig& config);
  ~ResetAndQualityMonitor() = default;
  Status Init();
  void PublicProcess(DeciderData* decider_data);
  void GetLLResetAndQuality(DeciderData* decider_data);
  void GetRLResetAndQuality(DeciderData* decider_data);
  void GetNLLResetAndQuality(DeciderData* decider_data);
  void GetNRLResetAndQuality(DeciderData* decider_data);

 private:
  DeciderData decider_data_;
  planning::PerceptionMapConfig config_;
  std::unique_ptr<LaneDetmReset> left_lane_detm_reset_;
  std::unique_ptr<LaneDetmReset> right_lane_detm_reset_;
  std::unique_ptr<LaneDetmReset> next_left_lane_detm_reset_;
  std::unique_ptr<LaneDetmReset> next_right_lane_detm_reset_;

  std::unique_ptr<LaneWidthQualityMonitor> left_lane_width_quality_monitor_;
  std::unique_ptr<LaneWidthQualityMonitor> right_lane_width_quality_monitor_;
  std::unique_ptr<LaneWidthQualityMonitor>
      next_left_lane_width_quality_monitor_;
  std::unique_ptr<LaneWidthQualityMonitor>
      next_right_lane_width_quality_monitor_;
};
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
