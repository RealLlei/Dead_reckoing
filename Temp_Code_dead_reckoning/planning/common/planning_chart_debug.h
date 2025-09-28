#ifndef PLANNING_COMMON_PLANNING_CHART_DEBUG_H
#define PLANNING_COMMON_PLANNING_CHART_DEBUG_H
/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *
 *****************************************************************************/

#pragma once

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "planning/common/frame.h"
#include "planning/localview/local_view.h"

namespace TL {
namespace planning {
class PlanningChartDebug {
 public:
  static void ExportFailedLaneChangeSTChart(
      const ReferenceLineInfo& best_ref_line_info,
      const planning_internal::Debug& debug_info,
      const TL::functionmanager::TaPilotMode& ta_pilot_mode,
      planning_internal::Debug* debug_chart);
  static void ExportOnLaneChart(
      const ReferenceLineInfo& best_ref_line_info,
      const TL::functionmanager::TaPilotMode& ta_pilot_mode,
      planning_internal::Debug* debug_chart);
  static void ExportTrajectoryDebug(const Frame* frame);
  static void ExportReferenceLineDebug(const Frame* frame,
                                       planning_internal::Debug* debug);
  /**
   * @brief Record Anchor Point Bound Debug Info
   * 
   * @param anchor_point_bound 
   * @param name
   * @param debug_chart 
   * @return true 
   * @return false 
   */
  static bool RecordAnchorPointBoundDebugInfo(
      const std::vector<std::pair<double, double>>& anchor_point_bound,
      const std::string& name, planning_internal::Debug* debug_chart);
};
}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_PLANNING_CHART_DEBUG_H
