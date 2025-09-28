/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 *   @file obstacle_intention_processor.h
 **/

#pragma once

#include <limits>
#include <set>
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

class ObstacleIntentionProcessor {
 public:
  enum class CrossDirection {
    FROM_LEFT_TO_RIGHT = 1,
    FROM_RIGHT_TO_LEFT = 2,
  };

  enum class CrossPosition {
    ON_LEFT = 1,
    ON_RIGHT = 2,
    ON_LANE = 3,
    UNKNOWN = 4,
  };

  ObstacleIntentionProcessor() = default;

  static bool GetPrintDebug() { return false; }

  void Process(const Frame& frame, ReferenceLineInfo* reference_line_info);

  void ProcessMergeIntention(ReferenceLineInfo* reference_line_info);

  static void ProcessCutinIntention(const Frame& frame,
                                    ReferenceLineInfo* reference_line_info);

  static void ProcessCrossIntention(const Frame& frame,
                                    ReferenceLineInfo* reference_line_info);

  static bool AddMergeIntentionObstacle(
      ReferenceLineInfo* reference_line_info, const Obstacle& origin_obstacle,
      const LateralIntention& lateral_intention,
      const LongitudinalIntention& longitudinal_intention);

  static bool AddCutinIntentionObstacle(
      ReferenceLineInfo* reference_line_info, const Obstacle& origin_obstacle,
      const LateralIntention& lateral_intention,
      const LongitudinalIntention& longitudinal_intention, double end_l);

  static void CheckIfCrossObstacles(const Frame& frame,
                                    ReferenceLineInfo* reference_line_info);

  static bool IsReverseObstacle(const ReferenceLineInfo& reference_line_info,
                                const Obstacle& origin_obstacle);

  static bool AddCutinPredictionProObstacle(
      ReferenceLineInfo* reference_line_info, const Obstacle& origin_obstacle,
      const LateralIntention& lateral_intention,
      const LongitudinalIntention& longitudinal_intention, double end_l);

 private:
  std::set<int32_t> near_merge_point_obstacles_;
};

}  // namespace planning
}  // namespace TL
