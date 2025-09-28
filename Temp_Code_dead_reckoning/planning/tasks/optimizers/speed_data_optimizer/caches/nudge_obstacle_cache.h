/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file nudge_obstacle_cache.h
 **/

#pragma once

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "planning/common/path/path_data.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/slt_obstacle_cache.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class NudgeObstacleCache
 *
 * @brief cache obstacle in the neighbor lane which needs to nudge
 */
class NudgeObstacleCache : public SLTObstacleCache {
 public:
  NudgeObstacleCache(const SpeedCacheConfig& config,
                     const ReferenceLineInfo& reference_line_info,
                     const Obstacle* obstacle, double time_unit, int time_count,
                     int index, const NudgeObstacleCache* old_cache,
                     double time_from_last_frame, bool frozon);

  /**
   * @brief Get big car info at time
   *
   * @param time
   * @return const BigCarInfo&
   */
  [[nodiscard]] const std::shared_ptr<TL::common::math::Polygon2d>&
  GetHexagonAtTime(const double time) const {
    const auto index = common::math::Clamp(
        static_cast<int>(round(time / GetTimeUnit())), 0, GetTimeCount() - 1);
    return hexagons_[index];
  }

  /**
   * @brief Get target nudge state
   * 
   * @return const SpeedCacheConfig::NudgeState& 
   */
  [[nodiscard]] const SpeedCacheConfig::NudgeState& GetTargetNudgeState()
      const {
    return target_nudge_state_;
  }

  /**
   * @brief Get target nudge state
   * 
   * @return const SpeedCacheConfig::NudgeState& 
   */
  void SetTargetNudgeState(SpeedCacheConfig::NudgeState nudge_state) {
    target_nudge_state_ = nudge_state;
  }

  /**
   * @brief Get current nudge state
   * 
   * @return const SpeedCacheConfig::NudgeState& 
   */
  [[nodiscard]] const SpeedCacheConfig::NudgeState& GetCurrentNudgeState()
      const {
    return current_nudge_state_;
  }

  /**
   * @brief Set current nudge state
   * 
   * @param nudge_state 
   */
  void SetCurrentNudgeState(SpeedCacheConfig::NudgeState nudge_state) {
    current_nudge_state_ = nudge_state;
  }

 private:
  /**
   * @brief Cache big car longitudinal speed
   *
   * @param path_data
   */
  void CacheObstacleSpeed(const NudgeObstacleCache& old_cache,
                          double time_from_last_frame);

  /**
   * @brief Cache big car hexagon
   *
   * @param config vt sample config
   * @param path_data path planning result
   */
  void CacheHexagon(const SpeedCacheConfig& config, const PathData& path_data);

  std::vector<std::shared_ptr<TL::common::math::Polygon2d>> hexagons_;
  // target nudge state
  SpeedCacheConfig::NudgeState target_nudge_state_ = SpeedCacheConfig::IGNORE;
  // current nudge state
  SpeedCacheConfig::NudgeState current_nudge_state_ = SpeedCacheConfig::IGNORE;
};

}  // namespace planning
}  // namespace TL
