/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_limit_cache.h
 **/

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "common/filters/mean_filter.h"
#include "common/math/double_type.h"
#include "common/math/math_utils.h"
#include "map/hdmap/path.h"
#include "planning/common/frame.h"
#include "planning/common/obstacle.h"
#include "planning/common/path/path_data.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/speed_limit.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/slt_obstacle_cache.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/common/vehicle_state.pb.h"

namespace TL {
namespace planning {

/**
 * @brief  speed limit changing with position
 * 
 */
struct PositionSpeedLimit {
  double map_speed_limit = 0.0;
  double real_map_speed_limit = 0.0;
  double curvature_speed_limit = 0.0;
  double decision_speed_limit = 0.0;
  double pedestrian_speed_limit = 0.0;
  double speed_limit = 0.0;
  double origin_map_speed_limit = 0.0;
  bool allow_over_speed = false;
};

/**
 * @brief speed limit changing with time
 * 
 */
struct TimeSpeedLimit {
  double cruise_speed_limit = 0.0;
  double map_speed_limit = 0.0;
  double comfortable_speed_limit = 0.0;
  double critical_speed_limit = 0.0;
};

/**
 * @brief speed limit cache is used to cache speed limit info
 *
 */
class SpeedLimitCache final {
 public:
  SpeedLimitCache();

  /**
   * @brief init cache
   *
   * @param sequence_num
   * @param distance_last_frame
   * @param config vt sample config
   * @param reference_line_info current reference line info
   * @param init_point planning start point
   * @param path_data path planning result
   */
  void Init(
      const SpeedCacheConfig& config, const Frame& frame,
      const ReferenceLineInfo& reference_line_info,
      const common::TrajectoryPoint& init_point, const Frame* last_frame,
      double distance_from_last_frame,
      const std::vector<const SLTObstacleCache*>& speed_limit_obstacle_caches_);

  /**
   * @brief Get last frame position speed limit at path s
   *
   * @param s
   * @return PositionSpeedLimit
   */
  [[nodiscard]] const PositionSpeedLimit& GetLastPositionSpeedLimit(
      const double s) const {
    const auto index = common::math::Clamp(
        static_cast<int>(s / last_unit_s_ + common::math::double_type::epslion),
        0, static_cast<int>(s_count_ - 1));
    return last_position_speed_limits_[index];
  }

  /**
   * @brief Get current frame position speed limit at path s
   *
   * @param s
   * @return const PositionSpeedLimit&
   */
  [[nodiscard]] const PositionSpeedLimit& GetPositionSpeedLimit(
      const double s) const {
    const auto index = common::math::Clamp(
        static_cast<int>(s / current_unit_s_ +
                         common::math::double_type::epslion),
        0, s_count_ - 1);
    return position_speed_limits_[index];
  }

  /**
   * @brief Get current frame time speed limit at time
   *
   * @param time
   * @return const TimeSpeedLimit&
   */
  [[nodiscard]] const TimeSpeedLimit& GetTimeSpeedLimit(double time) const {
    const auto index = common::math::Clamp(
        static_cast<int>(time / time_unit_ +
                         common::math::double_type::epslion),
        0, time_count_ - 1);
    return time_speed_limits_[index];
  }

  /**
   * @brief Get min speed limit at path range [start_s, end_s]
   *
   * @param start_s
   * @param end_s
   * @return double
   */
  [[nodiscard]] double GetMinPositionSpeedLimit(const double start_s,
                                                const double end_s) const {
    double min_speed_limit = std::numeric_limits<double>::max();
    const auto unit_s = fmax(current_unit_s_, 0.1);
    const auto s_count =
        static_cast<int>(std::round((end_s - start_s) / unit_s));
    for (int i = 0; i <= s_count; ++i) {
      const auto& speed_limit_info =
          GetPositionSpeedLimit(start_s + i * unit_s);
      min_speed_limit =
          fmin(min_speed_limit, speed_limit_info.curvature_speed_limit);
      min_speed_limit =
          fmin(min_speed_limit, speed_limit_info.decision_speed_limit);
      min_speed_limit = fmin(min_speed_limit, speed_limit_info.map_speed_limit);
    }
    return min_speed_limit;
  }

  [[nodiscard]] double GetNudgeSpeedLimit(
      const ReferenceLineInfo& reference_line_info,
      const double max_over_cruise_speed_limit_ratio_when_nudge_obstacle,
      const double max_over_map_speed_limit_ratio_when_nudge_obstacle,
      const double max_speed_limit_when_nudge_obstacle) const {
    auto min_speed_limit =
        fmin(max_speed_limit_when_nudge_obstacle,
             reference_line_info.GetCruiseSpeed() *
                 (1.0 + max_over_cruise_speed_limit_ratio_when_nudge_obstacle));
    for (int i = 0; i < s_count_; ++i) {
      const auto& speed_limit_info = position_speed_limits_.at(i);
      min_speed_limit =
          fmin(min_speed_limit, speed_limit_info.curvature_speed_limit);
      min_speed_limit =
          fmin(min_speed_limit, speed_limit_info.decision_speed_limit);
      min_speed_limit =
          fmin(min_speed_limit,
               speed_limit_info.map_speed_limit *
                   (1.0 + max_over_map_speed_limit_ratio_when_nudge_obstacle));
    }
    return min_speed_limit;
  }

  /**
   * @brief Get min speed limit at time range [start_t, end_t]
   *
   * @param start_t
   * @param end_t
   * @return double
   */
  [[nodiscard]] double GetMinTimeSpeedLimit(const double start_t,
                                            const double end_t) const {
    double min_speed_limit = std::numeric_limits<double>::max();
    const auto t_count = static_cast<int>((end_t - start_t) / time_unit_);
    for (int i = 0; i <= t_count; ++i) {
      const auto t = start_t + i * time_unit_;
      const auto& speed_limit_info = GetTimeSpeedLimit(t);
      min_speed_limit = fmin(min_speed_limit, speed_limit_info.map_speed_limit);
      min_speed_limit =
          fmin(min_speed_limit, speed_limit_info.critical_speed_limit);
    }
    return min_speed_limit;
  }

  /**
   * @brief Get low speed threshold info at time
   *
   * @param s
   * @return double
   */
  [[nodiscard]] double GetLowSpeedThresholdByTime(const double time) const {
    const auto index = common::math::Clamp(
        static_cast<int>(time / time_unit_ +
                         common::math::double_type::epslion),
        0, time_count_ - 1);
    return low_speed_thresholds_[index];
  }

  /**
   * @brief Get path sl range at s
   *
   * @param s
   * @return double
   */
  [[nodiscard]] double GetNudgeSpeedLimit(const double time,
                                          const double s) const {
    const auto t_index = common::math::Clamp(
        static_cast<int>(time / time_unit_ +
                         common::math::double_type::epslion),
        0, time_count_ - 1);
    const auto s_index = common::math::Clamp(
        static_cast<int>(s / current_unit_s_ +
                         common::math::double_type::epslion),
        0, static_cast<int>(s_count_ - 1));
    return nudge_speed_limits_[t_index][s_index];
  }

  /**
   * @brief Get cruise target speed
   *
   * @return double
   */
  [[nodiscard]] double cruise_target_speed() const {
    return cruise_target_speed_;
  }

  /**
   * @brief ModifySpeedLimits
   * 
   * @param cruise_target_speed 
   */
  void ModifySpeedLimits(double cruise_target_speed);

  /**
   * @brief Check whether path s is in keep clear zone
   *
   * @param s path s
   * @return true path s is in keep clear zone
   * @return false path s is not in keep clear zone
   */
  [[nodiscard]] bool InKeepClearRange(double s) const;

 private:
  /**
  * @brief Cache cruise speed limits
  * 
  * @param config speed config
  * @param reference_line_info current reference line info
  * @param init_point planning start point
  */
  void CacheCruiseSpeedLimits(const SpeedCacheConfig& config,
                              const ReferenceLineInfo& reference_line_info,
                              const common::TrajectoryPoint& init_point);

  /**
   * @brief Cache map speed limits
   * 
   * @param config vt sample config
   * @param init_point planning start point
   */
  void ProcessMapSpeedLimitsDecreased(
      const SpeedCacheConfig& config, const common::TrajectoryPoint& init_point,
      const ReferenceLineInfo& reference_line_info, const Frame* last_frame);

  /**
   * @brief Cache curvature speed limit
   *
   * @param frame current frame
   * @param config vt sample config
   * @param reference_line_info current reference line info
   * @param last_frame last frame
   */
  void CachePositionSpeedLimits(const SpeedCacheConfig& config,
                                const Frame& frame,
                                const ReferenceLineInfo& reference_line_info,
                                const Frame* last_frame);

  void SmoothCurvatureSpeedLimit(const SpeedCacheConfig& config,
                                 const PathData& path_data);

  /**
   * @brief Cache curvature speed limit
   *
   * @param config vt sample config
   * @param reference_line_info current reference line info
   * @param init_point planning start point
   */
  void CacheTimeSpeedLimits(const SpeedCacheConfig& config, const Frame& frame,
                            const ReferenceLineInfo& reference_line_info,
                            const common::TrajectoryPoint& init_point,
                            const Frame* last_frame);

  /**
   * @brief 
   * 
   * @param init_point planning start point
   * @param target_speed target speed limit
   * @param target_speed_start_s target speed limit start s
   * @param comfort_decel comfort decel
   * @param comfort_jerk comfort jerk
   * @param time_length time length
   * @param speed_limits 
   * @return true 
   * @return false 
   */
  bool CalculateComfortDecelSpeedLimits(
      const common::TrajectoryPoint& init_point, double target_speed,
      double target_speed_start_s, double comfort_decel, double comfort_jerk,
      double time_length, std::vector<double>* speed_limits) const;

  /**
   * @brief Update preview distance
   *
   * @param reference_line_info current reference line info
   * @param preview_distance
   */
  void UpdatePreviewDistance(const common::TrajectoryPoint& init_point);

  /**
   * @brief Smooth cruise target speed
   *
   * @param path_data path planning result
   */
  void CalculateCruiseTargetSpeed(const ReferenceLineInfo& reference_line_info);

  /**
   * @brief Add keep clear range to cache
   *
   * @param obstacles
   */
  void AddToKeepClearRange(const std::vector<const Obstacle*>& obstacles);

  /**
   * @brief Sort and merge keep clear range
   *
   * @param keep_clear_range
   */
  static void SortAndMergeRange(
      std::vector<std::pair<double, double>>* keep_clear_range);

  /**
   * @brief Check if lane change finish
   * 
   * @param reference_line_info 
   */
  void CheckIfLaneChangeFinish(const ReferenceLineInfo& reference_line_info);

  /**
   * @brief 
   * 
   * @param config 
   * @param init_point 
   */
  void CachePedestrianSpeedLimit(const SpeedCacheConfig& config,
                                 const common::TrajectoryPoint& init_point);

  void CacheRampSpeedLimit(const SpeedCacheConfig& config, const Frame& frame,
                           const common::TrajectoryPoint& init_point,
                           const ReferenceLineInfo& reference_line_info);

  void CacheTunnelSpeedLimit(const SpeedCacheConfig& config, const Frame& frame,
                             const common::TrajectoryPoint& init_point,
                             const ReferenceLineInfo& reference_line_info);

  void CacheConeSpeedLimit(const SpeedCacheConfig& config,
                           const ReferenceLineInfo& reference_line_info,
                           const common::TrajectoryPoint& init_point);

  /**
   * @brief 
   * 
   * @param config 
   * @param init_point 
   * @param frame 
   */
  void CacheCriticalSpeedLimit(const SpeedCacheConfig& config,
                               const common::TrajectoryPoint& init_point,
                               const Frame& frame,
                               const ReferenceLineInfo& reference_line_info);
  /**
   * @brief 
   * 
   * @param config 
   * @param init_point 
   * @param frame 
   */
  void CacheComfortableSpeedLimit(const SpeedCacheConfig& config,
                                  const common::TrajectoryPoint& init_point,
                                  const Frame& frame,
                                  const ReferenceLineInfo& reference_line_info);

  /**
   * @brief 
   * 
   * @param config 
   * @param init_point 
   * @param front_lane 
   * @param reference_line_info 
   * @param last_lane_info 
   * @param deceleration_for_lane 
   */
  std::pair<bool, std::vector<double>> ModifyFrontLaneSpeedLimit(
      const common::TrajectoryPoint& init_point,
      const hdmap::LaneRangeInfo& front_lane, double comfort_decel,
      double comfort_jerk, double speed_limit_point_distance_to_frone_lane,
      hdmap::LaneRangeInfo* last_lane_info, bool* deceleration_for_lane);

  /**
   * @brief Cache low speed threshold
   * 
   * @param init_point 
   * @param reference_line_info 
   */
  void CacheLowSpeedThreshold(const common::TrajectoryPoint& init_point,
                              const ReferenceLineInfo& reference_line_info);

  /**
   * @brief ModifyPositionMapSpeedLimitInfosByOverSpeed
   * 
   * @param speed_limit 
   */
  void ModifyPositionMapSpeedLimitInfosByOverSpeed(
      const TL::planning::SpeedLimit& speed_limit, const Frame& frame);

  /**
   * @brief Cache nudge speed limit
   * 
   * @param config 
   * @param slt_obstacle_caches 
   */
  void CacheNudgeSpeedLimit(
      const SpeedCacheConfig& config,
      const std::vector<const SLTObstacleCache*>& slt_obstacle_caches,
      const Frame& frame, const common::TrajectoryPoint& init_poin,
      const ReferenceLineInfo& reference_line_info);
  /**
   * @brief 
   * 
   * @param config 
   * @param frame 
   * @param init_point 
   */
  void CacheTollhouseSpeedLimit(const SpeedCacheConfig& config,
                                const Frame& frame,
                                const common::TrajectoryPoint& init_point);

  const double time_unit_ = 0.2;
  double last_unit_s_ = 0.1;
  double last_start_s_ = 0.0;
  double last_end_s_ = 0.0;
  double current_unit_s_ = 0.1;
  double current_start_s_ = 0.0;
  double current_end_s_ = 0.0;
  int time_count_ = 1;
  const int s_count_ = 250;

  struct ObstacleHistoryInfo {
    std::deque<SLTObstacleCache::ObstacleInfo> historical_trajectory;
    const SLTObstacleCache* slt_obstacle_cache;
    double obstacle_average_l_upper;
    double obstacle_average_l_lower;
    double obstacle_average_ds;
  };

  std::map<int, ObstacleHistoryInfo> obstacle_history_info_map_;

  // current frame time speed limits
  std::vector<TimeSpeedLimit> time_speed_limits_;
  // current frame position speed limits
  std::vector<PositionSpeedLimit> position_speed_limits_;
  // low speed thresholds
  std::vector<double> low_speed_thresholds_;
  // keep clear
  std::vector<std::pair<double, double>> keep_clear_range_;
  // cruise target speed filter
  std::shared_ptr<common::MeanFilter> cruise_target_speed_filter_;
  double preview_distance_ = 0.0;
  // cruise target speed
  double cruise_target_speed_ = FLAGS_planning_upper_speed_limit;
  // frame sequence number
  uint32_t sequence_num_ = std::numeric_limits<uint32_t>::max();
  // last frame speed limit info
  std::vector<PositionSpeedLimit> last_position_speed_limits_;
  // adc move distance from last frame to current frame
  double distance_from_last_frame_ = 0.0;
  // ramp speed limit info
  hdmap::LaneRangeInfo last_ramp_info_;
  // whether adc is deceleration for ramp
  bool deceleration_for_ramp_ = false;
  // tunnel speed limit info
  hdmap::LaneRangeInfo last_tunnel_info_;
  // whether adc is deceleration for tunnel
  bool deceleration_for_tunnel_ = false;
  std::vector<std::vector<double>> nudge_speed_limits_;
  //
  common::VehicleParam vehicle_param_;
  static constexpr size_t kObstacleHistorySize = 10;
  std::vector<double> speeds_;
  std::vector<double> nudge_speed_limit_coefs_;
  bool adc_in_tunnel_ = false;
};

}  // namespace planning
}  // namespace TL
