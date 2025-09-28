/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_cost.h
 **/

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "common/interpolation/interpolation_2d.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/basic_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/nudge_obstacle_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/slt_obstacle_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_limit_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/st_obstacle_cache.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {
using TL::common::Interpolation2D;

class SpeedCache final {
 public:
  struct LaneChangePrepareTarget {
    double s = 0.0;
    double v = 0.0;
    double t = 0.0;
  };

  explicit SpeedCache(const SpeedCacheConfig& config);

  /**
   * @brief Init vt sample cache
   *
   * @param injector
   * @param frame current frame
   * @param reference_line_info current reference line info
   */
  void Init(const std::shared_ptr<DependencyInjector>& injector,
            const Frame& frame, ReferenceLineInfo* reference_line_info,
            const common::TrajectoryPoint& init_point);

  /**
   * @brief Init vt sample cache
   *
   * @param injector
   * @param frame current frame
   * @param reference_line_info current reference line info
   */
  void InitObstacle(const std::shared_ptr<DependencyInjector>& injector,
                    const Frame& frame,
                    const ReferenceLineInfo& reference_line_info,
                    const common::TrajectoryPoint& init_point);

  void Clear();

  /**
   * @brief Get basic cache
   *
   * @return const speed_evaluator::BasicCache&
   */
  [[nodiscard]] const speed_evaluator::BasicCache& GetBasicCache() const {
    return basic_cache_;
  }

  /**
   * @brief Get original init point
   *
   * @return double
   */
  const common::TrajectoryPoint& GetOriginInitPoint() const {
    return origin_init_point;
  }

  /**
   * @brief Set is last fallback
   *
   * @param falg
   */
  void SetIsLastFallback(bool falg) { is_last_fallback_ = falg; }

  /**
   * @brief Get is last fallback
   *
   * @return true
   * @return false
   */
  bool GetIsLastFallback() const { return is_last_fallback_; }

  /**
   * @brief Get mutable basic cache
   *
   * @return speed_evaluator::BasicCach*
   */
  speed_evaluator::BasicCache* GetMutableBasicCache() { return &basic_cache_; }

  /**
   * @brief Get the Obstacle Caches object
   *
   * @return const std::vector<STObstacleCache>&
   */
  [[nodiscard]] const std::vector<STObstacleCache>& GetSTObstacleCaches()
      const {
    return st_obstacle_caches_;
  }

  /**
   * @brief Get the Obstacle Caches object
   *
   * @return const std::vector<SLTObstacleCache>&
   */
  [[nodiscard]] const std::vector<SLTObstacleCache>& GetOldSLTObstacleCaches()
      const {
    return old_slt_obstacle_caches_;
  }

  /**
   * @brief Get the Obstacle Caches object
   *
   * @return const std::vector<SLTObstacleCache>&
   */
  [[nodiscard]] const std::vector<SLTObstacleCache>& GetSLTObstacleCaches()
      const {
    return slt_obstacle_caches_;
  }

  /**
   * @brief Get mutable obstacle caches
   *
   * @return std::vector<SLTObstacleCache>*
   */
  std::vector<STObstacleCache>* GetMutableSTObstacleCaches() {
    return &st_obstacle_caches_;
  }

  /**
   * @brief Get mutable obstacle caches
   *
   * @return std::vector<SLTObstacleCache>*
   */
  std::vector<SLTObstacleCache>* GetMutableSLTObstacleCaches() {
    return &slt_obstacle_caches_;
  }

  /**
   * @brief Get gap lead obstacle cache
   *
   * @return const SLTObstacleCache::ObstacleInfo*
   */
  [[nodiscard]] const SLTObstacleCache::ObstacleInfo* GetGapFrontObstacleInfo()
      const {
    return gap_front_obstacle_info_;
  }

  /**
   * @brief Get gap tail obstacle cache
   *
   * @return const SLTObstacleCache::ObstacleInfo*
   */
  [[nodiscard]] const SLTObstacleCache::ObstacleInfo* GetGapRearObstacleInfo()
      const {
    return gap_rear_obstacle_info_;
  }

  /**
   * @brief Get big Car caches
   *
   * @return const std::vector<NudgeObstacleCache>&
   */
  [[nodiscard]] const std::vector<NudgeObstacleCache>& GetNudgeObstacleCaches()
      const {
    return nudge_obstacle_caches_;
  }

  [[nodiscard]] const std::vector<const STObstacleCache*>&
  GetSafeSTObstacleCaches() const {
    return safe_st_obstacle_caches_;
  }

  /**
   * @brief Get slt obstacles without decision for safe cost
   * 
   * @return const std::vector<SLTObstacleCache*>&
   */
  [[nodiscard]] const std::vector<SLTObstacleCache*>&
  GetSafeSLTObstacleCachesWithOutDecision() const {
    return safe_slt_obstacle_caches_without_decision_;
  }

  /**
   * @brief Get slt obstacles with decision for safe cost
   * 
   * @return const std::vector<SLTObstacleCache*>&
   */
  [[nodiscard]] const std::vector<SLTObstacleCache*>&
  GetSafeSLTObstacleCachesWithDecision() const {
    return safe_slt_obstacle_caches_with_decision_;
  }

  /**
   * @brief Get cross caution obstacle caches
   * 
   * @return const std::vector<speed_evaluator::YieldObstacleCache>& 
   */
  [[nodiscard]] const std::vector<const SLTObstacleCache*>&
  GetSpeedLimitSLTObstacleCaches() const {
    return speed_limit_slt_obstacle_caches_;
  }

  /**
   * @brief Get speed limit cache
   *
   * @return const SpeedLimitCache&
   */
  [[nodiscard]] const SpeedLimitCache& GetSpeedLimitCache() const {
    return speed_limit_cache_;
  }

  /**
   * @brief Get mutable speed limit cache
   *
   * @return const SpeedLimitCache&
   */
  SpeedLimitCache* GetMutableSpeedLimitCache() { return &speed_limit_cache_; }

  const std::vector<const SLTObstacleCache*>& GetCrossSLTObstacleCaches()
      const {
    return cross_slt_obstacle_caches_;
  }

  /**
   * @brief Check whether cache is empty
   *
   * @return true cache is empty
   * @return false cache is not empty
   */
  [[nodiscard]] bool IsEmpty() const;

  /**
   * @brief Get the Follow SLT Obstacle Cache object
   * 
   * @return const SLTObstacleCache* 
   */
  [[nodiscard]] const SLTObstacleCache* GetFollowSLTObstacleCache() const {
    return follow_slt_obstacle_cache_;
  }

  /**
   * @brief Get the Follow ST Obstacle Cache object
   * 
   * @return const STObstacleCache* 
   */
  [[nodiscard]] const STObstacleCache* GetFollowSTObstacleCache() const {
    return follow_st_obstacle_cache_;
  }

  /**
   * @brief Set the Follow Obstacle Index object
   * 
   * @param follow_obstacle_index 
   */
  void SetFollowObstacle(const Frame& frame,
                         const ReferenceLineInfo& reference_line_info,
                         const common::TrajectoryPoint& init_point,
                         const std::string& obstacle_id);

  void UpdateSTBoundary(ReferenceLineInfo* reference_line_info);

  /**
   * @brief Get follow time
   * 
   * @return double 
   */
  [[nodiscard]] double GetFollowTime() const { return follow_time_; }

  /**
   * @brief Set follow time
   * 
   * @param follow_time 
   */
  void SetFollowTime(double follow_time) { follow_time_ = follow_time; }

  /**
   * @brief Get if there are any non ignore nudge obstacle
   * 
   * @return true there are non ignore nudge obstacle
   * @return false there is no non ignore nudge obstacle
   */
  [[nodiscard]] bool GetHasNonIgnoreNudgeObstacle() const {
    return has_non_ignore_nudge_obstacle_;
  }

  /**
   * @brief Update target nudge state
   * 
   * @param reference_line_info 
   */
  void UpdateTargetNudgeState(const ReferenceLineInfo& reference_line_info);

  /**
   * @brief Update current nudge state
   * 
   * @param speed_data 
   */
  void UpdateCurrentNudgeState(const SpeedData* speed_data);

  const std::vector<double>& GetFollowTimes() const { return follow_times_; }

  void SetFollowTimes(std::vector<double>&& follow_times) {
    follow_times_ = std::move(follow_times);
  }

  void SetNeedFollowCurve(const bool need_follow_curve) {
    need_follow_curve_ = need_follow_curve;
  }

  bool GetNeedFollowCurve() const { return need_follow_curve_; }

  /**
   * @brief Consider decision 
   * 
   */
  void ConsiderDecision();

  /**
   * @brief Ignore decision 
   * 
   */
  void IgnoreDecision();

  /**
   * @brief Set the Start After Stop object
   * 
   * @param start_after_stop 
   */
  void SetStartAfterStop(const bool start_after_stop) {
    start_after_stop_ = start_after_stop;
  }

  /**
   * @brief StartAfterStop
   * 
   * @return true 
   * @return false 
   */
  [[nodiscard]] bool StartAfterStop() const { return start_after_stop_; }

  /**
   * @brief last start after stop 
   * 
   * @return true 
   * @return false 
   */
  [[nodiscard]] bool GetLastStartAfterStop() const {
    return last_start_after_stop_;
  }

  [[nodiscard]] int32_t GetOldFollowObstacleId() const {
    return old_follow_obstacle_id_;
  }

  [[nodiscard]] const LaneChangePrepareTarget& GetLaneChangePrepareTarget()
      const {
    return lane_change_prepare_target_;
  }

 private:
  /**
    * @brief Cache obstacles
    * 
    * @param reference_line_info current reference line info
    */
  void CacheSTObstacles(const ReferenceLineInfo& reference_line_info);

  /**
   * @brief Cache yield obstacles
   * 
   * @param config speed cache config
   * @param reference_line_info current reference line info
   */
  void CacheSLTObstacles(const ReferenceLineInfo& reference_line_info);

  /**
   * @brief Cache nudge obstacles
   * 
   * @param reference_line_info current reference line info
   * @param time_from_last_frame time from last frame
   */
  void CacheNudgeObstacles(const ReferenceLineInfo& reference_line_info,
                           double time_from_last_frame);

  /**
   * @brief Cache gap obstacles
   *
   * @param reference_line_info
   */
  void CacheGapObstacles(const std::shared_ptr<DependencyInjector>& injector,
                         const ReferenceLineInfo& reference_line_info,
                         const common::TrajectoryPoint& init_point);

  void CheckIfIgnoreCollision(const Frame& frame,
                              const ReferenceLineInfo& reference_line_info);

  const SLTObstacleCache::ObstacleInfo* CacheGapObstacleInfo(
      const SLTObstacleCache& slt_obstacle_cache);

  /**
   * @brief Calculate calibration table
   * 
   * @param config 
   */
  void CalculateCalibrationTable();

  // cache config
  SpeedCacheConfig config_;
  // basic cache
  speed_evaluator::BasicCache basic_cache_;
  // st obstacles to consider when calculate cost
  std::vector<STObstacleCache> st_obstacle_caches_;
  // st obstacles to consider safe cost
  std::vector<const STObstacleCache*> safe_st_obstacle_caches_;
  // gap lead obstacle cache
  const SLTObstacleCache::ObstacleInfo* gap_front_obstacle_info_ = nullptr;
  // gap tail obstacle cache
  const SLTObstacleCache::ObstacleInfo* gap_rear_obstacle_info_ = nullptr;
  LaneChangePrepareTarget lane_change_prepare_target_;
  // slt obstacles
  std::vector<SLTObstacleCache> slt_obstacle_caches_;
  // last frame slt obstacles
  std::vector<SLTObstacleCache> old_slt_obstacle_caches_;
  // slt obstacles to consider safe cost
  std::vector<SLTObstacleCache*> safe_slt_obstacle_caches_without_decision_;
  // slt obstacle cache to consider safe cost
  std::vector<SLTObstacleCache*> safe_slt_obstacle_caches_with_decision_;
  // slt obstacles to consider safe cost
  std::vector<const SLTObstacleCache*> speed_limit_slt_obstacle_caches_;
  // st obstacles to consider cross cost
  std::vector<const SLTObstacleCache*> cross_slt_obstacle_caches_;
  // obstacle to consider when avoid parallel drive
  std::vector<NudgeObstacleCache> nudge_obstacle_caches_;
  // speed limit cache
  SpeedLimitCache speed_limit_cache_;
  // follow target index in obstacle_caches_
  const STObstacleCache* follow_st_obstacle_cache_ = nullptr;
  // follow target index in obstacle_caches_
  const SLTObstacleCache* follow_slt_obstacle_cache_ = nullptr;
  // follow time
  double follow_time_ = 2.0;

  std::unique_ptr<Interpolation2D> jerk_interpolation_2d_ = nullptr;

  common::TrajectoryPoint origin_init_point;

  // vehicle param
  common::VehicleParam vehicle_param_;
  // follow times
  std::vector<double> follow_times_;
  // last frame follow obstacle id
  int old_follow_obstacle_id_ = -1;

  // info from last lane change finish
  std::set<int32_t> obstacle_from_last_lane_change_finish_;
  double time_from_last_lane_change_finish_ = 0.0;

  std::set<int32_t> intention_obstacle_ids_;

  // is last frame speed fallback
  bool is_last_fallback_ = false;
  // whether need follow curve
  bool need_follow_curve_ = true;
  //
  bool has_non_ignore_nudge_obstacle_ = false;
  // start after stop
  bool start_after_stop_ = false;
  // last frame start after stop
  bool last_start_after_stop_ = false;
};

}  // namespace planning
}  // namespace TL
