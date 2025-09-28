/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file yield_obstacle_cache.h
 **/

#pragma once

#include <deque>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "planning/common/obstacle.h"
#include "planning/common/reference_line_info.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {
namespace speed_evaluator {

/**
 * @class ObstacleCache
 *
 * @brief cache obstacle info for obstacles which is in st graph
 * These obstacles are used in calculate obstacle cost / collision check
 */
class YieldObstacleCache {
 public:
  struct ObstacleInfo {
    // obstacle s speed
    double ds = 0.0;
    // obstacle l speed
    double dl = 0.0;
    // obstacle boundary s_lower
    double s_lower = std::numeric_limits<double>::max();
    // obstacle boundary s_upper
    double s_upper = std::numeric_limits<double>::lowest();
    // obstacle boundary l_lower
    double l_lower = std::numeric_limits<double>::max();
    // obstacle boundary l_upper
    double l_upper = std::numeric_limits<double>::lowest();
    // obstacle boundary s_lower
    double s_lower_with_buffer = std::numeric_limits<double>::max();
    // obstacle boundary s_upper
    double s_upper_with_buffer = std::numeric_limits<double>::lowest();
    // obstacle boundary l_lower
    double l_lower_with_buffer = std::numeric_limits<double>::max();
    // obstacle boundary l_upper
    double l_upper_with_buffer = std::numeric_limits<double>::lowest();
  };

  YieldObstacleCache(const SpeedCacheConfig& config,
                     const ReferenceLineInfo& reference_line_info,
                     const Obstacle* obstacle, double time_unit,
                     int st_graph_time_count);

  virtual ~YieldObstacleCache() = default;

  /**
   * @brief Get obstacle infos
   *
   * @param time
   * @return const ObstacleInfo&
   */
  [[nodiscard]] const std::vector<ObstacleInfo>& GetObstacleInfos() const {
    return obstacle_infos_;
  }

  /**
   * @brief Get obstacle infos
   *
   * @param time
   * @return const ObstacleInfo&
   */
  [[nodiscard]] std::vector<ObstacleInfo>* GetMutableObstacleInfos() {
    return &obstacle_infos_;
  }

  /**
   * @brief Get obstacle infos
   *
   * @param time
   * @return const ObstacleInfo&
   */
  [[nodiscard]] int GetStGraphTimeCount() const { return st_graph_time_count_; }

  /**
   * @brief Get obstacle info at time
   *
   * @param time
   * @return const ObstacleInfo&
   */
  [[nodiscard]] const ObstacleInfo& GetObstacleInfoAtTime(
      const double time) const {
    const auto index =
        common::math::Clamp(static_cast<int>(round(time / time_unit_)), 0,
                            st_graph_time_count_ - 1);
    return obstacle_infos_[index];
  }

  /**
   * @brief Get obstacle
   *
   * @return const Obstacle*
   */
  [[nodiscard]] const Obstacle* GetObstacle() const { return obstacle_; }

  /**
   * @brief Get min obstacle stop distance
   *
   * @return double
   */
  [[nodiscard]] double GetMinStopDistance() const { return min_stop_distacne_; }

  /**
   * @brief Get the Max T object
   * 
   * @return double 
   */
  [[nodiscard]] double GetMaxT() const { return max_t_; }

  /**
   * @brief Get the Min T object
   * 
   * @return double 
   */
  [[nodiscard]] double GetMinT() const { return min_t_; }

  /**
   * @brief Get the Max T object
   * 
   * @return double 
   */
  [[nodiscard]] double GetTimeUnit() const { return time_unit_; }

  /**
   * @brief Get the Is Cut Out object
   * 
   * @return true 
   * @return false 
   */
  [[nodiscard]] bool GetIsCutOut() const { return is_cut_out_; }

  /**
   * @brief Set the Is Front object
   * 
   * @param is_front 
   */
  void SetIsFront(const bool is_front) { is_front_ = is_front; }

  /**
   * @brief Get the Is Front object
   * 
   * @param is_front 
   */
  bool GetIsFront() const { return is_front_; }

 private:
  /**
   * @brief Cache obstacle speed
   *
   * @param path_data
   */
  void CacheObstacleInfos(const ReferenceLineInfo& reference_line_info);

 private:
  // obstacle pointer
  const Obstacle* obstacle_ = nullptr;
  // obstacle info in st graph time
  std::vector<ObstacleInfo> obstacle_infos_;
  const double time_unit_ = 0.2;
  // time count from 0s to st graph time length
  int st_graph_time_count_ = 0;
  // min stop distance
  double min_stop_distacne_ = 0.0;
  //
  double max_t_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  // whether this obstacle cut out
  bool is_cut_out_ = false;

 private:
  bool is_front_ = true;
};

}  // namespace speed_evaluator
}  // namespace planning
}  // namespace TL
