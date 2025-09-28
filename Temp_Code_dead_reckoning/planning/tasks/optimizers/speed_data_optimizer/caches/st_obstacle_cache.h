/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_cost.h
 **/

#pragma once

#include <algorithm>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/interpolation/interpolation_2d.h"
#include "planning/common/frame.h"
#include "planning/common/obstacle.h"
#include "planning/common/path/path_data.h"
#include "planning/common/reference_line_info.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

enum STObstacleLocation {
  UNKNOWN = 0,
  ABOVE = 1,
  BELOW = 2,
  CROSS = 3,
};

using LTObstacleLocation = STObstacleLocation;

/**
 * @class ObstacleCache
 *
 * @brief cache obstacle info st obstacle
 */
class STObstacleCache {
 public:
  struct ObstacleInfo {
    // obstacle longitudinal speed
    double v = 0.0;
    // obstacle longitudinal acceleration
    double a = 0.0;
    // obstacle boundary s_lower
    double s_lower = std::numeric_limits<double>::max();
    // obstacle boundary s_upper
    double s_upper = std::numeric_limits<double>::lowest();
  };

  STObstacleCache(const SpeedCacheConfig& config,
                  const ReferenceLineInfo& reference_line_info,
                  const Obstacle* obstacle, double time_unit,
                  int st_graph_time_count);

  virtual ~STObstacleCache() = default;

  /**
   * @brief Get obstacle info at time
   *
   * @param time
   * @return const ObstacleInfo&
   */
  [[nodiscard]] const ObstacleInfo& GetObstacleInfoAtTime(
      const double time) const {
    const auto index = common::math::Clamp(
        static_cast<int>(time / time_unit_ +
                         common::math::double_type::epslion),
        0, st_graph_time_count_ - 1);
    return obstacle_infos_[index];
  }

  /**
   * @brief Get obstacle
   *
   * @return const Obstacle*
   */
  [[nodiscard]] const Obstacle* GetObstacle() const { return obstacle_; }

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
   * @brief Get the Max T object
   * 
   * @return double 
   */
  [[nodiscard]] double GetTimeUnit() const { return time_unit_; }

  /**
   * @brief Get min obstacle stop distance
   *
   * @return double
   */
  [[nodiscard]] double GetMinStopDistance() const { return min_stop_distacne_; }

  /**
   * @brief Get collision check distance buffer, if obstacle trajectory cross
   * reference line, we give a big buffer, otherwise we give s small buffer
   *
   * @return double
   */
  [[nodiscard]] double GetCollisionCheckDistanceBuffer() const {
    return collision_check_distance_buffer_;
  }

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
   * @brief Get the Is Ignore object
   * 
   * @return true 
   * @return false 
   */
  [[nodiscard]] bool GetIsIgnore() const { return is_ignore_; }

  /**
   * @brief Get the Is Ignore object
   * 
   * @return true 
   * @return false 
   */
  void SetIsIgnore(bool is_ignore) { is_ignore_ = is_ignore; }

  /**
   * @brief Get st location in st graph
   * 
   * @return const STLocation& 
   */
  [[nodiscard]] const STObstacleLocation& GetSTObstacleLocation() const {
    return st_obstacle_location_;
  }

  /**
   * @brief Set st location in st graph
   * 
   * @param st_location 
   */
  void SetSTObstacleLocation(const STObstacleLocation st_obstacle_location) {
    st_obstacle_location_ = st_obstacle_location;
  }

  /**
   * @brief Estimate obstacle infos for follow target 
   * 
   * @param jerk_value_calibration_table 
   * @param jerk_coef_calibration_table 
   */
  void EstimateObstacleInfos(
      const Frame& frame, const std::unique_ptr<TL::common::Interpolation2D>&
                              jerk_interpolation_ptr);

  void ModifyObstacleInfos(const Frame& frame, double accel, double jerk,
                           bool extended = false);

  /**
   * @brief Get the Is Front object
   * 
   * @param is_front 
   */
  bool GetIsFront() const { return is_front_; }

  /**
   * @brief Set the Is Front object
   * 
   * @param is_front 
   */
  void SetIsFront(const bool is_front) { is_front_ = is_front; }

  /**
   * @brief Get obstacle id
   * 
   * @return const std::string& 
   */
  [[nodiscard]] const std::string& GetId() const { return id_; }

  /**
   * @brief Get perception id
   * 
   * @return int32_t 
   */
  [[nodiscard]] int32_t GetPerceptionId() const { return perception_id_; }

  /**
   * @brief Set enable collision check
   * 
   * @return true 
   * @return false 
   */
  [[nodiscard]] bool GetEnableCollisionCheck() const {
    return enable_collision_check_;
  }

  void SetEnableCollisionCheck(const bool enable_collision_check) {
    enable_collision_check_ = enable_collision_check;
  }

  /**
   * @brief Get slt index
   * 
   * @return int 
   */
  int GetSLTIndex() const { return slt_index_; }

  /**
   * @brief Set slt index
   * 
   * @param slt_index 
   */
  void SetSLTIndex(const int slt_index) { slt_index_ = slt_index; }

 private:
  /**
   * @brief Cache obstacle speed
   *
   * @param path_data
   */
  void CacheObstacleInfos(const ReferenceLineInfo& reference_line_info,
                          const PathData& path_data);

  int slt_index_ = 0;
  // obstacle id
  std::string id_;
  // obstacle perception id
  int32_t perception_id_ = -1;
  // obstacle pointer
  const Obstacle* obstacle_ = nullptr;
  // obstacle info in st graph time
  std::vector<ObstacleInfo> obstacle_infos_;
  const double time_unit_ = 0.2;
  // min stop distance
  double min_stop_distacne_ = 0.0;
  // collision check distance buffer
  double collision_check_distance_buffer_ = 0.0;
  // min t and max t of st boundary
  double max_t_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  // time count from 0s to st graph time length
  int st_graph_time_count_ = 0;
  // whether ignore this obstacle
  bool is_ignore_ = false;
  // st location
  STObstacleLocation st_obstacle_location_ = STObstacleLocation::UNKNOWN;
  // whether this obstacle is front obstacle
  bool is_front_ = true;
  // whether this obstacle is enable collision check
  bool enable_collision_check_ = true;
};

}  // namespace planning
}  // namespace TL
