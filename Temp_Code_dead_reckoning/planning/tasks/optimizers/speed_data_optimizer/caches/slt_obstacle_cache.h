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
#include <string>
#include <utility>
#include <vector>

#include "common/interpolation/interpolation_2d.h"
#include "planning/common/obstacle.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/st_obstacle_cache.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class ObstacleCache
 *
 * @brief cache obstacle info for slt obstacle
 */
class SLTObstacleCache {
 public:
  struct ObstacleInfo {
    // obstacle t
    double t = 0.0;
    // obstacle s speed
    double ds = 0.0;
    // obstacle l speed
    double dl = 0.0;
    // obstacle s accel
    double dds = 0.0;
    // obstacle l accel
    double ddl = 0.0;
    // obstacle boundary s_lower
    double s_lower = std::numeric_limits<double>::max();
    // obstacle boundary s_upper
    double s_upper = std::numeric_limits<double>::lowest();
    // obstacle boundary l_lower
    double l_lower = std::numeric_limits<double>::max();
    // obstacle boundary l_upper
    double l_upper = std::numeric_limits<double>::lowest();
  };

  SLTObstacleCache(const SpeedCacheConfig& config,
                   const ReferenceLineInfo& reference_line_info,
                   const Obstacle* obstacle, double time_unit, int time_count,
                   int index);

  virtual ~SLTObstacleCache() = default;

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
  [[nodiscard]] int GetTimeCount() const { return time_count_; }

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
        0, time_count_ - 1);
    return obstacle_infos_[index];
  }

  /**
   * @brief Get obstacle info at time
   *
   * @param time
   * @return const ObstacleInfo&
   */
  [[nodiscard]] const ObstacleInfo& GetObstacleInfoAtTimeCeil(
      const double time) const {
    const auto index = common::math::Clamp(
        static_cast<int>(ceil(time / time_unit_)), 0, time_count_ - 1);
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
  [[nodiscard]] double GetMinStopDistance() const { return min_stop_distance_; }

  /**
   * @brief Get min obstacle follow distance
   *
   * @return double
   */
  [[nodiscard]] double GetMinFollowDistance() const {
    return min_follow_distance_;
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
   * @brief Get the Max T object
   * 
   * @return double 
   */
  [[nodiscard]] double GetTimeUnit() const { return time_unit_; }

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

  /**
   * @brief Get the Is Cut In object
   * 
   * @return true 
   * @return false 
   */
  [[nodiscard]] bool GetIsCutIn() const { return is_cut_in_; }

  /**
   * @brief Set the Is Cut In object
   * 
   * @param is_cutin_when_lane_keep 
   */
  void SetIsCutIn(const bool is_cut_in) { is_cut_in_ = is_cut_in; }

  /**
   * @brief Get the Is Big Car object
   * 
   * @param is_front 
   */
  double GetCutInSafty() const { return cut_in_safty_; }

  /**
   * @brief Set the Is Big car object
   * 
   * @param is_front 
   */
  void SetCutInSafty(const double cut_in_safty) {
    cut_in_safty_ = cut_in_safty;
  }

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
   * @brief Get st location in st graph
   * 
   * @return const STLocation& 
   */
  [[nodiscard]] const LTObstacleLocation& GetLTObstacleLocation() const {
    return lt_obstacle_location_;
  }

  /**
   * @brief Set st location in st graph
   * 
   * @param st_location 
   */
  void SetLTObstacleLocation(const LTObstacleLocation lt_obstacle_location) {
    lt_obstacle_location_ = lt_obstacle_location;
  }

  /**
   * @brief Get safe distance costs
   * 
   * @return const std::vector<double>& 
   */
  const std::vector<double>& GetSSafeDistanceCosts() const {
    return s_safe_distance_costs_;
  }

  /**
   * @brief Set safe distance costs
   * 
   * @param safe_distance_costs 
   */
  void SetSSafeDistanceCosts(const std::vector<double>& s_safe_distance_costs) {
    s_safe_distance_costs_ = s_safe_distance_costs;
  }

  /**
   * @brief Get safe distance costs
   * 
   * @return const std::vector<double>& 
   */
  const std::vector<double>& GetLSafeDistanceCosts() const {
    return l_safe_distance_costs_;
  }

  /**
   * @brief Set safe distance costs
   * 
   * @param safe_distance_costs 
   */
  void SetLSafeDistanceCosts(const std::vector<double>& l_safe_distance_costs) {
    l_safe_distance_costs_ = l_safe_distance_costs;
  }

  /**
   * @brief 
   * 
   * @param frame 
   * @param reference_line_info 
   */
  void CheckIfCutInBegin(const Frame& frame,
                         const ReferenceLineInfo& reference_line_info);

  /**
   * @brief Check if obstacle perception sl is on reference line
   * 
   * @param reference_line_info 
   * @return true 
   * @return false 
   */
  bool IsPerceptionSLOnReferenceLine(
      const ReferenceLineInfo& reference_line_info);

  /**
   * @brief Check if obstacle trajectory is on reference line
   * 
   * @param frame 
   * @param reference_line_info 
   * @return true 
   * @return false 
   */
  bool IsTrajectoryOnReferenceLine(
      const Frame& frame, const ReferenceLineInfo& reference_line_info);

  /**
   * @brief Estimate obstacle infos for follow target 
   * 
   * @param jerk_value_calibration_table 
   * @param jerk_coef_calibration_table 
   */
  void EstimateObstacleInfos(
      const Frame& frame, const ReferenceLineInfo& reference_line_info,
      const std::unique_ptr<TL::common::Interpolation2D>&
          jerk_interpolation_ptr);

  /**
   * @brief Get index
   * 
   * @return const std::string& 
   */
  [[nodiscard]] int GetIndex() const { return index_; }

  /**
   * @brief 
   * 
   * @param index 
   */
  void UpdateIndex(const int index) { index_ = index; }

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
   * @brief Get cost weight
   * 
   * @return double 
   */
  [[nodiscard]] double GetCostWeight() const { return cost_weight_; }

  /**
   * @brief Set cost weight
   * 
   * @param cost_weight 
   */
  void SetCostWeight(const double cost_weight) { cost_weight_ = cost_weight; }

  bool Forecast(const Obstacle* obstacle, double time);

  bool GetIsIgnoreCollision() const { return is_ignore_collision_; }

  void SetIsIgnoreCollision(const bool is_ignore_collision) {
    is_ignore_collision_ = is_ignore_collision;
  }

  /**
   * @brief 
   * 
   * @return double 
   */
  [[nodiscard]] double StTimeLengthCoef() const { return st_time_length_coef_; }

  [[nodiscard]] double GetMinL() const { return min_l_; }

  [[nodiscard]] double GetMaxL() const { return max_l_; }

 private:
  /**
   * @brief Cache obstacle speed
   *
   * @param path_data
   */
  void CacheObstacleInfos(const ReferenceLineInfo& reference_line_info);
  /**
   * @brief CalculateStTimeLengthCoef
   * 
   */
  void CalculateStTimeLengthCoef();

  void CalculateLRange();

 private:
  int index_ = 0;
  // obstacle id
  std::string id_;
  // obstacle perception id
  int32_t perception_id_ = -1;
  // obstacle pointer
  const Obstacle* obstacle_ = nullptr;
  // sample obstacle infos
  std::vector<ObstacleInfo> obstacle_infos_;
  // sample time unit
  const double time_unit_ = 0.2;
  // sample time count
  int time_count_ = 0;
  // min stop distance
  double min_stop_distance_ = 0.0;
  // min follow distance_
  double min_follow_distance_ = 0.0;
  // sample time range
  double max_t_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  // whether this obstacle cut in
  bool is_cut_in_ = false;
  // whether this obstacle is safe cut in
  double cut_in_safty_ = 0.0;
  // whether  this obstacle is front
  bool is_front_ = true;
  // st obstacle location
  STObstacleLocation st_obstacle_location_ = STObstacleLocation::UNKNOWN;
  // lt obstacle location
  LTObstacleLocation lt_obstacle_location_ = LTObstacleLocation::UNKNOWN;
  // s_safe distance costs for safe curve
  std::vector<double> s_safe_distance_costs_;
  // s_safe distance costs for safe curve
  std::vector<double> l_safe_distance_costs_;
  // cost weight
  double cost_weight_ = 1.0;
  bool is_ignore_collision_ = false;
  double st_time_length_coef_ = 1.0;
  double min_l_ = -std::numeric_limits<double>::infinity();
  double max_l_ = std::numeric_limits<double>::infinity();
};

}  // namespace planning
}  // namespace TL
