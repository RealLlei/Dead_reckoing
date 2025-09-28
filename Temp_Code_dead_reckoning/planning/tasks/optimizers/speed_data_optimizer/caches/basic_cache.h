/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_cost.h
 **/

#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/filters/mean_filter.h"
#include "planning/common/dependency_injector.h"
#include "planning/common/frame.h"
#include "planning/common/reference_line_info.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/st_drivable_boundary.pb.h"
#include "planning/proto/task_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/planning/planning_status.pb.h"

namespace TL {
namespace planning {
namespace speed_evaluator {

/**
 * @brief basic cache is used to cache common info
 *
 */
class BasicCache {
 public:
  /**
   * @brief Init basic cache
   *
   * @param config speed cache config
   * @param frame current frame
   * @param reference_line_info current reference line info
   * @param last_frame last frame
   * @param distance_from_last_frame distance from last frame
   */
  void Init(const SpeedCacheConfig& config,
            const std::shared_ptr<DependencyInjector>& injector,
            const Frame& frame, const ReferenceLineInfo& reference_line_info,
            const common::TrajectoryPoint& init_point);

  /**
   * @brief Get last frame block obstacle id
   *
   * @return const std::string& 
   */
  [[nodiscard]] const std::string& GetLastFrameBlockObstacleId() const {
    return last_frame_block_obstacle_id_;
  }

  /**
   * @brief Get current frame block obstacle id
   *
   * @return const std::string& 
   */
  [[nodiscard]] const std::string& GetBlockObstacleId() const {
    return block_obstacle_id_;
  }

  /**
   * @brief Get stop slide time threshold
   *
   * @return double
   */
  [[nodiscard]] double GetStopSlideTimeThreshold() const {
    return stop_slide_time_threshold_;
  }

  /**
   * @brief Get real stop s
   *
   * @return double
   */
  [[nodiscard]] double GetRealStopS() const { return real_stop_s_; }

  /**
   * @brief Set real stop s
   *
   * @return 
   */
  void SetRealStopS(const double real_stop_s) { real_stop_s_ = real_stop_s; }

  /**
   * @brief Get expected stop s
   *
   * @return double
   */
  [[nodiscard]] double GetExpectedStopS() const { return expected_stop_s_; }

  /**
   * @brief Set expected stop s
   *
   * @return double
   */
  void SetExpectedStopS(const double expected_stop_s) {
    expected_stop_s_ = expected_stop_s;
  }

  /**
   * @brief Get nearest stop s
   *
   * @return double
   */
  [[nodiscard]] double GetNearestStopS() const { return nearest_stop_s_; }

  /**
   * @brief Get farthest_stop_s
   *
   * @return double
   */
  [[nodiscard]] double GetFarthestStopS() const { return farthest_stop_s_; }

  /**
   * @brief Get is change lane path
   *
   * @return true is change lane path
   * @return false is not change lane path
   */
  [[nodiscard]] bool GetIsChangeLanePath() const {
    return is_change_lane_path_;
  }

  /**
   * @brief Set is change lane path
   *
   * @return 
   */
  void SetIsChangeLanePath(const bool is_change_lane_path) {
    is_change_lane_path_ = is_change_lane_path;
  }

  /**
   * @brief Get is change lane path
   *
   * @return true is change lane path
   * @return false is not change lane path
   */
  [[nodiscard]] bool GetIsChangeLaneReturnPath() const {
    return is_change_lane_return_path_;
  }

  /**
   * @brief Set is change lane path
   *
   * @return 
   */
  void SetIsChangeLaneReturnPath(const bool is_change_lane_return_path) {
    is_change_lane_return_path_ = is_change_lane_return_path;
  }

  /**
   * @brief Get is change lane path
   *
   * @return true is change lane path
   * @return false is not change lane path
   */
  [[nodiscard]] bool GetIsLastChangeLanePath() const {
    return is_last_change_lane_path_;
  }

  /**
   * @brief Set is change lane path
   *
   * @return 
   */
  void SetIsLastChangeLanePath(const bool is_last_change_lane_path) {
    is_last_change_lane_path_ = is_last_change_lane_path;
  }

  /**
   * @brief Get high road right end position
   *
   * @return double
   */
  [[nodiscard]] double GetHighRoadRightEndS() const {
    return high_road_right_end_s_;
  }

  /**
   * @brief Get low road right end position
   *
   * @return double
   */
  [[nodiscard]] double GetLowRoadRightEndS() const {
    return low_road_right_end_s_;
  }

  /**
   * @brief Get has low road right 
   *
   * @return double
   */
  [[nodiscard]] bool GetHasLowRoadRight() const { return has_low_road_right_; }

  /**
   * @brief Get min speed for wait gap
   *
   * @return double
   */
  [[nodiscard]] double GetMinSpeedForWaitGap() const {
    return min_speed_for_wait_gap_;
  }

  /**
   * @brief Get the Is Forward Path object
   * 
   * @return true 
   * @return false 
   */
  [[nodiscard]] bool GetIsForwardPath() const { return is_forward_path_; }

  /**
   * @brief Get whether has pedestrian
   * 
   * @return true 
   * @return false 
   */
  [[nodiscard]] bool GetHasPedestrian() const { return has_pedestrian_; }

  /**
   * @brief Get the Last Frame object
   * 
   * @return const Frame* 
   */
  const Frame* GetLastFrame() const { return last_frame_; }

  /**
   * @brief Get the Time From Last Frame object
   * 
   * @return double 
   */
  double GetTimeFromLastFrame() const { return time_from_last_frame_; }

  /**
   * @brief Get the Distance From Last Frame object
   * 
   * @return double 
   */
  double GetDistanceFromLastFrame() const { return distance_from_last_frame_; }

  /**
   * @brief Get the Distance From Last Frame object
   * 
   * @return double 
   */
  bool GetIsStableFollow() const { return is_stable_follow_; }

  /**
   * @brief Get the Distance From Last Frame object
   * 
   * @return double 
   */
  void SetIsStableFollow(bool is_stable_follow) {
    is_stable_follow_ = is_stable_follow;
  }

  /**
   * @brief Get the Is Stop Prefinish object
   * 
   * @return true 
   * @return false 
   */
  [[nodiscard]] bool GetIsStopPrefinish() const { return is_stop_prefinish_; }

  void SetAccelLimitTable(const std::vector<double>& accel_speeds,
                          const std::vector<double>& accels,
                          const std::vector<double>& decel_speeds,
                          const std::vector<double>& decels);

  void SetJerkLimitTable(const std::vector<double>& speeds,
                         const std::vector<double>& jerk);

  const std::pair<double, double>& GetAccelLimit(const double speed) const {
    const auto index =
        common::math::Clamp(static_cast<int>(speed / speed_epsilon_), 0,
                            static_cast<int>(accel_limit_table_.size() - 1));
    return accel_limit_table_.at(index);
  }

  /**
   * @brief Get smoothed accel limit
   * 
   * @param speed 
   * @return std::pair<double, double> 
   */
  std::pair<double, double> GetSmoothedAccelLimit(double speed) const;

  const std::pair<double, double>& GetJerkLimit(const double speed) const {
    const auto index =
        common::math::Clamp(static_cast<int>(speed / speed_epsilon_), 0,
                            static_cast<int>(jerk_limit_table_.size() - 1));
    return jerk_limit_table_.at(index);
  }

  /**
   * @brief Get smoothed jerk limit
   * 
   * @param speed 
   * @return std::pair<double, double> 
   */
  std::pair<double, double> GetSmoothedJerkLimit(double speed) const;

 private:
  /**
  * @brief Calculate time and distance from last frame
  * 
  * @param injector 
  * @param frame 
  */
  void CalculateInfoFromLastFrame(
      const std::shared_ptr<DependencyInjector>& injector, const Frame& frame);

  /**
   * @brief Calculate expected stop s
   *
   * @param config speed cache config
   * @param frame current frame
   * @param reference_line_info current reference line info
   */
  void CalculateExpectedStopS(const SpeedCacheConfig& config,
                              const Frame& frame,
                              const ReferenceLineInfo& reference_line_info,
                              const common::TrajectoryPoint& init_point);

  void CalculateStopBuffer(const common::TrajectoryPoint& init_point,
                           double stop_s, bool is_avp_mode,
                           const SpeedCacheConfig& config);

  bool CheckIfStopPostionFarAwayFromVehicle(
      const Frame& frame, const common::TrajectoryPoint& init_point) const;

  /**
   * @brief Calculate road right range
   *
   * @param reference_line_info current reference line info
   */
  void CalculateRoadRightRange(const ReferenceLineInfo& reference_line_info);

  // real stop position
  double real_stop_s_ = std::numeric_limits<double>::infinity();
  // expected stop position
  double expected_stop_s_ = std::numeric_limits<double>::infinity();
  // acceptable nearest stop s
  double nearest_stop_s_ = std::numeric_limits<double>::infinity();
  // acceptable farthest stop s
  double farthest_stop_s_ = std::numeric_limits<double>::infinity();
  // stop_s_buffer used reduce braking force in stop case
  double stop_s_buffer_ = 0.0;
  // adc speed when begin stop
  double stop_slide_time_threshold_ = 0.0;
  // high_road_right_end_s_ is the pre point of the first low road path point
  double high_road_right_end_s_ = std::numeric_limits<double>::max();
  // low_road_right_end_s_ is the last low road right path point
  double low_road_right_end_s_ = std::numeric_limits<double>::lowest();
  // min speed when wait for gap
  double min_speed_for_wait_gap_ = 5.0;
  // last frame pointer
  const Frame* last_frame_ = nullptr;
  // time from last frame
  double time_from_last_frame_ = 0.0;
  // distance from last frame
  double distance_from_last_frame_ = 0.0;
  // last frame block obstacle id
  std::string last_frame_block_obstacle_id_;
  // block obstacle id
  std::string block_obstacle_id_;
  // is stop prefinish
  bool is_stop_prefinish_ = false;
  // whether last path data is change lane path
  bool is_last_change_lane_path_ = false;
  // whether this path data is change lane path
  bool is_change_lane_path_ = false;
  // whether last path data is change lane path
  bool is_change_lane_return_path_ = false;
  // has_low_road_right_ means whether this path data has low road right part
  bool has_low_road_right_ = false;
  // whether this path is forward path
  bool is_forward_path_ = true;
  // whether has pedestrian
  bool has_pedestrian_ = false;
  // is_stable_follow_
  bool is_stable_follow_ = true;
  //
  const double speed_epsilon_ = 0.5;
  //
  std::array<std::pair<double, double>, 80> accel_limit_table_;
  //
  std::array<std::pair<double, double>, 80> jerk_limit_table_;
  //
  std::vector<double> accel_limit_speeds_;
  std::vector<double> accel_limits_;
  std::vector<double> decel_limit_speeds_;
  std::vector<double> decel_limits_;
  std::vector<double> jerk_limit_speeds_;
  std::vector<double> jerk_limits_;

  std::pair<std::vector<double>, std::vector<double>>
      stop_average_accel_thresholds_;
};

}  // namespace speed_evaluator
}  // namespace planning
}  // namespace TL
