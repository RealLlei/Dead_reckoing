/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#pragma once
#include <deque>
#include <memory>
#include <vector>

#include "Eigen/LU"
#include "common/filters/mean_filter.h"
#include "common/interpolation/interpolation_1d.h"
#include "common/math/linear_interpolation.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "planning/common/planning_gflags.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/pnc_map/pnc_map.h"
#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
using TL::common::Interpolation1D;
using TL::common::Status;

class LaneMarkerFilter {
 public:
  LaneMarkerFilter() = default;
  explicit LaneMarkerFilter(const planning::PerceptionMapConfig& config);
  ~LaneMarkerFilter() = default;
  Status Init();
  void LoadKalmanGainScheduler(
      const planning::LanemarkerFilterConfig& filter_conf);
  /**
   * @brief left lanemarker filter
   *
   * @param decider_data all data for lanemarker decider
   */
  void LeftLaneFilter(DeciderData* decider_data);
  /**
   * @brief right lanemarker filter
   *
   * @param decider_data
   */
  void RightLaneFilter(DeciderData* decider_data);
  /**
   * @brief next left lanemarker filter
   *
   * @param decider_data
   */
  void NLeftLaneFilter(DeciderData* decider_data);
  /**
   * @brief next right lanemarker filter
   *
   * @param decider_data
   */
  void NRightLaneFilter(DeciderData* decider_data);
  /**
   * @brief compute common data
   *
   * @param decider_data
   */
  void ComputeFilterData(const DeciderData& decider_data);

 private:
  bool DoPredictUpdate(double view_range, std::vector<Vec2d>* history_esti,
                       Eigen::MatrixXd* pre_points_x,
                       Eigen::MatrixXd* pre_points_y);
  static bool DoMeasureUpdate(const LaneMarker& copy_lanemarker,
                              const Eigen::MatrixXd& pre_points_x,
                              Eigen::MatrixXd* measure_points_y);
  bool DoKalmanGainUpdate(const Eigen::MatrixXd& pre_points_x,
                          const Eigen::MatrixXd& pre_points_y,
                          const Eigen::MatrixXd& measure_points_y,
                          double view_range, bool reset, double speed,
                          bool splitlane_flag, bool lane_change_flag,
                          std::vector<Vec2d>* esti_points_y,
                          Eigen::MatrixXd* kalman_gain);
  void CalculateDxyphi(double input_spd, double input_yaw_rate, double ts);
  bool AdjustEstiPoints(double view_range, std::vector<Vec2d>* history_esti);
  static bool AdjustEstiPointsFromExtend(double start_range, double end_range,
                                         std::vector<Vec2d>* points);
  static bool AdjustEstiPointsFromFit(double start_range, double end_range,
                                      double step, std::vector<Vec2d>* points);
  void CreatInitLaneMarkerPoints(const LaneMarker& input_lanemarker,
                                 std::vector<Vec2d>* history_esti);
  static bool Interpolation(
      std::vector<TL::common::math::Vec2d>* esti_points);
  /**
   * @brief Set the Debug Info object
   *
   * @param filter_debug
   * @param history_esti history points
   * @param pre_points_x
   * @param pre_points_y
   * @param measure_points_y
   * @param kalman_gain
   * @param has_history_points
   */
  static void SetDebugInfo(LanePointsFilter* filter_debug,
                           const std::vector<Vec2d>& history_esti,
                           const Eigen::MatrixXd& pre_points_x,
                           const Eigen::MatrixXd& pre_points_y,
                           const Eigen::MatrixXd& measure_points_y,
                           const Eigen::MatrixXd& kalman_gain,
                           bool has_history_points);

  DeciderData decider_data_;
  planning::PerceptionMapConfig config_;
  LaneMarkers history_out_lane_marker_;
  std::vector<Vec2d> left_history_esti_{};
  std::vector<Vec2d> right_history_esti_{};
  std::vector<Vec2d> next_left_history_esti_{};
  std::vector<Vec2d> next_right_history_esti_{};
  //   double step_point_dis_;
  std::unique_ptr<Interpolation1D> length_interpolation_{nullptr};
  std::unique_ptr<Interpolation1D> diff_length_interpolation_{nullptr};
  bool has_no_right_history_points_ = false;
  bool has_no_left_history_points_ = false;
  bool has_no_next_right_history_points_ = false;
  bool has_no_next_left_history_points_ = false;
  double dx_{0.0};
  double dy_{0.0};
  double dphi_{0.0};
  double history_yaw_rate_{0.0};
};
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
