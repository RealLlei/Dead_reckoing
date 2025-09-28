/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>
// #include <stdlib.h>
#include "common/interpolation/interpolation_2d.h"
#include "common/status/status.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_line_delay.h"
#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
using TL::common::Interpolation2D;
using TL::common::Status;
using thrbools = std::tuple<bool, bool, bool>;

// enum PredictState { Init, Update, Coast, LftChange, RgtChange };
class LaneWidthPredictor {
 public:
  LaneWidthPredictor() = default;
  explicit LaneWidthPredictor(const planning::PerceptionMapConfig& config);
  ~LaneWidthPredictor() = default;
  Status Init();
  void LoadDistanceCalibrationTable(
      const planning::LaneWidthPredictorConfig& lnewde_conf);
  void WdePredict(DeciderData* decider_data);
  void LeftLaneWdePredict(const LaneMarker& left_lanemarker,
                          const LaneMarker& right_lanemarker,
                          DeciderData* decider_data);
  void RightLaneWdePredict(const LaneMarker& left_lanemarker,
                           const LaneMarker& right_lanemarker,
                           DeciderData* decider_data);
  void EgoLaneWdePredict(const LaneMarker& left_lanemarker,
                         const LaneMarker& right_lanemarker,
                         DeciderData* decider_data);

 private:
  static bool LaneQualityTerrible(bool* state, bool lane_wide_variance_terrible,
                                  bool lane_wide_terrible);
  int NextLaneWidePredictValid(PredictState* state, double* timer,
                               bool predict_lane_wide_terrible,
                               double left_lane_quality,
                               double right_lane_quality,
                               bool host_lane_change) const;
  static double NextLaneWidePredictValue(PredictState* state,
                                         double now_lane_wide_value,
                                         int lane_wide_predict_valid,
                                         double last_lane_wide_value);
  static double CalculateLanemarkerY(double distance,
                                     const LaneMarker& lane_marker);
  int EgoLaneWidePredictValid(const double& left_lane_quality,
                              const double& right_lane_quality,
                              bool is_lane_wide_terrible,
                              bool next_right_lane_wide_predict_valid,
                              bool next_left_lane_wide_predict_valid);
  double EgoLaneWidePredictValue(double lane_wide, int lane_wide_valid);
  double EgoLaneWideLock(PredictState* state, bool is_lane_change_terrible,
                         double lane_wide_last_const, double lane_wide_last,
                         double left_quality, double right_quality) const;
  /**
   * @description: 计算车道线distance处headingangle
   * @return {*}
   */
  static double CalculateLanemarkerHeadingAngle(double distance,
                                                const LaneMarker& lane_marker);
  /**
   * @description: 计算distance处车道宽度
   * @return {*}
   */
  double CalculateLaneWidth(double distance, const LaneMarker& left_lane_marker,
                            const LaneMarker& right_lane_marker,
                            double* last_lane_wdith, bool left_lane_exist,
                            bool right_lane_exist);
  bool LaneWidthExtraWide(double lane_width);
  bool LaneWidthUltraNarrow(double lane_width);

  double good_quality_threshold_ = 0.3;
  double batter_quality_threshold_ = 0.6;
  double ego_lane_width_predict_distance_{0.0};
  double average_speed_{0.0};
  double main_loop_time_{0.1};
  double left_lane_wide_coast_time_{0.0};

  std::unique_ptr<Interpolation2D> distance_interpolation_ = nullptr;
  double left_lane_wide_predict_fir_timer_ = 0.0;
  double left_lane_wide_last_input_sg_ = 0.0;
  double right_lane_wide_predict_fir_timer_ = 0.0;
  double right_lane_wide_last_input_sg_ = 2 * kInitLaneWidth;
  double ego_lane_wide_last_input_ = 2 * kInitLaneWidth;
  double ego_lane_wide_predict_fir_timer_ = 0.0;
  double ego_lane_wide_last_sec_sg_ = 3.75;
  double next_right_lane_wide_predict_sec_sg_input_ = 3.75;
  double next_left_lane_wide_predict_sec_sg_input_ = 3.75;
  DebounceModule ego_debounce_module_;
  // predict left lane wide
  DebounceModule left_debounce_module_;
  // predict right lane wide
  DebounceModule right_debounce_module_;
  Delay<double> left_lane_left_line_quality_delay_;
  Delay<double> left_lane_right_line_quality_delay_;
  Delay<double> left_lane_wide_delay_;
  Delay<int> left_lane_wide_predict_valid_delay_;
  Delay<double> right_lane_left_line_quality_delay_;
  Delay<double> right_lane_right_line_quality_delay_;
  Delay<double> right_lane_wide_delay_;
  Delay<int> right_lane_wide_predict_valid_delay_;
  // predict ego lane wide
  Delay<double> ego_lane_left_line_quality_delay_;
  Delay<double> ego_lane_right_line_quality_delay_;
  Delay<double> ego_lane_wide_delta_delay_;
  Delay<bool> is_prelane_wide_change_terrible_;
  Delay<double> ego_lane_wide_delay_;
  // change_delay_ 0:left or right 1:left 2:right
  Delay<thrbools> change_delay_{};
  Delay<int> next_left_lane_predict_valid_delay_;
  Delay<int> next_right_lane_predict_valid_delay_;
  Delay<int> ego_lane_predict_valid_delay_;
  Delay<double> next_right_lane_predict_sg_delay_;
  Delay<double> next_left_lane_predict_sg_delay_;

  planning::PerceptionMapConfig config_;
  DeciderData decider_data_;
  PredictState left_lane_wide_predict_fir_state_ = PredictState::Init;
  PredictState left_lane_wide_predict_sec_state_ = PredictState::Init;
  PredictState right_lane_wide_predict_fir_state_ = PredictState::Init;
  PredictState right_lane_wide_predict_sec_state_ = PredictState::Init;
  PredictState ego_lane_lock_state_ = PredictState::Init;
  int next_left_lane_wide_predict_valid_record_{0};
  int next_right_lane_wide_predict_valid_record_{0};
  PredictState ego_lane_wide_predict_fir_state_{PredictState::Init};
  PredictState ego_lane_wide_predict_sec_state_ = PredictState::Init;
  bool left_lane_good_path_ = false;
  bool right_lane_good_path_ = false;
  bool ego_lanewde_is_nomal_monitor_state_ = true;
  double last_lane_wide_delta_ = 0.0;
  double last_ego_lane_wdith_ = 0.0;
  double last_left_lane_wdith_ = 0.0;
  double last_right_lane_wdith_ = 0.0;
  common::VehicleParam vehicle_param_;
  DebounceModule ego_left_lane_exist_debounce_module_;
  DebounceModule ego_right_lane_exist_debounce_module_;
  DebounceModule left_left_lane_exist_debounce_module_;
  DebounceModule left_right_lane_exist_debounce_module_;
  DebounceModule right_left_lane_exist_debounce_module_;
  DebounceModule right_right_lane_exist_debounce_module_;
};
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
