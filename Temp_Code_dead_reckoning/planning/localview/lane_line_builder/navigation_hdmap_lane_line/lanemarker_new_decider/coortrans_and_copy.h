/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#pragma once
#include <cmath>
#include <tuple>
#include <utility>

#include "common/status/status.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_line_delay.h"
#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
using TL::common::Status;
using TL::perception::LaneMarker;

using LanemarkerTuple =
    std::tuple<LaneMarker, LaneMarker, double*, bool*, int, Delay<double>>;
using LanemarkerState = std::tuple<LaneMarkerState, LaneMarkerState,
                                   LaneMarkerState, LaneMarkerState>;

// LaneMarker, history_LaneMarker, timer, state, bridgecopy_type for BridgeCopy
// function bridgecopy_type, left:1  right:-1  next_left:2  next_right:-2
class CoorTransAndCopy {
 public:
  CoorTransAndCopy() = default;
  explicit CoorTransAndCopy(const planning::PerceptionMapConfig& config);
  ~CoorTransAndCopy() = default;
  Status Init();
  bool LaneMarkerCoorTrans(DeciderData* decider_data);
  void EgoLaneMarkerCopy(DeciderData* decider_data);
  void NextLaneMarkerCopy(DeciderData* decider_data);
  static void TimerScheduler(DeciderData* decider_data);

 private:
  bool LaneCoordinateTransformation(const LaneMarker& lanemarker_input,
                                    LaneMarker* lanemarker_output) const;
  static double RoadCurvatureCalculate(const LaneMarker& lane_marker,
                                       double length);
  /**
   * @brief lane line BridgeCopy,copy bad lanemarker using good lanemarker
   *
   * @param BridgeCopyTuple
   * @param good_lanemarker
   * @param copy_lanemarker
   * @param lanewidth_prediction_valid
   * @param copy_flag
   * @param lanemarker_state
   * @return true
   * @return false
   */
  bool BridgeCopy(LanemarkerTuple* BridgeCopyTuple,
                  const LaneMarker& good_lanemarker,
                  LaneMarker* copy_lanemarker, int lanewidth_prediction_valid,
                  std::pair<bool, bool>* copy_flag,
                  const LanemarkerState& lanemarker_state);
  bool CopyDecider(bool* state, double* timer, bool is_trigger_copy,
                   int lane_wide_predict_valid, double good_lane_quality,
                   double good_lane_start, double bad_lane_quality,
                   const LanemarkerState& lanemarker_state) const;
  //   double CalculateLanemarkerY(const double distance,
  //                               const LaneMarker& lane_marker);
  DeciderData decider_data_;
  planning::PerceptionMapConfig config_;
  double good_quality_threshold_ = 0.3;
  double batter_quality_threshold_ = 0.6;
  double max_allow_copy_lane_time_ = 0.0;
  double lanemarker_back_length_ = 0.0;
  double camera_position_offset_ = 0.0;
  LaneMarker history_left_lanemarker_;
  LaneMarker history_right_lanemarker_;
  LaneMarker history_next_left_lanemarker_;
  LaneMarker history_next_right_lanemarker_;
  Delay<double> ego_lanewidth_prediction_last_;
  Delay<double> left_lanewidth_prediction_last_;
  Delay<double> right_lanewidth_prediction_last_;
  double left_timer_ = 0.0;
  double right_timer_ = 0.0;
  double next_left_timer_ = 0.0;
  double next_right_timer_ = 0.0;
  double main_loop_time_ = 0.1;
  bool left_copy_state_ = false;
  bool right_copy_state_ = false;
  bool next_left_copy_state_ = false;
  bool next_right_copy_state_ = false;
  LaneMarkersState history_lanemarkers_state_{BAD_LANEMARKER, BAD_LANEMARKER,
                                              BAD_LANEMARKER, BAD_LANEMARKER};
  DebounceModule left_good_lanemaker_quality_debounce_{0.5, 1.0, 0.1};
  DebounceModule right_good_lanemaker_quality_debounce_{0.5, 1.0, 0.1};
};
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
