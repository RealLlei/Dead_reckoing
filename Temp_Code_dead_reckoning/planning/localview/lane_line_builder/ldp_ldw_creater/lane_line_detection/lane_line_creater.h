/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#pragma once
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include "common/status/status.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/local_view.h"
#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
using TL::common::Status;

enum LaneLineCase { NoneLine, LeftGood, RightGood, BothGood };

class LaneLineMarkerDetection {
 public:
  // friend class LdwCreater;
  LaneLineMarkerDetection();
  explicit LaneLineMarkerDetection(const PerceptionMapConfig& config);
  ~LaneLineMarkerDetection() = default;

  Status Init();
  Status Start();
  void Stop();

  void SetLaneMg(const std::shared_ptr<LocalView>& local_view);

  bool ODDCorridorLengthDetection(double start_range, double end_range);
  bool ODDLaneLineQualityDetectin(double line_quality);
  bool ODDLaneLineCurvatureDetection(double line_a2);
  bool ODDRideLineDetection(double width, double a0, bool line_quality,
                            bool change_line, bool* odd_ride_line_condition);
  bool LeftODDRideLineDetection(double width, double a0, bool line_quality,
                                bool change_line);
  bool RightODDRideLineDetection(double width, double a0, bool line_quality,
                                 bool change_line);
  bool ODDLaneLineParamJump(double lookahead_curvature,
                            double lookahead_curvature_last, double a1,
                            double a1_last, double a0, double a0_last,
                            bool quality, bool quality_last);
  bool ODDCorridorWidthDection(double ego_lane_width);
  double ODDLaneLineParamDerviation(double current, double last);
  double ODDLaneWidthCalc(double vehicle_width, double left_a0, double left_a1,
                          bool left_quality, double right_ao, double right_a1,
                          bool right_quality);
  uint16_t ODDLaneLineProcess(const std::shared_ptr<LocalView>& local_view);
  void ODDLaneLineDelay(
      const std::shared_ptr<const perception::LaneMarkers> & lane_marker);
  double LookAheadCurvature(double a2, double a3) const;
  double SaturationDynamicLimit(const double& x, const double& low,
                                const double& high);
  void ODDLaneLineCondition(bool left_q, bool right_q);
  bool VehicleHeadingDection(double a1, bool quality);

 private:
//   std::shared_ptr<const perception::PerceptionObstacles>
//       perception_obstaclesLine_{nullptr};
//   std::shared_ptr<const perception::LaneMarkers> lane_marker_{nullptr};
  PerceptionMapConfig navi_hdmap_config_;
  common::VehicleParam vehicle_paramline_;
  LaneLineCase lanelinecase_ = BothGood;
  bool odd_line_curvature_is_enabel_last_ = true;
  bool left_ride_line_condition_last_ = false;
  bool right_ride_line_condition_last_ = false;
  double left_look_ahead_curvature_last_ = 0;
  double right_look_ahead_curvature_last_ = 0;
  bool odd_max_corridor_width_condition_last_ = false;
  bool odd_min_corridor_width_condition_last_ = false;
  TL::perception::LaneMarkers lane_markers_last_;
  TL::planning::lanelineprocess::DebounceModule line_length_debounce_;
  TL::planning::lanelineprocess::LowPassFilter
      line_range_start_low_pass_filter_;
  TL::planning::lanelineprocess::DebounceModule line_quality_debounce_;
  TL::planning::lanelineprocess::LowPassFilter
      line_curvature_low_pass_filter_;
  TL::planning::lanelineprocess::DebounceModule line_curvature_debounce_;
  TL::planning::lanelineprocess::DebounceModule
      left_line_ride_line_debounce_;
  TL::planning::lanelineprocess::DebounceModule
      right_line_ride_line_debounce_;
  TL::planning::lanelineprocess::LowPassFilter line_jump_pass_filter_;
  TL::planning::lanelineprocess::DebounceModule line_terrible_debounce_;
  TL::planning::lanelineprocess::DebounceModule line_width_high_debounce_;
  TL::planning::lanelineprocess::DebounceModule line_width_low_debounce_;
};

}  // namespace planning
}  // namespace TL
