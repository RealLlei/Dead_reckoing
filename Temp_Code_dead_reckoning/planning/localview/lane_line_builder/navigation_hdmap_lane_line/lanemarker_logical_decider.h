/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#pragma once

#include <deque>
#include <list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/filters/digital_filter.h"
#include "common/filters/digital_filter_coefficients.h"
#include "common/filters/mean_filter.h"
#include "common/status/status.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/navigation_lanecentral_constructor.h"

#include "proto/common/vehicle_state.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/map/navigation.pb.h"

#include "planning/proto/navigation_hdmap_config.pb.h"

/**
 * @namespace TL::navigation_hdmap
 * @brief TL::navigation_hdmap
 */
namespace TL {
namespace planning {
using TL::common::math::Vec2d;  // NOLINT
using TL::perception::LaneMarker;
using TL::perception::LaneMarkers;

/**
 * @class LanemarkerLogicalDecider
 * @brief LanemarkerLogicalDecider generates a real-time relative map based on
 * navagation lines.
 */
class LanemarkerLogicalDecider {
 public:
  LanemarkerLogicalDecider() = default;
  explicit LanemarkerLogicalDecider(
      const planning::PerceptionMapConfig& config);
  ~LanemarkerLogicalDecider() = default;

  /**
   * @brief Set the configuration information required by the
   * `LanemarkerLogicalDecider`.
   * @param config Configuration object.
   * @return None.
   */
  TL::common::Status Init();

  void SetVehicleState(const common::VehicleState& vehicle_state);

  bool Decision(LaneMarkers* lane_marker, std::vector<bool>* using_history);

 private:
  bool CentralLaneDecision(LaneMarkers* lane_marker, LaneMarker* right_marker,
                           LaneMarker* left_marker);
  void RightLaneDecision(LaneMarker* next_right_marker,
                         LaneMarker* right_marker, LaneMarker* left_marker);
  void LeftLaneDecision(LaneMarker* next_left_marker, LaneMarker* right_marker,
                        LaneMarker* left_marker);
  void LanemarkerReverse(LaneMarker* marker);
  void NoUseLanemarkerC2C3(LaneMarkers* markers);
  void DigitalFilter(LaneMarker* right_marker, LaneMarker* left_marker);
  void NextLanemarkerReverse(LaneMarker* next_marker, LaneMarker* marker);
  bool JudgeHaveLanemarker(LaneMarker* lane_marker);
  bool GenerateNeighborLanemarker(LaneMarker* next_marker, LaneMarker* marker,
                                  double width,
                                  std::deque<LaneMarker>* lane_markers);
  double ComputeNoLanemarkerTime();
  int ComputeQuality(double ref_lanemarker_variance);
  bool CreatVirtualMarker(bool have_right, bool have_left,
                          LaneMarker* right_lane_marker,
                          LaneMarker* left_lane_marker);
  double ComputeLaneWidth(double* leftwidth);
  double ComputeLanemarkerQuality(std::deque<LaneMarker>* lane_markers,
                                  const LaneMarker& lanemarker);
  double ComputeLanemarkerStepQuality(const LaneMarker& lanemarker,
                                      const LaneMarker& history_lanemarker);

  void LogIndividualLanemarker(const LaneMarkers& lane_marker);
  void LogAfterDecisionLanemarker(const LaneMarkers& lane_marker);
  void InChangeLane(LaneMarker* right_lane_marker,
                    LaneMarker* left_lane_marker);

  double TruncateIntersectingBoundaryLines(
      const LaneMarker& retain_lane_marker,
      const LaneMarker& truncate_lane_marker);

 private:
  // the configuration information required by the `LanemarkerLogicalDecider`
  planning::PerceptionMapConfig config_;

  // The standard lane width of China's expressway is 3.75 meters.
  double has_no_mobileye_lanemarker_time_ = 100.0;
  bool using_history_mapmsg_ = false;
  double start_no_lanemarker_time_ = 0.0;
  bool flag_log_no_lanemarker_time_ = false;
  double central_lanemarker_width_ = 3.75;
  double central_lanemarker_width_history_ = 3.75;
  int central_line_too_narrow_index_ = 0;

  bool flag_using_history_left_centralline_ = false;
  bool flag_using_history_right_centralline_ = false;
  // neighbor lanemarker flag
  bool is_generate_left_neighbor_lanemarker_ = false;
  bool is_generate_right_neighbor_lanemarker_ = false;
  int using_left_neighbor_history_num_ = 0;
  int using_right_neighbor_history_num_ = 0;
  // Standard Deviation
  std::deque<LaneMarker> left_lanemarker_variance_{};
  std::deque<LaneMarker> right_lanemarker_variance_{};
  std::deque<LaneMarker> left_neighbor_lanemarker_variance_{};
  std::deque<LaneMarker> right_neighbor_lanemarker_variance_{};
  int variance_num_ = 5;
  bool in_change_lane_ = false;
  bool good_left_lane_quality_ = false;
  bool good_right_lane_quality_ = false;

  // It is used to transfer data acquired from perception to output layer,
  // only used as an intermediate variable
  LaneMarker history_left_lanemarker_;
  LaneMarker history_right_lanemarker_;
  LaneMarker history_left_neighbor_lanemarker_;
  LaneMarker history_right_neighbor_lanemarker_;
  LaneMarkers history_lanemarkers_;
  common::VehicleState vehicle_state_;
  TL::common::DigitalFilter digital_filter_right_c2_;
  TL::common::DigitalFilter digital_filter_left_c2_;
  TL::common::DigitalFilter digital_filter_right_c3_;
  TL::common::DigitalFilter digital_filter_left_c3_;
  TL::common::MeanFilter mean_filter_;
};

}  // namespace planning
}  // namespace TL
