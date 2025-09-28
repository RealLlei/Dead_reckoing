/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_logical_decider.h"

#include <algorithm>

#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "common/time/clock.h"
// #include "common/log.h"
#include "common/math/math_utils.h"
#include "common/util/message_util.h"
#include "planning/common/planning_gflags.h"

namespace TL {
namespace planning {
using TL::common::Clock;
using TL::common::Status;

// NOLINTBEGIN
namespace {
bool IsNewLaneMarkers(const LaneMarkers& history_lane_markers,
                      const LaneMarkers& current_lane_markers) {
  if (!history_lane_markers.header().has_sequence_num()) {
    return true;
  }
  ADEBUG << "history_lane_marker_header_sequence_num = "
         << history_lane_markers.header().seq();
  ADEBUG << "current_lane_markers_header_sequence_num = "
         << current_lane_markers.header().seq();
  ACHECK(history_lane_markers.header().has_sequence_num() &&
         current_lane_markers.header().has_sequence_num());
  // ACHECK(history_lane_markers.header().has_data_stamp() &&
  //        current_lane_markers.header().has_data_stamp());
  if (history_lane_markers.header().seq() ==
      current_lane_markers.header().seq()) {
    return false;
  } else {
    return true;
  }
  // return !common::util::IsProtoEqual(history_lane_markers,
  // current_lane_markers);
}

}  // namespace

Status LanemarkerLogicalDecider::Init() {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  TL::common::LpfCoefficients(0.03, 10, &den, &num);  // ts_ , cutoff_freq;
  digital_filter_right_c2_.set_coefficients(den, num);
  digital_filter_left_c2_.set_coefficients(den, num);
  digital_filter_right_c3_.set_coefficients(den, num);
  digital_filter_left_c3_.set_coefficients(den, num);
  mean_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(10));
  return Status::OK();
}

LanemarkerLogicalDecider::LanemarkerLogicalDecider(
    const planning::PerceptionMapConfig& config)
    : config_(config) {}

void LanemarkerLogicalDecider::SetVehicleState(
    const common::VehicleState& vehicle_state) {
  vehicle_state_ = vehicle_state;
  // vehicle_state_.set_x(0.0);
  // vehicle_state_.set_y(0.0);
  // vehicle_state_.set_z(0.0);
  // vehicle_state_.set_heading(0.0);
  // vehicle_state_.set_yaw(0.0);
}

bool LanemarkerLogicalDecider::Decision(LaneMarkers* lane_marker,
                                        std::vector<bool>* using_history) {
  ADEBUG << "lane_marker_time:" << FIXED << SETPRECISION(3)
         << Clock::NowInSeconds();
  if (!lane_marker->has_front_left_lane_marker()) {
    AERROR << "Chassis reader has no lanemarker message!!!";
    return false;
  }
  LogIndividualLanemarker(*lane_marker);
  NoUseLanemarkerC2C3(lane_marker);
  auto* right_marker = lane_marker->mutable_front_right_lane_marker();
  auto* left_marker = lane_marker->mutable_front_left_lane_marker();
  auto* next_right_marker =
      &lane_marker->mutable_front_next_right_lane_marker()->at(0);
  auto* next_left_marker =
      &lane_marker->mutable_front_next_left_lane_marker()->at(0);
  // individual right/left boundary
  if (!CentralLaneDecision(lane_marker, right_marker, left_marker)) {
    return false;
  }
  DigitalFilter(right_marker, left_marker);
  history_left_lanemarker_.CopyFrom(*left_marker);
  history_right_lanemarker_.CopyFrom(*right_marker);

  // neighbor right boundary
  RightLaneDecision(next_right_marker, right_marker, left_marker);
  // neighbor left boundary
  LeftLaneDecision(next_left_marker, right_marker, left_marker);

  history_lanemarkers_.CopyFrom(*lane_marker);
  LogAfterDecisionLanemarker(*lane_marker);
  (*using_history)[0] = flag_using_history_left_centralline_;
  (*using_history)[1] = flag_using_history_right_centralline_;
  (*using_history)[2] = using_history_mapmsg_;
  ADEBUG << "LogicalDecision end";
  return true;
}

void LanemarkerLogicalDecider::RightLaneDecision(LaneMarker* next_right_marker,
                                                 LaneMarker* right_marker,
                                                 LaneMarker* left_marker) {
  if (!GenerateNeighborLanemarker(next_right_marker, right_marker,
                                  central_lanemarker_width_,
                                  &right_neighbor_lanemarker_variance_)) {
    ADEBUG << "failed Generate next right lanemarker";
    is_generate_right_neighbor_lanemarker_ = false;
    history_right_neighbor_lanemarker_.Clear();
  } else {
    ADEBUG << "success Generate next right lanemarker";
    is_generate_right_neighbor_lanemarker_ = true;
    history_right_neighbor_lanemarker_.CopyFrom(*next_right_marker);
  }
  if (!is_generate_right_neighbor_lanemarker_) {
    using_right_neighbor_history_num_++;
    flag_using_history_right_centralline_ =
        (using_right_neighbor_history_num_ > 10 ||
         fabs(left_marker->c0_position()) < 0.9)
            ? false
            : true;
  } else {
    using_right_neighbor_history_num_ = 0;
    flag_using_history_right_centralline_ = false;
  }
  ADEBUG << "flag_using_history_right_centralline_ = "
         << flag_using_history_right_centralline_
         << "; using_right_neighbor_history_num_ = "
         << using_right_neighbor_history_num_;
}

void LanemarkerLogicalDecider::LeftLaneDecision(LaneMarker* next_left_marker,
                                                LaneMarker* right_marker,
                                                LaneMarker* left_marker) {
  if (!GenerateNeighborLanemarker(next_left_marker, left_marker,
                                  central_lanemarker_width_,
                                  &left_neighbor_lanemarker_variance_)) {
    ADEBUG << "failed Generate next left lanemarker";
    is_generate_left_neighbor_lanemarker_ = false;
    history_left_neighbor_lanemarker_.Clear();
  } else {
    ADEBUG << "success Generate next left lanemarker";
    is_generate_left_neighbor_lanemarker_ = true;
    history_left_neighbor_lanemarker_.CopyFrom(*next_left_marker);
  }
  if (!is_generate_left_neighbor_lanemarker_) {
    using_left_neighbor_history_num_++;
    flag_using_history_left_centralline_ =
        (using_left_neighbor_history_num_ > 10 ||
         fabs(right_marker->c0_position()) < 0.9)
            ? false
            : true;
  } else {
    using_left_neighbor_history_num_ = 0;
    flag_using_history_left_centralline_ = false;
  }
  ADEBUG << "flag_using_history_left_centralline_ = "
         << flag_using_history_left_centralline_
         << "; using_left_neighbor_history_num = "
         << using_left_neighbor_history_num_;
}

bool LanemarkerLogicalDecider::CentralLaneDecision(LaneMarkers* lane_marker,
                                                   LaneMarker* right_marker,
                                                   LaneMarker* left_marker) {
  bool has_right = false;
  bool has_left = false;
  has_right = JudgeHaveLanemarker(right_marker);
  ADEBUG << "right_lanemarker_quality:";
  good_right_lane_quality_ =
      has_right && ComputeLanemarkerQuality(&right_lanemarker_variance_,
                                            *right_marker) > 60
          ? true
          : false;
  has_left = JudgeHaveLanemarker(left_marker);
  ADEBUG << "left_lanemarker_quality:";
  good_left_lane_quality_ =
      has_right && ComputeLanemarkerQuality(&left_lanemarker_variance_,
                                            *left_marker) > 60
          ? true
          : false;
  ADEBUG << "after decide---has_left = " << has_left
         << " ; has_right = " << has_right;
  double vehicle_time = history_right_lanemarker_.has_view_range()
                            ? history_right_lanemarker_.view_range() /
                                  vehicle_state_.linear_velocity()
                            : 0;
  bool is_new_lane_marker =
      IsNewLaneMarkers(history_lanemarkers_, *lane_marker);
  ADEBUG << "IsNewLaneMarkers = " << is_new_lane_marker;
  if ((!is_new_lane_marker) || (!has_left && !has_right) ||
      !CreatVirtualMarker(has_right, has_left, right_marker, left_marker)) {
    has_no_mobileye_lanemarker_time_ = ComputeNoLanemarkerTime();
    if (has_no_mobileye_lanemarker_time_ <
        std::min(FLAGS_has_no_lanemarker_time, vehicle_time)) {
      using_history_mapmsg_ = true;
      right_marker->CopyFrom(history_right_lanemarker_);
      left_marker->CopyFrom(history_left_lanemarker_);
      ADEBUG << "using_history_mapmsg";
      return true;
    } else {
      using_history_mapmsg_ = false;
      history_left_lanemarker_.Clear();
      history_left_lanemarker_.Clear();
      AERROR << "In " << std::min(FLAGS_has_no_lanemarker_time, vehicle_time)
             << " second using history lanmarker,but now not received "
                "lanemarker message for "
             << has_no_mobileye_lanemarker_time_ << " second";
      return false;
    }
  } else {
    ADEBUG << "------------has lanemarker-----------";
    flag_log_no_lanemarker_time_ = false;
    using_history_mapmsg_ = false;
  }
  return true;
}

void LanemarkerLogicalDecider::NoUseLanemarkerC2C3(LaneMarkers* markers) {
  if (FLAGS_navigation_hdmap_no_lane_marker_c2) {
    markers->mutable_front_right_lane_marker()->set_c2_curvature(0.0f);
    markers->mutable_front_left_lane_marker()->set_c2_curvature(0.0f);
    markers->mutable_front_next_right_lane_marker()->at(0).set_c2_curvature(
        0.0f);
    markers->mutable_front_next_left_lane_marker()->at(0).set_c2_curvature(
        0.0f);
  }
  if (FLAGS_navigation_hdmap_no_lane_marker_c3) {
    markers->mutable_front_right_lane_marker()->set_c3_curvature_derivative(
        0.0f);
    markers->mutable_front_left_lane_marker()->set_c3_curvature_derivative(
        0.0f);
    markers->mutable_front_next_right_lane_marker()
        ->at(0)
        .set_c3_curvature_derivative(0.0f);
    markers->mutable_front_next_left_lane_marker()
        ->at(0)
        .set_c3_curvature_derivative(0.0f);
  }
}

void LanemarkerLogicalDecider::LogIndividualLanemarker(
    const LaneMarkers& lane_marker) {
  const auto& left_marker = lane_marker.front_left_lane_marker();
  const auto& right_marker = lane_marker.front_right_lane_marker();
  auto next_left_marker = lane_marker.front_next_left_lane_marker().at(0);
  auto next_right_marker = lane_marker.front_next_right_lane_marker().at(0);
  ADEBUG << "before decision lane marker:"
         << "left_individual_6ac_c0:" << left_marker.c0_position()
         << "  left_individual_6ac_c1:" << left_marker.c1_heading_angle()
         << "  left_individual_6ac_c2:" << left_marker.c2_curvature()
         << "  left_individual_6ac_c3:" << left_marker.c3_curvature_derivative()
         << "left_individual_6ac_viewrange:" << left_marker.view_range();
  ADEBUG << "right_individual_6ab_c0:" << right_marker.c0_position()
         << "  right_individual_6ab_c1:" << right_marker.c1_heading_angle()
         << "  right_individual_6ab_c2:" << right_marker.c2_curvature()
         << "  right_individual_6ab_c3:"
         << right_marker.c3_curvature_derivative()
         << "right_individual_6ab_viewrange:" << right_marker.view_range();
  ADEBUG << "next_left_individual_6a9_c0:" << next_left_marker.c0_position()
         << "  next_left_individual_6a9_c1:"
         << next_left_marker.c1_heading_angle()
         << "  next_left_individual_6a9_c2:" << next_left_marker.c2_curvature()
         << "  next_left_individual_6a9_c3:"
         << next_left_marker.c3_curvature_derivative()
         << "next_left_individual_6a9_viewrange:"
         << next_left_marker.view_range();
  ADEBUG << "next_right_individual_6a8_c0:" << next_right_marker.c0_position()
         << "  next_right_individual_6a8_c1:"
         << next_right_marker.c1_heading_angle()
         << "  next_right_individual_6a8_c2:"
         << next_right_marker.c2_curvature()
         << "  next_right_individual_6a8_c3:"
         << next_right_marker.c3_curvature_derivative()
         << "next_right_individual_6a8_viewrange:"
         << next_right_marker.view_range();
}

void LanemarkerLogicalDecider::LogAfterDecisionLanemarker(
    const LaneMarkers& lane_marker) {
  const auto& left_marker = lane_marker.front_left_lane_marker();
  const auto& right_marker = lane_marker.front_right_lane_marker();
  auto next_left_marker = lane_marker.front_next_left_lane_marker().at(0);
  auto next_right_marker = lane_marker.front_next_right_lane_marker().at(0);
  ADEBUG << "after decision lane marker:"
         << "left_decision_6ac_c0:" << left_marker.c0_position()
         << "  left_decision_6ac_c1:" << left_marker.c1_heading_angle()
         << "  left_decision_6ac_c2:" << left_marker.c2_curvature()
         << "  left_decision_6ac_c3:" << left_marker.c3_curvature_derivative()
         << "left_decision_6ac_viewrange:" << left_marker.view_range();
  ADEBUG << "right_decision_6ab_c0:" << right_marker.c0_position()
         << "  right_decision_6ab_c1:" << right_marker.c1_heading_angle()
         << "  right_decision_6ab_c2:" << right_marker.c2_curvature()
         << "  right_decision_6ab_c3:" << right_marker.c3_curvature_derivative()
         << "right_decision_6ab_viewrange:" << right_marker.view_range();
  ADEBUG << "next_left_decision_6a9_c0:" << next_left_marker.c0_position()
         << "  next_left_decision_6a9_c1:"
         << next_left_marker.c1_heading_angle()
         << "  next_left_decision_6a9_c2:" << next_left_marker.c2_curvature()
         << "  next_left_decision_6a9_c3:"
         << next_left_marker.c3_curvature_derivative()
         << "next_left_decision_6a9_viewrange:"
         << next_left_marker.view_range();
  ADEBUG << "next_right_decision_6a8_c0:" << next_right_marker.c0_position()
         << "  next_right_decision_6a8_c1:"
         << next_right_marker.c1_heading_angle()
         << "  next_right_decision_6a8_c2:" << next_right_marker.c2_curvature()
         << "  next_right_decision_6a8_c3:"
         << next_right_marker.c3_curvature_derivative()
         << "next_right_decision_6a8_viewrange:"
         << next_right_marker.view_range();
}

void LanemarkerLogicalDecider::DigitalFilter(LaneMarker* right_marker,
                                             LaneMarker* left_marker) {
  if (FLAGS_using_filter_of_c2) {
    ADEBUG << "Before filter:" << FIXED << SETPRECISION(6)
           << "right_c2 = " << right_marker->c2_curvature()
           << "left_c2 = " << left_marker->c2_curvature();
    right_marker->set_c2_curvature(
        digital_filter_right_c2_.Filter(right_marker->c2_curvature()));
    left_marker->set_c2_curvature(
        digital_filter_left_c2_.Filter(left_marker->c2_curvature()));
    ADEBUG << "After filter:" << FIXED << SETPRECISION(6)
           << "right_c2 = " << right_marker->c2_curvature()
           << "left_c2 = " << left_marker->c2_curvature();
  }

  if (FLAGS_using_filter_of_c3) {
    ADEBUG << "Before filter " << FIXED << SETPRECISION(6)
           << "right_c3 = " << right_marker->c3_curvature_derivative()
           << "left_c3 = " << left_marker->c3_curvature_derivative();
    right_marker->set_c3_curvature_derivative(digital_filter_right_c3_.Filter(
        right_marker->c3_curvature_derivative()));
    left_marker->set_c3_curvature_derivative(digital_filter_right_c3_.Filter(
        left_marker->c3_curvature_derivative()));
    ADEBUG << "After filter " << FIXED << SETPRECISION(6)
           << "right_c3 = " << right_marker->c3_curvature_derivative()
           << "left_c3 = " << left_marker->c3_curvature_derivative();
  }
}

void LanemarkerLogicalDecider::LanemarkerReverse(LaneMarker* marker) {
  marker->set_c0_position(-marker->c0_position());
  marker->set_c1_heading_angle(-marker->c1_heading_angle());
  marker->set_c2_curvature(-marker->c2_curvature());
  marker->set_c3_curvature_derivative(-marker->c3_curvature_derivative());
  marker->set_longitude_start(-5);
}

void LanemarkerLogicalDecider::NextLanemarkerReverse(LaneMarker* next_marker,
                                                     LaneMarker* marker) {
  next_marker->set_c0_position(-next_marker->c0_position());
  next_marker->set_c1_heading_angle(-next_marker->c1_heading_angle());
  next_marker->set_c2_curvature(-next_marker->c2_curvature());
  next_marker->set_c3_curvature_derivative(
      -next_marker->c3_curvature_derivative());
  next_marker->set_longitude_start(-5);
}

bool LanemarkerLogicalDecider::JudgeHaveLanemarker(LaneMarker* marker) {
  if (marker->view_range() > 0.01) {
    // LanemarkerReverse(marker);
    ADEBUG << "original lanemarker:" << marker->DebugString();
    return true;
  } else {
    marker->Clear();
    return false;
  }
}

bool LanemarkerLogicalDecider::GenerateNeighborLanemarker(
    LaneMarker* next_marker, LaneMarker* marker, double width,
    std::deque<LaneMarker>* lane_markers) {
  if (next_marker->view_range() > 0.01 &&
      std::fabs(next_marker->c0_position() - marker->c0_position()) >=
          2 * config_.min_lane_half_width()) {
    // NextLanemarkerReverse(next_marker, marker);
    if (!in_change_lane_ &&
        !(next_marker->quality() > 0 ||
          ComputeLanemarkerQuality(lane_markers, *next_marker) > 50)) {
      AERROR << "lanemarker Quality err";
      return false;
    }
    ADEBUG << "original next lanemarker:" << next_marker->DebugString();
    double view_range =
        TruncateIntersectingBoundaryLines(*marker, *next_marker);
    if (view_range < 0.01) {
      AWARN << "next_lane_marker boundary too short"
            << "lane range = " << view_range;
      next_marker->Clear();
      return false;
    } else {
      next_marker->set_longitude_end(view_range);
      return true;
    }
    ADEBUG << "view range after truncate intersecting next left = "
           << view_range;
  } else if (FLAGS_generate_three_centralines_alltime) {
    next_marker->CopyFrom(*marker);
    next_marker->set_c0_position(marker->c0_position() - width);
    return true;
  } else {
    next_marker->Clear();
    return false;
  }
}

double LanemarkerLogicalDecider::ComputeLanemarkerStepQuality(
    const LaneMarker& lanemarker, const LaneMarker& history_lanemarker) {
  if (!history_lanemarker.has_c0_position()) {
    return 0.0;
  }
  double c0 = lanemarker.c0_position();
  double c1 = std::atan(lanemarker.c1_heading_angle());
  double c2 = lanemarker.c2_curvature() * 2;
  double c3 = lanemarker.c3_curvature_derivative() * 6;
  double delta_c0 = fabs(c0 - history_lanemarker.c0_position());
  double delta_c1 = fabs(c1 - std::atan(history_lanemarker.c1_heading_angle()));
  double delta_c2 = fabs(c2 - history_lanemarker.c2_curvature() * 2);
  double delta_c3 = fabs(c3 - history_lanemarker.c3_curvature_derivative() * 6);
  double quality = delta_c0 / 0.06 * 50 + delta_c1 / 0.004 * 40 +
                   delta_c2 / 0.0003 * 50 + delta_c3 / 0.000003 * 20;
  return quality;
}

double LanemarkerLogicalDecider::ComputeLanemarkerQuality(
    std::deque<LaneMarker>* lane_markers, const LaneMarker& lanemarker) {
  lane_markers->push_back(lanemarker);
  if (lane_markers->size() < 5) {
    return 0.0;
  }
  if (lane_markers->size() > variance_num_) {
    lane_markers->pop_front();
  }
  std::deque<LaneMarker>::iterator it;
  double sum_c0{0.0}, sum_c1{0.0}, sum_c2{0.0}, sum_c3{0.0};
  for (it = lane_markers->begin(); it != lane_markers->end(); it++) {
    sum_c0 += it->c0_position();
    sum_c1 += std::atan(it->c1_heading_angle());
    sum_c2 += it->c2_curvature() * 2;
    sum_c3 += it->c3_curvature_derivative() * 6;
  }
  double avg_c0 = sum_c0 / lane_markers->size();
  double avg_c1 = sum_c1 / lane_markers->size();
  double avg_c2 = sum_c2 / lane_markers->size();
  double avg_c3 = sum_c3 / lane_markers->size();
  sum_c0 = 0.0;
  sum_c1 = 0.0;
  sum_c2 = 0.0;
  sum_c3 = 0.0;
  for (it = lane_markers->begin(); it != lane_markers->end(); it++) {
    double c0_tmp = it->c0_position() - avg_c0;
    sum_c0 += c0_tmp * c0_tmp;
    double c1_tmp = std::atan(it->c1_heading_angle()) - avg_c1;
    sum_c1 += c1_tmp * c1_tmp;
    double c2_tmp = it->c2_curvature() * 2 - avg_c2;
    sum_c2 += c2_tmp * c2_tmp;
    double c3_tmp = it->c3_curvature_derivative() * 6 - avg_c3;
    sum_c3 += c3_tmp * c3_tmp;
  }
  double c0_variance = std::sqrt(sum_c0 / lane_markers->size());
  double c1_variance = std::sqrt(sum_c1 / lane_markers->size());
  double c2_variance = std::sqrt(sum_c2 / lane_markers->size());
  double c3_variance = std::sqrt(sum_c3 / lane_markers->size());
  double quality_c_0 = ComputeQuality(c0_variance / 0.09);  // 0.07
  double quality_c_1 = ComputeQuality(c1_variance / 0.009);
  double quality_c_2 = ComputeQuality(c2_variance / 0.0007);
  double quality_c_3 = ComputeQuality(c3_variance / 0.0000012);
  double quality_c = quality_c_0 + quality_c_1 + quality_c_2 + quality_c_3;
  double quality_range = std::fmin(lanemarker.view_range() / 30 * 100, 100);
  ADEBUG << "deque_lane_markers_size = " << lane_markers->size();
  ADEBUG << "lane_marker_quality_c = " << quality_c
         << "; quality_range = " << quality_range
         << "; quality_c1 = " << quality_c_0 << "; quality_c2 = " << quality_c_1
         << "; quality_c3 = " << quality_c_2
         << "; quality_c4 = " << quality_c_3;
  return quality_c;
}

int LanemarkerLogicalDecider::ComputeQuality(double ref_lanemarker_variance) {
  int quality_c = 0;
  if (ref_lanemarker_variance >= 0.9) {
    quality_c = 0;
  } else if (ref_lanemarker_variance >= 0.7) {
    quality_c = 10;
  } else if (ref_lanemarker_variance >= 0.5) {
    quality_c = 20;
  } else if (ref_lanemarker_variance >= 0) {
    quality_c = 25;
  }
  return quality_c;
}

double LanemarkerLogicalDecider::ComputeNoLanemarkerTime() {
  double no_lanemarker_time = 0.0;
  if (!flag_log_no_lanemarker_time_) {
    start_no_lanemarker_time_ = Clock::NowInSeconds();
    flag_log_no_lanemarker_time_ = true;
    AERROR << "start no lanemarker and start_time = "
           << start_no_lanemarker_time_;
  }
  no_lanemarker_time = Clock::NowInSeconds() - start_no_lanemarker_time_;
  AERROR << "totle_time_no_lanemarker = " << no_lanemarker_time;
  return no_lanemarker_time;
}

bool LanemarkerLogicalDecider::CreatVirtualMarker(
    bool have_right, bool have_left, LaneMarker* right_lane_marker,
    LaneMarker* left_lane_marker) {
  if (have_right && !have_left &&
      (right_lane_marker->quality() > 0 || good_right_lane_quality_) &&
      fabs(right_lane_marker->c0_position()) < 5) {
    left_lanemarker_variance_.clear();
    left_lane_marker->CopyFrom(*right_lane_marker);
    left_lane_marker->set_c0_position(right_lane_marker->c0_position() +
                                      central_lanemarker_width_history_);
    return true;
  } else if (!have_right && have_left &&
             (left_lane_marker->quality() > 0 || good_left_lane_quality_) &&
             fabs(left_lane_marker->c0_position()) < 5) {
    right_lanemarker_variance_.clear();
    right_lane_marker->CopyFrom(*left_lane_marker);
    right_lane_marker->set_c0_position(left_lane_marker->c0_position() -
                                       central_lanemarker_width_history_);
    return true;
  } else if (have_right && have_left &&
             fabs(left_lane_marker->c0_position()) < 5 &&
             fabs(right_lane_marker->c0_position()) < 5 &&
             (left_lane_marker->quality() > 0 ||
              right_lane_marker->quality() > 0 || good_left_lane_quality_ ||
              good_right_lane_quality_)) {
    InChangeLane(right_lane_marker, left_lane_marker);
    central_lanemarker_width_ = std::fabs(left_lane_marker->c0_position() -
                                          right_lane_marker->c0_position());
    if (central_lanemarker_width_ < 2 * config_.min_lane_half_width() ||
        central_lanemarker_width_ > 2 * config_.max_lane_half_width()) {
      central_lanemarker_width_ = central_lanemarker_width_history_;
      central_line_too_narrow_index_++;
    } else {
      central_lanemarker_width_history_ = central_lanemarker_width_;
      central_line_too_narrow_index_ = 0;
    }
    if (central_line_too_narrow_index_ >
        static_cast<int>(FLAGS_has_no_lanemarker_time / 0.1)) {
      AERROR << "Central lane too narrow for " << FLAGS_has_no_lanemarker_time
             << " second";
      return false;
    }
    // auto short_lane_marker = lane_marker->left_lane_marker();
    double view_range = TruncateIntersectingBoundaryLines(*left_lane_marker,
                                                          *right_lane_marker);
    if (view_range < 0.01) {
      AERROR << "Central line boundary too short between max lane width and min"
                "lane width = "
             << view_range;
      return false;
    } else {
      left_lane_marker->set_longitude_end(view_range);
      right_lane_marker->set_longitude_end(view_range);
    }
    ADEBUG << "view range after truncate intersecting left and right = "
           << view_range;
    return true;
  }
  AERROR << "VirtualMarker false";
  return false;
}

double LanemarkerLogicalDecider::TruncateIntersectingBoundaryLines(
    const LaneMarker& retain_lane_marker,
    const LaneMarker& truncate_lane_marker) {
  int sgn =
      truncate_lane_marker.c0_position() - retain_lane_marker.c0_position() > 0
          ? 1
          : -1;
  double start_retain_length = retain_lane_marker.longitude_start();
  double x_2(0.0);
  double x_3(0.0);
  double end_length = start_retain_length;
  double max_min_lane_width = 0;
  double truncate_y = 0;
  double retain_y = 0;
  for (double x = start_retain_length;
       x < fmin(retain_lane_marker.longitude_end(),
                truncate_lane_marker.longitude_end());) {
    x_2 = x * x;
    x_3 = x_2 * x;
    truncate_y = x_3 * truncate_lane_marker.c3_curvature_derivative() +
                 x_2 * truncate_lane_marker.c2_curvature() +
                 x * truncate_lane_marker.c1_heading_angle() +
                 truncate_lane_marker.c0_position();
    retain_y = x_3 * retain_lane_marker.c3_curvature_derivative() +
               x_2 * retain_lane_marker.c2_curvature() +
               x * retain_lane_marker.c1_heading_angle() +
               retain_lane_marker.c0_position();
    end_length = x;
    max_min_lane_width = sgn * (truncate_y - retain_y);
    if (max_min_lane_width <= 2 * config_.min_lane_half_width() ||
        max_min_lane_width >= 2 * config_.max_lane_half_width()) {
      break;
    }
    x += 1.0;
  }
  ADEBUG << "retain_y = " << retain_y << "; truncate_y = " << truncate_y
         << "; max_min_lane_width = " << max_min_lane_width
         << "; end_length = " << end_length;
  return end_length;
}

void LanemarkerLogicalDecider::InChangeLane(LaneMarker* right_lane_marker,
                                            LaneMarker* left_lane_marker) {
  bool near_central_line = fabs(right_lane_marker->c0_position()) < 0.5 ||
                           fabs(left_lane_marker->c0_position()) < 0.5;
  // bool c0_reverse =
  //     right_lane_marker->c0_position() *
  //             history_right_lanemarker_.c0_position() <
  //         0 ||
  //     left_lane_marker->c0_position() * history_left_lanemarker_.c0_position() <
  //         0;
  in_change_lane_ = near_central_line;  // ||c0_reverse
  ADEBUG << "------in_change_lane = " << in_change_lane_;
}

// NOLINTEND
}  // namespace planning
}  // namespace TL
