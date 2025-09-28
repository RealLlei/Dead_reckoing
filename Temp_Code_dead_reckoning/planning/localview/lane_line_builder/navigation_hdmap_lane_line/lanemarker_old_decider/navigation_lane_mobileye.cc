/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief This file provides the implementation of the class
 * `NavigationLaneMobileye`.
 */

#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_old_decider/navigation_lane_mobileye.h"

#include <algorithm>
#include <deque>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>

#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "common/time/clock.h"
// #include "common/log.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/util/message_util.h"
#include "common/util/points_downsampler.h"
#include "common/util/util.h"
#include "map/hdmap/path.h"
#include "planning/common/planning_gflags.h"

#include "proto/common/types.pb.h"
#include "proto/map/map_lane.pb.h"

namespace TL {
namespace planning {
// NOLINTBEGIN
using TL::common::VehicleState;
using TL::common::util::DistanceXY;
using TL::common::util::DownsampleByAngle;
using TL::common::util::DownsampleCurve;
using TL::hdmap::Lane;
using TL::hdmap::LaneBoundaryType;
using TL::common::util::operator+;
using TL::common::Clock;
using TL::common::PointENU;
using TL::common::Status;
using TL::common::util::IsFloatEqual;
using TL::hdmap::Curve;

namespace {
bool IsNewLaneMarkers(const perception::LaneMarkers& history_lane_markers,
                      const perception::LaneMarkers& current_lane_markers) {
  if (!history_lane_markers.header().has_seq()) {
    return true;
  }
  ADEBUG << "history_lane_marker_header_sequence_num = "
         << history_lane_markers.header().seq();
  ADEBUG << "current_lane_markers_header_sequence_num = "
         << current_lane_markers.header().seq();
  ACHECK(history_lane_markers.header().has_seq() &&
         current_lane_markers.header().has_seq());
  // ACHECK(history_lane_markers.header().has_data_stamp() &&
  //        current_lane_markers.header().has_data_stamp());
  return !(history_lane_markers.header().seq() ==
           current_lane_markers.header().seq());
  // return !common::util::IsProtoEqual(history_lane_markers,
  // current_lane_markers);
}
}  // namespace

Status NavigationLaneMobileye::Init() {
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

bool NavigationLaneMobileye::CreateSingleLaneMap(
    const NaviPathTupleOld& navi_path_tuple,
    const perception::LaneMarkers& lane_marker, hdmap::Map* const hdmap,
    google::protobuf::Map<std::string, navigation_hdmap::NavigationPath>* const
        navigation_path) const {
  CHECK_NOTNULL(hdmap);
  CHECK_NOTNULL(navigation_path);

  const auto& navi_path = std::get<3>(navi_path_tuple);
  const auto& path = navi_path->path();
  const auto& left_boundary_path = std::get<4>(navi_path_tuple);
  const auto& right_boundary_path = std::get<5>(navi_path_tuple);
  double size_navipath = navi_path->path().path_point().size();
  if (path.path_point_size() < 2) {
    ADEBUG << "The path length of line index is invalid";
    return false;
  }
  auto* lane = hdmap->add_lane();
  lane->mutable_id()->set_id(
      absl::StrCat(navi_path->path_priority(), "_", path.name()));
  (*navigation_path)[lane->id().id()] = *navi_path;
  // lane types
  lane->set_type(Lane::CITY_DRIVING);
  lane->set_turn(Lane::NO_TURN);

  // speed limit
  lane->set_speed_limit(config_.default_speed_limit());

  // center line
  auto* curve_segment = lane->mutable_central_curve()->add_segment();
  curve_segment->set_heading(path.path_point(0).theta());
  curve_segment->set_length(path.path_point(path.path_point_size() - 1).s());
  lane->set_length(path.path_point(path.path_point_size() - 1).s());
  auto* line_segment = curve_segment->mutable_line_segment();

  // left boundary
  hdmap::LineSegment* left_segment = nullptr;
  if (FLAGS_navigation_hdmap_generate_left_boundray) {
    auto* left_boundary = lane->mutable_left_boundary();
    auto* left_boundary_type = left_boundary->add_boundary_type();
    left_boundary->set_virtual_(false);
    left_boundary_type->set_s(0.0);
    left_boundary_type->add_types(
        lane_marker.front_left_lane_marker().lane_type());
    left_segment =
        left_boundary->mutable_curve()->add_segment()->mutable_line_segment();
  }

  // right boundary
  auto* right_boundary = lane->mutable_right_boundary();
  auto* right_boundary_type = right_boundary->add_boundary_type();
  right_boundary->set_virtual_(false);
  right_boundary_type->set_s(0.0);
  right_boundary_type->add_types(
      lane_marker.front_right_lane_marker().lane_type());
  auto* right_segment =
      right_boundary->mutable_curve()->add_segment()->mutable_line_segment();

  const double lane_left_width = std::get<1>(navi_path_tuple);
  const double lane_right_width = std::get<2>(navi_path_tuple);
  double index = 0;
  int point_size = path.path_point().size();
  for (size_t i = 0; i < path.path_point().size(); i++) {
    auto* point = line_segment->add_point();
    point->set_x(path.path_point().at(i).x());
    point->set_y(path.path_point().at(i).y());
    point->set_z(path.path_point().at(i).z());
    index = index + 1;
    double point_x = path.path_point().at(i).x();
    double point_y = path.path_point().at(i).y();
    double theta_i = path.path_point().at(i).theta();
    if (FLAGS_navigation_hdmap_generate_left_boundray) {
      auto* left_sample = lane->add_left_sample();
      left_sample->set_s(path.path_point().at(i).s());
      left_sample->set_width(lane_left_width);
      if (FLAGS_using_original_lanemarker_boundary &&
          i < left_boundary_path->point().size()) {
        left_segment->add_point()->CopyFrom(left_boundary_path->point().at(i));
      } else if (i < left_boundary_path->point().size()) {
        left_segment->add_point()->CopyFrom(
            *point +
            lane_left_width * Vec2d::CreateUnitVec2d(
                                  path.path_point().at(i).theta() + M_PI_2));
      }
    }

    auto* right_sample = lane->add_right_sample();
    right_sample->set_s(path.path_point().at(i).s());
    right_sample->set_width(lane_right_width);
    if (FLAGS_using_original_lanemarker_boundary &&
        i < right_boundary_path->point().size()) {
      right_segment->add_point()->CopyFrom(right_boundary_path->point().at(i));
    } else if (i < right_boundary_path->point().size()) {
      right_segment->add_point()->CopyFrom(
          *point +
          lane_right_width *
              Vec2d::CreateUnitVec2d(path.path_point().at(i).theta() - M_PI_2));
    }
  }
  DownsampleCurve(lane->mutable_central_curve());
  DownsampleCurve(lane->mutable_left_boundary()->mutable_curve());
  DownsampleCurve(lane->mutable_right_boundary()->mutable_curve());
  /*打印稀疏点信息
  AERROR << "----------------central_points_size------------ = "
         << lane->central_curve().segment(0).line_segment().point_size();
  for (const auto& point :
       lane->central_curve().segment(0).line_segment().point()) {
    ADEBUG << "central_point_x = " << point.x()
              << "; central_point_y = " << point.y() << ".";
  }
  AERROR
      << "----------------left_points_size------------ = "
      << lane->left_boundary().curve().segment(0).line_segment().point_size();
  for (const auto& point :
       lane->left_boundary().curve().segment(0).line_segment().point()) {
    ADEBUG << "left_point_x = " << point.x()
              << "; left_point_y = " << point.y() << ".";
  }
  AERROR
      << "----------------right_points_size------------ = "
      << lane->right_boundary().curve().segment(0).line_segment().point_size();
  for (const auto& point :
       lane->right_boundary().curve().segment(0).line_segment().point()) {
    ADEBUG << "right_point_x = " << point.x()
              << "; right_point_y = " << point.y() << ".";
  }*/
  return true;
}

NavigationLaneMobileye::NavigationLaneMobileye(
    const planning::PerceptionMapConfig& config)
    : config_(config) {}

void NavigationLaneMobileye::SetConfig(
    const planning::PerceptionMapConfig& config) {
  config_ = config;
}

void NavigationLaneMobileye::SetVehicleState(
    const std::shared_ptr<const common::VehicleState>& vehicle_state) {
  vehicle_state_ = vehicle_state;
  // vehicle_state_->set_x(0.0);
  // vehicle_state_->set_y(0.0);
  // vehicle_state_->set_z(0.0);
  // vehicle_state_->set_heading(0.0);
  // vehicle_state_->set_yaw(0.0);
}

bool NavigationLaneMobileye::GeneratePath() {
  navigation_path_list_.clear();
  current_navi_path_tuple_ =
      std::make_tuple(-1, -1.0, -1.0, nullptr, nullptr, nullptr);

  auto lane_marker = lane_marker_;

  auto generate_path_on_perception_func = [this, &lane_marker]() {
    auto current_navi_path =
        std::make_shared<navigation_hdmap::NavigationPath>();
    auto* path = current_navi_path->mutable_path();
    ConvertLaneMarkerToPath(&lane_marker, path);
    current_navi_path->set_path_priority(0);
    auto left_boundary = std::make_shared<hdmap::LineSegment>();
    auto right_boundary = std::make_shared<hdmap::LineSegment>();
    double left_width = perceived_left_width_ > 0.0 ? perceived_left_width_
                                                    : default_left_width_;
    double right_width = perceived_right_width_ > 0.0 ? perceived_right_width_
                                                      : default_right_width_;
    current_navi_path_tuple_ =
        std::make_tuple(0, left_width, right_width, current_navi_path,
                        left_boundary, right_boundary);
  };

  ADEBUG << "Beginning of NavigationLaneMobileye::GeneratePath().";

  // Generate a navigation path where the vehicle is located based on perceived
  // lane markers.
  generate_path_on_perception_func();
  return true;
}

double NavigationLaneMobileye::EvaluateCubicPolynomial(const double c0,
                                                       const double c1,
                                                       const double c2,
                                                       const double c3,
                                                       const double x) const {
  return ((c3 * x + c2) * x + c1) * x + c0;
}

double NavigationLaneMobileye::GetKappa(const double c1, const double c2,
                                        const double c3, const double x) {
  const double dy = 3 * c3 * x * x + 2 * c2 * x + c1;
  const double d2y = 6 * c3 * x + 2 * c2;
  return d2y / std::pow((1 + dy * dy), 1.5);
}

double NavigationLaneMobileye::TruncateIntersectingBoundaryLines(
    const perception::LaneMarker& retain_lane_marker,
    const perception::LaneMarker& truncate_lane_marker) {
  int sgn =
      truncate_lane_marker.c0_position() - retain_lane_marker.c0_position() > 0
          ? 1
          : -1;
  double start_retain_length = retain_lane_marker.longitude_start();
  double x_2(0.0f);
  double x_3(0.0f);
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

void NavigationLaneMobileye::ConvertLaneMarkerToPath(
    perception::LaneMarkers* lane_marker, common::Path* const path,
    bool is_mobileye) {
  CHECK_NOTNULL(path);
  if (!LaneMarkerLogicalDecision(lane_marker)) {
    ADEBUG << " LaneMarker Logical Decision faild";
    return;
  }
  if (is_mobileye) {
    ConvertMobileyeLaneMarkerToPath(*lane_marker, path);
  }
  path->set_name("Path from lane markers.");
}

bool NavigationLaneMobileye::LaneMarkerLogicalDecision(
    perception::LaneMarkers* lane_marker) {
  ADEBUG << "lane_marker_time:" << FIXED << SETPRECISION(3)
         << Clock::NowInSeconds();
  if (!lane_marker->has_front_right_lane_marker()) {
    ADEBUG << "Chassis reader has no lanemarker message!!!";
    return false;
  }
  LogIndividualLanemarker(*lane_marker);
  NoUseLanemarkerC2C3(lane_marker);
  auto right_marker = lane_marker->mutable_front_right_lane_marker();
  auto left_marker = lane_marker->mutable_front_left_lane_marker();
  auto next_right_marker =
      &lane_marker->mutable_front_next_right_lane_marker()->at(0);
  auto next_left_marker =
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
  ADEBUG << "LogicalDecision end";
  return true;
}

void NavigationLaneMobileye::RightLaneDecision(
    perception::LaneMarker* next_right_marker,
    perception::LaneMarker* right_marker, perception::LaneMarker* left_marker) {
  if (!GenerateNeighborLanemarker(next_right_marker, right_marker,
                                  central_lanemarker_width_,
                                  &right_neighbor_lanemarker_variance_)) {
    ADEBUG << "failed Generate next right lanemarker";
    is_generate_right_neighbor_lanemarker = false;
    history_right_neighbor_lanemarker_.Clear();
  } else {
    ADEBUG << "success Generate next right lanemarker";
    is_generate_right_neighbor_lanemarker = true;
    history_right_neighbor_lanemarker_.CopyFrom(*next_right_marker);
  }
  if (!is_generate_right_neighbor_lanemarker) {
    using_right_neighbor_history_num++;
    flag_using_history_right_centralline_ =
        (using_right_neighbor_history_num > 10 ||
         fabs(left_marker->c0_position()) < 0.9)
            ? false
            : true;
  } else {
    using_right_neighbor_history_num = 0;
    flag_using_history_right_centralline_ = false;
  }
  ADEBUG << "flag_using_history_right_centralline_ = "
         << flag_using_history_right_centralline_
         << "; using_right_neighbor_history_num = "
         << using_right_neighbor_history_num;
}

void NavigationLaneMobileye::LeftLaneDecision(
    perception::LaneMarker* next_left_marker,
    perception::LaneMarker* right_marker, perception::LaneMarker* left_marker) {
  if (!GenerateNeighborLanemarker(next_left_marker, left_marker,
                                  central_lanemarker_width_,
                                  &left_neighbor_lanemarker_variance_)) {
    ADEBUG << "failed Generate next left lanemarker";
    is_generate_left_neighbor_lanemarker = false;
    history_left_neighbor_lanemarker_.Clear();
  } else {
    ADEBUG << "success Generate next left lanemarker";
    is_generate_left_neighbor_lanemarker = true;
    history_left_neighbor_lanemarker_.CopyFrom(*next_left_marker);
  }
  if (!is_generate_left_neighbor_lanemarker) {
    using_left_neighbor_history_num++;
    flag_using_history_left_centralline_ =
        (using_left_neighbor_history_num > 10 ||
         fabs(right_marker->c0_position()) < 0.9)
            ? false
            : true;
  } else {
    using_left_neighbor_history_num = 0;
    flag_using_history_left_centralline_ = false;
  }
  ADEBUG << "flag_using_history_left_centralline_ = "
         << flag_using_history_left_centralline_
         << "; using_left_neighbor_history_num = "
         << using_left_neighbor_history_num;
}

bool NavigationLaneMobileye::CentralLaneDecision(
    perception::LaneMarkers* lane_marker, perception::LaneMarker* right_marker,
    perception::LaneMarker* left_marker) {
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
                                  vehicle_state_->linear_velocity()
                            : 0;
  bool is_new_lane_marker =
      IsNewLaneMarkers(history_lanemarkers_, *lane_marker);
  ADEBUG << "IsNewLaneMarkers = " << is_new_lane_marker;
  if ((!is_new_lane_marker) || (!has_left && !has_right) ||
      !CreatVirtualMarker(has_right, has_left, right_marker, left_marker)) {
    has_no_mobileye_lanemarker_time = ComputeNoLanemarkerTime();
    if (has_no_mobileye_lanemarker_time <
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
             << has_no_mobileye_lanemarker_time << " second";
      return false;
    }
  } else {
    ADEBUG << "------------has lanemarker-----------";
    flag_log_no_lanemarker_time = false;
    using_history_mapmsg_ = false;
  }
  return true;
}

void NavigationLaneMobileye::NoUseLanemarkerC2C3(
    perception::LaneMarkers* markers) {
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

void NavigationLaneMobileye::LogIndividualLanemarker(
    const perception::LaneMarkers& lane_marker) {
  auto left_marker = lane_marker.front_left_lane_marker();
  auto right_marker = lane_marker.front_right_lane_marker();
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
  if (lane_marker.front_next_left_lane_marker().size() > 0) {
    auto next_left_marker = lane_marker.front_next_left_lane_marker().at(0);
    ADEBUG << "next_left_individual_6a9_c0:" << next_left_marker.c0_position()
           << "  next_left_individual_6a9_c1:"
           << next_left_marker.c1_heading_angle()
           << "  next_left_individual_6a9_c2:"
           << next_left_marker.c2_curvature()
           << "  next_left_individual_6a9_c3:"
           << next_left_marker.c3_curvature_derivative()
           << "next_left_individual_6a9_viewrange:"
           << next_left_marker.view_range();
  }
  if (lane_marker.front_next_right_lane_marker().size() > 0) {
    auto next_right_marker = lane_marker.front_next_right_lane_marker().at(0);
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
}

void NavigationLaneMobileye::LogAfterDecisionLanemarker(
    const perception::LaneMarkers& lane_marker) {
  auto left_marker = lane_marker.front_left_lane_marker();
  auto right_marker = lane_marker.front_right_lane_marker();
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

void NavigationLaneMobileye::DigitalFilter(
    perception::LaneMarker* right_marker, perception::LaneMarker* left_marker) {
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

void NavigationLaneMobileye::LanemarkerReverse(perception::LaneMarker* marker) {
  marker->set_c0_position(-marker->c0_position());
  marker->set_c1_heading_angle(-marker->c1_heading_angle());
  marker->set_c2_curvature(-marker->c2_curvature());
  marker->set_c3_curvature_derivative(-marker->c3_curvature_derivative());
  marker->set_longitude_start(config_.lanemarker_back_length());
}

void NavigationLaneMobileye::NextLanemarkerReverse(
    perception::LaneMarker* next_marker, perception::LaneMarker* marker) {
  next_marker->set_c0_position(-next_marker->c0_position());
  next_marker->set_c1_heading_angle(-next_marker->c1_heading_angle());
  next_marker->set_c2_curvature(-next_marker->c2_curvature());
  next_marker->set_c3_curvature_derivative(
      -next_marker->c3_curvature_derivative());
  next_marker->set_longitude_start(config_.lanemarker_back_length());
}

bool NavigationLaneMobileye::JudgeHaveLanemarker(
    perception::LaneMarker* marker) {
  if (marker->view_range() > 0.01) {
    // LanemarkerReverse(marker);
    ADEBUG << "original lanemarker:" << marker->DebugString();
    return true;
  } else {
    marker->Clear();
    return false;
  }
}

bool NavigationLaneMobileye::GenerateNeighborLanemarker(
    perception::LaneMarker* next_marker, perception::LaneMarker* marker,
    double width, std::deque<perception::LaneMarker>* lane_markers) {
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

double NavigationLaneMobileye::ComputeLanemarkerStepQuality(
    const perception::LaneMarker& lanemarker,
    const perception::LaneMarker& history_lanemarker) {
  if (!history_lanemarker.has_c0_position()) {
    return true;
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

double NavigationLaneMobileye::ComputeLanemarkerQuality(
    std::deque<perception::LaneMarker>* lane_markers,
    const perception::LaneMarker& lanemarker) {
  lane_markers->push_back(lanemarker);
  if (lane_markers->size() < 5) {
    return true;
  }
  if (lane_markers->size() > variance_num_) {
    lane_markers->pop_front();
  }
  std::deque<perception::LaneMarker>::iterator it;
  double sum_c0, sum_c1, sum_c2, sum_c3;
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

int NavigationLaneMobileye::ComputeQuality(double ref_lanemarker_variance) {
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

double NavigationLaneMobileye::ComputeNoLanemarkerTime() {
  double no_lanemarker_time = 0.0;
  if (!flag_log_no_lanemarker_time) {
    start_no_lanemarker_time = Clock::NowInSeconds();
    flag_log_no_lanemarker_time = true;
    AERROR << "start no lanemarker and start_time = "
           << start_no_lanemarker_time;
  }
  no_lanemarker_time = Clock::NowInSeconds() - start_no_lanemarker_time;
  AERROR << "totle_time_no_lanemarker = " << no_lanemarker_time;
  return no_lanemarker_time;
}

bool NavigationLaneMobileye::CreatVirtualMarker(
    bool have_right, bool have_left, perception::LaneMarker* right_lane_marker,
    perception::LaneMarker* left_lane_marker) {
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
      central_line_too_narrow_index++;
    } else {
      central_lanemarker_width_history_ = central_lanemarker_width_;
      central_line_too_narrow_index = 0;
    }
    if (central_line_too_narrow_index >
        static_cast<int>(FLAGS_has_no_lanemarker_time / 0.1)) {
      AERROR << "Central lane too narrow for " << FLAGS_has_no_lanemarker_time
             << " second";
      return false;
    }
    // auto short_lane_marker = lane_marker->front_left_lane_marker();
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

void NavigationLaneMobileye::InChangeLane(
    perception::LaneMarker* right_lane_marker,
    perception::LaneMarker* left_lane_marker) {
  bool near_central_line = fabs(right_lane_marker->c0_position()) < 0.5 ||
                           fabs(left_lane_marker->c0_position()) < 0.5;
  bool c0_reverse =
      right_lane_marker->c0_position() *
              history_right_lanemarker_.c0_position() <
          0 ||
      left_lane_marker->c0_position() * history_left_lanemarker_.c0_position() <
          0;
  in_change_lane_ = near_central_line;  // ||c0_reverse
  ADEBUG << "------in_change_lane = " << in_change_lane_;
}

void NavigationLaneMobileye::ConvertMobileyeLaneMarkerToPath(
    const perception::LaneMarkers& lane_marker, common::Path* const path) {
  // Generate the centerline of the lane, the vehicle is currently located
  const double current_speed = vehicle_state_->linear_velocity();
  double central_length = 0;
  // FLAGS_buffer_gainst_lookford_distance +
  // hdmap::PncMap::LookForwardDistance(
  //     functionmanager::MachineStateType::PERCEPTION_TYPE,
  //     config_.default_speed_limit(), FLAGS_default_cruise_speed);
  double lane_width =
      config_.default_left_width() + config_.default_right_width();
  ADEBUG << "Central line constructor start";
  central_line_pts_.clear();
  auto central_navi_path = std::make_shared<navigation_hdmap::NavigationPath>();
  auto* central_path = central_navi_path->mutable_path();
  central_path->set_name("current");
  Vec2d delta_end_point;
  // ADEBUG << "central_line";
  std::vector<Vec2d> left_boundary_point;
  std::vector<Vec2d> right_boundary_point;
  auto left_boundary_of_centrallane = std::make_shared<hdmap::LineSegment>();
  auto right_boundary_of_centrallane = std::make_shared<hdmap::LineSegment>();
  if (!navi_central_line_constructor_.BuildReferenceLine(
          central_length, lane_width, lane_marker.front_right_lane_marker(),
          lane_marker.front_left_lane_marker(), &central_line_pts_,
          &left_boundary_point, &right_boundary_point, &delta_end_point)) {
    AERROR << "Central line constructor failed";
    return;
  }

  TranslatedReferenceLine(lane_marker, central_line_pts_, central_path,
                          left_boundary_point, right_boundary_point,
                          left_boundary_of_centrallane.get(),
                          right_boundary_of_centrallane.get());
  central_navi_path->set_path_priority(FLAGS_current_navi_path_priority);  //
  perceived_left_width_ =
      std::fabs(lane_marker.front_left_lane_marker().c0_position());
  perceived_right_width_ =
      std::fabs(lane_marker.front_right_lane_marker().c0_position());
  double perceived_lane_width = perceived_left_width_ + perceived_right_width_;
  if (perceived_lane_width < 2.0 * config_.min_lane_half_width() ||
      perceived_lane_width > 2.0 * config_.max_lane_half_width()) {
    perceived_left_width_ = default_left_width_;
    perceived_right_width_ = default_right_width_;
  }

  double left_width = (perceived_left_width_ + perceived_right_width_) / 2;
  double right_width = left_width;
  auto central_navi_path_two =
      std::make_shared<navigation_hdmap::NavigationPath>();
  central_navi_path_two->set_path_priority(FLAGS_current_navi_path_priority);
  navigation_path_list_.emplace_back(
      2, left_width, right_width, central_navi_path,
      left_boundary_of_centrallane, right_boundary_of_centrallane);  // 2

  /// construct left centerline
  if (flag_using_history_left_centralline_) {
    navigation_path_list_.emplace_front(
        1, left_width, right_width, left_central_navi_path_history_,
        left_boundary_of_leftlane_history_, left_boundary_of_centrallane);
  } else {
    left_central_line_pts_.clear();
    auto left_central_navi_path =
        std::make_shared<navigation_hdmap::NavigationPath>();
    auto* left_central_path = left_central_navi_path->mutable_path();
    left_central_path->set_name("left");
    const double right_to_central = -default_right_width_;
    // ADEBUG << "left_central_line";
    left_boundary_point.clear();
    right_boundary_point.clear();
    auto left_boundary_of_leftlane = std::make_shared<hdmap::LineSegment>();
    auto right_boundary_of_leftlane = std::make_shared<hdmap::LineSegment>();

    if (!lane_marker.front_next_left_lane_marker().at(0).longitude_end() <
            0.01 &&
        navi_central_line_constructor_.BuildReferenceLine(
            central_length, lane_width, lane_marker.front_left_lane_marker(),
            lane_marker.front_next_left_lane_marker().at(0),
            &left_central_line_pts_, &left_boundary_point,
            &right_boundary_point, &delta_end_point)) {
      // double left_central_line_one_y = left_central_line_pts_.front().y();
      TranslatedReferenceLine(
          lane_marker, left_central_line_pts_, left_central_path,
          left_boundary_point, right_boundary_point,
          left_boundary_of_leftlane.get(), right_boundary_of_leftlane.get());
      left_central_navi_path->set_path_priority(
          FLAGS_left_navi_path_priority);  //
      double left_width =
          std::fabs(
              lane_marker.front_left_lane_marker().c0_position() -
              lane_marker.front_next_left_lane_marker().at(0).c0_position()) /
          2;
      double right_width = ComputeLaneWidth(&left_width);
      navigation_path_list_.emplace_front(
          1, left_width, right_width, left_central_navi_path,
          left_boundary_of_leftlane, right_boundary_of_leftlane);
      left_central_navi_path_history_->CopyFrom(*left_central_navi_path);
      left_boundary_of_leftlane_history_->CopyFrom(*left_boundary_of_leftlane);
    }
  }

  /// construct right centerline
  if (flag_using_history_right_centralline_) {
    navigation_path_list_.emplace_back(
        3, left_width, right_width, right_central_navi_path_history_,
        right_boundary_of_rightlane_history_, right_boundary_of_centrallane);
  } else {
    right_central_line_pts_.clear();
    auto right_central_navi_path =
        std::make_shared<navigation_hdmap::NavigationPath>();
    auto* right_central_path = right_central_navi_path->mutable_path();
    right_central_path->set_name("right");
    const double left_to_central = default_right_width_;
    // bool right_c0 = lane_marker.next_right_lane_marker().empty();
    left_boundary_point.clear();
    right_boundary_point.clear();
    auto left_boundary_of_rightlane = std::make_shared<hdmap::LineSegment>();
    auto right_boundary_of_rightlane = std::make_shared<hdmap::LineSegment>();
    if (!lane_marker.front_next_right_lane_marker().at(0).longitude_end() <
            0.01 &&
        navi_central_line_constructor_.BuildReferenceLine(
            central_length, lane_width,
            lane_marker.front_next_right_lane_marker().at(0),
            lane_marker.front_right_lane_marker(), &right_central_line_pts_,
            &left_boundary_point, &right_boundary_point, &delta_end_point)) {
      TranslatedReferenceLine(
          lane_marker, right_central_line_pts_, right_central_path,
          left_boundary_point, right_boundary_point,
          left_boundary_of_rightlane.get(), right_boundary_of_rightlane.get());
      right_central_navi_path->set_path_priority(
          FLAGS_right_navi_path_priority);  //
      double left_width =
          std::fabs(
              lane_marker.front_next_right_lane_marker().at(0).c0_position() -
              lane_marker.front_right_lane_marker().c0_position()) /
          2;
      double right_width = ComputeLaneWidth(&left_width);
      navigation_path_list_.emplace_back(
          3, left_width, right_width, right_central_navi_path,
          left_boundary_of_rightlane, right_boundary_of_rightlane);
      right_central_navi_path_history_->CopyFrom(*right_central_navi_path);
      right_boundary_of_rightlane_history_->CopyFrom(
          *right_boundary_of_rightlane);
    }
  }

  ADEBUG << "Central line constructor end";
}

double NavigationLaneMobileye::ComputeLaneWidth(double* leftwidth) {
  double rightwidth = *leftwidth;
  if (*leftwidth < config_.min_lane_half_width() ||
      *leftwidth > config_.max_lane_half_width()) {
    *leftwidth = default_left_width_;
    rightwidth = default_right_width_;
  }
  return rightwidth;
}

void NavigationLaneMobileye::TranslatedReferenceLine(
    const perception::LaneMarkers& lane_marker,
    const std::vector<Vec2d>& ref_line_point, common::Path* const path,
    const std::vector<Vec2d>& left_line_point,
    const std::vector<Vec2d>& right_line_point,
    hdmap::LineSegment* const left_path, hdmap::LineSegment* const right_path) {
  const auto& left_lane = lane_marker.front_left_lane_marker();
  const auto& right_lane = lane_marker.front_right_lane_marker();

  double path_c0 = (left_lane.c0_position() + right_lane.c0_position()) / 2.0;

  double left_quality = left_lane.quality() + 0.001;
  double right_quality = right_lane.quality() + 0.001;

  double quality_divider = left_quality + right_quality;

  double path_c1 = (left_lane.c1_heading_angle() * left_quality +
                    right_lane.c1_heading_angle() * right_quality) /
                   quality_divider;

  double path_c2 = (left_lane.c2_curvature() * left_quality +
                    right_lane.c2_curvature() * right_quality) /
                   quality_divider;

  double path_c3 = (left_lane.c3_curvature_derivative() * left_quality +
                    right_lane.c3_curvature_derivative() * right_quality) /
                   quality_divider;

  const double start_s = left_lane.longitude_start();
  double accumulated_s = start_s;

  int ref_line_point_size = ref_line_point.size();
  for (size_t i = 1; i < ref_line_point.size(); ++i) {
    if (i == 1) {
      auto* point = path->add_path_point();
      Eigen::Vector2d enu_coordinate = common::math::RotateVector2d(
          {ref_line_point[i - 1].x(), ref_line_point[i - 1].y()},
          vehicle_state_->heading());
      double x_enu = enu_coordinate.x() + vehicle_state_->x();
      double y_enu = enu_coordinate.y() + vehicle_state_->y();
      point->set_x(x_enu);
      point->set_y(y_enu);
      point->set_theta(vehicle_state_->heading());
      point->set_s(accumulated_s);
      point->set_kappa(
          GetKappa(path_c1, path_c2, path_c3, ref_line_point[i - 1].x()));
      point->set_dkappa(0.0);
    }
    double x1 = ref_line_point[i].x();
    double y1 = ref_line_point[i].y();
    auto* point = path->add_path_point();
    Vec2d vec = ref_line_point[i] - ref_line_point[i - 1];
    double theta = std::atan2(vec.y(), vec.x());
    Eigen::Vector2d enu_coordinate =
        common::math::RotateVector2d({x1, y1}, vehicle_state_->heading());
    double x_enu = enu_coordinate.x() + vehicle_state_->x();
    double y_enu = enu_coordinate.y() + vehicle_state_->y();
    double theta_enu = common::math::NormalizeAngle(
        common::math::NormalizeAngle(theta) + vehicle_state_->heading());
    if (path->path_point_size() > 1) {
      auto& pre_point = path->path_point(path->path_point_size() - 2);
      accumulated_s += std::hypot(x_enu - pre_point.x(), y_enu - pre_point.y());
    }
    point->set_x(x_enu);
    point->set_y(y_enu);
    point->set_theta(theta_enu);
    point->set_s(accumulated_s);
    point->set_kappa(GetKappa(path_c1, path_c2, path_c3, x1));

    const double k1 = GetKappa(path_c1, path_c2, path_c3, x1 - 0.0001);
    const double k2 = GetKappa(path_c1, path_c2, path_c3, x1 + 0.0001);
    point->set_dkappa((k2 - k1) / 0.0002);
  }
  BoundaryLineBus2Earth(left_line_point, left_path);
  BoundaryLineBus2Earth(right_line_point, right_path);
}

void NavigationLaneMobileye::BoundaryLineBus2Earth(
    const std::vector<Vec2d>& line_point, hdmap::LineSegment* const path) {
  for (size_t i = 0; i < line_point.size(); ++i) {
    double x1 = line_point[i].x();
    double y1 = line_point[i].y();
    auto point = path->add_point();
    Eigen::Vector2d enu_coordinate =
        common::math::RotateVector2d({x1, y1}, vehicle_state_->heading());
    double x_enu = enu_coordinate.x() + vehicle_state_->x();
    double y_enu = enu_coordinate.y() + vehicle_state_->y();
    point->set_x(x_enu);
    point->set_y(y_enu);
  }
}

bool NavigationLaneMobileye::CreateMap(
    navigation_hdmap::MapMsg* const map_msg) {
  if (using_history_mapmsg_) {
    map_msg->CopyFrom(history_map_msg_);
    common::util::FillHeader("navigation_hdmap", map_msg);
    common::util::FillHeader("from_navigation_hdmap",
                             map_msg->mutable_hdmap()->mutable_header());
    common::util::FillHeader("from_navigation_routing",
                             map_msg->mutable_routing());
    return true;
  }
  auto* navigation_path = map_msg->mutable_navigation_path();
  auto* hdmap = map_msg->mutable_hdmap();
  auto* lane_marker = map_msg->mutable_lane_marker();
  lane_marker->CopyFrom(lane_marker_);

  // If no navigation path is generated based on navigation lines, we try to
  // create map with "current_navi_path_tuple_" which is generated based on
  // perceived lane markers.
  if (navigation_path_list_.empty()) {
    return false;
  }
  // double list_one_y =
  // std::get<3>(*navigation_path_list_.begin())->path().path_point().at(0).y();
  // double list_two_y =
  // std::get<3>(*(navigation_path_list_.begin()++))->path().path_point().at(0).y();

  int fail_num = 0;
  FLAGS_navigation_hdmap_generate_left_boundray = true;
  for (auto iter = navigation_path_list_.cbegin();
       iter != navigation_path_list_.cend(); ++iter) {
    std::size_t index = std::distance(navigation_path_list_.cbegin(), iter);
    if (!CreateSingleLaneMap(*iter, lane_marker_, hdmap, navigation_path)) {
      AWARN << "Failed to generate lane: " << index;
      ++fail_num;
      FLAGS_navigation_hdmap_generate_left_boundray = true;
      continue;
    }
    // double list_y = std::get<3>(*iter)->path().path_point().at(0).y();
    // double path_central_y =
    // (*navigation_path)["0_current"].path().path_point().at(0).y(); double
    // path_left_y = 0.0; if(index > 0){
    //   path_left_y =
    //   (*navigation_path)["0_left"].path().path_point().at(0).y();
    // }

    FLAGS_navigation_hdmap_generate_left_boundray = true;
    // The left border of the middle lane uses the right border of the left
    // lane.
    int lane_index = static_cast<int>(index) - fail_num;
    if (lane_index > 0) {
      auto* left_boundary =
          hdmap->mutable_lane(lane_index)->mutable_left_boundary();
      left_boundary->CopyFrom(hdmap->lane(lane_index - 1).right_boundary());
      auto* left_sample =
          hdmap->mutable_lane(lane_index)->mutable_left_sample();
      left_sample->CopyFrom(hdmap->lane(lane_index - 1).right_sample());
    }
  }
  // Set routing info
  auto* routing = map_msg->mutable_routing();
  SetRoutingAndRoad(routing, hdmap);
  history_map_msg_ = *map_msg;
  return true;
}

bool NavigationLaneMobileye::SetRoutingAndRoad(
    TL::routing::RoutingResponse* inrouting, TL::hdmap::Map* hd_map) {
  // Set road boundary
  int lane_num = hd_map->lane_size();
  ADEBUG << "The Lane number is: " << lane_num;
  auto* road = hd_map->add_road();
  road->mutable_id()->set_id("road_navigation");
  auto* section = road->add_section();
  for (int i = 0; i < lane_num; ++i) {
    auto* lane_id = section->add_lane_id();
    lane_id->CopyFrom(hd_map->lane(i).id());
  }
  auto* outer_polygon = section->mutable_boundary()->mutable_outer_polygon();
  auto* left_edge = outer_polygon->add_edge();
  left_edge->set_type(TL::hdmap::BoundaryEdge::LEFT_BOUNDARY);
  left_edge->mutable_curve()->CopyFrom(hd_map->lane(0).left_boundary().curve());

  auto* right_edge = outer_polygon->add_edge();
  right_edge->set_type(TL::hdmap::BoundaryEdge::RIGHT_BOUNDARY);
  right_edge->mutable_curve()->CopyFrom(
      hd_map->lane(lane_num - 1).right_boundary().curve());
  // Set routing info
  auto* routing_road = inrouting->add_road();
  routing_road->set_id(road->id().id());
  for (int i = 0; i < lane_num; ++i) {
    if (absl::EndsWith(hd_map->lane(i).id().id(), "current")) {
      auto* passage = routing_road->add_passage();
      passage->set_can_exit(false);
      passage->set_change_lane_type(routing::ChangeLaneType::FORWARD);
      auto* segment = passage->add_segment();
      segment->set_id(hd_map->lane(i).id().id());
      auto adc_lane_segment_points = hd_map->lane(i)
                                         .central_curve()
                                         .segment()
                                         .at(0)
                                         .line_segment()
                                         .point();
      common::PointENU start_point = adc_lane_segment_points.at(0);
      int max_index = adc_lane_segment_points.size() - 1;
      common::PointENU end_point = adc_lane_segment_points.at(max_index);
      auto* routing_request = inrouting->mutable_routing_request();
      common::util::FillHeader("from_navigation_routingrequest",
                               routing_request);
      routing::LaneWaypoint waypoint;
      waypoint.set_id(hd_map->lane(i).id().id());
      waypoint.mutable_pose()->set_x(start_point.x());
      waypoint.mutable_pose()->set_y(start_point.y());
      waypoint.set_s(0.0);
      segment->set_start_s(0.0);
      routing_request->add_waypoint()->CopyFrom(waypoint);
      waypoint.set_s(hd_map->lane(i).length());
      segment->set_end_s(hd_map->lane(i).length());
      waypoint.mutable_pose()->set_x(end_point.x());
      waypoint.mutable_pose()->set_y(end_point.y());
      routing_request->add_waypoint()->CopyFrom(waypoint);
    }
  }
  common::util::FillHeader("from_navigation_routing", inrouting);
  // Set neighbor information for each lane
  if (lane_num < 2) {
    return true;
  }
  for (int i = 0; i < lane_num; ++i) {
    auto* lane = hd_map->mutable_lane(i);
    if (i > 0) {
      lane->add_left_neighbor_forward_lane_id()->CopyFrom(
          hd_map->lane(i - 1).id());
      ADEBUG << "Left neighbor is: " << hd_map->lane(i - 1).id().id();
    }
    if (i < lane_num - 1) {
      lane->add_right_neighbor_forward_lane_id()->CopyFrom(
          hd_map->lane(i + 1).id());
      ADEBUG << "Right neighbor is: " << hd_map->lane(i + 1).id().id();
    }
  }
  ADEBUG << "hdmap lane number " << hd_map->lane_size();
  return true;
}

// NOLINTEND
}  // namespace planning
}  // namespace TL
